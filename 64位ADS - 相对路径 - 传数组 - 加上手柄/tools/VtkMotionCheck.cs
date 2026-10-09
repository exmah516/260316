using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Security.Cryptography;
using System.Threading.Tasks;
using System.Windows;
using Kitware.VTK;
using MasterConsole.Controls;

// 只创建模型控件；网络模式仅接收显示测试值，不创建任何机器人控制服务。
class VtkMotionCheck
{
    static readonly BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;
    static readonly List<string> Results = new List<string>();
    static RobotViewport Model;
    static string Output;
    static bool Closed;
    static int ExitCode = 1;
    static TcpListener Listener;
    static TcpClient Client;

    static object Field(string name) { return typeof(RobotViewport).GetField(name, Private).GetValue(Model); }
    static void Check(bool condition, string message)
    {
        Results.Add((condition ? "PASS: " : "FAIL: ") + message);
        File.WriteAllLines(Path.Combine(Output, "result.txt"), Results);
        if (!condition) throw new Exception(message);
    }
    static double[] Matrices()
    {
        var values = new List<double>();
        var actors = ((vtkRenderer)Field("_renderer")).GetActors();
        actors.InitTraversal();
        for (int n = 0; n < actors.GetNumberOfItems(); n++)
        {
            var matrix = actors.GetNextActor().GetMatrix();
            for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) values.Add(matrix.GetElement(r, c));
        }
        return values.ToArray();
    }
    static double Difference(double[] a, double[] b)
    {
        if (a.Length != b.Length || a.Length == 0) throw new Exception("VTK actor count changed or empty");
        double delta = 0;
        for (int i = 0; i < a.Length; i++)
        {
            if (double.IsNaN(b[i]) || double.IsInfinity(b[i])) throw new Exception("Invalid actor matrix");
            delta = Math.Max(delta, Math.Abs(a[i] - b[i]));
        }
        return delta;
    }
    static string Capture(string name)
    {
        string path = Path.Combine(Output, name + ".png");
        var control = (RenderWindowControl)Field("ModelShow");
        control.RenderWindow.Render();
        using (var capture = vtkWindowToImageFilter.New())
        using (var writer = vtkPNGWriter.New())
        {
            capture.SetInput(control.RenderWindow);
            capture.ReadFrontBufferOff();
            capture.Update();
            writer.SetFileName(path);
            writer.SetInput(capture.GetOutput());
            writer.Write();
        }
        using (var sha = SHA256.Create()) return Convert.ToBase64String(sha.ComputeHash(File.ReadAllBytes(path)));
    }
    static async Task Pose(int joint, double value)
    {
        var sj = new double[7]; var xj = new double[2];
        if (joint < 7) sj[joint] = value; else xj[joint - 7] = value;
        await Apply(new DsaArmDisplayState(sj, xj, 0, 0));
    }
    static async Task Apply(DsaArmDisplayState state)
    {
        if (Closed) throw new OperationCanceledException("Window closed before completion");
        var previous = Field("_lastState");
        if (!Model.PublishTestDsaArmState(state)) throw new Exception("Test state rejected");
        // 等待真实控件的 50 ms 定时器，不能直接调用 ApplyState 冒充更新链路。
        for (int i = 0; i < 40; i++)
        {
            await Task.Delay(25);
            if (Closed) throw new OperationCanceledException("Window closed before completion");
            var applied = (DsaArmDisplayState)Field("_lastState");
            if (applied == null || ReferenceEquals(previous, applied)) continue;
            bool matches = applied.V1 == state.V1 && applied.V2 == state.V2;
            for (int k = 0; k < 7; k++) matches &= applied.Sj[k] == state.Sj[k];
            for (int k = 0; k < 2; k++) matches &= applied.Xj[k] == state.Xj[k];
            if (matches) return;
        }
        throw new TimeoutException("Model timer did not apply input");
    }
    static string Option(string[] args, string name, string fallback)
    {
        int i = Array.IndexOf(args, name);
        return i >= 0 && i + 1 < args.Length ? args[i + 1] : fallback;
    }
    static async Task NetworkTest(Window window, string[] args)
    {
        var address = IPAddress.Parse(Option(args, "--listen", "192.168.50.1"));
        var peer = IPAddress.Parse(Option(args, "--peer", "192.168.50.2"));
        int port = int.Parse(Option(args, "--port", "32110"), CultureInfo.InvariantCulture);
        Listener = new TcpListener(address, port);
        Listener.Start(1);
        window.Title = "模型网络测试：等待 " + peer + " → " + address + ":" + port;
        Check(true, "LISTEN " + address + ":" + port + "; allowed peer " + peer);
        var accept = Listener.AcceptTcpClientAsync();
        if (await Task.WhenAny(accept, Task.Delay(120000)) != accept) throw new TimeoutException("No sender within 120 seconds");
        Client = await accept;
        if (!((IPEndPoint)Client.Client.RemoteEndPoint).Address.Equals(peer)) throw new InvalidDataException("Unexpected sender IP");
        Client.NoDelay = true;
        var stream = Client.GetStream();
        stream.WriteTimeout = 2000;
        await Pose(0, 0);
        var origin = Matrices();
        var baseline = Capture("00_zero");
        var moved = new bool[9]; var negative = new bool[9]; var returned = new bool[9];
        int frames = 0, rejected = 0;
        while (!Closed)
        {
            // 固定 92 字节：VTK1 + 11 个小端 double；不接受任意长度或命令。
            var frame = new byte[92];
            var deadline = System.Diagnostics.Stopwatch.StartNew();
            for (int offset = 0; offset < frame.Length;)
            {
                var read = stream.ReadAsync(frame, offset, frame.Length - offset);
                int remaining = (int)Math.Max(0, 10000 - deadline.ElapsedMilliseconds);
                if (await Task.WhenAny(read, Task.Delay(remaining)) != read) throw new TimeoutException("Sender stopped for 10 seconds");
                int n = await read;
                if (n == 0) throw new EndOfStreamException("Sender disconnected before all 9 joints completed");
                offset += n;
            }
            var values = new double[11];
            bool valid = BitConverter.ToInt32(frame, 0) == 0x314B5456;
            for (int i = 0; i < values.Length; i++)
            {
                values[i] = BitConverter.ToDouble(frame, 4 + i * 8);
                double limit = i == 0 || i == 1 || i == 5 ? 0.1 : 0.5;
                valid &= !double.IsNaN(values[i]) && !double.IsInfinity(values[i]) && Math.Abs(values[i]) <= limit;
            }
            if (!valid)
            {
                stream.WriteByte(0);
                Check(true, "Rejected invalid display frame " + (++rejected));
                if (rejected >= 10) throw new InvalidDataException("Too many invalid frames");
                continue;
            }
            var sj = new double[7]; var xj = new double[2];
            Array.Copy(values, sj, 7); Array.Copy(values, 7, xj, 0, 2);
            await Apply(new DsaArmDisplayState(sj, xj, values[9], values[10]));
            window.Title = "仅模型网络测试：已应用 " + (++frames) + " 帧，来源 " + peer;
            int nonzero = 0, active = -1;
            for (int i = 0; i < 9; i++) if (Math.Abs(values[i]) > 1e-9) { nonzero++; active = i; }
            if (nonzero == 1)
            {
                double peak = active == 0 || active == 1 || active == 5 ? 0.08 : 0.35;
                if (!moved[active] && Math.Abs(values[active] - peak) < 1e-8)
                {
                    string name = active < 7 ? "sj" + (active + 1) : "xj" + (active - 6);
                    Check(Difference(origin, Matrices()) > 1e-6, name + " network input changed actor matrix");
                    Check(Capture(name + "_network") != baseline, name + " network input changed rendered image");
                    moved[active] = true;
                }
                if (moved[active] && !negative[active] && Math.Abs(values[active] + peak) < 1e-8)
                {
                    Check(Difference(origin, Matrices()) > 1e-6, "Joint " + active + " negative excursion verified");
                    negative[active] = true;
                }
            }
            if (nonzero == 0)
            {
                Check(Difference(origin, Matrices()) < 1e-8, "Network zero pose restored");
                for (int i = 0; i < 9; i++) if (moved[i] && negative[i]) returned[i] = true;
            }
            bool done = Array.TrueForAll(returned, delegate(bool value) { return value; });
            if (done) Capture("10_returned_zero");
            // 回执发生在真实定时器已应用且检查完成之后，不以收到数据冒充画面更新。
            stream.WriteByte(done ? (byte)2 : (byte)1);
            if (done)
            {
                Check(true, "9/9 network joints verified; frames=" + frames + "; rejected=" + rejected);
                return;
            }
        }
    }
    [STAThread]
    static int Main(string[] args)
    {
        bool autoClose = Array.IndexOf(args, "--auto-close") >= 0;
        Output = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "VTK测试结果", DateTime.Now.ToString("yyyyMMdd_HHmmss_fff"));
        Directory.CreateDirectory(Output);
        var app = new Application();
        var window = new Window { Title = "旧 VTK 模型离线运动测试（不连接设备）", Width = 1100, Height = 800 };
        try
        {
            Model = new RobotViewport();
            window.Content = Model;
            window.Closed += delegate { Closed = true; if (Client != null) Client.Close(); if (Listener != null) Listener.Stop(); Model.Dispose(); };
            window.Loaded += async delegate
            {
                try
                {
                    await Task.Delay(300);
                    Check((bool)Field("_modelReady"), "URDF / VTK loaded");
                    if (Array.IndexOf(args, "--listen") >= 0)
                    {
                        await NetworkTest(window, args);
                        ExitCode = 0;
                        window.Title = "网络测试通过：9/9 关节可动，已恢复零姿态；关闭窗口退出";
                        return;
                    }
                    await Pose(0, 0);
                    var origin = Matrices();
                    var baseline = Capture("00_zero");
                    for (int joint = 0; joint < 9; joint++)
                    {
                        string name = joint < 7 ? "sj" + (joint + 1) : "xj" + (joint - 6);
                        double amplitude = joint == 0 || joint == 1 || joint == 5 ? 0.08 : 0.35;
                        window.Title = "仅模型动画：" + name + " 往返；" + (amplitude == 0.08 ? "±0.08 m" : "±0.35 rad");
                        for (int step = 1; step <= 32; step++)
                        {
                            await Pose(joint, step == 32 ? 0 : amplitude * Math.Sin(step * 2 * Math.PI / 32));
                            await Task.Delay(25);
                            if (step == 8)
                            {
                                double delta = Difference(origin, Matrices());
                                Check(delta > 1e-6, name + " actor matrix changed; max delta=" + delta.ToString("G6", CultureInfo.InvariantCulture));
                                Check(Capture(name + "_positive") != baseline, name + " rendered image changed");
                            }
                        }
                        Check(Difference(origin, Matrices()) < 1e-8, name + " returned to zero");
                    }
                    Capture("10_returned_zero");
                    Check(true, "9/9 joints: actual timer, transform, rendered image and return verified; no hardware connection");
                    ExitCode = 0;
                    window.Title = "测试通过：9/9 关节可动，已恢复零姿态；关闭窗口退出";
                }
                catch (Exception ex)
                {
                    Results.Add("FAIL: " + ex);
                    File.WriteAllLines(Path.Combine(Output, "result.txt"), Results);
                    if (!Closed) window.Title = "测试未通过，见 VTK测试结果 / result.txt";
                }
                finally
                {
                    if (Client != null) Client.Close();
                    if (Listener != null) Listener.Stop();
                    if (autoClose && !Closed) window.Close();
                }
            };
            app.Run(window);
        }
        catch (Exception ex) { Results.Add("FAIL: " + ex); }
        File.WriteAllLines(Path.Combine(Output, "result.txt"), Results);
        Console.WriteLine(string.Join(Environment.NewLine, Results));
        Console.WriteLine("Results: " + Output);
        return ExitCode;
    }
}
