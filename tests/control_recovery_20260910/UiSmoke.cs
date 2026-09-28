using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.IO.Pipes;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using AdsControlUI;

internal static class UiSmoke
{
    private static readonly ConcurrentQueue<int[]> Commands = new ConcurrentQueue<int[]>();
    private static readonly object PipeLock = new object();
    private static NamedPipeServerStream Server;
    private static volatile byte[] LatestBytes;
    private static int Checks;

    private static void Check(bool value, string label)
    {
        if (!value) throw new Exception(label);
        ++Checks;
    }

    private static void Pump(int milliseconds = 160)
    {
        var watch = Stopwatch.StartNew();
        while (watch.ElapsedMilliseconds < milliseconds)
        {
            var frame = new DispatcherFrame();
            Dispatcher.CurrentDispatcher.BeginInvoke(DispatcherPriority.Background,
                new Action(() => frame.Continue = false));
            Dispatcher.PushFrame(frame);
            Thread.Sleep(5);
        }
    }

    private static void Publish(VisState state)
    {
        int size = Marshal.SizeOf<VisState>();
        var pointer = Marshal.AllocHGlobal(size);
        try
        {
            Marshal.StructureToPtr(state, pointer, false);
            var bytes = new byte[size];
            Marshal.Copy(pointer, bytes, 0, size);
            LatestBytes = bytes;
        }
        finally { Marshal.FreeHGlobal(pointer); }
        Pump();
    }

    private static int[] NextCommand()
    {
        for (int i = 0; i < 20; ++i)
        {
            if (Commands.TryDequeue(out var command)) return command;
            Pump(20);
        }
        throw new Exception("No command received.");
    }

    private static ToggleButton CylinderButton(DependencyObject root, int index)
    {
        if (root is ToggleButton button && Equals(button.Content, "电缸 " + (index + 1)))
            return button;
        for (int i = 0; i < VisualTreeHelper.GetChildrenCount(root); ++i)
        {
            var found = CylinderButton(VisualTreeHelper.GetChild(root, i), index);
            if (found != null) return found;
        }
        return null;
    }

    [STAThread]
    private static void Main(string[] args)
    {
        // 离线测试超时只结束本测试进程，防止管道清理阻塞验证任务。
        using var timeout = new Timer(_ =>
        {
            Console.Error.WriteLine("FAIL: UI smoke timeout after " + Checks + " checks.");
            Environment.Exit(2);
        }, null, 30000, Timeout.Infinite);
        try { Run(args); }
        catch (Exception error)
        {
            // 逐层输出消息，避免装载异常在格式化完整异常时再次失败。
            for (var current = error; current != null; current = current.InnerException)
                Console.Error.WriteLine(current.GetType().FullName + ": " + current.Message);
            Environment.ExitCode = 1;
        }
    }

    private static void Run(string[] args)
    {
        // 只模拟界面管道，绝不启动上位机后台或访问 ADS。
        Check(Process.GetProcessesByName("ADS").Length == 0, "Close ADS before isolated UI test.");
        Server = new NamedPipeServerStream("ADS_Control_Vis", PipeDirection.InOut, 1,
            PipeTransmissionMode.Message, PipeOptions.Asynchronous);
        var serverTask = Task.Run(() =>
        {
            try
            {
                Server.WaitForConnection();
                var reader = new BinaryReader(Server);
                while (Server.IsConnected)
                {
                    uint magic = reader.ReadUInt32();
                    reader.ReadUInt16();
                    reader.ReadUInt16();
                    int type = reader.ReadInt32();
                    int p1 = reader.ReadInt32();
                    int p2 = reader.ReadInt32();
                    uint length = reader.ReadUInt32();
                    reader.ReadBytes((int)length);
                    if (magic != 0x31434D56) throw new Exception("Invalid command framing.");
                    Commands.Enqueue(new[] { type, p1, p2 });
                }
            }
            catch (IOException) { }
            catch (ObjectDisposedException) { }
        });
        var app = new Application { ShutdownMode = ShutdownMode.OnExplicitShutdown };
        app.Resources.MergedDictionaries.Add(new ResourceDictionary
        {
            Source = new Uri("/AdsControlUI;component/Themes/LightClinical.xaml", UriKind.Relative)
        });
        var window = new MainWindow();
        // 实际后台持续发送状态；单帧后停发会让同步客户端读写互相等待。
        using var publisher = new Timer(_ =>
        {
            var bytes = LatestBytes;
            if (bytes == null) return;
            try
            {
                lock (PipeLock)
                    if (Server.IsConnected) Server.Write(bytes, 0, bytes.Length);
            }
            catch (IOException) { }
            catch (ObjectDisposedException) { }
        }, null, 0, 30);
        // 离屏布局与渲染，不创建可见窗口，也不操作用户桌面。
        var view = (FrameworkElement)window.Content;
        view.Measure(new Size(1040, 860));
        view.Arrange(new Rect(0, 0, 1040, 860));
        view.UpdateLayout();
        for (int i = 0; i < 100 && !Server.IsConnected; ++i) Pump(20);
        Check(Server.IsConnected, "Mock pipe connection");
        Check(Marshal.SizeOf<VisState>() == 867, "Protocol size");

        object boxed = new VisState();
        foreach (var field in typeof(VisState).GetFields())
        {
            var attr = field.GetCustomAttribute<MarshalAsAttribute>();
            if (field.FieldType.IsArray)
                field.SetValue(boxed, Array.CreateInstance(field.FieldType.GetElementType(), attr.SizeConst));
        }
        var state = (VisState)boxed;
        state.ads_state = 2;
        state.self_check_done = true;
        state.control_active = true;
        state.cylinder_manual_allowed = true;
        state.startup_completed = true;
        state.cylinder_cmd = new ushort[] { 400, 600, 400, 500 };
        Publish(state);
        var vm = (AdsControlViewModel)window.DataContext;
        Check(vm.CylinderManualAllowed, "Snapshot enables cylinder controls");
        view.UpdateLayout();
        var click = typeof(MainWindow).GetMethod("SetCylinderState", BindingFlags.Instance | BindingFlags.NonPublic);
        var error = (TextBlock)window.FindName("CylinderError");
        for (int index = 0; index < 4; ++index)
        {
            Console.WriteLine("Checking cylinder " + (index + 1));
            var button = CylinderButton(view, index);
            var input = (TextBox)window.FindName("TbCyl" + (index + 1) + "Position");
            Check(button != null, "Cylinder button exists in visual tree");
            Check(button.IsEnabled, "Cylinder button enabled");
            Check(input.ActualWidth >= 60 && input.ActualHeight >= 30, "Input has usable bounds");
            Check(!vm.IsCylinderManual(index), "Automatic position is not manual state");
            input.Text = (index % 2 == 0 ? 0 : 2000).ToString();
            click.Invoke(window, new object[] { button, index });
            var command = NextCommand();
            Check(command[0] == 39 && command[1] == index &&
                command[2] == int.Parse(input.Text), "Send indexed target including endpoints");
            state.cylinder_manual_mask = (byte)(1 << index);
            Publish(state);
            Check(button.IsChecked == true, "Selection follows backend mask");
            input.Text = "bad";
            click.Invoke(window, new object[] { button, index });
            command = NextCommand();
            Check(command[0] == 40 && command[1] == index, "Second click restores even with invalid text");
            state.cylinder_manual_mask = 0;
            Publish(state);
            Check(button.IsChecked == false, "Automatic takeover clears selection");
            foreach (string invalid in new[] { "-1", "2001", "", "1.5", "bad" })
            {
                input.Text = invalid;
                click.Invoke(window, new object[] { button, index });
                Pump(30);
                Check(!string.IsNullOrEmpty(error.Text) && Commands.IsEmpty, "Reject invalid target: " + invalid);
            }
            input.Text = "1234";
        }
        error.Text = "";
        state.cylinder_manual_allowed = false;
        Publish(state);
        Check(!CylinderButton(view, 0).IsEnabled, "Automatic sequence disables manual inputs");
        Check(!vm.SetCylinderManualPosition(0, 1000), "Disabled model rejects movement");
        Check(!vm.SetCylinderManualPosition(4, 1000), "Invalid index rejected");
        state.cylinder_manual_allowed = true;
        Publish(state);
        // 新增定位臂界面只连接模拟管道，验证单位、命令和窗口内归零状态。
        Check(!vm.ArmCartesianAvailable, "No arm controls without valid arm feedback");
        state.arm_snapshot_valid = true;
        state.arm_manual_enable = true;
        state.arm_at_program_zero = true;
        state.arm_act_pos = new double[] { 100, 90, 180, 180, 0 };
        Publish(state);
        Check(vm.ArmCartesianAvailable, "Valid arm feedback enables Cartesian controls");
        Check(vm.ArmZeroText.Contains("已到位"), "Program zero displayed");
        Check(vm.ArmProgramAnglesText.Contains("0.00"), "Mechanical offsets displayed as program angles");
        var mode = (ComboBox)window.FindName("ArmCartesianMode");
        Check(mode.Items.Count == 5, "Five Cartesian jog modes");
        var settings = typeof(MainWindow).GetMethod("SendCartesianSettings",
            BindingFlags.Instance | BindingFlags.NonPublic);
        Check(!(bool)settings.Invoke(window, new object[] { false }) && Commands.IsEmpty,
            "Missing lift calibration sends no command");
        ((TextBox)window.FindName("ArmLiftMin")).Text = "-25.125";
        ((TextBox)window.FindName("ArmHomeTravel")).Text = "15";
        ((TextBox)window.FindName("ArmHomeTip")).Text = "100";
        ((TextBox)window.FindName("ArmHomeRotation")).Text = "20";
        Check((bool)settings.Invoke(window, new object[] { true }), "Explicit home settings accepted");
        int[] parameters = { -25125, 15000, 100000, 20000 };
        for (int field = 0; field < 4; ++field) {
            var command = NextCommand();
            Check(command[0] == 44 && command[1] == field && command[2] == parameters[field],
                "Cartesian parameter index and fixed-point units");
        }
        for (int axis = 1; axis <= 5; ++axis) {
            Check(vm.SetArmCartesianJog(axis, -1250), "Cartesian command sent");
            var command = NextCommand();
            Check(command[0] == 41 && command[1] == axis && command[2] == -1250,
                "Cartesian mode and signed speed transmitted");
        }
        typeof(MainWindow).GetMethod("ArmProgramZero_Click",
            BindingFlags.Instance | BindingFlags.NonPublic).Invoke(window,
                new object[] { null, new RoutedEventArgs() });
        for (int field = 0; field < 4; ++field) Check(NextCommand()[0] == 44, "Home sends settings first");
        Check(NextCommand()[0] == 42, "Home button sends program-zero command");
        state.arm_home_request_id = 1;
        state.arm_cartesian_status = 3;
        Publish(state);
        Check(vm.ArmCartesianStatusText.Contains("未移动"), "Zero window reports no movement");
        Check(!(bool)typeof(MainWindow).GetField("_programReturnActive",
            BindingFlags.Instance | BindingFlags.NonPublic).GetValue(window), "Home lease ends after response");
        while (Commands.TryDequeue(out var heartbeat))
            Check(heartbeat[0] == 45, "Only home heartbeat may follow home request");
        vm.StopArmCartesian();
        Check(NextCommand()[0] == 43, "Cartesian stop command");
        state.arm_cartesian_error = 1104;
        Publish(state);
        Check(vm.ArmCartesianStatusText.Contains("奇异"), "Singularity error is visible");
        state.arm_snapshot_valid = false;
        Publish(state);
        Check(!vm.ArmCartesianAvailable, "Stale feedback disables Cartesian controls");
        state.arm_snapshot_valid = true;
        state.arm_cartesian_error = 0;
        Publish(state);
        foreach (var size in new[] { new Size(1040, 860), new Size(920, 700) })
        {
            window.Width = size.Width;
            window.Height = size.Height;
            view.Measure(size);
            view.Arrange(new Rect(new Point(), size));
            view.UpdateLayout();
            var group = FindCylinderGroup(CylinderButton(view, 0));
            Check(group.ActualWidth > 300, "Cylinder group fits minimum window width");
            var image = new RenderTargetBitmap((int)Math.Ceiling(group.ActualWidth),
                (int)Math.Ceiling(group.ActualHeight), 96, 96, PixelFormats.Pbgra32);
            image.Render(group);
            var encoder = new PngBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(image));
            Directory.CreateDirectory(args[0]);
            using (var file = File.Create(Path.Combine(args[0], "cylinders-" + size.Width + ".png")))
                encoder.Save(file);
            var armGroup = FindCylinderGroup(mode);
            Check(armGroup.ActualWidth > 300 && mode.ActualWidth >= 80,
                "Arm controls fit both window widths");
            var armPanel = (FrameworkElement)VisualTreeHelper.GetParent(VisualTreeHelper.GetParent(mode));
            image = new RenderTargetBitmap((int)Math.Ceiling(armPanel.ActualWidth),
                (int)Math.Ceiling(armPanel.ActualHeight), 96, 96, PixelFormats.Pbgra32);
            var drawing = new DrawingVisual();
            using (var context = drawing.RenderOpen()) {
                var bounds = new Rect(0, 0, image.PixelWidth, image.PixelHeight);
                context.DrawRectangle(Brushes.White, null, bounds);
                context.DrawRectangle(new VisualBrush(armPanel), null, bounds);
            }
            image.Render(drawing);
            var pixels = new byte[image.PixelWidth * image.PixelHeight * 4];
            image.CopyPixels(pixels, image.PixelWidth * 4, 0);
            Check(pixels.Where((value, index) => index % 4 != 3 && value < 200).Count() > 500,
                "Arm panel render contains visible controls");
            encoder = new PngBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(image));
            using (var file = File.Create(Path.Combine(args[0], "arm-" + size.Width + ".png")))
                encoder.Save(file);
        }
        vm.Dispose();
        Console.WriteLine("Client disposed.");
        window.Close();
        Server.Dispose();
        Check(serverTask.Wait(2000), "Mock server stopped");
        app.Shutdown();
        Console.WriteLine("PASS: " + Checks + " WPF/protocol checks; 4 offscreen renders; no hardware.");
    }

    private static GroupBox FindCylinderGroup(DependencyObject child)
    {
        while (!(child is GroupBox)) child = VisualTreeHelper.GetParent(child);
        return (GroupBox)child;
    }
}
