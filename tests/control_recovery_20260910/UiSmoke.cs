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
            lock (PipeLock) Server.Write(bytes, 0, bytes.Length);
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
        // 离屏布局与渲染，不创建可见窗口，也不操作用户桌面。
        window.Measure(new Size(1040, 860));
        window.Arrange(new Rect(0, 0, 1040, 860));
        window.UpdateLayout();
        for (int i = 0; i < 100 && !Server.IsConnected; ++i) Pump(20);
        Check(Server.IsConnected, "Mock pipe connection");
        Check(Marshal.SizeOf<VisState>() == 884, "Protocol size");

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
        var click = typeof(MainWindow).GetMethod("SetCylinderState", BindingFlags.Instance | BindingFlags.NonPublic);
        var error = (TextBlock)window.FindName("CylinderError");
        for (int index = 0; index < 4; ++index)
        {
            var button = CylinderButton(window, index);
            var input = (TextBox)window.FindName("TbCyl" + (index + 1) + "Position");
            Check(button != null && button.IsEnabled, "Cylinder button enabled");
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
        Check(!CylinderButton(window, 0).IsEnabled, "Automatic sequence disables manual inputs");
        Check(!vm.SetCylinderManualPosition(0, 1000), "Disabled model rejects movement");
        Check(!vm.SetCylinderManualPosition(4, 1000), "Invalid index rejected");
        state.cylinder_manual_allowed = true;
        Publish(state);
        foreach (var size in new[] { new Size(1040, 860), new Size(920, 700) })
        {
            window.Width = size.Width;
            window.Height = size.Height;
            window.Measure(size);
            window.Arrange(new Rect(new Point(), size));
            window.UpdateLayout();
            var group = FindCylinderGroup(CylinderButton(window, 0));
            Check(group.ActualWidth > 300, "Cylinder group fits minimum window width");
            var image = new RenderTargetBitmap((int)Math.Ceiling(group.ActualWidth),
                (int)Math.Ceiling(group.ActualHeight), 96, 96, PixelFormats.Pbgra32);
            image.Render(group);
            var encoder = new PngBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(image));
            Directory.CreateDirectory(args[0]);
            using (var file = File.Create(Path.Combine(args[0], "cylinders-" + size.Width + ".png")))
                encoder.Save(file);
        }
        vm.Dispose();
        window.Close();
        Server.Dispose();
        Check(serverTask.Wait(2000), "Mock server stopped");
        app.Shutdown();
        Console.WriteLine("PASS: " + Checks + " WPF/protocol checks; 2 offscreen renders; no hardware.");
    }

    private static GroupBox FindCylinderGroup(DependencyObject child)
    {
        while (!(child is GroupBox)) child = VisualTreeHelper.GetParent(child);
        return (GroupBox)child;
    }
}
