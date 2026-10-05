using System;
using System.Globalization;
using System.IO;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using MasterConsole.Services;
using MasterConsole.ViewModels;

namespace MasterConsole.Views
{
    public partial class MainWindow : Window
    {
        private readonly MainViewModel _vm;

        public MainWindow()
        {
            InitializeComponent();
            _vm = new MainViewModel(CreateLink());
            DataContext = _vm;
            Closed += (s, e) => _vm.Dispose();
        }

        /// <summary>
        /// 链路选择：找到 remote.token 就连真实从端，否则用模拟链路。
        /// 命令行：--sim 强制模拟；--host &lt;地址&gt; 指定从端地址（默认 127.0.0.1，即同机联调）；
        /// --token &lt;文件&gt; 指定密钥文件。密钥文件默认在 exe 同目录、其上两级的 config 目录中查找。
        /// </summary>
        private static IRobotLink CreateLink()
        {
            string[] args = Environment.GetCommandLineArgs();
            string host = "127.0.0.1";
            string tokenPath = null;
            bool sim = false;
            for (int i = 1; i < args.Length; i++)
            {
                if (args[i] == "--sim") sim = true;
                else if (args[i] == "--host" && i + 1 < args.Length) host = args[++i];
                else if (args[i] == "--token" && i + 1 < args.Length) tokenPath = args[++i];
            }
            if (sim) return new SimulatedRobotLink();

            string baseDir = AppDomain.CurrentDomain.BaseDirectory;
            string[] candidates =
            {
                tokenPath,
                Path.Combine(baseDir, "remote.token"),
                Path.GetFullPath(Path.Combine(baseDir, "..", "..", "config", "remote.token")),
                Path.GetFullPath(Path.Combine(baseDir, "..", "..", "..", "config", "remote.token")),
                Path.GetFullPath(Path.Combine(baseDir, "..", "..", "..", "..", "config", "remote.token")),
            };
            foreach (string c in candidates)
            {
                if (string.IsNullOrEmpty(c) || !File.Exists(c)) continue;
                byte[] token = Encoding.UTF8.GetBytes(File.ReadAllText(c, Encoding.UTF8).Trim());
                if (token.Length < 16) continue;
                return new RemoteRobotLink(new RemoteLinkSettings { Host = host, Token = token });
            }
            return new SimulatedRobotLink();
        }

        // ---------------- 按住才动的按钮：注射器 1/2 推拉、轴4点动 ----------------
        // Tag："1"/"2" 为注射器编号，"A4" 为轴4；CommandParameter 为方向（+1 / -1）。
        // 按下发送方向，抬起/失去鼠标捕获/窗口失焦一律归零，保证松手即停。

        private void HoldButton_Down(object sender, MouseButtonEventArgs e)
        {
            if (!(sender is Button b) || !TryParse(b, out string target, out int dir)) return;
            b.CaptureMouse();
            Apply(target, dir);
        }

        private void HoldButton_Up(object sender, RoutedEventArgs e)
        {
            if (!(sender is Button b) || !TryParse(b, out string target, out _)) return;
            if (b.IsMouseCaptured) b.ReleaseMouseCapture();
            Apply(target, 0);
        }

        private void Window_Deactivated(object sender, EventArgs e) => _vm.StopAllHoldActions();

        private void Apply(string target, int direction)
        {
            switch (target)
            {
                case "1": _vm.SetInjector(1, direction); break;
                case "2": _vm.SetInjector(2, direction); break;
                case "A4": _vm.SetAxis4(direction); break;
            }
        }

        private static bool TryParse(Button b, out string target, out int dir)
        {
            target = Convert.ToString(b.Tag, CultureInfo.InvariantCulture);
            dir = 0;
            if (target != "1" && target != "2" && target != "A4") return false;
            return int.TryParse(Convert.ToString(b.CommandParameter, CultureInfo.InvariantCulture), out dir) &&
                   (dir == 1 || dir == -1);
        }
    }
}
