using System;
using System.Globalization;
using System.IO;
using System.Text;
using System.Windows;
using System.Windows.Input;
using System.Windows.Controls;
using MasterConsole.Services;
using MasterConsole.ViewModels;

namespace MasterConsole.Views
{
    public partial class MainWindow : Window
    {
        private readonly MainViewModel _vm;
        // CreateLink 的结果说明，窗口建好后写进事件日志，避免“静默退回模拟链路”。
        private static string _linkNotice;
        private static string _linkTokenPath;
        private readonly DsaJpegReceiver _dsaReceiver;
        private readonly string[] _dsaStreamStatus = { "未连接", "未连接" };
        private readonly object _dsaStatusSync = new object();

        public MainWindow()
        {
            InitializeComponent();
            _vm = new MainViewModel(CreateLink());
            DataContext = _vm;
            _dsaReceiver = new DsaJpegReceiver(DsaJpegReceiverSettings.Load());
            _dsaReceiver.FrameReceived += DsaFrameReceived;
            _dsaReceiver.StatusChanged += DsaStatusChanged;
            _dsaReceiver.Start();
            if (_linkNotice != null) _vm.PostLog("warn", _linkNotice);
            else if (_linkTokenPath != null) _vm.PostLog("info", "已加载密钥：" + _linkTokenPath);
            Closed += (s, e) => { _dsaReceiver.Dispose(); DsaImageView.Dispose(); HikCamera1.Dispose(); HikCamera2.Dispose(); _vm.Dispose(); RobotModelViewport.Dispose(); };
        }

        private void DsaFrameReceived(int stream, byte[] jpeg, DateTime receivedUtc) => DsaImageView.PublishJpeg(jpeg, receivedUtc);

        private void DsaStatusChanged(int stream, string status, string detail)
        {
            lock (_dsaStatusSync) _dsaStreamStatus[stream] = status;
            string summary;
            string overall;
            lock (_dsaStatusSync)
            {
                summary = string.Format("流0 {0}；流1 {1}；{2}", _dsaStreamStatus[0], _dsaStreamStatus[1], detail);
                overall = _dsaStreamStatus[0] == "失败" || _dsaStreamStatus[1] == "失败" ? "失败" : "等待";
            }
            DsaImageView.UpdateReceiverStatus(overall, summary);
        }

        private void DsaFast_Click(object sender, RoutedEventArgs e)
        {
            if (!_dsaReceiver.SendSpeedCommand(true)) _vm.PostLog("warn", "DSA快发命令未发送：未配置有效 ScreenCut IP 或发送失败。");
        }

        private void DsaSlow_Click(object sender, RoutedEventArgs e)
        {
            if (!_dsaReceiver.SendSpeedCommand(false)) _vm.PostLog("warn", "DSA慢发命令未发送：未配置有效 ScreenCut IP 或发送失败。");
        }

        private void DisplaySelect_Click(object sender, RoutedEventArgs e)
        {
            if (!(sender is Button button)) return;
            SetDisplayLayout(Convert.ToString(button.Tag, CultureInfo.InvariantCulture));
        }

        private void SetDisplayLayout(string selected)
        {
            UIElement[] views = { DsaImageView, HikCamera1, HikCamera2 };
            var keys = new[] { "dsa", "camera1", "camera2" };
            int main = Array.IndexOf(keys, selected);
            if (main < 0) main = 0;
            for (int i = 0, aux = 0; i < views.Length; i++)
            {
                if (i == main)
                {
                    Grid.SetColumn(views[i], 0);
                    Grid.SetRow(views[i], 0);
                    Grid.SetRowSpan(views[i], 2);
                }
                else
                {
                    Grid.SetColumn(views[i], 2);
                    Grid.SetRow(views[i], aux++);
                    Grid.SetRowSpan(views[i], 1);
                }
            }
            if (string.Equals(selected, "model", StringComparison.OrdinalIgnoreCase))
            {
                DisplayTopRow.Height = new GridLength(2, GridUnitType.Star);
                DisplayBottomRow.Height = new GridLength(3, GridUnitType.Star);
                DisplayLogColumn.Width = new GridLength(120);
            }
            else
            {
                DisplayTopRow.Height = new GridLength(3, GridUnitType.Star);
                DisplayBottomRow.Height = new GridLength(2, GridUnitType.Star);
                DisplayLogColumn.Width = new GridLength(180);
            }
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
            if (sim)
            {
                _linkNotice = "已按 --sim 参数使用模拟链路，命令不会发给从端。";
                return new SimulatedRobotLink();
            }

            // 从 exe 所在目录逐级向上查找（exe 通常在 bin/Debug/net472 下，密钥在项目根的 config 目录）。
            var candidates = new System.Collections.Generic.List<string>();
            if (!string.IsNullOrEmpty(tokenPath)) candidates.Add(tokenPath);
            string dir = AppDomain.CurrentDomain.BaseDirectory;
            for (int level = 0; level < 10 && !string.IsNullOrEmpty(dir); level++)
            {
                candidates.Add(Path.Combine(dir, "remote.token"));
                candidates.Add(Path.Combine(dir, "config", "remote.token"));
                dir = Path.GetDirectoryName(dir.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar));
            }
            string rejected = null;
            foreach (string c in candidates)
            {
                if (!File.Exists(c)) continue;
                byte[] token = Encoding.UTF8.GetBytes(File.ReadAllText(c, Encoding.UTF8).Trim());
                if (token.Length < 16) { rejected = c; continue; }
                _linkNotice = null;
                _linkTokenPath = c;
                // 手柄服务随链路创建：应用启动即开始打开本机手柄，连接从端时已就绪。
                return new RemoteRobotLink(new RemoteLinkSettings { Host = host, Token = token }, new HandleService());
            }
            _linkNotice = rejected != null
                ? "密钥文件太短（至少 16 个字符）：" + rejected + "。已退回模拟链路，命令不会发给从端。"
                : "未找到 remote.token，已退回模拟链路，命令不会发给从端。请把密钥文件放到 config 目录或 exe 同目录。";
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
