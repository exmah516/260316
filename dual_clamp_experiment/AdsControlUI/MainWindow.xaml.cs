using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.IO.Pipes;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace DualClampExperimentUI
{
    public partial class MainWindow : Window
    {
        private NamedPipeClientStream? _pipe;
        private StreamWriter? _writer;
        private StreamReader? _reader;
        private readonly SemaphoreSlim _ioLock = new SemaphoreSlim(1, 1);
        private readonly List<double> _force1 = new List<double>();
        private readonly List<double> _force2 = new List<double>();
        private readonly List<double> _torque1 = new List<double>();
        private readonly List<double> _torque2 = new List<double>();
        private readonly DispatcherTimer _pollTimer;
        private bool _isPolling;
        private bool _selfcheckDone;
        private bool _selfcheckBusy;
        private bool _leftLimitValid;
        private bool _setupBusy;
        private bool _setupDone;

        public MainWindow()
        {
            InitializeComponent();
            _pollTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(100) };
            _pollTimer.Tick += async (_, _) => await PollAsync();
            Loaded += async (_, _) =>
            {
                await ConnectAsync();
                _pollTimer.Start();
            };
            Closed += async (_, _) =>
            {
                _pollTimer.Stop();
                try
                {
                    await SendAsync("QUIT");
                }
                catch { }
                DisconnectPipe();
            };
        }

        // 若后端程序未在运行，尝试从相对路径自动拉起
        private void EnsureBackendStarted()
        {
            try
            {
                Process[] existing = Process.GetProcessesByName("DualClampExperiment");
                if (existing != null && existing.Length > 0) return;

                string baseDir = AppDomain.CurrentDomain.BaseDirectory;
                string[] candidatePaths = new[]
                {
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"..\..\..\..\..\x64\Debug\DualClampExperiment.exe")),
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"..\..\..\..\..\x64\Release\DualClampExperiment.exe")),
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"..\..\..\..\x64\Debug\DualClampExperiment.exe")),
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"..\..\..\..\x64\Release\DualClampExperiment.exe")),
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"DualClampExperiment.exe")),
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"..\DualClampExperiment.exe")),
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"..\..\DualClampExperiment.exe"))
                };

                foreach (var path in candidatePaths)
                {
                    if (File.Exists(path))
                    {
                        var psi = new ProcessStartInfo
                        {
                            FileName = path,
                            Arguments = "--no-ui",
                            WorkingDirectory = System.IO.Path.GetDirectoryName(path),
                            UseShellExecute = true
                        };
                        Process.Start(psi);
                        Thread.Sleep(400);
                        break;
                    }
                }
            }
            catch { }
        }

        private void DisconnectPipe()
        {
            try
            {
                _writer?.Dispose();
                _writer = null;
                _reader?.Dispose();
                _reader = null;
                if (_pipe != null)
                {
                    _pipe.Dispose();
                    _pipe = null;
                }
            }
            catch { }
        }

        private async Task ConnectAsync()
        {
            await _ioLock.WaitAsync();
            try
            {
                DisconnectPipe();
                EnsureBackendStarted();

                var pipe = new NamedPipeClientStream(".", "DualClampExperiment", PipeDirection.InOut, PipeOptions.Asynchronous);
                await pipe.ConnectAsync(1500);

                _pipe = pipe;
                _writer = new StreamWriter(_pipe, new UTF8Encoding(false), 4096, true) { AutoFlush = true };
                _reader = new StreamReader(_pipe, new UTF8Encoding(false), false, 4096, true);

                SetPipeStatus(true, "UI管道: 已连接");
                ErrorText.Text = string.Empty;

                // 连上管道后，主动通知后端确保 ADS 连接并获取初始状态
                await SendCommandInternalAsync("CONNECT_ADS");
                await SendCommandInternalAsync("GET");
            }
            catch (Exception ex)
            {
                DisconnectPipe();
                SetPipeStatus(false, "UI管道: 未连接");
                SetAdsStatus(false, "ADS: 未连接");
                ErrorText.Text = "管道连接失败（请确认后端 DualClampExperiment.exe 是否运行）：" + ex.Message;
            }
            finally
            {
                _ioLock.Release();
            }
        }

        private void SetPipeStatus(bool connected, string text)
        {
            PipeStatusText.Text = text;
            PipeIndicator.Fill = new SolidColorBrush(connected ? Color.FromRgb(16, 185, 129) : Color.FromRgb(239, 68, 68));
        }

        private void SetAdsStatus(bool connected, string text)
        {
            AdsStatusText.Text = text;
            AdsIndicator.Fill = new SolidColorBrush(connected ? Color.FromRgb(16, 185, 129) : Color.FromRgb(239, 68, 68));
        }

        private async void Connect_Click(object sender, RoutedEventArgs e) => await ConnectAsync();

        private async void ConnectAds_Click(object sender, RoutedEventArgs e) => await SendAsync("CONNECT_ADS");

        private async void SelfCheck_Click(object sender, RoutedEventArgs e)
        {
            await SendAsync("SELF_CHECK");
        }

        private async void Prepare_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                const string command = "PREPARE|instrument={0}|moving_axis={1}|axis1_distance={2}|axis6_distance={3}|axis2_angle={4}|axis7_angle={5}|return_retract={6}|return_velocity={7}|return_acc={8}|return_dec={9}|return_jerk={10}|recovery_mode={11}";
                string text = string.Format(CultureInfo.InvariantCulture, command,
                    InstrumentBox.SelectedIndex,
                    ((ComboBoxItem)MovingAxisBox.SelectedItem).Tag,
                    Number(Axis1Pos), Number(Axis6Pos), Number(Axis2Angle), Number(Axis7Angle),
                    Number(ReturnDistance), Number(ReturnVelocity), Number(ReturnAcceleration),
                    Number(ReturnDeceleration), Number(ReturnJerk), RecoveryBox.SelectedIndex);
                await SendAsync(text);
            }
            catch (Exception ex)
            {
                ErrorText.Text = "准备定位参数无效：" + ex.Message;
            }
        }

        private async Task PollAsync()
        {
            if (_isPolling) return;
            if (_pipe == null || !_pipe.IsConnected) return;
            _isPolling = true;
            try
            {
                await SendAsync("GET");
            }
            finally
            {
                _isPolling = false;
            }
        }

        private async void Start_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (!_setupDone)
                {
                    ErrorText.Text = "请先完成自检并点击准备定位";
                    return;
                }
                await SendAsync("START");
            }
            catch (Exception ex) { ErrorText.Text = ex.Message; }
        }

        private async void Abort_Click(object sender, RoutedEventArgs e) => await SendAsync("ABORT");

        private async void Save_Click(object sender, RoutedEventArgs e)
        {
            string path = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "records", DateTime.Now.ToString("yyyyMMdd_HHmmss"));
            await SendAsync("SAVE|" + path);
        }

        private static string Number(TextBox box) => double.Parse(box.Text, CultureInfo.InvariantCulture).ToString(CultureInfo.InvariantCulture);

        private async Task<string> SendAsync(string command)
        {
            await _ioLock.WaitAsync();
            try
            {
                return await SendCommandInternalAsync(command);
            }
            finally
            {
                _ioLock.Release();
            }
        }

        private async Task<string> SendCommandInternalAsync(string command)
        {
            if (_pipe == null || !_pipe.IsConnected || _writer == null || _reader == null)
            {
                SetPipeStatus(false, "UI管道: 未连接");
                return string.Empty;
            }
            try
            {
                await _writer.WriteLineAsync(command);
                string response = await _reader.ReadLineAsync() ?? string.Empty;
                if (string.IsNullOrEmpty(response))
                {
                    SetPipeStatus(false, "UI管道: 未连接");
                    DisconnectPipe();
                    return string.Empty;
                }
                ParseState(response);
                return response;
            }
            catch (Exception ex)
            {
                SetPipeStatus(false, "UI管道: 未连接");
                DisconnectPipe();
                ErrorText.Text = "管道通讯中断：" + ex.Message;
                return string.Empty;
            }
        }

        private void ParseState(string response)
        {
            if (response.StartsWith("STATE|", StringComparison.Ordinal))
            {
                SetPipeStatus(true, "UI管道: 已连接");
                string[] p = response.Split('|');
                if (p.Length >= 27)
                {
                    double axis1Pos = double.Parse(p[2], CultureInfo.InvariantCulture);
                    double axis6Pos = double.Parse(p[3], CultureInfo.InvariantCulture);
                    double axis1Vel = double.Parse(p[4], CultureInfo.InvariantCulture);
                    double axis6Vel = double.Parse(p[5], CultureInfo.InvariantCulture);
                    double axis1Acc = double.Parse(p[6], CultureInfo.InvariantCulture);
                    double axis6Acc = double.Parse(p[7], CultureInfo.InvariantCulture);
                    double axis2Angle = double.Parse(p[8], CultureInfo.InvariantCulture);
                    double axis7Angle = double.Parse(p[9], CultureInfo.InvariantCulture);
                    LiveMotionText.Text = string.Format(CultureInfo.InvariantCulture,
                        "轴1 位置/速度/加速度：{0:F3} mm / {1:F3} mm/s / {2:F3} mm/s²\n轴6 位置/速度/加速度：{3:F3} mm / {4:F3} mm/s / {5:F3} mm/s²\n轴2/轴7 周向角度：{6:F3}° / {7:F3}°",
                        axis1Pos, axis1Vel, axis1Acc, axis6Pos, axis6Vel, axis6Acc, axis2Angle, axis7Angle);
                    Add(_force1, double.Parse(p[10], CultureInfo.InvariantCulture));
                    Add(_force2, double.Parse(p[11], CultureInfo.InvariantCulture));
                    Add(_torque1, double.Parse(p[12], CultureInfo.InvariantCulture));
                    Add(_torque2, double.Parse(p[13], CultureInfo.InvariantCulture));
                    bool adsConnected = p[14] == "1";
                    _selfcheckDone = p[15] == "1";
                    _selfcheckBusy = p[16] == "1";
                    _leftLimitValid = p[17] == "1";
                    _setupBusy = p[20] == "1";
                    _setupDone = p[21] == "1";
                    PhaseText.Text = _selfcheckBusy
                        ? "SelfCheck (正在执行自检)"
                        : PhaseName(int.Parse(p[1], CultureInfo.InvariantCulture));
                    SetAdsStatus(adsConnected, adsConnected ? "ADS: 正常 (Port 851)" : "ADS: 未连接");
                    ErrorText.Text = p[26];
                    SelfCheckButton.IsEnabled = adsConnected && !_selfcheckBusy;
                    PrepareButton.IsEnabled = adsConnected && _selfcheckDone && _leftLimitValid && !_selfcheckBusy && !_setupBusy;
                    StartButton.IsEnabled = adsConnected && _setupDone && !_setupBusy;
                    Draw(ForceCanvas, Force1Line, _force1, Force2Line, _force2);
                    Draw(TorqueCanvas, Torque1Line, _torque1, Torque2Line, _torque2);
                }
            }
            else if (response.StartsWith("OK|CONNECT_ADS", StringComparison.Ordinal))
            {
                SetAdsStatus(true, "ADS: 正常 (Port 851)");
            }
            else if (response.StartsWith("OK|DISCONNECT_ADS", StringComparison.Ordinal))
            {
                SetAdsStatus(false, "ADS: 已断开");
            }
            else if (response.StartsWith("ERROR|", StringComparison.Ordinal))
            {
                ErrorText.Text = response.Substring(6);
            }
        }

        private static void Add(List<double> values, double value)
        {
            values.Add(value);
            if (values.Count > 240) values.RemoveAt(0);
        }

        private static string PhaseName(int phase) => phase switch
        {
            0 => "Idle (空闲待命)",
            1 => "Prepare (准备阶段)",
            2 => "Rotate (旋转对齐)",
            3 => "Baseline (基准采样)",
            4 => "FixedHold (固定端保持)",
            5 => "ReleaseMoving (运动端释放)",
            6 => "ReturnMoving (运动端回程)",
            7 => "ReclampMoving (运动端重新夹紧)",
            8 => "RecoverHold (恢复保持)",
            9 => "RecoverMove (恢复输送)",
            10 => "Completed (实验完成)",
            11 => "Aborted (已中止)",
            12 => "Error (错误)",
            13 => "SelfCheckDone (自检完成，等待准备定位)",
            14 => "SetupMove (准备定位)",
            15 => "ReadyForClamp (准备完成，等待开始)",
            _ => "Unknown"
        };

        private static void Draw(Canvas canvas, Polyline a, List<double> av, Polyline b, List<double> bv)
        {
            if (canvas.ActualWidth < 10 || canvas.ActualHeight < 10) return;
            double min = double.PositiveInfinity, max = double.NegativeInfinity;
            foreach (double v in av) { min = Math.Min(min, v); max = Math.Max(max, v); }
            foreach (double v in bv) { min = Math.Min(min, v); max = Math.Max(max, v); }
            if (double.IsNaN(min) || double.IsInfinity(min) || double.IsNaN(max) || double.IsInfinity(max)) return;
            if (Math.Abs(max - min) < 1e-9) { max += 1; min -= 1; }
            PointCollection pa = new PointCollection(), pb = new PointCollection();
            AddPoints(pa, av, canvas, min, max); AddPoints(pb, bv, canvas, min, max);
            a.Points = pa; b.Points = pb;
        }

        private static void AddPoints(PointCollection points, List<double> values, Canvas canvas, double min, double max)
        {
            for (int i = 0; i < values.Count; i++)
            {
                double x = values.Count <= 1 ? 0 : i * canvas.ActualWidth / (values.Count - 1);
                double y = canvas.ActualHeight - (values[i] - min) / (max - min) * canvas.ActualHeight;
                points.Add(new Point(x, y));
            }
        }
    }
}
