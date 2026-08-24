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
        private bool _loaded;
        private bool _selfcheckDone;
        private bool _selfcheckBusy;
        private bool _leftLimitValid;
        private bool _setupBusy;
        private bool _setupDone;
        private string _guidewireAxis5Text = "430";
        private string CurrentMode => ((ComboBoxItem)ExperimentModeBox.SelectedItem)?.Tag?.ToString() ?? "legacy";

        public MainWindow()
        {
            InitializeComponent();
            _pollTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(100) };
            _pollTimer.Tick += async (_, _) => await PollAsync();
            Loaded += async (_, _) =>
            {
                _loaded = true;
                UpdateModeView();
                await ConnectAsync();
                _pollTimer.Start();
            };
            Closed += async (_, _) =>
            {
                _pollTimer.Stop();
                try { await SendAsync("QUIT"); } catch { }
                DisconnectPipe();
            };
        }

        private void EnsureBackendStarted()
        {
            try
            {
                if (Process.GetProcessesByName("DualClampExperiment").Length > 0) return;
                string baseDir = AppDomain.CurrentDomain.BaseDirectory;
                string[] candidates =
                {
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"..\..\..\..\x64\Debug\DualClampExperiment.exe")),
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"..\..\..\..\x64\Release\DualClampExperiment.exe")),
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"..\..\..\..\..\x64\Debug\DualClampExperiment.exe")),
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"..\..\..\..\..\x64\Release\DualClampExperiment.exe")),
                    System.IO.Path.Combine(baseDir, "DualClampExperiment.exe"),
                    System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, @"..\DualClampExperiment.exe"))
                };
                foreach (string path in candidates)
                {
                    if (!File.Exists(path)) continue;
                    Process.Start(new ProcessStartInfo
                    {
                        FileName = path,
                        Arguments = "--no-ui",
                        WorkingDirectory = System.IO.Path.GetDirectoryName(path),
                        UseShellExecute = true
                    });
                    Thread.Sleep(400);
                    return;
                }
            }
            catch { }
        }

        private void DisconnectPipe()
        {
            try
            {
                _writer?.Dispose(); _writer = null;
                _reader?.Dispose(); _reader = null;
                _pipe?.Dispose(); _pipe = null;
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
                _writer = new StreamWriter(pipe, new UTF8Encoding(false), 4096, true) { AutoFlush = true };
                _reader = new StreamReader(pipe, new UTF8Encoding(false), false, 4096, true);
                SetPipeStatus(true, "UI管道: 已连接");
                await SendCommandInternalAsync("CONNECT_ADS");
                await SendCommandInternalAsync(CurrentMode == "legacy" ? "GET" : "GET_PROGRAM");
            }
            catch (Exception ex)
            {
                DisconnectPipe();
                SetPipeStatus(false, "UI管道: 未连接");
                SetAdsStatus(false, "ADS: 未连接");
                ErrorText.Text = "管道连接失败：" + ex.Message;
            }
            finally { _ioLock.Release(); }
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

        private async void ExperimentMode_Changed(object sender, SelectionChangedEventArgs e)
        {
            if (!_loaded || _pipe == null || !_pipe.IsConnected) return;
            UpdateModeView();
            try
            {
                string mode = CurrentMode;
                if (mode == "legacy")
                    await SendAsync("PROGRAM_MODE|mode=legacy");
                else
                    await SendAsync("PROGRAM_MODE|mode=" + mode);
                await PollAsync();
            }
            catch (Exception ex) { ErrorText.Text = "切换实验模式失败：" + ex.Message; }
        }

        private void UpdateModeView()
        {
            bool legacy = CurrentMode == "legacy";
            LegacyToolbar.Visibility = legacy ? Visibility.Visible : Visibility.Collapsed;
            LegacyPanel.Visibility = legacy ? Visibility.Visible : Visibility.Collapsed;
            ProgramPanel.Visibility = legacy ? Visibility.Collapsed : Visibility.Visible;
            bool guidewire = CurrentMode == "guidewire";
            ProgramPanelTitle.Text = guidewire ? "导丝程序递送参数" : "导管程序递送参数";
            Axis5PositionLabel.Text = guidewire ? "轴5距左限位 (mm)" : "轴1准备位置 (mm)";
            Axis6CalculatedLabel.Text = guidewire ? "轴6自动位置 (mm)" : "轴1触发位置 (mm)";
            if (guidewire)
            {
                if (!ProgramAxis5Pos.IsEnabled) ProgramAxis5Pos.Text = _guidewireAxis5Text;
                ProgramAxis5Pos.IsEnabled = true;
            }
            else
            {
                if (ProgramAxis5Pos.IsEnabled) _guidewireAxis5Text = ProgramAxis5Pos.Text;
                ProgramAxis5Pos.Text = "23";
                ProgramAxis5Pos.IsEnabled = false;
            }
            ProgramAxis6Calculated.Text = guidewire ? (ParseOrDefault(ProgramAxis5Pos, 430.0) + 21.0).ToString("F3", CultureInfo.InvariantCulture) : "3.000";
            ProgramAngleLabel.Text = guidewire ? "轴7角度 (deg)" : "轴2角度 (deg)";
            ForceTitle.Text = guidewire ? "导丝侧轴向力 fn2" : legacy ? "旧模式轴向力 fn1 / fn2" : "导管侧轴向力 fn1";
            TorqueTitle.Text = guidewire ? "导丝侧切向力 ft2" : legacy ? "旧模式切向力 ft1 / ft2" : "导管侧切向力 ft1";
            Force2Line.Visibility = legacy ? Visibility.Visible : Visibility.Collapsed;
            Torque2Line.Visibility = legacy ? Visibility.Visible : Visibility.Collapsed;
            _force1.Clear(); _force2.Clear(); _torque1.Clear(); _torque2.Clear();
        }

        private void ProgramAxis5Pos_Changed(object sender, TextChangedEventArgs e)
        {
            if (_loaded && CurrentMode == "guidewire")
                ProgramAxis6Calculated.Text = (ParseOrDefault(ProgramAxis5Pos, 430.0) + 21.0).ToString("F3", CultureInfo.InvariantCulture);
        }

        private async void Prepare_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (CurrentMode == "legacy")
                {
                    string command = string.Format(CultureInfo.InvariantCulture,
                        "PREPARE|moving_axis={0}|axis1_distance={1}|axis6_distance={2}|axis2_angle={3}|axis7_angle={4}|return_retract={5}|return_velocity={6}|return_acc={7}|return_dec={8}|return_jerk={9}|recovery_mode={10}|record_name={11}",
                        ((ComboBoxItem)MovingAxisBox.SelectedItem).Tag, Number(Axis1Pos), Number(Axis6Pos), Number(Axis2Angle), Number(Axis7Angle),
                        Number(ReturnDistance), Number(ReturnVelocity), Number(ReturnAcceleration), Number(ReturnDeceleration), Number(ReturnJerk), RecoveryBox.SelectedIndex, RecordSuffix());
                    await SendAsync(command);
                    return;
                }
                string mode = CurrentMode;
                string angleKey = mode == "guidewire" ? "axis7_angle" : "axis2_angle";
                string commandText = string.Format(CultureInfo.InvariantCulture,
                    "PROGRAM_PREPARE|mode={0}|axis5_from_left={1}|{2}={3}|cycle_count={4}|final_forward_distance={5}|forward_velocity={6}|forward_acceleration={7}|forward_deceleration={8}|forward_jerk={9}|return_velocity={10}|return_acceleration={11}|return_deceleration={12}|return_jerk={13}|record_name={14}",
                    mode, Number(ProgramAxis5Pos), angleKey, Number(ProgramAngle), Int(ProgramCycleCount), Number(ProgramFinalDistance), Number(ProgramForwardVelocity),
                    Number(ProgramForwardAcceleration), Number(ProgramForwardDeceleration), Number(ProgramForwardJerk), Number(ProgramReturnVelocity),
                    Number(ProgramReturnAcceleration), Number(ProgramReturnDeceleration), Number(ProgramReturnJerk), RecordSuffix());
                await SendAsync(commandText);
            }
            catch (Exception ex) { ErrorText.Text = "准备定位参数无效：" + ex.Message; }
        }

        private async Task PollAsync()
        {
            if (_isPolling || _pipe == null || !_pipe.IsConnected) return;
            _isPolling = true;
            try { await SendAsync(CurrentMode == "legacy" ? "GET" : "GET_PROGRAM"); }
            finally { _isPolling = false; }
        }

        private async void Start_Click(object sender, RoutedEventArgs e)
        {
            if (!_setupDone) { ErrorText.Text = "请等待PLC自动自检完成并完成准备定位"; return; }
            await SendAsync(CurrentMode == "legacy" ? "START" : "PROGRAM_START");
        }

        private async void Zero_Click(object sender, RoutedEventArgs e)
        {
            await SendAsync(CurrentMode == "legacy" ? "ZERO_FORCE" : "PROGRAM_ZERO_FORCE");
        }

        private async void Abort_Click(object sender, RoutedEventArgs e) => await SendAsync(CurrentMode == "legacy" ? "ABORT" : "PROGRAM_ABORT");

        private async void Save_Click(object sender, RoutedEventArgs e)
        {
            await SendAsync(CurrentMode == "legacy" ? "SAVE|" : "PROGRAM_SAVE|");
        }

        private async Task<string> SendAsync(string command)
        {
            await _ioLock.WaitAsync();
            try { return await SendCommandInternalAsync(command); }
            finally { _ioLock.Release(); }
        }

        private async Task<string> SendCommandInternalAsync(string command)
        {
            if (_pipe == null || !_pipe.IsConnected || _writer == null || _reader == null) return string.Empty;
            try
            {
                await _writer.WriteLineAsync(command);
                string response = await _reader.ReadLineAsync() ?? string.Empty;
                if (string.IsNullOrEmpty(response)) { DisconnectPipe(); return string.Empty; }
                ParseState(response);
                return response;
            }
            catch (Exception ex) { DisconnectPipe(); ErrorText.Text = "管道通讯中断：" + ex.Message; return string.Empty; }
        }

        private void ParseState(string response)
        {
            if (response.StartsWith("STATE|", StringComparison.Ordinal)) ParseLegacyState(response);
            else if (response.StartsWith("PROGRAM_STATE|", StringComparison.Ordinal)) ParseProgramState(response);
            else if (response.StartsWith("OK|CONNECT_ADS", StringComparison.Ordinal)) SetAdsStatus(true, "ADS: 正常 (Port 851)");
            else if (response.StartsWith("ERROR|", StringComparison.Ordinal)) ErrorText.Text = response.Substring(6);
        }

        private void ParseLegacyState(string response)
        {
            string[] p = response.Split('|');
            if (p.Length < 27) return;
            double a1 = D(p[2]), a6 = D(p[3]), v1 = D(p[4]), v6 = D(p[5]), acc1 = D(p[6]), acc6 = D(p[7]);
            LiveMotionText.Text = string.Format(CultureInfo.InvariantCulture, "轴1：{0:F3} mm / {1:F3} mm/s / {2:F3} mm/s²\n轴6：{3:F3} mm / {4:F3} mm/s / {5:F3} mm/s²\n轴2/轴7角度：{6:F3}° / {7:F3}°", a1, v1, acc1, a6, v6, acc6, D(p[8]), D(p[9]));
            Add(_force1, D(p[10])); Add(_force2, D(p[11])); Add(_torque1, D(p[12])); Add(_torque2, D(p[13]));
            bool ads = p[14] == "1"; _selfcheckDone = p[15] == "1"; _selfcheckBusy = p[16] == "1"; _leftLimitValid = p[17] == "1"; _setupBusy = p[20] == "1"; _setupDone = p[21] == "1";
            PhaseText.Text = _selfcheckBusy ? "SelfCheck (正在执行自检)" : LegacyPhase(int.Parse(p[1], CultureInfo.InvariantCulture));
            SelfCheckText.Text = _selfcheckBusy ? "PLC自检: 执行中" : _selfcheckDone ? "PLC自检: 已完成" : "PLC自检: 未完成";
            CycleText.Text = "旧模式"; SetAdsStatus(ads, ads ? "ADS: 正常 (Port 851)" : "ADS: 未连接");
            int legacyZeroBusy = 27, legacyZeroDone = 28, legacyError = 26;
            ZeroStatusText.Text = p.Length > legacyZeroDone && p[legacyZeroBusy] == "1" ? "力感零点：采集中" : p.Length > legacyZeroDone && p[legacyZeroDone] == "1" ? "力感零点：已完成" : "力感零点：未完成";
            string legacyDirectory = p.Length > 36 ? p[36] : string.Empty;
            bool legacyArchived = p.Length > 37 && p[37] == "1";
            RecordStatusText.Text = legacyArchived ? "实时记录：已归档（" + p[34] + "点）" : p.Length > 34 && p[33] == "1" ? "实时记录：进行中（" + p[34] + "点）" : p.Length > 34 && p[34] != "0" ? "实时记录：待归档（" + p[34] + "点）" : "实时记录：未开始";
            if (!string.IsNullOrWhiteSpace(legacyDirectory)) RecordStatusText.Text += "\n目录：" + legacyDirectory;
            ZeroValuesText.Text = p.Length > 32 ? string.Format(CultureInfo.InvariantCulture, "fn1零点：{0:F3}  ft1零点：{1:F3}\nfn2零点：{2:F3}  ft2零点：{3:F3}", D(p[29]), D(p[30]), D(p[31]), D(p[32])) : "";
            ErrorText.Text = p[legacyError];
            PrepareButton.IsEnabled = ads && _selfcheckDone && _leftLimitValid && !_selfcheckBusy && !_setupBusy; StartButton.IsEnabled = ads && _setupDone && p.Length > 28 && p[28] == "1" && !_setupBusy; ZeroButton.IsEnabled = ads && _selfcheckDone && _setupDone && !_setupBusy && !_selfcheckBusy;
            Draw(ForceCanvas, Force1Line, _force1, Force2Line, _force2); Draw(TorqueCanvas, Torque1Line, _torque1, Torque2Line, _torque2);
        }

        private void ParseProgramState(string response)
        {
            string[] p = response.Split('|');
            if (p.Length < 42) return;
            int phase = int.Parse(p[2], CultureInfo.InvariantCulture); _setupBusy = p[5] == "1"; _setupDone = p[6] == "1"; _selfcheckDone = p[7] == "1";
            SelfCheckText.Text = _selfcheckDone ? "PLC自检: 已完成" : "PLC自检: 执行中";
            bool guidewire = CurrentMode == "guidewire";
            if (guidewire)
            {
                LiveMotionText.Text = string.Format(CultureInfo.InvariantCulture, "轴5：{0:F3} mm / {1:F3} mm/s / {2:F3} mm/s²\n轴6：{3:F3} mm / {4:F3} mm/s / {5:F3} mm/s²\n轴7角度：{6:F3}°", D(p[14]), D(p[15]), D(p[16]), D(p[17]), D(p[18]), D(p[19]), D(p[20]));
                Add(_force1, D(p[25])); Add(_torque1, D(p[26]));
            }
            else
            {
                LiveMotionText.Text = string.Format(CultureInfo.InvariantCulture, "轴1：{0:F3} mm / {1:F3} mm/s / {2:F3} mm/s²\n轴2角度：{3:F3}°", D(p[8]), D(p[9]), D(p[10]), D(p[11]));
                Add(_force1, D(p[23])); Add(_torque1, D(p[24]));
            }
            CycleText.Text = string.Format(CultureInfo.InvariantCulture, "周期：{0} / {1}", p[3], p[4]);
            bool ads = p[40] == "1";
            int programZeroBusy = 42, programZeroDone = 43, programError = 41;
            ZeroStatusText.Text = p.Length > programZeroDone && p[programZeroBusy] == "1" ? "力感零点：采集中" : p.Length > programZeroDone && p[programZeroDone] == "1" ? "力感零点：已完成" : "力感零点：未完成";
            string programDirectory = p.Length > 51 ? p[51] : string.Empty;
            bool programArchived = p.Length > 52 && p[52] == "1";
            RecordStatusText.Text = programArchived ? "实时记录：已归档（" + p[49] + "点）" : p.Length > 49 && p[48] == "1" ? "实时记录：进行中（" + p[49] + "点）" : p.Length > 49 && p[49] != "0" ? "实时记录：待归档（" + p[49] + "点）" : "实时记录：未开始";
            if (!string.IsNullOrWhiteSpace(programDirectory)) RecordStatusText.Text += "\n目录：" + programDirectory;
            ZeroValuesText.Text = p.Length > 47 ? string.Format(CultureInfo.InvariantCulture, "fn1零点：{0:F3}  ft1零点：{1:F3}\nfn2零点：{2:F3}  ft2零点：{3:F3}", D(p[44]), D(p[45]), D(p[46]), D(p[47])) : "";
            PhaseText.Text = ProgramPhase(phase); ErrorText.Text = p[programError];
            SetAdsStatus(ads, ads ? "ADS: 正常 (Port 851)" : "ADS: 未连接");
            PrepareButton.IsEnabled = ads && _selfcheckDone && !_setupBusy; StartButton.IsEnabled = ads && _setupDone && p.Length > programZeroDone && p[programZeroDone] == "1" && !_setupBusy; ZeroButton.IsEnabled = ads && _selfcheckDone && _setupDone && !_setupBusy && phase == 2;
            Draw(ForceCanvas, Force1Line, _force1, Force2Line, _force2); Draw(TorqueCanvas, Torque1Line, _torque1, Torque2Line, _torque2);
        }

        private static double D(string value) => double.Parse(value, CultureInfo.InvariantCulture);
        private static string Number(TextBox box) => double.Parse(box.Text, CultureInfo.InvariantCulture).ToString(CultureInfo.InvariantCulture);
        private static string Int(TextBox box) => uint.Parse(box.Text, CultureInfo.InvariantCulture).ToString(CultureInfo.InvariantCulture);
        private string RecordSuffix() => (RecordSuffixText.Text ?? "experiment").Replace("|", " ").Replace("\r", " ").Replace("\n", " ").Trim();
        private static bool phase_legacy_recording(string[] parts) => parts.Length > 1 && int.TryParse(parts[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out int phase) && phase >= 3 && phase <= 9;
        private static double ParseOrDefault(TextBox box, double fallback) { double value; return double.TryParse(box.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out value) ? value : fallback; }
        private static void Add(List<double> values, double value) { values.Add(value); if (values.Count > 240) values.RemoveAt(0); }
        private static string LegacyPhase(int phase) => phase == 13 ? "自检完成" : phase == 14 ? "准备定位" : phase == 15 ? "准备完成" : phase.ToString(CultureInfo.InvariantCulture);
        private static string ProgramPhase(int phase) => new[] { "空闲", "准备定位", "准备完成", "基准采样", "前向至触发边", "运动端释放", "运动端回退", "重新夹紧", "周期判定", "最终前向", "完成", "已中止", "错误" }[Math.Max(0, Math.Min(12, phase))];

        private static void Draw(Canvas canvas, Polyline a, List<double> av, Polyline b, List<double> bv)
        {
            if (canvas.ActualWidth < 10 || canvas.ActualHeight < 10) return;
            var all = new List<double>(av); all.AddRange(bv); if (all.Count == 0) return;
            double min = double.PositiveInfinity, max = double.NegativeInfinity; foreach (double v in all) { min = Math.Min(min, v); max = Math.Max(max, v); }
            if (Math.Abs(max - min) < 1e-9) { max += 1; min -= 1; }
            var pa = new PointCollection(); var pb = new PointCollection(); AddPoints(pa, av, canvas, min, max); AddPoints(pb, bv, canvas, min, max); a.Points = pa; b.Points = pb;
        }
        private static void AddPoints(PointCollection points, List<double> values, Canvas canvas, double min, double max)
        { for (int i = 0; i < values.Count; i++) points.Add(new Point(values.Count <= 1 ? 0 : i * canvas.ActualWidth / (values.Count - 1), canvas.ActualHeight - (values[i] - min) / (max - min) * canvas.ActualHeight)); }
    }
}
