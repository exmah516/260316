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
        private bool _standaloneRecording;
        private bool _standaloneSelfcheckDone;
        private bool _standaloneLegacyBusy;
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
                if (CurrentMode == "legacy") await SendCommandInternalAsync("GET_STANDALONE_RECORD");
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
            Visibility catheterVisibility = guidewire ? Visibility.Collapsed : Visibility.Visible;
            Visibility guidewireVisibility = guidewire ? Visibility.Visible : Visibility.Collapsed;
            CatheterAxis1PrepareLabel.Visibility = catheterVisibility;
            CatheterAxis1TriggerLabel.Visibility = catheterVisibility;
            ProgramAxis1PreparePos.Visibility = catheterVisibility;
            ProgramAxis1TriggerPos.Visibility = catheterVisibility;
            Axis5PositionLabel.Visibility = guidewireVisibility;
            ProgramAxis5Pos.Visibility = guidewireVisibility;
            Axis6PrepareLabel.Visibility = guidewireVisibility;
            ProgramAxis6PreparePos.Visibility = guidewireVisibility;
            Axis6TriggerLabel.Visibility = guidewireVisibility;
            ProgramAxis6TriggerPos.Visibility = guidewireVisibility;
            ProgramAngleLabel.Text = guidewire ? "轴7角度 (deg)" : "轴2角度 (deg)";
            ProgramCylinder1Coupling.Visibility = guidewire ? Visibility.Collapsed : Visibility.Visible;
            ProgramCylinder3Coupling.Visibility = guidewire ? Visibility.Visible : Visibility.Collapsed;
            ProgramCylinder2OpenLabel.Visibility = guidewire ? Visibility.Collapsed : Visibility.Visible;
            ProgramCylinder2OpenValue.Visibility = guidewire ? Visibility.Collapsed : Visibility.Visible;
            ProgramCylinder2CloseLabel.Visibility = guidewire ? Visibility.Collapsed : Visibility.Visible;
            ProgramCylinder2CloseValue.Visibility = guidewire ? Visibility.Collapsed : Visibility.Visible;
            ProgramCylinder4OpenLabel.Visibility = guidewire ? Visibility.Visible : Visibility.Collapsed;
            ProgramCylinder4OpenValue.Visibility = guidewire ? Visibility.Visible : Visibility.Collapsed;
            ProgramCylinder4CloseLabel.Visibility = guidewire ? Visibility.Visible : Visibility.Collapsed;
            ProgramCylinder4CloseValue.Visibility = guidewire ? Visibility.Visible : Visibility.Collapsed;
			ForceTitle.Text = guidewire ? "导丝侧轴向力 fn2 (N)" : legacy ? "实时轴向力 (N)" : "导管侧轴向力 fn1 (N)";
			TorqueTitle.Text = guidewire ? "导丝侧力 ft2 (N)" : legacy ? "实时 ft 力 (N)" : "导管侧力 ft1 (N)";
			Force1LegendText.Text = guidewire ? "fn2 (N)" : "fn1 (N)";
			Torque1LegendText.Text = guidewire ? "ft2 (N)" : "ft1 (N)";
            ForceValueText.Text = "未取零";
            TorqueValueText.Text = "未取零";
            Force2Line.Visibility = legacy ? Visibility.Visible : Visibility.Collapsed;
            Torque2Line.Visibility = legacy ? Visibility.Visible : Visibility.Collapsed;
            Force2Legend.Visibility = legacy ? Visibility.Visible : Visibility.Collapsed;
            Torque2Legend.Visibility = legacy ? Visibility.Visible : Visibility.Collapsed;
            _force1.Clear(); _force2.Clear(); _torque1.Clear(); _torque2.Clear();
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
                string positionFields = mode == "guidewire"
                    ? "axis5_from_left=" + Number(ProgramAxis5Pos) + "|axis6_prepare_from_left=" + Number(ProgramAxis6PreparePos) + "|axis6_trigger_from_left=" + Number(ProgramAxis6TriggerPos)
                    : "axis1_prepare_from_left=" + Number(ProgramAxis1PreparePos) + "|axis1_trigger_from_left=" + Number(ProgramAxis1TriggerPos);
                string commandText = string.Format(CultureInfo.InvariantCulture,
                    "PROGRAM_PREPARE|mode={0}|{1}|{2}={3}|cycle_count={4}|final_forward_distance={5}|cylinder1_coupling={6}|cylinder3_coupling={7}|cylinder2_open={8}|cylinder2_close={9}|cylinder4_open={10}|cylinder4_close={11}|release_wait_ms={12}|reclamp_wait_ms={13}|forward_velocity={14}|forward_acceleration={15}|forward_deceleration={16}|forward_jerk={17}|return_velocity={18}|return_acceleration={19}|return_deceleration={20}|return_jerk={21}|record_name={22}",
                    mode, positionFields, angleKey, Number(ProgramAngle), Int(ProgramCycleCount), Number(ProgramFinalDistance),
                    ProgramCylinder1Coupling.IsChecked == true ? 1 : 0, ProgramCylinder3Coupling.IsChecked == true ? 1 : 0,
                    Word(ProgramCylinder2OpenValue), Word(ProgramCylinder2CloseValue), Word(ProgramCylinder4OpenValue), Word(ProgramCylinder4CloseValue),
                    Int(ProgramReleaseWait), Int(ProgramReclampWait), Number(ProgramForwardVelocity), Number(ProgramForwardAcceleration),
                    Number(ProgramForwardDeceleration), Number(ProgramForwardJerk), Number(ProgramReturnVelocity), Number(ProgramReturnAcceleration),
                    Number(ProgramReturnDeceleration), Number(ProgramReturnJerk), RecordSuffix());
                await SendAsync(commandText);
            }
            catch (Exception ex) { ErrorText.Text = "准备定位参数无效：" + ex.Message; }
        }

        private async Task PollAsync()
        {
            if (_isPolling || _pipe == null || !_pipe.IsConnected) return;
            _isPolling = true;
            try
            {
                await SendAsync(CurrentMode == "legacy" ? "GET" : "GET_PROGRAM");
                if (CurrentMode == "legacy") await SendAsync("GET_STANDALONE_RECORD");
            }
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

        private async void ManualCylinderOpen_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button button && int.TryParse(button.Tag?.ToString(), out int cylinder))
                await SendManualCylinderConfigAndAction(cylinder, true);
        }

        private async void ManualCylinderClose_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button button && int.TryParse(button.Tag?.ToString(), out int cylinder))
                await SendManualCylinderConfigAndAction(cylinder, false);
        }

        private async Task SendManualCylinderConfigAndAction(int cylinder, bool open)
        {
            try
            {
                CheckBox enabled = cylinder == 1 ? Cylinder1Enabled : cylinder == 2 ? Cylinder2Enabled : cylinder == 3 ? Cylinder3Enabled : Cylinder4Enabled;
                TextBox openBox = cylinder == 1 ? Cylinder1OpenValue : cylinder == 2 ? Cylinder2OpenValue : cylinder == 3 ? Cylinder3OpenValue : Cylinder4OpenValue;
                TextBox closeBox = cylinder == 1 ? Cylinder1CloseValue : cylinder == 2 ? Cylinder2CloseValue : cylinder == 3 ? Cylinder3CloseValue : Cylinder4CloseValue;
                uint openValue = uint.Parse(openBox.Text, CultureInfo.InvariantCulture);
                uint closeValue = uint.Parse(closeBox.Text, CultureInfo.InvariantCulture);
                if (openValue > 65535 || closeValue > 65535) throw new ArgumentOutOfRangeException();
                string config = string.Format(CultureInfo.InvariantCulture,
                    "MANUAL_CYLINDER_CONFIG|cylinder={0}|enabled={1}|open={2}|close={3}", cylinder, enabled.IsChecked == true ? 1 : 0, openValue, closeValue);
                string result = await SendAsync(config);
                if (result.StartsWith("OK|", StringComparison.Ordinal))
                    await SendAsync(string.Format(CultureInfo.InvariantCulture, "MANUAL_CYLINDER_{0}|cylinder={1}", open ? "OPEN" : "CLOSE", cylinder));
            }
            catch (Exception ex) { ErrorText.Text = "电缸参数无效：" + ex.Message; }
        }

        private async void StandaloneRecordStart_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                ulong fields = BuildStandaloneFieldMask();
                if (fields == 0) { ErrorText.Text = "请至少选择一个独立记录字段"; return; }
                string command = string.Format(CultureInfo.InvariantCulture,
                    "STANDALONE_RECORD_START|record_name={0}|fields=0x{1:X}", RecordSuffix(), fields);
                await SendAsync(command);
            }
            catch (Exception ex) { ErrorText.Text = "独立记录参数无效：" + ex.Message; }
        }

        private async void StandaloneRecordStop_Click(object sender, RoutedEventArgs e) => await SendAsync("STANDALONE_RECORD_STOP");

        private async void StandaloneZero_Click(object sender, RoutedEventArgs e) => await SendAsync("STANDALONE_ZERO_FORCE");

        private void StandaloneSelectAll_Click(object sender, RoutedEventArgs e)
        {
            foreach (CheckBox box in StandaloneFieldBoxes()) box.IsChecked = true;
        }

        private void StandaloneClearAll_Click(object sender, RoutedEventArgs e)
        {
            foreach (CheckBox box in StandaloneFieldBoxes()) box.IsChecked = false;
        }

        private async void Abort_Click(object sender, RoutedEventArgs e) => await SendAsync(CurrentMode == "legacy" ? "ABORT" : "PROGRAM_ABORT");

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
            else if (response.StartsWith("STANDALONE_STATE|", StringComparison.Ordinal)) ParseStandaloneState(response);
            else if (response.StartsWith("OK|CONNECT_ADS", StringComparison.Ordinal)) SetAdsStatus(true, "ADS: 正常 (Port 851)");
            else if (response.StartsWith("ERROR|", StringComparison.Ordinal)) ErrorText.Text = response.Substring(6);
        }

        private void ParseLegacyState(string response)
        {
            string[] p = response.Split('|');
            if (p.Length < 27) return;
            double a1 = D(p[2]), a6 = D(p[3]), v1 = D(p[4]), v6 = D(p[5]), acc1 = D(p[6]), acc6 = D(p[7]);
            LiveMotionText.Text = string.Format(CultureInfo.InvariantCulture, "轴1：{0:F3} mm / {1:F3} mm/s / {2:F3} mm/s²\n轴6：{3:F3} mm / {4:F3} mm/s / {5:F3} mm/s²\n轴2/轴7角度：{6:F3}° / {7:F3}°", a1, v1, acc1, a6, v6, acc6, D(p[8]), D(p[9]));
            bool ads = p[14] == "1"; _selfcheckDone = p[15] == "1"; _selfcheckBusy = p[16] == "1"; _leftLimitValid = p[17] == "1"; _setupBusy = p[20] == "1"; _setupDone = p[21] == "1";
            PhaseText.Text = _selfcheckBusy ? "SelfCheck (正在执行自检)" : LegacyPhase(int.Parse(p[1], CultureInfo.InvariantCulture));
            SelfCheckText.Text = _selfcheckBusy ? "PLC自检: 执行中" : _selfcheckDone ? "PLC自检: 已完成" : "PLC自检: 未完成";
            CycleText.Text = "旧模式"; SetAdsStatus(ads, ads ? "ADS: 正常 (Port 851)" : "ADS: 未连接");
            int legacyZeroBusy = 27, legacyZeroDone = 28, legacyError = 26;
            ZeroStatusText.Text = p.Length > legacyZeroDone && p[legacyZeroBusy] == "1" ? "力感零点：采集中" : p.Length > legacyZeroDone && p[legacyZeroDone] == "1" ? "力感零点：已完成" : "力感零点：未完成";
            string legacyDirectory = p.Length > 36 ? p[36] : string.Empty;
            bool legacyArchived = p.Length > 37 && p[37] == "1";
            RecordStatusText.Text = legacyArchived ? "实时记录：已归档（" + p[34] + "点）" : p.Length > 34 && p[33] == "1" ? "实时记录：进行中（" + p[34] + "点）" : p.Length > 34 && p[34] != "0" ? "实时记录：自动归档中（" + p[34] + "点）" : "实时记录：未开始";
            if (!string.IsNullOrWhiteSpace(legacyDirectory)) RecordStatusText.Text += "\n目录：" + legacyDirectory;
            ZeroValuesText.Text = p.Length > 32 ? string.Format(CultureInfo.InvariantCulture, "零点值（原始计数 count）\nfn1：{0:F3}  ft1：{1:F3}\nfn2：{2:F3}  ft2：{3:F3}", D(p[29]), D(p[30]), D(p[31]), D(p[32])) : "";
            ErrorText.Text = p[legacyError];
            bool forceValid = p.Length > 38 && p[38] == "1";
            if (forceValid)
            {
                double fn1 = D(p[39]), ft1 = D(p[40]), fn2 = D(p[41]), ft2 = D(p[42]);
                Add(_force1, fn1); Add(_force2, fn2); Add(_torque1, ft1); Add(_torque2, ft2);
                ForceValueText.Text = string.Format(CultureInfo.InvariantCulture, "fn1: {0:F3} N   fn2: {1:F3} N", fn1, fn2);
				TorqueValueText.Text = string.Format(CultureInfo.InvariantCulture, "ft1: {0:F6} N   ft2: {1:F6} N", ft1, ft2);
            }
            else
            {
                _force1.Clear(); _force2.Clear(); _torque1.Clear(); _torque2.Clear();
                ForceValueText.Text = "未取零";
                TorqueValueText.Text = "未取零";
            }
            PrepareButton.IsEnabled = ads && _selfcheckDone && _leftLimitValid && !_selfcheckBusy && !_setupBusy; StartButton.IsEnabled = ads && _setupDone && p.Length > 28 && p[28] == "1" && !_setupBusy; ZeroButton.IsEnabled = ads && _selfcheckDone && _setupDone && !_setupBusy && !_selfcheckBusy;
            Draw(ForceCanvas, Force1Line, _force1, Force2Line, _force2); Draw(TorqueCanvas, Torque1Line, _torque1, Torque2Line, _torque2);
        }

        private void ParseProgramState(string response)
        {
            string[] p = response.Split('|');
            if (p.Length < 42) return;
            int phase = int.Parse(p[2], CultureInfo.InvariantCulture); _setupBusy = p[5] == "1"; _setupDone = p[6] == "1"; _selfcheckDone = p[7] == "1";
            bool ads = p[40] == "1";
            _selfcheckBusy = p.Length > 64 && p[64] == "1";
            SelfCheckText.Text = !ads
                ? "PLC自检: 状态未知"
                : _selfcheckBusy
                    ? "PLC自检: 执行中"
                    : _selfcheckDone
                        ? "PLC自检: 已完成"
                        : "PLC自检: 未完成";
            bool guidewire = CurrentMode == "guidewire";
            if (guidewire)
            {
                LiveMotionText.Text = string.Format(CultureInfo.InvariantCulture, "轴5：{0:F3} mm / {1:F3} mm/s / {2:F3} mm/s²\n轴6：{3:F3} mm / {4:F3} mm/s / {5:F3} mm/s²\n轴7角度：{6:F3}°", D(p[14]), D(p[15]), D(p[16]), D(p[17]), D(p[18]), D(p[19]), D(p[20]));
            }
            else
            {
                LiveMotionText.Text = string.Format(CultureInfo.InvariantCulture, "轴1：{0:F3} mm / {1:F3} mm/s / {2:F3} mm/s²\n轴2角度：{3:F3}°", D(p[8]), D(p[9]), D(p[10]), D(p[11]));
            }
            CycleText.Text = string.Format(CultureInfo.InvariantCulture, "往复夹持次数：{0} / {1}", p[3], p[4]);
            int programZeroBusy = 42, programZeroDone = 43, programError = 41;
            ZeroStatusText.Text = p.Length > programZeroDone && p[programZeroBusy] == "1" ? "力感零点：采集中" : p.Length > programZeroDone && p[programZeroDone] == "1" ? "力感零点：已完成" : "力感零点：未完成";
            string programDirectory = p.Length > 51 ? p[51] : string.Empty;
            bool programArchived = p.Length > 52 && p[52] == "1";
            RecordStatusText.Text = programArchived ? "实时记录：已归档（" + p[49] + "点）" : p.Length > 49 && p[48] == "1" ? "实时记录：进行中（" + p[49] + "点）" : p.Length > 49 && p[49] != "0" ? "实时记录：自动归档中（" + p[49] + "点）" : "实时记录：未开始";
            if (!string.IsNullOrWhiteSpace(programDirectory)) RecordStatusText.Text += "\n目录：" + programDirectory;
            ZeroValuesText.Text = p.Length > 47 ? string.Format(CultureInfo.InvariantCulture, "零点值（原始计数 count）\nfn1：{0:F3}  ft1：{1:F3}\nfn2：{2:F3}  ft2：{3:F3}", D(p[44]), D(p[45]), D(p[46]), D(p[47])) : "";
            int waitAction = p.Length > 53 ? int.Parse(p[53], CultureInfo.InvariantCulture) : 0;
            PhaseText.Text = waitAction == 1 ? "等待电缸释放" : waitAction == 2 ? "等待重新夹紧" : ProgramPhase(phase);
            ErrorText.Text = p[programError];
            if (p.Length > 58 && int.Parse(p[39], CultureInfo.InvariantCulture) != 0 && p[54] != "0")
            {
                string source = p[54] == "1" ? "准备定位" : p[54] == "2" ? "前向至触发位置" : p[54] == "3" ? "回退" : p[54] == "4" ? "最终前向" : "未知动作";
                ErrorText.Text = p[55] == "1" || p[55] == "5" || p[55] == "6"
                    ? string.Format(CultureInfo.InvariantCulture, "PLC运动错误：ID {0}；轴{1}；{2}；目标距左限位 {3:F3} mm", p[39], p[55], source, D(p[58]))
                    : string.Format(CultureInfo.InvariantCulture, "PLC运动错误：ID {0}；轴{1}；{2}；目标值 {3:F3}", p[39], p[55], source, D(p[57]));
            }
            bool forceValid = p.Length > 63 && p[59] == "1";
            if (forceValid)
            {
                double fn1 = D(p[60]), ft1 = D(p[61]), fn2 = D(p[62]), ft2 = D(p[63]);
                if (guidewire)
                {
                    Add(_force1, fn2); Add(_torque1, ft2);
                    ForceValueText.Text = string.Format(CultureInfo.InvariantCulture, "fn2: {0:F3} N", fn2);
					TorqueValueText.Text = string.Format(CultureInfo.InvariantCulture, "ft2: {0:F6} N", ft2);
                }
                else
                {
                    Add(_force1, fn1); Add(_torque1, ft1);
                    ForceValueText.Text = string.Format(CultureInfo.InvariantCulture, "fn1: {0:F3} N", fn1);
					TorqueValueText.Text = string.Format(CultureInfo.InvariantCulture, "ft1: {0:F6} N", ft1);
                }
            }
            else
            {
                _force1.Clear(); _force2.Clear(); _torque1.Clear(); _torque2.Clear();
                ForceValueText.Text = "未取零";
                TorqueValueText.Text = "未取零";
            }
            SetAdsStatus(ads, ads ? "ADS: 正常 (Port 851)" : "ADS: 未连接");
            PrepareButton.IsEnabled = ads && _selfcheckDone && !_selfcheckBusy && !_setupBusy; StartButton.IsEnabled = ads && _setupDone && phase == 2 && p.Length > programZeroDone && p[programZeroDone] == "1" && !_selfcheckBusy && !_setupBusy; ZeroButton.IsEnabled = ads && _selfcheckDone && _setupDone && !_selfcheckBusy && !_setupBusy && phase == 2;
            bool programCouplingEditable = ads && !_setupBusy && (phase == 0 || phase >= 10);
            ProgramCylinder1Coupling.IsEnabled = programCouplingEditable;
            ProgramCylinder3Coupling.IsEnabled = programCouplingEditable;
            ProgramCylinder2OpenValue.IsEnabled = programCouplingEditable;
            ProgramCylinder2CloseValue.IsEnabled = programCouplingEditable;
            ProgramCylinder4OpenValue.IsEnabled = programCouplingEditable;
            ProgramCylinder4CloseValue.IsEnabled = programCouplingEditable;
            Draw(ForceCanvas, Force1Line, _force1, Force2Line, _force2); Draw(TorqueCanvas, Torque1Line, _torque1, Torque2Line, _torque2);
        }

        private void ParseStandaloneState(string response)
        {
            string[] p = response.Split('|');
            if (p.Length < 23) return;
            bool ads = p[1] == "1";
            _standaloneSelfcheckDone = p[2] == "1";
            int legacyPhase = int.Parse(p[3], CultureInfo.InvariantCulture);
            _standaloneLegacyBusy = legacyPhase >= 3 && legacyPhase <= 9 || legacyPhase == 14;
            _standaloneRecording = p[9] == "1";
            SelfCheckText.Text = _standaloneSelfcheckDone ? "PLC自检: 已完成" : "PLC自检: 执行中或未完成";
            Cylinder1Current.Text = p[4]; Cylinder2Current.Text = p[5]; Cylinder3Current.Text = p[6]; Cylinder4Current.Text = p[7];
            ManualCylinderHint.Text = _standaloneSelfcheckDone
                ? (_standaloneLegacyBusy ? "实验运动或夹爪切换中，电缸手动按钮暂时禁用。" : "电缸手动控制可用；独立记录可单独开始。")
                : "记录可直接开始，电缸控制等待PLC自检完成。";
            bool cylinderEnabled = ads && _standaloneSelfcheckDone && !_standaloneLegacyBusy;
            foreach (Button button in FindVisualChildren<Button>(LegacyPanel))
            {
                if (button.Tag is string tag && (button.Content?.ToString() == "开" || button.Content?.ToString() == "闭")) button.IsEnabled = cylinderEnabled;
            }
            bool standaloneStopping = p[22] == "1";
            StandaloneRecordStartButton.IsEnabled = ads && !_standaloneRecording && !standaloneStopping && !_standaloneLegacyBusy;
            StandaloneRecordStopButton.IsEnabled = ads && _standaloneRecording;
            StandaloneZeroButton.IsEnabled = ads && !_standaloneRecording && !standaloneStopping && !_standaloneLegacyBusy;
            if (_standaloneRecording || standaloneStopping || !string.IsNullOrWhiteSpace(p[12]))
            {
                string statusText = _standaloneRecording ? "独立记录：进行中（" + p[10] + "点）" : standaloneStopping ? "独立记录：停止收尾中（" + p[10] + "点）" : "独立记录：已归档（" + p[10] + "点）";
                RecordStatusText.Text = statusText +
                    (string.IsNullOrWhiteSpace(p[12]) ? string.Empty : "\n目录：" + p[12]);
            }
            ZeroStatusText.Text = p[14] == "1" ? "力感零点：采集中" : p[15] == "1" ? "力感零点：已完成" : "力感零点：未完成";
            if (!string.IsNullOrWhiteSpace(p[21])) ErrorText.Text = p[21];
        }

        private ulong BuildStandaloneFieldMask()
        {
            ulong mask = 0;
            AddField(ref mask, FieldAxis1Pos, 0); AddField(ref mask, FieldAxis1Velocity, 1); AddField(ref mask, FieldAxis1Acceleration, 2);
            AddField(ref mask, FieldAxis2Pos, 3); AddField(ref mask, FieldAxis2Velocity, 4); AddField(ref mask, FieldAxis2Acceleration, 5);
            AddField(ref mask, FieldAxis5Pos, 6); AddField(ref mask, FieldAxis5Velocity, 7); AddField(ref mask, FieldAxis5Acceleration, 8);
            AddField(ref mask, FieldAxis6Pos, 9); AddField(ref mask, FieldAxis6Velocity, 10); AddField(ref mask, FieldAxis6Acceleration, 11);
            AddField(ref mask, FieldAxis7Pos, 12); AddField(ref mask, FieldAxis7Velocity, 13); AddField(ref mask, FieldAxis7Acceleration, 14);
            AddField(ref mask, FieldCylinder1, 15); AddField(ref mask, FieldCylinder2, 16); AddField(ref mask, FieldCylinder3, 17); AddField(ref mask, FieldCylinder4, 18);
            AddField(ref mask, FieldFn1Raw, 19); AddField(ref mask, FieldFt1Raw, 20); AddField(ref mask, FieldFn2Raw, 21); AddField(ref mask, FieldFt2Raw, 22);
            AddField(ref mask, FieldFn1Zeroed, 23); AddField(ref mask, FieldFt1Zeroed, 24); AddField(ref mask, FieldFn2Zeroed, 25); AddField(ref mask, FieldFt2Zeroed, 26);
            AddField(ref mask, FieldFn1Sensor, 27); AddField(ref mask, FieldFt1Sensor, 28);
            AddField(ref mask, FieldFn1CalDelta, 29); AddField(ref mask, FieldFn1CalAbs, 30); AddField(ref mask, FieldFt1CalDelta, 31); AddField(ref mask, FieldFt1CalAbs, 32);
            AddField(ref mask, FieldTorque1CalDelta, 33); AddField(ref mask, FieldTorque1CalAbs, 34);
            AddField(ref mask, FieldFn1DecoupledDelta, 35); AddField(ref mask, FieldTorque1DecoupledDelta, 36);
            AddField(ref mask, FieldFn1DecoupledAbs, 37); AddField(ref mask, FieldTorque1DecoupledAbs, 38); AddField(ref mask, FieldAxis2Angle, 39);
            AddField(ref mask, FieldFn2Sensor, 40); AddField(ref mask, FieldFt2Sensor, 41);
            AddField(ref mask, FieldFn2CalDelta, 42); AddField(ref mask, FieldFn2CalAbs, 43); AddField(ref mask, FieldFt2CalDelta, 44); AddField(ref mask, FieldFt2CalAbs, 45);
            AddField(ref mask, FieldTorque2CalDelta, 46); AddField(ref mask, FieldTorque2CalAbs, 47);
            AddField(ref mask, FieldFn2DecoupledDelta, 48); AddField(ref mask, FieldTorque2DecoupledDelta, 49);
            AddField(ref mask, FieldFn2DecoupledAbs, 50); AddField(ref mask, FieldTorque2DecoupledAbs, 51); AddField(ref mask, FieldAxis7Angle, 52);
            return mask;
        }

        private static void AddField(ref ulong mask, CheckBox box, int bit)
        {
            if (box.IsChecked == true) mask |= 1UL << bit;
        }

        private IEnumerable<CheckBox> StandaloneFieldBoxes()
        {
            return new[] { FieldAxis1Pos, FieldAxis1Velocity, FieldAxis1Acceleration, FieldAxis2Pos, FieldAxis2Velocity, FieldAxis2Acceleration,
                FieldAxis5Pos, FieldAxis5Velocity, FieldAxis5Acceleration, FieldAxis6Pos, FieldAxis6Velocity, FieldAxis6Acceleration,
                FieldAxis7Pos, FieldAxis7Velocity, FieldAxis7Acceleration, FieldCylinder1, FieldCylinder2, FieldCylinder3, FieldCylinder4,
                FieldFn1Raw, FieldFt1Raw, FieldFn2Raw, FieldFt2Raw, FieldFn1Zeroed, FieldFt1Zeroed, FieldFn2Zeroed, FieldFt2Zeroed,
                FieldFn1Sensor, FieldFt1Sensor, FieldFn1CalDelta, FieldFn1CalAbs, FieldFt1CalDelta, FieldFt1CalAbs,
                FieldTorque1CalDelta, FieldTorque1CalAbs, FieldFn1DecoupledDelta, FieldTorque1DecoupledDelta, FieldFn1DecoupledAbs, FieldTorque1DecoupledAbs, FieldAxis2Angle,
                FieldFn2Sensor, FieldFt2Sensor, FieldFn2CalDelta, FieldFn2CalAbs, FieldFt2CalDelta, FieldFt2CalAbs,
                FieldTorque2CalDelta, FieldTorque2CalAbs, FieldFn2DecoupledDelta, FieldTorque2DecoupledDelta, FieldFn2DecoupledAbs, FieldTorque2DecoupledAbs, FieldAxis7Angle };
        }

        private static IEnumerable<T> FindVisualChildren<T>(DependencyObject dependencyObject) where T : DependencyObject
        {
            if (dependencyObject == null) yield break;
            for (int i = 0; i < VisualTreeHelper.GetChildrenCount(dependencyObject); i++)
            {
                DependencyObject child = VisualTreeHelper.GetChild(dependencyObject, i);
                if (child is T typedChild) yield return typedChild;
                foreach (T descendant in FindVisualChildren<T>(child)) yield return descendant;
            }
        }

        private static double D(string value) => double.Parse(value, CultureInfo.InvariantCulture);
        private static string Number(TextBox box) => double.Parse(box.Text, CultureInfo.InvariantCulture).ToString(CultureInfo.InvariantCulture);
        private static string Int(TextBox box) => uint.Parse(box.Text, CultureInfo.InvariantCulture).ToString(CultureInfo.InvariantCulture);
        private static string Word(TextBox box)
        {
            uint value = uint.Parse(box.Text, CultureInfo.InvariantCulture);
            if (value > 65535) throw new ArgumentOutOfRangeException(box.Name, "电缸开闭值必须在0至65535之间");
            return value.ToString(CultureInfo.InvariantCulture);
        }
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
