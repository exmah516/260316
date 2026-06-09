using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows.Threading;

namespace AdsControlUI
{
    public class AdsControlViewModel : INotifyPropertyChanged, IDisposable
    {
        private const double Axis1StrokeMm = 99.0;
        private const double Axis3StrokeMm = 666.0;
        private const double Axis5StrokeMm = 688.0;
        private const double Axis6StrokeMm = 688.0;

        private readonly VisPipeClient _client = new VisPipeClient();
        private readonly DispatcherTimer _refreshTimer;
        private VisState _state;

        public AdsControlViewModel()
        {
            _client.Start();
            _refreshTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(33) };
            _refreshTimer.Tick += (s, e) => RefreshFromPipe();
            _refreshTimer.Start();
        }

        private void RefreshFromPipe()
        {
            if (!_client.TryGetLatestState(out var state)) return;
            _state = state;
            OnPropertyChanged(nameof(Axis1Pos));
            OnPropertyChanged(nameof(Axis2Pos));
            OnPropertyChanged(nameof(Axis3Pos));
            OnPropertyChanged(nameof(Axis4Pos));
            OnPropertyChanged(nameof(Axis5Pos));
            OnPropertyChanged(nameof(Axis6Pos));
            OnPropertyChanged(nameof(Axis7Pos));
            OnPropertyChanged(nameof(Axis1FromLeft));
            OnPropertyChanged(nameof(Axis3FromLeft));
            OnPropertyChanged(nameof(Axis5FromLeft));
            OnPropertyChanged(nameof(Axis6FromLeft));
            OnPropertyChanged(nameof(Axis1EffectiveTravel));
            OnPropertyChanged(nameof(Axis3EffectiveTravel));
            OnPropertyChanged(nameof(Axis5EffectiveTravel));
            OnPropertyChanged(nameof(Axis6EffectiveTravel));
            OnPropertyChanged(nameof(Axis1StrokeText));
            OnPropertyChanged(nameof(Axis2RotationText));
            OnPropertyChanged(nameof(Axis3StrokeText));
            OnPropertyChanged(nameof(Axis4RotationText));
            OnPropertyChanged(nameof(Axis5StrokeText));
            OnPropertyChanged(nameof(Axis6StrokeText));
            OnPropertyChanged(nameof(Axis7RotationText));
            OnPropertyChanged(nameof(Cyl1Open));
            OnPropertyChanged(nameof(Cyl2Open));
            OnPropertyChanged(nameof(Cyl3Open));
            OnPropertyChanged(nameof(Cyl4Open));
            OnPropertyChanged(nameof(ModeText));
            OnPropertyChanged(nameof(ControlActive));
            OnPropertyChanged(nameof(FreezeActive));
            OnPropertyChanged(nameof(EstopHold));
            OnPropertyChanged(nameof(FfEnabled));
            OnPropertyChanged(nameof(CalZeroed));
            OnPropertyChanged(nameof(Force582F));
            OnPropertyChanged(nameof(Force582N));
            OnPropertyChanged(nameof(IsConnected));
            OnPropertyChanged(nameof(Axis1Reverse));
            OnPropertyChanged(nameof(Axis6Reverse));
            OnPropertyChanged(nameof(ForceLogRunning));
            OnPropertyChanged(nameof(StartupWaiting));
            OnPropertyChanged(nameof(StartupCompleted));
            OnPropertyChanged(nameof(PhaseText));
        }

        public bool IsConnected => _client.IsConnected;
        public double Axis1Pos => GetAxisPos(0);
        public double Axis2Pos => GetAxisPos(1);
        public double Axis3Pos => GetAxisPos(2);
        public double Axis4Pos => GetAxisPos(3);
        public double Axis5Pos => GetAxisPos(4);
        public double Axis6Pos => GetAxisPos(5);
        public double Axis7Pos => GetAxisPos(6);

        public double Axis1StrokeMax => Axis1StrokeMm;
        public double Axis3StrokeMax => Axis3StrokeMm;
        public double Axis5StrokeMax => Axis5StrokeMm;
        public double Axis6StrokeMax => Axis6StrokeMm;

        public double Axis1FromLeft => GetAxisFromLeft(0);
        public double Axis3FromLeft => GetAxisFromLeft(2);
        public double Axis5FromLeft => GetAxisFromLeft(4);
        public double Axis6FromLeft => GetAxisFromLeft(5);

        public double Axis1EffectiveTravel => GetEffectiveTravel(Axis1FromLeft, Axis1StrokeMm);
        public double Axis3EffectiveTravel => GetEffectiveTravel(Axis3FromLeft, Axis3StrokeMm);
        public double Axis5EffectiveTravel => GetEffectiveTravel(Axis5FromLeft, Axis5StrokeMm);
        public double Axis6EffectiveTravel => GetEffectiveTravel(Axis6FromLeft, Axis6StrokeMm);

        public string Axis1StrokeText => FormatLinearAxis("轴1", "导管推送", Axis1EffectiveTravel, Axis1FromLeft, Axis1StrokeMm);
        public string Axis2RotationText => FormatRotationAxis("轴2", "导管旋转", Axis2Pos);
        public string Axis3StrokeText => FormatLinearAxis("轴3", "递送", Axis3EffectiveTravel, Axis3FromLeft, Axis3StrokeMm);
        public string Axis4RotationText => FormatRotationAxis("轴4", "点动旋转", Axis4Pos);
        public string Axis5StrokeText => FormatLinearAxis("轴5", "镜像", Axis5EffectiveTravel, Axis5FromLeft, Axis5StrokeMm);
        public string Axis6StrokeText => FormatLinearAxis("轴6", "导丝推送", Axis6EffectiveTravel, Axis6FromLeft, Axis6StrokeMm);
        public string Axis7RotationText => FormatRotationAxis("轴7", "导丝旋转", Axis7Pos);

        public bool Cyl1Open => (_state.cylinder_cmd?[0] ?? 0) > 200;
        public bool Cyl2Open => (_state.cylinder_cmd?[1] ?? 0) < 200;
        public bool Cyl3Open => (_state.cylinder_cmd?[2] ?? 0) > 200;
        public bool Cyl4Open => (_state.cylinder_cmd?[3] ?? 0) < 200;

        public string ModeText
        {
            get
            {
                string mode = _state.guidewire_mode == 0 ? "导管" : "导丝";
                bool rev = _state.guidewire_mode == 0 ? _state.axis1_reverse : _state.axis6_reverse;
                return mode + (rev ? "撤出" : "递送");
            }
        }

        public bool Axis1Reverse => _state.axis1_reverse;
        public bool Axis6Reverse => _state.axis6_reverse;
        public bool ForceLogRunning => _state.force_log_running;
        public bool StartupWaiting => _state.startup_waiting;
        public bool StartupCompleted => _state.startup_completed;

        public string PhaseText
        {
            get
            {
                if (_state.startup_completed) return "已就绪";
                if (_state.startup_waiting) return "等待启动选择...";
                return "启动准备中...";
            }
        }

        public bool ControlActive => _state.control_active;
        public bool FreezeActive => _state.freeze_active;
        public bool EstopHold => _state.estop_hold;
        public bool FfEnabled => _state.ff_enabled;
        public bool CalZeroed => _state.cal_zeroed;
        public double Force582F => _state.force_582_f;
        public double Force582N => _state.force_582_n;

        private double GetAxisPos(int index)
        {
            if (_state.axis_pos_from_left != null && _state.axis_pos_from_left.Length > index)
            {
                return _state.axis_pos_from_left[index];
            }

            return (_state.axis_pos != null && _state.axis_pos.Length > index) ? _state.axis_pos[index] : 0.0;
        }

        private double GetAxisFromLeft(int index)
        {
            return (_state.axis_pos_from_left != null && _state.axis_pos_from_left.Length > index)
                ? _state.axis_pos_from_left[index]
                : 0.0;
        }

        private static double GetEffectiveTravel(double fromLeftMm, double fullStrokeMm)
        {
            return Clamp(fullStrokeMm - fromLeftMm, 0.0, fullStrokeMm);
        }

        private static double Clamp(double value, double min, double max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        private static string FormatLinearAxis(string axisName, string description, double effectiveTravel, double fromLeft, double fullStroke)
        {
            return $"{axisName} ({description}): 有效 {effectiveTravel:F2}/{fullStroke:F0} mm，距左限位 {fromLeft:F2} mm";
        }

        private static string FormatRotationAxis(string axisName, string description, double angle)
        {
            return $"{axisName} ({description}): {angle:F2}°";
        }

        public void SetCylinderOverride(int index) =>
            _client.SendCommand(VisCommandType.SetCylinderOverride, index);

        public void ClearCylinderOverride(int index) =>
            _client.SendCommand(VisCommandType.ClearCylinderOverride, index);

        public void SetMode(int guidewireMode, int reverse) =>
            _client.SendCommand(VisCommandType.SetReverseMode, guidewireMode, reverse);

        public void ZeroForceSensor() =>
            _client.SendCommand(VisCommandType.ZeroForceSensor);

        public void ToggleForceFeedback() =>
            _client.SendCommand(VisCommandType.ToggleForceFeedback);

        public void ToggleForceLog() =>
            _client.SendCommand(VisCommandType.ToggleForceLog);

        public void SelectDirectControl() =>
            _client.SendCommand(VisCommandType.SelectDirectControl);

        public void SendStartupParams(double a1, double a3, double a5, double a6, double a2deg, double a7deg, double speed)
        {
            _client.SendCommand(VisCommandType.SetStartupAxisPos, 1, (int)(a1 * 100));
            _client.SendCommand(VisCommandType.SetStartupAxisPos, 3, (int)(a3 * 100));
            _client.SendCommand(VisCommandType.SetStartupAxisPos, 5, (int)(a5 * 100));
            _client.SendCommand(VisCommandType.SetStartupAxisPos, 6, (int)(a6 * 100));
            _client.SendCommand(VisCommandType.SetStartupAxisDeg, 2, (int)(a2deg * 100));
            _client.SendCommand(VisCommandType.SetStartupAxisDeg, 7, (int)(a7deg * 100));
            _client.SendCommand(VisCommandType.SetStartupSpeed, (int)(speed * 100000));
            _client.SendCommand(VisCommandType.ExecuteStartup);
        }

        public event PropertyChangedEventHandler PropertyChanged;
        private void OnPropertyChanged([CallerMemberName] string name = null) =>
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));

        public void Dispose()
        {
            _refreshTimer.Stop();
            _client.Dispose();
        }
    }
}
