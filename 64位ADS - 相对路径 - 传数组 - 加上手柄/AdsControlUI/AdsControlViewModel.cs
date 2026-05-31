using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows.Threading;

namespace AdsControlUI
{
    public class AdsControlViewModel : INotifyPropertyChanged, IDisposable
    {
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
        public double Axis1Pos => _state.axis_pos?[0] ?? 0;
        public double Axis2Pos => _state.axis_pos?[1] ?? 0;
        public double Axis3Pos => _state.axis_pos?[2] ?? 0;
        public double Axis4Pos => _state.axis_pos?[3] ?? 0;
        public double Axis5Pos => _state.axis_pos?[4] ?? 0;
        public double Axis6Pos => _state.axis_pos?[5] ?? 0;
        public double Axis7Pos => _state.axis_pos?[6] ?? 0;

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
