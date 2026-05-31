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

        public string ModeText => _state.guidewire_mode switch
        {
            0 => "导管",
            1 => "导丝独立",
            2 => "协同",
            _ => "未知"
        };

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

        public void RequestModeSwitch(int mode) =>
            _client.SendCommand(VisCommandType.RequestModeSwitch, mode);

        public void ZeroForceSensor() =>
            _client.SendCommand(VisCommandType.ZeroForceSensor);

        public void ToggleForceFeedback() =>
            _client.SendCommand(VisCommandType.ToggleForceFeedback);

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
