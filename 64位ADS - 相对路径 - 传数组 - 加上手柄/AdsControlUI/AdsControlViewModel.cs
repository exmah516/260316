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

        public event Action<VisState> StateUpdated;

        public AdsControlViewModel()
        {
            _client.Start();
            _refreshTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(33) };
            _refreshTimer.Tick += (s, e) => RefreshFromPipe();
            _refreshTimer.Start();
        }

        // 位置类显示值变化阈值，避免 33ms 全量刷绑定。
        private const double PosEpsilon = 1e-4;
        private const double ForceEpsilon = 1e-6;

		private bool _hasUiSnapshot;
		private bool _prevConnected;
		private VisState _prevState;
		private string _physicalButtonNoticeText = "";

        private void RefreshFromPipe()
        {
            bool connected = _client.IsConnected;
            if (!_client.TryGetLatestState(out var state))
            {
				bool connectionChanged = _prevConnected != connected;
                NotifyIfChanged(ref _prevConnected, connected, nameof(IsConnected));
				if (connectionChanged)
					NotifyAdsProperties();
                return;
            }

			_state = state;
			if (!_hasUiSnapshot)
			{
				if (state.physical_button_event_counter != 0)
					_physicalButtonNoticeText = PhysicalButtonEventText(state.physical_button_event_code);
				NotifyAllUiProperties();
                _prevState = state;
                _prevConnected = connected;
                _hasUiSnapshot = true;
                StateUpdated?.Invoke(state);
                return;
            }

            var prev = _prevState;

            // 轴位置与文本
            if (AxisChanged(prev, state, 0))
            {
                OnPropertyChanged(nameof(Axis1Pos));
                OnPropertyChanged(nameof(Axis1FromLeft));
                OnPropertyChanged(nameof(Axis1EffectiveTravel));
                OnPropertyChanged(nameof(Axis1StrokeText));
                OnPropertyChanged(nameof(SpacingRecoveryStatusText));
            }
            if (AxisChanged(prev, state, 1))
            {
                OnPropertyChanged(nameof(Axis2Pos));
                OnPropertyChanged(nameof(Axis2RotationText));
            }
            if (AxisChanged(prev, state, 2))
            {
                OnPropertyChanged(nameof(Axis3Pos));
                OnPropertyChanged(nameof(Axis3FromLeft));
                OnPropertyChanged(nameof(Axis3EffectiveTravel));
                OnPropertyChanged(nameof(Axis3StrokeText));
                OnPropertyChanged(nameof(SpacingRecoveryStatusText));
            }
            if (AxisChanged(prev, state, 3))
            {
                OnPropertyChanged(nameof(Axis4Pos));
                OnPropertyChanged(nameof(Axis4RotationText));
            }
            if (AxisChanged(prev, state, 4))
            {
                OnPropertyChanged(nameof(Axis5Pos));
                OnPropertyChanged(nameof(Axis5FromLeft));
                OnPropertyChanged(nameof(Axis5EffectiveTravel));
                OnPropertyChanged(nameof(Axis5StrokeText));
            }
            if (AxisChanged(prev, state, 5))
            {
                OnPropertyChanged(nameof(Axis6Pos));
                OnPropertyChanged(nameof(Axis6FromLeft));
                OnPropertyChanged(nameof(Axis6EffectiveTravel));
                OnPropertyChanged(nameof(Axis6StrokeText));
            }
            if (AxisChanged(prev, state, 6))
            {
                OnPropertyChanged(nameof(Axis7Pos));
                OnPropertyChanged(nameof(Axis7RotationText));
            }

            // 电缸开合（与属性阈值一致）
            if (CylOpenChanged(prev, state, 0)) OnPropertyChanged(nameof(Cyl1Open));
            if (CylOpenChanged(prev, state, 1)) OnPropertyChanged(nameof(Cyl2Open));
            if (CylOpenChanged(prev, state, 2)) OnPropertyChanged(nameof(Cyl3Open));
            if (CylOpenChanged(prev, state, 3)) OnPropertyChanged(nameof(Cyl4Open));

            // 模式与方向
			if (prev.guidewire_mode != state.guidewire_mode ||
                prev.axis1_reverse != state.axis1_reverse ||
                prev.axis6_reverse != state.axis6_reverse ||
                prev.cooperative_direction != state.cooperative_direction)
            {
                OnPropertyChanged(nameof(ModeText));
                OnPropertyChanged(nameof(ModeCathFwdSelected));
                OnPropertyChanged(nameof(ModeCathRevSelected));
                OnPropertyChanged(nameof(ModeGuideFwdSelected));
                OnPropertyChanged(nameof(ModeGuideRevSelected));
                OnPropertyChanged(nameof(ModeCooperativeDeliverySelected));
                OnPropertyChanged(nameof(ModeCooperativeRetractionSelected));
                OnPropertyChanged(nameof(CooperativeStatusText));
                OnPropertyChanged(nameof(Axis1Reverse));
				OnPropertyChanged(nameof(Axis6Reverse));
			}
			if (prev.physical_button_event_counter != state.physical_button_event_counter)
			{
				_physicalButtonNoticeText = PhysicalButtonEventText(state.physical_button_event_code);
				OnPropertyChanged(nameof(PhysicalButtonNoticeText));
			}

            if (prev.control_active != state.control_active)
            {
                OnPropertyChanged(nameof(ControlActive));
                OnPropertyChanged(nameof(CooperativeModeEnabled));
            }
            if (prev.freeze_active != state.freeze_active)
            {
                OnPropertyChanged(nameof(FreezeActive));
                OnPropertyChanged(nameof(CooperativeModeEnabled));
            }
            if (prev.estop_hold != state.estop_hold)
            {
                OnPropertyChanged(nameof(EstopHold));
                OnPropertyChanged(nameof(CooperativeModeEnabled));
            }
            if (prev.ff_enabled != state.ff_enabled) OnPropertyChanged(nameof(FfEnabled));
            if (prev.cal_zeroed != state.cal_zeroed) OnPropertyChanged(nameof(CalZeroed));
            if (prev.gravity_comp_enabled != state.gravity_comp_enabled) OnPropertyChanged(nameof(GravityCompEnabled));
            if (prev.startup_waiting != state.startup_waiting) OnPropertyChanged(nameof(StartupWaiting));
            if (prev.startup_completed != state.startup_completed)
            {
                OnPropertyChanged(nameof(StartupCompleted));
                OnPropertyChanged(nameof(CooperativeModeEnabled));
            }
            if (prev.startup_waiting != state.startup_waiting || prev.startup_completed != state.startup_completed)
                OnPropertyChanged(nameof(PhaseText));

            if (prev.dual_handle_ready != state.dual_handle_ready)
            {
                OnPropertyChanged(nameof(DualHandleReady));
                OnPropertyChanged(nameof(CooperativeModeEnabled));
                OnPropertyChanged(nameof(CooperativeStatusText));
            }
            if (prev.cooperative_return_owner != state.cooperative_return_owner)
            {
                OnPropertyChanged(nameof(CooperativeReturnOwner));
                OnPropertyChanged(nameof(ModeSwitchAllowed));
                OnPropertyChanged(nameof(CooperativeModeEnabled));
                OnPropertyChanged(nameof(CooperativeStatusText));
            }
            if (prev.axis6_soft_limit_hold != state.axis6_soft_limit_hold)
            {
                OnPropertyChanged(nameof(Axis6SoftLimitHold));
                OnPropertyChanged(nameof(Axis6SoftLimitText));
                OnPropertyChanged(nameof(CooperativeModeEnabled));
            }
            if (prev.axis1_phase != state.axis1_phase || prev.axis6_phase != state.axis6_phase)
            {
                OnPropertyChanged(nameof(CooperativeModeEnabled));
            }

            if (Changed(prev.force_582_f, state.force_582_f, ForceEpsilon)) OnPropertyChanged(nameof(Force582F));
            if (Changed(prev.force_582_n, state.force_582_n, ForceEpsilon)) OnPropertyChanged(nameof(Force582N));
            if (Changed(prev.force_582_theory_f, state.force_582_theory_f, ForceEpsilon)) OnPropertyChanged(nameof(Force582TheoryF));
            if (Changed(prev.force_582_theory_n, state.force_582_theory_n, ForceEpsilon)) OnPropertyChanged(nameof(Force582TheoryN));

            if (prev.ft_exp_phase != state.ft_exp_phase)
            {
                OnPropertyChanged(nameof(FtExpPhase));
                OnPropertyChanged(nameof(FtExpPhaseText));
            }
            if (prev.ft_exp_velocity_level != state.ft_exp_velocity_level) OnPropertyChanged(nameof(FtExpVelocityLevel));
            if (prev.ft_exp_trial_id != state.ft_exp_trial_id) OnPropertyChanged(nameof(FtExpTrialId));
            if (prev.ft_exp_repeat_in_lvl != state.ft_exp_repeat_in_lvl) OnPropertyChanged(nameof(FtExpRepeatInLevel));
            if (Changed(prev.ft_exp_v_ratio_curr, state.ft_exp_v_ratio_curr, ForceEpsilon)) OnPropertyChanged(nameof(FtExpVRatioCurr));
            if (Changed(prev.ft_exp_axis1_target, state.ft_exp_axis1_target, PosEpsilon)) OnPropertyChanged(nameof(FtExpAxis1Target));
            if (prev.ft_exp_active != state.ft_exp_active)
            {
                OnPropertyChanged(nameof(FtExpActive));
                OnPropertyChanged(nameof(CooperativeModeEnabled));
            }
            if (prev.ft_exp_aborted != state.ft_exp_aborted) OnPropertyChanged(nameof(FtExpAborted));

            if (prev.spacing_recovery_phase != state.spacing_recovery_phase)
            {
                OnPropertyChanged(nameof(SpacingRecoveryActive));
                OnPropertyChanged(nameof(SpacingRecoveryInactive));
                OnPropertyChanged(nameof(SpacingRecoveryStatusText));
                OnPropertyChanged(nameof(ModeText));
                OnPropertyChanged(nameof(ModeCathFwdSelected));
                OnPropertyChanged(nameof(ModeCathRevSelected));
                OnPropertyChanged(nameof(ModeGuideFwdSelected));
                OnPropertyChanged(nameof(ModeGuideRevSelected));
                OnPropertyChanged(nameof(ModeCooperativeDeliverySelected));
			OnPropertyChanged(nameof(ModeCooperativeRetractionSelected));
			OnPropertyChanged(nameof(PhysicalButtonNoticeText));
			OnPropertyChanged(nameof(ModeSwitchAllowed));
                OnPropertyChanged(nameof(CooperativeModeEnabled));
                OnPropertyChanged(nameof(CooperativeStatusText));
                OnPropertyChanged(nameof(PhaseText));
            }
            if (Changed(prev.spacing_recovery_moved_mm, state.spacing_recovery_moved_mm, PosEpsilon) ||
                Changed(prev.spacing_recovery_remaining_mm, state.spacing_recovery_remaining_mm, PosEpsilon))
            {
                OnPropertyChanged(nameof(SpacingRecoveryStatusText));
            }
            // 即使后端拒绝进入且 phase 仍为 Idle，也要把 ToggleButton 校正回实际状态。
            OnPropertyChanged(nameof(SpacingRecoveryActive));
            OnPropertyChanged(nameof(SpacingRecoveryInactive));
            // 协同入口被拒绝时，RadioButton 仍可能保留本地点击状态；按后端快照校正。
            OnPropertyChanged(nameof(ModeCathFwdSelected));
            OnPropertyChanged(nameof(ModeCathRevSelected));
            OnPropertyChanged(nameof(ModeGuideFwdSelected));
            OnPropertyChanged(nameof(ModeGuideRevSelected));
			OnPropertyChanged(nameof(ModeCooperativeDeliverySelected));
			OnPropertyChanged(nameof(ModeCooperativeRetractionSelected));
			OnPropertyChanged(nameof(TrackingCompensationEnabled));
			OnPropertyChanged(nameof(Axis1TrackingError));
			OnPropertyChanged(nameof(Axis6TrackingError));
			OnPropertyChanged(nameof(Axis1CompensationGain));
			OnPropertyChanged(nameof(Axis6CompensationGain));
			OnPropertyChanged(nameof(TrackingCompensationCanEnable));
			OnPropertyChanged(nameof(TrackingCompensationToggleEnabled));
			OnPropertyChanged(nameof(TrackingParametersEditable));
			OnPropertyChanged(nameof(TrackingStatusText));
			NotifyExperimentProperties();
			NotifyAdsProperties();

            NotifyIfChanged(ref _prevConnected, connected, nameof(IsConnected));
			OnPropertyChanged(nameof(CanStartExperimentRecording));
			OnPropertyChanged(nameof(CanOpenCameraPreview));
            _prevState = state;
            StateUpdated?.Invoke(state);
        }

        private void NotifyAllUiProperties()
        {
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
            OnPropertyChanged(nameof(ModeCathFwdSelected));
            OnPropertyChanged(nameof(ModeCathRevSelected));
            OnPropertyChanged(nameof(ModeGuideFwdSelected));
            OnPropertyChanged(nameof(ModeGuideRevSelected));
            OnPropertyChanged(nameof(ModeCooperativeDeliverySelected));
            OnPropertyChanged(nameof(ModeCooperativeRetractionSelected));
            OnPropertyChanged(nameof(ModeSwitchAllowed));
            OnPropertyChanged(nameof(CooperativeModeEnabled));
            OnPropertyChanged(nameof(DualHandleReady));
            OnPropertyChanged(nameof(CooperativeReturnOwner));
            OnPropertyChanged(nameof(CooperativeStatusText));
            OnPropertyChanged(nameof(Axis6SoftLimitHold));
            OnPropertyChanged(nameof(Axis6SoftLimitText));
            OnPropertyChanged(nameof(ControlActive));
            OnPropertyChanged(nameof(FreezeActive));
            OnPropertyChanged(nameof(EstopHold));
            OnPropertyChanged(nameof(FfEnabled));
            OnPropertyChanged(nameof(CalZeroed));
            OnPropertyChanged(nameof(Force582F));
            OnPropertyChanged(nameof(Force582N));
            OnPropertyChanged(nameof(Force582TheoryF));
            OnPropertyChanged(nameof(Force582TheoryN));
            OnPropertyChanged(nameof(GravityCompEnabled));
            OnPropertyChanged(nameof(IsConnected));
            OnPropertyChanged(nameof(Axis1Reverse));
            OnPropertyChanged(nameof(Axis6Reverse));
            OnPropertyChanged(nameof(StartupWaiting));
            OnPropertyChanged(nameof(StartupCompleted));
            OnPropertyChanged(nameof(PhaseText));
            OnPropertyChanged(nameof(FtExpPhase));
            OnPropertyChanged(nameof(FtExpVelocityLevel));
            OnPropertyChanged(nameof(FtExpTrialId));
            OnPropertyChanged(nameof(FtExpRepeatInLevel));
            OnPropertyChanged(nameof(FtExpVRatioCurr));
            OnPropertyChanged(nameof(FtExpAxis1Target));
            OnPropertyChanged(nameof(FtExpActive));
            OnPropertyChanged(nameof(FtExpAborted));
            OnPropertyChanged(nameof(FtExpPhaseText));
            OnPropertyChanged(nameof(SpacingRecoveryActive));
            OnPropertyChanged(nameof(SpacingRecoveryInactive));
            OnPropertyChanged(nameof(SpacingRecoveryStatusText));
			OnPropertyChanged(nameof(TrackingCompensationEnabled));
			OnPropertyChanged(nameof(Axis1TrackingError));
			OnPropertyChanged(nameof(Axis6TrackingError));
			OnPropertyChanged(nameof(Axis1CompensationGain));
			OnPropertyChanged(nameof(Axis6CompensationGain));
			OnPropertyChanged(nameof(TrackingCompensationCanEnable));
			OnPropertyChanged(nameof(TrackingCompensationToggleEnabled));
			OnPropertyChanged(nameof(TrackingParametersEditable));
			OnPropertyChanged(nameof(TrackingStatusText));
			NotifyExperimentProperties();
			NotifyAdsProperties();
        }

		private void NotifyExperimentProperties()
		{
			OnPropertyChanged(nameof(RecordingState));
			OnPropertyChanged(nameof(RecordingStateText));
			OnPropertyChanged(nameof(RecordingErrorText));
			OnPropertyChanged(nameof(RecordingElapsedText));
			OnPropertyChanged(nameof(RecordingLossText));
			OnPropertyChanged(nameof(CanStartExperimentRecording));
			OnPropertyChanged(nameof(CanStopExperimentRecording));
			OnPropertyChanged(nameof(ExperimentNameEditable));
			OnPropertyChanged(nameof(CameraStatusText));
			OnPropertyChanged(nameof(CameraParameterText));
			OnPropertyChanged(nameof(CameraRecordingElapsedText));
			OnPropertyChanged(nameof(CameraRecording));
			OnPropertyChanged(nameof(CanOpenCameraPreview));
			OnPropertyChanged(nameof(ForceSampleValid));
			OnPropertyChanged(nameof(CleanForceValid));
			OnPropertyChanged(nameof(RawFnVoltage));
			OnPropertyChanged(nameof(RawFtVoltage));
			OnPropertyChanged(nameof(CleanForceN));
			OnPropertyChanged(nameof(CleanHandleTorqueNm));
		}

		private void NotifyAdsProperties()
		{
			OnPropertyChanged(nameof(AdsState));
			OnPropertyChanged(nameof(AdsHealthy));
			OnPropertyChanged(nameof(HostCommTimeout));
			OnPropertyChanged(nameof(AdsStateText));
			OnPropertyChanged(nameof(AdsDiagnosticsText));
			OnPropertyChanged(nameof(AdsCounterText));
		}

        private static bool Changed(double a, double b, double eps) => Math.Abs(a - b) > eps;

        private static double AxisValue(VisState s, int index)
        {
            if (s.axis_pos_from_left != null && s.axis_pos_from_left.Length > index)
                return s.axis_pos_from_left[index];
            if (s.axis_pos != null && s.axis_pos.Length > index)
                return s.axis_pos[index];
            return 0.0;
        }

        private static bool AxisChanged(VisState prev, VisState cur, int index) =>
            Changed(AxisValue(prev, index), AxisValue(cur, index), PosEpsilon);

        private static bool CylOpen(VisState s, int index)
        {
            ushort v = (s.cylinder_cmd != null && s.cylinder_cmd.Length > index) ? s.cylinder_cmd[index] : (ushort)0;
            // 与 Cyl1Open/Cyl2Open 等属性阈值一致：1/3 大开，2/4 小开
            return (index == 0 || index == 2) ? (v > 200) : (v < 200);
        }

        private static bool CylOpenChanged(VisState prev, VisState cur, int index) =>
            CylOpen(prev, index) != CylOpen(cur, index);

        private void NotifyIfChanged(ref bool cache, bool value, string name)
        {
            if (cache == value) return;
            cache = value;
            OnPropertyChanged(name);
        }

        public bool IsConnected => _client.IsConnected;
		public int AdsState => _state.ads_state;
		public bool HostCommTimeout => _state.host_comm_timeout;
		public bool AdsHealthy => IsConnected && AdsState == 2 && !HostCommTimeout;

		public string AdsStateText
		{
			get
			{
				if (!IsConnected) return "上位机管道未连接";
				if (HostCommTimeout) return "PLC 通信超时";
				switch (AdsState)
				{
					case 0: return "ADS 未启动";
					case 1: return "ADS 正在连接";
					case 2: return "ADS 正常";
					case 3: return "ADS 单拍软保持";
					case 4: return "ADS 重连中";
					case 5: return "PLC 已重启，等待人工恢复";
					case 6: return "ADS 通信错误";
					default: return $"ADS 未知状态（{AdsState}）";
				}
			}
		}

		public string AdsDiagnosticsText
		{
			get
			{
				string frequency = _state.ads_actual_hz > 0.0 &&
					!double.IsNaN(_state.ads_actual_hz) && !double.IsInfinity(_state.ads_actual_hz)
					? $"{_state.ads_actual_hz:F1} Hz"
					: "-- Hz";
				return $"实际频率 {frequency} · 数据龄 {FormatLatency(_state.ads_snapshot_age_us)} · RTT {FormatLatency(_state.ads_rtt_us)}";
			}
		}

		public string AdsCounterText =>
			$"失败 {_state.ads_failed_cycles} · 重连 {_state.ads_reconnect_count} · PLC 重启 {_state.plc_restart_count}";
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
                if (SpacingRecoveryActive) return "屈曲恢复";
                if (_state.guidewire_mode == 2)
                    return _state.cooperative_direction == 2 ? "协同撤出" : "协同递送";
                string mode = _state.guidewire_mode == 0 ? "导管" : "导丝";
                bool rev = _state.guidewire_mode == 0 ? _state.axis1_reverse : _state.axis6_reverse;
                return mode + (rev ? "撤出" : "递送");
            }
        }

        public bool Axis1Reverse => _state.axis1_reverse;
        public bool Axis6Reverse => _state.axis6_reverse;
        public bool ModeCathFwdSelected => SpacingRecoveryInactive && _state.guidewire_mode == 0 && !_state.axis1_reverse;
        public bool ModeCathRevSelected => SpacingRecoveryInactive && _state.guidewire_mode == 0 && _state.axis1_reverse;
        public bool ModeGuideFwdSelected => SpacingRecoveryInactive && _state.guidewire_mode == 1 && !_state.axis6_reverse;
        public bool ModeGuideRevSelected => SpacingRecoveryInactive && _state.guidewire_mode == 1 && _state.axis6_reverse;
        public bool ModeCooperativeDeliverySelected =>
            SpacingRecoveryInactive &&
            _state.guidewire_mode == 2 &&
            _state.cooperative_direction == 1;
        public bool ModeCooperativeRetractionSelected =>
            SpacingRecoveryInactive &&
            _state.guidewire_mode == 2 &&
            _state.cooperative_direction == 2;
        public bool DualHandleReady => _state.dual_handle_ready;
        public int CooperativeReturnOwner => _state.cooperative_return_owner;
		public bool ModeSwitchAllowed => CooperativeReturnOwner == 0;
		public string PhysicalButtonNoticeText => _physicalButtonNoticeText;

		private static string PhysicalButtonEventText(int eventCode)
		{
			switch (eventCode)
			{
				case 1: return "物理按钮触发：导管递送";
				case 2: return "物理按钮触发：导管撤出";
				case 3: return "物理按钮触发：导丝递送";
				case 4: return "物理按钮触发：导丝撤出";
				case 5: return "物理按钮触发：协同递送";
				case 6: return "物理按钮触发：协同撤出";
				default: return "物理按钮触发";
			}
		}
        public bool Axis6SoftLimitHold => _state.axis6_soft_limit_hold;
        public string Axis6SoftLimitText => Axis6SoftLimitHold
            ? "轴6软件限位：当前动作已阻断，松手或回到安全窗口后自动重新评估。"
            : "轴6软件限位：正常（<= 670 mm）。";
        public bool CooperativeModeEnabled =>
            DualHandleReady &&
            ModeSwitchAllowed &&
            StartupCompleted &&
            ControlActive &&
            !FreezeActive &&
            !EstopHold &&
            !FtExpActive &&
            !Axis6SoftLimitHold &&
            _state.axis1_phase == 0 &&
            _state.axis6_phase == 0;
        public string CooperativeStatusText
        {
            get
            {
                if (!DualHandleReady) return "双手柄未就绪（需重启上位机）";
                if (CooperativeReturnOwner == 1) return "导管换手中";
                if (CooperativeReturnOwner == 2) return "导丝换手中";
                if (ModeCooperativeDeliverySelected) return "协同递送 / 双手柄就绪";
                if (ModeCooperativeRetractionSelected) return "协同撤出 / 双手柄就绪";
                return "双手柄就绪：587 导管，582 导丝";
            }
        }
		public bool TrackingCompensationEnabled => _state.tracking_compensation_enabled;
		public double Axis1TrackingError => _state.axis1_tracking_error_mm;
		public double Axis6TrackingError => _state.axis6_tracking_error_mm;
		public double Axis1CompensationGain => _state.axis1_compensation_gain;
		public double Axis6CompensationGain => _state.axis6_compensation_gain;
		public bool TrackingParametersEditable => !TrackingCompensationEnabled;
		public bool TrackingCompensationCanEnable
		{
			get
			{
				bool forwardMode = (_state.guidewire_mode == 0 && !_state.axis1_reverse) ||
					(_state.guidewire_mode == 1 && !_state.axis6_reverse);
				return ControlActive && !FreezeActive && !EstopHold &&
					!SpacingRecoveryActive && !FtExpActive && forwardMode &&
					_state.axis1_phase == 0 && _state.axis6_phase == 0;
			}
		}
		public bool TrackingCompensationToggleEnabled =>
			TrackingCompensationEnabled || TrackingCompensationCanEnable;
		public string TrackingStatusText
		{
			get
			{
				string compensation = TrackingCompensationEnabled ? "补偿已开启" : "补偿关闭";
				return $"{compensation} · 轴1换手待补 {Axis1TrackingError:F3} mm / 增益 {Axis1CompensationGain:F3} · 轴6换手待补 {Axis6TrackingError:F3} mm / 增益 {Axis6CompensationGain:F3}";
			}
		}
        public bool StartupWaiting => _state.startup_waiting;
        public bool StartupCompleted => _state.startup_completed;

        public string PhaseText
        {
            get
            {
                if (_state.spacing_recovery_phase == 1) return "夹爪准备中";
                if (_state.spacing_recovery_phase == 2) return "手动恢复中";
                if (_state.spacing_recovery_phase == 3) return "停止并同步";
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
        public double Force582TheoryF => _state.force_582_theory_f;
        public double Force582TheoryN => _state.force_582_theory_n;
        public bool GravityCompEnabled => _state.gravity_comp_enabled;
        public VisState LatestState => _state;

		public int RecordingState => _state.recording_state;
		public bool CanStartExperimentRecording => IsConnected && (RecordingState == 0 || RecordingState == 4);
		public bool CanStopExperimentRecording => IsConnected && RecordingState == 2;
		public bool ExperimentNameEditable => RecordingState == 0 || RecordingState == 4;
		public bool CameraRecording => _state.camera_recording;
		public bool CanOpenCameraPreview => IsConnected;
		public bool ForceSampleValid => _state.force_sample_valid;
		public bool CleanForceValid => _state.clean_force_valid;
		public double RawFnVoltage => _state.fn_1_v;
		public double RawFtVoltage => _state.ft_1_v;
		public double CleanForceN => _state.clean_force_n;
		public double CleanHandleTorqueNm => _state.clean_handle_torque_nm;

		public string RecordingStateText
		{
			get
			{
				switch (RecordingState)
				{
					case 1: return "正在启动";
					case 2: return "记录中";
					case 3: return "正在停止并封装";
					case 4: return "启动失败";
					default: return "空闲";
				}
			}
		}

		public string RecordingErrorText
		{
			get
			{
				switch (_state.recording_error)
				{
					case 1: return "实验名称不是有效的 UTF-8 文本";
					case 2: return "无法创建会话目录";
					case 3: return "无法创建 force.csv";
					case 4: return "无法创建 motion.csv";
					case 5: return "force.csv 写入失败，记录已自动停止";
					case 6: return "motion.csv 写入失败，记录已自动停止";
					case 7: return "无法启动停止后台线程";
					case 8: return "力过渡专用 CSV 写入失败，统一记录已自动停止";
					case 9: return "video_frames.csv 写入失败，统一记录已自动停止";
					default: return "";
				}
			}
		}

		public string RecordingElapsedText => FormatElapsed(_state.recording_elapsed_us);
		public string CameraRecordingElapsedText => FormatElapsed(_state.camera_recording_elapsed_us);

		public string RecordingLossText
		{
			get
			{
				ulong dropped = _state.force_writer_dropped + _state.motion_writer_dropped;
				ulong forceMissed = _state.force_schedule_missed;
				ulong motionMissed = _state.motion_schedule_missed;
				ulong missed = forceMissed + motionMissed;
				return dropped == 0 && missed == 0
					? "CSV 队列与采样调度正常"
					: $"队列丢样 {dropped} · 调度错过：力 {forceMissed} / 位置 {motionMissed}";
			}
		}

		public string CameraStatusText
		{
			get
			{
				switch (_state.camera_state)
				{
					case 1: return "正在打开摄像设备：OsmoAction4";
					case 2: return "摄像设备预览中：OsmoAction4";
					case 3: return "摄像设备录像中：OsmoAction4";
					case 4: return "无法找到摄像设备：OsmoAction4";
					case 5: return "摄像设备已断开：OsmoAction4";
					case 6: return $"摄像设备错误：OsmoAction4（0x{unchecked((uint)_state.camera_error_code):X8}）";
					default: return "摄像设备未打开：OsmoAction4";
				}
			}
		}

		public string CameraParameterText
		{
			get
			{
				if (_state.camera_width <= 0 || _state.camera_height <= 0)
					return "等待实际视频参数";
				double fps = _state.camera_fps_denominator > 0
					? (double)_state.camera_fps_numerator / _state.camera_fps_denominator
					: 0.0;
				string input = _state.camera_input_format == 1 ? "H.264" :
					(_state.camera_input_format == 2 ? "MJPEG" :
					(_state.camera_input_format == 3 ? "RGB32" : "未知"));
				return $"{_state.camera_width}×{_state.camera_height} · {fps:F1} fps · 输入 {input} · 输出 H.264 / 无音频";
			}
		}

		private static string FormatElapsed(ulong elapsedUs)
		{
			TimeSpan value = TimeSpan.FromMilliseconds(elapsedUs / 1000.0);
			return value.ToString(@"hh\:mm\:ss");
		}

		private static string FormatLatency(ulong elapsedUs)
		{
			if (elapsedUs == ulong.MaxValue)
				return "--";
			if (elapsedUs >= 1_000_000)
				return $"{elapsedUs / 1_000_000.0:F2} s";
			if (elapsedUs >= 1_000)
				return $"{elapsedUs / 1_000.0:F1} ms";
			return $"{elapsedUs} us";
		}

        public bool SpacingRecoveryActive => _state.spacing_recovery_phase != 0;
        public bool SpacingRecoveryInactive => !SpacingRecoveryActive;
        public string SpacingRecoveryStatusText
        {
            get
            {
                switch (_state.spacing_recovery_phase)
                {
                    case 1:
                        return "夹爪准备中";
                    case 2:
                        return $"已恢复 {_state.spacing_recovery_moved_mm:F1} mm · 余量 {_state.spacing_recovery_remaining_mm:F1} mm";
                    case 3:
                        return "正在停止并同步";
                    default:
                        return $"轴1-轴3间距 {Axis3FromLeft - Axis1FromLeft:F1} mm";
                }
            }
        }

        // 力过渡决定性预实验（论文 §6.1）状态镜像。
        public int FtExpPhase => _state.ft_exp_phase;
        public int FtExpVelocityLevel => _state.ft_exp_velocity_level;
        public int FtExpTrialId => _state.ft_exp_trial_id;
        public int FtExpRepeatInLevel => _state.ft_exp_repeat_in_lvl;
        public double FtExpVRatioCurr => _state.ft_exp_v_ratio_curr;
        public double FtExpAxis1Target => _state.ft_exp_axis1_target;
        public bool FtExpActive => _state.ft_exp_active;
        public bool FtExpAborted => _state.ft_exp_aborted;
        public string FtExpPhaseText
        {
            get
            {
                switch (_state.ft_exp_phase)
                {
                    case 0: return "空闲";
                    case 1: return "起始静置";
                    case 2: return "接近起点";
                    case 3: return "推送中";
                    case 4: return "触发回退";
                    case 5: return "PLC 计划回退";
                    case 6: return "回退完成";
                    case 7: return "档间静置";
                    case 8: return "推进试次";
                    case 9: return "已完成";
                    case 10: return "已中止";
                    default: return "?";
                }
            }
        }

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
            return $"{axisName} ({description}): 已行进 {effectiveTravel:F2}/{fullStroke:F0} mm，距左限位 {fromLeft:F2} mm";
        }

        private static string FormatRotationAxis(string axisName, string description, double angle)
        {
            return $"{axisName} ({description}): {angle:F2}°";
        }

        public void SetCylinderManualState(int index, bool open) =>
            _client.SendCommand(open
                ? VisCommandType.SetCylinderManualOpen
                : VisCommandType.SetCylinderManualClosed, index);

        public void SetMode(int guidewireMode, int reverse) =>
            _client.SendCommand(VisCommandType.SetReverseMode, guidewireMode, reverse);

        public void SetCooperativeDelivery(bool enabled) =>
            _client.SendCommand(VisCommandType.SetCooperativeDelivery, enabled ? 1 : 0);

        public void SetCooperativeRetraction(bool enabled) =>
            _client.SendCommand(VisCommandType.SetCooperativeRetraction, enabled ? 1 : 0);

        public void SetAxis1PostReturnLead(double leadMm) =>
            _client.SendCommand(
                VisCommandType.SetAxis1PostReturnLead,
                (int)Math.Round(leadMm * 1000.0));

        public void SetSpacingRecovery(bool enabled) =>
            _client.SendCommand(VisCommandType.SetSpacingRecovery, enabled ? 1 : 0);

        public void ZeroForceSensor() =>
            _client.SendCommand(VisCommandType.ZeroForceSensor);

        public void ToggleForceFeedback() =>
            _client.SendCommand(VisCommandType.ToggleForceFeedback);

        public void SetGravityCompensation(bool enabled) =>
            _client.SendCommand(VisCommandType.SetGravityCompensation, enabled ? 1 : 0);

		public void SetTrackingCompensation(bool enabled) =>
			_client.SendCommand(VisCommandType.SetTrackingCompensation, enabled ? 1 : 0);

		public void SetTrackingCompensationParam(int fieldId, double value) =>
			_client.SendCommand(
				VisCommandType.SetTrackingCompensationParam,
				fieldId,
				(int)Math.Round(value * 1000.0));

		public void SendTrackingCompensationParams(
			double axis1Kp, double axis1Ki, double axis1MaxGain, double axis1MaxError,
			double axis6Kp, double axis6Ki, double axis6MaxGain, double axis6MaxError)
		{
			SetTrackingCompensationParam(0, axis1Kp);
			SetTrackingCompensationParam(1, axis1Ki);
			SetTrackingCompensationParam(2, axis1MaxGain);
			SetTrackingCompensationParam(3, axis1MaxError);
			SetTrackingCompensationParam(4, axis6Kp);
			SetTrackingCompensationParam(5, axis6Ki);
			SetTrackingCompensationParam(6, axis6MaxGain);
			SetTrackingCompensationParam(7, axis6MaxError);
		}

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

        // 力过渡决定性预实验：参数下发与启停。
        // 参数 field_id 编码与 C++ ForceTransitionExperiment::set_param_a/b 一致。
        public void SetFtExpParamInt(int fieldId, int intVal) =>
            _client.SendCommand(VisCommandType.SetFtExpParamA, fieldId, intVal);

        public void SetFtExpParamFixedX1000(int fieldId, double val) =>
            _client.SendCommand(VisCommandType.SetFtExpParamB, fieldId, (int)Math.Round(val * 1000.0));

        public void SendFtExpConfig(
            int numLevels, double[] vRatios, int repeatsPerLevel,
            double startPosMm, double pushTargetMm, double returnTriggerMm,
            double approachSpeedRatio, int dwellBetweenMs)
        {
            SetFtExpParamInt(0, numLevels);
            SetFtExpParamInt(1, repeatsPerLevel);
            SetFtExpParamInt(2, dwellBetweenMs);
            for (int i = 0; i < 6; ++i)
            {
                double v = (vRatios != null && i < vRatios.Length) ? vRatios[i] : 0.0;
                SetFtExpParamFixedX1000(10 + i, v);
            }
            SetFtExpParamFixedX1000(20, startPosMm);
            SetFtExpParamFixedX1000(21, pushTargetMm);
            SetFtExpParamFixedX1000(22, returnTriggerMm);
            SetFtExpParamFixedX1000(23, approachSpeedRatio);
        }

        public void StartForceTransitionExperiment() =>
            _client.SendCommand(VisCommandType.StartForceTransitionExperiment);

        public void StopForceTransitionExperiment() =>
            _client.SendCommand(VisCommandType.StopForceTransitionExperiment);

		public void StartExperimentRecording(string experimentName) =>
			_client.SendCommand(
				VisCommandType.StartExperimentRecording,
				0,
				0,
				experimentName ?? string.Empty);

		public void StopExperimentRecording() =>
			_client.SendCommand(VisCommandType.StopExperimentRecording);

		public void SetCameraPreview(bool enabled) =>
			_client.SendCommand(VisCommandType.SetCameraPreview, enabled ? 1 : 0);

		public void SetCleanForceMonitor(bool enabled) =>
			_client.SendCommand(VisCommandType.SetCleanForceMonitor, enabled ? 1 : 0);

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
