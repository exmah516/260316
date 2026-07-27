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

        private void RefreshFromPipe()
        {
            bool connected = _client.IsConnected;
            if (!_client.TryGetLatestState(out var state))
            {
                NotifyIfChanged(ref _prevConnected, connected, nameof(IsConnected));
                return;
            }

            _state = state;
            if (!_hasUiSnapshot)
            {
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
            if (prev.force_log_running != state.force_log_running) OnPropertyChanged(nameof(ForceLogRunning));
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
			OnPropertyChanged(nameof(TrackingLogRunning));
			OnPropertyChanged(nameof(TrackingCompensationEnabled));
			OnPropertyChanged(nameof(Axis1TrackingError));
			OnPropertyChanged(nameof(Axis6TrackingError));
			OnPropertyChanged(nameof(Axis1CompensationGain));
			OnPropertyChanged(nameof(Axis6CompensationGain));
			OnPropertyChanged(nameof(TrackingLogDropped));
			OnPropertyChanged(nameof(TrackingCompensationCanEnable));
			OnPropertyChanged(nameof(TrackingCompensationToggleEnabled));
			OnPropertyChanged(nameof(TrackingParametersEditable));
			OnPropertyChanged(nameof(TrackingStatusText));

            NotifyIfChanged(ref _prevConnected, connected, nameof(IsConnected));
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
            OnPropertyChanged(nameof(ForceLogRunning));
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
			OnPropertyChanged(nameof(TrackingLogRunning));
			OnPropertyChanged(nameof(TrackingCompensationEnabled));
			OnPropertyChanged(nameof(Axis1TrackingError));
			OnPropertyChanged(nameof(Axis6TrackingError));
			OnPropertyChanged(nameof(Axis1CompensationGain));
			OnPropertyChanged(nameof(Axis6CompensationGain));
			OnPropertyChanged(nameof(TrackingLogDropped));
			OnPropertyChanged(nameof(TrackingCompensationCanEnable));
			OnPropertyChanged(nameof(TrackingCompensationToggleEnabled));
			OnPropertyChanged(nameof(TrackingParametersEditable));
			OnPropertyChanged(nameof(TrackingStatusText));
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
        public bool ModeSwitchAllowed => SpacingRecoveryInactive && CooperativeReturnOwner == 0;
        public bool Axis6SoftLimitHold => _state.axis6_soft_limit_hold;
        public string Axis6SoftLimitText => Axis6SoftLimitHold
            ? "轴6软件限位：已锁止，需安全处理后重新启动控制。"
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
        public bool ForceLogRunning => _state.force_log_running;
		public bool TrackingLogRunning => _state.tracking_log_running;
		public bool TrackingCompensationEnabled => _state.tracking_compensation_enabled;
		public double Axis1TrackingError => _state.axis1_tracking_error_mm;
		public double Axis6TrackingError => _state.axis6_tracking_error_mm;
		public double Axis1CompensationGain => _state.axis1_compensation_gain;
		public double Axis6CompensationGain => _state.axis6_compensation_gain;
		public ulong TrackingLogDropped => _state.tracking_log_dropped;
		public bool TrackingParametersEditable => !TrackingCompensationEnabled;
		public bool TrackingCompensationCanEnable
		{
			get
			{
				bool forwardMode = (_state.guidewire_mode == 0 && !_state.axis1_reverse) ||
					(_state.guidewire_mode == 1 && !_state.axis6_reverse);
				return TrackingLogRunning && ControlActive && !FreezeActive && !EstopHold &&
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
				if (!TrackingLogRunning) return "自动记录启动失败";
				string compensation = TrackingCompensationEnabled ? "补偿已开启" : "补偿关闭";
				string dropped = TrackingLogDropped > 0 ? $" · 丢弃 {TrackingLogDropped}" : "";
				return $"自动 20 Hz 记录 · {compensation} · 轴1换手待补 {Axis1TrackingError:F3} mm / 增益 {Axis1CompensationGain:F3} · 轴6换手待补 {Axis6TrackingError:F3} mm / 增益 {Axis6CompensationGain:F3}{dropped}";
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

        public void ToggleForceLog() =>
            _client.SendCommand(VisCommandType.ToggleForceLog);

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
