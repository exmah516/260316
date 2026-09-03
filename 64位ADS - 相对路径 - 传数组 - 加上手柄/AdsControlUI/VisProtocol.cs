using System.Runtime.InteropServices;

namespace AdsControlUI
{
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct VisState
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] axis_pos;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] axis_pos_from_left;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public ushort[] cylinder_cmd;
        public int guidewire_mode;
        public int axis1_phase;
        public int axis6_phase;
        public int startup_phase;
        [MarshalAs(UnmanagedType.I1)] public bool control_active;
        [MarshalAs(UnmanagedType.I1)] public bool freeze_active;
        [MarshalAs(UnmanagedType.I1)] public bool estop_hold;
        [MarshalAs(UnmanagedType.I1)] public bool axis1_fast_return;
        [MarshalAs(UnmanagedType.I1)] public bool axis6_fast_retract;
        [MarshalAs(UnmanagedType.I1)] public bool self_check_done;
        [MarshalAs(UnmanagedType.I1)] public bool ff_enabled;
        [MarshalAs(UnmanagedType.I1)] public bool cal_zeroed;
        [MarshalAs(UnmanagedType.I1)] public bool axis1_reverse;
        [MarshalAs(UnmanagedType.I1)] public bool axis6_reverse;
        [MarshalAs(UnmanagedType.I1)] public bool startup_waiting;
        [MarshalAs(UnmanagedType.I1)] public bool startup_completed;
        public double ft_1_v;
        public double fn_1_v;
        public double force_582_f;
        public double force_582_n;
        public double force_587_f;
        public double force_587_n;
        public int loop_count;
        public uint tick_ms;
        public double force_582_theory_f;
        public double force_582_theory_n;
        [MarshalAs(UnmanagedType.I1)] public bool gravity_comp_enabled;
        // 力过渡决定性预实验（论文 §6.1）状态字段。
        public int ft_exp_phase;
        public int ft_exp_velocity_level;
        public int ft_exp_trial_id;
        public int ft_exp_repeat_in_lvl;
        public double ft_exp_v_ratio_curr;
        public double ft_exp_axis1_target;
        [MarshalAs(UnmanagedType.I1)] public bool ft_exp_active;
        [MarshalAs(UnmanagedType.I1)] public bool ft_exp_aborted;
        // 手动屈曲/间距恢复状态。
        public int spacing_recovery_phase;
        public double spacing_recovery_moved_mm;
        public double spacing_recovery_remaining_mm;
        // 协同递送状态。
        [MarshalAs(UnmanagedType.I1)] public bool dual_handle_ready;
        public int cooperative_return_owner;
        // 主从位移补偿只发布控制状态，不再持有磁盘会话状态。
        [MarshalAs(UnmanagedType.I1)] public bool tracking_compensation_enabled;
        public double axis1_tracking_error_mm;
        public double axis6_tracking_error_mm;
        public double axis1_compensation_gain;
        public double axis6_compensation_gain;
        // 协同方向：0=None，1=Delivery，2=Retraction。必须与 C++ VisState 末尾字段一致。
        public int cooperative_direction;
        // axis6 当前软限位阻断状态。必须与 C++ VisState 末尾字段一致。
        [MarshalAs(UnmanagedType.I1)] public bool axis6_soft_limit_hold;

        // 统一实验记录、纯净力与 Action 4 状态。必须与 C++ VisState 顺序完全一致。
        public int recording_state;
        public int recording_error;
        public ulong recording_elapsed_us;
        public ulong force_writer_dropped;
        public ulong motion_writer_dropped;
        public ulong force_schedule_missed;
        public ulong motion_schedule_missed;
        [MarshalAs(UnmanagedType.I1)] public bool force_sample_valid;
        [MarshalAs(UnmanagedType.I1)] public bool clean_force_valid;
        public double clean_force_n;
        public double clean_handle_torque_nm;
        public int camera_state;
        public int camera_input_format;
        public int camera_width;
        public int camera_height;
        public int camera_fps_numerator;
        public int camera_fps_denominator;
        [MarshalAs(UnmanagedType.I1)] public bool camera_preview_enabled;
        [MarshalAs(UnmanagedType.I1)] public bool camera_recording;
        public ulong camera_recording_elapsed_us;
		public ulong camera_frame_count;
		public ulong camera_dropped_frames;
		public int camera_error_code;
		// 物理 B7 模式选择的有效按下沿事件。必须与 C++ VisState 末尾字段一致。
		public uint physical_button_event_counter;
		public int physical_button_event_code;
		// ADS 通信诊断。必须与 C++ VisState 末尾字段顺序完全一致。
		public int ads_state;
		public double ads_actual_hz;
		public ulong ads_snapshot_age_us;
		public ulong ads_rtt_us;
		public ulong ads_failed_cycles;
		public ulong ads_reconnect_count;
		public ulong plc_restart_count;
		[MarshalAs(UnmanagedType.I1)] public bool host_comm_timeout;

		// 定位臂独立低频 ADS 状态。必须与 C++ VisState 末尾顺序一致。
		[MarshalAs(UnmanagedType.I1)] public bool arm_manual_enable;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_enable_req;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_power_done;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_power_busy;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_power_active;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_power_error;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public uint[] arm_power_error_id;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_reset_done;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_reset_busy;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_reset_active;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_reset_error;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public uint[] arm_reset_error_id;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public double[] arm_act_pos;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public double[] arm_act_vel;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_motion_busy;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_motion_done;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_motion_error;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public uint[] arm_motion_error_id;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public sbyte[] arm_cmd_dir;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5, ArraySubType = UnmanagedType.I1)] public bool[] arm_cmd_conflict;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public double[] arm_jog_velocity;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public double[] arm_jog_acc;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public double[] arm_jog_dec;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public double[] arm_jog_jerk;

		[MarshalAs(UnmanagedType.I1)] public bool axis4_manual_busy;
		[MarshalAs(UnmanagedType.I1)] public bool axis4_manual_done;
		[MarshalAs(UnmanagedType.I1)] public bool axis4_manual_error;
		public uint axis4_manual_error_id;
		// 力反馈-保持模式状态：0=无保持，1=导管，2=导丝，3=双侧。
		[MarshalAs(UnmanagedType.I1)] public bool force_feedback_hold_enabled;
		[MarshalAs(UnmanagedType.I1)] public bool force_feedback_hold_active;
		public int force_feedback_hold_owner;
	}

    public enum VisCommandType : int
    {
        None = 0,
        SetCylinderManualOpen = 1,
        SetCylinderManualClosed = 2,
        RequestModeSwitch = 3,
        ZeroForceSensor = 4,
        ToggleForceFeedback = 5,
        SetReverseMode = 6,
        // 7 为已删除的旧力记录命令，保留数值空洞。
        SetStartupAxisPos = 8,
        SetStartupAxisDeg = 9,
        SetStartupSpeed = 10,
        ExecuteStartup = 11,
        SelectDirectControl = 12,
        SetGravityCompensation = 13,
        // 力过渡决定性预实验（论文 §6.1）控制命令。
        StartForceTransitionExperiment = 14,
        StopForceTransitionExperiment = 15,
        SetFtExpParamA = 16,
        SetFtExpParamB = 17,
        SetSpacingRecovery = 18,
        SetCooperativeDelivery = 19,
        // 20 为已删除的旧位移记录命令，保留数值空洞。
        SetTrackingCompensation = 21,
        SetTrackingCompensationParam = 22,
        SetCooperativeRetraction = 23,
        SetAxis1PostReturnLead = 24,
        StartExperimentRecording = 25,
        StopExperimentRecording = 26,
        SetCameraPreview = 27,
        SetCleanForceMonitor = 28,
		SetArmManualEnable = 29,
		SetArmAxisEnable = 30,
		RequestArmAxisReset = 31,
		SetArmAxisJog = 32,
		SetArmJogParameter = 33,
		SetAxis4ManualJog = 34,
		SetForceFeedbackHold = 35,
		SetYValveOpen = 36,
		SetInjectorManualJog = 37,
		EmergencyRetractDevice = 38,
	}
}
