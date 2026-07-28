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
        [MarshalAs(UnmanagedType.I1)] public bool force_log_running;
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
        // 主从位移实验状态。必须与 C++ VisState 末尾字段完全一致。
        [MarshalAs(UnmanagedType.I1)] public bool tracking_log_running;
        [MarshalAs(UnmanagedType.I1)] public bool tracking_compensation_enabled;
        public double axis1_tracking_error_mm;
        public double axis6_tracking_error_mm;
        public double axis1_compensation_gain;
        public double axis6_compensation_gain;
        public ulong tracking_log_dropped;
        // 协同方向：0=None，1=Delivery，2=Retraction。必须与 C++ VisState 末尾字段一致。
        public int cooperative_direction;
        // axis6 当前软限位阻断状态。必须与 C++ VisState 末尾字段一致。
        [MarshalAs(UnmanagedType.I1)] public bool axis6_soft_limit_hold;
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
        ToggleForceLog = 7,
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
        SetTrackingLog = 20,
        SetTrackingCompensation = 21,
        SetTrackingCompensationParam = 22,
        SetCooperativeRetraction = 23,
        SetAxis1PostReturnLead = 24,
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct VisCommand
    {
        public VisCommandType type;
        public int param1;
        public int param2;
    }
}
