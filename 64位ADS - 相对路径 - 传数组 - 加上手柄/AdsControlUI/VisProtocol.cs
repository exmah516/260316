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
    }

    public enum VisCommandType : int
    {
        None = 0,
        SetCylinderOverride = 1,
        ClearCylinderOverride = 2,
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
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct VisCommand
    {
        public VisCommandType type;
        public int param1;
        public int param2;
    }
}
