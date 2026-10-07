namespace MasterConsole.Protocol
{
    /// <summary>帧头（16 字节）。</summary>
    public struct FrameHeader
    {
        public byte Type;
        public uint Session;
        public uint Seq;
        public uint TsMs;
    }

    /// <summary>单只手柄的一次采样（26 字节）。</summary>
    public struct HandleSample
    {
        public byte Buttons;
        public bool Valid;
        public int Encoder0, Encoder1;
        public float Joint0, Joint1;
        public float Vel0, Vel1;
    }

    /// <summary>主→从：手柄输入 + 注射器按住状态，100 Hz。</summary>
    public struct ControlFrame
    {
        public HandleSample HandleA; // 序列号 582
        public HandleSample HandleB; // 序列号 587
        /// <summary>注射器 1 方向：-1 拉，0 停，+1 推。</summary>
        public sbyte Injector1Dir;
        /// <summary>注射器 2 方向：-1 拉，0 停，+1 推。</summary>
        public sbyte Injector2Dir;
        /// <summary>轴4点动方向：-1 后退，0 停，+1 前进。</summary>
        public sbyte Axis4Dir;
    }

    /// <summary>手柄力输出（10 字节）。</summary>
    public struct HapticOut
    {
        public bool Enable;
        /// <summary>力作用的 SDK 轴（0..2），由从端按 axial_force_axis 指定。</summary>
        public sbyte Axis;
        public float ForceN;
        public float TorqueNm;
    }

    /// <summary>从→主：手柄力反馈输出，100 Hz。</summary>
    public struct HapticFrame
    {
        public uint EchoTsMs;
        public ushort HoldMs;
        public HapticOut HandleA;
        public HapticOut HandleB;
    }

    /// <summary>从→主：整机状态，15 Hz。</summary>
    public struct StatusFrame
    {
        public uint EchoTsMs;
        public ushort HoldMs;
        public StatusFlags Flags;
        public int Mode;
        public int Phase;
        public int SelfCheckStatus;
        public int AdsState;
        public ushort Cylinder1, Cylinder2, Cylinder3, Cylinder4;
        public byte CylinderManualMask;
        public sbyte InjectorActive1, InjectorActive2;
        public sbyte Axis4Active;
        public float Force582F, Force582N, Force587F, Force587N, CleanForceN;
        public float AdsActualHz;
        /// <summary>预留：七轴位置（界面暂不显示）。长度 7。</summary>
        public float[] AxisPos;
        /// <summary>预留：七轴距左限位（界面暂不显示）。长度 7。</summary>
        public float[] AxisFromLeft;

        public bool Has(StatusFlags f) => (Flags & f) != 0;
    }
}
