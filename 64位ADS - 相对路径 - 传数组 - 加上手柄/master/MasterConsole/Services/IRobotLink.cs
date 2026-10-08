using System;
using System.Threading.Tasks;
using MasterConsole.Protocol;

namespace MasterConsole.Services
{
    /// <summary>命令回执：对应协议 ack 的 state。</summary>
    public sealed class CommandResult
    {
        public bool Ok { get; private set; }
        public string Reason { get; private set; } = "";

        public static CommandResult Done() => new CommandResult { Ok = true };
        public static CommandResult DoneWithReason(string reason) => new CommandResult { Ok = true, Reason = reason ?? "" };
        public static CommandResult Rejected(string reason) => new CommandResult { Ok = false, Reason = reason };
    }

    public sealed class LogMessage
    {
        public DateTime Time { get; set; } = DateTime.Now;
        /// <summary>info / warn / error</summary>
        public string Level { get; set; } = "info";
        public string Text { get; set; } = "";
    }

    /// <summary>链路质量统计，显示在链路区。</summary>
    public sealed class LinkStats
    {
        public uint Session { get; set; }
        public double RttMs { get; set; }
        public double StatusAgeMs { get; set; }
        public double HapticAgeMs { get; set; }
        public double StatusHz { get; set; }
        public ulong DroppedFrames { get; set; }
        /// <summary>主端本机手柄状态（物理序列号 582 / 587）。</summary>
        public bool Handle582Online { get; set; }
        public bool Handle587Online { get; set; }
        public bool HandleSdkMissing { get; set; }
        public bool CatheterReversePressed { get; set; }
    }

    /// <summary>
    /// 主端与从端（数据端）之间的链路抽象。界面只依赖这个接口：
    /// 现在由 <see cref="SimulatedRobotLink"/> 实现以便独立开发界面，
    /// 后续由 UDP/TCP 实现（RemoteRobotLink）替换，界面不用改。
    /// 事件可能在非 UI 线程触发，订阅方负责切回 UI 线程。
    /// </summary>
    public interface IRobotLink : IDisposable
    {
        string Name { get; }
        bool IsConnected { get; }
        bool HasControl { get; }

        event EventHandler ConnectionChanged;
        event EventHandler<StatusFrame> StatusReceived;
        event EventHandler<LogMessage> LogReceived;

        Task ConnectAsync();
        void Disconnect();

        Task<CommandResult> AcquireControlAsync();
        Task<CommandResult> ReleaseControlAsync();

        /// <summary>进入器械准备位置：导管搓捻机构位置 + Y 阀及导丝机构位置（mm）。</summary>
        Task<CommandResult> PreparePositionAsync(double catheterMm, double wireMm);

        /// <summary>开始控制：已到达器械准备位置后，在当前位置直接进入手柄控制。</summary>
        Task<CommandResult> StartControlAsync();

        /// <summary>重读主端手柄并请求从端按成功读取的手柄重建中立基准。</summary>
        Task<CommandResult> RefreshHandlesAsync();

        Task<CommandResult> SetArmManualEnableAsync(bool enabled);
        Task<CommandResult> SetArmAxisEnableAsync(int axis, bool enabled);
        Task<CommandResult> ResetArmAxisAsync(int axis);
        Task<CommandResult> SetArmCartesianJogAsync(int mode, int speedMilli);
        Task<CommandResult> StopArmAsync();
        Task<CommandResult> ReturnArmProgramZeroAsync();
        Task<CommandResult> SetArmCartesianParameterAsync(int field, int valueMilli);
        void SetArmAxisJog(int axis, int direction);
        void KeepArmCartesianAlive();

        /// <summary>力反馈开关；开启前的零点采集由从端自动完成。</summary>
        Task<CommandResult> SetForceFeedbackAsync(bool enable);

        /// <summary>电缸 1–4：engaged=true 打开，false 恢复原状态。</summary>
        Task<CommandResult> SetCylinderAsync(int index, bool engaged);

        /// <summary>Y 阀：closed=true 关闭，false 打开（取消关闭）。</summary>
        Task<CommandResult> SetYValveClosedAsync(bool closed);

        /// <summary>注射器按住状态（-1 拉 / 0 停 / +1 推），随 100 Hz 控制帧持续发送。</summary>
        void SetInjector(int index, int direction);

        /// <summary>轴4点动按住状态（-1 后退 / 0 停 / +1 前进），随控制帧持续发送。</summary>
        void SetAxis4(int direction);

        LinkStats GetStats();
    }
}
