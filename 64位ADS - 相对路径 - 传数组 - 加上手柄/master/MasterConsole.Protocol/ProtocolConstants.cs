using System;

namespace MasterConsole.Protocol
{
    /// <summary>协议常量。必须与 protocol/PROTOCOL.md、protocol/cpp/remote_protocol.h 一致。</summary>
    public static class ProtocolConstants
    {
        public const ushort Magic = 0x4956;
        public const byte Version = 1;

        public const byte TypeControl = 1;
        public const byte TypeHaptic = 2;
        public const byte TypeStatus = 3;

        public const int HeaderLen = 16;
        public const int MacLen = 8;
        public const int ControlPayloadLen = 55;
        public const int HapticPayloadLen = 26;
        public const int StatusPayloadLen = 118;

        public const int ControlFrameLen = HeaderLen + ControlPayloadLen + MacLen; // 79
        public const int HapticFrameLen = HeaderLen + HapticPayloadLen + MacLen;   // 50
        public const int StatusFrameLen = HeaderLen + StatusPayloadLen + MacLen;   // 142

        public const int TcpPort = 32000;
        public const int UdpPort = 32001;

        /// <summary>主端 200 ms 未收到触觉帧即把手柄力输出清零。</summary>
        public const int HapticTimeoutMs = 200;
        /// <summary>主端 1 s 未收到状态帧即视为数据过期。</summary>
        public const int StatusTimeoutMs = 1000;
    }

    /// <summary>StatusFrame.flags 位定义。</summary>
    [Flags]
    public enum StatusFlags : uint
    {
        None = 0,
        ControlActive = 1u << 0,
        EstopHold = 1u << 1,
        SelfCheckDone = 1u << 2,
        FfEnabled = 1u << 3,
        CalZeroed = 1u << 4,
        HostCommTimeout = 1u << 5,
        YValveClosed = 1u << 6,
        LeaseHeld = 1u << 7,
        StartupCompleted = 1u << 8,
        AdsHealthy = 1u << 9,
        FfZeroing = 1u << 10,
    }
}
