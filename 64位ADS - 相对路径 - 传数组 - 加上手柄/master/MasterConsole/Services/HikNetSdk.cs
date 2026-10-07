using System;
using System.Runtime.InteropServices;

namespace MasterConsole.Services
{
    internal static class HikNetSdk
    {
        internal const int StreamData = 2;
        internal const uint PreviewException = 32771;
        internal const uint Reconnect = 32773;
        internal const uint RealPlayNetClose = 112;
        internal const uint RealPlayNoData = 113;
        internal const uint RealPlayReconnect = 114;

        [StructLayout(LayoutKind.Sequential)]
        internal struct DeviceInfoV30
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 48, ArraySubType = UnmanagedType.I1)] public byte[] Serial;
            public byte AlarmIn, AlarmOut, Disk, DvrType, Chan, StartChan, AudioChan, IpChan, ZeroChan;
            public byte MainProto, SubProto, Support, Support1, Support2;
            public ushort DevType;
            public byte Support3, MultiStreamProto, StartDChan, StartDTalkChan, HighDChanNum, Support4, LanguageType;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9, ArraySubType = UnmanagedType.I1)] public byte[] Reserved;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct DeviceInfoV40
        {
            public DeviceInfoV30 Device;
            public byte SupportLock, RetryLogin, PasswordLevel, ProxyType;
            public uint SurplusLockTime;
            public byte CharEncodeType, SupportDev5, Support, LoginMode;
            public int OemCode, ResidualValidity;
            public byte ResidualValidityValid;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 243, ArraySubType = UnmanagedType.I1)] public byte[] Reserved;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct UserLoginInfo
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 129, ArraySubType = UnmanagedType.I1)] public byte[] DeviceAddress;
            public byte UseTransport;
            public ushort Port;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64, ArraySubType = UnmanagedType.I1)] public byte[] UserName;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64, ArraySubType = UnmanagedType.I1)] public byte[] Password;
            public LoginResultCallback LoginCallback;
            public IntPtr User;
            [MarshalAs(UnmanagedType.Bool)] public bool AsyncLogin;
            public byte ProxyType, UseUtcTime, LoginMode, Https;
            public int ProxyId;
            public byte VerifyMode;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 119, ArraySubType = UnmanagedType.I1)] public byte[] Reserved;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct PreviewInfo
        {
            public int Channel;
            public uint StreamType, LinkMode;
            public IntPtr PlayWnd;
            [MarshalAs(UnmanagedType.Bool)] public bool Blocked, PassbackRecord;
            public byte PreviewMode;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32, ArraySubType = UnmanagedType.I1)] public byte[] StreamId;
            public byte ProtoType, Reserved1, VideoCodingType;
            public uint DisplayBufNum;
            public byte NpqMode;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 215, ArraySubType = UnmanagedType.I1)] public byte[] Reserved;
        }

        internal delegate void RealDataCallback(int realHandle, uint dataType, IntPtr buffer, uint bufferSize, IntPtr user);
        internal delegate void ExceptionCallback(uint type, int userId, int handle, IntPtr user);
        internal delegate void LoginResultCallback(int userId, int result, IntPtr deviceInfo, IntPtr user);

        [DllImport("HCNetSDK.dll", CallingConvention = CallingConvention.StdCall)] internal static extern bool NET_DVR_Init();
        [DllImport("HCNetSDK.dll", CallingConvention = CallingConvention.StdCall)] internal static extern bool NET_DVR_Cleanup();
        [DllImport("HCNetSDK.dll", CallingConvention = CallingConvention.StdCall)] internal static extern uint NET_DVR_GetLastError();
        [DllImport("HCNetSDK.dll", CallingConvention = CallingConvention.StdCall)] internal static extern int NET_DVR_Login_V40(ref UserLoginInfo login, ref DeviceInfoV40 device);
        [DllImport("HCNetSDK.dll", CallingConvention = CallingConvention.StdCall)] internal static extern bool NET_DVR_Logout(int userId);
        [DllImport("HCNetSDK.dll", CallingConvention = CallingConvention.StdCall)] internal static extern int NET_DVR_RealPlay_V40(int userId, ref PreviewInfo preview, RealDataCallback callback, IntPtr user);
        [DllImport("HCNetSDK.dll", CallingConvention = CallingConvention.StdCall)] internal static extern bool NET_DVR_StopRealPlay(int realHandle);
        [DllImport("HCNetSDK.dll", CallingConvention = CallingConvention.StdCall)] internal static extern bool NET_DVR_SetExceptionCallBack_V30(uint message, IntPtr window, ExceptionCallback callback, IntPtr user);
    }
}
