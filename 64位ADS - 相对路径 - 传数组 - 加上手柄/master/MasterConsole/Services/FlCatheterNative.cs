using System;
using System.Runtime.InteropServices;

namespace MasterConsole.Services
{
    /// <summary>
    /// FLCatheter.dll（手柄 SDK，x64）的原生调用。签名与 手柄/include/FLCatheter.h 一一对应：
    /// C++ 的 long 在 Windows x64 上是 32 位，因此编码器数组用 int[]；bool 是 1 字节。
    /// DLL 由 MasterConsole.csproj 复制到输出目录。
    /// </summary>
    internal static class FlCatheterNative
    {
        private const string Dll = "FLCatheter.dll";

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate int ServoCallback(IntPtr param);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int openDevice(uint sn);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void closeDevice();

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int startServoLoop(ServoCallback func, IntPtr param);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int stopServoLoop();

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void enableForces([MarshalAs(UnmanagedType.I1)] bool en, int id);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getSerialNumber(int id);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int deviceStatus(int id);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void sendForce([In] double[] force, double torque, int id);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void getEncoders([Out] int[] encs, int id);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void getEncVel([Out] double[] evels, int id);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void getSwitch(out byte swts, int id);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void getJoints([Out] double[] joints, int id);
    }
}
