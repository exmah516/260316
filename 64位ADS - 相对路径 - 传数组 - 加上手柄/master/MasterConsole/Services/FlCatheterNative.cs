using System;
using System.ComponentModel;
using System.IO;
using System.Runtime.InteropServices;

namespace MasterConsole.Services
{
    // x64 SDK；Windows long 为 32 位、bool 为 1 字节。生命周期仅由手柄线程操作。
    internal static class FlCatheterNative
    {
        private static IntPtr _module;
        private static bool _deviceLayerUsed;
        private static ServoCallback _callback;
        public static bool IsLoaded => _module != IntPtr.Zero && _open != null;

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr LoadLibraryW(string path);
        [DllImport("kernel32.dll", CharSet = CharSet.Ansi, ExactSpelling = true, SetLastError = true)]
        private static extern IntPtr GetProcAddress(IntPtr module, string name);
        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool FreeLibrary(IntPtr module);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate int ServoCallback(IntPtr param);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate int Open(uint sn);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void Close();
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate int Start(ServoCallback cb, IntPtr param);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate int Stop();
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        private delegate bool Running();
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void Enable([MarshalAs(UnmanagedType.I1)] bool en, int id);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate int Status(int id);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void Force([In] double[] force, double torque, int id);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void Encoders([Out] int[] encs, int id);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void Values([Out] double[] values, int id);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void Switch(out byte buttons, int id);

        private static Open _open;
        private static Close _close;
        private static Start _start;
        private static Stop _stop;
        private static Running _running;
        private static Enable _enable;
        private static Status _serial, _status;
        private static Force _force;
        private static Encoders _encoders;
        private static Values _velocity, _joints;
        private static Switch _switch;

        public static void Load()
        {
            if (IsLoaded) return;
            if (_module != IntPtr.Zero) throw new InvalidOperationException("加载失败：上次 DLL 释放失败，资源仍保留");
            string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "FLCatheter.dll");
            _module = LoadLibraryW(path);
            if (_module == IntPtr.Zero) throw NativeError("加载失败：" + path);
            try
            {
                // 先全部绑定，最后发布 _open；采样时不重复查找函数。
                _close = Bind<Close>("closeDevice");
                _start = Bind<Start>("startServoLoop");
                _stop = Bind<Stop>("stopServoLoop");
                _running = Bind<Running>("isServoLoopRunning");
                _enable = Bind<Enable>("enableForces");
                _serial = Bind<Status>("getSerialNumber");
                _status = Bind<Status>("deviceStatus");
                _force = Bind<Force>("sendForce");
                _encoders = Bind<Encoders>("getEncoders");
                _velocity = Bind<Values>("getEncVel");
                _joints = Bind<Values>("getJoints");
                _switch = Bind<Switch>("getSwitch");
                _open = Bind<Open>("openDevice");
            }
            catch (Exception bindError)
            {
                // 尚未调用设备接口；绑定失败时释放此次加载，不留下半套委托。
                try { Unload(); }
                catch (Exception unloadError) { throw new InvalidOperationException(bindError.Message + "；" + unloadError.Message, bindError); }
                throw;
            }
        }

        private static Win32Exception NativeError(string stage)
        {
            int code = Marshal.GetLastWin32Error();
            return new Win32Exception(code, stage + $"（Win32 {code}）");
        }

        private static T Bind<T>(string name) where T : class
        {
            IntPtr address = GetProcAddress(_module, name);
            if (address == IntPtr.Zero) throw NativeError("绑定失败：" + name);
            return (T)(object)Marshal.GetDelegateForFunctionPointer(address, typeof(T));
        }

        public static void Unload()
        {
            if (_module == IntPtr.Zero) return;
            // ponytail: 当前 SDK 无线程 join/退出确认契约；取得厂商保证前拒绝热卸载已使用的设备层。
            // stop 返回 0、isServoLoopRunning=false 均不能证明全部原生线程/回调已经退出。
            if (_deviceLayerUsed) throw new InvalidOperationException("卸载未执行：SDK 未提供设备线程及回调退出确认；DLL 和回调引用保留，需厂商确认安全卸载契约");
            if (!FreeLibrary(_module)) throw NativeError("卸载失败：FreeLibrary");
            _module = IntPtr.Zero;
            _open = null; _close = null; _start = null; _stop = null; _running = null;
            _enable = null; _serial = _status = null; _force = null; _encoders = null;
            _velocity = _joints = null; _switch = null; _callback = null;
        }

        private static void RequireLoaded()
        {
            if (!IsLoaded) throw new InvalidOperationException("SDK 未加载或绑定未完成");
        }

        public static int openDevice(uint sn) { RequireLoaded(); _deviceLayerUsed = true; return _open(sn); }
        public static void closeDevice() { RequireLoaded(); _close(); }
        public static int startServoLoop(ServoCallback func, IntPtr param)
        { RequireLoaded(); _deviceLayerUsed = true; _callback = func; return _start(_callback, param); }
        public static int stopServoLoop() { RequireLoaded(); return _stop(); }
        public static bool isServoLoopRunning() { RequireLoaded(); return _running(); }
        public static void enableForces(bool en, int id) { RequireLoaded(); _enable(en, id); }
        public static int getSerialNumber(int id) { RequireLoaded(); return _serial(id); }
        public static int deviceStatus(int id) { RequireLoaded(); return _status(id); }
        public static void sendForce(double[] force, double torque, int id) { RequireLoaded(); _force(force, torque, id); }
        public static void getEncoders(int[] encs, int id) { RequireLoaded(); _encoders(encs, id); }
        public static void getEncVel(double[] values, int id) { RequireLoaded(); _velocity(values, id); }
        public static void getSwitch(out byte buttons, int id) { RequireLoaded(); _switch(out buttons, id); }
        public static void getJoints(double[] joints, int id) { RequireLoaded(); _joints(joints, id); }
    }
}
