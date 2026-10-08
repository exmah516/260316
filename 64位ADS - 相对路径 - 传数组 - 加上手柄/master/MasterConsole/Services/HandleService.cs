using System;
using System.Diagnostics;
using System.Threading;
using MasterConsole.Protocol;

namespace MasterConsole.Services
{
    /// <summary>
    /// 主端手柄服务：独占 FLCatheter SDK，在专用线程里完成打开重试、采样、力输出。
    /// 所有 SDK 调用（除 SDK 自己的伺服回调线程外）都只发生在这一个线程，避免并发访问设备。
    /// 手柄 A / B 对应物理序列号 582 / 587（协议约定，角色映射由从端决定）。
    /// </summary>
    public sealed class HandleService : IDisposable
    {
        public const uint SerialA = 582;
        public const uint SerialB = 587;

        private const int LoopPeriodMs = 5;           // 约 200 Hz 采样
        private const int OpenRetryMs = 1000;         // 未打开的手柄每秒重试一次
        private const long SampleMaxAgeMs = 100;      // 采样超过此龄期则对外标记无效
        private const long HapticTimeoutMs = ProtocolConstants.HapticTimeoutMs;

        // SDK 伺服循环回调：与原从端 SyncUpdate 相同，只返回设备状态。必须保持引用，防止被回收。
        private static readonly FlCatheterNative.ServoCallback ServoCb = _ => FlCatheterNative.deviceStatus(0);
        private static bool _servoStarted;
        private static int _openCount;

        private sealed class Device
        {
            public uint Serial;
            public int Id = -1;
            public long NextOpenMs;
            public bool Online;
            // 对外采样（受 lock 保护）
            public HandleSample Sample;
            public long SampleMs = -1;
            // 触觉指令（受 lock 保护）
            public bool HapEnable;
            public int HapAxis;
            public double HapForce;
            public double HapTorque;
            public long HapMs = -1;
            // 线程内状态：当前是否已向设备写入非零力
            public bool ForceNonZero;
            public long LastZeroSendMs;
        }

        private readonly Device[] _dev = { new Device { Serial = SerialA }, new Device { Serial = SerialB } };
        private readonly object _lock = new object();
        private readonly Stopwatch _clock = Stopwatch.StartNew();
        private readonly Thread _thread;
        private readonly ManualResetEventSlim _refreshDone = new ManualResetEventSlim(false);
        private volatile bool _stop;
        private volatile bool _sdkMissing;
        private int _refreshRequested;
        private int _refreshResultMask;
        private int _refreshInProgress;

        public HandleService()
        {
            _thread = new Thread(Run) { IsBackground = true, Name = "HandleService", Priority = ThreadPriority.AboveNormal };
            _thread.Start();
        }

        /// <summary>手柄 SDK（FLCatheter.dll）无法加载时为 true。</summary>
        public bool SdkMissing => _sdkMissing;

        public bool IsOnline(int slot) => slot >= 0 && slot < 2 && _dev[slot].Online;

        /// <summary>在手柄线程中立即重读两只设备，返回本次成功采样的位掩码（bit0=582，bit1=587）。</summary>
        public int RefreshNow(int timeoutMs = 500)
        {
            if (Interlocked.Exchange(ref _refreshInProgress, 1) != 0) return -1;
            try
            {
                _refreshDone.Reset();
                Interlocked.Exchange(ref _refreshRequested, 1);
                if (!_refreshDone.Wait(timeoutMs)) return 0;
                return Volatile.Read(ref _refreshResultMask);
            }
            finally
            {
                Volatile.Write(ref _refreshInProgress, 0);
            }
        }

        /// <summary>取最新采样；打开失败、读取失败或采样过期时 Valid=false。</summary>
        public HandleSample GetSample(int slot)
        {
            var d = _dev[slot];
            lock (_lock)
            {
                var s = d.Sample;
                if (d.SampleMs < 0 || _clock.ElapsedMilliseconds - d.SampleMs > SampleMaxAgeMs) s.Valid = false;
                return s;
            }
        }

        /// <summary>收到从端触觉帧时调用：记录力指令，由手柄线程执行。</summary>
        public void SetHaptic(int slot, HapticOut o)
        {
            var d = _dev[slot];
            lock (_lock)
            {
                d.HapEnable = o.Enable;
                d.HapAxis = o.Axis < 0 ? 0 : (o.Axis > 2 ? 2 : o.Axis);
                d.HapForce = Finite(o.ForceN);
                d.HapTorque = Finite(o.TorqueNm);
                d.HapMs = _clock.ElapsedMilliseconds;
            }
        }

        /// <summary>链路断开或失去控制权时调用：立即撤销力指令。</summary>
        public void ClearHaptic()
        {
            lock (_lock)
            {
                foreach (var d in _dev) { d.HapEnable = false; d.HapForce = 0; d.HapTorque = 0; d.HapMs = -1; }
            }
        }

        public void Dispose()
        {
            _stop = true;
            if (_thread.IsAlive && !_thread.Join(2000)) { /* 线程异常时不阻塞退出 */ }
        }

        // ================================================================ 线程主体

        private void Run()
        {
            try
            {
                while (!_stop)
                {
                    long now = _clock.ElapsedMilliseconds;
                    if (Interlocked.Exchange(ref _refreshRequested, 0) != 0)
                    {
                        int mask = 0;
                        foreach (var d in _dev)
                        {
                            if (d.Id < 0) TryOpen(d);
                            if (d.Id >= 0 && PollOne(d, now)) mask |= d.Serial == SerialA ? 1 : 2;
                            else if (d.Id >= 0) CloseDevice(d);
                        }
                        Volatile.Write(ref _refreshResultMask, mask);
                        _refreshDone.Set();
                    }
                    foreach (var d in _dev)
                    {
                        if (d.Id < 0)
                        {
                            if (now >= d.NextOpenMs)
                            {
                                d.NextOpenMs = now + OpenRetryMs;
                                TryOpen(d);
                            }
                            continue;
                        }
                        if (!PollOne(d, now)) CloseDevice(d);
                        else ApplyForce(d, now);
                    }
                    Thread.Sleep(LoopPeriodMs);
                }
            }
            catch (DllNotFoundException) { _sdkMissing = true; }
            catch (BadImageFormatException) { _sdkMissing = true; }
            catch (EntryPointNotFoundException) { _sdkMissing = true; }
            finally
            {
                _refreshDone.Set();
                foreach (var d in _dev)
                {
                    try { if (d.Id >= 0) { ZeroForce(d); CloseDevice(d); } } catch { }
                }
            }
        }

        private void TryOpen(Device d)
        {
            int id = FlCatheterNative.openDevice(d.Serial);
            if (id < 0) return;
            d.Id = id;
            if (!_servoStarted)
            {
                FlCatheterNative.startServoLoop(ServoCb, IntPtr.Zero);
                _servoStarted = true;
            }
            FlCatheterNative.enableForces(true, id);
            _openCount++;
            d.Online = true;
            d.ForceNonZero = false;
        }

        private bool PollOne(Device d, long now)
        {
            var vel = new double[2];
            var joints = new double[2];
            var enc = new int[2];
            FlCatheterNative.getEncVel(vel, d.Id);
            FlCatheterNative.getJoints(joints, d.Id);
            FlCatheterNative.getEncoders(enc, d.Id);
            FlCatheterNative.getSwitch(out byte buttons, d.Id);

            var s = new HandleSample
            {
                Buttons = buttons,
                Valid = true,
                Encoder0 = enc[0], Encoder1 = enc[1],
                Joint0 = (float)joints[0], Joint1 = (float)joints[1],
                Vel0 = (float)vel[0], Vel1 = (float)vel[1],
            };
            lock (_lock) { d.Sample = s; d.SampleMs = now; }
            return true;
        }

        private void ApplyForce(Device d, long now)
        {
            bool enable; int axis; double f, t; long hapMs;
            lock (_lock) { enable = d.HapEnable; axis = d.HapAxis; f = d.HapForce; t = d.HapTorque; hapMs = d.HapMs; }

            // 触觉帧超时（含从未收到）：力输出置 0。
            bool fresh = hapMs >= 0 && now - hapMs <= HapticTimeoutMs;
            if (enable && fresh)
            {
                var vec = new double[3];
                vec[axis] = f;
                FlCatheterNative.sendForce(vec, t, d.Id);
                d.ForceNonZero = true;
                return;
            }
            // 需要清零：状态变化时立即发送，之后每 100 ms 重发一次，防止设备保留旧值。
            if (d.ForceNonZero || now - d.LastZeroSendMs >= 100)
            {
                ZeroForce(d);
                d.ForceNonZero = false;
                d.LastZeroSendMs = now;
            }
        }

        private static void ZeroForce(Device d)
        {
            FlCatheterNative.sendForce(new double[3], 0.0, d.Id);
        }

        private void CloseDevice(Device d)
        {
            if (d.Id < 0) return;
            try { FlCatheterNative.enableForces(false, d.Id); } catch { }
            d.Id = -1;
            d.Online = false;
            lock (_lock) { d.Sample = default; d.SampleMs = -1; }
            if (_openCount > 0) _openCount--;
            if (_openCount == 0 && _servoStarted)
            {
                // 最后一个设备关闭：与原从端一致，统一停止伺服循环并关闭设备层。
                try { FlCatheterNative.stopServoLoop(); FlCatheterNative.closeDevice(); } catch { }
                _servoStarted = false;
            }
        }

        private static double Finite(double v) => double.IsNaN(v) || double.IsInfinity(v) ? 0.0 : v;
    }
}
