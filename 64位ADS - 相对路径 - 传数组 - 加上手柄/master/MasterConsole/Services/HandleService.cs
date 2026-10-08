using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
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
        private const int RefreshOpenAttempts = 3;     // 同一次刷新只重试失败设备
        private const long SampleMaxAgeMs = 100;      // 采样超过此龄期则对外标记无效
        private const long HapticTimeoutMs = ProtocolConstants.HapticTimeoutMs;

        // SDK 伺服循环回调：与原从端 SyncUpdate 相同，只返回设备状态。必须保持引用，防止被回收。
        private static readonly FlCatheterNative.ServoCallback ServoCb = _ => FlCatheterNative.deviceStatus(0);
        private static bool _servoStarted;
        private static int _openCount;

        private sealed class Device
        {
            public uint Serial;
            public int OpenAttempts;
            public string RefreshStatus = "未尝试";
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
        private TaskCompletionSource<int> _refresh;
        private volatile bool _stop;
        private volatile bool _sdkMissing;
        private int _refreshRequested;
        private long _refreshDeadline;
        private volatile bool _inputPaused;
        private string _lastError = "";

        public HandleService()
        {
            _thread = new Thread(Run) { IsBackground = true, Name = "HandleService", Priority = ThreadPriority.AboveNormal };
            _thread.Start();
        }

        /// <summary>手柄 SDK（FLCatheter.dll）无法加载时为 true。</summary>
        public bool SdkMissing => _sdkMissing;

        public bool IsOnline(int slot) { lock (_lock) return slot >= 0 && slot < 2 && _dev[slot].Online; }
        public string LastError { get { lock (_lock) return _lastError; } }
        public string RefreshDetails
        {
            get { lock (_lock) return $"582：{_dev[0].RefreshStatus}（打开 {_dev[0].OpenAttempts} 次）；587：{_dev[1].RefreshStatus}（打开 {_dev[1].OpenAttempts} 次）。" + _lastError; }
        }
        public bool RefreshInProgress { get { lock (_lock) return _refresh != null && !_refresh.Task.IsCompleted; } }

        /// <summary>整体重启共享 SDK；bit0=582、bit1=587，-1=忙、-2=超时。超时不取消仍在执行的 SDK 调用。</summary>
        public int RefreshNow(int timeoutMs = 5000)
        {
            if (timeoutMs <= 0) return -2;
            long deadline = _clock.ElapsedMilliseconds + timeoutMs;
            TaskCompletionSource<int> request;
            lock (_lock)
            {
                if (_refresh != null && !_refresh.Task.IsCompleted) return -1;
                if (_stop || !_thread.IsAlive) return 0;
                PauseInput();
                foreach (var d in _dev) { d.Sample = default; d.SampleMs = -1; d.Online = false; d.OpenAttempts = 0; d.RefreshStatus = "未尝试"; }
                _lastError = "";
                _refreshDeadline = deadline;
                request = _refresh = new TaskCompletionSource<int>(TaskCreationOptions.RunContinuationsAsynchronously);
                Interlocked.Exchange(ref _refreshRequested, 1);
            }
            int remaining = (int)Math.Max(0, deadline - _clock.ElapsedMilliseconds);
            return request.Task.Wait(remaining) ? request.Task.Result : -2;
        }

        public void PauseInput()
        {
            lock (_lock) { _inputPaused = true; ClearHaptic(); }
        }

        // 仅在从端确认新采样基准重建后调用；失败/超时保持暂停。
        public void ResumeInput()
        {
            lock (_lock)
            {
                if (_refresh != null && (!_refresh.Task.IsCompleted || _refresh.Task.Result != 3)) return;
                ClearHaptic();
                _inputPaused = false;
            }
        }

        /// <summary>取最新采样；打开失败、读取失败或采样过期时 Valid=false。</summary>
        public HandleSample GetSample(int slot, bool forRefresh = false)
        {
            var d = _dev[slot];
            lock (_lock)
            {
                var s = d.Sample;
                if ((_inputPaused && !forRefresh) || (_refresh != null && (!_refresh.Task.IsCompleted || _refresh.Task.Result <= 0)) ||
                    d.SampleMs < 0 || _clock.ElapsedMilliseconds - d.SampleMs > SampleMaxAgeMs) s.Valid = false;
                return s;
            }
        }

        /// <summary>收到从端触觉帧时调用：记录力指令，由手柄线程执行。</summary>
        public void SetHaptic(int slot, HapticOut o)
        {
            var d = _dev[slot];
            lock (_lock)
            {
                if (_inputPaused) return;
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
                    try
                    {
                        if (Interlocked.Exchange(ref _refreshRequested, 0) != 0)
                        {
                            int mask = 0;
                            try { mask = RefreshDevices(); }
                            catch (TimeoutException ex) { mask = -2; SetError(ex.Message); }
                            catch (Exception ex) { SetError(ex.Message); }
                            finally
                            {
                                lock (_lock)
                                {
                                    if (mask <= 0) foreach (var d in _dev) { d.Sample = default; d.SampleMs = -1; d.Online = false; }
                                    _refresh.TrySetResult(mask);
                                }
                            }
                        }
                        // 失败/超时不再后台重开或启动；成功采样继续更新，供从端确认使用。
                        if (_inputPaused && (_refresh == null || !_refresh.Task.IsCompleted || _refresh.Task.Result <= 0 ||
                            (!_dev[0].Online && !_dev[1].Online)))
                        {
                            // 等待从端保持确认期间也必须撤销已写入的力；不能等到 SDK 重启才清零。
                            foreach (var d in _dev)
                                if (d.Id >= 0 && d.ForceNonZero) { ZeroForce(d); d.ForceNonZero = false; }
                            Thread.Sleep(LoopPeriodMs);
                            continue;
                        }
                        FlCatheterNative.Load();
                        _sdkMissing = false;
                        long now = _clock.ElapsedMilliseconds;
                        foreach (var d in _dev)
                        {
                            if (!_inputPaused && d.Id < 0 && now >= d.NextOpenMs)
                            {
                                d.NextOpenMs = now + OpenRetryMs;
                                TryOpen(d);
                            }
                        }
                        if (_openCount > 0 && !_servoStarted && !StartServo()) { PauseInput(); CloseAll(); }
                        foreach (var d in _dev)
                        {
                            if (d.Id < 0 || !_servoStarted) continue;
                            if (!PollOne(d, now)) CloseDevice(d);
                            else ApplyForce(d, now);
                        }
                    }
                    catch (Exception ex)
                    {
                        PauseInput();
                        _sdkMissing = !FlCatheterNative.IsLoaded;
                        SetError(ex.Message);
                        lock (_lock) foreach (var d in _dev) { d.Online = false; d.Sample = default; d.SampleMs = -1; }
                        // 保留线程，修复 DLL 文件后下一次刷新仍可重新加载。
                    }
                    Thread.Sleep(LoopPeriodMs);
                }
            }
            finally
            {
                try { CloseAll(); } catch { }
                lock (_lock) _refresh?.TrySetResult(0);
            }
        }

        private void CheckRefreshBudget(string stage)
        {
            if (_stop || _clock.ElapsedMilliseconds >= _refreshDeadline)
                throw new TimeoutException(stage + "阶段超时/停止；输入保持暂停");
        }

        private int RefreshDevices()
        {
            CheckRefreshBudget("停止");
            if (!CloseAll()) return 0;
            CheckRefreshBudget("停止/关闭");
            // 已使用 SDK 的线程退出无法确认时，Unload 拒绝执行；不可绕过此保护继续打开。
            FlCatheterNative.Unload();
            CheckRefreshBudget("卸载");
            try { FlCatheterNative.Load(); _sdkMissing = false; }
            catch { _sdkMissing = true; throw; }
            CheckRefreshBudget("加载");
            for (int attempt = 0; attempt < RefreshOpenAttempts; ++attempt)
            {
                foreach (var d in _dev)
                {
                    if (d.Id >= 0) continue;
                    CheckRefreshBudget("打开");
                    lock (_lock) ++d.OpenAttempts;
                    TryOpen(d);
                    CheckRefreshBudget("打开");
                }
                if (_openCount == 2 || attempt + 1 == RefreshOpenAttempts) break;
                Thread.Sleep((int)Math.Min(OpenRetryMs, Math.Max(0, _refreshDeadline - _clock.ElapsedMilliseconds)));
            }
            if (_openCount == 0) return 0;
            CheckRefreshBudget("伺服启动");
            if (!StartServo()) { CloseAll(); return 0; }
            CheckRefreshBudget("伺服启动");
            while (true)
            {
                CheckRefreshBudget("采样");
                int mask = 0;
                foreach (var d in _dev)
                {
                    if (d.Id < 0) continue;
                    try
                    {
                        if (PollOne(d, _clock.ElapsedMilliseconds))
                        { mask |= d.Serial == SerialA ? 1 : 2; SetDeviceStatus(d, "有效新采样"); }
                        else SetDeviceStatus(d, "采样无效（设备身份、伺服状态或数值检查未通过）");
                    }
                    catch (Exception ex) { SetDeviceStatus(d, "采样异常：" + ex.Message); }
                    CheckRefreshBudget("采样");
                }
                if (((mask & 1) != 0 ? 1 : 0) + ((mask & 2) != 0 ? 1 : 0) == _openCount) return mask;
                Thread.Sleep(LoopPeriodMs);
            }
        }

        private void TryOpen(Device d)
        {
            int id;
            try { id = FlCatheterNative.openDevice(d.Serial); }
            catch (Exception ex) { SetDeviceStatus(d, "打开异常：" + ex.Message); return; }
            if (id < 0) { SetDeviceStatus(d, $"打开失败（{id}）"); return; }
            SetDeviceStatus(d, "已打开，等待伺服及采样");
            d.Id = id;
            _openCount++;
            if (_servoStarted) FlCatheterNative.enableForces(true, id);
            d.ForceNonZero = false;
        }

        private bool StartServo()
        {
            try
            {
                if (_servoStarted) return FlCatheterNative.isServoLoopRunning();
                if (_openCount == 0) return false;
                // 当前随项目发布的 SDK：start/stop 返回 0 成功，负数失败；同时核对真实运行状态。
                int result = FlCatheterNative.startServoLoop(ServoCb, IntPtr.Zero);
                _servoStarted = result == 0 && FlCatheterNative.isServoLoopRunning();
                if (!_servoStarted) { SetError($"伺服启动失败（{result}）"); return false; }
                foreach (var d in _dev) if (d.Id >= 0) FlCatheterNative.enableForces(true, d.Id);
                return true;
            }
            catch (Exception ex) { SetError("伺服启动/启力异常：" + ex.Message); return false; }
        }

        private bool PollOne(Device d, long now)
        {
            if (!FlCatheterNative.isServoLoopRunning() || FlCatheterNative.getSerialNumber(d.Id) != d.Serial)
                return false;
            var vel = new double[2];
            var joints = new double[2];
            var enc = new int[2];
            FlCatheterNative.getEncVel(vel, d.Id);
            FlCatheterNative.getJoints(joints, d.Id);
            FlCatheterNative.getEncoders(enc, d.Id);
            FlCatheterNative.getSwitch(out byte buttons, d.Id);
            // 读取接口返回 void；只验证 SDK 可提供的身份、伺服状态及数值有效性，不以静止判失效。
            for (int i = 0; i < 2; ++i)
                if (!IsFinite(vel[i]) || !IsFinite(joints[i]) || Math.Abs(vel[i]) > float.MaxValue || Math.Abs(joints[i]) > float.MaxValue)
                    return false;

            var s = new HandleSample
            {
                Buttons = buttons,
                Valid = true,
                Encoder0 = enc[0], Encoder1 = enc[1],
                Joint0 = (float)joints[0], Joint1 = (float)joints[1],
                Vel0 = (float)vel[0], Vel1 = (float)vel[1],
            };
            lock (_lock) { d.Sample = s; d.SampleMs = _clock.ElapsedMilliseconds; d.Online = true; }
            return true;
        }

        private void ApplyForce(Device d, long now)
        {
            bool enable; int axis; double f, t; long hapMs;
            lock (_lock) { enable = !_inputPaused && d.HapEnable; axis = d.HapAxis; f = d.HapForce; t = d.HapTorque; hapMs = d.HapMs; }

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
            lock (_lock) { d.Online = false; d.Sample = default; d.SampleMs = -1; }
            ZeroForce(d);
            FlCatheterNative.enableForces(false, d.Id);
            d.Id = -1;
            d.Online = false;
            lock (_lock) { d.Sample = default; d.SampleMs = -1; }
            if (_openCount > 0) _openCount--;
            if (_openCount == 0) CloseAll();
        }

        private bool CloseAll()
        {
            ClearHaptic();
            if (!FlCatheterNative.IsLoaded) return true;
            bool ok = true;
            foreach (var d in _dev)
            {
                lock (_lock) { d.Online = false; d.Sample = default; d.SampleMs = -1; }
                if (d.Id < 0) continue;
                try { ZeroForce(d); d.ForceNonZero = false; FlCatheterNative.enableForces(false, d.Id); }
                catch (Exception ex) { ok = false; SetDeviceStatus(d, "清零/禁力失败：" + ex.Message); }
            }
            int result;
            try { result = FlCatheterNative.stopServoLoop(); _servoStarted = FlCatheterNative.isServoLoopRunning(); }
            catch (Exception ex) { SetError("停止异常：" + ex.Message); return false; }
            if (result != 0 || _servoStarted)
            {
                PauseInput();
                SetError($"伺服停止失败（{result}，运行状态 {_servoStarted}），未关闭/卸载设备");
                return false;
            }
            try { FlCatheterNative.closeDevice(); }
            catch (Exception ex) { SetError("关闭异常：" + ex.Message); return false; }
            foreach (var d in _dev)
            {
                d.Id = -1;
                d.NextOpenMs = _clock.ElapsedMilliseconds + OpenRetryMs;
                d.ForceNonZero = false;
                d.LastZeroSendMs = 0;
            }
            _openCount = 0;
            return ok;
        }

        private void SetDeviceStatus(Device d, string status) { lock (_lock) d.RefreshStatus = status; }
        private void SetError(string error)
        { lock (_lock) { if (!_lastError.Contains(error)) _lastError += (_lastError.Length == 0 ? "" : "；") + error; } }
        private static bool IsFinite(double v) => !double.IsNaN(v) && !double.IsInfinity(v);
        private static double Finite(double v) => double.IsNaN(v) || double.IsInfinity(v) ? 0.0 : v;
    }
}
