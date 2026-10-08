using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using MasterConsole.Protocol;

namespace MasterConsole.Services
{
    public sealed class RemoteLinkSettings
    {
        public string Host { get; set; } = "127.0.0.1";
        public int TcpPort { get; set; } = ProtocolConstants.TcpPort;
        /// <summary>预共享密钥（与从端 remote.token 内容一致）。</summary>
        public byte[] Token { get; set; }
    }

    /// <summary>
    /// 真实网络链路：TCP 命令通道（长度前缀 JSON）+ UDP 控制/状态通道，协议见 protocol/PROTOCOL.md。
    /// 手柄在主端：由 <see cref="HandleService"/> 读取本机手柄，采样随 100 Hz 控制帧上行；
    /// 从端回传的触觉帧驱动本机手柄的力输出（200 ms 无触觉帧即清零）。
    /// </summary>
    public sealed class RemoteRobotLink : IRobotLink
    {
        [DllImport("winmm.dll")] private static extern uint timeBeginPeriod(uint ms);
        [DllImport("winmm.dll")] private static extern uint timeEndPeriod(uint ms);

        private const int ControlPeriodMs = 10;      // 100 Hz
        private const int PingPeriodMs = 5000;
        private const int AcquireGraceMs = 1500;     // 申请控制权后，状态帧里的租约标志可能滞后

        private readonly RemoteLinkSettings _cfg;
        private readonly object _stateLock = new object();
        private readonly object _writeLock = new object();
        private readonly ConcurrentDictionary<int, TaskCompletionSource<CommandResult>> _pending =
            new ConcurrentDictionary<int, TaskCompletionSource<CommandResult>>();

        private TcpClient _tcp;
        private NetworkStream _stream;
        private UdpClient _udp;
        private CancellationTokenSource _cts;
        private uint _session;
        private byte[] _key;
        private uint _txSeq;
        private SeqGuard _rxGuard = new SeqGuard();          // 状态帧
        private SeqGuard _rxGuardHaptic = new SeqGuard();    // 触觉帧（与状态帧共用发送序号，各自独立判重）
        private int _nextId;
        private readonly HandleService _handles;
        private readonly DsaCanControlSender _dsaCan;
        private long _lastHapticMs = -1;

        private volatile bool _connected;
        private volatile bool _hasControl;
        private long _acquiredAtMs;
        private volatile int _inj1, _inj2, _axis4;
        private volatile StatusFlags _statusFlags;

        // 统计
        private readonly Stopwatch _clock = Stopwatch.StartNew();
        private readonly object _statsLock = new object();
        private double _rttMs = double.NaN;
        private long _lastStatusMs = -1;
        private int _statusCountWindow;
        private long _windowStartMs;
        private double _statusHz;
        private long _dropped;

        /// <param name="handles">本机手柄服务；传入后由本链路负责释放。为 null 时不上传手柄。</param>
        public RemoteRobotLink(RemoteLinkSettings cfg, HandleService handles = null, DsaCanControlSender dsaCan = null)
        {
            _cfg = cfg ?? throw new ArgumentNullException(nameof(cfg));
            _handles = handles;
            _dsaCan = dsaCan;
        }

        public string Name => "远程链路 " + _cfg.Host;
        public bool IsConnected => _connected;
        public bool HasControl => _hasControl;

        public event EventHandler ConnectionChanged;
        public event EventHandler<StatusFrame> StatusReceived;
        public event EventHandler<LogMessage> LogReceived;

        // ================================================================ 连接

        public async Task ConnectAsync()
        {
            if (_connected) return;
            if (_cfg.Token == null || _cfg.Token.Length < 16)
                throw new InvalidOperationException("密钥无效：remote.token 至少 16 个字符。");

            Log("info", $"正在连接从端 {_cfg.Host}:{_cfg.TcpPort} …");
            var tcp = new TcpClient { NoDelay = true };
            NetworkStream stream = null;
            uint session;
            byte[] key;
            int udpPort;
            try
            {
                var connect = tcp.ConnectAsync(_cfg.Host, _cfg.TcpPort);
                if (await Task.WhenAny(connect, Task.Delay(3000)) != connect)
                    throw new TimeoutException("连接从端超时。");
                await connect;
                stream = tcp.GetStream();

                byte[] nc = FrameAuth.NewNonce();
                await WriteFrameAsync(stream, CommandMessages.Hello(nc));
                var challenge = await ReadFrameAsync(stream, 5000);
                if (GetString(challenge, "t") != "challenge")
                    throw new InvalidOperationException("从端握手响应异常：" + (GetString(challenge, "reason") ?? GetString(challenge, "t")));
                byte[] ns = FrameAuth.FromHex(GetString(challenge, "nonce_s"));

                await WriteFrameAsync(stream, CommandMessages.Auth(FrameAuth.ComputeAuth(_cfg.Token, nc, ns)));
                var ack = await ReadFrameAsync(stream, 5000);
                if (GetString(ack, "t") != "hello_ack")
                    throw new InvalidOperationException("认证被拒绝：" + (GetString(ack, "reason") ?? "未知原因"));

                session = Convert.ToUInt32(ack["session"]);
                udpPort = Convert.ToInt32(ack["udp_port"]);
                key = FrameAuth.DeriveSessionKey(_cfg.Token, nc, ns);
            }
            catch
            {
                try { stream?.Close(); } catch { }
                try { tcp.Close(); } catch { }
                throw;
            }

            var udp = new UdpClient(AddressFamily.InterNetwork);
            udp.Client.ReceiveTimeout = 500;
            udp.Connect(_cfg.Host, udpPort);

            lock (_stateLock)
            {
                _tcp = tcp;
                _stream = stream;
                _udp = udp;
                _session = session;
                _key = key;
                _txSeq = 0;
                _rxGuard = new SeqGuard();
                _rxGuardHaptic = new SeqGuard();
                _lastHapticMs = -1;
                _cts = new CancellationTokenSource();
                _hasControl = false;
                _statusFlags = StatusFlags.None;
                _inj1 = _inj2 = _axis4 = 0;
                _lastStatusMs = -1;
                _statusCountWindow = 0;
                _windowStartMs = _clock.ElapsedMilliseconds;
                _statusHz = 0;
                _rttMs = double.NaN;
                Interlocked.Exchange(ref _dropped, 0);
                _connected = true;
            }
            timeBeginPeriod(1);

            try { _dsaCan?.Open(); }
            catch (Exception ex) { Log("warn", "DSA CAN 原始链路未启动：" + ex.Message); }

            var token = _cts.Token;
            StartThread("控制帧发送", () => ControlLoop(token));
            StartThread("DSA CAN 原始发送", () => DsaLoop(token));
            StartThread("状态帧接收", () => UdpReceiveLoop(token));
            StartThread("命令通道接收", () => TcpReadLoop(token));
            StartThread("链路保活", () => KeepAliveLoop(token));

            ConnectionChanged?.Invoke(this, EventArgs.Empty);
            Log("info", $"已连接从端（会话 0x{session:X8}）。");
        }

        public void Disconnect() => Shutdown("链路已断开。", "warn");

        public void Dispose()
        {
            Shutdown(null, null);
            _handles?.Dispose();
        }

        private void Shutdown(string message, string level)
        {
            bool wasConnected;
            lock (_stateLock)
            {
                wasConnected = _connected;
                _connected = false;
                _hasControl = false;
                _inj1 = _inj2 = _axis4 = 0;
                _handles?.ClearHaptic();
                try { _cts?.Cancel(); } catch { }
                try { _stream?.Close(); } catch { }
                try { _tcp?.Close(); } catch { }
                try { _udp?.Close(); } catch { }
                try { _dsaCan?.Dispose(); } catch { }
                _stream = null; _tcp = null; _udp = null;
            }
            foreach (var kv in _pending)
                if (_pending.TryRemove(kv.Key, out var tcs))
                    tcs.TrySetResult(CommandResult.Rejected("链路已断开"));
            if (!wasConnected) return;
            timeEndPeriod(1);
            ConnectionChanged?.Invoke(this, EventArgs.Empty);
            if (message != null) Log(level ?? "info", message);
        }

        // ================================================================ 命令

        public async Task<CommandResult> AcquireControlAsync()
        {
            var r = await RequestAsync(CommandMessages.Acquire, 5000);
            if (r.Ok)
            {
                _acquiredAtMs = _clock.ElapsedMilliseconds;
                _hasControl = true;
                ConnectionChanged?.Invoke(this, EventArgs.Empty);
            }
            return r;
        }

        public async Task<CommandResult> ReleaseControlAsync()
        {
            _inj1 = _inj2 = _axis4 = 0;
            var r = await RequestAsync(CommandMessages.Release, 5000);
            if (r.Ok)
            {
                _hasControl = false;
                _handles?.ClearHaptic();
                ConnectionChanged?.Invoke(this, EventArgs.Empty);
            }
            return r;
        }

        // 进入准备位置要等 PLC 运动完成，最长 5 分 30 秒；其余命令 10 秒内应有最终回执。
        public Task<CommandResult> PreparePositionAsync(double catheterMm, double wireMm)
            => RequestAsync(id => CommandMessages.PreparePosition(id, catheterMm, wireMm), 330000);

        public Task<CommandResult> StartControlAsync()
            => RequestAsync(id => CommandMessages.StartControl(id), 10000);

        public async Task<CommandResult> RefreshHandlesAsync()
        {
            int mask = _handles?.RefreshNow() ?? 0;
            if (mask < 0) return CommandResult.Rejected("刷新已在进行中");
            var result = await RequestAsync(id => CommandMessages.RefreshHandles(id, mask), 5000);
            if (!result.Ok) return result;
            return CommandResult.DoneWithReason(HandleRefreshMessage(mask));
        }

        public Task<CommandResult> SetArmManualEnableAsync(bool enabled)
            => RequestAsync(id => CommandMessages.ArmManualEnable(id, enabled), 5000);

        public Task<CommandResult> SetArmAxisEnableAsync(int axis, bool enabled)
            => RequestAsync(id => CommandMessages.ArmAxisEnable(id, axis, enabled), 5000);

        public Task<CommandResult> ResetArmAxisAsync(int axis)
            => RequestAsync(id => CommandMessages.ArmAxisReset(id, axis), 5000);

        public Task<CommandResult> SetArmCartesianJogAsync(int mode, int speedMilli)
            => RequestAsync(id => CommandMessages.ArmCartesianJog(id, mode, speedMilli), 5000);

        public Task<CommandResult> StopArmAsync()
            => RequestAsync(id => CommandMessages.ArmStop(id), 5000);

        public Task<CommandResult> ReturnArmProgramZeroAsync()
            => RequestAsync(id => CommandMessages.ArmProgramZero(id), 5000);

        public Task<CommandResult> SetArmCartesianParameterAsync(int field, int valueMilli)
            => RequestAsync(id => CommandMessages.ArmCartesianParameter(id, field, valueMilli), 5000);

        public void SetArmAxisJog(int axis, int direction)
        {
            if (!_connected || !_hasControl) direction = 0;
            if (axis < 1 || axis > 5 || direction < -1 || direction > 1) return;
            _ = RequestAsync(id => CommandMessages.ArmAxisJog(id, axis, direction), 2000);
        }

        public void KeepArmCartesianAlive()
        {
            if (!_connected || !_hasControl) return;
            _ = RequestAsync(id => CommandMessages.ArmCartesianAlive(id), 2000);
        }

        private static string HandleRefreshMessage(int mask)
            => mask == 3 ? "两只手柄已重读并建立新中立基准。"
             : mask == 1 ? "仅手柄 582 重读成功；587 保留原状态和基准。"
             : mask == 2 ? "仅手柄 587 重读成功；582 保留原状态和基准。"
             : "本次未读到有效手柄；原状态和基准已保留。";

        public Task<CommandResult> SetForceFeedbackAsync(bool enable)
            => RequestAsync(id => CommandMessages.ForceFeedback(id, enable), 15000);

        public Task<CommandResult> SetCylinderAsync(int index, bool engaged)
            => RequestAsync(id => CommandMessages.Cylinder(id, index, engaged), 10000);

        public Task<CommandResult> SetYValveClosedAsync(bool closed)
            => RequestAsync(id => CommandMessages.YValve(id, closed), 10000);

        public void SetInjector(int index, int direction)
        {
            if (!_connected || !_hasControl) direction = 0;
            direction = Math.Max(-1, Math.Min(1, direction));
            if (index == 1) _inj1 = direction;
            else if (index == 2) _inj2 = direction;
        }

        public void SetAxis4(int direction)
        {
            if (!_connected || !_hasControl) direction = 0;
            _axis4 = Math.Max(-1, Math.Min(1, direction));
        }

        public LinkStats GetStats()
        {
            long now = _clock.ElapsedMilliseconds;
            lock (_statsLock)
            {
                long window = now - _windowStartMs;
                if (window >= 1000)
                {
                    _statusHz = _statusCountWindow * 1000.0 / window;
                    _statusCountWindow = 0;
                    _windowStartMs = now;
                }
                return new LinkStats
                {
                    Session = _connected ? _session : 0,
                    RttMs = _rttMs,
                    StatusAgeMs = _lastStatusMs < 0 ? double.NaN : now - _lastStatusMs,
                    HapticAgeMs = _lastHapticMs < 0 ? double.NaN : now - _lastHapticMs,
                    StatusHz = _connected ? _statusHz : 0,
                    DroppedFrames = (ulong)Interlocked.Read(ref _dropped) + _rxGuard.Dropped + _rxGuardHaptic.Dropped,
                    Handle582Online = _handles != null && _handles.IsOnline(0),
                    Handle587Online = _handles != null && _handles.IsOnline(1),
                    HandleSdkMissing = _handles != null && _handles.SdkMissing,
                    // 从端当前 swap_handle_roles=true：导管/轴1使用物理 587（ControlFrame.HandleB）。
                    CatheterReversePressed = _handles != null && ((_handles.GetSample(1).Buttons & 0x40) != 0),
                };
            }
        }

        private async Task<CommandResult> RequestAsync(Func<int, string> build, int timeoutMs)
        {
            if (!_connected) return CommandResult.Rejected("未连接");
            int id = Interlocked.Increment(ref _nextId);
            var tcs = new TaskCompletionSource<CommandResult>(TaskCreationOptions.RunContinuationsAsynchronously);
            _pending[id] = tcs;
            try
            {
                WriteFrame(build(id));
            }
            catch (Exception ex)
            {
                _pending.TryRemove(id, out _);
                return CommandResult.Rejected("发送失败：" + ex.Message);
            }
            var finished = await Task.WhenAny(tcs.Task, Task.Delay(timeoutMs));
            if (finished != tcs.Task)
            {
                _pending.TryRemove(id, out _);
                return CommandResult.Rejected("从端无响应（超时）");
            }
            return tcs.Task.Result;
        }

        // ================================================================ 工作线程

        private void StartThread(string name, Action body)
        {
            var t = new Thread(() => body()) { IsBackground = true, Name = "RemoteLink-" + name };
            t.Start();
        }

        /// <summary>100 Hz 控制帧：本机手柄采样 + 注射器/轴4按住状态。</summary>
        private void ControlLoop(CancellationToken token)
        {
            long next = 0;
            while (!token.IsCancellationRequested)
            {
                long now = _clock.ElapsedMilliseconds;
                if (now < next)
                {
                    Thread.Sleep((int)Math.Max(1, next - now));
                    continue;
                }
                next = now + ControlPeriodMs;

                var frame = new ControlFrame
                {
                    HandleA = _handles != null ? _handles.GetSample(0) : default,
                    HandleB = _handles != null ? _handles.GetSample(1) : default,
                    Injector1Dir = (sbyte)_inj1,
                    Injector2Dir = (sbyte)_inj2,
                    Axis4Dir = (sbyte)_axis4,
                };
                var header = new FrameHeader
                {
                    Session = _session,
                    Seq = ++_txSeq,
                    TsMs = unchecked((uint)_clock.ElapsedMilliseconds),
                };
                try
                {
                    byte[] buf = FrameCodec.EncodeControl(header, frame, _key);
                    _udp.Send(buf, buf.Length);
                }
                catch (SocketException) { /* 对端暂不可达：下一拍继续 */ }
                catch (ObjectDisposedException) { break; }
                catch (NullReferenceException) { break; }
            }
        }

        private void DsaLoop(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                if (DsaControlReady())
                {
                    try { _dsaCan?.PollAndSend(); }
                    catch (Exception ex) { Log("warn", "DSA CAN 原始采样失败：" + ex.Message); break; }
                }
                if (token.WaitHandle.WaitOne(20)) break;
            }
        }

        private bool DsaControlReady()
        {
            lock (_statsLock)
            {
                if (_lastStatusMs < 0 || _clock.ElapsedMilliseconds - _lastStatusMs >= ProtocolConstants.StatusTimeoutMs)
                    return false;
            }
            StatusFlags flags = _statusFlags;
            return _connected && _hasControl &&
                   (flags & StatusFlags.ControlActive) != 0 &&
                   (flags & (StatusFlags.EstopHold | StatusFlags.HostCommTimeout)) == 0;
        }

        private void UdpReceiveLoop(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                byte[] data;
                try
                {
                    var ep = new IPEndPoint(IPAddress.Any, 0);
                    data = _udp.Receive(ref ep);
                }
                catch (SocketException e)
                {
                    if (e.SocketErrorCode == SocketError.TimedOut || e.SocketErrorCode == SocketError.ConnectionReset) continue;
                    break;
                }
                catch (ObjectDisposedException) { break; }
                catch (NullReferenceException) { break; }

                // 触觉帧（从端 → 主端手柄力输出）：按帧长和类型字节分流。
                if (data.Length == ProtocolConstants.HapticFrameLen && data[3] == ProtocolConstants.TypeHaptic)
                {
                    if (!FrameCodec.TryDecodeHaptic(data, data.Length, _session, _key, out var hh, out var hap, out _))
                    {
                        Interlocked.Increment(ref _dropped);
                        continue;
                    }
                    if (!_rxGuardHaptic.Accept(hh.Seq)) continue;
                    lock (_statsLock) { _lastHapticMs = _clock.ElapsedMilliseconds; }
                    _handles?.SetHaptic(0, hap.HandleA);
                    _handles?.SetHaptic(1, hap.HandleB);
                    continue;
                }

                if (!FrameCodec.TryDecodeStatus(data, data.Length, _session, _key, out var h, out var st, out _))
                {
                    Interlocked.Increment(ref _dropped);
                    continue;
                }
                if (!_rxGuard.Accept(h.Seq)) continue;

                long now = _clock.ElapsedMilliseconds;
                lock (_statsLock)
                {
                    _lastStatusMs = now;
                    _statusCountWindow++;
                    if (st.EchoTsMs != 0)
                    {
                        uint rtt = unchecked((uint)now - st.EchoTsMs - st.HoldMs);
                        if (rtt < 10000)
                            _rttMs = double.IsNaN(_rttMs) ? rtt : _rttMs * 0.8 + rtt * 0.2;
                    }
                }
                _statusFlags = st.Flags;

                // 控制权以从端为准：从端因超时等原因收回时，主端同步更新。
                bool lease = st.Has(StatusFlags.LeaseHeld);
                if (_hasControl && !lease && now - _acquiredAtMs > AcquireGraceMs)
                {
                    _hasControl = false;
                    _inj1 = _inj2 = _axis4 = 0;
                    _handles?.ClearHaptic();
                    Log("warn", "从端已收回控制权，请重新申请。");
                    ConnectionChanged?.Invoke(this, EventArgs.Empty);
                }
                StatusReceived?.Invoke(this, st);
            }
        }

        private void TcpReadLoop(CancellationToken token)
        {
            try
            {
                while (!token.IsCancellationRequested)
                {
                    var msg = ReadFrameBlocking(_stream);
                    if (msg == null) break;
                    Dispatch(msg);
                }
            }
            catch (Exception) { /* 连接被关闭或读取失败，统一按断线处理 */ }
            if (!token.IsCancellationRequested) Shutdown("与从端的命令通道已断开。", "error");
        }

        private void Dispatch(System.Collections.Generic.Dictionary<string, object> m)
        {
            string type = GetString(m, "t");
            switch (type)
            {
                case "ack":
                {
                    int id = Convert.ToInt32(m["id"]);
                    string state = GetString(m, "state");
                    if (state == "accepted") return; // 已接收，等最终回执
                    if (_pending.TryRemove(id, out var tcs))
                        tcs.TrySetResult(state == "done" ? CommandResult.Done() : CommandResult.Rejected(GetString(m, "reason") ?? "被从端拒绝"));
                    break;
                }
                case "event":
                    Log(GetString(m, "level") ?? "info", GetString(m, "text") ?? "");
                    break;
                case "error":
                    Log("error", "从端报错：" + (GetString(m, "reason") ?? ""));
                    break;
                // pong 等：忽略
            }
        }

        private void KeepAliveLoop(CancellationToken token)
        {
            long lastPing = 0;
            while (!token.IsCancellationRequested)
            {
                Thread.Sleep(200);
                long now = _clock.ElapsedMilliseconds;
                if (now - lastPing >= PingPeriodMs)
                {
                    lastPing = now;
                    try { WriteFrame(CommandMessages.Ping(Interlocked.Increment(ref _nextId))); }
                    catch (Exception) { break; }
                }
            }
        }

        // ================================================================ 帧读写

        private void WriteFrame(string json)
        {
            var stream = _stream ?? throw new IOException("命令通道未连接");
            byte[] body = Encoding.UTF8.GetBytes(json);
            byte[] frame = new byte[4 + body.Length];
            BitConverter.GetBytes((uint)body.Length).CopyTo(frame, 0); // 小端
            Buffer.BlockCopy(body, 0, frame, 4, body.Length);
            lock (_writeLock) stream.Write(frame, 0, frame.Length);
        }

        private static async Task WriteFrameAsync(NetworkStream stream, string json)
        {
            byte[] body = Encoding.UTF8.GetBytes(json);
            byte[] frame = new byte[4 + body.Length];
            BitConverter.GetBytes((uint)body.Length).CopyTo(frame, 0);
            Buffer.BlockCopy(body, 0, frame, 4, body.Length);
            await stream.WriteAsync(frame, 0, frame.Length);
        }

        private static async Task<System.Collections.Generic.Dictionary<string, object>> ReadFrameAsync(NetworkStream stream, int timeoutMs)
        {
            var task = ReadFrameCoreAsync(stream);
            if (await Task.WhenAny(task, Task.Delay(timeoutMs)) != task)
                throw new TimeoutException("等待从端响应超时。");
            return await task;
        }

        private static async Task<System.Collections.Generic.Dictionary<string, object>> ReadFrameCoreAsync(NetworkStream stream)
        {
            byte[] head = new byte[4];
            if (!await ReadExactAsync(stream, head, 4)) throw new IOException("从端关闭了连接。");
            uint len = BitConverter.ToUInt32(head, 0);
            if (len == 0 || len > 65536) throw new IOException("从端返回的帧长度无效。");
            byte[] body = new byte[len];
            if (!await ReadExactAsync(stream, body, (int)len)) throw new IOException("从端关闭了连接。");
            return CommandMessages.Parse(Encoding.UTF8.GetString(body));
        }

        private static async Task<bool> ReadExactAsync(NetworkStream stream, byte[] buf, int count)
        {
            int got = 0;
            while (got < count)
            {
                int n = await stream.ReadAsync(buf, got, count - got);
                if (n <= 0) return false;
                got += n;
            }
            return true;
        }

        private static System.Collections.Generic.Dictionary<string, object> ReadFrameBlocking(NetworkStream stream)
        {
            byte[] head = new byte[4];
            if (!ReadExact(stream, head, 4)) return null;
            uint len = BitConverter.ToUInt32(head, 0);
            if (len == 0 || len > 65536) return null;
            byte[] body = new byte[len];
            if (!ReadExact(stream, body, (int)len)) return null;
            return CommandMessages.Parse(Encoding.UTF8.GetString(body));
        }

        private static bool ReadExact(NetworkStream stream, byte[] buf, int count)
        {
            int got = 0;
            while (got < count)
            {
                int n = stream.Read(buf, got, count - got);
                if (n <= 0) return false;
                got += n;
            }
            return true;
        }

        private static string GetString(System.Collections.Generic.Dictionary<string, object> m, string key)
            => m != null && m.TryGetValue(key, out var v) ? v as string : null;

        private void Log(string level, string text)
            => LogReceived?.Invoke(this, new LogMessage { Level = level, Text = text });
    }
}
