using System;
using System.Threading.Tasks;
using System.Windows.Threading;
using MasterConsole.Protocol;

namespace MasterConsole.Services
{
    /// <summary>
    /// 模拟链路：不连接任何真实设备，用于独立开发/演示主端界面。
    /// 电缸数值语义与协议一致：电缸 1/3 打开=2000，电缸 2/4 打开=10。
    /// 注意：这里的行为只是演示，不代表从端的真实时序。
    /// </summary>
    public sealed class SimulatedRobotLink : IRobotLink
    {
        private readonly DispatcherTimer _timer;
        private readonly Random _rng = new Random(1);
        private readonly bool[] _engaged = new bool[4];
        private readonly int[] _injectorDir = new int[2];
        private int _axis4Dir;
        private uint _seq;
        private DateTime _t0 = DateTime.Now;

        private bool _connected;
        private bool _hasControl;
        private bool _ffEnabled;
        private bool _ffZeroing;
        private bool _calZeroed;
        private bool _yClosed;
        private bool _selfCheckDone;
        private bool _controlStarted;
        private int _selfCheckStatus; // 1 等待 2 运行中

        public SimulatedRobotLink()
        {
            _timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(66) };
            _timer.Tick += (s, e) => PublishStatus();
        }

        public string Name => "模拟链路";
        public bool IsConnected => _connected;
        public bool HasControl => _hasControl;

        public event EventHandler ConnectionChanged;
        public event EventHandler<StatusFrame> StatusReceived;
        public event EventHandler<LogMessage> LogReceived;

        public async Task ConnectAsync()
        {
            Log("info", "正在连接从端（模拟）…");
            await Task.Delay(400);
            _connected = true;
            _t0 = DateTime.Now;
            _timer.Start();
            ConnectionChanged?.Invoke(this, EventArgs.Empty);
            Log("info", "已连接（模拟链路，不会驱动任何真实设备）。");
        }

        public void Disconnect()
        {
            if (!_connected) return;
            _timer.Stop();
            _connected = false;
            _hasControl = false;
            _controlStarted = false;
            _ffEnabled = _ffZeroing = false;
            _injectorDir[0] = _injectorDir[1] = 0;
            _axis4Dir = 0;
            ConnectionChanged?.Invoke(this, EventArgs.Empty);
            Log("warn", "链路已断开。");
        }

        public async Task<CommandResult> AcquireControlAsync()
        {
            await Task.Delay(120);
            if (!_connected) return CommandResult.Rejected("未连接");
            _hasControl = true;
            ConnectionChanged?.Invoke(this, EventArgs.Empty);
            Log("info", "已获得控制权。");
            return CommandResult.Done();
        }

        public async Task<CommandResult> ReleaseControlAsync()
        {
            await Task.Delay(80);
            _hasControl = false;
            _controlStarted = false;
            _injectorDir[0] = _injectorDir[1] = 0;
            _axis4Dir = 0;
            ConnectionChanged?.Invoke(this, EventArgs.Empty);
            Log("info", "已释放控制权。");
            return CommandResult.Done();
        }

        public async Task<CommandResult> PreparePositionAsync(double catheterMm, double wireMm)
        {
            var guard = Guard();
            if (guard != null) return guard;
            await Task.Delay(150);
            _selfCheckDone = false;
            _selfCheckStatus = 2;
            Log("info", $"已接收：进入器械准备位置（导管 {catheterMm:0.#} mm，Y阀及导丝 {wireMm:0.#} mm）。");
            _ = FinishPrepareAsync();
            return CommandResult.Done();
        }

        private async Task FinishPrepareAsync()
        {
            await Task.Delay(3000);
            if (!_connected) return;
            _selfCheckDone = true;
            _selfCheckStatus = 0;
            Log("info", "已到达器械准备位置（模拟）。");
        }

        public async Task<CommandResult> SetForceFeedbackAsync(bool enable)
        {
            var guard = Guard();
            if (guard != null) return guard;
            await Task.Delay(100);
            if (!enable)
            {
                _ffEnabled = false;
                _ffZeroing = false;
                Log("info", "力反馈已关闭。");
                return CommandResult.Done();
            }
            _ffZeroing = true;
            Log("info", "力反馈开启前自动零点采集…");
            _ = FinishZeroingAsync();
            return CommandResult.Done();
        }

        private async Task FinishZeroingAsync()
        {
            await Task.Delay(1500);
            if (!_connected || !_ffZeroing) return;
            _ffZeroing = false;
            _calZeroed = true;
            _ffEnabled = true;
            Log("info", "零点采集完成，力反馈已开启。");
        }

        public async Task<CommandResult> SetCylinderAsync(int index, bool engaged)
        {
            var guard = Guard();
            if (guard != null) return guard;
            if (index < 1 || index > 4) return CommandResult.Rejected("电缸编号无效");
            await Task.Delay(100);
            _engaged[index - 1] = engaged;
            Log("info", $"电缸 {index} {(engaged ? "已打开" : "已恢复原状态")}。");
            return CommandResult.Done();
        }

        public async Task<CommandResult> StartControlAsync()
        {
            var guard = Guard();
            if (guard != null) return guard;
            await Task.Delay(100);
            if (!_selfCheckDone) return CommandResult.Rejected("尚未到达器械准备位置，请先点击“进入器械准备位置”");
            _controlStarted = true;
            Log("info", "已在当前位置开始控制（模拟）。");
            return CommandResult.Done();
        }

        public async Task<CommandResult> SetYValveClosedAsync(bool closed)
        {
            var guard = Guard();
            if (guard != null) return guard;
            await Task.Delay(100);
            _yClosed = closed;
            Log("info", closed ? "Y 阀已关闭。" : "Y 阀已打开。");
            return CommandResult.Done();
        }

        public void SetInjector(int index, int direction)
        {
            if (index < 1 || index > 2) return;
            if (!_connected || !_hasControl) direction = 0;
            _injectorDir[index - 1] = Math.Max(-1, Math.Min(1, direction));
        }

        public void SetAxis4(int direction)
        {
            if (!_connected || !_hasControl) direction = 0;
            _axis4Dir = Math.Max(-1, Math.Min(1, direction));
        }

        public LinkStats GetStats() => new LinkStats
        {
            Session = 0xA1B2C3D4,
            RttMs = 26 + _rng.NextDouble() * 6,
            StatusAgeMs = _connected ? 66 : double.NaN,
            HapticAgeMs = _connected ? 10 : double.NaN,
            StatusHz = _connected ? 15 : 0,
            DroppedFrames = 0,
        };

        public void Dispose()
        {
            _timer.Stop();
        }

        // ------------------------------------------------------------ 内部

        private CommandResult Guard()
        {
            if (!_connected) return CommandResult.Rejected("未连接");
            if (!_hasControl) return CommandResult.Rejected("未持有控制权");
            return null;
        }

        private void PublishStatus()
        {
            double t = (DateTime.Now - _t0).TotalSeconds;
            var flags = StatusFlags.AdsHealthy;
            if (_hasControl) flags |= StatusFlags.LeaseHeld;
            if (_hasControl && _controlStarted && _selfCheckDone) flags |= StatusFlags.ControlActive;
            if (_ffEnabled) flags |= StatusFlags.FfEnabled;
            if (_ffZeroing) flags |= StatusFlags.FfZeroing;
            if (_calZeroed) flags |= StatusFlags.CalZeroed;
            if (_yClosed) flags |= StatusFlags.YValveClosed;
            if (_selfCheckDone) flags |= StatusFlags.SelfCheckDone | StatusFlags.StartupCompleted;

            byte mask = 0;
            for (int i = 0; i < 4; i++) if (_engaged[i]) mask |= (byte)(1 << i);

            float wave = _ffEnabled ? (float)Math.Sin(t * 1.3) : 0f;
            var frame = new StatusFrame
            {
                Flags = flags,
                Mode = ((int)(t / 20)) % 2,
                Phase = _selfCheckDone ? 1 : 0,
                SelfCheckStatus = _selfCheckStatus,
                AdsState = 2,
                // 原状态（演示）：电缸 1/3 为 0，电缸 2/4 为 2000；打开：1/3=2000，2/4=10。
                Cylinder1 = (ushort)(_engaged[0] ? 2000 : 0),
                Cylinder2 = (ushort)(_engaged[1] ? 10 : 2000),
                Cylinder3 = (ushort)(_engaged[2] ? 2000 : 0),
                Cylinder4 = (ushort)(_engaged[3] ? 10 : 2000),
                CylinderManualMask = mask,
                InjectorActive1 = (sbyte)_injectorDir[0],
                InjectorActive2 = (sbyte)_injectorDir[1],
                Axis4Active = (sbyte)_axis4Dir,
                Force582F = 0.30f * wave + Noise(),
                Force582N = 0.012f * wave + Noise() * 0.05f,
                Force587F = 0.22f * (float)Math.Sin(t * 0.9) * (_ffEnabled ? 1 : 0) + Noise(),
                Force587N = 0.009f * wave + Noise() * 0.05f,
                CleanForceN = 0.25f * wave,
                AdsActualHz = 99.6f,
                AxisPos = new float[7],
                AxisFromLeft = new float[7],
            };
            _seq++;
            StatusReceived?.Invoke(this, frame);
        }

        private float Noise() => (float)((_rng.NextDouble() - 0.5) * 0.01);

        private void Log(string level, string text)
            => LogReceived?.Invoke(this, new LogMessage { Level = level, Text = text });
    }
}
