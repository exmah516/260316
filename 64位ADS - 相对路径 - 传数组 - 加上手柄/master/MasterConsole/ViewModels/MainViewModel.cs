using System;
using System.Collections.ObjectModel;
using System.Globalization;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Threading;
using MasterConsole.Protocol;
using MasterConsole.Services;

namespace MasterConsole.ViewModels
{
    /// <summary>电缸 1–4 的一个按钮项。电缸 1/3 打开值=2000，电缸 2/4 打开值=10。</summary>
    public sealed class CylinderItem : ObservableObject
    {
        private bool _engaged;
        private ushort _currentValue;

        public CylinderItem(int index)
        {
            Index = index;
            OpenValue = (index == 1 || index == 3) ? 2000 : 10;
        }

        public int Index { get; }
        public int OpenValue { get; }
        public string Title => $"电缸 {Index}";
        public string Caption => $"打开 = {OpenValue}";

        /// <summary>只认从端回报的手动覆盖状态（与旧 AdsControlUI 一致）。</summary>
        public bool Engaged { get => _engaged; set => SetField(ref _engaged, value); }
        public ushort CurrentValue { get => _currentValue; set { if (SetField(ref _currentValue, value)) OnPropertyChanged(nameof(CurrentText)); } }
        public string CurrentText => $"当前值 {CurrentValue}";

        /// <summary>命令被拒绝或完成后强制按钮重新读取状态，避免本地点击后显示与真实状态不一致。</summary>
        public void Refresh() => OnPropertyChanged(nameof(Engaged));
    }

    public sealed class LogItem
    {
        public string Time { get; set; }
        public string Level { get; set; }
        public string Text { get; set; }
    }

    public sealed class MainViewModel : ObservableObject, IDisposable
    {
        private const int MaxLogItems = 300;

        private readonly IRobotLink _link;
        private readonly Dispatcher _ui;
        private readonly DispatcherTimer _statsTimer;
        private StatusFrame _s;
        private bool _hasStatus;
        private DateTime _lastStatusUtc = DateTime.MinValue;
        private bool _connecting;

        private string _catheterText = "28";
        private string _wireText = "610";
        private string _prepareError = "";
        private string _linkStatsText = "";
        private LinkStats _stats = new LinkStats();

        public MainViewModel(IRobotLink link)
        {
            _link = link;
            _ui = Dispatcher.CurrentDispatcher;

            Cylinders = new ObservableCollection<CylinderItem>
            {
                new CylinderItem(1), new CylinderItem(2), new CylinderItem(3), new CylinderItem(4),
            };

            ConnectCommand = new RelayCommand(ToggleConnect);
            AcquireCommand = new RelayCommand(ToggleControl);
            PrepareCommand = new RelayCommand(Prepare);
            ForceFeedbackCommand = new RelayCommand(ToggleForceFeedback);
            CylinderCommand = new RelayCommand(p => ToggleCylinder(p));
            YValveCommand = new RelayCommand(ToggleYValve);

            _link.ConnectionChanged += (s, e) => _ui.BeginInvoke(new Action(RaiseConnectionProps));
            _link.StatusReceived += (s, f) => _ui.BeginInvoke(new Action(() => OnStatus(f)));
            _link.LogReceived += (s, m) => _ui.BeginInvoke(new Action(() => AddLog(m.Level, m.Text, m.Time)));

            _statsTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(250) };
            _statsTimer.Tick += (s, e) => RefreshStats();
            _statsTimer.Start();

            AddLog("info", "主端控制台已启动。当前使用：" + _link.Name + "。");
        }

        // ============================================================ 集合与命令

        public ObservableCollection<CylinderItem> Cylinders { get; }
        public ObservableCollection<LogItem> Log { get; } = new ObservableCollection<LogItem>();

        public ICommand ConnectCommand { get; }
        public ICommand AcquireCommand { get; }
        public ICommand PrepareCommand { get; }
        public ICommand ForceFeedbackCommand { get; }
        public ICommand CylinderCommand { get; }
        public ICommand YValveCommand { get; }

        // ============================================================ 连接与控制权

        public string LinkName => _link.Name;
        public bool IsConnected => _link.IsConnected;
        public bool HasControl => _link.HasControl;
        public bool StatusFresh => _hasStatus && (DateTime.UtcNow - _lastStatusUtc).TotalMilliseconds < ProtocolConstants.StatusTimeoutMs;

        /// <summary>所有运动/动作类按钮的总开关：已连接 + 持有控制权 + 状态数据新鲜。</summary>
        public bool CanOperate => IsConnected && HasControl && StatusFresh;

        public string ConnectButtonText => _connecting ? "连接中…" : (IsConnected ? "断开" : "连接从端");
        public string ControlButtonText => HasControl ? "释放控制权" : "申请控制权";

        private async void ToggleConnect()
        {
            if (_connecting) return;
            if (IsConnected) { _link.Disconnect(); _hasStatus = false; RaiseConnectionProps(); return; }
            _connecting = true;
            RaiseConnectionProps();
            try { await _link.ConnectAsync(); }
            catch (Exception ex) { AddLog("error", "连接失败：" + ex.Message); }
            finally { _connecting = false; RaiseConnectionProps(); }
        }

        private async void ToggleControl()
        {
            if (!IsConnected) return;
            var r = HasControl ? await _link.ReleaseControlAsync() : await _link.AcquireControlAsync();
            if (!r.Ok) AddLog("warn", "控制权操作被拒绝：" + r.Reason);
            RaiseConnectionProps();
        }

        private void RaiseConnectionProps()
        {
            OnPropertyChanged(nameof(IsConnected));
            OnPropertyChanged(nameof(HasControl));
            OnPropertyChanged(nameof(CanOperate));
            OnPropertyChanged(nameof(ConnectButtonText));
            OnPropertyChanged(nameof(ControlButtonText));
            OnPropertyChanged(nameof(StatusFresh));
            OnPropertyChanged(nameof(OperateHintText));
        }

        public string OperateHintText
        {
            get
            {
                if (!IsConnected) return "未连接从端：操作区已锁定。";
                if (!HasControl) return "已连接，尚未持有控制权：请先点击“申请控制权”。";
                if (!StatusFresh) return "状态数据已过期：操作区已锁定。";
                return "";
            }
        }

        // ============================================================ 状态区（只读）

        private bool Has(StatusFlags f) => _hasStatus && _s.Has(f);

        public bool EstopHold => Has(StatusFlags.EstopHold);
        public bool ControlActive => Has(StatusFlags.ControlActive);
        public bool FfEnabled => Has(StatusFlags.FfEnabled);
        public bool FfZeroing => Has(StatusFlags.FfZeroing);
        public bool YValveClosed => Has(StatusFlags.YValveClosed);
        public bool AdsHealthy => Has(StatusFlags.AdsHealthy);

        /// <summary>模式只显示：0 导管 / 1 导丝（对应从端 guidewire_mode，当前仅作提示）。</summary>
        public bool IsCatheterMode => _hasStatus && _s.Mode == 0;
        public bool IsGuidewireMode => _hasStatus && _s.Mode == 1;
        public string ModeText => !_hasStatus ? "—" : (_s.Mode == 0 ? "导管" : "导丝");
        public string PhaseText => !_hasStatus ? "未连接" : (Has(StatusFlags.StartupCompleted) ? "已就绪" : "待机中");

        public string Force582F => FormatForce(_s.Force582F, "N");
        public string Force582N => FormatForce(_s.Force582N, "N·m");
        public string Force587F => FormatForce(_s.Force587F, "N");
        public string Force587N => FormatForce(_s.Force587N, "N·m");
        public string CleanForce => FormatForce(_s.CleanForceN, "N");

        private string FormatForce(float v, string unit)
            => _hasStatus ? v.ToString("0.000", CultureInfo.InvariantCulture) + " " + unit : "—";

        // ============================================================ 操作区

        public string CatheterText { get => _catheterText; set => SetField(ref _catheterText, value); }
        public string WireText { get => _wireText; set => SetField(ref _wireText, value); }
        public string PrepareError { get => _prepareError; private set => SetField(ref _prepareError, value); }

        public string PrepareStatusText
        {
            get
            {
                if (!_hasStatus) return "等待从端状态";
                if (Has(StatusFlags.SelfCheckDone)) return "已到达器械准备位置";
                switch (_s.SelfCheckStatus)
                {
                    case 1: return "等待开始";
                    case 2: return "运行中";
                    case 3: return "PLC 拒绝目标参数";
                    case 4: return "从端拒绝命令或 ADS 写入失败";
                    case 5: return "请求已发送，等待 PLC 确认";
                    default: return "待机";
                }
            }
        }

        public string FfCaption => FfZeroing ? "正在自动零点采集…" : (FfEnabled ? "已开启" : "已关闭（开启前自动采集零点）");
        public string YValveCaption => YValveClosed ? "Y 阀已关闭（再次点击取消并打开）" : "Y 阀已打开";

        public string Injector1Text => InjectorText(_hasStatus ? _s.InjectorActive1 : 0);
        public string Injector2Text => InjectorText(_hasStatus ? _s.InjectorActive2 : 0);
        public string Axis4Text => !_hasStatus ? "停止" : (_s.Axis4Active > 0 ? "前进中" : (_s.Axis4Active < 0 ? "后退中" : "停止"));
        private static string InjectorText(int dir) => dir > 0 ? "推进中" : (dir < 0 ? "回拉中" : "停止");

        private async void Prepare()
        {
            PrepareError = "";
            if (!double.TryParse(CatheterText, NumberStyles.Float, CultureInfo.InvariantCulture, out double cath) ||
                !double.TryParse(WireText, NumberStyles.Float, CultureInfo.InvariantCulture, out double wire) ||
                double.IsNaN(cath) || double.IsInfinity(cath) || double.IsNaN(wire) || double.IsInfinity(wire))
            {
                PrepareError = "请输入有效的数值。";
                return;
            }
            if (cath < 5 || cath > 95) { PrepareError = "导管搓捻机构位置范围为 5–95 mm。"; return; }
            if (wire < 10 || wire > 639) { PrepareError = "Y阀及导丝机构位置范围为 10–639 mm。"; return; }
            await Report("进入器械准备位置", await _link.PreparePositionAsync(cath, wire));
        }

        private async void ToggleForceFeedback()
        {
            if (FfZeroing) return; // 零点采集期间不响应
            bool target = !FfEnabled;
            await Report(target ? "开启力反馈" : "关闭力反馈", await _link.SetForceFeedbackAsync(target));
            OnPropertyChanged(nameof(FfEnabled));
        }

        private async void ToggleCylinder(object parameter)
        {
            if (!(parameter is CylinderItem item)) return;
            bool target = !item.Engaged;
            await Report($"电缸 {item.Index}", await _link.SetCylinderAsync(item.Index, target));
            item.Refresh();
        }

        private async void ToggleYValve()
        {
            bool target = !YValveClosed;
            await Report(target ? "关闭 Y 阀" : "打开 Y 阀", await _link.SetYValveClosedAsync(target));
            OnPropertyChanged(nameof(YValveClosed));
        }

        /// <summary>注射器按住推/拉：由窗口的鼠标按下/抬起事件调用。direction：-1 拉，0 停，+1 推。</summary>
        public void SetInjector(int index, int direction)
        {
            if (direction != 0 && !CanOperate) return;
            _link.SetInjector(index, direction);
        }

        /// <summary>轴4点动按住：-1 后退，0 停，+1 前进。</summary>
        public void SetAxis4(int direction)
        {
            if (direction != 0 && !CanOperate) return;
            _link.SetAxis4(direction);
        }

        /// <summary>窗口失焦、关闭等场景：所有“按住才动”的动作一律归零。</summary>
        public void StopAllHoldActions()
        {
            _link.SetInjector(1, 0);
            _link.SetInjector(2, 0);
            _link.SetAxis4(0);
        }

        private Task Report(string what, CommandResult r)
        {
            if (!r.Ok) AddLog("warn", $"{what}被拒绝：{r.Reason}");
            return Task.CompletedTask;
        }

        // ============================================================ 链路区

        public string SessionText => _stats.Session == 0 ? "—" : "0x" + _stats.Session.ToString("X8");
        public string RttText => IsConnected ? _stats.RttMs.ToString("0") + " ms" : "—";
        public string StatusAgeText => IsConnected && !double.IsNaN(_stats.StatusAgeMs) ? _stats.StatusAgeMs.ToString("0") + " ms" : "—";
        public string HapticAgeText => IsConnected && !double.IsNaN(_stats.HapticAgeMs) ? _stats.HapticAgeMs.ToString("0") + " ms" : "—";
        public string StatusHzText => IsConnected ? _stats.StatusHz.ToString("0") + " Hz" : "—";
        public string DroppedText => _stats.DroppedFrames.ToString();
        public string AdsText => !_hasStatus ? "—" : (AdsHealthy ? "正常" : "异常");

        private void RefreshStats()
        {
            _stats = _link.GetStats();
            OnPropertyChanged(nameof(SessionText));
            OnPropertyChanged(nameof(RttText));
            OnPropertyChanged(nameof(StatusAgeText));
            OnPropertyChanged(nameof(HapticAgeText));
            OnPropertyChanged(nameof(StatusHzText));
            OnPropertyChanged(nameof(DroppedText));
            OnPropertyChanged(nameof(AdsText));
            // 状态过期是由时间推移产生的，需要在这里重新评估。
            OnPropertyChanged(nameof(StatusFresh));
            OnPropertyChanged(nameof(CanOperate));
            OnPropertyChanged(nameof(OperateHintText));
        }

        // ============================================================ 状态帧入口

        private void OnStatus(StatusFrame f)
        {
            _s = f;
            _hasStatus = true;
            _lastStatusUtc = DateTime.UtcNow;

            foreach (var c in Cylinders)
            {
                c.Engaged = (f.CylinderManualMask & (1 << (c.Index - 1))) != 0;
                switch (c.Index)
                {
                    case 1: c.CurrentValue = f.Cylinder1; break;
                    case 2: c.CurrentValue = f.Cylinder2; break;
                    case 3: c.CurrentValue = f.Cylinder3; break;
                    default: c.CurrentValue = f.Cylinder4; break;
                }
            }

            foreach (var name in StatusPropertyNames) OnPropertyChanged(name);
            OnPropertyChanged(nameof(CanOperate));
            OnPropertyChanged(nameof(OperateHintText));
        }

        private static readonly string[] StatusPropertyNames =
        {
            nameof(EstopHold), nameof(ControlActive), nameof(FfEnabled), nameof(FfZeroing), nameof(YValveClosed),
            nameof(AdsHealthy), nameof(IsCatheterMode), nameof(IsGuidewireMode), nameof(ModeText), nameof(PhaseText),
            nameof(Force582F), nameof(Force582N), nameof(Force587F), nameof(Force587N), nameof(CleanForce),
            nameof(PrepareStatusText), nameof(FfCaption), nameof(YValveCaption), nameof(Injector1Text), nameof(Injector2Text), nameof(Axis4Text),
            nameof(StatusFresh),
        };

        // ============================================================ 日志

        private void AddLog(string level, string text, DateTime? time = null)
        {
            Log.Insert(0, new LogItem
            {
                Time = (time ?? DateTime.Now).ToString("HH:mm:ss"),
                Level = level,
                Text = text,
            });
            while (Log.Count > MaxLogItems) Log.RemoveAt(Log.Count - 1);
        }

        public void Dispose()
        {
            _statsTimer.Stop();
            _link.Dispose();
        }
    }
}
