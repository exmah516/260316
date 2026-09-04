using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Windows;
using Microsoft.Win32;
using ScottPlot;
using ScottPlot.Plottable;

namespace AdsControlUI
{
    public partial class ForceTransitionWindow : Window
    {
        private readonly AdsControlViewModel _vm;

        // 每档一个时间序列与位移序列（用 List<double> 持续追加）。
        // 注：跨试次保留所有点，便于直接判读"力-位移"族曲线是否重叠/分离（论文 §6.1 步骤 5）。
        private readonly List<List<double>> _xt = new List<List<double>>();
        private readonly List<List<double>> _yt = new List<List<double>>();
        private readonly List<List<double>> _xs = new List<List<double>>();
        private readonly List<List<double>> _ys = new List<List<double>>();
        private readonly List<ScatterPlot> _ftSeries = new List<ScatterPlot>();
        private readonly List<ScatterPlot> _fxSeries = new List<ScatterPlot>();

        // 当前 trial 的 t 基准：拿到第一个 PushForward 拍时记录，后续相对它换算 t(秒)。
        private uint _trialT0Ms = 0;
        private int _lastTrialId = -1;

        // 重绘节流：UI 数据按 ~30 Hz 接收，但 Refresh + AxisAuto 是 UI 线程重活，
        // 节流到约 10 Hz；trial 切换或档位切换时强制刷新一次。
        private DateTime _lastRefreshUtc = DateTime.MinValue;
        private static readonly TimeSpan kRefreshInterval = TimeSpan.FromMilliseconds(100);
        private bool _pendingRefresh = false;

        private const int kMaxLevels = 6;
        private static readonly Color[] kLevelColors = new[]
        {
            Color.FromArgb(220, 30, 30),
            Color.FromArgb(30, 120, 220),
            Color.FromArgb(30, 170, 80),
            Color.FromArgb(200, 130, 30),
            Color.FromArgb(150, 60, 180),
            Color.FromArgb(80, 80, 80),
        };

        public ForceTransitionWindow(AdsControlViewModel vm)
        {
            InitializeComponent();
            _vm = vm;
            DataContext = _vm;

            for (int i = 0; i < kMaxLevels; ++i)
            {
                _xt.Add(new List<double>());
                _yt.Add(new List<double>());
                _xs.Add(new List<double>());
                _ys.Add(new List<double>());
            }

            ConfigurePlots();
        }

        private void ConfigurePlots()
        {
            PlotFt.Plot.Title("F vs t");
            PlotFt.Plot.XLabel("t (s) from PushForward start");
            PlotFt.Plot.YLabel("F feedback (N)");
            PlotFt.Plot.Legend();

            PlotFx.Plot.Title("F vs 从端位移");
            PlotFx.Plot.XLabel("axis1 act pos (mm)");
            PlotFx.Plot.YLabel("F feedback (N)");
            PlotFx.Plot.Legend();

            for (int i = 0; i < kMaxLevels; ++i)
            {
                var ft = PlotFt.Plot.AddScatter(new double[] { }, new double[] { }, kLevelColors[i], lineWidth: 1f, markerSize: 0);
                ft.Label = $"档 {i + 1}";
                _ftSeries.Add(ft);

                var fx = PlotFx.Plot.AddScatter(new double[] { }, new double[] { }, kLevelColors[i], lineWidth: 1f, markerSize: 0);
                fx.Label = $"档 {i + 1}";
                _fxSeries.Add(fx);
            }

            PlotFt.Refresh();
            PlotFx.Refresh();
        }

        public void OnState(VisState state)
        {
            // 仅 PushForward (3) 与 ReachTriggerPos (4) 段计入"力过渡曲线"。
            // 其他阶段（接近 / 静置 / 快退）不画。
            if (!state.ft_exp_active && state.ft_exp_phase != 9) // 9=Done 时残留快照也允许显示已采集数据
            {
                AbortIndicator.Visibility = state.ft_exp_aborted ? Visibility.Visible : Visibility.Collapsed;
                return;
            }
            AbortIndicator.Visibility = state.ft_exp_aborted ? Visibility.Visible : Visibility.Collapsed;

            if (state.ft_exp_phase != 3 && state.ft_exp_phase != 4)
                return;

            int level = state.ft_exp_velocity_level;
            if (level < 0 || level >= kMaxLevels) return;

            if (state.ft_exp_trial_id != _lastTrialId)
            {
                _lastTrialId = state.ft_exp_trial_id;
                _trialT0Ms = state.tick_ms;
                _pendingRefresh = true; // trial 切换强制重绘
            }

            double tSec = (state.tick_ms - _trialT0Ms) / 1000.0;
            double xMm = state.axis_pos != null && state.axis_pos.Length > 0 ? state.axis_pos[0] : 0.0;
            double f = state.force_582_theory_f;

            _xt[level].Add(tSec);
            _yt[level].Add(f);
            _xs[level].Add(xMm);
            _ys[level].Add(f);

            // 限制每档点数避免内存爆炸（30 Hz × 长时实验仍可能很多）。
            const int kCap = 20000;
            TrimFront(_xt[level], kCap);
            TrimFront(_yt[level], kCap);
            TrimFront(_xs[level], kCap);
            TrimFront(_ys[level], kCap);

            MaybeRefresh(level);
        }

        private void MaybeRefresh(int level)
        {
            var now = DateTime.UtcNow;
            if (!_pendingRefresh && (now - _lastRefreshUtc) < kRefreshInterval)
                return;
            _lastRefreshUtc = now;
            _pendingRefresh = false;
            UpdateSeries(level);
        }

        private static void TrimFront(List<double> list, int cap)
        {
            if (list.Count > cap)
            {
                list.RemoveRange(0, list.Count - cap);
            }
        }

        private void UpdateSeries(int level)
        {
            // 更新所有档（不仅是当前 level），避免在节流期间其它档的新数据被遗漏。
            for (int i = 0; i < kMaxLevels; ++i)
            {
                _ftSeries[i].Update(_xt[i].ToArray(), _yt[i].ToArray());
                _fxSeries[i].Update(_xs[i].ToArray(), _ys[i].ToArray());
            }
            PlotFt.Plot.AxisAuto();
            PlotFx.Plot.AxisAuto();
            PlotFt.Refresh();
            PlotFx.Refresh();
        }

        private void BtnStart_Click(object sender, RoutedEventArgs e)
        {
            ErrorText.Text = "";
            if (!ParseInputs(out var cfg, out string err))
            {
                ErrorText.Text = err;
                return;
            }
            if (!_vm.FfEnabled) { ErrorText.Text = "请先开启力反馈。"; return; }
            if (!_vm.CalZeroed) { ErrorText.Text = "请先完成力传感器零点采集。"; return; }
            if (!_vm.ControlActive) { ErrorText.Text = "请先进入控制模式 (启动准备完成或直接控制)。"; return; }
            if (_vm.EstopHold) { ErrorText.Text = "急停保持中，无法启动。"; return; }

            _vm.SendFtExpConfig(
                cfg.NumLevels, cfg.VRatios, cfg.Repeats,
                cfg.StartPosMm, cfg.PushTargetMm, cfg.ReturnTriggerMm,
                cfg.ApproachRatio, cfg.DwellMs);
            _vm.StartForceTransitionExperiment();
        }

        private void BtnStop_Click(object sender, RoutedEventArgs e)
        {
            _vm.StopForceTransitionExperiment();
        }

        private void BtnClear_Click(object sender, RoutedEventArgs e)
        {
            for (int i = 0; i < kMaxLevels; ++i)
            {
                _xt[i].Clear(); _yt[i].Clear();
                _xs[i].Clear(); _ys[i].Clear();
                _ftSeries[i].Update(new double[] { }, new double[] { });
                _fxSeries[i].Update(new double[] { }, new double[] { });
            }
            _lastTrialId = -1;
            _pendingRefresh = true;
            PlotFt.Plot.AxisAuto();
            PlotFx.Plot.AxisAuto();
            PlotFt.Refresh();
            PlotFx.Refresh();
        }

        private void BtnExport_Click(object sender, RoutedEventArgs e)
        {
            var dlg = new SaveFileDialog
            {
                Filter = "PNG image (*.png)|*.png",
                FileName = $"ForceTransition_{DateTime.Now:yyyyMMdd_HHmmss}.png",
            };
            if (dlg.ShowDialog() != true) return;
            string baseName = Path.Combine(
                Path.GetDirectoryName(dlg.FileName) ?? ".",
                Path.GetFileNameWithoutExtension(dlg.FileName));
            string ftPath = baseName + "_Ft.png";
            string fxPath = baseName + "_Fx.png";
            PlotFt.Plot.SaveFig(ftPath);
            PlotFx.Plot.SaveFig(fxPath);
            ErrorText.Foreground = System.Windows.Media.Brushes.DarkGreen;
            ErrorText.Text = $"已导出: {Path.GetFileName(ftPath)}, {Path.GetFileName(fxPath)}";
        }

        private struct ExpConfig
        {
            public int NumLevels;
            public double[] VRatios;
            public int Repeats;
            public double StartPosMm;
            public double PushTargetMm;
            public double ReturnTriggerMm;
            public double ApproachRatio;
            public int DwellMs;
        }

        private bool ParseInputs(out ExpConfig cfg, out string err)
        {
            cfg = new ExpConfig();
            err = "";

            if (!int.TryParse(TbNumLevels.Text, out int n) || n < 2 || n > kMaxLevels)
            {
                err = "档位数必须为 2-6 的整数。"; return false;
            }
            var vs = new double[kMaxLevels];
            var raw = new[] { TbV1.Text, TbV2.Text, TbV3.Text, TbV4.Text, TbV5.Text, TbV6.Text };
            for (int i = 0; i < n; ++i)
            {
                if (!double.TryParse(raw[i], out double v) || v < 1 || v > 150)
                {
                    err = $"速度档 {i + 1} 必须为 1-150 之间的百分比。"; return false;
                }
                vs[i] = v / 100.0;
            }
            if (!int.TryParse(TbRepeats.Text, out int repeats) || repeats < 1 || repeats > 50)
            {
                err = "每档重复次数必须为 1-50 的整数。"; return false;
            }
            if (!double.TryParse(TbStartPos.Text, out double start) || start < 0 || start > 99)
            {
                err = "起始位置必须在 0-99 mm 之间。"; return false;
            }
            if (!double.TryParse(TbPushTarget.Text, out double push) || push <= start || push > 99)
            {
                err = "推送目标必须 > 起始位置且 ≤ 99 mm。"; return false;
            }
            if (!double.TryParse(TbReturnTrigger.Text, out double trig) || trig < push || trig > 99)
            {
                err = "回退触发位必须 ≥ 推送目标且 ≤ 99 mm。"; return false;
            }
            if (!double.TryParse(TbApproachRatio.Text, out double approach) || approach < 1 || approach > 150)
            {
                err = "接近速度比例必须为 1-150 之间的百分比。"; return false;
            }
            if (!int.TryParse(TbDwellMs.Text, out int dwell) || dwell < 100 || dwell > 60000)
            {
                err = "档间静置必须为 100-60000 ms。"; return false;
            }

            cfg.NumLevels = n;
            cfg.VRatios = vs;
            cfg.Repeats = repeats;
            cfg.StartPosMm = start;
            cfg.PushTargetMm = push;
            cfg.ReturnTriggerMm = trig;
            cfg.ApproachRatio = approach / 100.0;
            cfg.DwellMs = dwell;
            return true;
        }
    }
}
