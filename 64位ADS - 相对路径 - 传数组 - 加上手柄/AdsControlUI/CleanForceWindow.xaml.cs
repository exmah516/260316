using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace AdsControlUI
{
    public partial class CleanForceWindow : Window
    {
        private struct CleanPoint
        {
            public double TimeS;
            public double ForceN;
            public double TorqueNm;
        }

        private const double WindowSeconds = 30.0;
        private readonly List<CleanPoint> _points = new List<CleanPoint>();
        private uint? _firstTick;

        public CleanForceWindow()
        {
            InitializeComponent();
            StatusText.Text = "等待数据";
        }

        public void AddState(VisState state)
        {
            FnRawText.Text = state.force_sample_valid && IsFinite(state.fn_1_v) ? state.fn_1_v.ToString("F6") : "不可用";
            FtRawText.Text = state.force_sample_valid && IsFinite(state.ft_1_v) ? state.ft_1_v.ToString("F6") : "不可用";

            if (!state.cal_zeroed)
            {
                ShowOverlay("等待零点采集", "纯净力将在零点有效后显示");
                StatusText.Text = "未调零";
                CleanForceText.Text = "—";
                CleanTorqueText.Text = "—";
                return;
            }
            if (!state.clean_force_valid || !IsFinite(state.clean_force_n) || !IsFinite(state.clean_handle_torque_nm))
            {
                ShowOverlay("等待有效力感采样", "原始采样恢复后曲线将自动继续");
                StatusText.Text = "采样无效";
                CleanForceText.Text = "—";
                CleanTorqueText.Text = "—";
                return;
            }

            if (!_firstTick.HasValue)
                _firstTick = state.tick_ms;
            double timeS = unchecked(state.tick_ms - _firstTick.Value) / 1000.0;
            _points.Add(new CleanPoint
            {
                TimeS = timeS,
                ForceN = state.clean_force_n,
                TorqueNm = state.clean_handle_torque_nm
            });
            double minTime = Math.Max(0.0, timeS - WindowSeconds);
            while (_points.Count > 0 && _points[0].TimeS < minTime)
                _points.RemoveAt(0);

            EmptyOverlay.Visibility = Visibility.Collapsed;
            StatusText.Text = "最近 30 s · 双 Y 轴";
            CleanForceText.Text = state.clean_force_n.ToString("F4");
            CleanTorqueText.Text = state.clean_handle_torque_nm.ToString("F7");
            Redraw();
        }

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        private void ShowOverlay(string title, string hint)
        {
            EmptyTitleText.Text = title;
            EmptyHintText.Text = hint;
            EmptyOverlay.Visibility = Visibility.Visible;
        }

        private void PlotCanvas_SizeChanged(object sender, SizeChangedEventArgs e) => Redraw();

        private void Redraw()
        {
            double width = PlotCanvas.ActualWidth;
            double height = PlotCanvas.ActualHeight;
            if (width < 80 || height < 80) return;

            PlotCanvas.Children.OfType<Line>().ToList().ForEach(line => PlotCanvas.Children.Remove(line));
            const double left = 58.0;
            const double right = 68.0;
            const double top = 16.0;
            const double bottom = 30.0;
            double plotWidth = Math.Max(1.0, width - left - right);
            double plotHeight = Math.Max(1.0, height - top - bottom);
            DrawAxes(left, top, plotWidth, plotHeight);

            if (_points.Count == 0)
            {
                ForceLine.Points = new PointCollection();
                TorqueLine.Points = new PointCollection();
                PositionLabels(left, top, plotWidth, plotHeight, -1, 1, -0.001, 0.001);
                return;
            }

            double latest = _points[_points.Count - 1].TimeS;
            double minTime = Math.Max(0.0, latest - WindowSeconds);
            double maxTime = Math.Max(WindowSeconds, latest);
            GetRange(_points.Select(p => p.ForceN), 0.02, out double forceMin, out double forceMax);
            GetRange(_points.Select(p => p.TorqueNm), 0.00002, out double torqueMin, out double torqueMax);
            ForceLine.Points = BuildPoints(left, top, plotWidth, plotHeight, minTime, maxTime, forceMin, forceMax, p => p.ForceN);
            TorqueLine.Points = BuildPoints(left, top, plotWidth, plotHeight, minTime, maxTime, torqueMin, torqueMax, p => p.TorqueNm);
            PositionLabels(left, top, plotWidth, plotHeight, forceMin, forceMax, torqueMin, torqueMax);
        }

        private static void GetRange(IEnumerable<double> values, double minimumRange, out double min, out double max)
        {
            min = values.Min();
            max = values.Max();
            double range = max - min;
            if (range < minimumRange)
            {
                double center = (min + max) * 0.5;
                min = center - minimumRange * 0.5;
                max = center + minimumRange * 0.5;
            }
            else
            {
                double padding = range * 0.1;
                min -= padding;
                max += padding;
            }
        }

        private PointCollection BuildPoints(
            double left, double top, double plotWidth, double plotHeight,
            double minTime, double maxTime, double minY, double maxY,
            Func<CleanPoint, double> selector)
        {
            var result = new PointCollection();
            double timeRange = Math.Max(0.001, maxTime - minTime);
            double yRange = Math.Max(double.Epsilon, maxY - minY);
            foreach (CleanPoint point in _points)
            {
                double x = left + (point.TimeS - minTime) / timeRange * plotWidth;
                double y = top + (1.0 - (selector(point) - minY) / yRange) * plotHeight;
                result.Add(new Point(x, y));
            }
            return result;
        }

        private void DrawAxes(double left, double top, double plotWidth, double plotHeight)
        {
            var axisBrush = new SolidColorBrush(Color.FromRgb(0x94, 0xA3, 0xB8));
            var midBrush = new SolidColorBrush(Color.FromRgb(0xE2, 0xE8, 0xF0));
            axisBrush.Freeze();
            midBrush.Freeze();
            AddLine(left, top, left, top + plotHeight, axisBrush, 1);
            AddLine(left + plotWidth, top, left + plotWidth, top + plotHeight, axisBrush, 1);
            AddLine(left, top + plotHeight, left + plotWidth, top + plotHeight, axisBrush, 1);
            AddLine(left, top + plotHeight * 0.5, left + plotWidth, top + plotHeight * 0.5, midBrush, 1);
        }

        private void AddLine(double x1, double y1, double x2, double y2, Brush brush, double thickness)
        {
            PlotCanvas.Children.Insert(0, new Line
            {
                X1 = x1,
                Y1 = y1,
                X2 = x2,
                Y2 = y2,
                Stroke = brush,
                StrokeThickness = thickness
            });
        }

        private void PositionLabels(
            double left, double top, double plotWidth, double plotHeight,
            double forceMin, double forceMax, double torqueMin, double torqueMax)
        {
            ForceMaxText.Text = forceMax.ToString("F3");
            ForceMinText.Text = forceMin.ToString("F3");
            TorqueMaxText.Text = torqueMax.ToString("F6");
            TorqueMinText.Text = torqueMin.ToString("F6");
            Canvas.SetLeft(ForceMaxText, 4);
            Canvas.SetTop(ForceMaxText, top - 5);
            Canvas.SetLeft(ForceMinText, 4);
            Canvas.SetTop(ForceMinText, top + plotHeight - 14);
            Canvas.SetLeft(TorqueMaxText, left + plotWidth + 8);
            Canvas.SetTop(TorqueMaxText, top - 5);
            Canvas.SetLeft(TorqueMinText, left + plotWidth + 8);
            Canvas.SetTop(TorqueMinText, top + plotHeight - 14);
            Canvas.SetLeft(XAxisText, left + plotWidth - 28);
            Canvas.SetTop(XAxisText, top + plotHeight + 7);
        }
    }
}
