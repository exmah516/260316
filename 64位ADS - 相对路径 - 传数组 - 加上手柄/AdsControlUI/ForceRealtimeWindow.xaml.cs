using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace AdsControlUI
{
    public partial class ForceRealtimeWindow : Window
    {
        private struct ForcePoint
        {
            public double TimeS;
            public double ForceActualN;
            public double TorqueActualNm;
            public double ForceTheoryN;
            public double TorqueTheoryNm;
        }

        private const double WindowSeconds = 30.0;
        private const double MinYRange = 0.01;
        private readonly List<ForcePoint> _points = new List<ForcePoint>();
        private uint? _firstTick;

        public ForceRealtimeWindow()
        {
            InitializeComponent();
            StatusText.Text = "等待实时力反馈数据...";
        }

        public void AddState(VisState state)
        {
            if (!state.ff_enabled || !state.cal_zeroed)
            {
                StatusText.Text = "力反馈未开启或未标零";
                return;
            }

            if (!_firstTick.HasValue)
            {
                _firstTick = state.tick_ms;
            }

            double t = TickDeltaSeconds(state.tick_ms, _firstTick.Value);
            _points.Add(new ForcePoint
            {
                TimeS = t,
                ForceActualN = state.force_582_f,
                TorqueActualNm = state.force_582_n,
                ForceTheoryN = state.force_582_theory_f,
                TorqueTheoryNm = state.force_582_theory_n
            });

            double minTime = Math.Max(0.0, t - WindowSeconds);
            while (_points.Count > 0 && _points[0].TimeS < minTime)
            {
                _points.RemoveAt(0);
            }

            StatusText.Text = $"显示最近 {WindowSeconds:F0} s";
            ForceValueText.Text = $"F actual = {state.force_582_f:F3} N";
            TorqueValueText.Text = $"T actual = {state.force_582_n:F5} N·m";
            ForceTheoryValueText.Text = $"F theory = {state.force_582_theory_f:F3} N";
            TorqueTheoryValueText.Text = $"T theory = {state.force_582_theory_n:F5} N·m";
            Redraw();
        }

        private static double TickDeltaSeconds(uint tick, uint firstTick)
        {
            return unchecked(tick - firstTick) / 1000.0;
        }

        private void PlotCanvas_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            Redraw();
        }

        private void Redraw()
        {
            double width = PlotCanvas.ActualWidth;
            double height = PlotCanvas.ActualHeight;
            if (width <= 20 || height <= 20)
            {
                return;
            }

            PlotCanvas.Children.OfType<Line>().ToList().ForEach(line => PlotCanvas.Children.Remove(line));

            double left = 44.0;
            double right = 12.0;
            double top = 12.0;
            double bottom = 30.0;
            double plotW = Math.Max(1.0, width - left - right);
            double plotH = Math.Max(1.0, height - top - bottom);

            DrawAxis(left, top, bottom, plotW, plotH);

            if (_points.Count == 0)
            {
                ForceLine.Points = new PointCollection();
                TorqueLine.Points = new PointCollection();
                ForceTheoryLine.Points = new PointCollection();
                TorqueTheoryLine.Points = new PointCollection();
                PositionLabels(left, top, bottom, plotW, plotH, -1.0, 1.0);
                return;
            }

            double latestTime = _points[_points.Count - 1].TimeS;
            double minTime = Math.Max(0.0, latestTime - WindowSeconds);
            double maxTime = Math.Max(WindowSeconds, latestTime);
            double minY = _points.Min(p => Math.Min(Math.Min(p.ForceActualN, p.TorqueActualNm), Math.Min(p.ForceTheoryN, p.TorqueTheoryNm)));
            double maxY = _points.Max(p => Math.Max(Math.Max(p.ForceActualN, p.TorqueActualNm), Math.Max(p.ForceTheoryN, p.TorqueTheoryNm)));
            if (maxY - minY < MinYRange)
            {
                double center = (maxY + minY) * 0.5;
                minY = center - MinYRange * 0.5;
                maxY = center + MinYRange * 0.5;
            }
            else
            {
                double pad = (maxY - minY) * 0.1;
                minY -= pad;
                maxY += pad;
            }

            ForceLine.Points = BuildPoints(left, top, plotW, plotH, minTime, maxTime, minY, maxY, p => p.ForceActualN);
            TorqueLine.Points = BuildPoints(left, top, plotW, plotH, minTime, maxTime, minY, maxY, p => p.TorqueActualNm);
            ForceTheoryLine.Points = BuildPoints(left, top, plotW, plotH, minTime, maxTime, minY, maxY, p => p.ForceTheoryN);
            TorqueTheoryLine.Points = BuildPoints(left, top, plotW, plotH, minTime, maxTime, minY, maxY, p => p.TorqueTheoryNm);
            PositionLabels(left, top, bottom, plotW, plotH, minY, maxY);
        }

        private PointCollection BuildPoints(
            double left,
            double top,
            double plotW,
            double plotH,
            double minTime,
            double maxTime,
            double minY,
            double maxY,
            Func<ForcePoint, double> selector)
        {
            var points = new PointCollection();
            double timeRange = Math.Max(0.001, maxTime - minTime);
            double yRange = Math.Max(MinYRange, maxY - minY);
            foreach (ForcePoint p in _points)
            {
                double x = left + ((p.TimeS - minTime) / timeRange) * plotW;
                double y = top + (1.0 - ((selector(p) - minY) / yRange)) * plotH;
                points.Add(new Point(x, y));
            }
            return points;
        }

        private void DrawAxis(double left, double top, double bottom, double plotW, double plotH)
        {
            AddLine(left, top, left, top + plotH, Brushes.Gray, 1);
            AddLine(left, top + plotH, left + plotW, top + plotH, Brushes.Gray, 1);
            AddLine(left, top + plotH * 0.5, left + plotW, top + plotH * 0.5, Brushes.LightGray, 1);
        }

        private void AddLine(double x1, double y1, double x2, double y2, Brush brush, double thickness)
        {
            var line = new Line
            {
                X1 = x1,
                Y1 = y1,
                X2 = x2,
                Y2 = y2,
                Stroke = brush,
                StrokeThickness = thickness
            };
            PlotCanvas.Children.Insert(0, line);
        }

        private void PositionLabels(double left, double top, double bottom, double plotW, double plotH, double minY, double maxY)
        {
            YAxisMaxText.Text = maxY.ToString("F4");
            YAxisMinText.Text = minY.ToString("F4");
            Canvas.SetLeft(YAxisMaxText, 4);
            Canvas.SetTop(YAxisMaxText, top - 4);
            Canvas.SetLeft(YAxisMinText, 4);
            Canvas.SetTop(YAxisMinText, top + plotH - 14);
            Canvas.SetLeft(XAxisText, left + plotW - 32);
            Canvas.SetTop(XAxisText, PlotCanvas.ActualHeight - bottom + 8);
        }
    }
}
