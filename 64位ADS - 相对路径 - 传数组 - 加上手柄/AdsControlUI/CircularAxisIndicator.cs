using System;
using System.Windows;
using System.Windows.Media;

namespace AdsControlUI
{
    public class CircularAxisIndicator : FrameworkElement
    {
        public static readonly DependencyProperty AngleProperty =
            DependencyProperty.Register(
                nameof(Angle),
                typeof(double),
                typeof(CircularAxisIndicator),
                new FrameworkPropertyMetadata(0.0, FrameworkPropertyMetadataOptions.AffectsRender));

        public double Angle
        {
            get => (double)GetValue(AngleProperty);
            set => SetValue(AngleProperty, value);
        }

        protected override Size MeasureOverride(Size availableSize)
        {
            return new Size(58, 58);
        }

        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);

            double size = Math.Min(ActualWidth, ActualHeight);
            if (size <= 0) return;

            double stroke = Math.Max(5.0, size * 0.12);
            double radius = (size - stroke) / 2.0;
            Point center = new Point(ActualWidth / 2.0, ActualHeight / 2.0);
            double normalized = NormalizeDegrees(Angle);

            var trackPen = new Pen(new SolidColorBrush(Color.FromRgb(218, 224, 231)), stroke);
            var activePen = new Pen(new SolidColorBrush(Color.FromRgb(46, 160, 67)), stroke)
            {
                StartLineCap = PenLineCap.Round,
                EndLineCap = PenLineCap.Round
            };
            var tickPen = new Pen(new SolidColorBrush(Color.FromRgb(96, 109, 125)), 1.0);
            var majorTickPen = new Pen(new SolidColorBrush(Color.FromRgb(64, 78, 96)), 1.6);
            var needlePen = new Pen(new SolidColorBrush(Color.FromRgb(31, 76, 125)), 2.4)
            {
                StartLineCap = PenLineCap.Round,
                EndLineCap = PenLineCap.Round
            };
            var centerBrush = new SolidColorBrush(Color.FromRgb(31, 76, 125));
            var markerBrush = new SolidColorBrush(Color.FromRgb(46, 160, 67));

            drawingContext.DrawEllipse(null, trackPen, center, radius, radius);
            if (normalized > 0.01)
            {
                DrawArc(drawingContext, center, radius, -90.0, Math.Min(normalized, 359.99), activePen);
            }

            for (int i = 0; i < 12; i++)
            {
                double degrees = i * 30.0 - 90.0;
                double outer = radius + stroke * 0.25;
                double inner = radius - stroke * (i % 3 == 0 ? 0.70 : 0.45);
                drawingContext.DrawLine(
                    i % 3 == 0 ? majorTickPen : tickPen,
                    PointAt(center, inner, degrees),
                    PointAt(center, outer, degrees));
            }

            double pointerDegrees = normalized - 90.0;
            Point pointerEnd = PointAt(center, radius * 0.72, pointerDegrees);
            Point markerCenter = PointAt(center, radius, pointerDegrees);
            drawingContext.DrawLine(needlePen, center, pointerEnd);
            drawingContext.DrawEllipse(markerBrush, null, markerCenter, stroke * 0.45, stroke * 0.45);
            drawingContext.DrawEllipse(centerBrush, null, center, stroke * 0.32, stroke * 0.32);
        }

        private static void DrawArc(DrawingContext drawingContext, Point center, double radius, double startDegrees, double sweepDegrees, Pen pen)
        {
            Point startPoint = PointAt(center, radius, startDegrees);
            Point endPoint = PointAt(center, radius, startDegrees + sweepDegrees);
            var geometry = new StreamGeometry();

            using (StreamGeometryContext context = geometry.Open())
            {
                context.BeginFigure(startPoint, false, false);
                context.ArcTo(
                    endPoint,
                    new Size(radius, radius),
                    0.0,
                    sweepDegrees > 180.0,
                    SweepDirection.Clockwise,
                    true,
                    false);
            }

            geometry.Freeze();
            drawingContext.DrawGeometry(null, pen, geometry);
        }

        private static Point PointAt(Point center, double radius, double degrees)
        {
            double radians = degrees * Math.PI / 180.0;
            return new Point(
                center.X + Math.Cos(radians) * radius,
                center.Y + Math.Sin(radians) * radius);
        }

        private static double NormalizeDegrees(double angle)
        {
            double value = angle % 360.0;
            return value < 0.0 ? value + 360.0 : value;
        }
    }
}
