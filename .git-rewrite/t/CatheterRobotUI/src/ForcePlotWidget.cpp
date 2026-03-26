#include "ForcePlotWidget.h"

#include <QPaintEvent>

ForcePlotWidget::ForcePlotWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumHeight(220);
    setAutoFillBackground(false);
}

void ForcePlotWidget::appendData(double time_sec, short fn, short ft)
{
    elapsed_ = time_sec;
    samples_.push_back({ time_sec, static_cast<double>(fn), static_cast<double>(ft) });

    while (samples_.size() > max_points_)
    {
        samples_.remove(0);
    }

    update();
}

void ForcePlotWidget::clear()
{
    samples_.clear();
    elapsed_ = 0.0;
    update();
}

void ForcePlotWidget::setWindowSeconds(double sec)
{
    window_sec_ = (sec > 0.1) ? sec : 0.1;
    update();
}

void ForcePlotWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(30, 30, 40));

    const QRect rect_plot = plotRect();
    painter.fillRect(rect_plot, QColor(24, 24, 32));
    painter.setPen(QColor(70, 70, 85));
    painter.drawRect(rect_plot);

    painter.setPen(QColor(90, 90, 105));
    for (int i = 1; i < 4; ++i)
    {
        const int y = rect_plot.top() + (rect_plot.height() * i) / 4;
        painter.drawLine(rect_plot.left(), y, rect_plot.right(), y);
    }
    for (int i = 1; i < 5; ++i)
    {
        const int x = rect_plot.left() + (rect_plot.width() * i) / 5;
        painter.drawLine(x, rect_plot.top(), x, rect_plot.bottom());
    }

    drawSeries(painter, rect_plot, true);
    drawSeries(painter, rect_plot, false);

    painter.setPen(QColor(205, 214, 244));
    painter.drawText(12, 22, "Force Signals");
    painter.setPen(QColor(80, 180, 255));
    painter.drawText(12, 42, "Fn");
    painter.setPen(QColor(255, 140, 80));
    painter.drawText(48, 42, "Ft");

    painter.setPen(QColor(180, 180, 190));
    painter.drawText(rect_plot.left(), height() - 10,
        QString("t: %1s").arg(elapsed_, 0, 'f', 1));
    painter.drawText(rect_plot.right() - 90, height() - 10,
        QString("Y [%1, %2]").arg(y_min_, 0, 'f', 0).arg(y_max_, 0, 'f', 0));
}

QRect ForcePlotWidget::plotRect() const
{
    return rect().adjusted(40, 28, -16, -28);
}

double ForcePlotWidget::visibleStartTime() const
{
    return (elapsed_ > window_sec_) ? (elapsed_ - window_sec_) : 0.0;
}

QPointF ForcePlotWidget::mapToPlot(const QRect& rect_plot, double time_sec, double value) const
{
    const double t0 = visibleStartTime();
    const double t1 = t0 + window_sec_;
    const double tx = (t1 > t0) ? ((time_sec - t0) / (t1 - t0)) : 0.0;
    const double ty = (y_max_ > y_min_) ? ((value - y_min_) / (y_max_ - y_min_)) : 0.5;

    const double x = rect_plot.left() + tx * rect_plot.width();
    const double y = rect_plot.bottom() - ty * rect_plot.height();
    return QPointF(x, y);
}

void ForcePlotWidget::drawSeries(QPainter& painter, const QRect& rect_plot, bool draw_fn) const
{
    if (samples_.size() < 2)
    {
        return;
    }

    const double t0 = visibleStartTime();
    QPainterPath path;
    bool started = false;

    for (const ForcePoint& sample : samples_)
    {
        if (sample.time_sec < t0)
        {
            continue;
        }

        const QPointF pt = mapToPlot(rect_plot, sample.time_sec, draw_fn ? sample.fn : sample.ft);
        if (!started)
        {
            path.moveTo(pt);
            started = true;
        }
        else
        {
            path.lineTo(pt);
        }
    }

    if (!started)
    {
        return;
    }

    painter.setPen(QPen(draw_fn ? QColor(80, 180, 255) : QColor(255, 140, 80), 1.8));
    painter.drawPath(path);
}
