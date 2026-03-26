#pragma once

#include <QPainterPath>
#include <QPainter>
#include <QPointF>
#include <QVector>
#include <QWidget>

class ForcePlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ForcePlotWidget(QWidget* parent = nullptr);

    void appendData(double time_sec, short fn, short ft);
    void clear();
    void setWindowSeconds(double sec);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    struct ForcePoint
    {
        double time_sec = 0.0;
        double fn = 0.0;
        double ft = 0.0;
    };

    QRect plotRect() const;
    double visibleStartTime() const;
    QPointF mapToPlot(const QRect& rect, double time_sec, double value) const;
    void drawSeries(QPainter& painter, const QRect& rect, bool draw_fn) const;

    QVector<ForcePoint> samples_;
    double window_sec_ = 15.0;
    double elapsed_ = 0.0;
    int max_points_ = 3000;
    double y_min_ = -2000.0;
    double y_max_ = 2000.0;
};
