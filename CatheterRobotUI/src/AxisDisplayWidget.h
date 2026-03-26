#pragma once
// ============================================================
// AxisDisplayWidget.h
// 单轴位置显示控件: 带软限位标记的进度条 + 数值
// ============================================================

#include <QWidget>
#include <QPainter>
#include <QLabel>
#include <QVBoxLayout>

class AxisDisplayWidget : public QWidget
{
    Q_OBJECT

public:
    explicit AxisDisplayWidget(const QString& name, QWidget* parent = nullptr)
        : QWidget(parent), axis_name_(name)
    {
        setMinimumHeight(40);
        setMinimumWidth(200);
    }

    void setRange(double left_limit, double right_limit)
    {
        left_limit_  = left_limit;
        right_limit_ = right_limit;
        update();
    }

    void setPosition(double pos, double refer)
    {
        current_pos_ = pos;
        refer_pos_   = refer;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);

        const int margin = 60;
        const int bar_h  = 16;
        const int bar_y  = (height() - bar_h) / 2;
        const int bar_w  = width() - 2 * margin;

        // 轴名
        p.setPen(Qt::white);
        QFont f = p.font();
        f.setPointSize(9);
        f.setBold(true);
        p.setFont(f);
        p.drawText(0, 0, margin - 4, height(), Qt::AlignVCenter | Qt::AlignRight, axis_name_);

        // 数值
        f.setBold(false);
        p.setFont(f);
        QString val_text = QString::number(current_pos_, 'f', 2);
        p.drawText(width() - margin + 4, 0, margin - 4, height(), Qt::AlignVCenter | Qt::AlignLeft, val_text);

        if (bar_w <= 0) return;

        double display_min = left_limit_ - 10.0;
        double display_max = right_limit_ + 10.0;
        double range = display_max - display_min;
        if (range < 1.0) range = 1.0;

        auto toX = [&](double val) -> int {
            return margin + static_cast<int>((val - display_min) / range * bar_w);
        };

        // 背景条
        p.fillRect(margin, bar_y, bar_w, bar_h, QColor(50, 50, 60));

        // 软限位区域 (红色半透明)
        int xl = toX(left_limit_);
        int xr = toX(right_limit_);
        p.fillRect(margin, bar_y, xl - margin, bar_h, QColor(180, 40, 40, 80));
        p.fillRect(xr, bar_y, margin + bar_w - xr, bar_h, QColor(180, 40, 40, 80));

        // 参考位置 (蓝线)
        int ref_x = toX(refer_pos_);
        p.setPen(QPen(QColor(80, 140, 255), 2));
        p.drawLine(ref_x, bar_y - 2, ref_x, bar_y + bar_h + 2);

        // 当前位置 (绿色圆点)
        int pos_x = toX(current_pos_);
        bool near_limit = (current_pos_ >= right_limit_ - 5.0) || (current_pos_ <= left_limit_ + 5.0);
        QColor dot_color = near_limit ? QColor(255, 80, 80) : QColor(80, 220, 120);
        p.setPen(Qt::NoPen);
        p.setBrush(dot_color);
        p.drawEllipse(QPoint(pos_x, bar_y + bar_h / 2), 6, 6);
    }

private:
    QString axis_name_;
    double  left_limit_  = -200.0;
    double  right_limit_ = 200.0;
    double  current_pos_ = 0.0;
    double  refer_pos_   = 0.0;
};
