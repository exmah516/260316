#pragma once
// ============================================================
// MainWindow.h
// ============================================================

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QElapsedTimer>

#include "SharedState.h"
#include "ControlThread.h"
#include "ForceRecorder.h"

class AxisDisplayWidget;
class ForcePlotWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(SharedState& shared, QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void onRefreshTimer();
    void onControlInitFinished(bool ok, const QString& msg);
    void onForceRecordToggle();
    void onForceFeedbackToggle(bool on);
    void onSpeedPresetChanged(int index);
    void onAxis4VelocityChanged(double val);
    void onForceGainAxialChanged(int val);
    void onForceGainTorqueChanged(int val);
    void onSensitivityChanged(int val);
    void onPauseToggle();
    void onEstop();

private:
    void buildUI();
    QWidget* buildStatusBar();
    QWidget* buildAxisPanel();
    QWidget* buildModePanel();
    QWidget* buildParamPanel();
    QWidget* buildForcePanel();
    QWidget* buildButtonPanel();
    void updateStatusIndicator(QLabel* label, bool connected, const QString& text);

    SharedState&   shared_;
    ControlThread* ctrl_thread_ = nullptr;
    ForceRecorder* recorder_    = nullptr;
    QTimer*        refresh_timer_ = nullptr;
    QElapsedTimer  elapsed_;

    // ── 状态栏控件 ──
    QLabel* lbl_h582_     = nullptr;
    QLabel* lbl_h587_     = nullptr;
    QLabel* lbl_ads_      = nullptr;
    QLabel* lbl_plc_      = nullptr;
    QLabel* lbl_loop_hz_  = nullptr;

    // ── 轴显示控件 ──
    AxisDisplayWidget* axis_widgets_[7] = {};

    // ── 模式面板 ──
    QLabel* lbl_mode_catheter_  = nullptr;
    QLabel* lbl_mode_guidewire_ = nullptr;
    QLabel* lbl_mode_both_      = nullptr;
    QLabel* lbl_crawl_axis1_    = nullptr;
    QLabel* lbl_crawl_axis6_    = nullptr;

    // ── 参数面板 ──
    QDoubleSpinBox* spin_axis4_vel_  = nullptr;
    QComboBox*      combo_speed_     = nullptr;
    QSlider*        slider_sens_     = nullptr;
    QLabel*         lbl_sens_val_    = nullptr;

    // ── 力面板 ──
    ForcePlotWidget* force_plot_     = nullptr;
    QPushButton*     btn_record_     = nullptr;
    QPushButton*     btn_ff_toggle_  = nullptr;
    QSlider*         slider_gain_fn_ = nullptr;
    QSlider*         slider_gain_ft_ = nullptr;
    QLabel*          lbl_fn_val_     = nullptr;
    QLabel*          lbl_ft_val_     = nullptr;

    // ── 按钮 ──
    QPushButton*     btn_pause_  = nullptr;
    QPushButton*     btn_estop_  = nullptr;

    // 本地 UI 命令缓存
    UIToControl      ui_cmd_;
};
