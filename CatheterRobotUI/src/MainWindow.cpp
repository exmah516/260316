// ============================================================
// MainWindow.cpp
// ============================================================

#include "MainWindow.h"
#include "AxisDisplayWidget.h"
#include "ForcePlotWidget.h"

// ── 样式常量 ──
static const char* STYLE_SHEET = R"(
    QMainWindow { background: #1e1e2e; }
    QGroupBox {
        color: #cdd6f4;
        border: 1px solid #45475a;
        border-radius: 6px;
        margin-top: 8px;
        padding-top: 14px;
        font-weight: bold;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 4px;
    }
    QLabel { color: #cdd6f4; }
    QPushButton {
        background: #313244;
        color: #cdd6f4;
        border: 1px solid #45475a;
        border-radius: 4px;
        padding: 6px 14px;
        font-size: 12px;
    }
    QPushButton:hover { background: #45475a; }
    QPushButton:pressed { background: #585b70; }
    QPushButton:checked { background: #89b4fa; color: #1e1e2e; }
    QPushButton#estop {
        background: #f38ba8;
        color: #1e1e2e;
        font-size: 14px;
        font-weight: bold;
        min-height: 40px;
    }
    QPushButton#estop:hover { background: #eba0ac; }
    QSlider::groove:horizontal {
        height: 6px;
        background: #45475a;
        border-radius: 3px;
    }
    QSlider::handle:horizontal {
        background: #89b4fa;
        width: 16px;
        margin: -5px 0;
        border-radius: 8px;
    }
    QDoubleSpinBox, QComboBox {
        background: #313244;
        color: #cdd6f4;
        border: 1px solid #45475a;
        border-radius: 4px;
        padding: 4px;
    }
)";

MainWindow::MainWindow(SharedState& shared, QWidget* parent)
    : QMainWindow(parent), shared_(shared)
{
    setWindowTitle("血管介入机器人控制系统");
    resize(1280, 800);
    setStyleSheet(STYLE_SHEET);

    recorder_ = new ForceRecorder(this);
    buildUI();

    // 定时刷新 (30ms ≈ 33fps)
    refresh_timer_ = new QTimer(this);
    connect(refresh_timer_, &QTimer::timeout, this, &MainWindow::onRefreshTimer);
    refresh_timer_->start(30);
    elapsed_.start();

    // 启动控制线程
    ctrl_thread_ = new ControlThread(shared_, this);
    connect(ctrl_thread_, &ControlThread::initFinished,
            this, &MainWindow::onControlInitFinished);
    ctrl_thread_->start();
}

MainWindow::~MainWindow()
{
    if (ctrl_thread_)
    {
        shared_.modifyUICommand([](UIToControl& c) { c.cmd_quit = true; });
        ctrl_thread_->requestStop();
        ctrl_thread_->wait(3000);
    }
    if (recorder_->isRecording())
        recorder_->stopRecording();
}

// ============================================================
// UI 构建
// ============================================================
void MainWindow::buildUI()
{
    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* root = new QVBoxLayout(central);
    root->setSpacing(6);
    root->setContentsMargins(8, 8, 8, 8);

    // 顶部状态栏
    root->addWidget(buildStatusBar());

    // 中部: 左(轴位置) + 右(模式+参数)
    auto* mid = new QHBoxLayout();
    mid->addWidget(buildAxisPanel(), 3);

    auto* right_col = new QVBoxLayout();
    right_col->addWidget(buildModePanel());
    right_col->addWidget(buildParamPanel());
    right_col->addStretch();
    mid->addLayout(right_col, 2);
    root->addLayout(mid, 2);

    // 定位臂 5 轴手动控制
    root->addWidget(buildArmPanel());

    // 底部: 力面板 + 按钮
    auto* bot = new QHBoxLayout();
    bot->addWidget(buildForcePanel(), 3);
    bot->addWidget(buildButtonPanel(), 1);
    root->addLayout(bot, 2);
}

// ── 区域一: 连接状态栏 ──
QWidget* MainWindow::buildStatusBar()
{
    auto* bar = new QWidget();
    bar->setFixedHeight(36);
    auto* lay = new QHBoxLayout(bar);
    lay->setContentsMargins(4, 0, 4, 0);

    auto makeIndicator = [&](const QString& text) -> QLabel* {
        auto* lbl = new QLabel(text);
        lbl->setAlignment(Qt::AlignCenter);
        lbl->setFixedWidth(140);
        lbl->setStyleSheet("background:#313244; border-radius:4px; padding:4px;");
        return lbl;
    };

    lbl_h582_    = makeIndicator("手柄582: --");
    lbl_h587_    = makeIndicator("手柄587: --");
    lbl_ads_     = makeIndicator("ADS: --");
    lbl_plc_     = makeIndicator("PLC: --");
    lbl_loop_hz_ = makeIndicator("频率: -- Hz");

    lay->addWidget(lbl_h582_);
    lay->addWidget(lbl_h587_);
    lay->addWidget(lbl_ads_);
    lay->addWidget(lbl_plc_);
    lay->addStretch();
    lay->addWidget(lbl_loop_hz_);

    return bar;
}

// ── 区域二: 轴位置面板 ──
QWidget* MainWindow::buildAxisPanel()
{
    auto* group = new QGroupBox("轴位置");
    auto* lay = new QVBoxLayout(group);

    const QString names[] = {
        "轴1 平移", "轴2 旋转", "轴3 平移",
        "轴4 球囊", "轴5 平移", "轴6 平移", "轴7 旋转"
    };

    for (int i = 0; i < 7; ++i)
    {
        axis_widgets_[i] = new AxisDisplayWidget(names[i], group);
        lay->addWidget(axis_widgets_[i]);
    }

    return group;
}

// ── 区域三: 控制模式 ──
QWidget* MainWindow::buildModePanel()
{
    auto* group = new QGroupBox("控制模式");
    auto* lay = new QVBoxLayout(group);

    auto makeModeLabel = [](const QString& text) -> QLabel* {
        auto* lbl = new QLabel(text);
        lbl->setAlignment(Qt::AlignCenter);
        lbl->setFixedHeight(28);
        lbl->setStyleSheet("background:#313244; border-radius:4px; padding:2px;");
        return lbl;
    };

    lbl_mode_catheter_  = makeModeLabel("导管模式");
    lbl_mode_guidewire_ = makeModeLabel("导丝模式");
    lbl_mode_both_      = makeModeLabel("导丝+导管模式");

    auto* mode_row = new QHBoxLayout();
    mode_row->addWidget(lbl_mode_catheter_);
    mode_row->addWidget(lbl_mode_guidewire_);
    mode_row->addWidget(lbl_mode_both_);
    lay->addLayout(mode_row);

    // 蠕动状态
    auto* crawl_row = new QHBoxLayout();
    lbl_crawl_axis1_ = makeModeLabel("轴1蠕动: --");
    lbl_crawl_axis6_ = makeModeLabel("轴6蠕动: --");
    crawl_row->addWidget(lbl_crawl_axis1_);
    crawl_row->addWidget(lbl_crawl_axis6_);
    lay->addLayout(crawl_row);

    return group;
}

// ── 区域四: 参数调整 ──
QWidget* MainWindow::buildParamPanel()
{
    auto* group = new QGroupBox("参数设置");
    auto* lay = new QGridLayout(group);

    // 球囊轴速度
    lay->addWidget(new QLabel("球囊轴速度:"), 0, 0);
    spin_axis4_vel_ = new QDoubleSpinBox();
    spin_axis4_vel_->setRange(0.1, 20.0);
    spin_axis4_vel_->setValue(1.5);
    spin_axis4_vel_->setSuffix(" mm/s");
    spin_axis4_vel_->setSingleStep(0.5);
    connect(spin_axis4_vel_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onAxis4VelocityChanged);
    lay->addWidget(spin_axis4_vel_, 0, 1);

    // 速度模式预设
    lay->addWidget(new QLabel("速度模式:"), 1, 0);
    combo_speed_ = new QComboBox();
    combo_speed_->addItems({"精细", "标准", "快速"});
    combo_speed_->setCurrentIndex(1);
    connect(combo_speed_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onSpeedPresetChanged);
    lay->addWidget(combo_speed_, 1, 1);

    // 手柄灵敏度
    lay->addWidget(new QLabel("灵敏度:"), 2, 0);
    slider_sens_ = new QSlider(Qt::Horizontal);
    slider_sens_->setRange(10, 300);  // 0.1x ~ 3.0x
    slider_sens_->setValue(100);
    connect(slider_sens_, &QSlider::valueChanged, this, &MainWindow::onSensitivityChanged);
    lay->addWidget(slider_sens_, 2, 1);
    lbl_sens_val_ = new QLabel("1.00x");
    lbl_sens_val_->setFixedWidth(50);
    lay->addWidget(lbl_sens_val_, 2, 2);

    return group;
}

// ── 区域五: 定位臂 5 轴手动点动 ──
QWidget* MainWindow::buildArmPanel()
{
    auto* group = new QGroupBox("定位臂手动控制");
    auto* lay = new QGridLayout(group);
    lay->setHorizontalSpacing(6);
    lay->setVerticalSpacing(4);

    btn_arm_manual_enable_ = new QPushButton("手动使能: 开");
    btn_arm_manual_enable_->setCheckable(true);
    btn_arm_manual_enable_->setChecked(true);
    btn_arm_manual_enable_->setFixedWidth(120);
    connect(btn_arm_manual_enable_, &QPushButton::toggled, this, [this](bool checked) {
        btn_arm_manual_enable_->setText(checked ? "手动使能: 开" : "手动使能: 关");
        shared_.modifyUICommand([checked](UIToControl& c) {
            c.arm_manual_enable = checked;
            if (!checked)
            {
                c.arm_jog_pos_req = {};
                c.arm_jog_neg_req = {};
            }
        });
    });
    lay->addWidget(btn_arm_manual_enable_, 0, 0, 1, 2);

    const QString headers[] = {
        "轴", "上电", "复位", "Jog-", "Jog+",
        "Vel", "Acc", "Dec", "Jerk", "位置", "速度", "状态"
    };
    for (int col = 0; col < 12; ++col)
    {
        auto* header = new QLabel(headers[col], group);
        header->setAlignment(Qt::AlignCenter);
        header->setStyleSheet("color:#a6adc8; font-size:11px;");
        lay->addWidget(header, 1, col);
    }

    auto makeButton = [group](const QString& text, int width = 54) -> QPushButton* {
        auto* btn = new QPushButton(text, group);
        btn->setFixedWidth(width);
        btn->setFixedHeight(26);
        return btn;
    };

    auto makeSpin = [group](double value, double step) -> QDoubleSpinBox* {
        auto* spin = new QDoubleSpinBox(group);
        spin->setRange(0.01, 10000.0);
        spin->setDecimals(2);
        spin->setValue(value);
        spin->setSingleStep(step);
        spin->setFixedWidth(74);
        return spin;
    };

    for (int i = 0; i < 5; ++i)
    {
        const int row = i + 2;
        auto* axis_label = new QLabel(QString("臂%1").arg(i + 1), group);
        axis_label->setAlignment(Qt::AlignCenter);
        lay->addWidget(axis_label, row, 0);

        btn_arm_enable_[i] = makeButton("上电", 60);
        btn_arm_enable_[i]->setCheckable(true);
        connect(btn_arm_enable_[i], &QPushButton::toggled, this, [this, i](bool checked) {
            shared_.modifyUICommand([i, checked](UIToControl& c) {
                c.arm_enable_req[i] = checked;
                if (!checked)
                {
                    c.arm_jog_pos_req[i] = false;
                    c.arm_jog_neg_req[i] = false;
                }
            });
        });
        lay->addWidget(btn_arm_enable_[i], row, 1);

        btn_arm_reset_[i] = makeButton("复位", 54);
        connect(btn_arm_reset_[i], &QPushButton::clicked, this, [this, i]() {
            shared_.modifyUICommand([i](UIToControl& c) {
                c.arm_reset_req[i] = true;
            });
        });
        lay->addWidget(btn_arm_reset_[i], row, 2);

        btn_arm_jog_neg_[i] = makeButton("Jog-", 58);
        connect(btn_arm_jog_neg_[i], &QPushButton::pressed, this, [this, i]() {
            shared_.modifyUICommand([i](UIToControl& c) {
                c.arm_jog_neg_req[i] = true;
            });
        });
        connect(btn_arm_jog_neg_[i], &QPushButton::released, this, [this, i]() {
            shared_.modifyUICommand([i](UIToControl& c) {
                c.arm_jog_neg_req[i] = false;
            });
        });
        lay->addWidget(btn_arm_jog_neg_[i], row, 3);

        btn_arm_jog_pos_[i] = makeButton("Jog+", 58);
        connect(btn_arm_jog_pos_[i], &QPushButton::pressed, this, [this, i]() {
            shared_.modifyUICommand([i](UIToControl& c) {
                c.arm_jog_pos_req[i] = true;
            });
        });
        connect(btn_arm_jog_pos_[i], &QPushButton::released, this, [this, i]() {
            shared_.modifyUICommand([i](UIToControl& c) {
                c.arm_jog_pos_req[i] = false;
            });
        });
        lay->addWidget(btn_arm_jog_pos_[i], row, 4);

        spin_arm_vel_[i] = makeSpin(5.0, 0.5);
        spin_arm_acc_[i] = makeSpin(50.0, 5.0);
        spin_arm_dec_[i] = makeSpin(50.0, 5.0);
        spin_arm_jerk_[i] = makeSpin(500.0, 50.0);
        connect(spin_arm_vel_[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, [this, i](double value) {
            shared_.modifyUICommand([i, value](UIToControl& c) { c.arm_jog_velocity[i] = value; });
        });
        connect(spin_arm_acc_[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, [this, i](double value) {
            shared_.modifyUICommand([i, value](UIToControl& c) { c.arm_jog_acc[i] = value; });
        });
        connect(spin_arm_dec_[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, [this, i](double value) {
            shared_.modifyUICommand([i, value](UIToControl& c) { c.arm_jog_dec[i] = value; });
        });
        connect(spin_arm_jerk_[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, [this, i](double value) {
            shared_.modifyUICommand([i, value](UIToControl& c) { c.arm_jog_jerk[i] = value; });
        });
        lay->addWidget(spin_arm_vel_[i], row, 5);
        lay->addWidget(spin_arm_acc_[i], row, 6);
        lay->addWidget(spin_arm_dec_[i], row, 7);
        lay->addWidget(spin_arm_jerk_[i], row, 8);

        lbl_arm_pos_[i] = new QLabel("0.00", group);
        lbl_arm_vel_[i] = new QLabel("0.00", group);
        lbl_arm_status_[i] = new QLabel("未上电", group);
        lbl_arm_pos_[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        lbl_arm_vel_[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        lbl_arm_status_[i]->setAlignment(Qt::AlignCenter);
        lbl_arm_pos_[i]->setMinimumWidth(72);
        lbl_arm_vel_[i]->setMinimumWidth(72);
        lbl_arm_status_[i]->setMinimumWidth(120);
        lbl_arm_status_[i]->setStyleSheet("background:#313244; border-radius:4px; padding:3px;");
        lay->addWidget(lbl_arm_pos_[i], row, 9);
        lay->addWidget(lbl_arm_vel_[i], row, 10);
        lay->addWidget(lbl_arm_status_[i], row, 11);
    }

    return group;
}

// ── 区域五: 力反馈面板 ──
QWidget* MainWindow::buildForcePanel()
{
    auto* group = new QGroupBox("力反馈");
    auto* lay = new QVBoxLayout(group);

    // 曲线
    force_plot_ = new ForcePlotWidget(group);
    lay->addWidget(force_plot_, 1);

    // 实时数值
    auto* val_row = new QHBoxLayout();
    lbl_fn_val_ = new QLabel("Fn: 0");
    lbl_ft_val_ = new QLabel("Ft: 0");
    val_row->addWidget(lbl_fn_val_);
    val_row->addWidget(lbl_ft_val_);
    lay->addLayout(val_row);

    // 增益滑块
    auto* gain_row = new QGridLayout();
    gain_row->addWidget(new QLabel("轴向增益:"), 0, 0);
    slider_gain_fn_ = new QSlider(Qt::Horizontal);
    slider_gain_fn_->setRange(0, 200);
    slider_gain_fn_->setValue(100);
    connect(slider_gain_fn_, &QSlider::valueChanged, this, &MainWindow::onForceGainAxialChanged);
    gain_row->addWidget(slider_gain_fn_, 0, 1);

    gain_row->addWidget(new QLabel("扭矩增益:"), 1, 0);
    slider_gain_ft_ = new QSlider(Qt::Horizontal);
    slider_gain_ft_->setRange(0, 200);
    slider_gain_ft_->setValue(100);
    connect(slider_gain_ft_, &QSlider::valueChanged, this, &MainWindow::onForceGainTorqueChanged);
    gain_row->addWidget(slider_gain_ft_, 1, 1);
    lay->addLayout(gain_row);

    // 按钮行
    auto* btn_row = new QHBoxLayout();
    btn_ff_toggle_ = new QPushButton("力反馈: 关");
    btn_ff_toggle_->setCheckable(true);
    connect(btn_ff_toggle_, &QPushButton::toggled, this, &MainWindow::onForceFeedbackToggle);
    btn_row->addWidget(btn_ff_toggle_);

    btn_record_ = new QPushButton("开始录制");
    connect(btn_record_, &QPushButton::clicked, this, &MainWindow::onForceRecordToggle);
    btn_row->addWidget(btn_record_);
    lay->addLayout(btn_row);

    return group;
}

// ── 区域六: 操作按钮 ──
QWidget* MainWindow::buildButtonPanel()
{
    auto* group = new QGroupBox("操作");
    auto* lay = new QVBoxLayout(group);

    btn_pause_ = new QPushButton("暂停");
    btn_pause_->setCheckable(true);
    connect(btn_pause_, &QPushButton::toggled, this, &MainWindow::onPauseToggle);
    lay->addWidget(btn_pause_);

    btn_estop_ = new QPushButton("急  停");
    btn_estop_->setObjectName("estop");
    connect(btn_estop_, &QPushButton::clicked, this, &MainWindow::onEstop);
    lay->addWidget(btn_estop_);

    lay->addStretch();
    return group;
}

// ============================================================
// 定时刷新 (30ms)
// ============================================================
void MainWindow::onRefreshTimer()
{
    ControlToUI state = shared_.readControlState();

    // ── 状态栏 ──
    updateStatusIndicator(lbl_h582_, state.handle582_connected, "手柄582");
    updateStatusIndicator(lbl_h587_, state.handle587_connected, "手柄587");
    updateStatusIndicator(lbl_ads_,  state.ads_connected,       "ADS");

    static const char* plc_names[] = {
        "初始化", "Jog1", "Jog2", "Jog3", "运输", "运输等待",
        "手柄控制", "自检", "清除错误", "错误", "复位"
    };
    int si = static_cast<int>(state.plc_state);
    QString plc_text = (si >= 0 && si <= 10) ? plc_names[si] : "未知";
    bool plc_ok = (state.plc_state == PlcState::Handle);
    updateStatusIndicator(lbl_plc_, plc_ok, "PLC: " + plc_text);

    lbl_loop_hz_->setText(QString("频率: %1 Hz").arg(state.loop_rate_hz, 0, 'f', 0));

    // ── 轴位置 ──
    for (int i = 0; i < 7; ++i)
    {
        double left  = state.rightlimit[i] - 600.0;  // TODO: 用实际 left_slimit
        double right = state.rightlimit[i];
        axis_widgets_[i]->setRange(left, right);
        axis_widgets_[i]->setPosition(state.act_pos[i], state.refer[i]);
    }

    // ── 定位臂状态 ──
    if (btn_arm_manual_enable_ != nullptr &&
        btn_arm_manual_enable_->isChecked() != state.arm_manual_enable)
    {
        btn_arm_manual_enable_->blockSignals(true);
        btn_arm_manual_enable_->setChecked(state.arm_manual_enable);
        btn_arm_manual_enable_->blockSignals(false);
    }
    if (btn_arm_manual_enable_ != nullptr)
    {
        btn_arm_manual_enable_->setText(state.arm_manual_enable ? "手动使能: 开" : "手动使能: 关");
    }

    auto errorText = [](const QString& prefix, unsigned long error_id) -> QString {
        if (error_id == 0)
        {
            return prefix;
        }
        return QString("%1 0x%2")
            .arg(prefix)
            .arg(static_cast<qulonglong>(error_id), 0, 16)
            .toUpper();
    };
    auto setArmStatus = [](QLabel* label, const QString& text, const QString& color) {
        label->setText(text);
        label->setStyleSheet(
            QString("background:#313244; border-radius:4px; padding:3px; color:%1; font-weight:bold;").arg(color));
    };

    for (int i = 0; i < 5; ++i)
    {
        lbl_arm_pos_[i]->setText(QString::number(state.arm_act_pos[i], 'f', 2));
        lbl_arm_vel_[i]->setText(QString::number(state.arm_act_vel[i], 'f', 2));

        const bool powered = state.arm_power_done[i] && !state.arm_power_error[i];
        if (state.arm_cmd_conflict[i])
        {
            setArmStatus(lbl_arm_status_[i], "命令冲突", "#f9e2af");
        }
        else if (state.arm_power_error[i])
        {
            setArmStatus(lbl_arm_status_[i], errorText("上电错误", state.arm_power_error_id[i]), "#f38ba8");
        }
        else if (state.arm_reset_error[i])
        {
            setArmStatus(lbl_arm_status_[i], errorText("复位错误", state.arm_reset_error_id[i]), "#f38ba8");
        }
        else if (state.arm_motion_error[i])
        {
            setArmStatus(lbl_arm_status_[i], errorText("运动错误", state.arm_motion_error_id[i]), "#f38ba8");
        }
        else if (state.arm_reset_busy[i])
        {
            setArmStatus(lbl_arm_status_[i], "复位中", "#89b4fa");
        }
        else if (state.arm_motion_busy[i])
        {
            QString dir = "点动中";
            if (state.arm_cmd_dir[i] > 0)
            {
                dir = "正向点动";
            }
            else if (state.arm_cmd_dir[i] < 0)
            {
                dir = "反向点动";
            }
            setArmStatus(lbl_arm_status_[i], dir, "#89b4fa");
        }
        else if (powered)
        {
            setArmStatus(lbl_arm_status_[i], "已上电", "#a6e3a1");
        }
        else if (state.arm_enable_req[i])
        {
            setArmStatus(lbl_arm_status_[i], "上电中", "#f9e2af");
        }
        else
        {
            setArmStatus(lbl_arm_status_[i], "未上电", "#a6adc8");
        }

        btn_arm_enable_[i]->setText(powered ? "已上电" : (btn_arm_enable_[i]->isChecked() ? "上电中" : "上电"));
    }

    // ── 控制模式 ──
    auto setModeActive = [](QLabel* lbl, bool active) {
        lbl->setStyleSheet(active
            ? "background:#89b4fa; color:#1e1e2e; border-radius:4px; font-weight:bold;"
            : "background:#313244; border-radius:4px;");
    };
    setModeActive(lbl_mode_catheter_,  state.current_mode == ControlMode::Catheter);
    setModeActive(lbl_mode_guidewire_, state.current_mode == ControlMode::Guidewire);
    setModeActive(lbl_mode_both_,      state.current_mode == ControlMode::GuidewireCatheter);

    // 蠕动状态
    static const char* phase_names[] = {"跟随", "切换", "快退", "夹紧", "恢复"};
    auto phaseText = [](CrawlPhase p, bool enabled) -> QString {
        if (!enabled) return "未激活";
        int pi = static_cast<int>(p);
        return (pi >= 0 && pi <= 4) ? phase_names[pi] : "未知";
    };
    lbl_crawl_axis1_->setText("轴1: " + phaseText(state.axis1_crawl_phase, state.axis1_crawl_enabled));
    lbl_crawl_axis6_->setText("轴6: " + phaseText(state.axis6_crawl_phase, state.axis6_crawl_enabled));

    // ── 力信号 ──
    lbl_fn_val_->setText(QString("Fn: %1 (滤波: %2)").arg(state.fn_raw).arg(state.fn_filtered, 0, 'f', 2));
    lbl_ft_val_->setText(QString("Ft: %1 (滤波: %2)").arg(state.ft_raw).arg(state.ft_filtered, 0, 'f', 2));

    double t = elapsed_.elapsed() / 1000.0;
    force_plot_->appendData(t, state.fn_raw, state.ft_raw);

    // ── 录制 ──
    if (recorder_->isRecording())
    {
        auto samples = shared_.drainForceSamples();
        if (!samples.empty())
            recorder_->writeSamples(samples);
    }
}

void MainWindow::updateStatusIndicator(QLabel* label, bool connected, const QString& text)
{
    QString color = connected ? "#a6e3a1" : "#f38ba8";
    label->setText(text + (connected ? ": 已连接" : ": 断开"));
    label->setStyleSheet(
        QString("background:#313244; border-radius:4px; padding:4px; color:%1; font-weight:bold;").arg(color));
}

// ============================================================
// 槽函数
// ============================================================
void MainWindow::onControlInitFinished(bool ok, const QString& msg)
{
    if (!ok)
    {
        lbl_plc_->setText("启动失败");
        lbl_plc_->setStyleSheet("background:#f38ba8; color:#1e1e2e; border-radius:4px; padding:4px;");
    }
}

void MainWindow::onForceRecordToggle()
{
    if (recorder_->isRecording())
    {
        shared_.modifyUICommand([](UIToControl& c) { c.force_record_stop = true; });
        QString path = recorder_->stopRecording();
        btn_record_->setText("开始录制");
        btn_record_->setStyleSheet("");
    }
    else
    {
        if (recorder_->startRecording())
        {
            shared_.modifyUICommand([](UIToControl& c) { c.force_record_start = true; });
            btn_record_->setText("停止录制 ●");
            btn_record_->setStyleSheet("color:#f38ba8;");
        }
    }
}

void MainWindow::onForceFeedbackToggle(bool on)
{
    shared_.modifyUICommand([on](UIToControl& c) { c.force_feedback_on = on; });
    btn_ff_toggle_->setText(on ? "力反馈: 开" : "力反馈: 关");
}

void MainWindow::onSpeedPresetChanged(int index)
{
    auto preset = static_cast<UIToControl::SpeedPreset>(index);
    shared_.modifyUICommand([preset](UIToControl& c) { c.speed_preset = preset; });
}

void MainWindow::onAxis4VelocityChanged(double val)
{
    shared_.modifyUICommand([val](UIToControl& c) { c.axis4_velocity = val; });
}

void MainWindow::onForceGainAxialChanged(int val)
{
    double gain = val / 100.0;
    shared_.modifyUICommand([gain](UIToControl& c) { c.force_gain_axial = gain; });
}

void MainWindow::onForceGainTorqueChanged(int val)
{
    double gain = val / 100.0;
    shared_.modifyUICommand([gain](UIToControl& c) { c.force_gain_torque = gain; });
}

void MainWindow::onSensitivityChanged(int val)
{
    double sens = val / 100.0;
    shared_.modifyUICommand([sens](UIToControl& c) { c.handle_sensitivity = sens; });
    lbl_sens_val_->setText(QString("%1x").arg(sens, 0, 'f', 2));
}

void MainWindow::onPauseToggle()
{
    bool paused = btn_pause_->isChecked();
    shared_.modifyUICommand([paused](UIToControl& c) { c.cmd_pause = paused; });
    btn_pause_->setText(paused ? "恢复" : "暂停");
}

void MainWindow::onEstop()
{
    shared_.modifyUICommand([](UIToControl& c) { c.cmd_estop = true; });
}
