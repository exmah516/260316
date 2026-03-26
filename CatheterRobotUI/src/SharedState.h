#pragma once
// ============================================================
// SharedState.h
// 控制线程 ↔ UI 线程的唯一数据桥梁
// 所有跨线程数据交换必须经过此结构体 + mutex
// ============================================================

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <deque>

// ─── 控制模式枚举 ───
enum class ControlMode
{
    Catheter,           // 导管模式: 587 都松开
    Guidewire,          // 导丝模式: 587 按下按钮 A
    GuidewireCatheter   // 导丝+导管模式: 587 按下按钮 B
};

// ─── 蠕动阶段枚举(镜像 CrawlState::Phase) ───
enum class CrawlPhase
{
    Follow, SwitchWait, FastMove, ClampWait, RestoreWait
};

// ─── PLC 状态枚举(镜像 state.TcDUT) ───
enum class PlcState
{
    Init = 0, Jog1, Jog2, Jog3, Trans, TransWait,
    Handle, SelfCheck, ClearErr, Err, Reset
};

// ─── 力信号采样点(带时间戳) ───
struct ForceSample
{
    double timestamp_sec;   // 相对于录制开始的秒数
    short  fn_raw;
    short  ft_raw;
    double fn_filtered;
    double ft_filtered;
};

// ============================================================
// 控制线程 → UI (只读方向: UI 只读, 控制线程只写)
// ============================================================
struct ControlToUI
{
    // 连接状态
    bool handle582_connected = false;
    bool handle587_connected = false;
    bool ads_connected       = false;

    // PLC 状态
    PlcState plc_state       = PlcState::Init;
    bool self_check_done     = false;
    bool control_active      = false;
    bool estop_hold_active   = false;
    bool freeze_active       = false;

    // 轴位置 (相对坐标, 对应 G.Act_pos)
    std::array<double, 7> act_pos      = {};
    std::array<double, 7> init_pos     = {};
    std::array<double, 7> rightlimit   = {};
    std::array<double, 7> refer        = {};

    // 蠕动状态
    CrawlPhase axis1_crawl_phase = CrawlPhase::Follow;
    CrawlPhase axis6_crawl_phase = CrawlPhase::Follow;
    bool axis1_crawl_enabled = false;
    bool axis6_crawl_enabled = false;

    // 控制模式
    ControlMode current_mode = ControlMode::Catheter;

    // 力信号
    short  fn_raw = 0;
    short  ft_raw = 0;
    double fn_filtered = 0.0;
    double ft_filtered = 0.0;
    double fn_bias     = 0.0;

    // 手柄原始数据 (调试用)
    unsigned char handle582_buttons = 0;
    unsigned char handle587_buttons = 0;
    std::array<double, 2> handle582_joints = {};
    std::array<double, 2> handle587_joints = {};

    // 电缸状态
    std::array<unsigned short, 4> cylinder_cmds = {};

    // 性能
    double loop_rate_hz = 0.0;
};

// ============================================================
// UI → 控制线程 (命令方向: UI 只写, 控制线程只读+清除)
// ============================================================
struct UIToControl
{
    // 速度参数
    double axis4_velocity     = 1.5;    // 球囊输送轴速度, 对应 v_limit[4]
    double handle_sensitivity = 1.0;    // 手柄映射灵敏度倍率 (乘到 k_handle_to_mm)

    // 力反馈
    bool   force_feedback_on  = false;
    double force_gain_axial   = 1.0;    // 轴向力增益倍率
    double force_gain_torque  = 1.0;    // 扭矩增益倍率

    // 录制
    bool   force_record_start = false;  // 上升沿触发开始
    bool   force_record_stop  = false;  // 上升沿触发停止

    // 控制指令
    bool   cmd_pause          = false;  // UI 软暂停
    bool   cmd_estop          = false;  // UI 软急停
    bool   cmd_redo_selfcheck = false;  // 重新自检
    bool   cmd_quit           = false;  // 退出

    // 速度模式预设
    enum class SpeedPreset { Fine, Normal, Fast };
    SpeedPreset speed_preset  = SpeedPreset::Normal;
};

// ============================================================
// 主共享状态: 用 mutex 保护
// ============================================================
class SharedState
{
public:
    // ── 读取控制线程的输出(UI 线程调用) ──
    ControlToUI readControlState()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return ctrl_to_ui_;
    }

    // ── 控制线程写入状态 ──
    void writeControlState(const ControlToUI& data)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        ctrl_to_ui_ = data;
    }

    // ── 读取 UI 命令(控制线程调用) ──
    UIToControl readUICommands()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        UIToControl copy = ui_to_ctrl_;
        // 清除一次性触发信号
        ui_to_ctrl_.force_record_start = false;
        ui_to_ctrl_.force_record_stop  = false;
        ui_to_ctrl_.cmd_redo_selfcheck = false;
        ui_to_ctrl_.cmd_estop          = false;
        return copy;
    }

    // ── UI 线程写入命令 ──
    void writeUICommand(const UIToControl& cmd)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        ui_to_ctrl_ = cmd;
    }

    // ── 单项更新(UI 线程, 避免覆盖整个结构体) ──
    template<typename Fn>
    void modifyUICommand(Fn&& fn)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        fn(ui_to_ctrl_);
    }

    // ── 力信号录制队列(无锁, 控制线程 push, UI 线程 drain) ──
    void pushForceSample(const ForceSample& s)
    {
        std::lock_guard<std::mutex> lock(force_mtx_);
        force_queue_.push_back(s);
        if (force_queue_.size() > 100000)  // 防止 OOM
            force_queue_.pop_front();
    }

    std::deque<ForceSample> drainForceSamples()
    {
        std::lock_guard<std::mutex> lock(force_mtx_);
        std::deque<ForceSample> out;
        out.swap(force_queue_);
        return out;
    }

private:
    std::mutex    mtx_;
    ControlToUI   ctrl_to_ui_;
    UIToControl   ui_to_ctrl_;

    std::mutex              force_mtx_;
    std::deque<ForceSample> force_queue_;
};
