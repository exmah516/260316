// 文件职责说明：
// 1) 力过渡决定性预实验（论文方案 §6.1）的状态机：
//    多档恒定速度下，让 axis1 在导管模式做 "推送 → 触发计划回退 → 静置" 完整循环；
//    每个档位重复 M 次，由 UI 完整参数化。
// 2) 状态机在主循环每拍 tick()；激活期间接管 axis1 refer，禁用手柄→axis1 路径，
//    其他轴/电缸调度沿用主循环现有逻辑。
// 3) 安全：遇到 estop_hold / freeze_active / cal_zeroed=false / control_active=false
//    任一边沿，立即 Abort 并安全收尾（清 axis1 计划回退请求 + 还原 v_limit）。
#pragma once

#include "control_types.h"

#include <cstdint>

namespace plc_io
{
    // 前置声明：避免重新 #include 整个 plc_io.h（force_transition_experiment.cpp 内会 include）。
}

class ForceTransitionLogger;

enum class FtExpPhase : int
{
    Idle = 0,
    ArmWait = 1,
    ApproachStart = 2,
    PushForward = 3,
    ReachTriggerPos = 4,
    FastReturn = 5,
    ReturnWaitDone = 6,
    SettleHold = 7,
    AdvanceTrial = 8,
    Done = 9,
    Abort = 10,
};

struct ForceTransitionConfig
{
    // 速度档位数 N (2..kMaxLevels)；每档速度比例 (0..1)；每档重复次数 M。
    static constexpr int kMaxLevels = 6;
    int num_levels = 4;
    double v_ratio[kMaxLevels] = { 0.25, 0.50, 0.75, 1.00, 0.0, 0.0 };
    int repeats_per_level = 3;

    // axis1 行程（绝对位置，单位 mm）。
    double start_pos_mm = 10.0;          // 每次试次起点
    double push_target_mm = 70.0;        // 推送目标
    double return_trigger_mm = 85.0;     // 到此触发计划回退
    double approach_speed_ratio = 0.20;  // 拉到起点的安全速度

    // 节拍。
    int dwell_between_ms = 1500;         // 档间/试次间静置毫秒
};

class ForceTransitionExperiment
{
public:
    ForceTransitionExperiment();

    // C# 通过 SetFtExpParamA/B 灌入参数，落到 pending_cfg_；StartForceTransitionExperiment 触发本函数。
    // 返回 false 表示前置检查未通过，状态机不变（仍 Idle）。失败原因通过 last_error_ 暴露。
    bool start(AppContext& ctx, const ForceTransitionConfig& cfg, ForceTransitionLogger* logger);

    // UI 停止键 / 急停 / 任何边沿失败 → 安全收尾，状态机回 Idle。
    void abort(AppContext& ctx, const char* reason);

    // 主循环每拍调用；返回 true 表示当前实验状态机接管 axis1 refer。
    // 调用前主循环应已更新 plc_act_pos / plc_act_pos_from_left / loop_count / tick_ms。
    bool tick(
        AppContext& ctx,
        std::uint32_t now_tick_ms,
        bool control_active,
        bool freeze_active,
        bool estop_hold_active,
        bool cal_zeroed,
        GuidewireMode guidewire_mode);

    // 实验期间 axis1 应下发的 refer（绝对位置，PLC 坐标）。仅在 active() 为 true 时有意义。
    double current_axis1_refer() const { return axis1_refer_cmd_; }

    // 通过设置 axis1_fast_return 给 PLC 平滑器关阀门、并复用 force_feedback 的快退冻结。
    bool axis1_fast_return_request() const { return fast_return_request_; }

    // 是否需要 v_limit 覆写（PushForward / FastReturn 段为 true）。
    bool wants_v_limit_override() const { return v_limit_override_active_; }
    double current_v_limit_axis1() const { return v_limit_axis1_cmd_; }
    // 提供 7 元 v_limit 快照（axis1 已替换为 current_v_limit_axis1()），调用方直接 write_v_limit 即可。
    void fill_v_limit_override(double out_v_limit[7]) const;

    bool active() const { return phase_ != FtExpPhase::Idle && phase_ != FtExpPhase::Done && phase_ != FtExpPhase::Abort; }
    bool aborted() const { return phase_ == FtExpPhase::Abort; }

    FtExpPhase current_phase() const { return phase_; }
    int current_velocity_level() const { return velocity_level_; }
    int current_trial_id() const { return global_trial_id_; }
    int current_repeat_in_level() const { return repeat_in_level_; }
    double current_v_ratio() const { return current_v_ratio_; }
    double current_axis1_target() const { return axis1_target_pos_; }
    // 当前 phase 起始 tick，用于 logger 计算 dt_ms_from_phase_start。
    std::uint32_t current_phase_t0_ms() const { return phase_t0_ms_; }

    const char* last_error() const { return last_error_; }

    // C# 灌参 sink：field_id 见下表。范围检查在此处统一做。
    // ParamA: field_id ∈ {0:num_levels, 1:repeats_per_level, 2:dwell_between_ms}
    // ParamB: field_id ∈
    //   {10..15:v_ratio[0..5] (×1000),
    //    20:start_pos_mm (×1000),
    //    21:push_target_mm (×1000),
    //    22:return_trigger_mm (×1000),
    //    23:approach_speed_ratio (×1000)}
    void set_param_a(int field_id, int int_val);
    void set_param_b(int field_id, int fixed_x1000);

    const ForceTransitionConfig& pending_cfg() const { return pending_cfg_; }

private:
    void enter_phase(FtExpPhase next, std::uint32_t now_tick_ms);
    void release_axis1_overrides(AppContext& ctx);
    bool axis1_pos_abs(const AppContext& ctx, double& out_mm) const;

    // 配置缓冲（C# 端通过 SetFtExpParam* 多次写入，然后 Start 一次锁定 active_cfg_）。
    ForceTransitionConfig pending_cfg_{};
    ForceTransitionConfig active_cfg_{};

    FtExpPhase phase_ = FtExpPhase::Idle;
    std::uint32_t phase_t0_ms_ = 0;

    int velocity_level_ = 0;   // 0-based
    int repeat_in_level_ = 0;  // 0-based 当前档内已完成次数
    int global_trial_id_ = 0;  // 自实验开始累计

    double current_v_ratio_ = 0.0;
    double axis1_target_pos_ = 0.0;  // 当前 phase 的 axis1 期望目标
    double axis1_refer_cmd_ = 0.0;   // 实际下发的 refer
    double v_limit_axis1_cmd_ = 0.0;
    bool v_limit_override_active_ = false;
    bool fast_return_request_ = false;

    double v_limit_backup_ = 0.0;
    bool v_limit_backed_up_ = false;
    // 实验开始时读到的 PLC v_limit 快照（7 轴），实验期间作为覆写底版，避免每拍 ADS 读取。
    double v_limit_snapshot_[7] = { 0, 0, 0, 0, 0, 0, 0 };

    bool plan_return_requested_ = false;

    ForceTransitionLogger* logger_ = nullptr;

    // 软态边沿的连续掉拍计数（避免一拍抖动误终止）。
    int cal_zeroed_drop_count_ = 0;

    const char* last_error_ = "";
};
