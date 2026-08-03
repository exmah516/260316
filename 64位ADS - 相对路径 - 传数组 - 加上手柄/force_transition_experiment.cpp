// 文件职责说明：力过渡决定性预实验（论文 §6.1）状态机实现。
//
// 设计要点：
// - 沿用 StartupPhase 状态机的"phase + 退出条件"模式（见 c++上位机项目代码说明.md §7/§12）。
// - 接管 axis1 refer 仅限实验状态机活动期间；不动其他轴/电缸的主循环逻辑。
// - 实验自身不直接写气缸——夹爪切换沿用主循环既有的"窗口边界 + CrawlState"动作链；
//   实验只负责"以恒定速度让 axis1 从 start_pos_mm 推到 return_trigger_mm"，
//   到位即下发计划回退（plc_io::request_axis_return），让 PLC 把 axis1 拉回 start_pos_mm。
// - axis1_fast_return 信号在 ReachTriggerPos~ReturnWaitDone 期间为 true，
//   让既有 force_feedback 快退冻结链路自动接管，保持与正常工况一致的"力盲窗"。

#include "force_transition_experiment.h"

#include "plc_io.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
constexpr double kStartPosTolMm = 0.05;       // ApproachStart 到位判定容差
constexpr double kPushArriveTolMm = 0.05;     // PushForward 到位（达到 return_trigger_mm）容差
constexpr double kMinVRatio = 0.01;
constexpr double kMaxVRatio = 1.50;
constexpr double kMinDwellMs = 100.0;
constexpr int kMaxDwellMs = 60000;
constexpr int kMaxRepeats = 50;

double clamp01(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
} // namespace

ForceTransitionExperiment::ForceTransitionExperiment() = default;

bool ForceTransitionExperiment::start(AppContext& ctx, const ForceTransitionConfig& cfg)
{
    last_error_ = "";
    if (phase_ != FtExpPhase::Idle && phase_ != FtExpPhase::Done && phase_ != FtExpPhase::Abort)
    {
        last_error_ = "already running";
        return false;
    }

    // 校验配置。
    if (cfg.num_levels < 2 || cfg.num_levels > ForceTransitionConfig::kMaxLevels)
    {
        last_error_ = "num_levels out of range";
        return false;
    }
    if (cfg.repeats_per_level < 1 || cfg.repeats_per_level > kMaxRepeats)
    {
        last_error_ = "repeats_per_level out of range";
        return false;
    }
    for (int i = 0; i < cfg.num_levels; ++i)
    {
        if (cfg.v_ratio[i] < kMinVRatio || cfg.v_ratio[i] > kMaxVRatio)
        {
            last_error_ = "v_ratio out of range";
            return false;
        }
    }
    if (!(cfg.push_target_mm > cfg.start_pos_mm))
    {
        last_error_ = "push_target_mm must > start_pos_mm";
        return false;
    }
    if (!(cfg.return_trigger_mm >= cfg.push_target_mm))
    {
        last_error_ = "return_trigger_mm must >= push_target_mm";
        return false;
    }
    if (cfg.approach_speed_ratio < kMinVRatio || cfg.approach_speed_ratio > kMaxVRatio)
    {
        last_error_ = "approach_speed_ratio out of range";
        return false;
    }
    if (cfg.dwell_between_ms < kMinDwellMs || cfg.dwell_between_ms > kMaxDwellMs)
    {
        last_error_ = "dwell_between_ms out of range";
        return false;
    }

    // 备份 axis1 当前 v_limit，实验结束时恢复。
    // 主循环平时不读 v_limit（read_v_limit 仅在启动序列里调用），所以这里主动读一次拿到 PLC 真值。
    // 实验期间用 v_limit_snapshot_[7] 作底版，避免每拍 ADS 读。
    if (ctx.plc_v_limit != nullptr && ctx.ads != nullptr)
    {
        plc_io::read_v_limit(ctx);
        std::memcpy(v_limit_snapshot_, ctx.plc_v_limit, sizeof(v_limit_snapshot_));
        v_limit_backup_ = ctx.plc_v_limit[0];
        v_limit_backed_up_ = true;
    }
    else
    {
        for (int i = 0; i < 7; ++i) v_limit_snapshot_[i] = 0.0;
        v_limit_backup_ = 0.0;
        v_limit_backed_up_ = false;
    }

    active_cfg_ = cfg;
    velocity_level_ = 0;
    repeat_in_level_ = 0;
    global_trial_id_ = 0;
    current_v_ratio_ = 0.0;
    axis1_target_pos_ = active_cfg_.start_pos_mm;
    axis1_refer_cmd_ = ctx.plc_act_pos != nullptr ? ctx.plc_act_pos[0] : 0.0;
    v_limit_axis1_cmd_ = v_limit_backup_;
    v_limit_override_active_ = false;
    fast_return_request_ = false;
    plan_return_requested_ = false;
    cal_zeroed_drop_count_ = 0;

    enter_phase(FtExpPhase::ArmWait, 0);
    return true;
}

void ForceTransitionExperiment::abort(AppContext& ctx, const char* reason)
{
    if (phase_ == FtExpPhase::Idle)
        return;
    last_error_ = (reason != nullptr) ? reason : "abort";

    release_axis1_overrides(ctx);

    phase_ = FtExpPhase::Abort;
    fast_return_request_ = false;
    v_limit_override_active_ = false;
}

void ForceTransitionExperiment::release_axis1_overrides(AppContext& ctx)
{
    // 清掉 axis1 的计划回退请求（即使没下发，调用也安全 —— 写入 false 即可）。
    if (ctx.ads != nullptr)
    {
        plc_io::clear_axis_return_request(ctx, AdsSymbol::axis1_return);
    }
    plan_return_requested_ = false;

    // 还原 v_limit[0]。使用 start 时拍下的 v_limit_snapshot_ 快照作底版，仅替换 axis1。
    if (v_limit_backed_up_ && ctx.ads != nullptr)
    {
        double v_limit_local[7];
        std::memcpy(v_limit_local, v_limit_snapshot_, sizeof(v_limit_local));
        v_limit_local[0] = v_limit_backup_;
        plc_io::write_v_limit(ctx, v_limit_local);
    }
    v_limit_override_active_ = false;
    fast_return_request_ = false;
}

void ForceTransitionExperiment::fill_v_limit_override(double out_v_limit[7]) const
{
    std::memcpy(out_v_limit, v_limit_snapshot_, sizeof(double) * 7);
    out_v_limit[0] = v_limit_axis1_cmd_;
}

bool ForceTransitionExperiment::axis1_pos_abs(const AppContext& ctx, double& out_mm) const
{
    if (ctx.plc_act_pos == nullptr)
        return false;
    out_mm = ctx.plc_act_pos[0];
    return true;
}

void ForceTransitionExperiment::enter_phase(FtExpPhase next, std::uint32_t now_tick_ms)
{
    phase_ = next;
    phase_t0_ms_ = now_tick_ms;
}

bool ForceTransitionExperiment::tick(
    AppContext& ctx,
    std::uint32_t now_tick_ms,
    bool control_active,
    bool freeze_active,
    bool estop_hold_active,
    bool cal_zeroed,
    GuidewireMode guidewire_mode)
{
    if (phase_ == FtExpPhase::Idle || phase_ == FtExpPhase::Done || phase_ == FtExpPhase::Abort)
        return false;

    // 安全边沿：任何一个掉边 → 立即 Abort 安全收尾。
    if (!control_active)
    {
        abort(ctx, "control_active=false");
        return false;
    }
    if (estop_hold_active)
    {
        abort(ctx, "estop_hold");
        return false;
    }
    // cal_zeroed 给一个 3 拍宽限：避免 ADS/DAQ 抖动或未来加入的"重置标定"流程
    // 触发短暂 false → 误终止实验。连续 3 拍掉边才 Abort。
    if (!cal_zeroed)
    {
        if (++cal_zeroed_drop_count_ >= 3)
        {
            abort(ctx, "cal not zeroed");
            return false;
        }
    }
    else
    {
        cal_zeroed_drop_count_ = 0;
    }
    if (guidewire_mode != GuidewireMode::None)
    {
        abort(ctx, "guidewire_mode changed (catheter required)");
        return false;
    }
    // freeze_active 自身不是错误：实验自己会在 FastReturn 段触发它。
    // 只有在 PushForward / ApproachStart 段意外冻结才算异常。
    if (freeze_active && (phase_ == FtExpPhase::PushForward || phase_ == FtExpPhase::ApproachStart || phase_ == FtExpPhase::ArmWait))
    {
        abort(ctx, "external freeze during push");
        return false;
    }

    double axis1_abs = 0.0;
    if (!axis1_pos_abs(ctx, axis1_abs))
    {
        abort(ctx, "axis1 pos unavailable");
        return false;
    }

    const std::uint32_t elapsed = now_tick_ms - phase_t0_ms_;

    switch (phase_)
    {
    case FtExpPhase::ArmWait:
    {
        // 静置等传感器稳定。
        axis1_refer_cmd_ = axis1_abs;
        v_limit_override_active_ = false;
        fast_return_request_ = false;
        if (elapsed >= static_cast<std::uint32_t>(active_cfg_.dwell_between_ms))
        {
            // 推进到 ApproachStart：用安全速度接近起点。
            current_v_ratio_ = active_cfg_.approach_speed_ratio;
            axis1_target_pos_ = active_cfg_.start_pos_mm;
            enter_phase(FtExpPhase::ApproachStart, now_tick_ms);
        }
        break;
    }

    case FtExpPhase::ApproachStart:
    {
        axis1_target_pos_ = active_cfg_.start_pos_mm;
        axis1_refer_cmd_ = active_cfg_.start_pos_mm;
        if (v_limit_backed_up_)
        {
            v_limit_axis1_cmd_ = v_limit_backup_ * active_cfg_.approach_speed_ratio;
            v_limit_override_active_ = true;
        }
        fast_return_request_ = false;
        if (std::fabs(axis1_abs - active_cfg_.start_pos_mm) < kStartPosTolMm)
        {
            // 切到 PushForward：进入正式记录段。
            current_v_ratio_ = clamp01(active_cfg_.v_ratio[velocity_level_], kMinVRatio, kMaxVRatio);
            axis1_target_pos_ = active_cfg_.push_target_mm;
            enter_phase(FtExpPhase::PushForward, now_tick_ms);
            ++global_trial_id_;
        }
        break;
    }

    case FtExpPhase::PushForward:
    {
        axis1_target_pos_ = active_cfg_.push_target_mm;
        axis1_refer_cmd_ = active_cfg_.push_target_mm;
        if (v_limit_backed_up_)
        {
            v_limit_axis1_cmd_ = v_limit_backup_ * current_v_ratio_;
            v_limit_override_active_ = true;
        }
        fast_return_request_ = false;
        if (axis1_abs >= active_cfg_.return_trigger_mm - kPushArriveTolMm)
        {
            enter_phase(FtExpPhase::ReachTriggerPos, now_tick_ms);
        }
        break;
    }

    case FtExpPhase::ReachTriggerPos:
    {
        // 锁 refer 在触发点；下发 PLC 计划回退；置 axis1_fast_return 让冻结链路接管。
        axis1_refer_cmd_ = axis1_abs;
        v_limit_override_active_ = false;
        fast_return_request_ = true;

        if (!plan_return_requested_)
        {
            const ControlConfig& cfg = *ctx.cfg;
            bool ok = plc_io::request_axis_return(
                ctx,
                AdsSymbol::axis1_return,
                active_cfg_.start_pos_mm,
                cfg.axis1_return_velocity_mm_s,
                cfg.axis1_return_acc_mm_s2,
                cfg.axis1_return_dec_mm_s2,
                cfg.axis1_return_jerk_mm_s3);
            if (!ok)
            {
                abort(ctx, "request_axis_return failed");
                return false;
            }
            plan_return_requested_ = true;
        }
        enter_phase(FtExpPhase::FastReturn, now_tick_ms);
        break;
    }

    case FtExpPhase::FastReturn:
    {
        axis1_refer_cmd_ = axis1_abs;
        v_limit_override_active_ = false;
        fast_return_request_ = true;

        AxisReturnStatus status{};
        if (!plc_io::read_axis_return_status(ctx, AdsSymbol::axis1_return, status))
        {
            abort(ctx, "read_axis_return_status failed");
            return false;
        }
        if (status.error)
        {
            abort(ctx, "PLC axis_return error");
            return false;
        }
        if (status.done)
        {
            enter_phase(FtExpPhase::ReturnWaitDone, now_tick_ms);
        }
        else if (elapsed > 15000) // 15s 超时保护
        {
            abort(ctx, "axis_return timeout");
            return false;
        }
        break;
    }

    case FtExpPhase::ReturnWaitDone:
    {
        // 单拍：清 Req、清快退标志、还原 v_limit（用 start 时快照作底版）。
        plc_io::clear_axis_return_request(ctx, AdsSymbol::axis1_return);
        plan_return_requested_ = false;
        fast_return_request_ = false;
        if (v_limit_backed_up_)
        {
            double v_limit_local[7];
            std::memcpy(v_limit_local, v_limit_snapshot_, sizeof(v_limit_local));
            v_limit_local[0] = v_limit_backup_;
            plc_io::write_v_limit(ctx, v_limit_local);
        }
        v_limit_override_active_ = false;
        axis1_refer_cmd_ = axis1_abs;
        enter_phase(FtExpPhase::SettleHold, now_tick_ms);
        break;
    }

    case FtExpPhase::SettleHold:
    {
        axis1_refer_cmd_ = axis1_abs;
        v_limit_override_active_ = false;
        fast_return_request_ = false;
        if (elapsed >= static_cast<std::uint32_t>(active_cfg_.dwell_between_ms))
        {
            enter_phase(FtExpPhase::AdvanceTrial, now_tick_ms);
        }
        break;
    }

    case FtExpPhase::AdvanceTrial:
    {
        ++repeat_in_level_;
        if (repeat_in_level_ >= active_cfg_.repeats_per_level)
        {
            repeat_in_level_ = 0;
            ++velocity_level_;
        }
        if (velocity_level_ >= active_cfg_.num_levels)
        {
            // 全部完成：恢复 v_limit + 结束。
            release_axis1_overrides(ctx);
            phase_ = FtExpPhase::Done;
            return false;
        }
        enter_phase(FtExpPhase::ApproachStart, now_tick_ms);
        break;
    }

    default:
        break;
    }

    return true;
}

void ForceTransitionExperiment::set_param_a(int field_id, int int_val)
{
    switch (field_id)
    {
    case 0:
        if (int_val >= 2 && int_val <= ForceTransitionConfig::kMaxLevels)
            pending_cfg_.num_levels = int_val;
        break;
    case 1:
        if (int_val >= 1 && int_val <= kMaxRepeats)
            pending_cfg_.repeats_per_level = int_val;
        break;
    case 2:
        if (int_val >= static_cast<int>(kMinDwellMs) && int_val <= kMaxDwellMs)
            pending_cfg_.dwell_between_ms = int_val;
        break;
    default:
        break;
    }
}

void ForceTransitionExperiment::set_param_b(int field_id, int fixed_x1000)
{
    const double v = static_cast<double>(fixed_x1000) / 1000.0;
    if (field_id >= 10 && field_id <= 15)
    {
        const int idx = field_id - 10;
        if (idx >= 0 && idx < ForceTransitionConfig::kMaxLevels)
            pending_cfg_.v_ratio[idx] = v;
        return;
    }
    switch (field_id)
    {
    case 20: pending_cfg_.start_pos_mm = v; break;
    case 21: pending_cfg_.push_target_mm = v; break;
    case 22: pending_cfg_.return_trigger_mm = v; break;
    case 23: pending_cfg_.approach_speed_ratio = v; break;
    default: break;
    }
}
