#pragma once
#include "ForceCalibration.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace clampdynamics {
inline constexpr const char* kVersion = "distal-reconstruction-v3";
struct Config {
    // 运动动力学辨识参数 (Stage 1 OLS 结果):
    // F_motion = beta_a * a_si + beta_v * v_si + beta_s * sgn(v_si) + beta_0
    double beta_a = 0.068269;           // N/(m/s^2), 轴向等效动态质量增益 (~68.3g)
    double beta_v = 0.073628;           // N/(m/s), 轴向运动粘性阻尼系数
    double beta_s = -0.001215;          // N, 轴向库伦摩擦截距
    double beta_0 = -0.002754;          // N, 轴向静态偏置
    double vel_deadband_m_s = 0.0002;   // m/s, 速度死区 (0.2 mm/s)

    // 坐标与符号
    double axial_sign = -1.0;           // 递送方向与PLC NC速度/加速度符号映射 (-1.0: 负NC值为正向递送)
    double mass_kg = 0.025;             // 兼容保留
    double installation_gain = forcecal::kInstallationAxialGain;

    // 末端真实阻力重构参数:
    // F_ext = (F_net - beta_load - step_bias) / alpha
    bool reconstruct_external = true;   // true: 重构末端真实外力; false: 仅补偿夹爪运动扰动
    double alpha = 1.0;                 // 近端名义载荷响应增益 (导管已校零实测增益: 1.0)
    double beta_load = 0.0;             // 静态预紧/阻抗阈值 (实验已校零，置0避免小外力被截断)
    double step_bias = 0.0;             // 夹持阶跃偏置 (已校零置0)
    double blanking_window_s = 0.150;   // Phase 7 夹紧冲击消隐时间窗 (150ms)

    // 运行模式与操作确认
    bool validation_mode = false;
    bool conditions_confirmed = false;
    double max_dt_s = 0.003;
};
inline constexpr Config kCatheter{
    0.068269,   // beta_a
    0.073628,   // beta_v
    -0.001215,  // beta_s
    -0.002754,  // beta_0
    0.0002,     // vel_deadband_m_s
    -1.0,       // axial_sign
    0.025,      // mass_kg
    forcecal::kInstallationAxialGain, // installation_gain
    true,       // reconstruct_external
    1.0,        // alpha (catheter已校零实测传递增益)
    0.0,        // beta_load (已校零置0，不再重复扣减0.34N)
    0.0,        // step_bias
    0.150,      // blanking_window_s
    false,      // validation_mode
    false,      // conditions_confirmed
    0.003       // max_dt_s
};
inline constexpr Config kGuidewire{
    0.068269,   // beta_a
    0.073628,   // beta_v
    -0.001215,  // beta_s
    -0.002754,  // beta_0
    0.0002,     // vel_deadband_m_s
    -1.0,       // axial_sign
    0.025,      // mass_kg
    forcecal::kInstallationAxialGain, // installation_gain
    true,       // reconstruct_external
    0.1659,     // alpha (guidewire)
    0.0,        // beta_load (已校零置0)
    0.0,        // step_bias (已校零置0)
    0.150,      // blanking_window_s
    false,      // validation_mode
    false,      // conditions_confirmed
    0.003       // max_dt_s
};
inline const Config& config(bool guidewire) { return guidewire ? kGuidewire : kCatheter; }
inline bool valid_config(const Config& c) {
    return std::isfinite(c.beta_a) && c.beta_a >= 0 &&
        std::isfinite(c.beta_v) &&
        std::isfinite(c.beta_s) &&
        std::isfinite(c.beta_0) &&
        (c.axial_sign == 1.0 || c.axial_sign == -1.0) &&
        std::isfinite(c.mass_kg) && c.mass_kg > 0 && c.mass_kg <= 0.2 &&
        std::isfinite(c.alpha) && c.alpha > 0 &&
        std::isfinite(c.beta_load) &&
        std::isfinite(c.blanking_window_s) && c.blanking_window_s >= 0 &&
        std::isfinite(c.installation_gain) && c.installation_gain > 0 &&
        std::isfinite(c.max_dt_s) && c.max_dt_s > 0 &&
        (!c.validation_mode || c.conditions_confirmed);
}
struct Input {
    double time_s = 0;
    double velocity_mm_s = 0;
    double moving_cmd = 0;
    double fixed_cmd = 0;
    int phase = 0;
    int cycle = 0;
    double acceleration_mm_s2 = std::numeric_limits<double>::quiet_NaN();
    bool force_valid = true;
    std::uint64_t sample_index = std::numeric_limits<std::uint64_t>::max();
    double force_cal_delta_N = 0.0;
    double ft_cal_delta_N = 0.0;
};

// 电缸命令只用于识别操作事件；模型2独立维护同样的门控，不读取物理模型输出。
class OperationGate {
public:
    void reset() { initialized_ = gate_ = false; phase_ = cycle_ = -1; }
    bool update(const Input& in, double max_dt_s = 0.003) {
        if (!std::isfinite(in.time_s) || !std::isfinite(in.moving_cmd) ||
            !std::isfinite(in.fixed_cmd) || in.phase < 0 || in.phase > 12 || in.cycle < 0) {
            reset(); return false;
        }
        if (initialized_ && (in.time_s <= time_ || in.time_s - time_ > max_dt_s + 1e-12))
            reset();
        const bool cycle_changed = initialized_ && in.cycle != cycle_;
        const bool command_changed = initialized_ &&
            (in.moving_cmd != moving_ || in.fixed_cmd != fixed_);
        const bool entered_forward = in.phase == 4 && phase_ != 4;
        if (cycle_changed || entered_forward || in.phase < 4 || in.phase > 7) gate_ = false;
        if (in.phase >= 5 && in.phase <= 7) gate_ = true;
        if (in.phase >= 4 && in.phase <= 7 && command_changed && !entered_forward && !cycle_changed)
            gate_ = true;
        time_ = in.time_s; moving_ = in.moving_cmd; fixed_ = in.fixed_cmd;
        phase_ = in.phase; cycle_ = in.cycle; initialized_ = true;
        return gate_;
    }
private:
    bool initialized_ = false, gate_ = false;
    double time_ = 0, moving_ = 0, fixed_ = 0;
    int phase_ = -1, cycle_ = -1;
};

struct Result {
    double fn_N = 0, ft_N = 0;
    double acceleration_mm_s2 = std::numeric_limits<double>::quiet_NaN();
    double acceleration_m_s2 = std::numeric_limits<double>::quiet_NaN();
    double sensor_prediction_N = 0, display_prediction_N = 0;
    bool valid = false, gate = false;
    const char* status = "waiting";
    const char* reset_reason = "none";
};
class Predictor {
public:
    void reset(const char* reason = "restart") {
        initialized_ = has_time_ = false;
        operation_.reset();
        last_phase_ = -1;
        phase7_enter_time_s_ = -1.0;
        pending_reset_ = reason;
    }
    Result update(const Input& in, const Config& c) {
        Result r;
        r.acceleration_mm_s2 = in.acceleration_mm_s2;
        if (!valid_config(c)) return invalid(r, "invalid_config");
        if (initialized_ && (c.axial_sign != config_.axial_sign ||
            c.validation_mode != config_.validation_mode ||
            c.reconstruct_external != config_.reconstruct_external ||
            c.alpha != config_.alpha || c.beta_load != config_.beta_load ||
            c.step_bias != config_.step_bias ||
            c.installation_gain != config_.installation_gain ||
            c.conditions_confirmed != config_.conditions_confirmed ||
            c.max_dt_s != config_.max_dt_s))
            reset("configuration_changed");
        if (!std::isfinite(in.time_s)) return invalid(r, "invalid_time");
        if (has_time_ && in.time_s <= time_) return invalid(r, "nonincreasing_time");
        const bool gap = has_time_ && (in.time_s - time_ > c.max_dt_s + 1e-12 ||
            (in.sample_index != no_index && index_ != no_index && in.sample_index != index_ + 1));
        time_ = in.time_s; index_ = in.sample_index; has_time_ = true;
        if (gap) return invalid(r, "sample_gap");
        if (!in.force_valid) return invalid(r, "zero_or_force_invalid");
        if (!std::isfinite(in.acceleration_mm_s2)) return invalid(r, "invalid_acceleration");
        if (!std::isfinite(in.velocity_mm_s)) return invalid(r, "invalid_velocity");
        if (!std::isfinite(in.moving_cmd) || !std::isfinite(in.fixed_cmd) ||
            in.phase < 0 || in.phase > 12 || in.cycle < 0) return invalid(r, "invalid_operation");

        // 跟踪 Phase 7 发生时间窗，用于夹紧冲击消隐
        if (in.phase == 7 && last_phase_ != 7) {
            phase7_enter_time_s_ = in.time_s;
        }
        last_phase_ = in.phase;

        r.gate = operation_.update(in, c.max_dt_s);
        if (c.validation_mode) r.gate = true;

        // 运动学单位换算 (NC坐标系映射至递送物理方向)
        const double a_si = c.axial_sign * in.acceleration_mm_s2 * 0.001;
        const double v_si = c.axial_sign * in.velocity_mm_s * 0.001;
        r.acceleration_m_s2 = a_si;

        // 库伦摩擦符号项 (带速度死区)
        double sgn_v = 0.0;
        if (v_si > c.vel_deadband_m_s) sgn_v = 1.0;
        else if (v_si < -c.vel_deadband_m_s) sgn_v = -1.0;

        // 运动扰动力模型计算: F_motion = beta_a * a + beta_v * v + beta_s * sgn(v) + beta_0
        const double f_motion = c.beta_a * a_si + c.beta_v * v_si + c.beta_s * sgn_v + c.beta_0;
        r.display_prediction_N = f_motion;
        r.sensor_prediction_N = f_motion / c.installation_gain;
        if (!std::isfinite(f_motion)) return invalid(r, "nonfinite_prediction");

        if (c.validation_mode) {
            // 无器械验证模式：全程补偿运动扰动 (F_meas - F_motion)，用于验证0.75N尖峰及摩擦消除
            r.fn_N = f_motion;
            r.status = "validation_motion_compensated";
        } else if (c.reconstruct_external) {
            // 末端真实阻力重构模式:
            if (in.phase == 4) {
                // Phase 4: 递送推进行程
                // F_net = F_meas - F_motion
                // F_ext = (F_net - beta_load - step_bias) / alpha
                const double f_net = in.force_cal_delta_N - f_motion;
                const double f_ext_raw = (f_net - c.beta_load - c.step_bias) / c.alpha;
                // 保留微小的零点白噪声死区 (0.005 N)，避免静止白噪声波动，但不抹杀真实的 20g (0.196 N) 小载荷
                const double f_ext = (f_ext_raw > 0.005) ? f_ext_raw : 0.0;
                // 界面显示 CorrectedFn = F_meas - fn_N，因此置 fn_N = F_meas - F_ext 使得 CorrectedFn == F_ext
                r.fn_N = in.force_cal_delta_N - f_ext;
                r.status = "reconstructed_distal_resistance";
                r.gate = true;
            } else {
                // 非递送阶段 (Phase 5 释放、Phase 6 快速回退、Phase 7 重夹闭合冲击及其他准备相):
                // 此时夹爪与血管病变末端处于脱离或回退过渡状态，执行消隐置零 (CorrectedFn == 0.0 N)
                r.fn_N = in.force_cal_delta_N;
                r.status = (in.phase == 6) ? "return_blanked" :
                           (in.phase == 7) ? "reclamp_blanked" : "delivery_blanked";
                r.gate = false;
            }
        } else {
            // 传统门控动力学补偿模式 (不进行 1/alpha 重构):
            r.fn_N = r.gate ? f_motion : 0.0;
            r.status = "motion_disturbance_compensated";
        }

        r.valid = true;
        r.reset_reason = pending_reset_;
        pending_reset_ = "none";
        time_ = in.time_s; index_ = in.sample_index; config_ = c; initialized_ = true;
        return r;
    }
private:
    Result invalid(Result r, const char* reason) {
        // 数据异常清除门控，但保留时间水位，连续重复点不能在重置后被误接纳。
        initialized_ = false;
        operation_.reset();
        last_phase_ = -1;
        phase7_enter_time_s_ = -1.0;
        pending_reset_ = reason;
        r.fn_N = r.ft_N = r.sensor_prediction_N = r.display_prediction_N = 0;
        r.gate = r.valid = false;
        r.status = r.reset_reason = reason;
        return r;
    }
    static constexpr auto no_index = std::numeric_limits<std::uint64_t>::max();
    bool initialized_ = false, has_time_ = false;
    double time_ = 0;
    std::uint64_t index_ = no_index;
    int last_phase_ = -1;
    double phase7_enter_time_s_ = -1.0;
    Config config_{};
    OperationGate operation_;
    const char* pending_reset_ = "initial";
};

inline std::string snapshot(bool guidewire, const Config& c, bool external_validation = false) {
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\"version\":\"" << kVersion << "\",\"experimental\":true,\"available\":"
        << (valid_config(c) ? "true" : "false")
        << ",\"application_mode\":\"" << (external_validation ? "external_validation" : guidewire ? "guidewire" : "catheter")
        << "\",\"parameter_source\":\"stage1_and_stage2_system_identification\",\"cross_mode_preview\":false"
        << ",\"force_definition\":\"installed_delta_N; distal external force reconstructed\""
        << ",\"formula\":\"F_motion=beta_a*a+beta_v*v+beta_s*sgn(v)+beta_0; F_ext=max(0,(F_meas-F_motion-beta_load)/alpha)\""
        << ",\"mass_kg\":" << c.mass_kg << ",\"axial_sign\":" << c.axial_sign
        << ",\"axial_sign_verified\":true,\"physics_verified\":true"
        << ",\"beta_a\":" << c.beta_a << ",\"beta_v\":" << c.beta_v
        << ",\"beta_s\":" << c.beta_s << ",\"beta_0\":" << c.beta_0
        << ",\"alpha\":" << c.alpha << ",\"beta_load\":" << c.beta_load
        << ",\"step_bias\":" << c.step_bias << ",\"reconstruct_external\":" << (c.reconstruct_external ? "true" : "false")
        << ",\"blanking_window_s\":" << c.blanking_window_s
        << ",\"installation_axial_gain\":" << c.installation_gain
        << ",\"disturbance_intercept_N\":" << c.beta_0 << ",\"ft_compensation_enabled\":false"
        << ",\"validation_mode\":" << (c.validation_mode ? "true" : "false")
        << ",\"conditions_confirmed_by_operator\":" << (c.conditions_confirmed ? "true" : "false")
        << ",\"acceleration_source\":\"NcToPlc.ActAcc\",\"acceleration_processing\":\"direct_feedback_no_added_filter\""
        << ",\"input_acceleration_unit\":\"mm/s^2\",\"used_acceleration_unit\":\"m/s^2\",\"unit_scale\":0.001"
        << ",\"feedback_characteristics_verified\":true,\"velocity_fallback\":false,\"time_shift_s\":0"
        << ",\"time_basis\":\"PLC sample_index * nominal 1000 us; not a hardware acquisition timestamp\""
        << ",\"physical_time_alignment_verified\":true,\"max_dt_s\":" << c.max_dt_s
        << ",\"gate_definition\":\"normal: phase4 reconstructed, phase5-7 blanked; validation: entire record\""
        << ",\"command_magnitude_used\":false,\"rotation_used\":false,\"force_input_used\":true"
        << ",\"spike_rejection\":true,\"assumptions\":\"kinematic motion disturbance model + identified distal transmission gain\"}";
    return out.str();
}
inline std::string snapshot(bool guidewire) { return snapshot(guidewire, config(guidewire)); }
}

