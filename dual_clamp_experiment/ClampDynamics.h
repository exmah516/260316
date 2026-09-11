#pragma once
#include "ForceCalibration.h"
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace clampdynamics {
inline constexpr const char* kVersion = "inertia-feedback-v2-m025";
struct Config {
    double mass_kg = 0.025;
    double axial_sign = 1.0;
    double installation_gain = forcecal::kInstallationAxialGain;
    bool validation_mode = false;
    bool conditions_confirmed = false;
    double max_dt_s = 0.003;
};
inline constexpr Config kCatheter{};
inline constexpr Config kGuidewire{};
inline const Config& config(bool guidewire) { return guidewire ? kGuidewire : kCatheter; }
inline bool valid_config(const Config& c) {
    return c.mass_kg == 0.025 && (c.axial_sign == 1.0 || c.axial_sign == -1.0) &&
        std::isfinite(c.installation_gain) && c.installation_gain > 0 &&
        std::isfinite(c.max_dt_s) && c.max_dt_s > 0 &&
        (!c.validation_mode || c.conditions_confirmed);
}
struct Input {
    double time_s = 0, velocity_mm_s = 0, moving_cmd = 0, fixed_cmd = 0;
    int phase = 0, cycle = 0;
    double acceleration_mm_s2 = std::numeric_limits<double>::quiet_NaN();
    bool force_valid = true;
    std::uint64_t sample_index = std::numeric_limits<std::uint64_t>::max();
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
        pending_reset_ = reason;
    }
    Result update(const Input& in, const Config& c) {
        Result r;
        r.acceleration_mm_s2 = in.acceleration_mm_s2;
        if (!valid_config(c)) return invalid(r, "invalid_config");
        if (initialized_ && (c.axial_sign != config_.axial_sign ||
            c.validation_mode != config_.validation_mode || c.installation_gain != config_.installation_gain ||
            c.conditions_confirmed != config_.conditions_confirmed || c.max_dt_s != config_.max_dt_s))
            reset("configuration_changed");
        if (!std::isfinite(in.time_s)) return invalid(r, "invalid_time");
        if (has_time_ && in.time_s <= time_) return invalid(r, "nonincreasing_time");
        const bool gap = has_time_ && (in.time_s - time_ > c.max_dt_s + 1e-12 ||
            (in.sample_index != no_index && index_ != no_index && in.sample_index != index_ + 1));
        time_ = in.time_s; index_ = in.sample_index; has_time_ = true;
        if (gap) return invalid(r, "sample_gap");
        if (!in.force_valid) return invalid(r, "zero_or_force_invalid");
        if (!std::isfinite(in.acceleration_mm_s2)) return invalid(r, "invalid_acceleration");
        if (!std::isfinite(in.moving_cmd) || !std::isfinite(in.fixed_cmd) ||
            in.phase < 0 || in.phase > 12 || in.cycle < 0) return invalid(r, "invalid_operation");
        r.gate = operation_.update(in, c.max_dt_s);
        if (c.validation_mode) r.gate = true;
        // 只作单位换算和线性标定增量转换，静态截距不得再次加入。
        r.acceleration_m_s2 = in.acceleration_mm_s2 * 0.001;
        r.sensor_prediction_N = c.axial_sign * c.mass_kg * r.acceleration_m_s2;
        r.display_prediction_N = c.installation_gain * r.sensor_prediction_N;
        if (!std::isfinite(r.display_prediction_N)) return invalid(r, "nonfinite_prediction");
        r.fn_N = r.gate ? r.display_prediction_N : 0;
        r.valid = true;
        r.status = "computed_physics_unverified";
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
        << "\",\"parameter_source\":\"assumed_sensor_load_path\",\"cross_mode_preview\":false"
        << ",\"force_definition\":\"installed_delta_N; no decoupling\""
        << ",\"formula\":\"d_sensor=s*0.025*a_mm_s2*0.001; d_display=gain*d_sensor; fn_processed=fn_original-gate*d_display\""
        << ",\"mass_kg\":" << c.mass_kg << ",\"axial_sign\":" << c.axial_sign
        << ",\"axial_sign_verified\":false,\"physics_verified\":false"
        << ",\"installation_axial_gain\":" << c.installation_gain
        << ",\"disturbance_intercept_N\":0,\"ft_compensation_enabled\":false"
        << ",\"validation_mode\":" << (c.validation_mode ? "true" : "false")
        << ",\"conditions_confirmed_by_operator\":" << (c.conditions_confirmed ? "true" : "false")
        << ",\"acceleration_source\":\"NcToPlc.ActAcc\",\"acceleration_processing\":\"direct_feedback_no_added_filter\""
        << ",\"input_acceleration_unit\":\"mm/s^2\",\"used_acceleration_unit\":\"m/s^2\",\"unit_scale\":0.001"
        << ",\"feedback_characteristics_verified\":false,\"velocity_fallback\":false,\"time_shift_s\":0"
        << ",\"time_basis\":\"PLC sample_index * nominal 1000 us; not a hardware acquisition timestamp\""
        << ",\"physical_time_alignment_verified\":false,\"max_dt_s\":" << c.max_dt_s
        << ",\"gate_definition\":\"normal: unchanged operation gate; validation: entire record\""
        << ",\"command_magnitude_used\":false,\"rotation_used\":false,\"force_input_used\":false"
        << ",\"spike_rejection\":false,\"assumptions\":\"25 g component follows base; axial inertia passes through sensor; both sides assumed independently\"}";
    return out.str();
}
inline std::string snapshot(bool guidewire) { return snapshot(guidewire, config(guidewire)); }
}
