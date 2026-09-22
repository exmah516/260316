#pragma once
#include "ForceCalibration.h"
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace clampdynamics {
inline constexpr const char* kVersion = "motion-only-axis1-v4-20260920";
struct Config {
    // Frozen axis1/fn1 OLS gains, not identified physical masses.
    double beta_a = 0.06826911006047173;
    double beta_v = 0.07362809525467874;
    double beta_s = -0.0012146051548365536;
    double beta_0 = -0.0027542992032783954; // Provenance only; never subtracted.
    double vel_deadband_m_s = 0.0002;
    double axial_sign = -1.0;
    double mass_kg = 0.025; // Legacy metadata, unused.
    double installation_gain = forcecal::kInstallationAxialGain;
    // Legacy protocol fields: inverse-load reconstruction is unsupported.
    bool reconstruct_external = false;
    double alpha = 1.0, beta_load = 0.0, step_bias = 0.0, blanking_window_s = 0.0;
    bool validation_mode = false, conditions_confirmed = false;
    double max_dt_s = 0.003;
    bool motion_model_available = true;
};
inline constexpr Config kCatheter{};
inline constexpr Config make_guidewire_config() {
    Config c{};
    c.motion_model_available = false;
    return c;
}
inline constexpr Config kGuidewire = make_guidewire_config();
inline const Config& config(bool guidewire) { return guidewire ? kGuidewire : kCatheter; }
inline bool valid_config(const Config& c) {
    return std::isfinite(c.beta_a) && c.beta_a >= 0 && std::isfinite(c.beta_v) &&
        std::isfinite(c.beta_s) && std::isfinite(c.beta_0) &&
        std::isfinite(c.vel_deadband_m_s) && c.vel_deadband_m_s >= 0 &&
        (c.axial_sign == 1.0 || c.axial_sign == -1.0) &&
        std::isfinite(c.mass_kg) && c.mass_kg > 0 && c.mass_kg <= 0.2 &&
        !c.reconstruct_external && c.alpha == 1.0 && c.beta_load == 0.0 &&
        c.step_bias == 0.0 && c.blanking_window_s == 0.0 &&
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
    double force_cal_delta_N = 0.0, ft_cal_delta_N = 0.0;
};

// For the separately labelled illustration only, not the motion predictor.
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
        pending_reset_ = reason;
    }
    Result update(const Input& in, const Config& c) {
        Result r;
        r.acceleration_mm_s2 = in.acceleration_mm_s2;
        if (!valid_config(c)) return invalid(r, "invalid_config");
        if (!c.motion_model_available) return invalid(r, "axis_model_unavailable");
        if (initialized_ && (c.axial_sign != config_.axial_sign ||
            c.beta_a != config_.beta_a || c.beta_v != config_.beta_v ||
            c.beta_s != config_.beta_s || c.beta_0 != config_.beta_0 ||
            c.vel_deadband_m_s != config_.vel_deadband_m_s ||
            c.validation_mode != config_.validation_mode ||
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
        if (!in.force_valid || !std::isfinite(in.force_cal_delta_N) ||
            !std::isfinite(in.ft_cal_delta_N)) return invalid(r, "zero_or_force_invalid");
        if (!std::isfinite(in.acceleration_mm_s2)) return invalid(r, "invalid_acceleration");
        if (!std::isfinite(in.velocity_mm_s)) return invalid(r, "invalid_velocity");
        if (!std::isfinite(in.moving_cmd) || !std::isfinite(in.fixed_cmd) ||
            in.phase < 0 || in.phase > 12 || in.cycle < 0) return invalid(r, "invalid_operation");
        const double a = c.axial_sign * in.acceleration_mm_s2 * .001;
        const double v = c.axial_sign * in.velocity_mm_s * .001;
        const double direction = v > c.vel_deadband_m_s ? 1.0 : v < -c.vel_deadband_m_s ? -1.0 : 0.0;
        const double motion = c.beta_a*a + c.beta_v*v + c.beta_s*direction;
        if (!std::isfinite(motion)) return invalid(r, "nonfinite_prediction");
        r.acceleration_m_s2 = a;
        r.fn_N = r.display_prediction_N = motion;
        r.sensor_prediction_N = motion/c.installation_gain;
        r.valid = r.gate = true;
        r.status = (in.phase == 4 || in.phase == 9) ? "delivery_motion_only" :
                   "mechanism_motion_only_not_external_force";
        r.reset_reason = pending_reset_;
        pending_reset_ = "none";
        config_ = c; initialized_ = true;
        return r;
    }
private:
    Result invalid(Result r, const char* reason) {
        initialized_ = false;
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
    const char* pending_reset_ = "initial";
};
inline std::string snapshot(bool guidewire, const Config& c, bool external_validation = false) {
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\"version\":\"" << kVersion << "\",\"experimental\":true,\"available\":"
        << (valid_config(c) && c.motion_model_available ? "true" : "false")
        << ",\"application_mode\":\"" << (external_validation ? "external_validation" : guidewire ? "guidewire" : "catheter")
        << "\",\"parameter_source\":\"frozen_stage1_OLS_axis1_fn1\",\"cross_mode_preview\":false"
        << ",\"force_definition\":\"installed_delta_N minus predicted motion; not distal force\""
        << ",\"formula\":\"d=beta_a*a+beta_v*v+beta_s*sgn(v); F_corr=F_raw-d\""
        << ",\"beta_a\":" << c.beta_a << ",\"beta_v\":" << c.beta_v
        << ",\"beta_s\":" << c.beta_s << ",\"beta_0\":" << c.beta_0
        << ",\"intercept_subtracted\":false,\"mass_kg\":" << c.mass_kg
        << ",\"axial_sign\":" << c.axial_sign << ",\"axial_sign_verified\":false"
        << ",\"physics_verified\":false,\"reconstruct_external\":false"
        << ",\"alpha\":1,\"beta_load\":0,\"step_bias\":0,\"blanking_window_s\":0"
        << ",\"installation_axial_gain\":" << c.installation_gain
        << ",\"validation_mode\":" << (c.validation_mode ? "true" : "false")
        << ",\"conditions_confirmed_by_operator\":" << (c.conditions_confirmed ? "true" : "false")
        << ",\"acceleration_source\":\"NcToPlc.ActAcc\",\"acceleration_processing\":\"direct_feedback_no_added_filter\""
        << ",\"input_acceleration_unit\":\"mm/s^2\",\"used_acceleration_unit\":\"m/s^2\",\"unit_scale\":0.001"
        << ",\"feedback_characteristics_verified\":false,\"velocity_fallback\":false,\"time_shift_s\":0"
        << ",\"physical_time_alignment_verified\":false,\"max_dt_s\":" << c.max_dt_s
        << ",\"gate_definition\":\"all valid phases; no blanking; phases outside 4/9 are mechanism response\""
        << ",\"force_input_used_for_prediction\":false,\"ft_compensation_enabled\":false"
        << ",\"spike_rejection\":false,\"prediction_input\":\"original_installed_delta_N\""
        << ",\"haptic_feedback_enabled\":false,\"invalid_corrected_csv\":\"empty\"}";
    return out.str();
}
inline std::string snapshot(bool guidewire) { return snapshot(guidewire, config(guidewire)); }
}
