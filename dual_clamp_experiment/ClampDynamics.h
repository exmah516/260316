#pragma once

#include "ForceCalibration.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace clampdynamics {

inline constexpr const char* kVersion = "dual-channel-event-compensation-v2-catheter-validation-20261005";

enum class EventType : std::uint8_t {
    None = 0,
    Release = 1,
    Reclamp = 2
};

struct Config {
    double beta_a = 0.06826911006047173;
    double beta_v = 0.07362809525467874;
    double beta_s = -0.0012146051548365536;
    double beta_0 = -0.0027542992032783954;
    double vel_deadband_m_s = 0.0002;
    double axial_sign = -1.0;
    double mass_kg = 0.025;
    double installation_gain = forcecal::kInstallationAxialGain;

    bool reconstruct_external = false;
    double alpha = 1.0, beta_load = 0.0, step_bias = 0.0, blanking_window_s = 0.0;
    bool validation_mode = false, conditions_confirmed = false;
    double max_dt_s = 0.003;
    bool motion_model_available = true;
    bool guidewire = false;

    // Identified separately from the recorded position/velocity pair.
    double acceleration_alignment_delay_s = 0.052;
    double velocity_alignment_delay_s = 0.008;
    std::uint16_t moving_open_word = 5;
    std::uint16_t moving_close_word = 600;

    // Catheter mechanics supplied by the operator. These remain metadata until
    // clamp displacement is recorded, because stiffness alone does not give force.
    double clamp_force_N = 11.56;
    double spring_constant_N_per_mm = 0.18;
    std::uint8_t opposing_spring_count = 2;
    double clamp_angle_deg = 0.0;
    double catheter_slip_load_N = 1.765197;

    // Exploratory E_C load-path fit; deliberately not used for live compensation.
    bool load_transfer_model_available = false;
    double load_transfer_alpha = 0.66726131;
    double load_transfer_beta_N = 0.19997443;
    double load_transfer_mae_N = 0.13804390;
    double load_transfer_r2 = 0.58242702;

    double release_delay_s = 0.200;
    double reclamp_delay_s = 0.085;
    double template_duration_s = 0.600;
    double template_frequency_hz = 12.0;
    double template_decay_s = 0.180;
    double template_rise_s = 0.012;
    double release_peak_N = 0.25;
    double reclamp_peak_N = 0.45;

    double low_frequency_start_s = 0.100;
    double low_frequency_duration_s = 0.400;
    double low_frequency_tau_s = 0.120;
    double low_frequency_limit_N = 0.600;

    double wavelet_short_s = 0.008;
    double wavelet_long_s = 0.032;
    double wavelet_reference_N = 0.080;
    double wavelet_min_gain = 0.50;
    double wavelet_max_gain = 2.50;
};

inline constexpr Config kCatheter{};

inline constexpr Config make_guidewire_config() {
    Config c{};
    c.guidewire = true;
    c.moving_close_word = 500;
    c.release_delay_s = 0.190;
    c.reclamp_delay_s = 0.255;
    c.release_peak_N = 0.20;
    c.reclamp_peak_N = 0.30;
    c.clamp_force_N = 0.0;
    c.spring_constant_N_per_mm = 0.0;
    c.opposing_spring_count = 0;
    c.clamp_angle_deg = 0.0;
    c.catheter_slip_load_N = 0.0;
    c.load_transfer_model_available = false;
    c.load_transfer_alpha = c.load_transfer_beta_N = c.load_transfer_mae_N = c.load_transfer_r2 = 0.0;
    return c;
}

inline constexpr Config kGuidewire = make_guidewire_config();
inline const Config& config(bool guidewire) { return guidewire ? kGuidewire : kCatheter; }

inline bool valid_config(const Config& c) {
    return std::isfinite(c.beta_a) && c.beta_a >= 0.0 &&
        std::isfinite(c.beta_v) && std::isfinite(c.beta_s) && std::isfinite(c.beta_0) &&
        std::isfinite(c.vel_deadband_m_s) && c.vel_deadband_m_s >= 0.0 &&
        (c.axial_sign == 1.0 || c.axial_sign == -1.0) &&
        std::isfinite(c.mass_kg) && c.mass_kg > 0.0 && c.mass_kg <= 0.2 &&
        !c.reconstruct_external && c.alpha == 1.0 && c.beta_load == 0.0 &&
        c.step_bias == 0.0 && c.blanking_window_s == 0.0 &&
        std::isfinite(c.installation_gain) && c.installation_gain > 0.0 &&
        std::isfinite(c.max_dt_s) && c.max_dt_s > 0.0 &&
        std::isfinite(c.acceleration_alignment_delay_s) && c.acceleration_alignment_delay_s >= 0.0 &&
        std::isfinite(c.velocity_alignment_delay_s) && c.velocity_alignment_delay_s >= 0.0 &&
        c.moving_open_word >= 5 && c.moving_close_word >= 5 &&
        std::isfinite(c.clamp_force_N) && c.clamp_force_N >= 0.0 &&
        std::isfinite(c.spring_constant_N_per_mm) && c.spring_constant_N_per_mm >= 0.0 &&
        c.opposing_spring_count <= 2 &&
        std::isfinite(c.clamp_angle_deg) &&
        std::isfinite(c.catheter_slip_load_N) && c.catheter_slip_load_N >= 0.0 &&
        std::isfinite(c.load_transfer_alpha) && c.load_transfer_alpha >= 0.0 &&
        std::isfinite(c.load_transfer_beta_N) &&
        std::isfinite(c.load_transfer_mae_N) && c.load_transfer_mae_N >= 0.0 &&
        std::isfinite(c.load_transfer_r2) && c.load_transfer_r2 >= 0.0 && c.load_transfer_r2 <= 1.0 &&
        std::isfinite(c.release_delay_s) && c.release_delay_s >= 0.0 &&
        std::isfinite(c.reclamp_delay_s) && c.reclamp_delay_s >= 0.0 &&
        std::isfinite(c.template_duration_s) && c.template_duration_s > 0.0 &&
        std::isfinite(c.template_frequency_hz) && c.template_frequency_hz > 0.0 &&
        std::isfinite(c.template_decay_s) && c.template_decay_s > 0.0 &&
        std::isfinite(c.template_rise_s) && c.template_rise_s > 0.0 &&
        std::isfinite(c.release_peak_N) && std::isfinite(c.reclamp_peak_N) &&
        std::isfinite(c.low_frequency_start_s) && c.low_frequency_start_s >= 0.0 &&
        std::isfinite(c.low_frequency_duration_s) && c.low_frequency_duration_s >= 0.0 &&
        std::isfinite(c.low_frequency_tau_s) && c.low_frequency_tau_s > 0.0 &&
        std::isfinite(c.low_frequency_limit_N) && c.low_frequency_limit_N >= 0.0 &&
        std::isfinite(c.wavelet_short_s) && c.wavelet_short_s > 0.0 &&
        std::isfinite(c.wavelet_long_s) && c.wavelet_long_s > c.wavelet_short_s &&
        std::isfinite(c.wavelet_reference_N) && c.wavelet_reference_N > 0.0 &&
        std::isfinite(c.wavelet_min_gain) && c.wavelet_min_gain > 0.0 &&
        std::isfinite(c.wavelet_max_gain) && c.wavelet_max_gain >= c.wavelet_min_gain &&
        (!c.validation_mode || c.conditions_confirmed);
}

struct Input {
    double time_s = 0.0, velocity_mm_s = 0.0, moving_cmd = 0.0, fixed_cmd = 0.0;
    int phase = 0, cycle = 0;
    double acceleration_mm_s2 = std::numeric_limits<double>::quiet_NaN();
    bool force_valid = true;
    std::uint64_t sample_index = std::numeric_limits<std::uint64_t>::max();
    double force_cal_delta_N = 0.0, ft_cal_delta_N = 0.0;
    double position_mm = 0.0;
    double angle_deg = 0.0;
};

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
    double time_ = 0.0, moving_ = 0.0, fixed_ = 0.0;
    int phase_ = -1, cycle_ = -1;
};

struct Result {
    double fn_N = 0.0, ft_N = 0.0;
    double acceleration_mm_s2 = std::numeric_limits<double>::quiet_NaN();
    double acceleration_m_s2 = std::numeric_limits<double>::quiet_NaN();
    double sensor_prediction_N = 0.0, display_prediction_N = 0.0;
    double motion_N = 0.0;
    double low_frequency_N = 0.0;
    double transient_N = 0.0;
    double wavelet_strength = 0.0;
    double event_relative_s = 0.0;
    int event_type = 0;
    bool valid = false, gate = false;
    const char* status = "waiting";
    const char* reset_reason = "none";
};

class Predictor {
public:
    void reset(const char* reason = "restart") {
        initialized_ = has_time_ = false;
        has_previous_command_ = has_previous_motion_ = has_previous_low_time_ = false;
        event_count_ = history_size_ = history_head_ = 0;
        low_residual_ = event_baseline_ = 0.0;
        pending_reset_ = reason;
        for (auto& event : events_) event = {};
    }

    Result update(const Input& in, const Config& c) {
        Result r;
        r.acceleration_mm_s2 = in.acceleration_mm_s2;
        if (!valid_config(c)) return invalid(r, "invalid_config");
        if (initialized_ && config_changed(c)) reset("configuration_changed");
        if (!std::isfinite(in.time_s)) return invalid(r, "invalid_time");
        if (has_time_ && in.time_s <= time_) return invalid(r, "nonincreasing_time");
        const bool gap = has_time_ && (in.time_s - time_ > c.max_dt_s + 1e-12 ||
            (in.sample_index != no_index && index_ != no_index && in.sample_index != index_ + 1));
        time_ = in.time_s;
        index_ = in.sample_index;
        has_time_ = true;
        if (gap) return invalid(r, "sample_gap");
        if (!in.force_valid || !std::isfinite(in.force_cal_delta_N) ||
            !std::isfinite(in.ft_cal_delta_N)) return invalid(r, "zero_or_force_invalid");
        if (!std::isfinite(in.acceleration_mm_s2) || !std::isfinite(in.velocity_mm_s))
            return invalid(r, "invalid_motion");
        if (!std::isfinite(in.moving_cmd) || !std::isfinite(in.fixed_cmd) ||
            in.phase < 0 || in.phase > 12 || in.cycle < 0)
            return invalid(r, "invalid_operation");

        const double a = c.axial_sign * in.acceleration_mm_s2 * 0.001;
        const double v = c.axial_sign * in.velocity_mm_s * 0.001;
        const double dt = has_previous_motion_ ? positive_dt(in.time_s - previous_motion_time_) : 0.001;
        const double aligned_a = has_previous_motion_ ?
            a + c.acceleration_alignment_delay_s * (a - previous_a_) / dt : a;
        const double aligned_v = has_previous_motion_ ?
            v + c.velocity_alignment_delay_s * (v - previous_v_) / dt : v;
        const double direction = aligned_v > c.vel_deadband_m_s ? 1.0 :
            aligned_v < -c.vel_deadband_m_s ? -1.0 : 0.0;
        const double motion = c.beta_a * aligned_a + c.beta_v * aligned_v + c.beta_s * direction;

        r.acceleration_m_s2 = aligned_a;
        r.motion_N = std::isfinite(motion) ? motion : 0.0;
        update_history(in.time_s, in.force_cal_delta_N - r.motion_N);
        detect_event(in, c, r.motion_N);
        const double wavelet = wavelet_strength(c);
        int current_type = 0;
        double current_relative_s = 0.0;
        const double transient = transient_compensation(in.time_s, c, wavelet,
            current_type, current_relative_s);
        const double low = low_frequency_compensation(in, c, r.motion_N);
        const bool delivery_forward = in.phase == 4 || in.phase == 9;
        const double final_compensation = delivery_forward ? 0.0 : r.motion_N + low + transient;

        r.low_frequency_N = delivery_forward ? 0.0 : low;
        r.transient_N = delivery_forward ? 0.0 : transient;
        r.wavelet_strength = wavelet;
        r.event_type = current_event_type(in.time_s, current_relative_s);
        if (r.event_type == 0) {
            r.event_type = current_type;
            r.event_relative_s = current_relative_s;
        }
        r.fn_N = std::isfinite(final_compensation) ? final_compensation : 0.0;
        r.display_prediction_N = r.fn_N;
        r.sensor_prediction_N = r.fn_N / c.installation_gain;
        r.valid = true;
        r.gate = true;
        r.status = delivery_forward ? "delivery_compensation_off" :
            (current_type != 0 ? "event_compensation_active" : "event_compensation_waiting");
        r.reset_reason = pending_reset_;
        pending_reset_ = "none";

        previous_a_ = a;
        previous_v_ = v;
        previous_motion_time_ = in.time_s;
        has_previous_motion_ = true;
        previous_moving_cmd_ = in.moving_cmd;
        config_ = c;
        initialized_ = true;
        return r;
    }

private:
    struct Event {
        EventType type = EventType::None;
        double start_s = 0.0;
        double baseline_N = 0.0;
        double peak_N = 0.0;
        bool active = false;
    };

    static constexpr auto no_index = std::numeric_limits<std::uint64_t>::max();
    static constexpr std::size_t kHistory = 64;

    static double positive_dt(double value) {
        return std::isfinite(value) && value > 1e-6 ? value : 0.001;
    }

    bool config_changed(const Config& c) const {
        return c.axial_sign != config_.axial_sign ||
            c.beta_a != config_.beta_a || c.beta_v != config_.beta_v ||
            c.beta_s != config_.beta_s || c.vel_deadband_m_s != config_.vel_deadband_m_s ||
            c.installation_gain != config_.installation_gain ||
            c.acceleration_alignment_delay_s != config_.acceleration_alignment_delay_s ||
            c.velocity_alignment_delay_s != config_.velocity_alignment_delay_s ||
            c.guidewire != config_.guidewire ||
            c.moving_open_word != config_.moving_open_word ||
            c.moving_close_word != config_.moving_close_word ||
            c.release_delay_s != config_.release_delay_s ||
            c.reclamp_delay_s != config_.reclamp_delay_s ||
            c.release_peak_N != config_.release_peak_N ||
            c.reclamp_peak_N != config_.reclamp_peak_N;
    }

    void update_history(double time_s, double residual_N) {
        history_[history_head_] = {time_s, residual_N};
        history_head_ = (history_head_ + 1) % kHistory;
        history_size_ = std::min(history_size_ + 1, kHistory);
    }

    double average_since(double time_s) const {
        double sum = 0.0;
        std::size_t count = 0;
        for (std::size_t i = 0; i < history_size_; ++i) {
            const std::size_t index = (history_head_ + kHistory - 1 - i) % kHistory;
            if (history_[index].time_s < time_s) break;
            sum += history_[index].value_N;
            ++count;
        }
        return count == 0 ? 0.0 : sum / static_cast<double>(count);
    }

    double wavelet_strength(const Config& c) const {
        if (history_size_ < 4) return 0.0;
        const double now = history_[(history_head_ + kHistory - 1) % kHistory].time_s;
        return std::abs(average_since(now - c.wavelet_short_s) -
            average_since(now - c.wavelet_long_s));
    }

    void detect_event(const Input& in, const Config& c, double motion_N) {
        if (!has_previous_command_) {
            previous_moving_cmd_ = in.moving_cmd;
            has_previous_command_ = true;
            event_baseline_ = in.force_cal_delta_N - motion_N;
            return;
        }
        const bool distinct_clamp_words = c.moving_open_word != c.moving_close_word;
        const bool release = distinct_clamp_words &&
            previous_moving_cmd_ == static_cast<double>(c.moving_close_word) &&
            in.moving_cmd == static_cast<double>(c.moving_open_word);
        const bool reclamp = distinct_clamp_words &&
            previous_moving_cmd_ == static_cast<double>(c.moving_open_word) &&
            in.moving_cmd == static_cast<double>(c.moving_close_word);
        if (release || reclamp) {
            Event& event = events_[event_count_ % events_.size()];
            event.type = release ? EventType::Release : EventType::Reclamp;
            event.start_s = in.time_s;
            event.baseline_N = in.force_cal_delta_N - motion_N;
            event.peak_N = release ? c.release_peak_N : c.reclamp_peak_N;
            event.active = true;
            ++event_count_;
            event_baseline_ = event.baseline_N;
            low_residual_ = event.baseline_N;
            previous_low_time_ = in.time_s;
            has_previous_low_time_ = true;
        }
        previous_moving_cmd_ = in.moving_cmd;
    }

    double transient_compensation(double time_s, const Config& c, double wavelet,
        int& current_type, double& current_relative_s) const {
        double total = 0.0;
        current_type = 0;
        current_relative_s = 0.0;
        const double gain = clamp(1.0 + wavelet / c.wavelet_reference_N,
            c.wavelet_min_gain, c.wavelet_max_gain);
        for (const auto& event : events_) {
            if (!event.active) continue;
            const double delay = event.type == EventType::Release ? c.release_delay_s : c.reclamp_delay_s;
            const double tau = time_s - event.start_s - delay;
            if (tau < 0.0 || tau > c.template_duration_s) continue;
            const double envelope = std::exp(-tau / c.template_decay_s) *
                (1.0 - std::exp(-tau / c.template_rise_s));
            const double shape = std::sin(2.0 * 3.14159265358979323846 *
                c.template_frequency_hz * tau);
            total += event.peak_N * gain * envelope * shape;
            if (current_type == 0 || tau < current_relative_s) {
                current_type = static_cast<int>(event.type);
                current_relative_s = tau;
            }
        }
        return total;
    }

    double low_frequency_compensation(const Input& in, const Config& c, double motion_N) {
        bool active = false;
        for (const auto& event : events_) {
            if (!event.active) continue;
            const double age = in.time_s - event.start_s;
            if (age >= c.low_frequency_start_s &&
                age <= c.low_frequency_start_s + c.low_frequency_duration_s) {
                active = true;
                break;
            }
        }
        if (!active || in.phase == 4 || in.phase == 9) return 0.0;
        const double residual = in.force_cal_delta_N - motion_N;
        const double dt = has_previous_low_time_ ? positive_dt(in.time_s - previous_low_time_) : 0.001;
        const double alpha = 1.0 - std::exp(-dt / c.low_frequency_tau_s);
        low_residual_ += alpha * (residual - low_residual_);
        previous_low_time_ = in.time_s;
        return clamp(low_residual_ - event_baseline_, -c.low_frequency_limit_N, c.low_frequency_limit_N);
    }

    int current_event_type(double time_s, double& relative_s) const {
        int type = 0;
        double best = std::numeric_limits<double>::infinity();
        for (const auto& event : events_) {
            if (!event.active) continue;
            const double age = time_s - event.start_s;
            if (age >= 0.0 && age < best) {
                best = age;
                type = static_cast<int>(event.type);
                relative_s = age;
            }
        }
        return type;
    }

    static double clamp(double value, double lower, double upper) {
        return std::max(lower, std::min(upper, value));
    }

    Result invalid(Result r, const char* reason) {
        initialized_ = false;
        has_previous_command_ = has_previous_motion_ = has_previous_low_time_ = false;
        r.fn_N = r.ft_N = r.motion_N = r.low_frequency_N = r.transient_N =
            r.sensor_prediction_N = r.display_prediction_N = 0.0;
        r.gate = r.valid = false;
        r.status = r.reset_reason = reason;
        pending_reset_ = reason;
        return r;
    }

    struct HistorySample {
        double time_s = 0.0;
        double value_N = 0.0;
    };

    bool initialized_ = false, has_time_ = false;
    bool has_previous_command_ = false, has_previous_motion_ = false, has_previous_low_time_ = false;
    double time_ = 0.0, previous_moving_cmd_ = 0.0;
    double previous_a_ = 0.0, previous_v_ = 0.0, previous_motion_time_ = 0.0;
    double previous_low_time_ = 0.0;
    double low_residual_ = 0.0, event_baseline_ = 0.0;
    std::uint64_t index_ = no_index;
    std::size_t event_count_ = 0, history_size_ = 0, history_head_ = 0;
    std::array<Event, 8> events_{};
    std::array<HistorySample, kHistory> history_{};
    Config config_{};
    const char* pending_reset_ = "initial";
};

inline std::string snapshot(bool guidewire, const Config& c, bool external_validation = false) {
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\"version\":\"" << kVersion << "\",\"experimental\":true,\"available\":"
        << (valid_config(c) ? "true" : "false")
        << ",\"application_mode\":\"" << (external_validation ? "external_validation" :
            guidewire ? "guidewire" : "catheter")
        << "\",\"parameter_source\":\""
        << (guidewire ? "not_validated_guidewire" :
            "stage1_motion_plus_catheter_validation_20261005") << "\""
        << ",\"validation_scope\":\""
        << (guidewire ? "not_validated" :
            external_validation ? "catheter_external" : "catheter_only") << "\""
        << ",\"force_definition\":\"installed_delta_N minus motion_low_frequency_and_transient_compensation\""
        << ",\"motion_model\":{\"beta_a\":" << c.beta_a << ",\"beta_v\":" << c.beta_v
        << ",\"beta_s\":" << c.beta_s
        << ",\"acceleration_alignment_delay_s\":" << c.acceleration_alignment_delay_s
        << ",\"velocity_alignment_delay_s\":" << c.velocity_alignment_delay_s << "}"
        << ",\"catheter_mechanics\":{\"clamp_force_N\":" << c.clamp_force_N
        << ",\"spring_constant_N_per_mm\":" << c.spring_constant_N_per_mm
        << ",\"opposing_spring_count\":" << unsigned(c.opposing_spring_count)
        << ",\"effective_spring_constant_N_per_mm\":"
        << c.spring_constant_N_per_mm * unsigned(c.opposing_spring_count)
        << ",\"clamp_angle_deg\":" << c.clamp_angle_deg
        << ",\"slip_load_N\":" << c.catheter_slip_load_N << "}"
        << ",\"load_transfer_validation\":{\"enabled\":"
        << (c.load_transfer_model_available ? "true" : "false")
        << ",\"alpha\":" << c.load_transfer_alpha
        << ",\"beta_N\":" << c.load_transfer_beta_N
        << ",\"mae_N\":" << c.load_transfer_mae_N
        << ",\"r2\":" << c.load_transfer_r2 << "}"
        << ",\"event_model\":{\"release_delay_s\":" << c.release_delay_s
        << ",\"reclamp_delay_s\":" << c.reclamp_delay_s
        << ",\"template_duration_s\":" << c.template_duration_s
        << ",\"template_frequency_hz\":" << c.template_frequency_hz
        << ",\"release_peak_N\":" << c.release_peak_N
        << ",\"reclamp_peak_N\":" << c.reclamp_peak_N << "}"
        << ",\"low_frequency_model\":{\"start_s\":" << c.low_frequency_start_s
        << ",\"duration_s\":" << c.low_frequency_duration_s
        << ",\"tau_s\":" << c.low_frequency_tau_s
        << ",\"limit_N\":" << c.low_frequency_limit_N << "}"
        << ",\"wavelet\":{\"causal\":\"haar_two_scale\",\"short_s\":" << c.wavelet_short_s
        << ",\"long_s\":" << c.wavelet_long_s
        << ",\"reference_N\":" << c.wavelet_reference_N << "}"
        << ",\"moving_open_word\":" << c.moving_open_word
        << ",\"moving_close_word\":" << c.moving_close_word
        << ",\"installation_axial_gain\":" << c.installation_gain
        << ",\"validation_mode\":" << (c.validation_mode ? "true" : "false")
        << ",\"conditions_confirmed_by_operator\":" << (c.conditions_confirmed ? "true" : "false")
        << ",\"acceleration_source\":\"NcToPlc.ActAcc\""
        << ",\"acceleration_processing\":\"causal_52ms_acceleration_and_8ms_velocity_lead_estimate\""
        << ",\"phase4_phase9_final_compensation_zero\":true"
        << ",\"haptic_feedback_enabled\":true}";
    return out.str();
}

inline std::string snapshot(bool guidewire) { return snapshot(guidewire, config(guidewire)); }
}
