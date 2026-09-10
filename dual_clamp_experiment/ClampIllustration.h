#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace clampillustration {
inline constexpr double kHoldAfterGate_s = 0.120;
inline constexpr double kFadeOut_s = 0.040;
struct Result {
    double fn = 0, ft = 0;
    double reference_fn = std::numeric_limits<double>::quiet_NaN();
    double reference_ft = std::numeric_limits<double>::quiet_NaN();
    double weight = 0;
    bool gate = false, valid_fn = false, valid_ft = false;
};
// A target illustration only. Neither a disturbance estimate nor an external-force estimate.
class Generator {
    struct Point { double time, fn, ft; };
    std::array<Point, 4096> history_{};
    std::size_t head_ = 0, size_ = 0;
    bool initialized_ = false, gate_ = false, episode_ = false, exit_pending_ = false;
    double time_ = 0, weight_ = 0, from_ = 0, target_ = 0, transition_ = 0, hold_until_ = 0;
    Result last_{};
    double reference(bool axial, double now) const {
        std::array<double, 4096> values;
        std::size_t count = 0;
        for (std::size_t i = 0; i < size_; ++i) {
            const auto& p = history_[(head_ + i) % history_.size()];
            const double age = now - p.time, v = axial ? p.fn : p.ft;
            if (age > .010 + 1e-12 && age <= .100 + 1e-12 && std::isfinite(v))
                values[count++] = v;
        }
        if (count < 5) return std::numeric_limits<double>::quiet_NaN();
        std::sort(values.begin(), values.begin() + count);
        return count % 2 ? values[count / 2] :
            .5 * values[count / 2 - 1] + .5 * values[count / 2];
    }
public:
    void reset() {
        head_ = size_ = 0;
        initialized_ = gate_ = episode_ = exit_pending_ = false;
        time_ = weight_ = from_ = target_ = transition_ = hold_until_ = 0;
        last_ = {};
    }
    Result update(double t, double fn, double ft, bool gate) {
        if (!std::isfinite(t)) { reset(); Result r; r.fn = fn; r.ft = ft; return r; }
        if (initialized_ && t == time_) return last_;
        if (initialized_ && (t < time_ || t - time_ > .003000001)) reset();
        const double u = std::clamp((t - transition_) / kFadeOut_s, 0.0, 1.0);
        weight_ = from_ + (target_ - from_) * u * u * (3 - 2 * u);
        if (!gate_ && weight_ == 0) episode_ = false;
        if (gate && !gate_ && !episode_) {
            last_.reference_fn = reference(true, t);
            last_.reference_ft = reference(false, t);
            episode_ = true;
        }
        if (gate && !gate_) {
            // Reentry during the hold or fade preserves both reference and current weight.
            from_ = weight_; target_ = 1.0; transition_ = t;
            exit_pending_ = false;
        } else if (!gate && gate_) {
            // Keep the correction active while the recorded mechanical response settles.
            exit_pending_ = true;
            hold_until_ = t + kHoldAfterGate_s;
        } else if (!gate && exit_pending_ && t >= hold_until_) {
            from_ = weight_; target_ = 0.0; transition_ = t;
            exit_pending_ = false;
        }
        last_.gate = gate;
        last_.weight = weight_;
        last_.valid_fn = episode_ && std::isfinite(last_.reference_fn) && std::isfinite(fn);
        last_.valid_ft = episode_ && std::isfinite(last_.reference_ft) && std::isfinite(ft);
        last_.fn = last_.valid_fn ? fn - .8 * weight_ * (fn - last_.reference_fn) : fn;
        last_.ft = last_.valid_ft ? ft - .8 * weight_ * (ft - last_.reference_ft) : ft;
        if (!episode_) {
            last_.reference_fn = last_.reference_ft = std::numeric_limits<double>::quiet_NaN();
        }
        while (size_ && t - history_[head_].time > .100000001) {
            head_ = (head_ + 1) % history_.size(); --size_;
        }
        if (size_ == history_.size()) { head_ = (head_ + 1) % history_.size(); --size_; }
        history_[(head_ + size_++) % history_.size()] = {t, fn, ft};
        initialized_ = true; time_ = t; gate_ = gate;
        return last_;
    }
};
inline const char* snapshot() {
    return R"({"version":"target-illustration-v2","purpose":"target_illustration","attenuation":0.8,"identified":false,"hold_after_gate_s":0.120,"fade_out_s":0.040,"transition":"cubic smoothstep from current weight","reference_age_s":[0.010,0.100],"reference_min_samples":5,"max_gap_s":0.003,"force_definition":"installed_delta_N; same as displayed original","gate":"existing command and phase gate","reentry":"preserve reference and current weight during hold/fade","warning":"Not dynamics output, external force or performance evidence; may suppress real external-force changes","training_target":false})";
}
}
