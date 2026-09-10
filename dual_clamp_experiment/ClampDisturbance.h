#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace clampmodel {
constexpr int kFeatureCount = 15;
struct Parameters {
    bool available = false;
    double center[kFeatureCount]{};
    double scale[kFeatureCount]{};
    double beta[kFeatureCount][2]{};
};
struct Input {
    double time = 0, velocity = 0, moving = 0, fixed = 0, angle = 0;
    int phase = 0, cycle = 0;
};
struct Result {
    std::array<double, 2> disturbance{};
    bool valid = false;
};
class Predictor {
public:
    void reset() {
        initialized_ = false;
        time_ = velocity_ = acceleration_ = jerk_ = start_ = stage_start_ = 0;
        phase_ = cycle_ = -1;
    }
    Result update(const Input& in, const Parameters& params) {
        Result result;
        if (!std::isfinite(in.time) || !std::isfinite(in.velocity) ||
            !std::isfinite(in.moving) || !std::isfinite(in.fixed) || !std::isfinite(in.angle)) {
            reset(); return result;
        }
        if (initialized_ && in.time <= time_) return result;
        if (!initialized_ || in.time - time_ > 0.003 + 1e-12) {
            reset(); start_ = in.time;
        }
        if (in.phase != phase_ || in.cycle != cycle_) {
            stage_start_ = in.time; phase_ = in.phase; cycle_ = in.cycle;
        }
        if (initialized_) {
            const double dt = in.time - time_;
            const double alpha = dt / (0.003 + dt);
            const double old_a = acceleration_;
            acceleration_ += alpha * ((in.velocity - velocity_) / dt - acceleration_);
            jerk_ += alpha * ((acceleration_ - old_a) / dt - jerk_);
        }
        time_ = in.time; velocity_ = in.velocity; initialized_ = true;
        if (!params.available || in.time - start_ < 0.010 - 1e-12) return result;
        result.valid = true;
        if (in.phase < 5 || in.phase > 7) return result;
        const double elapsed = std::clamp(in.time - stage_start_, 0.0, 0.5);
        const double angle = in.angle * 3.14159265358979323846 / 180.0;
        const double x[kFeatureCount] = {1., in.velocity, std::abs(in.velocity),
            acceleration_, jerk_, in.moving, in.fixed, std::sin(angle), std::cos(angle),
            double(in.phase == 5), double(in.phase == 6), double(in.phase == 7),
            elapsed, elapsed * elapsed, in.velocity * acceleration_};
        for (int i = 0; i < kFeatureCount; ++i) {
            if (!(params.scale[i] > 0)) return {};
            const double z = (x[i] - params.center[i]) / params.scale[i];
            for (int j = 0; j < 2; ++j) result.disturbance[j] += z * params.beta[i][j];
        }
        if (!std::isfinite(result.disturbance[0]) || !std::isfinite(result.disturbance[1])) return {};
        return result;
    }
private:
    bool initialized_ = false;
    double time_ = 0, velocity_ = 0, acceleration_ = 0, jerk_ = 0, start_ = 0, stage_start_ = 0;
    int phase_ = -1, cycle_ = -1;
};
} // namespace clampmodel
