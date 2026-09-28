#pragma once
#include "positioning_arm/kinematics.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>

// 唯一曲线契约，单位始终为PLC的毫米、度、秒；序号单独提交。
// 0..4起点，5..9终点，10..14弯曲系数，15时间，16升降下端。
struct ArmCurve {
    std::array<double, 17> values{};
};

struct ArmCartesianSettings {
    double lift_min_mm = std::numeric_limits<double>::quiet_NaN();
    double home_travel_deg = 0;
    double home_tip_mm = 0;
    double home_rotation_deg = 0;
};

struct ArmCurvePlan {
    int error = 0;
    bool already_home = false;
    ArmCurve curve;
};

inline ArmCurvePlan plan_arm_curve(const std::array<double, 5>& actual,
    const std::array<double, 5>& velocity, const std::array<double, 5>& acceleration,
    const ArmCartesianSettings& settings, int mode, double speed)
{
    using namespace positioning_arm;
    constexpr double rad = 3.14159265358979323846 / 180;
    ArmCurvePlan out;
    if (!std::isfinite(settings.lift_min_mm) || std::abs(settings.lift_min_mm) > 100000
        || mode < 0 || mode > 5 || !std::isfinite(speed)) {
        out.error = 1001; return out;
    }
    ArmKinematicConfig c;
    c.lower << settings.lift_min_mm * 0.001, 0,
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity();
    c.upper << (settings.lift_min_mm + 200) * 0.001, 180 * rad,
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity();
    JointVector current;
    JointVector amax;
    for (int i = 0; i < 5; ++i) {
        const double scale = i == 0 ? 0.001 : rad;
        if (!std::isfinite(actual[i]) || std::abs(actual[i]) > 100000
            || !std::isfinite(velocity[i]) || velocity[i] <= 0
            || !std::isfinite(acceleration[i]) || acceleration[i] <= 0) {
            out.error = 1001; return out;
        }
        c.calibration[i] = {0, scale, 1};
        c.max_velocity[i] = velocity[i] * scale;
        current[i] = actual[i] * scale;
        amax[i] = acceleration[i] * scale;
    }
    PositioningArmKinematics arm(c);
    if (!arm.checkLimits(current)) { out.error = 1002; return out; }
    JointVector target = current;
    JointVector bend = JointVector::Zero();
    double duration = 0.1;
    if (mode == 0) {
        HomeOptions options;
        options.max_joint_travel.setConstant(settings.home_travel_deg * rad);
        options.max_acceleration = amax.tail<4>();
        options.max_tip_displacement = settings.home_tip_mm * 0.001;
        options.max_tool_rotation = settings.home_rotation_deg * rad;
        const auto r = arm.planProgramZeroReturn(current, options);
        if (!r.success()) { out.error = 1200 + static_cast<int>(r.status); return out; }
        if (r.status == HomeStatus::AlreadyAtZero) { out.already_home = true; return out; }
        target = r.target;
        bend.tail<4>() = r.bend;
        duration = r.duration;
    } else {
        // UI次序：X、Y、Z、俯仰、偏航。每段从反馈位置重新求解，不累计盲发目标。
        const auto r = arm.solveJog(current, {
            static_cast<JogAxis>(mode - 1), speed * (mode <= 3 ? 0.001 : rad), duration});
        if (!r.success()) { out.error = 1100 + static_cast<int>(r.status); return out; }
        target = r.target;
        const auto distance = (target - current).cwiseAbs().eval();
        for (int i = 0; i < 5; ++i) {
            duration = std::max(duration, 1.875 * distance[i] / c.max_velocity[i]);
            duration = std::max(duration, std::sqrt(6 * distance[i] / amax[i]));
        }
    }
    for (int i = 0; i < 5; ++i) {
        const double scale = i == 0 ? 0.001 : rad;
        out.curve.values[i] = actual[i];
        out.curve.values[5 + i] = target[i] / scale;
        out.curve.values[10 + i] = bend[i] / scale;
    }
    out.curve.values[15] = duration;
    out.curve.values[16] = settings.lift_min_mm;
    return out;
}
