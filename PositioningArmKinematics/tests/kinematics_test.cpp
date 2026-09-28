#include "positioning_arm/kinematics.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace positioning_arm;
namespace {
constexpr double Pi = 3.14159265358979323846;
constexpr double Degree = Pi / 180;
void check(bool ok, const char* message)
{
    if (!ok) throw std::runtime_error(message);
}
template<typename F> void throws(F action)
{
    bool thrown = false;
    try { action(); } catch (const std::invalid_argument&) { thrown = true; }
    check(thrown, "Expected invalid_argument");
}
ArmKinematicConfig fixture()
{
    ArmKinematicConfig c;
    // 尺寸来自用户；端点、速度、编码器单位仅为离线合成配置。
    c.lower << 0, 0, -4 * Pi, -4 * Pi, -Pi / 2;
    c.upper << 0.2, Pi, 4 * Pi, 4 * Pi, Pi / 2;
    c.max_velocity << 0.1, 1, 1, 1, 1;
    for (int i = 0; i < AxisCount; ++i)
        c.calibration[i] = {0, i == 0 ? 0.001 : Degree, 1};
    return c;
}
JointVector start()
{
    JointVector q;
    q << 0.1, 0.6, 1.2, 0.2, -0.1;
    return q;
}
JointVector home()
{
    JointVector q;
    q << 0.1, Pi / 2, Pi, Pi, 0;
    return q;
}
JointVector outside()
{
    JointVector q = home();
    q.tail<4>() += Degree * Eigen::Vector4d(-6, -4, 4, 2);
    return q;
}
HomeOptions homeOptions()
{
    // 所有预算均为测试值，不能用于实机执行。
    HomeOptions o;
    o.max_joint_travel.setConstant(20 * Degree);
    o.max_acceleration.setConstant(0.4);
    o.max_tip_displacement = 0.2;
    o.max_tool_rotation = 0.5;
    return o;
}
void samePose(const ArmPose& a, const ArmPose& b, double tolerance = 2e-7)
{
    check((a.position - b.position).norm() < tolerance, "Position mismatch");
    check(a.quaternion.angularDistance(b.quaternion) < tolerance, "Orientation mismatch");
}
void rejected(const IkResult& r, const JointVector& q)
{
    check(!r.success() && (r.target - q).norm() == 0, "Failure exposed partial IK");
}
void rejected(const HomeResult& r, const JointVector& q, HomeStatus expected)
{
    check(r.status == expected, "Unexpected home failure reason");
    check(!r.success() && r.samples.empty() && (r.target - q).norm() == 0,
        "Failure exposed partial home plan");
}
void verifyHome(PositioningArmKinematics& arm, const ArmKinematicConfig& config,
    const JointVector& q, const HomeOptions& options, const HomeResult& result)
{
    check(result.status == HomeStatus::Planned, "Expected home plan");
    auto target = home(); target[0] = q[0];
    check((result.target - target).norm() == 0, "Wrong mechanical home target");
    check(result.samples.size() >= 2, "Missing trajectory");
    check((result.samples.front().position - q).norm() == 0, "Wrong start");
    check((result.samples.back().position - target).norm() == 0, "Wrong end");
    check(result.samples.front().velocity.norm() == 0
        && result.samples.back().velocity.norm() == 0, "Nonzero endpoint velocity");
    check(result.samples.front().acceleration.norm() == 0
        && result.samples.back().acceleration.norm() == 0, "Nonzero endpoint acceleration");
    check(result.optimized_cost <= result.baseline_cost + 1e-12, "Optimization worsened cost");
    check(result.tip_displacement_bound <= options.max_tip_displacement
        && result.tool_rotation_bound <= options.max_tool_rotation, "Motion budgets exceeded");
    check(result.tip_displacement_bound + 1e-12 >= result.sampled_tip_displacement
        && result.tool_rotation_bound + 1e-12 >= result.sampled_tool_rotation, "Invalid bound");
    const auto initial = arm.forward(q);
    for (size_t i = 0; i < result.samples.size(); ++i) {
        const auto& sample = result.samples[i];
        check(arm.checkLimits(sample.position), "Home sample outside limits");
        check(sample.position[0] == q[0] && sample.velocity[0] == 0
            && sample.acceleration[0] == 0, "Home moved axis1");
        check((sample.position.array() >= q.cwiseMin(target).array() - 1e-12).all()
            && (sample.position.array() <= q.cwiseMax(target).array() + 1e-12).all(),
            "Home joint detour");
        check((sample.velocity.cwiseAbs().array() <= config.max_velocity.array() + 1e-12).all(),
            "Home overspeed");
        check((sample.acceleration.tail<4>().cwiseAbs().array()
            <= options.max_acceleration.array() + 1e-12).all(), "Home overacceleration");
        const auto pose = arm.forward(sample.position);
        check((pose.position - initial.position).norm() <= result.tip_displacement_bound + 1e-12,
            "Displacement escaped bound");
        if (i > 0) {
            const auto& previous = result.samples[i - 1];
            const auto travel = (sample.position - previous.position).eval();
            check((travel.array() * (target - q).array() >= -1e-14).all(),
                "Nonmonotonic home motion");
            const double dt = sample.time - previous.time;
            check(dt > 0 && dt <= options.sample_period + 1e-12, "Invalid sample time");
            check((travel.cwiseAbs().array() / dt <= config.max_velocity.array() + 1e-10).all(),
                "Sampled average overspeed");
        }
    }
}
} // namespace

int main()
{
    const std::vector<std::pair<const char*, std::function<void()>>> cases = {
        {"Known mechanical and program-zero geometry", [] {
            PositioningArmKinematics arm(fixture());
            auto q = JointVector::Zero().eval(); q[0] = 0.1;
            auto pose = arm.forward(q);
            check((pose.position - Eigen::Vector3d(-0.5545, -0.8363, 0.1)).norm() < 1e-12,
                "Wrong mechanical-zero L geometry");
            pose = arm.forward(home());
            check((pose.position - Eigen::Vector3d(0.4763, -0.5545, 0.1)).norm() < 1e-12,
                "Wrong program-zero geometry");
            check(pose.quaternion.angularDistance(Eigen::Quaterniond::Identity()) < 1e-12,
                "Program-zero tool axes not aligned");
            q = home(); q[4] = -30 * Degree;
            pose = arm.forward(q);
            check(std::abs(pose.position.z() - 0.37725) < 1e-12, "Negative pitch did not lift tip");
            check(std::abs(pose.position.x() - 0.4763) < 1e-12, "Pitch changed lateral offset");
            check(arm.degreesOfFreedom() == 5, "Wrong DOF");
        }},
        {"Analytical Jacobian finite differences", [] {
            PositioningArmKinematics arm(fixture());
            auto q = start();
            for (int sample = 0; sample < 12; ++sample) {
                q[1] += 0.1; q[4] += 0.03;
                const auto j = arm.computeFrameJacobian(q);
                constexpr double eps = 1e-7;
                for (int axis = 0; axis < AxisCount; ++axis) {
                    auto plus = q, minus = q;
                    plus[axis] += eps; minus[axis] -= eps;
                    const auto a = arm.forward(plus), b = arm.forward(minus);
                    const Eigen::AngleAxisd angle(a.quaternion * b.quaternion.conjugate());
                    check(((a.position - b.position) / (2 * eps) - j.col(axis).head<3>()).norm()
                        < 1e-7, "Linear Jacobian mismatch");
                    check((angle.axis() * angle.angle() / (2 * eps) - j.col(axis).tail<3>()).norm()
                        < 1e-7, "Angular Jacobian mismatch");
                }
            }
        }},
        {"Mechanical encoder and program coordinates are distinct", [] {
            PositioningArmKinematics arm(fixture());
            JointVector reading; reading << 100, 90, 180, 180, 0;
            const auto q = arm.fromEncoder(reading);
            check((q - home()).norm() < 1e-12, "Offsets applied to encoder conversion");
            check(arm.toProgramCoordinates(q).tail<4>().norm() < 1e-12, "Program offset missing");
            check((arm.fromProgramCoordinates(arm.toProgramCoordinates(start())) - start()).norm()
                < 1e-12, "Program roundtrip");
            check((arm.toEncoder(q) - reading).norm() < 1e-10, "Encoder roundtrip");
            auto shifted = reading; shifted[4] -= 1;
            check(arm.forward(arm.fromEncoder(shifted)).position.z() > arm.forward(q).position.z(),
                "PLC axis5 sign wrong");
            auto c = fixture(); c.calibration[1].direction = -1;
            PositioningArmKinematics reverse(c);
            check(reverse.toEncoder(start())[1] < 0, "Encoder direction ignored");
        }},
        {"Invalid configuration and explicit unlimited axes", [] {
            throws([] { PositioningArmKinematics arm(ArmKinematicConfig{}); });
            auto c = fixture(); c.calibration[2].scale = 0;
            throws([&] { PositioningArmKinematics arm(c); });
            c = fixture(); c.link_lengths[0] = 0;
            throws([&] { PositioningArmKinematics arm(c); });
            c = fixture(); c.upper[1] = c.lower[1];
            throws([&] { PositioningArmKinematics arm(c); });
            c = fixture(); c.lower.tail<3>().setConstant(-std::numeric_limits<double>::infinity());
            c.upper.tail<3>().setConstant(std::numeric_limits<double>::infinity());
            PositioningArmKinematics arm(c);
            check(arm.checkLimits(home()), "Explicit unlimited test axes rejected");
        }},
        {"XY and yaw isolation", [] {
            PositioningArmKinematics arm(fixture());
            for (JogAxis axis : {JogAxis::X, JogAxis::Y, JogAxis::Yaw}) {
                for (double sign : {-1., 1.}) {
                    const auto q = start();
                    const auto before = arm.forward(q);
                    const auto r = arm.solveJog(q, {axis, sign * 0.004, 0.05});
                    check(r.success(), "Planar jog rejected");
                    check(r.target[0] == q[0] && r.target[4] == q[4], "Planar jog moved axis1/5");
                    auto expected = before;
                    if (axis == JogAxis::X) expected.position.x() += sign * 0.0002 * r.speed_scale;
                    if (axis == JogAxis::Y) expected.position.y() += sign * 0.0002 * r.speed_scale;
                    if (axis == JogAxis::Yaw) expected.quaternion =
                        Eigen::Quaterniond(Eigen::AngleAxisd(sign * 0.0002 * r.speed_scale,
                            Eigen::Vector3d::UnitZ()))
                        * before.quaternion;
                    samePose(r.pose, expected);
                }
            }
        }},
        {"Z and pitch bypass planar singularity", [] {
            PositioningArmKinematics arm(fixture());
            const auto q = home();
            check(arm.isNearSingularity(q), "Folded program zero must be singular");
            for (auto axis : {JogAxis::X, JogAxis::Y, JogAxis::Yaw}) {
                const auto r = arm.solveJog(q, {axis, 0.01, 0.05});
                rejected(r, q);
                check(r.status == IkStatus::NearSingularity, "Singularity not reported");
            }
            auto r = arm.solveJog(q, {JogAxis::Z, 0.01, 0.05});
            check(r.success() && (r.target.tail<4>() - q.tail<4>()).norm() == 0, "Z isolation failed");
            check(std::abs(r.target[0] - q[0] - 0.0005) < 1e-12, "Z speed ignored");
            r = arm.solveJog(q, {JogAxis::Pitch, 0.1, 0.05});
            check(r.success() && (r.target.head<4>() - q.head<4>()).norm() == 0,
                "Pitch isolation failed");
            check(std::abs(r.target[4] + 0.005) < 1e-12, "Pitch sign wrong");
            check(r.pose.position.z() > arm.forward(q).position.z(), "Pitch did not lift tip");
            check(arm.solveCartesianJog(q, {}).success(), "Stationary hold rejected");
        }},
        {"Jog input, velocity and position rejection", [] {
            PositioningArmKinematics arm(fixture());
            CartesianJog both; both.translation.x() = both.yaw = 0.001;
            check(arm.solveCartesianJog(start(), both).status == IkStatus::InvalidInput,
                "Mixed jog accepted");
            auto q = start(); q[0] = 0.2;
            auto r = arm.solveJog(q, {JogAxis::Z, 0.01, 0.05});
            rejected(r, q); check(r.status == IkStatus::JointLimit, "Missing limit reason");
            auto c = fixture(); c.max_velocity[4] = 0.001;
            PositioningArmKinematics slow(c);
            r = slow.solveJog(start(), {JogAxis::Pitch, 0.1, 0.05});
            rejected(r, start()); check(r.status == IkStatus::StepLimit, "Missing speed reason");
            check(arm.solveJog(start(), {JogAxis::X, 1, 0}).status == IkStatus::InvalidInput,
                "Zero duration accepted");
            check(arm.solveJog(start(), {static_cast<JogAxis>(99), 1, 0.05}).status
                == IkStatus::InvalidInput, "Invalid mode accepted");
            check(arm.solveJog(start(), {JogAxis::Yaw,
                std::numeric_limits<double>::quiet_NaN(), 0.05}).status
                == IkStatus::InvalidInput, "NaN accepted");
        }},
        {"All finite physical boundaries", [] {
            const auto c = fixture(); PositioningArmKinematics arm(c);
            for (int i = 0; i < AxisCount; ++i) {
                auto q = start(); q[i] = c.lower[i];
                check(arm.checkLimits(q), "Lower endpoint rejected");
                q[i] -= 1e-7; check(!arm.checkLimits(q), "Under limit accepted");
                q[i] = c.upper[i]; check(arm.checkLimits(q), "Upper endpoint rejected");
                q[i] += 1e-7; check(!arm.checkLimits(q), "Over limit accepted");
            }
        }},
        {"Continuous planar jog and quaternion sign", [] {
            PositioningArmKinematics arm(fixture());
            auto q = start(); const auto initial = arm.forward(q);
            for (int i = 0; i < 100; ++i) {
                const auto r = arm.solveJog(q, {JogAxis::Y, i < 50 ? 0.001 : -0.001, 0.05});
                check(r.success(), "Continuous jog failed");
                check((r.target - q).cwiseAbs().maxCoeff() < 0.01, "Joint jump");
                check(std::sin(q[2]) * std::sin(r.target[2]) > 0, "Branch changed");
                check(r.pose.quaternion.dot(arm.forward(q).quaternion) >= 0, "Quaternion sign jump");
                q = r.target;
            }
            samePose(arm.forward(q), initial, 1e-5);
            auto negative = initial.quaternion; negative.coeffs() *= -2;
            const auto aligned = PositioningArmKinematics::continuousQuaternion(negative, initial.quaternion);
            check(aligned.dot(initial.quaternion) > 1 - 1e-12, "Sign normalization failed");
        }},
        {"Unreachable and iteration exhaustion preserve state", [] {
            PositioningArmKinematics arm(fixture());
            auto q = home(); q[2] -= 0.001;
            rejected(arm.solveJog(q, {JogAxis::X, 0.3, 0.05}), q);
            SolverOptions o; o.max_iterations = 1;
            PositioningArmKinematics limited(fixture(), o);
            rejected(limited.solveJog(start(), {JogAxis::X, 0.04, 0.05}), start());
        }},
        {"Home windows include exact boundaries without motion", [] {
            PositioningArmKinematics arm(fixture());
            for (int mask = 0; mask < 16; ++mask) {
                auto q = home();
                for (int i = 1; i < 5; ++i)
                    q[i] += ((mask & (1 << (i - 1))) ? 1 : -1) * (i == 4 ? 1 : 3) * Degree;
                check(arm.isAtProgramZero(q), "Tolerance boundary rejected");
                const auto r = arm.planProgramZeroReturn(q, {});
                check(r.status == HomeStatus::AlreadyAtZero && r.samples.empty()
                    && (r.target - q).norm() == 0, "Unrequested exact-zero correction");
                for (int i = 1; i < 5; ++i) {
                    auto beyond = q;
                    beyond[i] += ((mask & (1 << (i - 1))) ? 1 : -1) * 1e-6;
                    check(!arm.isAtProgramZero(beyond), "Out of window accepted");
                }
            }
        }},
        {"Home does not wrap angles or relabel measured position", [] {
            PositioningArmKinematics arm(fixture());
            auto q = home(); q[2] += 2 * Pi;
            check(!arm.isAtProgramZero(q), "Wrapped angle accepted");
            rejected(arm.planProgramZeroReturn(q, homeOptions()), q, HomeStatus::TravelLimit);
            q = home(); q[2] += 2 * Degree;
            check(std::abs(arm.toProgramCoordinates(q)[2] - 2 * Degree) < 1e-12,
                "Within tolerance incorrectly reset to zero");
        }},
        {"Home path approaches folded zero with bounded dynamics", [] {
            const auto c = fixture(); PositioningArmKinematics arm(c);
            const auto options = homeOptions();
            const auto r = arm.planProgramZeroReturn(outside(), options);
            verifyHome(arm, c, outside(), options, r);
            check(r.optimized_cost < r.baseline_cost, "Expected optimizer improvement");
            std::cout << "Home example: duration=" << r.duration
                << " s, tip bound=" << r.tip_displacement_bound * 1000
                << " mm, rotation bound=" << r.tool_rotation_bound / Degree << " deg\n";
        }},
        {"Home positive and negative corrections", [] {
            const auto c = fixture(); PositioningArmKinematics arm(c);
            for (int mask = 0; mask < 16; ++mask) {
                auto q = home();
                for (int i = 1; i < 5; ++i)
                    q[i] += ((mask & (1 << (i - 1))) ? 1 : -1) * (i == 4 ? 2 : 4) * Degree;
                verifyHome(arm, c, q, homeOptions(), arm.planProgramZeroReturn(q, homeOptions()));
            }
        }},
        {"Home fails closed without budgets or with excessive tool motion", [] {
            PositioningArmKinematics arm(fixture()); const auto q = outside();
            rejected(arm.planProgramZeroReturn(q, {}), q, HomeStatus::InvalidInput);
            auto o = homeOptions(); o.max_joint_travel.setConstant(Degree);
            rejected(arm.planProgramZeroReturn(q, o), q, HomeStatus::TravelLimit);
            o = homeOptions(); o.max_tip_displacement = 0.00001;
            rejected(arm.planProgramZeroReturn(q, o), q, HomeStatus::ToolMotionLimit);
            o = homeOptions(); o.max_tool_rotation = 0.00001;
            rejected(arm.planProgramZeroReturn(q, o), q, HomeStatus::ToolMotionLimit);
            o = homeOptions(); o.max_samples = 2;
            rejected(arm.planProgramZeroReturn(q, o), q, HomeStatus::SamplingLimit);
            o = homeOptions(); o.max_acceleration[0] = 0;
            rejected(arm.planProgramZeroReturn(q, o), q, HomeStatus::InvalidInput);
        }},
        {"Home checks initial and final joint limits", [] {
            auto c = fixture(); c.upper[2] = Pi - Degree;
            PositioningArmKinematics arm(c);
            rejected(arm.planProgramZeroReturn(outside(), homeOptions()), outside(), HomeStatus::JointLimit);
            c = fixture(); PositioningArmKinematics normal(c);
            auto q = outside(); q[1] = -Degree;
            rejected(normal.planProgramZeroReturn(q, homeOptions()), q, HomeStatus::JointLimit);
        }},
        {"Home retiming and deterministic planning", [] {
            auto c = fixture(); c.max_velocity.tail<4>().setConstant(0.02);
            PositioningArmKinematics arm(c); auto o = homeOptions();
            o.max_acceleration.setConstant(0.01);
            const auto a = arm.planProgramZeroReturn(outside(), o);
            const auto b = arm.planProgramZeroReturn(outside(), o);
            verifyHome(arm, c, outside(), o, a);
            check(a.duration > 5 && a.duration == b.duration && a.samples.size() == b.samples.size(),
                "Retiming or determinism failed");
            for (size_t i = 0; i < a.samples.size(); ++i)
                check((a.samples[i].position - b.samples[i].position).norm() == 0, "Hidden state");
            o.optimization_iterations = 0;
            const auto baseline = arm.planProgramZeroReturn(outside(), o);
            verifyHome(arm, c, outside(), o, baseline);
            check(baseline.baseline_cost == baseline.optimized_cost, "Baseline option ignored");
        }},
        {"Near-singular speed reduction and malformed inputs", [] {
            PositioningArmKinematics arm(fixture());
            auto q = start(); q[2] = 0.25;
            const auto r = arm.solveJog(q, {JogAxis::X, 0.0002, 0.05});
            check(r.speed_scale > 0 && r.speed_scale < 1, "Near-singular speed unchanged");
            check(r.success(), "Small slowed jog rejected");
            check(std::abs(r.pose.position.x() - arm.forward(q).position.x()
                - 0.00001 * r.speed_scale) < 1e-7, "Reported speed scale not applied");
            q = outside(); q[2] = std::numeric_limits<double>::quiet_NaN();
            check(!arm.isAtProgramZero(q), "NaN recognized as home");
            check(arm.planProgramZeroReturn(q, homeOptions()).status == HomeStatus::InvalidInput,
                "Non-finite home state accepted");
            auto options = homeOptions(); options.sample_period = 0;
            rejected(arm.planProgramZeroReturn(outside(), options), outside(), HomeStatus::InvalidInput);
        }},
        {"Home single-axis recovery and dense dynamic consistency", [] {
            const auto config = fixture(); PositioningArmKinematics arm(config);
            for (int axis = 1; axis < AxisCount; ++axis) {
                for (double sign : {-1., 1.}) {
                    auto q = home(); q[0] = 0.15;
                    q[axis] += sign * (axis == 4 ? 2 : 4) * Degree;
                    auto options = homeOptions(); options.sample_period = 0.001;
                    const auto r = arm.planProgramZeroReturn(q, options);
                    verifyHome(arm, config, q, options, r);
                    for (size_t i = 1; i + 1 < r.samples.size(); ++i) {
                        const auto& before = r.samples[i - 1];
                        const auto& after = r.samples[i + 1];
                        const double dt = after.time - before.time;
                        check(((after.position - before.position) / dt
                            - r.samples[i].velocity).norm() < 1e-5, "Incorrect path velocity");
                        check(((after.velocity - before.velocity) / dt
                            - r.samples[i].acceleration).norm() < 1e-5, "Incorrect path acceleration");
                    }
                }
            }
        }}
    };
    int failed = 0;
    for (const auto& test : cases) {
        try { test.second(); std::cout << "PASS " << test.first << '\n'; }
        catch (const std::exception& e) {
            ++failed; std::cerr << "FAIL " << test.first << ": " << e.what() << '\n';
        }
    }
    std::cout << cases.size() - failed << "/" << cases.size() << " tests passed (no hardware).\n";
    return failed ? 1 : 0;
}
