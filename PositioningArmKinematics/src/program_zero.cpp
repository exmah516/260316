#include "positioning_arm/kinematics.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace positioning_arm {
namespace {
constexpr double Pi = 3.14159265358979323846;
using Residual = Eigen::Matrix<double, 7, 1>;
using Derivative = Eigen::Matrix<double, 7, 4>;

Eigen::Vector4d zeroAngles()
{
    return Eigen::Vector4d(Pi / 2, Pi, Pi, 0);
}

bool positive(double value)
{
    return std::isfinite(value) && value > 0;
}

// 单个有界弯曲参数调整每个关节的进度，端点固定，不绕圈、不改变轴1。
JointVector path(const JointVector& start, const JointVector& goal,
    const Eigen::Vector4d& bend, double progress)
{
    if (progress == 0) return start;
    if (progress == 1) return goal;
    JointVector q = start;
    q.tail<4>() += progress * (goal - start).tail<4>()
        + (4 * progress * (1 - progress)) * bend;
    return q;
}
} // namespace

bool PositioningArmKinematics::isAtProgramZero(const JointVector& mechanical) const
{
    if (!checkLimits(mechanical)) return false;
    Eigen::Vector4d tolerance(3, 3, 3, 1);
    tolerance *= Pi / 180;
    // 只容忍浮点换算误差，不能将360度绕圈等价为原始读数。
    return ((mechanical.tail<4>() - zeroAngles()).cwiseAbs().array()
        <= tolerance.array() + 1e-12).all();
}

JointVector PositioningArmKinematics::toProgramCoordinates(
    const JointVector& mechanical) const
{
    if (!checkLimits(mechanical)) throw std::invalid_argument("机械轴位置无效");
    JointVector program = mechanical;
    program.tail<4>() -= zeroAngles();
    return program;
}

JointVector PositioningArmKinematics::fromProgramCoordinates(
    const JointVector& program) const
{
    JointVector mechanical = program;
    mechanical.tail<4>() += zeroAngles();
    if (!checkLimits(mechanical)) throw std::invalid_argument("程序轴位置无效");
    return mechanical;
}

HomeResult PositioningArmKinematics::planProgramZeroReturn(
    const JointVector& current, const HomeOptions& options) const
{
    HomeResult result;
    result.target = current;
    if (!current.allFinite()) return result;
    if (!checkLimits(current)) {
        result.status = HomeStatus::JointLimit;
        return result;
    }
    if (isAtProgramZero(current)) {
        result.status = HomeStatus::AlreadyAtZero;
        return result;
    }
    if (!options.max_joint_travel.allFinite()
        || !(options.max_joint_travel.array() > 0).all()
        || !options.max_acceleration.allFinite()
        || !(options.max_acceleration.array() > 0).all()
        || !positive(options.max_tip_displacement)
        || !positive(options.max_tool_rotation)
        || !positive(options.minimum_duration)
        || !positive(options.sample_period)
        || options.max_samples < 2 || options.max_samples > 1000000
        || options.optimization_iterations < 0 || options.optimization_iterations > 100)
        return result;

    JointVector goal = current;
    goal.tail<4>() = zeroAngles();
    if (!checkLimits(goal)) {
        result.status = HomeStatus::JointLimit;
        return result;
    }
    const Eigen::Vector4d delta = (goal - current).tail<4>();
    if (!delta.allFinite() || (delta.cwiseAbs().array()
        > options.max_joint_travel.array()).any()) {
        result.status = HomeStatus::TravelLimit;
        return result;
    }

    const auto initial_pose = forward(current);
    const double length = characteristic_length_;
    const double regularization = 1e-4;
    constexpr int quadrature = 24;
    const auto evaluate = [&](const Eigen::Vector4d& bend,
        Eigen::Matrix4d* normal, Eigen::Vector4d* gradient) {
        double cost = regularization * bend.squaredNorm();
        if (normal) *normal = regularization * Eigen::Matrix4d::Identity();
        if (gradient) *gradient = regularization * bend;
        for (int i = 1; i < quadrature; ++i) {
            const double s = static_cast<double>(i) / quadrature;
            const double factor = 4 * s * (1 - s);
            const auto q = path(current, goal, bend, s);
            const auto pose = forward(q);
            const auto quaternion = continuousQuaternion(pose.quaternion, initial_pose.quaternion);
            Residual residual;
            residual.head<3>() = (pose.position - initial_pose.position) / length;
            residual.tail<4>() = 2 * (quaternion.coeffs() - initial_pose.quaternion.coeffs());
            cost += residual.squaredNorm() / quadrature;
            if (!normal || !gradient) continue;
            const auto full = computeFrameJacobian(q);
            Derivative jacobian;
            jacobian.topRows<3>() = full.block<3, 4>(0, 1) * (factor / length);
            for (int joint = 0; joint < 4; ++joint) {
                const Eigen::Vector3d axis = full.block<3, 1>(3, joint + 1);
                const Eigen::Quaterniond omega(0, axis.x(), axis.y(), axis.z());
                jacobian.block<4, 1>(3, joint) = factor * (omega * quaternion).coeffs();
            }
            *normal += jacobian.transpose() * jacobian / quadrature;
            *gradient += jacobian.transpose() * residual / quadrature;
        }
        return cost;
    };

    Eigen::Vector4d bend = Eigen::Vector4d::Zero();
    double cost = evaluate(bend, nullptr, nullptr);
    result.baseline_cost = cost;
    // 局部低扰动优化，不承诺全局最优；数值失败直接拒绝，不切换备用轨迹。
    for (int iteration = 0; iteration < options.optimization_iterations; ++iteration) {
        Eigen::Matrix4d normal;
        Eigen::Vector4d gradient;
        evaluate(bend, &normal, &gradient);
        normal.diagonal().array() += 1e-6;
        const Eigen::Vector4d correction = -normal.ldlt().solve(gradient);
        if (!correction.allFinite() || !std::isfinite(cost)) {
            result.status = HomeStatus::NumericalFailure;
            return result;
        }
        bool improved = false;
        for (double alpha = 1; alpha > 1e-6; alpha *= 0.5) {
            // 参数边界保证每个关节单调运动，整段均处于起点、终点之间。
            const Eigen::Vector4d limit = delta.cwiseAbs() / 4;
            const Eigen::Vector4d next =
                (bend + alpha * correction).cwiseMax(-limit).cwiseMin(limit);
            const double candidate_cost = evaluate(next, nullptr, nullptr);
            if (!std::isfinite(candidate_cost)) {
                result.status = HomeStatus::NumericalFailure;
                return result;
            }
            if (candidate_cost < cost - 1e-14) {
                bend = next;
                cost = candidate_cost;
                improved = true;
                break;
            }
        }
        if (!improved) break;
    }
    result.optimized_cost = cost;

    // 五次时间律保证首尾速度、加速度为零；以下上界覆盖连续曲线而非仅采样点。
    const Eigen::Vector4d slope_bound = delta.cwiseAbs() + 4 * bend.cwiseAbs();
    double duration = options.minimum_duration;
    for (int i = 0; i < 4; ++i) {
        duration = std::max(duration, 1.875 * slope_bound[i] / config_.max_velocity[i + 1]);
        duration = std::max(duration, std::sqrt(
            (6 * slope_bound[i] + 28.125 * std::abs(bend[i]))
            / options.max_acceleration[i]));
    }
    const double segment_count = std::ceil(duration / options.sample_period);
    if (!std::isfinite(duration) || !std::isfinite(segment_count)
        || segment_count < 1 || segment_count > options.max_samples - 1) {
        result.status = HomeStatus::SamplingLimit;
        return result;
    }
    const int segments = static_cast<int>(segment_count);
    std::vector<HomeSample> samples;
    samples.reserve(segments + 1);
    const double tool_radius = std::hypot(config_.lateral_offset, config_.tool_length);
    Eigen::Vector4d lever_bounds;
    lever_bounds << config_.link_lengths.sum() + tool_radius,
        config_.link_lengths[1] + tool_radius, tool_radius, config_.tool_length;
    double previous_displacement = 0;
    double previous_rotation = 0;
    for (int i = 0; i <= segments; ++i) {
        const double u = static_cast<double>(i) / segments;
        const double s = i == segments ? 1 : u * u * u * (10 + u * (-15 + 6 * u));
        const double ds = 30 * u * u * (1 - u) * (1 - u) / duration;
        const double dds = 60 * u * (1 - u) * (1 - 2 * u) / (duration * duration);
        HomeSample sample;
        sample.time = u * duration;
        sample.position = path(current, goal, bend, s);
        sample.velocity.tail<4>() = (delta + 4 * (1 - 2 * s) * bend) * ds;
        sample.acceleration.tail<4>() =
            (delta + 4 * (1 - 2 * s) * bend) * dds - 8 * bend * ds * ds;
        if (!checkLimits(sample.position)) {
            result.status = HomeStatus::JointLimit;
            return result;
        }
        const auto pose = forward(sample.position);
        const double displacement = (pose.position - initial_pose.position).norm();
        const double rotation = pose.quaternion.angularDistance(initial_pose.quaternion);
        result.sampled_tip_displacement = std::max(result.sampled_tip_displacement, displacement);
        result.sampled_tool_rotation = std::max(result.sampled_tool_rotation, rotation);
        if (!samples.empty()) {
            const Eigen::Vector4d travel =
                (sample.position - samples.back().position).tail<4>().cwiseAbs();
            // 单调关节路径的弧长上界覆盖两采样点之间的末端偏离。
            result.tip_displacement_bound = std::max(result.tip_displacement_bound,
                (previous_displacement + displacement + lever_bounds.dot(travel)) / 2);
            result.tool_rotation_bound = std::max(result.tool_rotation_bound,
                (previous_rotation + rotation + travel.sum()) / 2);
        }
        previous_displacement = displacement;
        previous_rotation = rotation;
        samples.push_back(sample);
    }
    if (result.tip_displacement_bound > options.max_tip_displacement
        || result.tool_rotation_bound > options.max_tool_rotation) {
        result.status = HomeStatus::ToolMotionLimit;
        return result;
    }
    result.status = HomeStatus::Planned;
    result.target = goal;
    result.duration = duration;
    result.bend = bend;
    result.samples = std::move(samples);
    return result;
}

const char* statusName(HomeStatus status)
{
    switch (status) {
    case HomeStatus::AlreadyAtZero: return "AlreadyAtZero";
    case HomeStatus::Planned: return "Planned";
    case HomeStatus::InvalidInput: return "InvalidInput";
    case HomeStatus::JointLimit: return "JointLimit";
    case HomeStatus::TravelLimit: return "TravelLimit";
    case HomeStatus::ToolMotionLimit: return "ToolMotionLimit";
    case HomeStatus::SamplingLimit: return "SamplingLimit";
    case HomeStatus::NumericalFailure: return "NumericalFailure";
    }
    return "Unknown";
}
} // namespace positioning_arm
