#include "positioning_arm/kinematics.h"

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace positioning_arm {
namespace {
using TaskVector = Eigen::Matrix<double, 6, 1>;
constexpr double Pi = 3.14159265358979323846;

void require(bool condition, const char* message)
{
    if (!condition) throw std::invalid_argument(message);
}

bool positive(double value)
{
    return std::isfinite(value) && value > 0;
}

Eigen::Quaterniond normalized(Eigen::Quaterniond value)
{
    require(value.coeffs().allFinite() && positive(value.norm()), "四元数无效");
    value.normalize();
    return value;
}

Eigen::Quaterniond rotationZ(double angle)
{
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
}

Eigen::Vector3d rotationError(const Eigen::Quaterniond& target,
    const Eigen::Quaterniond& current)
{
    Eigen::Quaterniond delta = normalized(target * current.conjugate());
    if (delta.w() < 0) delta.coeffs() *= -1;
    const double norm = delta.vec().norm();
    if (norm < 1e-12) return 2 * delta.vec();
    return delta.vec() * (2 * std::atan2(norm, delta.w()) / norm);
}

ArmKinematicConfig validate(ArmKinematicConfig config, const SolverOptions& options)
{
    require(config.link_lengths.allFinite() && (config.link_lengths.array() > 0).all(),
        "两根连杆长度必须为正");
    require(positive(config.lateral_offset) && positive(config.tool_length),
        "L形工具尺寸必须为正");
    require((config.lower.array() < config.upper.array()).all()
        && config.lower.head<2>().allFinite() && config.upper.head<2>().allFinite(),
        "轴1、2必须设置有限范围，其他轴范围必须有序且无NaN");
    require(config.max_velocity.allFinite()
        && (config.max_velocity.array() > 0).all(),
        "五轴速度限位必须为正");
    for (const auto& calibration : config.calibration) {
        require(std::isfinite(calibration.zero) && positive(calibration.scale)
            && (calibration.direction == -1 || calibration.direction == 1),
            "编码器标定参数无效");
    }
    require(options.max_iterations > 0 && options.max_iterations <= 10000,
        "迭代次数无效");
    for (double value : {options.position_tolerance, options.orientation_tolerance,
        options.damping, options.singularity_stop, options.singularity_slow,
        options.max_translation, options.max_rotation, options.max_linear_iteration,
        options.max_angular_iteration}) {
        require(positive(value), "求解器参数必须为正");
    }
    require(std::isfinite(options.posture_weight) && options.posture_weight >= 0,
        "姿态保持权重无效");
    require(options.singularity_stop < options.singularity_slow
        && options.max_rotation < Pi && options.max_angular_iteration < Pi,
        "奇异阈值或角度步长无效");
    return config;
}
} // namespace

PositioningArmKinematics::PositioningArmKinematics(
    const ArmKinematicConfig& config, const SolverOptions& options)
    : config_(validate(config, options)), options_(options)
{
    characteristic_length_ = config_.link_lengths.sum()
        + std::hypot(config_.lateral_offset, config_.tool_length);
    require(positive(characteristic_length_), "特征长度无效");
}

PositioningArmKinematics::~PositioningArmKinematics() = default;

ArmPose PositioningArmKinematics::forward(const JointVector& joints) const
{
    require(checkLimits(joints), "轴位置无效或超出限位");

    ArmPose result;
    // 省略固定高度，位置Z仅用于增量计算，不代表相对地面高度。
    result.position << 0, 0, joints[0];

    double yaw = 0;
    for (int i = 0; i < 2; ++i) {
        yaw += joints[i + 1];
        result.position += config_.link_lengths[i]
            * Eigen::Vector3d(std::sin(yaw), -std::cos(yaw), 0);
    }

    // 机械角总和450度时工具坐标与基座对齐，程序偏置不能在这里重复添加。
    yaw += joints[3] - Pi / 2;
    const Eigen::Quaterniond wrist_rotation = rotationZ(yaw)
        * Eigen::Quaterniond(Eigen::AngleAxisd(joints[4], Eigen::Vector3d::UnitX()));
    result.position += wrist_rotation
        * Eigen::Vector3d(config_.lateral_offset, -config_.tool_length, 0);
    result.quaternion = normalized(wrist_rotation);
    if (result.quaternion.w() < 0) result.quaternion.coeffs() *= -1;
    return result;
}

FrameJacobian PositioningArmKinematics::computeFrameJacobian(
    const JointVector& joints) const
{
    require(checkLimits(joints), "轴位置无效或超出限位");

    const ArmPose pose = forward(joints);
    FrameJacobian result = FrameJacobian::Zero();
    result.block<3, 1>(0, 0) = Eigen::Vector3d::UnitZ();

    const double z = joints[0];
    Eigen::Vector3d joint_origin(0, 0, z);
    double yaw = 0;
    for (int i = 0; i < 3; ++i) {
        const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
        result.block<3, 1>(0, i + 1) = axis.cross(pose.position - joint_origin);
        result.block<3, 1>(3, i + 1) = axis;
        yaw += joints[i + 1];
        if (i < 2) joint_origin += config_.link_lengths[i]
            * Eigen::Vector3d(std::sin(yaw), -std::cos(yaw), 0);
    }

    const Eigen::Vector3d pitch_axis_world =
        rotationZ(yaw - Pi / 2) * Eigen::Vector3d::UnitX();
    result.block<3, 1>(0, 4) =
        pitch_axis_world.cross(pose.position - joint_origin);
    result.block<3, 1>(3, 4) = pitch_axis_world;
    return result;
}

bool PositioningArmKinematics::checkLimits(const JointVector& joints) const
{
    return joints.allFinite()
        && (joints.array() >= config_.lower.array()).all()
        && (joints.array() <= config_.upper.array()).all();
}

bool PositioningArmKinematics::isNearSingularity(const JointVector& joints) const
{
    require(checkLimits(joints), "轴位置无效或超出限位");

    // 平面机构在轴3伸直或折叠时，优先按机械结构直接判定。
    if (std::abs(std::sin(joints[2])) < options_.singularity_slow) return true;

    const auto full = computeFrameJacobian(joints);
    Eigen::Matrix3d jacobian;
    jacobian.topRows<2>() = full.block<2, 3>(0, 1) / characteristic_length_;
    jacobian.bottomRows<1>().setOnes();
    return Eigen::JacobiSVD<Eigen::Matrix3d>(jacobian)
        .singularValues().minCoeff() < options_.singularity_slow;
}

JointVector PositioningArmKinematics::fromEncoder(const JointVector& encoder) const
{
    require(encoder.allFinite(), "编码器位置无效");
    JointVector joints;
    for (int i = 0; i < AxisCount; ++i) {
        const auto& calibration = config_.calibration[i];
        joints[i] = (encoder[i] - calibration.zero)
            * calibration.scale * calibration.direction;
    }
    require(checkLimits(joints), "编码器换算后超出轴限位");
    return joints;
}

JointVector PositioningArmKinematics::toEncoder(const JointVector& joints) const
{
    require(checkLimits(joints), "轴位置无效或超出限位");
    JointVector encoder;
    for (int i = 0; i < AxisCount; ++i) {
        const auto& calibration = config_.calibration[i];
        encoder[i] = joints[i]
            / (calibration.scale * calibration.direction) + calibration.zero;
    }
    require(encoder.allFinite(), "编码器换算结果无效");
    return encoder;
}

Eigen::Quaterniond PositioningArmKinematics::continuousQuaternion(
    Eigen::Quaterniond value, const Eigen::Quaterniond& reference)
{
    value = normalized(value);
    if (value.dot(normalized(reference)) < 0) value.coeffs() *= -1;
    return value;
}

IkResult PositioningArmKinematics::solveCartesianJog(
    const JointVector& current, const CartesianJog& command) const
{
    IkResult result;
    result.target = current;
    if (!current.allFinite() || !command.translation.allFinite()
        || !std::isfinite(command.yaw) || !std::isfinite(command.pitch)
        || !positive(command.duration) || command.duration > 1) {
        return result;
    }
    if (!checkLimits(current)) {
        result.status = IkStatus::JointLimit;
        return result;
    }
    const ArmPose current_pose = forward(current);
    result.pose = current_pose;
    const auto active = (command.translation.array() != 0).count()
        + (command.yaw != 0) + (command.pitch != 0);
    if (active > 1) return result;
    if (command.translation.norm() > options_.max_translation
        || std::abs(command.yaw) + std::abs(command.pitch) > options_.max_rotation) {
        result.status = IkStatus::StepLimit;
        return result;
    }

    // 升降、抬头和静止命令不经过平面逆解，也不受平面机械奇异影响。
    if (active == 0 || command.translation.z() != 0 || command.pitch != 0) {
        JointVector next = current;
        next[0] += command.translation.z();
        next[4] -= command.pitch;
        if (!checkLimits(next)) {
            result.status = IkStatus::JointLimit;
            return result;
        }
        if (((next - current).cwiseAbs().array()
            > (config_.max_velocity * command.duration).array()).any()) {
            result.status = IkStatus::StepLimit;
            return result;
        }
        result.status = IkStatus::Success;
        result.target = next;
        result.pose = forward(next);
        result.pose.quaternion = continuousQuaternion(result.pose.quaternion, current_pose.quaternion);
        result.position_error = result.orientation_error = 0;
        return result;
    }

    ArmPose target = current_pose;
    target.position += command.translation;
    target.quaternion = normalized(
        rotationZ(command.yaw) * current_pose.quaternion);
    target.quaternion = continuousQuaternion(target.quaternion, current_pose.quaternion);

    const JointVector travel = config_.max_velocity * command.duration;
    if (!travel.allFinite()) return result;
    const JointVector lower = config_.lower.cwiseMax(current - travel);
    const JointVector upper = config_.upper.cwiseMin(current + travel);
    JointVector joints = current;
    IkStatus blocked = IkStatus::NoConvergence;

    for (int iteration = 0; iteration <= options_.max_iterations; ++iteration) {
        result.iterations = iteration;
        const ArmPose actual = forward(joints);
        TaskVector error;
        error.head<3>() = (target.position - actual.position) / characteristic_length_;
        error.tail<3>() = rotationError(target.quaternion, actual.quaternion);
        result.position_error = (target.position - actual.position).norm();
        result.orientation_error = error.tail<3>().norm();

        FrameJacobian jacobian = computeFrameJacobian(joints);
        jacobian.topRows<3>() /= characteristic_length_;
        Eigen::Matrix3d planar;
        planar.topRows<2>() = jacobian.block<2, 3>(0, 1);
        planar.bottomRows<1>().setOnes();
        const double sigma = Eigen::JacobiSVD<Eigen::Matrix3d>(planar)
            .singularValues().minCoeff();
        jacobian.col(0).setZero();
        jacobian.col(4).setZero();
        if (result.position_error <= options_.position_tolerance
            && result.orientation_error <= options_.orientation_tolerance) {
            if (sigma < options_.singularity_stop
                && (joints - current).norm() > 0) {
                result.status = IkStatus::NearSingularity;
                return result;
            }
            result.status = IkStatus::Success;
            result.target = joints;
            result.pose = actual;
            result.pose.quaternion = continuousQuaternion(
                actual.quaternion, current_pose.quaternion);
            return result;
        }
        if (sigma < options_.singularity_stop) {
            result.status = IkStatus::NearSingularity;
            return result;
        }
        if (iteration == options_.max_iterations) break;

        const double slowdown = std::min(1.0, sigma / options_.singularity_slow);
        const double damping = options_.damping / std::max(slowdown, 0.01);
        JointVector scaled_offset = joints - current;
        scaled_offset[0] /= characteristic_length_;
        Eigen::Matrix<double, AxisCount, AxisCount> normal =
            jacobian.transpose() * jacobian;
        normal.diagonal().array() += damping * damping + options_.posture_weight;
        const JointVector scaled_step = normal.ldlt().solve(
            jacobian.transpose() * error
            - options_.posture_weight * scaled_offset);
        JointVector step = scaled_step;
        step[0] *= characteristic_length_;
        if (!step.allFinite()) {
            result.status = IkStatus::NoConvergence;
            return result;
        }

        double alpha = slowdown;
        if (std::abs(step[0]) > options_.max_linear_iteration)
            alpha = std::min(alpha, options_.max_linear_iteration / std::abs(step[0]));
        for (int i = 1; i < AxisCount; ++i) {
            if (std::abs(step[i]) > options_.max_angular_iteration)
                alpha = std::min(alpha,
                    options_.max_angular_iteration / std::abs(step[i]));
        }
        for (int i = 0; i < AxisCount; ++i) {
            if (step[i] == 0) continue;
            const double bound = step[i] > 0 ? upper[i] : lower[i];
            const double allowed = (bound - joints[i]) / step[i];
            if (allowed < alpha) {
                alpha = std::max(0.0, allowed);
                const bool physical = step[i] > 0
                    ? upper[i] == config_.upper[i] : lower[i] == config_.lower[i];
                blocked = physical ? IkStatus::JointLimit : IkStatus::StepLimit;
            }
        }

        const double merit = error.squaredNorm()
            + options_.posture_weight * scaled_offset.squaredNorm();
        bool accepted = false;
        for (int backtrack = 0; backtrack < 24 && alpha > 1e-14;
            ++backtrack, alpha *= 0.5) {
            const JointVector candidate = joints + alpha * step;
            if (!checkLimits(candidate)
                || (candidate.array() < lower.array()).any()
                || (candidate.array() > upper.array()).any()) {
                continue;
            }
            // 不跨越当前肘部支路，也不主动穿越机械奇异位形。
            const auto candidate_full = computeFrameJacobian(candidate);
            Eigen::Matrix3d candidate_planar;
            candidate_planar.topRows<2>() =
                candidate_full.block<2, 3>(0, 1) / characteristic_length_;
            candidate_planar.bottomRows<1>().setOnes();
            if (std::sin(joints[2]) * std::sin(candidate[2]) <= 0
                || Eigen::JacobiSVD<Eigen::Matrix3d>(candidate_planar)
                    .singularValues().minCoeff() < options_.singularity_stop) {
                blocked = IkStatus::NearSingularity;
                continue;
            }
            const ArmPose next_pose = forward(candidate);
            TaskVector next_error;
            next_error.head<3>() =
                (target.position - next_pose.position) / characteristic_length_;
            next_error.tail<3>() =
                rotationError(target.quaternion, next_pose.quaternion);
            JointVector next_offset = candidate - current;
            next_offset[0] /= characteristic_length_;
            if (next_error.squaredNorm()
                    + options_.posture_weight * next_offset.squaredNorm() < merit) {
                joints = candidate;
                accepted = true;
                break;
            }
        }
        if (!accepted) break;
    }
    result.status = blocked;
    return result;
}

IkResult PositioningArmKinematics::solveJog(
    const JointVector& current, const JogCommand& command) const
{
    CartesianJog jog;
    jog.duration = command.duration;
    double scale = 1;
    if (checkLimits(current) && (command.axis == JogAxis::X
        || command.axis == JogAxis::Y || command.axis == JogAxis::Yaw)) {
        const auto full = computeFrameJacobian(current);
        Eigen::Matrix3d planar;
        planar.topRows<2>() = full.block<2, 3>(0, 1) / characteristic_length_;
        planar.bottomRows<1>().setOnes();
        const double sigma = Eigen::JacobiSVD<Eigen::Matrix3d>(planar)
            .singularValues().minCoeff();
        // 硬停止区保留非零命令，让逆解明确报告奇异，而非伪装成静止成功。
        if (sigma >= options_.singularity_stop)
            scale = std::min(1.0, sigma / options_.singularity_slow);
    }
    const double displacement = command.speed * command.duration * scale;
    switch (command.axis) {
    case JogAxis::X: jog.translation.x() = displacement; break;
    case JogAxis::Y: jog.translation.y() = displacement; break;
    case JogAxis::Z: jog.translation.z() = displacement; break;
    case JogAxis::Pitch: jog.pitch = displacement; break;
    case JogAxis::Yaw: jog.yaw = displacement; break;
    default: {
        IkResult result;
        result.target = current;
        return result;
    }
    }
    auto result = solveCartesianJog(current, jog);
    result.speed_scale = scale;
    return result;
}

const char* statusName(IkStatus status)
{
    switch (status) {
    case IkStatus::Success: return "Success";
    case IkStatus::InvalidInput: return "InvalidInput";
    case IkStatus::JointLimit: return "JointLimit";
    case IkStatus::StepLimit: return "StepLimit";
    case IkStatus::NearSingularity: return "NearSingularity";
    case IkStatus::NoConvergence: return "NoConvergence";
    }
    return "Unknown";
}
} // namespace positioning_arm
