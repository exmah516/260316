// 文件职责说明：
// 1) 实现主从位移实验的分段累计与单向受限 PI。
// 2) 只放大当前持续的正向手柄输入，不产生松手后的自主追赶或反向命令。
// 3) 不访问 ADS、不做磁盘 I/O，也不修改 PLC 的废弃 Kp/Ki/Kd 变量。
#include "delivery_tracking.h"

#include <algorithm>
#include <cmath>

namespace
{
	constexpr double kIncrementEpsilonMm = 1e-9;

	double clamp_double(double value, double lower, double upper)
	{
		return std::max(lower, std::min(value, upper));
	}
}

DeliveryTrackingController::DeliveryTrackingController()
{
	for (int i = 0; i < 2; ++i)
	{
		axis_[i].snapshot.invalid_reason = static_cast<int>(TrackingInvalidReason::NotLogging);
	}
}

void DeliveryTrackingController::start_session()
{
	session_active_ = true;
	compensation_enabled_ = false;
	next_segment_id_ = 0;
	for (int i = 0; i < 2; ++i)
	{
		axis_[i] = AxisRuntime{};
		axis_[i].snapshot.invalid_reason = static_cast<int>(TrackingInvalidReason::NotForwardDelivery);
	}
}

void DeliveryTrackingController::stop_session()
{
	session_active_ = false;
	compensation_enabled_ = false;
	for (int i = 0; i < 2; ++i)
	{
		end_segment(axis_[i], TrackingInvalidReason::NotLogging, false);
	}
}

bool DeliveryTrackingController::set_parameter(TrackingParameterField field, double value)
{
	if (compensation_enabled_ || !is_finite(value))
	{
		return false;
	}

	DeliveryTrackingAxis axis = DeliveryTrackingAxis::Axis1;
	enum class ParameterPart { Kp, Ki, MaxGain, MaxError } part = ParameterPart::Kp;
	switch (field)
	{
	case TrackingParameterField::Axis1Kp: axis = DeliveryTrackingAxis::Axis1; part = ParameterPart::Kp; break;
	case TrackingParameterField::Axis1Ki: axis = DeliveryTrackingAxis::Axis1; part = ParameterPart::Ki; break;
	case TrackingParameterField::Axis1MaxGain: axis = DeliveryTrackingAxis::Axis1; part = ParameterPart::MaxGain; break;
	case TrackingParameterField::Axis1MaxError: axis = DeliveryTrackingAxis::Axis1; part = ParameterPart::MaxError; break;
	case TrackingParameterField::Axis6Kp: axis = DeliveryTrackingAxis::Axis6; part = ParameterPart::Kp; break;
	case TrackingParameterField::Axis6Ki: axis = DeliveryTrackingAxis::Axis6; part = ParameterPart::Ki; break;
	case TrackingParameterField::Axis6MaxGain: axis = DeliveryTrackingAxis::Axis6; part = ParameterPart::MaxGain; break;
	case TrackingParameterField::Axis6MaxError: axis = DeliveryTrackingAxis::Axis6; part = ParameterPart::MaxError; break;
	default: return false;
	}

	if (part == ParameterPart::Kp && (value < 0.0 || value > kMaxUiKp))
	{
		return false;
	}
	if (part == ParameterPart::Ki && (value < 0.0 || value > kMaxUiKi))
	{
		return false;
	}
	if (part == ParameterPart::MaxGain && (value <= 1.0 || value > kMaxUiGain))
	{
		return false;
	}
	if (part == ParameterPart::MaxError && (value <= 0.0 || value > kMaxUiErrorMm))
	{
		return false;
	}

	DeliveryTrackingParameters& params = parameters_[static_cast<int>(axis)];
	switch (part)
	{
	case ParameterPart::Kp: params.kp = value; break;
	case ParameterPart::Ki: params.ki = value; break;
	case ParameterPart::MaxGain: params.max_gain = value; break;
	case ParameterPart::MaxError:
		params.max_error_mm = value;
		{
			AxisRuntime& state = runtime(axis);
			if (state.pending_handover_error_mm > value)
			{
				state.pending_handover_error_mm = value;
				state.pending_handover_error_limited = true;
			}
			refresh_error(state);
		}
		break;
	}
	return true;
}

const DeliveryTrackingParameters& DeliveryTrackingController::parameters(DeliveryTrackingAxis axis) const
{
	return parameters_[static_cast<int>(axis)];
}

bool DeliveryTrackingController::can_enable_compensation(DeliveryTrackingAxis active_axis) const
{
	const DeliveryTrackingParameters& params = parameters(active_axis);
	const AxisRuntime& state = runtime(active_axis);
	return session_active_ && state.active && params.kp > 0.0 &&
		params.max_gain > 1.0 && params.max_error_mm > 0.0;
}

bool DeliveryTrackingController::set_compensation_enabled(bool enabled, DeliveryTrackingAxis active_axis)
{
	if (enabled && !can_enable_compensation(active_axis))
	{
		return false;
	}
	compensation_enabled_ = enabled;
	if (!enabled)
	{
		for (int i = 0; i < 2; ++i)
		{
			reset_pi(axis_[i]);
		}
	}
	return true;
}

void DeliveryTrackingController::disable_compensation()
{
	compensation_enabled_ = false;
	for (int i = 0; i < 2; ++i)
	{
		reset_pi(axis_[i]);
	}
}

void DeliveryTrackingController::update_gate(
	DeliveryTrackingAxis axis,
	bool base_eligible,
	bool grip_command_active,
	std::uint32_t now_ms,
	double actual_abs_mm,
	TrackingInvalidReason invalid_reason)
{
	AxisRuntime& state = runtime(axis);
	if (!session_active_)
	{
		end_segment(state, TrackingInvalidReason::NotLogging, false);
		return;
	}

	if (!base_eligible)
	{
		end_segment(state, invalid_reason, false);
		return;
	}

	if (!grip_command_active)
	{
		end_segment(state, TrackingInvalidReason::GripSettling, false);
		return;
	}

	if (state.grip_command_since_ms == 0)
	{
		state.grip_command_since_ms = now_ms;
	}
	if ((now_ms - state.grip_command_since_ms) < kGripSettleMs)
	{
		end_segment(state, TrackingInvalidReason::GripSettling, true);
		return;
	}

	if (!state.active)
	{
		begin_segment(state, now_ms, actual_abs_mm);
	}
	else
	{
		update_actual_progress(state, actual_abs_mm);
	}
	state.snapshot.segment_active = true;
	state.snapshot.grip_assumed = true;
	state.snapshot.invalid_reason = static_cast<int>(TrackingInvalidReason::None);
}

void DeliveryTrackingController::invalidate_all(TrackingInvalidReason reason)
{
	for (int i = 0; i < 2; ++i)
	{
		end_segment(axis_[i], reason, false);
	}
}

void DeliveryTrackingController::begin_cycle(
	DeliveryTrackingAxis axis,
	double handle_raw,
	double handle_filtered,
	double raw_mapping_increment_axis_mm,
	double actual_rel_mm)
{
	AxisRuntime& state = runtime(axis);
	state.snapshot.handle_raw = handle_raw;
	state.snapshot.handle_filtered = handle_filtered;
	state.snapshot.raw_mapping_increment_axis_mm = raw_mapping_increment_axis_mm;
	state.snapshot.nominal_forward_increment_mm = 0.0;
	state.snapshot.compensated_requested_forward_increment_mm = 0.0;
	state.snapshot.effective_forward_increment_mm = 0.0;
	state.snapshot.handover_forward_increment_mm = 0.0;
	state.snapshot.compensation_gain = 1.0;
	state.snapshot.actual_rel_mm = actual_rel_mm;
}

void DeliveryTrackingController::record_handover_forward_increment(
	DeliveryTrackingAxis axis,
	double forward_increment_mm)
{
	AxisRuntime& state = runtime(axis);
	if (!session_active_ || forward_increment_mm <= kIncrementEpsilonMm)
	{
		return;
	}

	const double forward_increment = std::max(0.0, forward_increment_mm);
	state.snapshot.handover_forward_increment_mm = forward_increment;
	state.snapshot.session_handover_forward_mm += forward_increment;
	// 会话总主端位移保留完整原始前推量；补偿队列只保留安全上界以内的欠账。
	state.snapshot.session_master_forward_mm += forward_increment;

	const double error_limit_mm = parameters(axis).max_error_mm;
	const double candidate_error_mm = state.pending_handover_error_mm + forward_increment;
	state.pending_handover_error_mm = std::min(candidate_error_mm, error_limit_mm);
	state.pending_handover_error_limited =
		candidate_error_mm > (error_limit_mm + kIncrementEpsilonMm);
	refresh_error(state);
}

double DeliveryTrackingController::request_compensated_forward_increment(
	DeliveryTrackingAxis axis,
	double nominal_forward_increment_mm,
	std::uint32_t now_ms)
{
	AxisRuntime& state = runtime(axis);
	state.snapshot.nominal_forward_increment_mm = std::max(0.0, nominal_forward_increment_mm);
	state.snapshot.compensated_requested_forward_increment_mm = state.snapshot.nominal_forward_increment_mm;
	state.snapshot.compensation_gain = 1.0;

	if (!compensation_enabled_ || !state.active || nominal_forward_increment_mm <= kIncrementEpsilonMm)
	{
		return std::max(0.0, nominal_forward_increment_mm);
	}

	refresh_error(state);
	const DeliveryTrackingParameters& params = parameters(axis);
	state.integral_before_last_update_mm_s = state.integral_mm_s;
	const double available_handover_error_mm = std::max(
		0.0,
		state.pending_handover_error_mm - state.compensation_forward_outstanding_mm);
	if (available_handover_error_mm <= kIncrementEpsilonMm)
	{
		// 已发出的补偿尚未由实际位置确认，或欠账已清零：只泄放积分，不重复下发补偿。
		state.integral_mm_s *= 0.75;
		state.snapshot.p_term = 0.0;
		state.snapshot.i_term = params.ki * state.integral_mm_s;
		return nominal_forward_increment_mm;
	}

	double dt_s = 0.0;
	if (state.last_pi_tick_ms != 0)
	{
		dt_s = static_cast<double>(now_ms - state.last_pi_tick_ms) / 1000.0;
		// 调试中断或断点恢复不能把长时间空档积分为一次大输出。
		dt_s = clamp_double(dt_s, 0.0, 0.10);
	}
	state.last_pi_tick_ms = now_ms;

	const double bounded_error_mm = std::min(available_handover_error_mm, params.max_error_mm);
	state.integral_mm_s += bounded_error_mm * dt_s;
	const double integral_before_clamp = state.integral_mm_s;
	state.integral_mm_s = clamp_double(state.integral_mm_s, 0.0, kMaxIntegralMmS);
	state.snapshot.integral_limited = std::abs(state.integral_mm_s - integral_before_clamp) > kIncrementEpsilonMm;
	state.snapshot.p_term = params.kp * bounded_error_mm;
	state.snapshot.i_term = params.ki * state.integral_mm_s;

	const double extra_gain = clamp_double(
		state.snapshot.p_term + state.snapshot.i_term,
		0.0,
		params.max_gain - 1.0);
	const double requested_extra_mm = std::min(
		std::min(nominal_forward_increment_mm * extra_gain, kMaxCompensationStepMm),
		available_handover_error_mm);
	const double requested_forward_mm = nominal_forward_increment_mm + requested_extra_mm;
	state.snapshot.compensated_requested_forward_increment_mm = requested_forward_mm;
	state.snapshot.compensation_gain = requested_forward_mm / nominal_forward_increment_mm;
	return requested_forward_mm;
}

void DeliveryTrackingController::commit_command(
	DeliveryTrackingAxis axis,
	double nominal_forward_increment_mm,
	double compensated_requested_forward_increment_mm,
	double effective_forward_increment_mm,
	bool output_clamped)
{
	AxisRuntime& state = runtime(axis);
	const double nominal = std::max(0.0, nominal_forward_increment_mm);
	const double requested = std::max(0.0, compensated_requested_forward_increment_mm);
	const double effective = std::max(0.0, effective_forward_increment_mm);
	state.snapshot.nominal_forward_increment_mm = nominal;
	state.snapshot.compensated_requested_forward_increment_mm = requested;
	state.snapshot.effective_forward_increment_mm = effective;
	if (nominal > kIncrementEpsilonMm)
	{
		state.snapshot.compensation_gain = requested / nominal;
	}
	else
	{
		state.snapshot.compensation_gain = 1.0;
	}

	if (state.active && nominal > kIncrementEpsilonMm)
	{
		state.snapshot.segment_master_forward_mm += nominal;
		state.snapshot.session_master_forward_mm += nominal;
		state.nominal_forward_outstanding_mm += nominal;
	}
	if (output_clamped || (requested > kIncrementEpsilonMm && effective + kIncrementEpsilonMm < requested))
	{
		// 输出被现有窗口或停止条件接管时，撤销本拍积分，避免形成不可执行的 PI 欠账。
		state.integral_mm_s = state.integral_before_last_update_mm_s;
		state.snapshot.i_term = parameters(axis).ki * state.integral_mm_s;
		state.snapshot.integral_limited = false;
		state.last_pi_tick_ms = 0;
	}

	if (state.active)
	{
		const double effective_extra_mm = std::max(0.0, effective - nominal);
		const double available_handover_error_mm = std::max(
			0.0,
			state.pending_handover_error_mm - state.compensation_forward_outstanding_mm);
		state.compensation_forward_outstanding_mm += std::min(
			effective_extra_mm,
			available_handover_error_mm);
	}
	refresh_error(state);
}

void DeliveryTrackingController::finish_cycle(DeliveryTrackingAxis axis, double refer_rel_mm, double actual_rel_mm)
{
	AxisRuntime& state = runtime(axis);
	state.snapshot.refer_rel_mm = refer_rel_mm;
	state.snapshot.actual_rel_mm = actual_rel_mm;
	refresh_error(state);
}

DeliveryTrackingAxisSnapshot DeliveryTrackingController::snapshot(DeliveryTrackingAxis axis) const
{
	return runtime(axis).snapshot;
}

DeliveryTrackingController::AxisRuntime& DeliveryTrackingController::runtime(DeliveryTrackingAxis axis)
{
	return axis_[static_cast<int>(axis)];
}

const DeliveryTrackingController::AxisRuntime& DeliveryTrackingController::runtime(DeliveryTrackingAxis axis) const
{
	return axis_[static_cast<int>(axis)];
}

void DeliveryTrackingController::reset_pi(AxisRuntime& state)
{
	state.integral_mm_s = 0.0;
	state.integral_before_last_update_mm_s = 0.0;
	state.last_pi_tick_ms = 0;
	state.snapshot.compensation_gain = 1.0;
	state.snapshot.p_term = 0.0;
	state.snapshot.i_term = 0.0;
	state.snapshot.integral_limited = false;
}

void DeliveryTrackingController::reset_handover_error(AxisRuntime& state)
{
	state.pending_handover_error_mm = 0.0;
	state.pending_handover_error_limited = false;
	state.snapshot.tracking_error_mm = 0.0;
	state.snapshot.tracking_error_limited = false;
}

void DeliveryTrackingController::clear_outstanding_motion(AxisRuntime& state)
{
	state.nominal_forward_outstanding_mm = 0.0;
	state.compensation_forward_outstanding_mm = 0.0;
}

void DeliveryTrackingController::end_segment(
	AxisRuntime& state,
	TrackingInvalidReason reason,
	bool keep_grip_timer)
{
	state.active = false;
	if (!keep_grip_timer)
	{
		state.grip_command_since_ms = 0;
	}
	reset_pi(state);
	clear_outstanding_motion(state);
	if (!should_preserve_handover_error(reason))
	{
		reset_handover_error(state);
	}
	state.snapshot.segment_active = false;
	state.snapshot.grip_assumed = false;
	state.snapshot.invalid_reason = static_cast<int>(reason);
	state.snapshot.actual_forward_delta_mm = 0.0;
	state.snapshot.segment_master_forward_mm = 0.0;
	state.snapshot.segment_actual_forward_mm = 0.0;
	if (reason != TrackingInvalidReason::CrawlReturnActive)
	{
		state.snapshot.handover_forward_increment_mm = 0.0;
	}
	refresh_error(state);
}

void DeliveryTrackingController::begin_segment(AxisRuntime& state, std::uint32_t now_ms, double actual_abs_mm)
{
	state.active = true;
	state.last_actual_abs_mm = actual_abs_mm;
	state.last_pi_tick_ms = now_ms;
	state.integral_mm_s = 0.0;
	state.snapshot.segment_id = ++next_segment_id_;
	state.snapshot.segment_active = true;
	state.snapshot.grip_assumed = true;
	state.snapshot.actual_forward_delta_mm = 0.0;
	state.snapshot.segment_master_forward_mm = 0.0;
	state.snapshot.segment_actual_forward_mm = 0.0;
	state.snapshot.p_term = 0.0;
	state.snapshot.i_term = 0.0;
	state.snapshot.integral_limited = false;
	state.snapshot.compensation_gain = 1.0;
	clear_outstanding_motion(state);
	refresh_error(state);
}

void DeliveryTrackingController::update_actual_progress(AxisRuntime& state, double actual_abs_mm)
{
	const double delta_forward_mm = std::max(0.0, state.last_actual_abs_mm - actual_abs_mm);
	state.last_actual_abs_mm = actual_abs_mm;
	state.snapshot.actual_forward_delta_mm = delta_forward_mm;
	state.snapshot.segment_actual_forward_mm += delta_forward_mm;
	state.snapshot.session_actual_forward_mm += delta_forward_mm;

	// 实际位移先满足普通手柄映射，再把超出的部分视为已确认的补偿位移。
	// 这样普通 Follow 的正常前进不会错误地冲销换手欠账。
	double remaining_forward_mm = delta_forward_mm;
	const double nominal_settled_mm = std::min(
		remaining_forward_mm,
		state.nominal_forward_outstanding_mm);
	state.nominal_forward_outstanding_mm -= nominal_settled_mm;
	remaining_forward_mm -= nominal_settled_mm;
	const double compensation_settled_mm = std::min(
		remaining_forward_mm,
		state.compensation_forward_outstanding_mm);
	state.compensation_forward_outstanding_mm -= compensation_settled_mm;
	state.pending_handover_error_mm = std::max(
		0.0,
		state.pending_handover_error_mm - compensation_settled_mm);
	if (compensation_settled_mm > kIncrementEpsilonMm)
	{
		state.pending_handover_error_limited = false;
	}
	refresh_error(state);
}

void DeliveryTrackingController::refresh_error(AxisRuntime& state)
{
	state.snapshot.tracking_error_mm = state.pending_handover_error_mm;
	state.snapshot.tracking_error_limited = state.pending_handover_error_limited;
}

bool DeliveryTrackingController::should_preserve_handover_error(TrackingInvalidReason reason)
{
	return reason == TrackingInvalidReason::CrawlReturnActive ||
		reason == TrackingInvalidReason::GripSettling;
}

bool DeliveryTrackingController::is_finite(double value)
{
	return std::isfinite(value) != 0;
}
