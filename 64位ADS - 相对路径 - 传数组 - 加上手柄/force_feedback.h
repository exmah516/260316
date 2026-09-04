#pragma once

#include "control_types.h"
#include "force_calibration.h"

void process_force_feedback(
	ForceFeedbackState& ff,
	const ForceSampleFrame& sample,
	Handle& catheter_feedback_handle,
	Handle& guidewire_feedback_handle,
	GuidewireMode guidewire_mode,
	bool control_active,
	bool estop_hold_active,
	bool axis1_fast_return,
	bool axis6_fast_retract,
	bool clamp_hold_582_trigger,
	bool clamp_hold_587_trigger,
	ULONGLONG now_ms,
	int loop_count,
	const ControlConfig& cfg,
	const ForceCalibrationConfig& cal_cfg,
	const ForceCalibrationState& cal_state);

// 保留标定自检等旧调用方的兼容入口；旧路径不触发夹爪保持。
void process_force_feedback(
	ForceFeedbackState& ff,
	const ForceSampleFrame& sample,
	Handle& catheter_feedback_handle,
	Handle& guidewire_feedback_handle,
	GuidewireMode guidewire_mode,
	bool control_active,
	bool estop_hold_active,
	bool axis1_fast_return,
	bool axis6_fast_retract,
	int loop_count,
	const ControlConfig& cfg,
	const ForceCalibrationConfig& cal_cfg,
	const ForceCalibrationState& cal_state);
