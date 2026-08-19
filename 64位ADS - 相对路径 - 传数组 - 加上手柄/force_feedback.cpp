#include "force_feedback.h"

void process_force_feedback(
	ForceFeedbackState& ff,
	const ForceSampleFrame& sample,
	Handle& catheter_feedback_handle,
	Handle& guidewire_feedback_handle,
	GuidewireMode guidewire_mode,
	bool control_active,
	bool freeze_active,
	bool estop_hold_active,
	bool axis1_fast_return,
	bool axis6_fast_retract,
	bool clamp_hold_582_trigger,
	bool clamp_hold_587_trigger,
	ULONGLONG now_ms,
	int loop_count,
	const ControlConfig& cfg,
	const ForceCalibrationConfig& cal_cfg,
	const ForceCalibrationState& cal_state)
{
	(void)loop_count;
	ForceOutputCmd out_cmd;
	const bool fast_move_active = axis1_fast_return || axis6_fast_retract;
	const bool output_enabled =
		ff.enabled &&
		control_active &&
		!freeze_active &&
		!estop_hold_active &&
		cal_state.zeroed &&
		sample.valid;
	const bool catheter_feedback_active =
		guidewire_mode == GuidewireMode::None || guidewire_mode == GuidewireMode::Cooperative;
	const bool guidewire_feedback_active =
		guidewire_mode == GuidewireMode::Independent || guidewire_mode == GuidewireMode::Cooperative;

	if (output_enabled)
	{
		if (catheter_feedback_active)
		{
			const CalibratedForce cal = calibrate_force(
				sample.fn_1_value_v,
				sample.ft_1_value_v,
				sample.axis2_pos_rel,
				cal_cfg, cal_state);
			const double theory_582_f = cal.f_feedback_n;
			const double theory_582_n = cal.t_feedback_nm;
			ff.force_582_theory_f = theory_582_f;
			ff.force_582_theory_n = theory_582_n;
			if (fast_move_active)
			{
				if (!ff.freeze_582_active)
				{
					ff.freeze_582_f = theory_582_f;
					ff.freeze_582_n = theory_582_n;
					ff.freeze_582_active = true;
				}
				out_cmd.force_582_f = ff.freeze_582_f;
				out_cmd.force_582_n = ff.freeze_582_n;
			}
			else
			{
				ff.freeze_582_active = false;
				out_cmd.force_582_f = theory_582_f;
				out_cmd.force_582_n = theory_582_n;
			}
			if (ff.clamp_hold_enabled && clamp_hold_582_trigger)
			{
				ff.clamp_hold_582_f = out_cmd.force_582_f;
				ff.clamp_hold_582_n = out_cmd.force_582_n;
				ff.clamp_hold_582_until_ms = now_ms + 200;
				ff.clamp_hold_582_active = true;
			}
			if (ff.clamp_hold_enabled && ff.clamp_hold_582_active)
			{
				if (now_ms < ff.clamp_hold_582_until_ms)
				{
					out_cmd.force_582_f = ff.clamp_hold_582_f;
					out_cmd.force_582_n = ff.clamp_hold_582_n;
				}
				else
				{
					ff.clamp_hold_582_active = false;
				}
			}
		}
		else
		{
			ff.freeze_582_active = false;
			ff.clamp_hold_582_active = false;
			out_cmd.force_582_f = 0.0;
			out_cmd.force_582_n = 0.0;
			ff.force_582_theory_f = 0.0;
			ff.force_582_theory_n = 0.0;
		}

		if (guidewire_feedback_active)
		{
			const CalibratedForce cal = calibrate_guidewire_force(
				sample.fn_2_value_v,
				sample.ft_2_value_v,
				sample.axis2_pos_rel,
				cal_cfg, cal_state);
			if (fast_move_active)
			{
				if (!ff.freeze_587_active)
				{
					ff.freeze_587_f = cal.f_feedback_n;
					ff.freeze_587_n = cal.t_feedback_nm;
					ff.freeze_587_active = true;
				}
				out_cmd.force_587_f = ff.freeze_587_f;
				out_cmd.force_587_n = ff.freeze_587_n;
			}
			else
			{
				ff.freeze_587_active = false;
				out_cmd.force_587_f = cal.f_feedback_n;
				out_cmd.force_587_n = cal.t_feedback_nm;
			}
			if (ff.clamp_hold_enabled && clamp_hold_587_trigger)
			{
				ff.clamp_hold_587_f = out_cmd.force_587_f;
				ff.clamp_hold_587_n = out_cmd.force_587_n;
				ff.clamp_hold_587_until_ms = now_ms + 200;
				ff.clamp_hold_587_active = true;
			}
			if (ff.clamp_hold_enabled && ff.clamp_hold_587_active)
			{
				if (now_ms < ff.clamp_hold_587_until_ms)
				{
					out_cmd.force_587_f = ff.clamp_hold_587_f;
					out_cmd.force_587_n = ff.clamp_hold_587_n;
				}
				else
				{
					ff.clamp_hold_587_active = false;
				}
			}
		}
		else
		{
			ff.freeze_587_active = false;
			ff.clamp_hold_587_active = false;
			out_cmd.force_587_f = 0.0;
			out_cmd.force_587_n = 0.0;
		}
	}
	else
	{
		ff.clear_clamp_holds();
		ff.freeze_582_active = false;
		out_cmd.force_582_f = 0.0;
		out_cmd.force_582_n = 0.0;
		ff.force_582_theory_f = 0.0;
		ff.force_582_theory_n = 0.0;
		ff.freeze_587_active = false;
		out_cmd.force_587_f = 0.0;
		out_cmd.force_587_n = 0.0;
	}

	// 单手柄模式的两个逻辑角色会指向同一对象，此时只下发当前模式对应的一条命令。
	if (&catheter_feedback_handle == &guidewire_feedback_handle)
	{
		const bool use_guidewire_output = guidewire_mode != GuidewireMode::None;
		catheter_feedback_handle.setforce_axis(
			use_guidewire_output ? out_cmd.force_587_f : out_cmd.force_582_f,
			cfg.axial_force_axis,
			use_guidewire_output ? out_cmd.force_587_n : out_cmd.force_582_n);
	}
	else
	{
		catheter_feedback_handle.setforce_axis(
			out_cmd.force_582_f, cfg.axial_force_axis, out_cmd.force_582_n);
		guidewire_feedback_handle.setforce_axis(
			out_cmd.force_587_f, cfg.axial_force_axis, out_cmd.force_587_n);
	}

	ff.force_582_f = out_cmd.force_582_f;
	ff.force_582_n = out_cmd.force_582_n;
	ff.force_587_f = out_cmd.force_587_f;
	ff.force_587_n = out_cmd.force_587_n;

	if (!output_enabled || !catheter_feedback_active)
	{
		ff.force_582_theory_f = 0.0;
		ff.force_582_theory_n = 0.0;
	}

	if (sample.valid)
	{
		ff.last_fn_1_raw = sample.fn_1_value;
		ff.last_ft_1_raw = sample.ft_1_value;
		ff.last_fn_2_raw = sample.fn_2_value;
		ff.last_ft_2_raw = sample.ft_2_value;
	}
}
