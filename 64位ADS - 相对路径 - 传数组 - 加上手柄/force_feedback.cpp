#include "force_feedback.h"

void process_force_feedback(
	ForceFeedbackState& ff,
	const ForceSampleFrame& sample,
	Handle& handle_582,
	Handle& handle_587,
	GuidewireMode guidewire_mode,
	bool control_active,
	bool freeze_active,
	bool estop_hold_active,
	bool axis1_fast_return,
	bool axis6_fast_retract,
	int loop_count,
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
		sample.valid;

	if (output_enabled)
	{
		if (guidewire_mode == GuidewireMode::None)
		{
			CalibratedForce cal = calibrate_force(
				sample.fn_1_value_v,
				sample.ft_1_value_v,
				0.0, 0.0,
				cal_cfg, cal_state);
			const double mapped_582_f = cal.f_feedback_n;
			const double mapped_582_n = cal.t_feedback_nm;
			if (fast_move_active)
			{
				if (!ff.freeze_582_active)
				{
					ff.freeze_582_f = mapped_582_f;
					ff.freeze_582_n = mapped_582_n;
					ff.freeze_582_active = true;
				}
				out_cmd.force_582_f = ff.freeze_582_f;
				out_cmd.force_582_n = ff.freeze_582_n;
			}
			else
			{
				ff.freeze_582_active = false;
				out_cmd.force_582_f = mapped_582_f;
				out_cmd.force_582_n = mapped_582_n;
			}
			out_cmd.force_587_f = 0.0;
			out_cmd.force_587_n = 0.0;
		}
		else
		{
			ff.freeze_582_active = false;
			out_cmd.force_582_f = 0.0;
			out_cmd.force_582_n = 0.0;
			out_cmd.force_587_f = 0.0;
			out_cmd.force_587_n = 0.0;
		}
	}
	else
	{
		ff.freeze_582_active = false;
		out_cmd.force_582_f = 0.0;
		out_cmd.force_582_n = 0.0;
		out_cmd.force_587_f = 0.0;
		out_cmd.force_587_n = 0.0;
	}

	handle_582.setforce(out_cmd.force_582_f, out_cmd.force_582_n);
	handle_587.setforce(out_cmd.force_587_f, out_cmd.force_587_n);

	ff.force_582_f = out_cmd.force_582_f;
	ff.force_582_n = out_cmd.force_582_n;
	ff.force_587_f = out_cmd.force_587_f;
	ff.force_587_n = out_cmd.force_587_n;

	if (sample.valid)
	{
		ff.last_fn_1_raw = sample.fn_1_value;
		ff.last_ft_1_raw = sample.ft_1_value;
		ff.last_fn_2_raw = sample.fn_2_value;
		ff.last_ft_2_raw = sample.ft_2_value;
	}
}

