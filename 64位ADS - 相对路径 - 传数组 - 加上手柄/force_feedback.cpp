// 文件职责说明：
// 1) 实现力反馈映射与输出路由逻辑。
// 2) 行为对齐原 main.cpp：导管模式输出 582、导丝模式输出 587（当前占位映射）。
// 3) 保留导管快进/快退阶段 582 力输出冻结策略。
#include "force_feedback.h"

double map_fn1_to_force_582(short fn_1_raw)
{
	const double x = static_cast<double>(fn_1_raw);
	double y = 0.0;
	if (x >= 1150.0 && x <= 1200.0)
	{
		y = 0.0;
	}
	else if (x > 1200.0)
	{
		y = (x - 1200.0) * (2.0 / (3000.0 - 1200.0));
	}
	else
	{
		y = (x - 1150.0) * ((-2.0 - 0.0) / (-1000.0 - 1150.0));
	}
	return clamp_double(y, -2.0, 2.0);
}

double map_ft1_to_torque_582(short ft_1_raw)
{
	const double x = static_cast<double>(ft_1_raw);
	double y = 0.0;
	if (x >= -350.0 && x <= -300.0)
	{
		y = 0.0;
	}
	else if (x > -300.0)
	{
		y = (x + 300.0) * ((-2.0 - 0.0) / (500.0 - (-300.0)));
	}
	else
	{
		y = (x + 350.0) * ((2.0 - 0.0) / (-1150.0 - (-350.0)));
	}
	return clamp_double(y, -2.0, 2.0);
}

void map_force_587_placeholder(short fn_2_raw, short ft_2_raw, double& out_f, double& out_n)
{
	(void)fn_2_raw;
	(void)ft_2_raw;
	out_f = 0.0;
	out_n = 0.0;
}

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
	int loop_count)
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
			const double mapped_582_f = map_fn1_to_force_582(sample.fn_1_value);
			const double mapped_582_n = map_ft1_to_torque_582(sample.ft_1_value);
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
			map_force_587_placeholder(sample.fn_2_value, sample.ft_2_value, out_cmd.force_587_f, out_cmd.force_587_n);
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

	// 使用 setforce(F,N) 直接对应 SDK axis=0 通道语义。
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

