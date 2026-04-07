// 文件职责说明：
// 1) 实现运动同步相关流程：窗口计算、窗口重建、axis1/axis6/all 重同步。
// 2) 复用原控制逻辑中的状态更新次序与基线刷新策略。
// 3) 不改变主循环状态机，仅提供可复用的同步函数。
#include "motion_sync.h"

#include "plc_io.h"

#include <cmath>
#include <iostream>

namespace motion_sync
{
	double from_left_to_abs(const AppContext& ctx, int axis_index, double from_left_mm)
	{
		// 左限位参考坐标 -> PLC 绝对坐标。
		return ctx.plc_leftlimit[axis_index] + from_left_mm;
	}

	double from_left_to_rel(const AppContext& ctx, int axis_index, double from_left_mm)
	{
		// 左限位参考坐标 -> 上位机相对坐标（refer 使用的坐标系）。
		return from_left_to_abs(ctx, axis_index, from_left_mm) - ctx.plc_init_pos[axis_index];
	}

	double axis1_window_left_abs(const AppContext& ctx)
	{
		return from_left_to_abs(ctx, 0, ctx.cfg->axis1_window_left_from_left_mm);
	}

	double axis1_window_right_abs(const AppContext& ctx)
	{
		return from_left_to_abs(ctx, 0, ctx.cfg->axis1_window_right_from_left_mm);
	}

	bool set_axis6_independent_window(AppContext& ctx, double window_right_abs, bool log_clamp)
	{
		const double shifted_window_right_abs = window_right_abs + ctx.cfg->axis6_window_shift_from_left_mm;
		const double requested_left_abs = shifted_window_right_abs - ctx.cfg->axis6_independent_window_size_mm;
		const double clamped_left_abs =
			(requested_left_abs < ctx.plc_leftlimit[5]) ? ctx.plc_leftlimit[5] : requested_left_abs;

		ctx.axis6_crawl->start_abs = clamped_left_abs;
		ctx.axis6_crawl->end_abs = shifted_window_right_abs;

		if ((ctx.axis6_crawl->end_abs - ctx.axis6_crawl->start_abs) < ctx.cfg->crawl_arrive_tol_mm)
		{
			if (log_clamp)
			{
				std::cout << "导丝模式切换已忽略：axis6 独立窗口距离左限位过近。" << std::endl;
			}
			return false;
		}

		if (log_clamp && clamped_left_abs > requested_left_abs)
		{
			std::cout << "axis6 独立窗口已按左限位进行夹紧修正。" << std::endl;
		}

		return true;
	}

	void lock_axis6_window_from_current(AppContext& ctx)
	{
		*ctx.axis6_window_locked = true;
		*ctx.axis6_locked_window_start_abs = ctx.axis6_crawl->start_abs;
		*ctx.axis6_locked_window_end_abs = ctx.axis6_crawl->end_abs;
	}

	void apply_locked_axis6_window(AppContext& ctx)
	{
		ctx.axis6_crawl->start_abs = *ctx.axis6_locked_window_start_abs;
		ctx.axis6_crawl->end_abs = *ctx.axis6_locked_window_end_abs;
	}

	bool rebuild_axis6_window_if_covered(AppContext& ctx, const char* reason, bool log_result)
	{
		(void)reason;
		(void)log_result;
		const double axis6_start_from_left_mm = ctx.axis6_crawl->start_abs - ctx.plc_leftlimit[5];
		const double axis6_end_from_left_mm = ctx.axis6_crawl->end_abs - ctx.plc_leftlimit[5];
		const double axis6_far_from_left_mm =
			(axis6_start_from_left_mm > axis6_end_from_left_mm)
			? axis6_start_from_left_mm
			: axis6_end_from_left_mm;
		const double axis5_from_left_mm = (ctx.plc_act_pos[4] + ctx.plc_init_pos[4]) - ctx.plc_leftlimit[4];
		const double delta_mm = axis6_far_from_left_mm - axis5_from_left_mm;

		if (delta_mm < ctx.cfg->axis6_window_cover_threshold_mm)
		{
			const double rebuilt_window_left_abs = ctx.plc_leftlimit[5] + axis5_from_left_mm;
			const double rebuilt_window_right_abs = rebuilt_window_left_abs + ctx.cfg->axis6_independent_window_size_mm;
			if (!set_axis6_independent_window(ctx, rebuilt_window_right_abs, true))
			{
				return false;
			}
			lock_axis6_window_from_current(ctx);
		}
		return true;
	}

	bool sync_axis1(AppContext& ctx, int samples, bool wait_rearm, int rearm_dir)
	{
		(void)wait_rearm;
		(void)rearm_dir;
		const double preserved_axis2_hold_rel = *ctx.axis2_hold_rel;
		plc_io::clear_axis1_group_return_requests(ctx);

		if (!plc_io::read_plc_state(ctx))
		{
			return false;
		}

		plc_io::load_pos_from_actual(ctx);
		ctx.pos[1] = preserved_axis2_hold_rel;
		ctx.pos[6] = *ctx.axis7_hold_rel;

		get_average_handle_pose(*ctx.axis1_input_handle, samples, ctx.axis1_crawl->handle_ref, ctx.axis1_crawl->rot_ref);
		ctx.axis1_handle_filter->reset(ctx.axis1_crawl->handle_ref, ctx.axis1_crawl->rot_ref);
		*ctx.axis1_prev_linear_filtered = ctx.axis1_handle_filter->axis0_filtered;
		*ctx.axis1_prev_rot_filtered = ctx.axis1_handle_filter->axis1_filtered;
		ctx.axis1_crawl->base_rel = ctx.plc_act_pos[0];
		ctx.axis1_crawl->rot_base_rel = preserved_axis2_hold_rel;
		*ctx.axis1_follow_cmd_abs = ctx.plc_act_pos[0] + ctx.plc_init_pos[0];
		ctx.axis1_crawl->start_abs = axis1_window_left_abs(ctx);
		ctx.axis1_crawl->end_abs = axis1_window_right_abs(ctx);
		ctx.axis1_crawl->target_abs = ctx.axis1_crawl->end_abs;
		ctx.axis1_crawl->plc_move_requested = false;
		ctx.axis1_crawl->window_active = is_within_range(
			ctx.plc_act_pos[0] + ctx.plc_init_pos[0],
			ctx.axis1_crawl->min_abs(),
			ctx.axis1_crawl->max_abs(),
			ctx.cfg->crawl_arrive_tol_mm);
		ctx.axis1_crawl->phase = CrawlState::Phase::Follow;
		ctx.axis1_crawl->phase_t0 = GetTickCount();
		ctx.axis1_crawl->cyl_seq_stage = 0;
		ctx.axis1_crawl->cyl_seq_t0 = ctx.axis1_crawl->phase_t0;
		ctx.axis1_crawl->wait_rearm = false;
		ctx.axis1_crawl->rearm_dir = 0;

		*ctx.axis2_hold_rel = preserved_axis2_hold_rel;
		*ctx.axis1_reverse_switch_guard_active = false;
		*ctx.axis1_prev_abs_for_trigger = ctx.plc_act_pos[0] + ctx.plc_init_pos[0];
		*ctx.axis1_prev_abs_valid = true;

		*ctx.axis3_base_rel = ctx.plc_act_pos[2];
		*ctx.axis5_base_rel = ctx.plc_act_pos[4];
		*ctx.axis6_mirror_base_rel = ctx.plc_act_pos[5];
		*ctx.axis6_coupled_active = false;
		*ctx.axis6_coupled_requested = false;
		*ctx.axis6_coupled_done = false;
		*ctx.axis6_coupled_error = false;
		*ctx.axis6_coupled_error_id = 0;

		return plc_io::write_refer(ctx);
	}

	bool sync_axis6(
		AppContext& ctx,
		int samples,
		bool capture_window,
		bool wait_rearm,
		int rearm_dir,
		bool check_window_cover,
		bool log_window_cover)
	{
		(void)wait_rearm;
		(void)rearm_dir;
		const double preserved_axis7_hold_rel = *ctx.axis7_hold_rel;
		plc_io::clear_axis_return_request(ctx, AdsSymbol::axis6_return);

		if (!plc_io::read_plc_state(ctx))
		{
			return false;
		}

		plc_io::load_pos_from_actual(ctx);
		ctx.pos[1] = *ctx.axis2_hold_rel;
		ctx.pos[6] = preserved_axis7_hold_rel;

		get_average_handle_pose(*ctx.axis6_input_handle, samples, ctx.axis6_crawl->handle_ref, ctx.axis6_crawl->rot_ref);
		ctx.axis6_handle_filter->reset(ctx.axis6_crawl->handle_ref, ctx.axis6_crawl->rot_ref);
		*ctx.axis6_prev_linear_filtered = ctx.axis6_handle_filter->axis0_filtered;
		*ctx.axis6_prev_rot_filtered = ctx.axis6_handle_filter->axis1_filtered;
		ctx.axis6_crawl->base_rel = ctx.plc_act_pos[5];
		ctx.axis6_crawl->rot_base_rel = preserved_axis7_hold_rel;
		*ctx.axis6_follow_cmd_abs = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		if (capture_window || !ctx.axis6_crawl->enabled)
		{
			if (!*ctx.axis6_window_locked)
			{
				if (!set_axis6_independent_window(ctx, ctx.plc_act_pos[5] + ctx.plc_init_pos[5], capture_window))
				{
					return false;
				}
				lock_axis6_window_from_current(ctx);
			}
			else
			{
				apply_locked_axis6_window(ctx);
			}
		}
		else if (*ctx.axis6_window_locked)
		{
			apply_locked_axis6_window(ctx);
		}
		if (check_window_cover)
		{
			if (!rebuild_axis6_window_if_covered(ctx, "sync_axis6", log_window_cover))
			{
				return false;
			}
		}
		ctx.axis6_crawl->target_abs = ctx.axis6_crawl->end_abs;
		ctx.axis6_crawl->plc_move_requested = false;
		ctx.axis6_crawl->window_active = is_within_range(
			ctx.plc_act_pos[5] + ctx.plc_init_pos[5],
			ctx.axis6_crawl->min_abs(),
			ctx.axis6_crawl->max_abs(),
			ctx.cfg->crawl_arrive_tol_mm);
		ctx.axis6_crawl->phase = CrawlState::Phase::Follow;
		ctx.axis6_crawl->phase_t0 = GetTickCount();
		ctx.axis6_crawl->cyl_seq_stage = 0;
		ctx.axis6_crawl->cyl_seq_t0 = ctx.axis6_crawl->phase_t0;
		ctx.axis6_crawl->wait_rearm = false;
		ctx.axis6_crawl->rearm_dir = 0;
		ctx.axis6_crawl->enabled = true;
		*ctx.axis6_coop_ff_inited = false;
		*ctx.axis6_coop_prev_axis1_cmd_abs = 0.0;

		*ctx.axis7_hold_rel = preserved_axis7_hold_rel;
		*ctx.axis6_reverse_switch_guard_active = false;
		*ctx.axis6_prev_abs_for_trigger = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		*ctx.axis6_prev_abs_valid = true;

		return plc_io::write_refer(ctx);
	}

	bool sync_all(AppContext& ctx, int samples)
	{
		double preserved_axis2_hold_rel = *ctx.axis2_hold_rel;
		double preserved_axis7_hold_rel = *ctx.axis7_hold_rel;
		plc_io::clear_axis1_group_return_requests(ctx);
		plc_io::clear_axis_return_request(ctx, AdsSymbol::axis6_return);
		plc_io::write_axis4_manual_requests(ctx, false, false);

		if (!plc_io::read_plc_state(ctx))
		{
			return false;
		}
		// 使用“当前轴实际位置”重置旋转保持基准，避免复用陈旧 hold 值导致意外回零。
		preserved_axis2_hold_rel = ctx.plc_act_pos[1];
		preserved_axis7_hold_rel = ctx.plc_act_pos[6];

		plc_io::load_pos_from_actual(ctx);
		ctx.pos[1] = preserved_axis2_hold_rel;
		ctx.pos[6] = preserved_axis7_hold_rel;
		if (!plc_io::write_refer(ctx))
		{
			return false;
		}

		get_average_dual_pos(
			*ctx.axis1_input_handle,
			*ctx.axis6_input_handle,
			samples,
			ctx.axis1_crawl->handle_ref,
			ctx.axis1_crawl->rot_ref,
			ctx.axis6_crawl->handle_ref,
			ctx.axis6_crawl->rot_ref);
		ctx.axis1_handle_filter->reset(ctx.axis1_crawl->handle_ref, ctx.axis1_crawl->rot_ref);
		ctx.axis6_handle_filter->reset(ctx.axis6_crawl->handle_ref, ctx.axis6_crawl->rot_ref);
		*ctx.axis1_prev_linear_filtered = ctx.axis1_handle_filter->axis0_filtered;
		*ctx.axis6_prev_linear_filtered = ctx.axis6_handle_filter->axis0_filtered;
		*ctx.axis1_prev_rot_filtered = ctx.axis1_handle_filter->axis1_filtered;
		*ctx.axis6_prev_rot_filtered = ctx.axis6_handle_filter->axis1_filtered;

		if (!plc_io::read_plc_state(ctx))
		{
			return false;
		}

		plc_io::load_pos_from_actual(ctx);
		ctx.pos[1] = preserved_axis2_hold_rel;
		ctx.pos[6] = preserved_axis7_hold_rel;
		if (!plc_io::write_refer(ctx))
		{
			return false;
		}

		ctx.axis1_crawl->base_rel = ctx.plc_act_pos[0];
		ctx.axis1_crawl->rot_base_rel = preserved_axis2_hold_rel;
		ctx.axis1_crawl->start_abs = axis1_window_left_abs(ctx);
		ctx.axis1_crawl->end_abs = axis1_window_right_abs(ctx);
		ctx.axis1_crawl->target_abs = ctx.axis1_crawl->end_abs;
		ctx.axis1_crawl->plc_move_requested = false;
		ctx.axis1_crawl->window_active = is_within_range(
			ctx.plc_act_pos[0] + ctx.plc_init_pos[0],
			ctx.axis1_crawl->min_abs(),
			ctx.axis1_crawl->max_abs(),
			ctx.cfg->crawl_arrive_tol_mm);
		ctx.axis1_crawl->phase = CrawlState::Phase::Follow;
		ctx.axis1_crawl->phase_t0 = GetTickCount();
		ctx.axis1_crawl->cyl_seq_stage = 0;
		ctx.axis1_crawl->cyl_seq_t0 = ctx.axis1_crawl->phase_t0;
		ctx.axis1_crawl->wait_rearm = false;
		ctx.axis1_crawl->rearm_dir = 0;
		ctx.axis1_crawl->enabled = true;
		*ctx.axis1_follow_cmd_abs = ctx.plc_act_pos[0] + ctx.plc_init_pos[0];

		*ctx.axis2_hold_rel = preserved_axis2_hold_rel;
		*ctx.axis1_reverse_switch_guard_active = false;
		*ctx.axis1_prev_abs_for_trigger = *ctx.axis1_follow_cmd_abs;
		*ctx.axis1_prev_abs_valid = true;

		*ctx.axis3_base_rel = ctx.plc_act_pos[2];
		*ctx.axis5_base_rel = ctx.plc_act_pos[4];
		*ctx.axis6_mirror_base_rel = ctx.plc_act_pos[5];

		ctx.axis6_crawl->base_rel = ctx.plc_act_pos[5];
		ctx.axis6_crawl->rot_base_rel = preserved_axis7_hold_rel;
		ctx.axis6_crawl->start_abs = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		ctx.axis6_crawl->end_abs = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		ctx.axis6_crawl->target_abs = ctx.axis6_crawl->end_abs;
		ctx.axis6_crawl->plc_move_requested = false;
		ctx.axis6_crawl->window_active = false;
		ctx.axis6_crawl->phase = CrawlState::Phase::Follow;
		ctx.axis6_crawl->phase_t0 = GetTickCount();
		ctx.axis6_crawl->cyl_seq_stage = 0;
		ctx.axis6_crawl->cyl_seq_t0 = ctx.axis6_crawl->phase_t0;
		ctx.axis6_crawl->wait_rearm = false;
		ctx.axis6_crawl->rearm_dir = 0;
		ctx.axis6_crawl->enabled = false;
		*ctx.axis6_window_locked = false;
		*ctx.axis6_coop_ff_inited = false;
		*ctx.axis6_coop_prev_axis1_cmd_abs = 0.0;
		*ctx.axis6_follow_cmd_abs = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		*ctx.axis6_reverse_switch_guard_active = false;
		*ctx.axis6_prev_abs_for_trigger = *ctx.axis6_follow_cmd_abs;
		*ctx.axis6_prev_abs_valid = true;
		*ctx.axis6_coupled_active = false;
		*ctx.axis6_coupled_requested = false;
		*ctx.axis6_coupled_done = false;
		*ctx.axis6_coupled_error = false;
		*ctx.axis6_coupled_error_id = 0;

		*ctx.axis7_hold_rel = preserved_axis7_hold_rel;

		return true;
	}

	void capture_axis1_follow_baseline(AppContext& ctx)
	{
		ctx.axis1_crawl->handle_ref = ctx.axis1_handle_filter->axis0_filtered;
		ctx.axis1_crawl->rot_ref = ctx.axis1_handle_filter->axis1_filtered;
		ctx.axis1_crawl->base_rel = ctx.plc_act_pos[0];
		ctx.axis1_crawl->rot_base_rel = *ctx.axis2_hold_rel;
		*ctx.axis1_follow_cmd_abs = ctx.plc_act_pos[0] + ctx.plc_init_pos[0];
		ctx.axis1_crawl->plc_move_requested = false;
		*ctx.axis3_base_rel = ctx.plc_act_pos[2];
		*ctx.axis5_base_rel = ctx.plc_act_pos[4];
		if (!ctx.axis6_crawl->enabled)
		{
			*ctx.axis6_mirror_base_rel = ctx.plc_act_pos[5];
		}
	}

	void apply_axis1_mirror_from_abs(AppContext& ctx, double axis1_abs_cmd, bool include_axis6)
	{
		const double axis1_delta_rel = axis1_abs_cmd - ctx.plc_init_pos[0] - ctx.axis1_crawl->base_rel;
		ctx.pos[2] = *ctx.axis3_base_rel + axis1_delta_rel;
		ctx.pos[4] = *ctx.axis5_base_rel + axis1_delta_rel;
		if (include_axis6)
		{
			ctx.pos[5] = *ctx.axis6_mirror_base_rel + axis1_delta_rel;
		}
	}
}

