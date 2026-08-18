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

	void calculate_axis6_window_from_axis5(
		const AppContext& ctx,
		double& window_start_abs,
		double& window_end_abs)
	{
		const double axis5_abs = ctx.plc_act_pos[4] + ctx.plc_init_pos[4];
		calculate_axis6_window_from_axis5_abs(ctx, axis5_abs, window_start_abs, window_end_abs);
	}

	void calculate_axis6_window_from_axis5_abs(
		const AppContext& ctx,
		double axis5_abs,
		double& window_start_abs,
		double& window_end_abs)
	{
		const double axis5_from_left_mm = axis5_abs - ctx.plc_leftlimit[4];
		window_start_abs = ctx.plc_leftlimit[5] + axis5_from_left_mm +
			ctx.cfg->axis6_window_min_gap_from_axis5_mm;
		window_end_abs = window_start_abs + ctx.cfg->axis6_window_size_mm;
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

	bool rebuild_axis6_window_from_axis5(AppContext& ctx, bool log_result)
	{
		double window_start_abs = 0.0;
		double window_end_abs = 0.0;
		calculate_axis6_window_from_axis5(ctx, window_start_abs, window_end_abs);
		if ((window_end_abs - window_start_abs) < ctx.cfg->crawl_arrive_tol_mm)
		{
			if (log_result)
			{
				std::cout << "导丝模式切换已忽略：axis6 窗口宽度无效。" << std::endl;
			}
			return false;
		}

		ctx.axis6_crawl->start_abs = window_start_abs;
		ctx.axis6_crawl->end_abs = window_end_abs;
		lock_axis6_window_from_current(ctx);
		if (log_result)
		{
			std::cout
				<< "axis6 窗口已按 axis5 重建：["
				<< (window_start_abs - ctx.plc_leftlimit[5])
				<< ", "
				<< (window_end_abs - ctx.plc_leftlimit[5])
				<< "] mm（距各自左限位）。"
				<< std::endl;
		}
		return true;
	}

	bool sync_axis1(AppContext& ctx, int samples)
	{
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
		ctx.axis1_crawl->window_active = is_within_range(
			ctx.plc_act_pos[0] + ctx.plc_init_pos[0],
			ctx.axis1_crawl->min_abs(),
			ctx.axis1_crawl->max_abs(),
			ctx.cfg->crawl_arrive_tol_mm);

		*ctx.axis2_hold_rel = preserved_axis2_hold_rel;
		*ctx.axis1_reverse_switch_guard_active = false;
		*ctx.axis1_prev_abs_for_trigger = ctx.plc_act_pos[0] + ctx.plc_init_pos[0];
		*ctx.axis1_prev_abs_valid = true;

		*ctx.axis3_base_rel = ctx.plc_act_pos[2];
		*ctx.axis5_base_rel = ctx.plc_act_pos[4];
		*ctx.axis6_mirror_base_rel = ctx.plc_act_pos[5];

		return plc_io::write_refer(ctx);
	}

	bool rebase_axis1_after_return(AppContext& ctx)
	{
		if (ctx.pos == nullptr || ctx.plc_act_pos == nullptr || ctx.plc_init_pos == nullptr ||
			ctx.axis1_handle_filter == nullptr || !ctx.axis1_handle_filter->inited ||
			ctx.axis1_crawl == nullptr || ctx.axis2_hold_rel == nullptr ||
			ctx.axis7_hold_rel == nullptr)
		{
			return false;
		}

		// 回退和自动先行期间手柄持续正常采样，因此直接使用当前滤波值即可。
		// 不再额外 poll/Sleep，避免把交接变成阻塞式多点平均。
		plc_io::load_pos_from_actual(ctx);
		*ctx.axis2_hold_rel = ctx.plc_act_pos[1];
		*ctx.axis7_hold_rel = ctx.plc_act_pos[6];
		ctx.pos[1] = *ctx.axis2_hold_rel;
		ctx.pos[6] = *ctx.axis7_hold_rel;

		ctx.axis1_crawl->handle_ref = ctx.axis1_handle_filter->axis0_filtered;
		ctx.axis1_crawl->rot_ref = ctx.axis1_handle_filter->axis1_filtered;
		*ctx.axis1_prev_linear_filtered = ctx.axis1_handle_filter->axis0_filtered;
		*ctx.axis1_prev_rot_filtered = ctx.axis1_handle_filter->axis1_filtered;
		ctx.axis1_crawl->base_rel = ctx.plc_act_pos[0];
		ctx.axis1_crawl->rot_base_rel = *ctx.axis2_hold_rel;
		*ctx.axis1_follow_cmd_abs = ctx.plc_act_pos[0] + ctx.plc_init_pos[0];
		ctx.axis1_crawl->start_abs = axis1_window_left_abs(ctx);
		ctx.axis1_crawl->end_abs = axis1_window_right_abs(ctx);
		ctx.axis1_crawl->window_active = is_within_range(
			ctx.plc_act_pos[0] + ctx.plc_init_pos[0],
			ctx.axis1_crawl->min_abs(),
			ctx.axis1_crawl->max_abs(),
			ctx.cfg->crawl_arrive_tol_mm);

		*ctx.axis1_reverse_switch_guard_active = false;
		*ctx.axis1_prev_abs_for_trigger = ctx.plc_act_pos[0] + ctx.plc_init_pos[0];
		*ctx.axis1_prev_abs_valid = true;
		*ctx.axis3_base_rel = ctx.plc_act_pos[2];
		*ctx.axis5_base_rel = ctx.plc_act_pos[4];
		*ctx.axis6_mirror_base_rel = ctx.plc_act_pos[5];
		return true;
	}

	bool rebase_axis6_after_return(AppContext& ctx)
	{
		if (ctx.pos == nullptr || ctx.plc_act_pos == nullptr || ctx.plc_init_pos == nullptr ||
			ctx.cfg == nullptr || ctx.axis6_handle_filter == nullptr ||
			!ctx.axis6_handle_filter->inited || ctx.axis6_crawl == nullptr ||
			ctx.axis2_hold_rel == nullptr || ctx.axis7_hold_rel == nullptr ||
			ctx.independent_axis1_hold_rel == nullptr || ctx.independent_axis2_hold_rel == nullptr ||
			ctx.independent_axis3_hold_rel == nullptr || ctx.independent_axis5_hold_rel == nullptr ||
			ctx.axis6_prev_linear_filtered == nullptr || ctx.axis6_prev_rot_filtered == nullptr ||
			ctx.axis6_follow_cmd_abs == nullptr || ctx.axis6_reverse_switch_guard_active == nullptr ||
			ctx.axis6_prev_abs_for_trigger == nullptr || ctx.axis6_prev_abs_valid == nullptr ||
			ctx.axis6_window_locked == nullptr || ctx.axis6_locked_window_start_abs == nullptr ||
			ctx.axis6_locked_window_end_abs == nullptr || ctx.axis6_coop_ff_inited == nullptr ||
			ctx.axis6_coop_prev_axis1_cmd_abs == nullptr)
		{
			return false;
		}

		// 独立模式的窗口在入模时锁定，回退交接不得按axis5重算。
		// 缺少锁定窗口表示模式上下文已不完整，由上层转入安全保持。
		if (!*ctx.axis6_window_locked ||
			!std::isfinite(*ctx.axis6_locked_window_start_abs) ||
			!std::isfinite(*ctx.axis6_locked_window_end_abs) ||
			std::abs(*ctx.axis6_locked_window_end_abs - *ctx.axis6_locked_window_start_abs) <
			ctx.cfg->crawl_arrive_tol_mm)
		{
			return false;
		}

		// load_pos_from_actual只复制已由通信线程发布的内存快照，
		// 不会发起ADS读写，也不会轮询手柄或等待。
		plc_io::load_pos_from_actual(ctx);

		// 独立导丝模式中导管侧始终冻结。用同一快照刷新保持位，
		// 避免回退期间的跟随误差在交接首拍被重新下发。
		*ctx.independent_axis1_hold_rel = ctx.plc_act_pos[0];
		*ctx.independent_axis2_hold_rel = ctx.plc_act_pos[1];
		*ctx.independent_axis3_hold_rel = ctx.plc_act_pos[2];
		*ctx.independent_axis5_hold_rel = ctx.plc_act_pos[4];
		*ctx.axis2_hold_rel = ctx.plc_act_pos[1];
		*ctx.axis7_hold_rel = ctx.plc_act_pos[6];
		ctx.pos[0] = *ctx.independent_axis1_hold_rel;
		ctx.pos[1] = *ctx.independent_axis2_hold_rel;
		ctx.pos[2] = *ctx.independent_axis3_hold_rel;
		ctx.pos[4] = *ctx.independent_axis5_hold_rel;
		ctx.pos[6] = *ctx.axis7_hold_rel;

		ctx.axis6_crawl->handle_ref = ctx.axis6_handle_filter->axis0_filtered;
		ctx.axis6_crawl->rot_ref = ctx.axis6_handle_filter->axis1_filtered;
		*ctx.axis6_prev_linear_filtered = ctx.axis6_handle_filter->axis0_filtered;
		*ctx.axis6_prev_rot_filtered = ctx.axis6_handle_filter->axis1_filtered;
		ctx.axis6_crawl->base_rel = ctx.plc_act_pos[5];
		ctx.axis6_crawl->rot_base_rel = *ctx.axis7_hold_rel;
		*ctx.axis6_follow_cmd_abs = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		apply_locked_axis6_window(ctx);
		ctx.axis6_crawl->window_active = is_within_range(
			*ctx.axis6_follow_cmd_abs,
			ctx.axis6_crawl->min_abs(),
			ctx.axis6_crawl->max_abs(),
			ctx.cfg->crawl_arrive_tol_mm);
		ctx.axis6_crawl->enabled = true;

		*ctx.axis6_reverse_switch_guard_active = false;
		*ctx.axis6_prev_abs_for_trigger = *ctx.axis6_follow_cmd_abs;
		*ctx.axis6_prev_abs_valid = true;
		*ctx.axis6_coop_ff_inited = false;
		*ctx.axis6_coop_prev_axis1_cmd_abs = 0.0;
		return true;
	}

	bool rebase_cooperative_after_return(AppContext& ctx)
	{
		if (ctx.pos == nullptr || ctx.plc_act_pos == nullptr || ctx.plc_init_pos == nullptr ||
			ctx.plc_leftlimit == nullptr || ctx.cfg == nullptr ||
			ctx.axis1_handle_filter == nullptr || !ctx.axis1_handle_filter->inited ||
			ctx.axis6_handle_filter == nullptr || !ctx.axis6_handle_filter->inited ||
			ctx.axis1_crawl == nullptr || ctx.axis6_crawl == nullptr ||
			ctx.axis2_hold_rel == nullptr || ctx.axis7_hold_rel == nullptr ||
			ctx.axis1_prev_linear_filtered == nullptr || ctx.axis1_prev_rot_filtered == nullptr ||
			ctx.axis6_prev_linear_filtered == nullptr || ctx.axis6_prev_rot_filtered == nullptr ||
			ctx.axis1_follow_cmd_abs == nullptr || ctx.axis6_follow_cmd_abs == nullptr ||
			ctx.axis1_reverse_switch_guard_active == nullptr ||
			ctx.axis6_reverse_switch_guard_active == nullptr ||
			ctx.axis1_prev_abs_for_trigger == nullptr || ctx.axis1_prev_abs_valid == nullptr ||
			ctx.axis6_prev_abs_for_trigger == nullptr || ctx.axis6_prev_abs_valid == nullptr ||
			ctx.axis3_base_rel == nullptr || ctx.axis5_base_rel == nullptr ||
			ctx.axis6_mirror_base_rel == nullptr || ctx.axis6_window_locked == nullptr ||
			ctx.axis6_coop_ff_inited == nullptr || ctx.axis6_coop_prev_axis1_cmd_abs == nullptr)
		{
			return false;
		}

		const double axis1_abs = ctx.plc_act_pos[0] + ctx.plc_init_pos[0];
		const double axis5_abs = ctx.plc_act_pos[4] + ctx.plc_init_pos[4];
		const double axis6_abs = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		double axis6_window_start_abs = 0.0;
		double axis6_window_end_abs = 0.0;
		calculate_axis6_window_from_axis5_abs(
			ctx,
			axis5_abs,
			axis6_window_start_abs,
			axis6_window_end_abs);
		if (!std::isfinite(axis6_window_start_abs) || !std::isfinite(axis6_window_end_abs) ||
			std::abs(axis6_window_end_abs - axis6_window_start_abs) < ctx.cfg->crawl_arrive_tol_mm)
		{
			return false;
		}

		// 双轴基准全部来自调用者已验证的同一份快照；
		// 本函数不采样外设、不Sleep、不调用同步ADS接口。
		plc_io::load_pos_from_actual(ctx);
		*ctx.axis2_hold_rel = ctx.plc_act_pos[1];
		*ctx.axis7_hold_rel = ctx.plc_act_pos[6];
		ctx.pos[1] = *ctx.axis2_hold_rel;
		ctx.pos[6] = *ctx.axis7_hold_rel;

		ctx.axis1_crawl->handle_ref = ctx.axis1_handle_filter->axis0_filtered;
		ctx.axis1_crawl->rot_ref = ctx.axis1_handle_filter->axis1_filtered;
		*ctx.axis1_prev_linear_filtered = ctx.axis1_handle_filter->axis0_filtered;
		*ctx.axis1_prev_rot_filtered = ctx.axis1_handle_filter->axis1_filtered;
		ctx.axis1_crawl->base_rel = ctx.plc_act_pos[0];
		ctx.axis1_crawl->rot_base_rel = *ctx.axis2_hold_rel;
		*ctx.axis1_follow_cmd_abs = axis1_abs;
		ctx.axis1_crawl->start_abs = axis1_window_left_abs(ctx);
		ctx.axis1_crawl->end_abs = axis1_window_right_abs(ctx);
		ctx.axis1_crawl->window_active = is_within_range(
			axis1_abs,
			ctx.axis1_crawl->min_abs(),
			ctx.axis1_crawl->max_abs(),
			ctx.cfg->crawl_arrive_tol_mm);

		ctx.axis6_crawl->handle_ref = ctx.axis6_handle_filter->axis0_filtered;
		ctx.axis6_crawl->rot_ref = ctx.axis6_handle_filter->axis1_filtered;
		*ctx.axis6_prev_linear_filtered = ctx.axis6_handle_filter->axis0_filtered;
		*ctx.axis6_prev_rot_filtered = ctx.axis6_handle_filter->axis1_filtered;
		ctx.axis6_crawl->base_rel = ctx.plc_act_pos[5];
		ctx.axis6_crawl->rot_base_rel = *ctx.axis7_hold_rel;
		*ctx.axis6_follow_cmd_abs = axis6_abs;
		ctx.axis6_crawl->start_abs = axis6_window_start_abs;
		ctx.axis6_crawl->end_abs = axis6_window_end_abs;
		ctx.axis6_crawl->window_active = is_within_range(
			axis6_abs,
			ctx.axis6_crawl->min_abs(),
			ctx.axis6_crawl->max_abs(),
			ctx.cfg->crawl_arrive_tol_mm);
		ctx.axis6_crawl->enabled = true;
		*ctx.axis6_window_locked = false;

		// 重建导管镜像基准，确保交接首拍axis3/5不追赶回退前命令。
		*ctx.axis3_base_rel = ctx.plc_act_pos[2];
		*ctx.axis5_base_rel = ctx.plc_act_pos[4];
		*ctx.axis6_mirror_base_rel = ctx.plc_act_pos[5];
		*ctx.axis1_reverse_switch_guard_active = false;
		*ctx.axis6_reverse_switch_guard_active = false;
		*ctx.axis1_prev_abs_for_trigger = axis1_abs;
		*ctx.axis6_prev_abs_for_trigger = axis6_abs;
		*ctx.axis1_prev_abs_valid = true;
		*ctx.axis6_prev_abs_valid = true;

		// 协同相对窗口的差分基准也必须来自本快照，
		// 否则恢复Follow的首拍会把交接期间的axis5变化误当为新增量。
		*ctx.axis6_coop_ff_inited = true;
		*ctx.axis6_coop_prev_axis1_cmd_abs = axis5_abs;
		return true;
	}

	bool sync_axis6(
		AppContext& ctx,
		int samples,
		bool rebuild_window,
		bool log_window_rebuild)
	{
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
		if (rebuild_window || !ctx.axis6_crawl->enabled)
		{
			if (!rebuild_axis6_window_from_axis5(ctx, log_window_rebuild))
			{
				return false;
			}
		}
		else if (*ctx.axis6_window_locked)
		{
			apply_locked_axis6_window(ctx);
		}
		ctx.axis6_crawl->window_active = is_within_range(
			ctx.plc_act_pos[5] + ctx.plc_init_pos[5],
			ctx.axis6_crawl->min_abs(),
			ctx.axis6_crawl->max_abs(),
			ctx.cfg->crawl_arrive_tol_mm);
		ctx.axis6_crawl->enabled = true;
		*ctx.axis6_coop_ff_inited = false;
		*ctx.axis6_coop_prev_axis1_cmd_abs = 0.0;

		*ctx.axis7_hold_rel = preserved_axis7_hold_rel;
		*ctx.axis6_reverse_switch_guard_active = false;
		*ctx.axis6_prev_abs_for_trigger = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		*ctx.axis6_prev_abs_valid = true;

		return plc_io::write_refer(ctx);
	}

	bool sync_cooperative_guidewire(
		AppContext& ctx,
		int samples,
		bool log_window_rebuild)
	{
		// 协同模式需要同时重建两只手柄的差分基准。仅同步其中一轴会让
		// 另一只手柄在回退结束后继续使用旧基准，造成下一拍目标跳变。
		if (!sync_axis1(ctx, samples))
		{
			return false;
		}
		if (!sync_axis6(ctx, samples, true, log_window_rebuild))
		{
			return false;
		}
		// 协同递送的窗口每拍按 axis5 命令位置动态更新，不沿用独立导丝模式的锁定边界。
		*ctx.axis6_window_locked = false;
		return true;
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
		ctx.axis1_crawl->window_active = is_within_range(
			ctx.plc_act_pos[0] + ctx.plc_init_pos[0],
			ctx.axis1_crawl->min_abs(),
			ctx.axis1_crawl->max_abs(),
			ctx.cfg->crawl_arrive_tol_mm);
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
		ctx.axis6_crawl->window_active = false;
		ctx.axis6_crawl->enabled = false;
		*ctx.axis6_window_locked = false;
		*ctx.axis6_coop_ff_inited = false;
		*ctx.axis6_coop_prev_axis1_cmd_abs = 0.0;
		*ctx.axis6_follow_cmd_abs = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		*ctx.axis6_reverse_switch_guard_active = false;
		*ctx.axis6_prev_abs_for_trigger = *ctx.axis6_follow_cmd_abs;
		*ctx.axis6_prev_abs_valid = true;
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

