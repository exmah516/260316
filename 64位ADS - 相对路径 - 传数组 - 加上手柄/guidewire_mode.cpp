// 文件职责说明：
// 1) 实现导丝模式独立/协同入模、退出与门限检查逻辑。
// 2) 保持原切换时序与同步策略，不修改业务行为。
// 3) 仅负责模式切换流程，不承载主循环状态机。
#include "guidewire_mode.h"

#include "motion_sync.h"
#include "plc_io.h"

#include <cmath>

namespace guidewire_mode_ctrl
{
	bool enter_independent_guidewire_mode(AppContext& ctx)
	{
		const double preserved_axis7_hold_rel = *ctx.axis7_hold_rel;

		if (!plc_io::read_plc_state(ctx))
		{
			return false;
		}

		plc_io::load_pos_from_actual(ctx);
		*ctx.independent_axis1_hold_rel = ctx.plc_act_pos[0];
		*ctx.independent_axis2_hold_rel = *ctx.axis2_hold_rel;
		*ctx.independent_axis3_hold_rel = ctx.plc_act_pos[2];
		*ctx.independent_axis5_hold_rel = ctx.plc_act_pos[4];

		*ctx.axis7_hold_rel = preserved_axis7_hold_rel;

		get_average_handle_pose(*ctx.axis6_input_handle, 20, ctx.axis6_crawl->handle_ref, ctx.axis6_crawl->rot_ref);
		ctx.axis6_handle_filter->reset(ctx.axis6_crawl->handle_ref, ctx.axis6_crawl->rot_ref);
		*ctx.axis6_prev_linear_filtered = ctx.axis6_handle_filter->axis0_filtered;
		*ctx.axis6_prev_rot_filtered = ctx.axis6_handle_filter->axis1_filtered;
		ctx.axis6_crawl->base_rel = ctx.plc_act_pos[5];
		ctx.axis6_crawl->rot_base_rel = *ctx.axis7_hold_rel;
		*ctx.axis6_follow_cmd_abs = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		if (!*ctx.axis6_window_locked)
		{
			if (!motion_sync::set_axis6_independent_window(ctx, ctx.plc_act_pos[5] + ctx.plc_init_pos[5], true))
			{
				return false;
			}
			motion_sync::lock_axis6_window_from_current(ctx);
		}
		else
		{
			motion_sync::apply_locked_axis6_window(ctx);
		}
		if (!motion_sync::rebuild_axis6_window_if_covered(ctx, "enter_independent", true))
		{
			return false;
		}
		ctx.axis6_crawl->target_abs = ctx.axis6_crawl->end_abs;
		ctx.axis6_crawl->plc_move_requested = false;
		ctx.axis6_crawl->phase = CrawlState::Phase::Follow;
		ctx.axis6_crawl->phase_t0 = GetTickCount();
		ctx.axis6_crawl->cyl_seq_stage = 0;
		ctx.axis6_crawl->cyl_seq_t0 = ctx.axis6_crawl->phase_t0;
		ctx.axis6_crawl->wait_rearm = false;
		ctx.axis6_crawl->rearm_dir = 0;
		ctx.axis6_crawl->window_active = is_within_range(
			ctx.plc_act_pos[5] + ctx.plc_init_pos[5],
			ctx.axis6_crawl->min_abs(),
			ctx.axis6_crawl->max_abs(),
			ctx.cfg->crawl_arrive_tol_mm);
		ctx.axis6_crawl->enabled = true;
		*ctx.axis6_coop_ff_inited = false;
		*ctx.axis6_coop_prev_axis1_cmd_abs = 0.0;
		*ctx.axis6_reverse_switch_guard_active = false;
		*ctx.axis6_prev_abs_for_trigger = *ctx.axis6_follow_cmd_abs;
		*ctx.axis6_prev_abs_valid = true;

		ctx.pos[0] = ctx.plc_act_pos[0];
		ctx.pos[1] = *ctx.axis2_hold_rel;
		ctx.pos[2] = ctx.plc_act_pos[2];
		ctx.pos[4] = ctx.plc_act_pos[4];
		ctx.pos[5] = ctx.plc_act_pos[5];
		ctx.pos[6] = *ctx.axis7_hold_rel;
		return plc_io::write_refer(ctx);
	}

	bool enter_cooperative_guidewire_mode(AppContext& ctx)
	{
		return motion_sync::sync_axis6(ctx, 20, true, false, 0, true, true);
	}

	bool check_axis6_guidewire_entry_gate(AppContext& ctx, double& axis6_from_left_mm)
	{
		if (!plc_io::read_plc_state(ctx))
		{
			return false;
		}
		const double axis6_abs = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		axis6_from_left_mm = std::abs(axis6_abs - ctx.plc_leftlimit[5]);
		return axis6_from_left_mm < ctx.cfg->guidewire_entry_axis6_from_left_max_mm;
	}

	bool exit_guidewire_mode_to_normal(AppContext& ctx)
	{
		*ctx.guidewire_mode = GuidewireMode::None;
		*ctx.axis6_window_locked = false;
		*ctx.axis6_coop_ff_inited = false;
		*ctx.axis6_coop_prev_axis1_cmd_abs = 0.0;
		// 退出导丝时用当前实际值刷新旋转保持位，避免沿用陈旧 hold 导致 axis2 偶发回零。
		if (!plc_io::read_plc_state(ctx))
		{
			return false;
		}
		*ctx.axis2_hold_rel = ctx.plc_act_pos[1];
		*ctx.axis7_hold_rel = ctx.plc_act_pos[6];
		*ctx.axis1_reverse_switch_guard_active = false;
		*ctx.axis6_reverse_switch_guard_active = false;
		return motion_sync::sync_axis1(ctx, 20, false, 0);
	}
}

