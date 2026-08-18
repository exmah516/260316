// 文件职责说明：
// 1) 实现导丝模式独立/协同入模、退出与门限检查逻辑。
// 2) 保持原切换时序与同步策略，不修改业务行为。
// 3) 模式入口按轴5当前位置重建 axis6 窗口，不承载主循环状态机。
#include "guidewire_mode.h"

#include "ads_communication.h"
#include "motion_sync.h"
#include "plc_io.h"

#include <cmath>
#include <iostream>

namespace
{
	bool prepare_axis6_guidewire_handoff(AppContext& ctx)
	{
		// 模式切换前先撤销残留 Req；已经 Busy 的 MC 回退不能由改写 refer 安全接管。
		if (!plc_io::clear_axis_return_request(ctx, AdsSymbol::axis6_return))
		{
			std::cout << "导丝模式切换失败：无法清除轴6计划回退请求。" << std::endl;
			return false;
		}

		if (ctx.ads_service != nullptr)
		{
			if (ctx.ads_service->stats().state != AdsConnectionState::Running)
			{
				std::cout << "导丝模式切换失败：ADS 通信服务未运行。" << std::endl;
				return false;
			}
			const AdsEventState events = ctx.ads_service->event_state();
			if (events.axis6_return_busy)
			{
				std::cout << "导丝模式切换已拒绝：轴6计划回退仍在执行，请等待 Busy 清除。" << std::endl;
				return false;
			}
			if (events.axis6_return_error)
			{
				std::cout << "导丝模式切换已拒绝：轴6计划回退仍有未清除错误，错误码: "
					<< events.axis6_return_error_id << std::endl;
				return false;
			}
			return true;
		}

		AxisReturnStatus status;
		if (!plc_io::read_axis_return_status(ctx, AdsSymbol::axis6_return, status))
		{
			std::cout << "导丝模式切换失败：无法读取轴6计划回退状态。" << std::endl;
			return false;
		}
		if (status.busy)
		{
			std::cout << "导丝模式切换已拒绝：轴6计划回退仍在执行，请等待 Busy 清除。" << std::endl;
			return false;
		}
		return true;
	}
}

namespace guidewire_mode_ctrl
{
	bool enter_independent_guidewire_mode(AppContext& ctx)
	{
		const double preserved_axis7_hold_rel = *ctx.axis7_hold_rel;
		if (!prepare_axis6_guidewire_handoff(ctx))
		{
			return false;
		}

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
		if (!motion_sync::rebuild_axis6_window_from_axis5(ctx, true))
		{
			return false;
		}
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
		if (!prepare_axis6_guidewire_handoff(ctx))
		{
			return false;
		}
		return motion_sync::sync_cooperative_guidewire(ctx, 20, true);
	}

	bool check_axis6_guidewire_entry_gate(AppContext& ctx, double& axis6_from_left_mm)
	{
		if (!plc_io::read_plc_state(ctx))
		{
			return false;
		}
		const double axis6_abs = ctx.plc_act_pos[5] + ctx.plc_init_pos[5];
		axis6_from_left_mm = std::abs(axis6_abs - ctx.plc_leftlimit[5]);
		// 返回值只表示 ADS 实际位置读取成功；入口距离门限由调用方统一判定，
		// 这样超过 667 mm 时可以输出准确的拒绝原因，而不会误报成读取失败。
		return true;
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
		return motion_sync::sync_axis1(ctx, 20);
	}
}

