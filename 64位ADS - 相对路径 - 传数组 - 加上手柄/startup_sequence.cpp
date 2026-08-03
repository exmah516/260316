// 文件职责说明：
// 1) 实现标准装卸启动与中断恢复启动的入口判定、首帧目标和速度上限恢复。
// 2) 通过 PLC 一次性资格位和实际装卸姿态共同选择启动路径。
// 3) 不承载主循环调度，仅提供启动流程相关函数。
#include "startup_sequence.h"

#include "motion_sync.h"
#include "plc_io.h"

#include <cmath>
#include <iostream>

namespace
{
	bool startup_loading_pose_matches(AppContext& ctx,
		double& axis1_from_left_mm,
		double& axis3_from_left_mm,
		double& axis5_from_left_mm,
		double& axis6_from_left_mm)
	{
		axis1_from_left_mm = (ctx.plc_act_pos[0] + ctx.plc_init_pos[0]) - ctx.plc_leftlimit[0];
		axis3_from_left_mm = (ctx.plc_act_pos[2] + ctx.plc_init_pos[2]) - ctx.plc_leftlimit[2];
		axis5_from_left_mm = (ctx.plc_act_pos[4] + ctx.plc_init_pos[4]) - ctx.plc_leftlimit[4];
		axis6_from_left_mm = (ctx.plc_act_pos[5] + ctx.plc_init_pos[5]) - ctx.plc_leftlimit[5];

		const double tolerance_mm = ctx.cfg->startup_loading_pose_tolerance_mm;
		return std::isfinite(axis1_from_left_mm) &&
			std::isfinite(axis3_from_left_mm) &&
			std::isfinite(axis5_from_left_mm) &&
			std::isfinite(axis6_from_left_mm) &&
			std::abs(axis1_from_left_mm - ctx.cfg->startup_loading_axis1_from_left_mm) <= tolerance_mm &&
			std::abs(axis3_from_left_mm - ctx.cfg->startup_loading_axis3_from_left_mm) <= tolerance_mm &&
			std::abs(axis5_from_left_mm - ctx.cfg->startup_loading_axis5_from_left_mm) <= tolerance_mm &&
			std::abs(axis6_from_left_mm - ctx.cfg->startup_loading_axis6_from_left_mm) <= tolerance_mm;
	}

	bool write_recovery_grip_commands(AppContext& ctx)
	{
		// 先闭合承载侧，再打开相对侧，避免切换瞬间出现两侧同时松开的状态。
		// plc_io 内部按 2、4、1、3 的顺序组成同一次 Sum Write。
		const unsigned short commands[4] = {
			ctx.cyl->cyl1_open,
			ctx.cyl->cyl2_clamp,
			ctx.cyl->cyl3_open,
			ctx.cyl->cyl4_clamp
		};
		return plc_io::write_cylinder_values(ctx, commands);
	}
}

namespace startup_sequence
{
	bool consume_startup_loading_ready(AppContext& ctx)
	{
		if (!ctx.startup->loading_ready_symbol_available)
		{
			return true;
		}

		if (!plc_io::write_startup_loading_ready(ctx, false))
		{
			std::cout << "启动准备已拒绝：无法消费 PLC 装卸位就绪标志。" << std::endl;
			return false;
		}
		ctx.startup->loading_ready_plc = false;
		return true;
	}

	bool start_startup_sequence(AppContext& ctx)
	{
		const bool rotation_targets_valid =
			std::isfinite(ctx.startup->final_axis2_deg) &&
			std::isfinite(ctx.startup->final_axis7_deg) &&
			ctx.startup->final_axis2_deg >= -360.0 &&
			ctx.startup->final_axis2_deg <= 360.0 &&
			ctx.startup->final_axis7_deg >= -360.0 &&
			ctx.startup->final_axis7_deg <= 360.0;
		if (!rotation_targets_valid)
		{
			std::cout << "启动准备已拒绝：axis2/axis7 目标必须是 [-360, 360] deg 内的有限数值。" << std::endl;
			return false;
		}

		if (!plc_io::read_plc_state(ctx))
		{
			return false;
		}

		plc_io::load_pos_from_actual(ctx);
		const double axis2_actual_rel = ctx.plc_act_pos[1];
		const double axis7_actual_rel = ctx.plc_act_pos[6];
		ctx.startup->axis1_hold_rel = ctx.plc_act_pos[0];
		ctx.startup->axis2_hold_rel = axis2_actual_rel;
		ctx.startup->axis3_hold_rel = ctx.plc_act_pos[2];
		ctx.startup->axis5_hold_rel = ctx.plc_act_pos[4];
		ctx.startup->axis6_hold_rel = ctx.plc_act_pos[5];
		ctx.startup->axis7_hold_rel = axis7_actual_rel;

		double axis1_from_left_mm = 0.0;
		double axis3_from_left_mm = 0.0;
		double axis5_from_left_mm = 0.0;
		double axis6_from_left_mm = 0.0;
		ctx.startup->loading_pose_match = startup_loading_pose_matches(
			ctx,
			axis1_from_left_mm,
			axis3_from_left_mm,
			axis5_from_left_mm,
			axis6_from_left_mm);
		if (ctx.startup->loading_ready_symbol_available)
		{
			if (!plc_io::read_startup_loading_ready(ctx, ctx.startup->loading_ready_plc))
			{
				std::cout << "启动准备已拒绝：读取 PLC 装卸位就绪标志失败。" << std::endl;
				return false;
			}
		}
		else
		{
			// 旧 PLC 没有新符号时仅按位置兼容判定，不阻断启动。
			ctx.startup->loading_ready_plc = ctx.startup->loading_pose_match;
		}
		const bool loading_ready_at_entry = ctx.startup->loading_ready_plc;
		ctx.startup->recovery_mode =
			!ctx.startup->loading_pose_match || !loading_ready_at_entry;

		// 点击开始时先刷新共享保持位，防止 Activate Configuration 后复用上一周期缓存。
		*ctx.axis2_hold_rel = axis2_actual_rel;
		*ctx.axis7_hold_rel = axis7_actual_rel;
		// 标准启动首拍只启动旋转轴；恢复启动首拍同时启动 axis1/2/6/7。
		ctx.pos[1] = ctx.startup->final_axis2_deg;
		ctx.pos[6] = ctx.startup->final_axis7_deg;
		if (ctx.startup->recovery_mode)
		{
			ctx.pos[0] = motion_sync::from_left_to_rel(ctx, 0, ctx.startup->final_axis1_from_left_mm);
			ctx.pos[5] = motion_sync::from_left_to_rel(ctx, 5, ctx.startup->final_axis6_from_left_mm);
		}

		if (!ctx.startup->v_limit_scaled)
		{
			if (!plc_io::read_v_limit(ctx))
			{
				return false;
			}

			copy_positions(ctx.plc_v_limit, ctx.startup->v_limit_backup, 7);
			double scaled_v_limit[7] = { 0 };
			copy_positions(ctx.plc_v_limit, scaled_v_limit, 7);
			// 轴1/3/5/6统一使用四轴原始上限中的最小值，确保启动准备时实际限速一致。
			const double axis13_common_v_limit =
				(ctx.plc_v_limit[0] < ctx.plc_v_limit[2]) ? ctx.plc_v_limit[0] : ctx.plc_v_limit[2];
			const double axis56_common_v_limit =
				(ctx.plc_v_limit[4] < ctx.plc_v_limit[5]) ? ctx.plc_v_limit[4] : ctx.plc_v_limit[5];
			const double axis1356_common_v_limit =
				(axis13_common_v_limit < axis56_common_v_limit) ? axis13_common_v_limit : axis56_common_v_limit;
			scaled_v_limit[0] = axis1356_common_v_limit * ctx.cfg->startup_motion_speed_scale;
			scaled_v_limit[1] *= ctx.cfg->startup_motion_speed_scale;
			scaled_v_limit[2] = axis1356_common_v_limit * ctx.cfg->startup_motion_speed_scale;
			scaled_v_limit[4] = axis1356_common_v_limit * ctx.cfg->startup_motion_speed_scale;
			scaled_v_limit[5] = axis1356_common_v_limit * ctx.cfg->startup_motion_speed_scale;
			scaled_v_limit[6] *= ctx.cfg->startup_motion_speed_scale;
			if (!plc_io::write_v_limit(ctx, scaled_v_limit))
			{
				return false;
			}
			ctx.startup->v_limit_scaled = true;
		}

		if (ctx.startup->recovery_mode && !write_recovery_grip_commands(ctx))
		{
			std::cout << "恢复启动已拒绝：无法建立电缸 1开2闭、3开4闭 的稳定夹持组合。" << std::endl;
			if (!restore_startup_v_limit(ctx))
			{
				std::cout << "警告：恢复启动入口失败后，启动期速度上限恢复失败。" << std::endl;
			}
			return false;
		}

		if (!consume_startup_loading_ready(ctx))
		{
			if (!restore_startup_v_limit(ctx))
			{
				std::cout << "警告：装卸位标志消费失败后，启动期速度上限恢复失败。" << std::endl;
			}
			return false;
		}

		// 首帧 refer 之前先通知 PLC 旁路旧轨迹缓存，保证恢复启动不会追随上次进程的参考历史。
		*ctx.startup_smoothing_bypass = true;
		if (!plc_io::write_startup_smoothing_bypass(ctx, true))
		{
			*ctx.startup_smoothing_bypass = false;
			if (!restore_startup_v_limit(ctx))
			{
				std::cout << "警告：启动旁路写入失败后，启动期速度上限恢复失败。" << std::endl;
			}
			return false;
		}

		if (!plc_io::write_refer(ctx))
		{
			*ctx.startup_smoothing_bypass = false;
			(void)plc_io::write_startup_smoothing_bypass(ctx, false);
			if (ctx.startup->v_limit_scaled && !restore_startup_v_limit(ctx))
			{
				std::cout << "警告：启动首帧 refer 写入失败，且启动期速度上限恢复失败；保留恢复标志以便后续重试。" << std::endl;
			}
			return false;
		}

		*ctx.guidewire_mode = GuidewireMode::None;
		ctx.axis6_crawl->enabled = false;
		*ctx.axis6_window_locked = false;
		*ctx.axis6_coop_ff_inited = false;
		*ctx.axis6_coop_prev_axis1_cmd_abs = 0.0;
		ctx.startup->phase = ctx.startup->recovery_mode
			? StartupPhase::RecoveryMoveAxis1267
			: StartupPhase::ReleaseClamps;
		ctx.startup->phase_t0 = GetTickCount();
		ctx.startup->completed = false;
		ctx.startup->prompted = false;
		std::cout << "启动准备旋转目标：axis2 " << axis2_actual_rel << " -> "
			<< ctx.startup->final_axis2_deg << " deg，axis7 " << axis7_actual_rel << " -> "
			<< ctx.startup->final_axis7_deg << " deg。" << std::endl;
		std::cout << "启动准备入口位置（距左限位）：axis1=" << axis1_from_left_mm
			<< "，axis3=" << axis3_from_left_mm
			<< "，axis5=" << axis5_from_left_mm
			<< "，axis6=" << axis6_from_left_mm
			<< " mm；PLC装卸标志="
			<< (ctx.startup->loading_ready_symbol_available
				? (loading_ready_at_entry ? "TRUE" : "FALSE")
				: "旧版无符号")
			<< "；选择" << (ctx.startup->recovery_mode ? "中断恢复启动" : "标准装卸启动")
			<< "。" << std::endl;

		return true;
	}

	bool restore_startup_v_limit(AppContext& ctx)
	{
		if (!ctx.startup->v_limit_scaled)
		{
			return true;
		}

		if (!plc_io::write_v_limit(ctx, ctx.startup->v_limit_backup))
		{
			return false;
		}

		ctx.startup->v_limit_scaled = false;
		return true;
	}

	void prompt_startup_mode(AppContext& ctx)
	{
		if (!ctx.startup->prompted)
		{
			std::cout << "请安装器械。" << std::endl;
			std::cout << "进入启动姿态请按键盘 S 键。" << std::endl;
			std::cout << "直接继续控制请按键盘 C 键。" << std::endl;
			ctx.startup->prompted = true;
		}
	}
}
