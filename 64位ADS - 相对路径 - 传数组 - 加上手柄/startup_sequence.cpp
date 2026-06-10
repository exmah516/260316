// 文件职责说明：
// 1) 实现启动准备流程入口与速度上限恢复逻辑。
// 2) 保持与原 main.cpp 一致的参数缩放与状态切换语义。
// 3) 不承载主循环调度，仅提供启动流程相关函数。
#include "startup_sequence.h"

#include "motion_sync.h"
#include "plc_io.h"

#include <iostream>

namespace startup_sequence
{
	bool start_startup_sequence(AppContext& ctx)
	{
		if (!plc_io::read_plc_state(ctx))
		{
			return false;
		}

		plc_io::load_pos_from_actual(ctx);
		ctx.startup->axis1_hold_rel = ctx.plc_act_pos[0];
		ctx.startup->axis2_hold_rel = *ctx.axis2_hold_rel;
		ctx.startup->axis3_hold_rel = ctx.plc_act_pos[2];
		ctx.startup->axis5_hold_rel = ctx.plc_act_pos[4];
		ctx.startup->axis6_hold_rel = ctx.plc_act_pos[5];
		ctx.startup->axis7_hold_rel = *ctx.axis7_hold_rel;

		ctx.pos[1] = *ctx.axis2_hold_rel;
		ctx.pos[6] = *ctx.axis7_hold_rel;

		if (!ctx.startup->v_limit_scaled)
		{
			if (!plc_io::read_v_limit(ctx))
			{
				return false;
			}

			copy_positions(ctx.plc_v_limit, ctx.startup->v_limit_backup, 7);
			double scaled_v_limit[7] = { 0 };
			copy_positions(ctx.plc_v_limit, scaled_v_limit, 7);
			scaled_v_limit[0] *= ctx.cfg->startup_motion_speed_scale;
			scaled_v_limit[1] *= ctx.cfg->startup_motion_speed_scale;
			scaled_v_limit[2] *= ctx.cfg->startup_motion_speed_scale;
			scaled_v_limit[4] *= ctx.cfg->startup_motion_speed_scale;
			scaled_v_limit[5] *= ctx.cfg->startup_motion_speed_scale;
			scaled_v_limit[6] *= ctx.cfg->startup_motion_speed_scale;
			if (!plc_io::write_v_limit(ctx, scaled_v_limit))
			{
				return false;
			}
			ctx.startup->v_limit_scaled = true;
		}

		*ctx.guidewire_mode = GuidewireMode::None;
		ctx.axis6_crawl->enabled = false;
		*ctx.axis6_window_locked = false;
		*ctx.axis6_coop_ff_inited = false;
		*ctx.axis6_coop_prev_axis1_cmd_abs = 0.0;
		ctx.startup->phase = StartupPhase::ReleaseClamps;
		ctx.startup->phase_t0 = GetTickCount();
		ctx.startup->completed = false;
		ctx.startup->prompted = false;

		if (!plc_io::write_refer(ctx))
		{
			if (ctx.startup->v_limit_scaled)
			{
				plc_io::write_v_limit(ctx, ctx.startup->v_limit_backup);
				ctx.startup->v_limit_scaled = false;
			}
			return false;
		}

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
