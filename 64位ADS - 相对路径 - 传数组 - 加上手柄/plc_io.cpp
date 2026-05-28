// 文件职责说明：
// 1) 实现 ADS 符号定义与 PLC I/O 封装。
// 2) 保持原控制程序 ADSRead/ADSWrite/ADSReadSum 的访问语义不变。
// 3) 不承载控制状态机，只提供数据访问能力。
#include "plc_io.h"

namespace AdsSymbol
{
	const char* refer = "G.refer";
	const char* act_pos = "G.Act_pos";
	const char* init_pos = "G.init_pos";
	const char* leftlimit = "G.leftlimit";
	const char* act_pos_from_left = "G.act_pos_from_left";
	const char* refer_from_left = "G.refer_from_left";
	const char* v_limit = "G.v_limit";
	const char* cylinder1_value = "G.cylinder1_value";
	const char* cylinder2_value = "G.cylinder2_value";
	const char* cylinder3_value = "G.cylinder3_value";
	const char* cylinder4_value = "G.cylinder4_value";
	const char* cylinder5_cmd = "G.cylinder5_cmd";
	const char* cylinder5_press_req = "G.cylinder5_press_req";
	const char* cylinder5_value = "G.cylinder5_value";
	const char* self_check_done = "G.self_check_done";
	const char* handle_reinit_req = "G.handle_reinit_req";
	const char* estop_hold_req = "G.estop_hold_req";
	const char* ft_1_value = "G.ft_1_value";
	const char* fn_1_value = "G.fn_1_value";
	const char* fn_2_value = "G.fn_2_value";
	const char* ft_2_value = "G.ft_2_value";
	const char* axis1_fast_return = "G.axis1_fast_return";
	const char* axis6_fast_retract = "G.axis6_fast_retract";
	const char* startup_smoothing_bypass = "G.startup_smoothing_bypass";
	const char* axis4_fwd_req = "G.axis4_fwd_req";
	const char* axis4_rev_req = "G.axis4_rev_req";
	const char* axis4_manual_busy = "G.axis4_manual_busy";
	const char* axis4_manual_error = "G.axis4_manual_error";
	const char* axis4_manual_error_id = "G.axis4_manual_error_id";
	const char* gen_state = "G.gen_state";
	const char* app_name = "TwinCAT_SystemInfoVarList._AppInfo.AppName";

	const AxisReturnAdsSymbols axis1_return = {
		"G.return_cmd[1].Req",
		"G.return_cmd[1].Busy",
		"G.return_cmd[1].Done",
		"G.return_cmd[1].Error",
		"G.return_cmd[1].ErrorId",
		"G.return_cmd[1].TargetAbs",
		"G.return_cmd[1].Velocity",
		"G.return_cmd[1].Acc",
		"G.return_cmd[1].Dec",
		"G.return_cmd[1].Jerk"
	};

	const AxisReturnAdsSymbols axis3_return = {
		"G.return_cmd[3].Req",
		"G.return_cmd[3].Busy",
		"G.return_cmd[3].Done",
		"G.return_cmd[3].Error",
		"G.return_cmd[3].ErrorId",
		"G.return_cmd[3].TargetAbs",
		"G.return_cmd[3].Velocity",
		"G.return_cmd[3].Acc",
		"G.return_cmd[3].Dec",
		"G.return_cmd[3].Jerk"
	};

	const AxisReturnAdsSymbols axis5_return = {
		"G.return_cmd[5].Req",
		"G.return_cmd[5].Busy",
		"G.return_cmd[5].Done",
		"G.return_cmd[5].Error",
		"G.return_cmd[5].ErrorId",
		"G.return_cmd[5].TargetAbs",
		"G.return_cmd[5].Velocity",
		"G.return_cmd[5].Acc",
		"G.return_cmd[5].Dec",
		"G.return_cmd[5].Jerk"
	};

	const AxisReturnAdsSymbols axis6_return = {
		"G.return_cmd[6].Req",
		"G.return_cmd[6].Busy",
		"G.return_cmd[6].Done",
		"G.return_cmd[6].Error",
		"G.return_cmd[6].ErrorId",
		"G.return_cmd[6].TargetAbs",
		"G.return_cmd[6].Velocity",
		"G.return_cmd[6].Acc",
		"G.return_cmd[6].Dec",
		"G.return_cmd[6].Jerk"
	};
}

namespace plc_io
{
	bool read_plc_state(AppContext& ctx)
	{
		const char* symbols[] = {
			AdsSymbol::act_pos,
			AdsSymbol::init_pos,
			AdsSymbol::leftlimit
		};
		const unsigned long lengths[] = {
			static_cast<unsigned long>(sizeof(double) * 7),
			static_cast<unsigned long>(sizeof(double) * 7),
			static_cast<unsigned long>(sizeof(double) * 7)
		};
		void* outputs[] = {
			ctx.plc_act_pos,
			ctx.plc_init_pos,
			ctx.plc_leftlimit
		};
		return ctx.ads->ADSReadSum(symbols, lengths, outputs, 3);
	}

	bool read_force_sample(AppContext& ctx, ForceSampleFrame& sample)
	{
		// 力采样仅保留 ADS 原有链路；TCP 仅用于日志替换 ft_1/fn_1，不进入控制链路。
		double act_pos_snapshot[7] = { 0.0 };
		const char* symbols[] = {
			AdsSymbol::ft_1_value,
			AdsSymbol::fn_1_value,
			AdsSymbol::fn_2_value,
			AdsSymbol::ft_2_value,
			AdsSymbol::act_pos
		};
		const unsigned long lengths[] = {
			static_cast<unsigned long>(sizeof(sample.ft_1_value)),
			static_cast<unsigned long>(sizeof(sample.fn_1_value)),
			static_cast<unsigned long>(sizeof(sample.fn_2_value)),
			static_cast<unsigned long>(sizeof(sample.ft_2_value)),
			static_cast<unsigned long>(sizeof(act_pos_snapshot))
		};
		void* outputs[] = {
			&sample.ft_1_value,
			&sample.fn_1_value,
			&sample.fn_2_value,
			&sample.ft_2_value,
			act_pos_snapshot
		};
		if (!ctx.ads->ADSReadSum(symbols, lengths, outputs, 5))
		{
			return false;
		}

		// 轴1位置采用 G.Act_pos 的第 1 轴（数组下标 0，单位与 PLC Act_pos 保持一致）。
		sample.axis1_pos_rel = act_pos_snapshot[0];
		sample.valid = true;
		sample.tick_ms = GetTickCount();
		return true;
	}

	bool write_refer(AppContext& ctx)
	{
		// 将当前 pos[7] 写回 PLC 参考位数组 G.refer。
		return ctx.ads->ADSWrite(AdsSymbol::refer, sizeof(double) * 7, ctx.pos);
	}

	bool read_v_limit(AppContext& ctx)
	{
		// 读取 PLC 当前速度上限（用于启动准备阶段缩放/恢复）。
		return ctx.ads->ADSRead(AdsSymbol::v_limit, sizeof(double) * 7, ctx.plc_v_limit);
	}

	bool write_v_limit(AppContext& ctx, const double* values)
	{
		return ctx.ads->ADSWrite(AdsSymbol::v_limit, sizeof(double) * 7, const_cast<double*>(values));
	}

	void load_pos_from_actual(AppContext& ctx)
	{
		// 以当前实际位置为基线重建一帧 refer，避免直接使用旧参考引发跳变。
		copy_positions(ctx.plc_act_pos, ctx.pos, 7);
	}

	bool read_axis_return_status(AppContext& ctx, const AxisReturnAdsSymbols& symbols, AxisReturnStatus& status)
	{
		// 轮询 PLC 计划回退命令状态位（Busy/Done/Error/ErrorId）。
		bool ok = true;
		ok = ctx.ads->ADSRead(symbols.busy, sizeof(status.busy), &status.busy) && ok;
		ok = ctx.ads->ADSRead(symbols.done, sizeof(status.done), &status.done) && ok;
		ok = ctx.ads->ADSRead(symbols.error, sizeof(status.error), &status.error) && ok;
		ok = ctx.ads->ADSRead(symbols.error_id, sizeof(status.error_id), &status.error_id) && ok;
		return ok;
	}

	bool clear_axis_return_request(AppContext& ctx, const AxisReturnAdsSymbols& symbols)
	{
		// 清除单轴计划回退触发位 Req。
		bool req = false;
		return ctx.ads->ADSWrite(symbols.req, sizeof(req), &req);
	}

	bool request_axis_return(
		AppContext& ctx,
		const AxisReturnAdsSymbols& symbols,
		double target_abs,
		double velocity,
		double acc,
		double dec,
		double jerk)
	{
		// 下发一次计划回退参数并置位 Req：
		// target_abs(mm), velocity(mm/s), acc/dec(mm/s^2), jerk(mm/s^3)。
		bool req = true;
		bool ok = true;
		ok = ctx.ads->ADSWrite(symbols.target_abs, sizeof(target_abs), &target_abs) && ok;
		ok = ctx.ads->ADSWrite(symbols.velocity, sizeof(velocity), &velocity) && ok;
		ok = ctx.ads->ADSWrite(symbols.acc, sizeof(acc), &acc) && ok;
		ok = ctx.ads->ADSWrite(symbols.dec, sizeof(dec), &dec) && ok;
		ok = ctx.ads->ADSWrite(symbols.jerk, sizeof(jerk), &jerk) && ok;
		ok = ctx.ads->ADSWrite(symbols.req, sizeof(req), &req) && ok;
		return ok;
	}

	bool clear_axis1_group_return_requests(AppContext& ctx)
	{
		// axis1 相关轴群共用的回退请求清理（1/3/5/6）。
		bool ok = true;
		ok = clear_axis_return_request(ctx, AdsSymbol::axis1_return) && ok;
		ok = clear_axis_return_request(ctx, AdsSymbol::axis3_return) && ok;
		ok = clear_axis_return_request(ctx, AdsSymbol::axis5_return) && ok;
		ok = clear_axis_return_request(ctx, AdsSymbol::axis6_return) && ok;
		return ok;
	}

	bool write_axis4_manual_requests(AppContext& ctx, bool forward_req, bool reverse_req)
	{
		// Axis4 点动请求由 PLC 侧状态机执行，本处仅负责写入请求位。
		bool ok = true;
		ok = ctx.ads->ADSWrite(AdsSymbol::axis4_fwd_req, sizeof(forward_req), &forward_req) && ok;
		ok = ctx.ads->ADSWrite(AdsSymbol::axis4_rev_req, sizeof(reverse_req), &reverse_req) && ok;
		return ok;
	}

	void clear_plc_reinit_req(AppContext& ctx)
	{
		bool clear_val = false;
		ctx.ads->ADSWrite(AdsSymbol::handle_reinit_req, sizeof(clear_val), &clear_val);
	}
}
