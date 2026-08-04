// 文件职责说明：
// 1) 实现 ADS 符号定义与 PLC I/O 封装。
// 2) 通信服务启动后，将运行期专项读写排入后台线程；启动前保留兼容直连。
// 3) 不承载控制状态机，只提供数据访问与低频状态缓存能力。
#include "plc_io.h"

#include "ads_communication.h"

#include <array>
#include <cstring>
#include <iostream>
#include <string>

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
	const char* startup_loading_ready = "G.startup_loading_ready";
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
	const char* axis4_manual_done = "G.axis4_manual_done";
	const char* axis4_manual_error = "G.axis4_manual_error";
	const char* axis4_manual_error_id = "G.axis4_manual_error_id";
	const char* gen_state = "G.gen_state";
	const char* host_session_id = "G.host_session_id";
	const char* host_heartbeat_sequence = "G.host_heartbeat_sequence";
	const char* host_recover_req = "G.host_recover_req";
	const char* host_comm_timeout = "G.host_comm_timeout";
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

namespace
{
	constexpr std::uint64_t kAxisReturnStatusCacheMs = 100;

	struct AxisReturnStatusCache
	{
		AxisReturnStatus status{};
		std::uint64_t update_tick_ms = 0;
		bool valid = false;
	};

	std::array<AxisReturnStatusCache, 4> g_axis_return_status_cache{};

	bool ads_read(AppContext& ctx, const char* symbol, unsigned long length, void* output)
	{
		if (ctx.ads_service != nullptr)
		{
			return ctx.ads_service->read(symbol, length, output);
		}
		return ctx.ads != nullptr && ctx.ads->ADSRead(symbol, length, output);
	}

	bool ads_write(AppContext& ctx, const char* symbol, unsigned long length, const void* input)
	{
		if (ctx.ads_service != nullptr)
		{
			return ctx.ads_service->write(symbol, length, input);
		}
		return ctx.ads != nullptr &&
			ctx.ads->ADSWrite(symbol, length, const_cast<void*>(input));
	}

	bool ads_read_sum(
		AppContext& ctx,
		const char* const* symbols,
		const unsigned long* lengths,
		void* const* outputs,
		unsigned long count)
	{
		if (ctx.ads_service != nullptr)
		{
			return ctx.ads_service->read_sum(symbols, lengths, outputs, count);
		}
		return ctx.ads != nullptr && ctx.ads->ADSReadSum(symbols, lengths, outputs, count);
	}

	bool ads_write_sum(
		AppContext& ctx,
		const char* const* symbols,
		const unsigned long* lengths,
		const void* const* inputs,
		unsigned long count)
	{
		if (ctx.ads_service != nullptr)
		{
			return ctx.ads_service->write_sum(symbols, lengths, inputs, count);
		}
		return ctx.ads != nullptr && ctx.ads->ADSWriteSum(symbols, lengths, inputs, count);
	}

	std::string ads_error_text(const AppContext& ctx)
	{
		if (ctx.ads == nullptr) return "ADS 对象不可用。\n";
		const std::string error = ctx.ads->GetLastErrorCopy();
		return error.empty() ? "后台 ADS 请求失败。\n" : error;
	}

	int axis_return_cache_index(const AxisReturnAdsSymbols& symbols)
	{
		if (std::strcmp(symbols.req, AdsSymbol::axis1_return.req) == 0) return 0;
		if (std::strcmp(symbols.req, AdsSymbol::axis3_return.req) == 0) return 1;
		if (std::strcmp(symbols.req, AdsSymbol::axis5_return.req) == 0) return 2;
		if (std::strcmp(symbols.req, AdsSymbol::axis6_return.req) == 0) return 3;
		return -1;
	}

	void invalidate_axis_return_cache(const AxisReturnAdsSymbols& symbols)
	{
		const int index = axis_return_cache_index(symbols);
		if (index >= 0)
		{
			g_axis_return_status_cache[static_cast<std::size_t>(index)].valid = false;
		}
	}
}

namespace plc_io
{
	bool read_plc_state(AppContext& ctx)
	{
		if (ctx.ads_service != nullptr)
		{
			AdsFastSnapshot snapshot{};
			if (!ctx.ads_service->latest_snapshot(snapshot) || !snapshot.position_valid)
			{
				return false;
			}
			copy_positions(snapshot.act_pos_rel, ctx.plc_act_pos, 7);
			copy_positions(snapshot.act_pos_from_left, ctx.plc_act_pos_from_left, 7);
			if (!ctx.ads_service->coordinate_cache(ctx.plc_init_pos, ctx.plc_leftlimit))
			{
				return false;
			}
			ctx.plc_snapshot_qpc_ticks = snapshot.qpc_ticks;
			ctx.plc_snapshot_sequence = snapshot.attempt_sequence;
			return true;
		}

		LARGE_INTEGER qpc_before{};
		LARGE_INTEGER qpc_after{};
		QueryPerformanceCounter(&qpc_before);
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
		const bool success = ads_read_sum(ctx, symbols, lengths, outputs, 3);
		QueryPerformanceCounter(&qpc_after);
		if (success)
		{
			ctx.plc_snapshot_qpc_ticks = qpc_before.QuadPart + (qpc_after.QuadPart - qpc_before.QuadPart) / 2;
			++ctx.plc_snapshot_sequence;
		}
		return success;
	}

	bool read_force_sample(AppContext& ctx, ForceSampleFrame& sample)
	{
		if (ctx.ads_service != nullptr)
		{
			AdsFastSnapshot snapshot{};
			if (!ctx.ads_service->latest_snapshot(snapshot) || !snapshot.force_valid)
			{
				return false;
			}
			sample.ft_1_value = snapshot.ft_1_value;
			sample.fn_1_value = snapshot.fn_1_value;
			sample.fn_2_value = snapshot.fn_2_value;
			sample.ft_2_value = snapshot.ft_2_value;
			sample.axis1_pos_rel = snapshot.act_pos_rel[0];
			sample.axis2_pos_rel = snapshot.act_pos_rel[1];
			sample.ft_1_value_v = static_cast<double>(sample.ft_1_value) / 1000.0;
			sample.fn_1_value_v = static_cast<double>(sample.fn_1_value) / 1000.0;
			sample.fn_2_value_v = static_cast<double>(sample.fn_2_value) / 1000.0;
			sample.ft_2_value_v = static_cast<double>(sample.ft_2_value) / 1000.0;
			sample.valid = true;
			sample.tick_ms = GetTickCount();
			sample.qpc_ticks = snapshot.qpc_ticks;
			return true;
		}

		LARGE_INTEGER qpc_before{};
		LARGE_INTEGER qpc_after{};
		QueryPerformanceCounter(&qpc_before);
		// ADS 侧力采样用于 PLC 原始变量读取；TCP_DAQ 模式会在主循环中覆盖 ft_1/fn_1。
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
		if (!ads_read_sum(ctx, symbols, lengths, outputs, 5))
		{
			QueryPerformanceCounter(&qpc_after);
			sample.qpc_ticks = qpc_before.QuadPart + (qpc_after.QuadPart - qpc_before.QuadPart) / 2;
			return false;
		}
		QueryPerformanceCounter(&qpc_after);

		// 轴1位置采用 G.Act_pos 的第 1 轴（数组下标 0，单位与 PLC Act_pos 保持一致）。
		sample.axis1_pos_rel = act_pos_snapshot[0];
		// 轴2位置由倍福直接给出角度，力反馈重力补偿不再使用电机 counts 换算。
		sample.axis2_pos_rel = act_pos_snapshot[1];
		// PLC 原始输入约定 1000 counts = 1 V。所有后续标零、标定和日志统一使用伏特。
		sample.ft_1_value_v = static_cast<double>(sample.ft_1_value) / 1000.0;
		sample.fn_1_value_v = static_cast<double>(sample.fn_1_value) / 1000.0;
		sample.fn_2_value_v = static_cast<double>(sample.fn_2_value) / 1000.0;
		sample.ft_2_value_v = static_cast<double>(sample.ft_2_value) / 1000.0;
		sample.valid = true;
		sample.tick_ms = GetTickCount();
		sample.qpc_ticks = qpc_before.QuadPart + (qpc_after.QuadPart - qpc_before.QuadPart) / 2;
		return true;
	}

	bool write_refer(AppContext& ctx)
	{
		// 将当前 pos[7] 写回 PLC 参考位数组 G.refer。
		return ads_write(ctx, AdsSymbol::refer, sizeof(double) * 7, ctx.pos);
	}

	bool read_v_limit(AppContext& ctx)
	{
		// 读取 PLC 当前速度上限（用于启动准备阶段缩放/恢复）。
		return ads_read(ctx, AdsSymbol::v_limit, sizeof(double) * 7, ctx.plc_v_limit);
	}

	bool write_v_limit(AppContext& ctx, const double* values)
	{
		return ads_write(ctx, AdsSymbol::v_limit, sizeof(double) * 7, values);
	}

	bool read_startup_loading_ready(AppContext& ctx, bool& ready)
	{
		if (ctx.ads_service != nullptr)
		{
			if (ctx.ads_service->stats().state != AdsConnectionState::Running) return false;
			ready = ctx.ads_service->event_state().startup_loading_ready;
			return true;
		}
		return ads_read(ctx, AdsSymbol::startup_loading_ready, sizeof(ready), &ready);
	}

	bool write_startup_loading_ready(AppContext& ctx, bool ready)
	{
		return ads_write(ctx, AdsSymbol::startup_loading_ready, sizeof(ready), &ready);
	}

	bool write_cylinder_values(AppContext& ctx, const unsigned short values[4])
	{
		if (values == nullptr) return false;
		const char* symbols[] = {
			AdsSymbol::cylinder2_value,
			AdsSymbol::cylinder4_value,
			AdsSymbol::cylinder1_value,
			AdsSymbol::cylinder3_value
		};
		const unsigned long lengths[] = {
			sizeof(values[1]), sizeof(values[3]), sizeof(values[0]), sizeof(values[2])
		};
		const void* inputs[] = { &values[1], &values[3], &values[0], &values[2] };
		return ads_write_sum(ctx, symbols, lengths, inputs, 4);
	}

	bool write_startup_smoothing_bypass(AppContext& ctx, bool enabled)
	{
		return ads_write(
			ctx,
			AdsSymbol::startup_smoothing_bypass,
			sizeof(enabled),
			&enabled);
	}

	void load_pos_from_actual(AppContext& ctx)
	{
		// 以当前实际位置为基线重建一帧 refer，避免直接使用旧参考引发跳变。
		copy_positions(ctx.plc_act_pos, ctx.pos, 7);
	}

	bool read_axis_return_status(AppContext& ctx, const AxisReturnAdsSymbols& symbols, AxisReturnStatus& status)
	{
		// 计划回退状态最多 10 Hz 刷新；同一缓存窗口内复用结果，避免挤占 100 Hz 高频事务。
		status = AxisReturnStatus{};
		const int cache_index = axis_return_cache_index(symbols);
		const std::uint64_t now_ms = GetTickCount64();
		if (ctx.ads_service != nullptr)
		{
			if (ctx.ads_service->stats().state != AdsConnectionState::Running)
			{
				invalidate_axis_return_cache(symbols);
				return false;
			}
			if (cache_index >= 0)
			{
				const AxisReturnStatusCache& cache =
					g_axis_return_status_cache[static_cast<std::size_t>(cache_index)];
				if (cache.valid && now_ms - cache.update_tick_ms < kAxisReturnStatusCacheMs)
				{
					status = cache.status;
					return true;
				}
			}
		}

		const char* names[] = { symbols.busy, symbols.done, symbols.error, symbols.error_id };
		const unsigned long lengths[] = {
			sizeof(status.busy), sizeof(status.done), sizeof(status.error), sizeof(status.error_id)
		};
		void* outputs[] = { &status.busy, &status.done, &status.error, &status.error_id };
		if (!ads_read_sum(ctx, names, lengths, outputs, 4))
		{
			invalidate_axis_return_cache(symbols);
			std::cout << "计划回退 ADS 状态批量读取失败，错误：" << ads_error_text(ctx);
			return false;
		}
		if (cache_index >= 0)
		{
			AxisReturnStatusCache& cache =
				g_axis_return_status_cache[static_cast<std::size_t>(cache_index)];
			cache.status = status;
			cache.update_tick_ms = now_ms;
			cache.valid = true;
		}
		return true;
	}

	bool clear_axis_return_request(AppContext& ctx, const AxisReturnAdsSymbols& symbols)
	{
		// 清除单轴计划回退触发位 Req。
		bool req = false;
		if (ads_write(ctx, symbols.req, sizeof(req), &req))
		{
			invalidate_axis_return_cache(symbols);
			return true;
		}
		std::cout << "计划回退 ADS 请求清除失败：" << symbols.req
			<< "，错误：" << ads_error_text(ctx);
		return false;
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
		// 先清 Req，再完整写入参数，最后置位 Req，避免 PLC 消费到半更新参数。
		bool req = false;
		auto write_required = [&](const char* symbol, unsigned long length, const void* value) -> bool
		{
			if (ads_write(ctx, symbol, length, value))
			{
				return true;
			}
			std::cout << "计划回退 ADS 写入失败：" << symbol
				<< "，错误：" << ads_error_text(ctx);
			return false;
		};

		const char* parameter_names[] = {
			symbols.req, symbols.target_abs, symbols.velocity,
			symbols.acc, symbols.dec, symbols.jerk
		};
		const unsigned long parameter_lengths[] = {
			sizeof(req), sizeof(target_abs), sizeof(velocity),
			sizeof(acc), sizeof(dec), sizeof(jerk)
		};
		const void* parameter_values[] = {
			&req, &target_abs, &velocity, &acc, &dec, &jerk
		};
		if (!ads_write_sum(ctx, parameter_names, parameter_lengths, parameter_values, 6))
		{
			std::cout << "计划回退 ADS 参数批量写入失败，错误：" << ads_error_text(ctx);
			req = false;
			(void)ads_write(ctx, symbols.req, sizeof(req), &req);
			return false;
		}

		req = true;
		if (!write_required(symbols.req, sizeof(req), &req))
		{
			req = false;
			(void)ads_write(ctx, symbols.req, sizeof(req), &req);
			return false;
		}
		invalidate_axis_return_cache(symbols);
		return true;
	}

	bool clear_axis1_group_return_requests(AppContext& ctx)
	{
		// axis1 相关轴群共用的回退请求清理（1/3/5/6）。
		bool req = false;
		const char* symbols[] = {
			AdsSymbol::axis1_return.req,
			AdsSymbol::axis3_return.req,
			AdsSymbol::axis5_return.req,
			AdsSymbol::axis6_return.req
		};
		const unsigned long lengths[] = { sizeof(req), sizeof(req), sizeof(req), sizeof(req) };
		const void* inputs[] = { &req, &req, &req, &req };
		const bool ok = ads_write_sum(ctx, symbols, lengths, inputs, 4);
		if (ok)
		{
			invalidate_axis_return_cache(AdsSymbol::axis1_return);
			invalidate_axis_return_cache(AdsSymbol::axis3_return);
			invalidate_axis_return_cache(AdsSymbol::axis5_return);
			invalidate_axis_return_cache(AdsSymbol::axis6_return);
		}
		return ok;
	}

	bool write_axis4_manual_requests(AppContext& ctx, bool forward_req, bool reverse_req)
	{
		// Axis4 点动请求由 PLC 侧状态机执行，本处仅负责写入请求位。
		const char* symbols[] = { AdsSymbol::axis4_fwd_req, AdsSymbol::axis4_rev_req };
		const unsigned long lengths[] = { sizeof(forward_req), sizeof(reverse_req) };
		const void* inputs[] = { &forward_req, &reverse_req };
		return ads_write_sum(ctx, symbols, lengths, inputs, 2);
	}

	void clear_plc_reinit_req(AppContext& ctx)
	{
		bool clear_val = false;
		(void)ads_write(ctx, AdsSymbol::handle_reinit_req, sizeof(clear_val), &clear_val);
	}
}
