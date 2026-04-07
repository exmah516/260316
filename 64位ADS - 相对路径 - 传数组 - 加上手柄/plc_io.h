// 文件职责说明：
// 1) 封装 ADS 相关符号名与底层读写函数。
// 2) 保持与原 main.cpp 一致的 ADS 调用顺序与符号语义。
// 3) 提供给主循环与同步模块的最小 I/O 接口。
#pragma once

#include "control_types.h"

namespace AdsSymbol
{
	extern const char* refer;
	extern const char* act_pos;
	extern const char* init_pos;
	extern const char* leftlimit;
	extern const char* act_pos_from_left;
	extern const char* refer_from_left;
	extern const char* v_limit;
	extern const char* cylinder1_value;
	extern const char* cylinder2_value;
	extern const char* cylinder3_value;
	extern const char* cylinder4_value;
	extern const char* cylinder5_cmd;
	extern const char* cylinder5_press_req;
	extern const char* cylinder5_value;
	extern const char* self_check_done;
	extern const char* handle_reinit_req;
	extern const char* estop_hold_req;
	extern const char* ft_1_value;
	extern const char* fn_1_value;
	extern const char* fn_2_value;
	extern const char* ft_2_value;
	extern const char* axis1_fast_return;
	extern const char* axis6_fast_retract;
	extern const char* startup_smoothing_bypass;
	extern const char* axis4_fwd_req;
	extern const char* axis4_rev_req;
	extern const char* axis4_manual_busy;
	extern const char* axis4_manual_error;
	extern const char* axis4_manual_error_id;
	extern const char* gen_state;
	extern const char* app_name;

	extern const AxisReturnAdsSymbols axis1_return;
	extern const AxisReturnAdsSymbols axis3_return;
	extern const AxisReturnAdsSymbols axis5_return;
	extern const AxisReturnAdsSymbols axis6_return;
}

namespace plc_io
{
	bool read_plc_state(AppContext& ctx);
	bool read_force_sample(AppContext& ctx, ForceSampleFrame& sample);
	bool write_refer(AppContext& ctx);
	bool read_v_limit(AppContext& ctx);
	bool write_v_limit(AppContext& ctx, const double* values);
	void load_pos_from_actual(AppContext& ctx);

	bool read_axis_return_status(AppContext& ctx, const AxisReturnAdsSymbols& symbols, AxisReturnStatus& status);
	bool clear_axis_return_request(AppContext& ctx, const AxisReturnAdsSymbols& symbols);
	bool request_axis_return(
		AppContext& ctx,
		const AxisReturnAdsSymbols& symbols,
		double target_abs,
		double velocity,
		double acc,
		double dec,
		double jerk);
	bool clear_axis1_group_return_requests(AppContext& ctx);
	bool write_axis4_manual_requests(AppContext& ctx, bool forward_req, bool reverse_req);
	void clear_plc_reinit_req(AppContext& ctx);
}

