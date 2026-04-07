// 文件职责说明：
// 1) 封装坐标换算、窗口重建与同步流程（sync_axis1/sync_axis6/sync_all）。
// 2) 对齐原 main.cpp 的执行顺序与状态更新，不改变同步语义。
// 3) 作为主循环与导丝模式模块的运动同步基础能力层。
#pragma once

#include "control_types.h"

namespace motion_sync
{
	double from_left_to_abs(const AppContext& ctx, int axis_index, double from_left_mm);
	double from_left_to_rel(const AppContext& ctx, int axis_index, double from_left_mm);
	double axis1_window_left_abs(const AppContext& ctx);
	double axis1_window_right_abs(const AppContext& ctx);

	bool set_axis6_independent_window(AppContext& ctx, double window_right_abs, bool log_clamp);
	void lock_axis6_window_from_current(AppContext& ctx);
	void apply_locked_axis6_window(AppContext& ctx);
	bool rebuild_axis6_window_if_covered(AppContext& ctx, const char* reason, bool log_result);

	bool sync_axis1(AppContext& ctx, int samples, bool wait_rearm, int rearm_dir);
	bool sync_axis6(
		AppContext& ctx,
		int samples,
		bool capture_window,
		bool wait_rearm,
		int rearm_dir,
		bool check_window_cover,
		bool log_window_cover);
	bool sync_all(AppContext& ctx, int samples);

	void capture_axis1_follow_baseline(AppContext& ctx);
	void apply_axis1_mirror_from_abs(AppContext& ctx, double axis1_abs_cmd, bool include_axis6);
}

