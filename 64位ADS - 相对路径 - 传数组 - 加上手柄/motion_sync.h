// 文件职责说明：
// 1) 封装坐标换算、窗口重建与同步流程（sync_axis1/sync_axis6/sync_all）。
// 2) 统一轴5相对的 axis6 窗口计算与同步状态更新。
// 3) 作为主循环与导丝模式模块的运动同步基础能力层。
#pragma once

#include "control_types.h"

namespace motion_sync
{
	double from_left_to_abs(const AppContext& ctx, int axis_index, double from_left_mm);
	double from_left_to_rel(const AppContext& ctx, int axis_index, double from_left_mm);
	double axis1_window_left_abs(const AppContext& ctx);
	double axis1_window_right_abs(const AppContext& ctx);

	void calculate_axis6_window_from_axis5(
		const AppContext& ctx,
		double& window_start_abs,
		double& window_end_abs);
	void lock_axis6_window_from_current(AppContext& ctx);
	void apply_locked_axis6_window(AppContext& ctx);
	bool rebuild_axis6_window_from_axis5(AppContext& ctx, bool log_result);

	bool sync_axis1(AppContext& ctx, int samples);
	bool sync_axis6(
		AppContext& ctx,
		int samples,
		bool rebuild_window,
		bool log_window_rebuild);
	bool sync_all(AppContext& ctx, int samples);

	void capture_axis1_follow_baseline(AppContext& ctx);
	void apply_axis1_mirror_from_abs(AppContext& ctx, double axis1_abs_cmd, bool include_axis6);
}

