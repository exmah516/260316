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
	void calculate_axis6_window_from_axis5_abs(
		const AppContext& ctx,
		double axis5_abs,
		double& window_start_abs,
		double& window_end_abs);
	void lock_axis6_window_from_current(AppContext& ctx);
	void apply_locked_axis6_window(AppContext& ctx);
	bool rebuild_axis6_window_from_axis5(AppContext& ctx, bool log_result);

	bool sync_axis1(AppContext& ctx, int samples);
	// 计划回退交接专用：使用当前100 Hz位置快照和已滤波手柄值重建内存基准，
	// 不轮询手柄、不等待、不直接写ADS。
	bool rebase_axis1_after_return(AppContext& ctx);
	// 独立导丝回退交接：仅消费当前PLC快照内存和axis6已滤波手柄值，
	// 重建axis6/7基准并保留入模时锁定的axis6窗口。
	bool rebase_axis6_after_return(AppContext& ctx);
	// 协同回退交接：用同一份PLC快照同时重建双手柄、双轴链基准，
	// 并按axis5当前实际位置建立axis6动态窗口。
	bool rebase_cooperative_after_return(AppContext& ctx);
	bool sync_axis6(
		AppContext& ctx,
		int samples,
		bool rebuild_window,
		bool log_window_rebuild);
	bool sync_cooperative_guidewire(
		AppContext& ctx,
		int samples,
		bool log_window_rebuild);
	bool sync_all(AppContext& ctx, int samples);

	void capture_axis1_follow_baseline(AppContext& ctx);
	void apply_axis1_mirror_from_abs(AppContext& ctx, double axis1_abs_cmd, bool include_axis6);
}

