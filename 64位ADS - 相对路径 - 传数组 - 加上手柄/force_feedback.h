// 文件职责说明：
// 1) 封装力反馈映射函数与输出路由策略。
// 2) 保持 F 开关、导管/导丝输出分流、快退冻结语义不变。
// 3) 对外提供 process_force_feedback，供主循环直接调用。
#pragma once

#include "control_types.h"

double map_fn1_to_force_582(short fn_1_raw);
double map_ft1_to_torque_582(short ft_1_raw);
void map_force_587_placeholder(short fn_2_raw, short ft_2_raw, double& out_f, double& out_n);

void process_force_feedback(
	ForceFeedbackState& ff,
	const ForceSampleFrame& sample,
	Handle& handle_582,
	Handle& handle_587,
	GuidewireMode guidewire_mode,
	bool control_active,
	bool freeze_active,
	bool estop_hold_active,
	bool axis1_fast_return,
	bool axis6_fast_retract,
	int loop_count);

