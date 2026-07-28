// 文件职责说明：
// 1) 封装启动准备入口、装卸资格位消费、速度限制恢复与操作提示输出。
// 2) 入口负责选择标准装卸启动或中断恢复启动，具体分阶段调度仍由主循环执行。
// 3) 不改变原 7 轴 ADS 数组契约。
#pragma once

#include "control_types.h"

namespace startup_sequence
{
	bool start_startup_sequence(AppContext& ctx);
	bool consume_startup_loading_ready(AppContext& ctx);
	bool restore_startup_v_limit(AppContext& ctx);
	void prompt_startup_mode(AppContext& ctx);
}
