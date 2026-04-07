// 文件职责说明：
// 1) 封装启动准备入口、速度限制恢复与操作提示输出。
// 2) 保持原启动阶段触发时机与状态写入顺序。
// 3) 不改变启动状态机核心行为，只抽离可复用流程。
#pragma once

#include "control_types.h"

namespace startup_sequence
{
	bool start_startup_sequence(AppContext& ctx);
	bool restore_startup_v_limit(AppContext& ctx);
	void prompt_startup_mode(AppContext& ctx);
}

