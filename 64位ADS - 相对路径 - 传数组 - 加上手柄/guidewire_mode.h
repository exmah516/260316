// 文件职责说明：
// 1) 封装导丝模式进入/退出与门控判断流程。
// 2) 保持原按键语义与日志语义，不改变模式切换条件。
// 3) 复用 motion_sync/plc_io 完成重同步与窗口管理。
#pragma once

#include "control_types.h"

namespace guidewire_mode_ctrl
{
	bool enter_independent_guidewire_mode(AppContext& ctx);
	bool enter_cooperative_guidewire_mode(AppContext& ctx);
	bool check_axis6_guidewire_entry_gate(AppContext& ctx, double& axis6_from_left_mm);
	bool exit_guidewire_mode_to_normal(AppContext& ctx);
}

