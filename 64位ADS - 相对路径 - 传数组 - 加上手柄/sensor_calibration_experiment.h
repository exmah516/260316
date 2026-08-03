#pragma once

namespace sensor_calibration_experiment
{
	// 标定工具模式必须在手柄和运动控制对象初始化之前分流。
	bool is_command(int argc, char* argv[]);
	int run(int argc, char* argv[]);
}
