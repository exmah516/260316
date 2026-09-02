// 文件职责说明：
// 1) 实现 control_types.h 中声明的公共工具函数。
// 2) 保持与原 main.cpp 一致的基础数学行为。
// 3) 不承载模式切换、同步状态机等业务逻辑。
#include "control_types.h"

#include <clocale>
#include <cstdio>
#include <windows.h>

void setup_console_utf8()
{
	SetConsoleOutputCP(CP_UTF8);
	SetConsoleCP(CP_UTF8);
	std::setlocale(LC_ALL, ".UTF-8");
}

double clamp_double(double value, double low, double high)
{
	if (value < low) return low;
	if (value > high) return high;
	return value;
}
bool is_within_range(double value, double low, double high, double tol)
{
	return (value >= (low - tol)) && (value <= (high + tol));
}

bool get_average_handle_pose(Handle& handle, int samples, double& axis0, double& axis1)
{
	if (samples <= 0) return false;
	double axis0_sum = 0.0;
	double axis1_sum = 0.0;
	int valid_samples = 0;

	for (int i = 0; i < samples; ++i)
	{
		if (handle.poll())
		{
			axis0_sum += handle.fJoints2[0];
			axis1_sum += handle.fJoints2[1];
			++valid_samples;
		}
		Sleep(10);
	}

	if (valid_samples == 0) return false;
	const double inv = 1.0 / static_cast<double>(valid_samples);
	axis0 = axis0_sum * inv;
	axis1 = axis1_sum * inv;
	return true;
}

bool get_average_dual_pos(
	Handle& handle_a,
	Handle& handle_b,
	int samples,
	double& a_axis0,
	double& a_axis1,
	double& b_axis0,
	double& b_axis1)
{
	if (samples <= 0) return false;
	double a0_sum = 0.0;
	double a1_sum = 0.0;
	double b0_sum = 0.0;
	double b1_sum = 0.0;
	int valid_samples = 0;

	for (int i = 0; i < samples; ++i)
	{
		const bool ok_a = handle_a.poll();
		const bool ok_b = handle_b.poll();
		if (ok_a && ok_b)
		{
			a0_sum += handle_a.fJoints2[0];
			a1_sum += handle_a.fJoints2[1];
			b0_sum += handle_b.fJoints2[0];
			b1_sum += handle_b.fJoints2[1];
			++valid_samples;
		}
		Sleep(10);
	}

	if (valid_samples == 0) return false;
	const double inv = 1.0 / static_cast<double>(valid_samples);
	a_axis0 = a0_sum * inv;
	a_axis1 = a1_sum * inv;
	b_axis0 = b0_sum * inv;
	b_axis1 = b1_sum * inv;
	return true;
}

void copy_positions(const double* src, double* dst, int count)
{
	for (int i = 0; i < count; ++i)
	{
		dst[i] = src[i];
	}
}
