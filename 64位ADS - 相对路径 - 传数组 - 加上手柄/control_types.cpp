// 文件职责说明：
// 1) 实现 control_types.h 中声明的公共工具函数与日志结构方法。
// 2) 保持与原 main.cpp 一致的时间戳、采样缓冲与基础数学行为。
// 3) 不承载模式切换、同步状态机等业务逻辑。
#include "control_types.h"

#include <clocale>
#include <cstdio>

void setup_console_utf8()
{
	SetConsoleOutputCP(CP_UTF8);
	SetConsoleCP(CP_UTF8);
	std::setlocale(LC_ALL, ".UTF-8");
}

std::string build_force_log_filename()
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	char name[128] = { 0 };
	sprintf_s(
		name,
		"Force_sensor_%04u%02u%02u_%02u%02u%02u.csv",
		static_cast<unsigned>(st.wYear),
		static_cast<unsigned>(st.wMonth),
		static_cast<unsigned>(st.wDay),
		static_cast<unsigned>(st.wHour),
		static_cast<unsigned>(st.wMinute),
		static_cast<unsigned>(st.wSecond));
	return std::string(name);
}

std::string build_force_log_timestamp()
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	char ts[64] = { 0 };
	sprintf_s(
		ts,
		"%04u-%02u-%02u %02u:%02u:%02u.%03u",
		static_cast<unsigned>(st.wYear),
		static_cast<unsigned>(st.wMonth),
		static_cast<unsigned>(st.wDay),
		static_cast<unsigned>(st.wHour),
		static_cast<unsigned>(st.wMinute),
		static_cast<unsigned>(st.wSecond),
		static_cast<unsigned>(st.wMilliseconds));
	return std::string(ts);
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

void get_average_handle_pose(Handle& handle, int samples, double& axis0, double& axis1)
{
	double axis0_sum = 0.0;
	double axis1_sum = 0.0;

	for (int i = 0; i < samples; ++i)
	{
		handle.poll();
		axis0_sum += handle.fJoints2[0];
		axis1_sum += handle.fJoints2[1];
		Sleep(10);
	}

	const double inv = 1.0 / static_cast<double>(samples);
	axis0 = axis0_sum * inv;
	axis1 = axis1_sum * inv;
}

void get_average_dual_pos(
	Handle& handle_a,
	Handle& handle_b,
	int samples,
	double& a_axis0,
	double& a_axis1,
	double& b_axis0,
	double& b_axis1)
{
	double a0_sum = 0.0;
	double a1_sum = 0.0;
	double b0_sum = 0.0;
	double b1_sum = 0.0;

	for (int i = 0; i < samples; ++i)
	{
		handle_a.poll();
		handle_b.poll();
		a0_sum += handle_a.fJoints2[0];
		a1_sum += handle_a.fJoints2[1];
		b0_sum += handle_b.fJoints2[0];
		b1_sum += handle_b.fJoints2[1];
		Sleep(10);
	}

	const double inv = 1.0 / static_cast<double>(samples);
	a_axis0 = a0_sum * inv;
	a_axis1 = a1_sum * inv;
	b_axis0 = b0_sum * inv;
	b_axis1 = b1_sum * inv;
}

void copy_positions(const double* src, double* dst, int count)
{
	for (int i = 0; i < count; ++i)
	{
		dst[i] = src[i];
	}
}

bool ForceLogState::open_file(const std::string& output_name)
{
	filename = output_name;
	file.open(filename.c_str(), std::ios::out | std::ios::trunc);
	if (!file.is_open())
	{
		return false;
	}

	// 表头沿用历史格式：仅替换 ft_1/fn_1 的来源，不改变列结构。
	file << "timestamp,ft_1_value,fn_1_value,fn_2_value,ft_2_value,mode_code,reverse_code,push_pull_code,rot_sign_code,axis1_pos_rel\n";
	last_sample_ms = 0;
	last_buffer_flush_ms = GetTickCount();
	return true;
}

bool ForceLogState::should_sample(DWORD now_ms) const
{
	if (!enabled)
	{
		return false;
	}
	if (period_ms == 0)
	{
		return true;
	}
	return (last_sample_ms == 0) || ((now_ms - last_sample_ms) >= period_ms);
}

void ForceLogState::append_sample(
	DWORD now_ms,
	double ft1_value,
	double fn1_value,
	short fn2_value,
	short ft2_value,
	int mode_code,
	int reverse_code,
	int push_pull_code,
	int rot_sign_code,
	double axis1_pos_rel)
{
	last_sample_ms = now_ms;
	if (!file.is_open())
	{
		return;
	}

	// 先写入内存缓冲，减少每周期磁盘写入对控制环的影响。
	char line[512] = { 0 };
	sprintf_s(
		line,
		"%s,%.6f,%.6f,%d,%d,%d,%d,%d,%d,%.6f\n",
		build_force_log_timestamp().c_str(),
		ft1_value,
		fn1_value,
		static_cast<int>(fn2_value),
		static_cast<int>(ft2_value),
		mode_code,
		reverse_code,
		push_pull_code,
		rot_sign_code,
		axis1_pos_rel);
	line_buffer += line;

	++buffered_lines;

	// 行数达到阈值或超过时间间隔就刷入文件。
	if (buffered_lines >= 100 || (now_ms - last_buffer_flush_ms) >= 500)
	{
		flush(false);
	}
}

void ForceLogState::flush(bool force_flush)
{
	if (!file.is_open())
	{
		return;
	}
	if (!line_buffer.empty())
	{
		file << line_buffer;
		line_buffer.clear();
		buffered_lines = 0;
	}
	if (force_flush)
	{
		file.flush();
	}
	last_buffer_flush_ms = GetTickCount();
}

void ForceLogState::close()
{
	flush(true);
	if (file.is_open())
	{
		file.close();
	}
}
