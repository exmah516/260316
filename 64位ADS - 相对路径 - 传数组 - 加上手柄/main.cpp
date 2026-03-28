#include "Handle.h"
#include <ADSComm1.h>

#include <cmath>
#include <conio.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <windows.h>

// 说明：
// 1) 本文件是 ADS 主控制程序入口，负责上位机状态机、手柄采样、ADS 读写与气缸/轴指令下发。
// 2) 坐标体系以左限位为基准：绝对位置 abs = plc_act_pos + plc_init_pos。
// 3) 关键运行模式：
//    - 普通导管模式（axis1 主导，axis3/5 随动）
//    - 导丝独立模式（axis6/7 独立）
//    - 导丝协同模式（axis1 链路保留，axis6 使用叠速前馈）
namespace
{

// 生成 Force_sensor 日志文件名：Force_sensor_YYYYMMDD_HHMMSS.csv
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

// 生成带毫秒时间戳：YYYY-MM-DD HH:MM:SS.mmm
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

// 当前运行流程使用 G.leftlimit，并将所有启动/爬行
// 窗口都从左限位参考的绝对位置推导出来。

// 将标量值限制在闭区间内。
// 将标量值限制在闭区间 [low, high]，用于目标位置与速度派生值的安全裁剪。
double clamp_double(double value, double low, double high)
{
	if (value < low) return low;
	if (value > high) return high;
	return value;
}

// 带容差的区间判断，用于窗口激活和到位检测。
// 带容差的区间判定，常用于窗口激活判断与“到位”判定。
bool is_within_range(double value, double low, double high, double tol = 0.0)
{
	return (value >= (low - tol)) && (value <= (high + tol));
}

// 在同一短时间窗内采集单个手柄姿态，确保平移/旋转基准对齐。
// 在固定采样窗口内取均值，降低手柄瞬时抖动对重同步基准的影响。
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

// 在同一时间窗内同时采集两个手柄，确保共享重同步基准一致。
// 同时采样两把手柄，保证两路基准在同一时刻对齐。
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

// 用于 PLC 镜像数组和 refer 缓冲区的小工具函数。
// 简单数组拷贝工具：用于 PLC 数组缓存/备份/恢复。
void copy_positions(const double* src, double* dst, int count)
{
	for (int i = 0; i < count; ++i)
	{
		dst[i] = src[i];
	}
}

struct AxisReturnAdsSymbols
{
	const char* req = nullptr;
	const char* busy = nullptr;
	const char* done = nullptr;
	const char* error = nullptr;
	const char* error_id = nullptr;
	const char* target_abs = nullptr;
	const char* velocity = nullptr;
	const char* acc = nullptr;
	const char* dec = nullptr;
	const char* jerk = nullptr;
};

struct AxisReturnStatus
{
	bool busy = false;
	bool done = false;
	bool error = false;
	unsigned long error_id = 0;
};

struct HandleFilterState
{
	double axis0_filtered = 0.0;
	double axis1_filtered = 0.0;
	bool inited = false;

	void reset(double axis0, double axis1)
	{
		axis0_filtered = axis0;
		axis1_filtered = axis1;
		inited = true;
	}

	void update(double axis0_raw, double axis1_raw, double axis0_alpha, double axis1_alpha)
	{
		if (!inited)
		{
			reset(axis0_raw, axis1_raw);
			return;
		}

		axis0_filtered += axis0_alpha * (axis0_raw - axis0_filtered);
		axis1_filtered += axis1_alpha * (axis1_raw - axis1_filtered);
	}
};

namespace AdsSymbol
{
	const char* refer = "G.refer";
	const char* act_pos = "G.Act_pos";
	const char* init_pos = "G.init_pos";
	const char* leftlimit = "G.leftlimit";
	const char* act_pos_from_left = "G.act_pos_from_left";
	const char* refer_from_left = "G.refer_from_left";
	const char* v_limit = "G.v_limit";
	const char* cylinder1_value = "G.cylinder1_value";
	const char* cylinder2_value = "G.cylinder2_value";
	const char* cylinder3_value = "G.cylinder3_value";
	const char* cylinder4_value = "G.cylinder4_value";
	const char* self_check_done = "G.self_check_done";
	const char* handle_reinit_req = "G.handle_reinit_req";
	const char* estop_hold_req = "G.estop_hold_req";
	const char* fn_value = "G.fn_value";
	const char* ft_value = "G.ft_value";
	const char* ft_1_value = "G.ft_1_value";
	const char* fn_1_value = "G.fn_1_value";
	const char* fn_2_value = "G.fn_2_value";
	const char* ft_2_value = "G.ft_2_value";
	const char* axis1_fast_return = "G.axis1_fast_return";
	const char* axis6_fast_retract = "G.axis6_fast_retract";
	const char* startup_smoothing_bypass = "G.startup_smoothing_bypass";
	const char* axis4_fwd_req = "G.axis4_fwd_req";
	const char* axis4_rev_req = "G.axis4_rev_req";
	const char* axis4_manual_busy = "G.axis4_manual_busy";
	const char* axis4_manual_error = "G.axis4_manual_error";
	const char* axis4_manual_error_id = "G.axis4_manual_error_id";

	const AxisReturnAdsSymbols axis1_return = {
		"G.axis1_return_cmd.Req",
		"G.axis1_return_cmd.Busy",
		"G.axis1_return_cmd.Done",
		"G.axis1_return_cmd.Error",
		"G.axis1_return_cmd.ErrorId",
		"G.axis1_return_cmd.TargetAbs",
		"G.axis1_return_cmd.Velocity",
		"G.axis1_return_cmd.Acc",
		"G.axis1_return_cmd.Dec",
		"G.axis1_return_cmd.Jerk"
	};

	const AxisReturnAdsSymbols axis3_return = {
		"G.axis3_return_cmd.Req",
		"G.axis3_return_cmd.Busy",
		"G.axis3_return_cmd.Done",
		"G.axis3_return_cmd.Error",
		"G.axis3_return_cmd.ErrorId",
		"G.axis3_return_cmd.TargetAbs",
		"G.axis3_return_cmd.Velocity",
		"G.axis3_return_cmd.Acc",
		"G.axis3_return_cmd.Dec",
		"G.axis3_return_cmd.Jerk"
	};

	const AxisReturnAdsSymbols axis5_return = {
		"G.axis5_return_cmd.Req",
		"G.axis5_return_cmd.Busy",
		"G.axis5_return_cmd.Done",
		"G.axis5_return_cmd.Error",
		"G.axis5_return_cmd.ErrorId",
		"G.axis5_return_cmd.TargetAbs",
		"G.axis5_return_cmd.Velocity",
		"G.axis5_return_cmd.Acc",
		"G.axis5_return_cmd.Dec",
		"G.axis5_return_cmd.Jerk"
	};

	const AxisReturnAdsSymbols axis6_return = {
		"G.axis6_return_cmd.Req",
		"G.axis6_return_cmd.Busy",
		"G.axis6_return_cmd.Done",
		"G.axis6_return_cmd.Error",
		"G.axis6_return_cmd.ErrorId",
		"G.axis6_return_cmd.TargetAbs",
		"G.axis6_return_cmd.Velocity",
		"G.axis6_return_cmd.Acc",
		"G.axis6_return_cmd.Dec",
		"G.axis6_return_cmd.Jerk"
	};
}

struct ControlConfig
{
	// 手柄运动缩放系数与符号约定。
	double k_handle_to_mm = 500.0 * (75.0 / 50.0); // 手柄线性位移差 -> 轴位移增量(mm)
	double axis_push_sign = -1.0; // 手柄“推/拉”到轴“正/负”方向的映射符号
	double axis_rot_scale_deg = Rad;

	// 力反馈仅作用于 582 手柄。
	int axial_force_axis = 1;
	double axial_force_sign = -1.0;

	// 按键映射来自 buttons2 位掩码。
	unsigned char btn_b0 = 0x01;
	unsigned char btn_b5 = 0x20;
	unsigned char btn_b6 = 0x40;
	unsigned char btn_b7 = 0x80;

	// 基于左限位参考的爬行窗口。
	double axis1_window_left_from_left_mm = 3.0;
	double axis1_window_right_from_left_mm = 18.0;
	double axis6_independent_window_size_mm = 20.0;
	double axis56_ready_gap_mm = 20.0;
	double axis3_delivery_stop_from_left_mm = 20.0;
	double axis3_delivery_release_hysteresis_mm = 2.0;
	double guidewire_entry_axis6_from_left_max_mm = 665.0;
	// 导丝入模时的窗口覆盖判定阈值：delta<18mm 时按轴5参考重建 axis6 窗口。
	double axis6_window_cover_threshold_mm = 18.0;

	// 爬行触发/到位阈值。
	double crawl_trigger_deadband_mm = 0.3; // |delta| 小于此值视为无效输入（不触发 push/pull）
	double crawl_rearm_threshold_mm = 0.3; // 快退后允许再次触发前需要越过的同向阈值
	double crawl_arrive_tol_mm = 0.2;
	double hold_recover_rearm_mm = 0.6;
	// 线性增量抗噪死区：仅用于抑制差分噪声，不承担方向门控。
	double linear_increment_noise_deadband_mm = 0.02;
	// 正反切换一次性触发保护距离（离开该距离后重新允许触发）。
	double reverse_switch_trigger_guard_mm = 2.0;

	// PLC 规划快速回退参数。
	double axis1_return_velocity_mm_s = 200.0;
	double axis1_return_acc_mm_s2 = 2400.0;
	double axis1_return_dec_mm_s2 = 2400.0;
	double axis1_return_jerk_mm_s3 = 35000.0;
	// 快退前先切缸等待（轴1主链路）。
	DWORD axis1_pre_move_cylinder_wait_ms = 100;
	// 快退完成后切回最终缸态等待（轴1主链路）。
	DWORD axis1_post_return_cylinder_wait_ms = 100;
	DWORD axis1_return_settle_hold_ms = 20;
	DWORD axis1_return_transfer_settle_ms = 20;
	double axis1_pretrigger_preclamp_mm = 3.0;
	double axis1_preend_preclamp_mm = 3.0;
	double axis6_return_velocity_mm_s = 200.0;
	double axis6_return_acc_mm_s2 = 2400.0;
	double axis6_return_dec_mm_s2 = 2400.0;
	double axis6_return_jerk_mm_s3 = 35000.0;
	// 快退前先切缸等待（轴6链路）。
	DWORD axis6_pre_move_cylinder_wait_ms = 80;
	// 快退完成后切回最终缸态等待（轴6链路）。
	DWORD axis6_post_return_cylinder_wait_ms = 80;
	DWORD axis6_return_settle_hold_ms = 20;
	DWORD axis6_return_transfer_settle_ms = 20;
	double axis6_pretrigger_preclamp_mm = 3.0;
	double axis6_preend_preclamp_mm = 3.0;

	// 手柄低通滤波。
	double linear_handle_alpha = 0.25;
	double rotational_handle_alpha = 0.20;
	bool cooperative_debug_log = false;
	// 力感记录周期：0=每循环记录；>0=按毫秒周期记录。
	DWORD force_log_period_ms = 0;

	// 启动准备阶段目标。
	DWORD startup_clamp_settle_delay_ms = 300;
	double startup_motion_speed_scale = 0.5;
	unsigned short startup_cyl3_open = 500;
	unsigned short startup_cyl4_open = 0;
	unsigned short startup_cyl3_clamp = 0;
	unsigned short startup_cyl4_clamp = 1000;
	double startup_axis1_ready_from_left_mm = 30.0;
	double startup_axis5_ready_from_left_mm = 290.0;
	double startup_axis3_ready_from_left_mm = 635.0;
	// 在 axis3 完全到达目标前提前触发 cylinder2 夹紧；现场调参使其领先约 0.5 s。
	double startup_axis3_cyl2_clamp_advance_mm = 50.0;
};

struct CylinderPreset
{
	// 命名遵循当前接线方式：
	// cyl1/cyl2 属于导管侧爬行夹爪对，
	// cyl3/cyl4 属于导丝侧爬行夹爪对。
	unsigned short cyl1_open = 400;
	unsigned short cyl1_clamp = 00;
	unsigned short cyl1_preclamp = 320;
	unsigned short cyl2_open = 0;
	unsigned short cyl2_clamp = 600;
	unsigned short cyl2_preopen = 300;
	unsigned short cyl2_preclamp = 400;
	unsigned short cyl3_open = 320;
	unsigned short cyl3_clamp = 0;
	unsigned short cyl3_preclamp = 200;
	unsigned short cyl4_open = 0;
	unsigned short cyl4_clamp = 500;
	unsigned short cyl4_preopen = 300;
	unsigned short cyl4_preclamp = 300;
	unsigned short cyl3_follow_release = 150;
	unsigned short cyl4_follow_release = 100;
};

enum class GuidewireMode
{
	None,
	Independent,
	Cooperative
};

// 启动准备是在自检后进入的显式上位机流程。
enum class StartupPhase
{
	WaitForEnter,
	ReleaseClamps,
	MoveAxis56ToLeftReady,
	ClampCylinder34Wait,
	MoveAxis356BackToReady,
	ClampCylinder2AfterAxis3,
	Done
};

struct CrawlState
{
	// Follow：在激活窗口内直接映射手柄输入。
	// Switch/Clamp/Restore：围绕快速回退/推进动作的夹爪时序封装。
	enum class Phase
	{
		Follow,
		SwitchWait,
		FastMove,
		SettleHold,
		ClampWait,
		RestoreWait
	};

	bool enabled = false;
	Phase phase = Phase::Follow;
	bool wait_rearm = false;
	bool window_active = false;
	int rearm_dir = 0;
	double handle_ref = 0.0; // 当前控制段的手柄线性基准（重同步/重建基线时更新）
	double rot_ref = 0.0; // 当前控制段的手柄旋转基准
	double base_rel = 0.0; // 当前控制段的轴相对基线（PLC Act_pos 坐标）
	double rot_base_rel = 0.0; // 当前控制段的旋转轴相对基线
	double start_abs = 0.0; // 窗口起点绝对坐标(mm)
	double end_abs = 0.0; // 窗口终点绝对坐标(mm)
	double target_abs = 0.0; // 本次快退目标绝对坐标(mm)
	bool plc_move_requested = false;
	DWORD phase_t0 = 0;

	double min_abs() const { return (start_abs < end_abs) ? start_abs : end_abs; }
	double max_abs() const { return (start_abs > end_abs) ? start_abs : end_abs; }
};

struct ForceFeedbackState
{
	// 将滤波与有效性状态保持在本地，这样禁用力反馈时无需
	// 污染运动控制状态。
	bool enabled = false;
	short fn_raw = 0;
	short ft_raw = 0;
	double fn_bias = 0.0;
	bool fn_bias_inited = false;
	double fn_force_f = 0.0;
	double ft_force_f = 0.0;
	int last_fn_raw = 0;
	int last_ft_raw = 0;
	short fn_last_valid = 0;
	short ft_last_valid = 0;
	bool fn_has_valid = false;
	bool ft_has_valid = false;
	int fn_invalid_streak = 0;
	int ft_invalid_streak = 0;

	void reset()
	{
		fn_bias_inited = false;
		fn_force_f = 0.0;
		ft_force_f = 0.0;
		fn_invalid_streak = 0;
		ft_invalid_streak = 0;
	}

	void clear_output()
	{
		fn_bias_inited = false;
		fn_force_f = 0.0;
		ft_force_f = 0.0;
	}
};

struct ForceLogState
{
	// 力感记录默认开启；period_ms=0 表示每个主循环都记录。
	bool enabled = true;
	DWORD period_ms = 0;
	DWORD last_sample_ms = 0;
	DWORD last_buffer_flush_ms = 0;
	std::ofstream file;
	std::string filename;
	std::string line_buffer;
	size_t buffered_lines = 0;

	bool open_file(const std::string& output_name)
	{
		filename = output_name;
		file.open(filename.c_str(), std::ios::out | std::ios::trunc);
		if (!file.is_open())
		{
			return false;
		}

		// 表头固定为 Force_sensor 约定字段顺序。
		file << "timestamp,ft_1_value,fn_1_value,fn_2_value,ft_2_value\n";
		last_sample_ms = 0;
		last_buffer_flush_ms = GetTickCount();
		return true;
	}

	bool should_sample(DWORD now_ms) const
	{
		if (!enabled || !file.is_open())
		{
			return false;
		}
		if (period_ms == 0)
		{
			return true;
		}
		return (last_sample_ms == 0) || ((now_ms - last_sample_ms) >= period_ms);
	}

	void append_sample(DWORD now_ms, short ft1, short fn1, short fn2, short ft2)
	{
		if (!file.is_open())
		{
			return;
		}

		// 先写入内存缓冲，减少每周期磁盘写入对控制环的影响。
		line_buffer += build_force_log_timestamp();
		line_buffer += ",";
		line_buffer += std::to_string(static_cast<int>(ft1));
		line_buffer += ",";
		line_buffer += std::to_string(static_cast<int>(fn1));
		line_buffer += ",";
		line_buffer += std::to_string(static_cast<int>(fn2));
		line_buffer += ",";
		line_buffer += std::to_string(static_cast<int>(ft2));
		line_buffer += "\n";

		last_sample_ms = now_ms;
		++buffered_lines;

		// 行数达到阈值或超过时间间隔就刷入文件。
		if (buffered_lines >= 100 || (now_ms - last_buffer_flush_ms) >= 500)
		{
			flush(false);
		}
	}

	void flush(bool force_flush)
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

	void close()
	{
		flush(true);
		if (file.is_open())
		{
			file.close();
		}
	}
};

struct StartupState
{
	// 启动序列开始时一次性采集保持位姿。
	// move_base_rel 是启动第二段运动的跟随基准。
	StartupPhase phase = StartupPhase::WaitForEnter;
	bool completed = false;
	bool prompted = false;
	DWORD phase_t0 = 0;

	double axis1_hold_rel = 0.0;
	double axis2_hold_rel = 0.0;
	double axis3_hold_rel = 0.0;
	double axis5_hold_rel = 0.0;
	double axis6_hold_rel = 0.0;
	double axis7_hold_rel = 0.0;

	double axis3_move_base_rel = 0.0;
	double axis5_move_base_rel = 0.0;
	double axis6_move_base_rel = 0.0;

	double v_limit_backup[7] = {};
	bool v_limit_scaled = false;

	bool is_active() const
	{
		return phase != StartupPhase::WaitForEnter && phase != StartupPhase::Done;
	}
};

// 力反馈与运动生成有意解耦；当运动冻结、
// 保持或尚未激活时，会在不改动运动状态的情况下将力输出归零。
void process_force_feedback(
	ForceFeedbackState& ff,
	CADSComm& ads,
	Handle& handle,
	bool control_active,
	bool freeze_active,
	bool estop_hold_active,
	int loop_count,
	int force_axis,
	double force_sign)
{
	if (!estop_hold_active &&
		ff.enabled &&
		ads.ADSRead(AdsSymbol::fn_value, sizeof(ff.fn_raw), &ff.fn_raw) &&
		ads.ADSRead(AdsSymbol::ft_value, sizeof(ff.ft_raw), &ff.ft_raw))
	{
		const bool fn_valid = (std::abs(static_cast<int>(ff.fn_raw)) <= 8000);
		const bool ft_valid = (std::abs(static_cast<int>(ff.ft_raw)) <= 8000);

		if (fn_valid)
		{
			ff.fn_last_valid = ff.fn_raw;
			ff.fn_has_valid = true;
			ff.fn_invalid_streak = 0;
		}
		else
		{
			++ff.fn_invalid_streak;
		}

		if (ft_valid)
		{
			ff.ft_last_valid = ff.ft_raw;
			ff.ft_has_valid = true;
			ff.ft_invalid_streak = 0;
		}
		else
		{
			++ff.ft_invalid_streak;
		}

		if (ff.fn_has_valid)
		{
			ff.fn_raw = ff.fn_last_valid;
		}
		if (ff.ft_has_valid)
		{
			ff.ft_raw = ff.ft_last_valid;
		}

		if (!ff.fn_bias_inited)
		{
			ff.fn_bias = static_cast<double>(ff.fn_raw);
			ff.fn_bias_inited = true;
		}
		else
		{
			const double fn_err = static_cast<double>(ff.fn_raw) - ff.fn_bias;
			if (std::abs(fn_err) <= 120.0)
			{
				ff.fn_bias = ff.fn_bias * 0.995 + static_cast<double>(ff.fn_raw) * 0.005;
			}
		}

		double fn_zeroed = static_cast<double>(ff.fn_raw) - ff.fn_bias;
		if (std::abs(fn_zeroed) < 20.0)
		{
			fn_zeroed = 0.0;
		}

		const double axial_gain = 1.0 / 1000.0;
		const double axial_limit = 6.0;
		const double axial_force = clamp_double(fn_zeroed * axial_gain, -axial_limit, axial_limit);

		double torque_force = 0.0;
		if (ff.ft_raw >= -870 && ff.ft_raw <= -700)
		{
			torque_force = 0.0;
		}
		else if (ff.ft_raw > -700)
		{
			torque_force = (static_cast<double>(ff.ft_raw) + 700.0) * (-1.0 / 600.0);
		}
		else
		{
			torque_force = (static_cast<double>(ff.ft_raw) + 870.0) / (-530.0);
		}
		torque_force = clamp_double(torque_force, -1.0, 1.0);

		if (control_active)
		{
			ff.fn_force_f = ff.fn_force_f * 0.7 + (axial_force * 0.3);
			ff.ft_force_f = ff.ft_force_f * 0.7 + (torque_force * 0.3);
			handle.setforce_axis(ff.fn_force_f * force_sign, force_axis, ff.ft_force_f);
		}
		else
		{
			handle.setforce_axis(0.0, force_axis, 0.0);
		}

		if ((loop_count % 100) == 0 || ff.fn_raw != ff.last_fn_raw || ff.ft_raw != ff.last_ft_raw)
		{
			ff.last_fn_raw = ff.fn_raw;
			ff.last_ft_raw = ff.ft_raw;
			std::cout
				<< "fn_raw=" << ff.fn_raw << " fn_bias=" << ff.fn_bias
				<< " fn_cmd=" << ff.fn_force_f
				<< " | ft_raw=" << ff.ft_raw << " ft_cmd=" << ff.ft_force_f
				<< " | fn_inv=" << ff.fn_invalid_streak << " ft_inv=" << ff.ft_invalid_streak
				<< std::endl;
		}
	}
	else
	{
		ff.fn_force_f = 0.0;
		ff.ft_force_f = 0.0;
		if (!freeze_active)
		{
			handle.setforce_axis(0.0, force_axis, 0.0);
		}
	}
}

} // 命名空间

int main(int argc, char* argv[])
{
	const DWORD serial_axis1_handle = 582;
	const DWORD serial_axis6_handle = 587;
	const char* hardcoded_ads_netid = "169.254.119.135.1.1";

	// 工具模式：不进入运动环前先查看按键位掩码。
	if (argc > 1 && (std::string(argv[1]) == "--buttons" || std::string(argv[1]) == "--btn"))
	{
		DWORD test_serial = serial_axis1_handle;
		if (argc > 2)
		{
			test_serial = static_cast<DWORD>(std::strtoul(argv[2], nullptr, 10));
		}

		Handle test_handle(test_serial);
		if (!test_handle.init())
		{
			std::cout << "Handle Init Failed. Serial: " << test_serial << std::endl;
			return 0;
		}

		std::cout << "=== Button Test Mode ===" << std::endl;
		std::cout << "Serial: " << test_serial << std::endl;
		std::cout << "Press buttons to see their bits." << std::endl;
		std::cout << "Press ESC or 'q' to exit." << std::endl;

		unsigned char last_btn = 0xFF;
		while (true)
		{
			test_handle.poll();
			const unsigned char cur_btn = test_handle.buttons2;

			if (cur_btn != last_btn)
			{
				std::cout << "Btns: 0x" << std::hex << static_cast<int>(cur_btn) << std::dec << " | Bits: ";
				for (int i = 0; i < 8; ++i)
				{
					std::cout << ((cur_btn >> i) & 1);
				}
				std::cout << std::endl;
				last_btn = cur_btn;
			}

			if (_kbhit())
			{
				const int ch = _getch();
				if (ch == 27 || ch == 'q')
				{
					break;
				}
			}
			Sleep(10);
		}

		test_handle.close();
		return 0;
	}

	// 工具模式：持续输出手柄原始状态用于诊断。
	if (argc > 1 && (std::string(argv[1]) == "--monitor" || std::string(argv[1]) == "--mon"))
	{
		DWORD test_serial = serial_axis1_handle;
		if (argc > 2)
		{
			test_serial = static_cast<DWORD>(std::strtoul(argv[2], nullptr, 10));
		}

		Handle test_handle(test_serial);
		if (!test_handle.init())
		{
			std::cout << "Handle Init Failed. Serial: " << test_serial << std::endl;
			return 0;
		}

		std::cout << "=== Handle Monitor Mode ===" << std::endl;
		std::cout << "Serial: " << test_serial << std::endl;
		std::cout << "Press ESC or 'q' to exit." << std::endl;

		while (true)
		{
			test_handle.showinfo();

			if (_kbhit())
			{
				const int ch = _getch();
				if (ch == 27 || ch == 'q')
				{
					break;
				}
			}
			Sleep(20);
		}

		std::cout << std::endl;
		test_handle.close();
		return 0;
	}

	const ControlConfig cfg;
	const CylinderPreset cyl;

	const unsigned char axis1_pause_button_mask = cfg.btn_b6;
	const unsigned char axis1_reverse_button_mask = cfg.btn_b0;
	const unsigned char axis6_reverse_button_mask = cfg.btn_b7;
	const unsigned char axis6_independent_button_mask = cfg.btn_b6;
	const unsigned char axis6_cooperative_button_mask = cfg.btn_b0;
	// Handle587（当前项目约定）：
	// - b0: 协同模式请求（常见按钮值 0x07）
	// - b6: 独立模式请求（常见按钮值 0x46）
	// - b7: 反向判定键；在协同模式下与 b0 同时按下时表现为协同反向（常见 0x47）
	// Handle587 在基值 0x06 下的状态：
	// 0x07 -> 进入协同模式
	// 0x47 -> 协同模式反向
	// Handle 582 上 Axis4 点动目标状态：
	// 正向  ~= 0x86（b7 开，b5 关，基值 0x06）
	// 反向  ~= 0x26（b5 开，b7 关，基值 0x06）
	// 释放  ~= 0x06
	// b0 可共存（0x87/0x27），不应阻止点动。
	const unsigned char axis4_buttons_base_mask = 0x06;
	const unsigned char axis4_buttons_forward_mask = cfg.btn_b7;
	const unsigned char axis4_buttons_reverse_mask = cfg.btn_b5;

	// 长生命周期运行时对象。
	Handle handle_axis1(serial_axis1_handle);
	Handle handle_axis6(serial_axis6_handle);
	CADSComm ads;
	HandleFilterState axis1_handle_filter;
	HandleFilterState axis6_handle_filter;

	if (!handle_axis1.init())
	{
		std::cout << "Handle Init Failed. Serial: " << serial_axis1_handle << std::endl;
		return 0;
	}

	if (!handle_axis6.init())
	{
		std::cout << "Handle Init Failed. Serial: " << serial_axis6_handle << std::endl;
		handle_axis1.close();
		return 0;
	}

	Sleep(1000);
	handle_axis1.poll();
	handle_axis6.poll();
	axis1_handle_filter.reset(handle_axis1.fJoints2[0], handle_axis1.fJoints2[1]);
	axis6_handle_filter.reset(handle_axis6.fJoints2[0], handle_axis6.fJoints2[1]);

	if (ads.OpenComm_inside())
	{
		std::cout << "ADS connected: local AMS route, port 851." << std::endl;
	}
	else
	{
		std::cout << "ADS Open Failed (Local). Error: " << ads.GetLastError() << std::endl;
		if (ads.OpenComm())
		{
			std::cout << "ADS connected: remote AMS NetId " << hardcoded_ads_netid << ", port 851." << std::endl;
		}
		else
		{
			std::cout << "ADS Open Failed (Hardcoded). Error: " << ads.GetLastError() << std::endl;
			handle_axis1.close();
			handle_axis6.close();
			return 0;
		}
	}


	// 仅在导管侧手柄输出力反馈。
	auto apply_cmd_force = [&](double cmd_force)
	{
		handle_axis1.setforce_axis(cmd_force * cfg.axial_force_sign, cfg.axial_force_axis, 0.0);
	};

	// PLC 镜像状态数组：在生成新的 refer 帧前会先通过 ADS 刷新。
	double pos[7] = { 0 }; // 上位机本周期目标（将写入 G.refer，坐标系：相对 init_pos）
	double plc_act_pos[7] = { 0 }; // PLC 当前相对位置（G.Act_pos）
	double plc_init_pos[7] = { 0 }; // PLC 相对零点偏置（G.init_pos）
	double plc_leftlimit[7] = { 0 }; // 左限位绝对位置（G.leftlimit）
	double plc_act_pos_from_left[7] = { 0 };
	double plc_refer_from_left[7] = { 0 };
	double plc_v_limit[7] = { 0 };
	bool startup_smoothing_bypass = false;

	// 读取上位机运动环所需的最小状态集合。
	auto read_plc_state = [&]() -> bool
	{
		const char* symbols[] = {
			AdsSymbol::act_pos,
			AdsSymbol::init_pos,
			AdsSymbol::leftlimit
		};
		const unsigned long lengths[] = {
			static_cast<unsigned long>(sizeof(plc_act_pos)),
			static_cast<unsigned long>(sizeof(plc_init_pos)),
			static_cast<unsigned long>(sizeof(plc_leftlimit))
		};
		void* outputs[] = {
			plc_act_pos,
			plc_init_pos,
			plc_leftlimit
		};
		return ads.ADSReadSum(symbols, lengths, outputs, 3);
	};

	auto write_refer = [&]() -> bool
	{
		// 将当前 pos[7] 写回 PLC 参考位数组 G.refer。
		return ads.ADSWrite(AdsSymbol::refer, sizeof(pos), pos);
	};

	auto read_v_limit = [&]() -> bool
	{
		// 读取 PLC 当前速度上限（用于启动准备阶段缩放/恢复）。
		return ads.ADSRead(AdsSymbol::v_limit, sizeof(plc_v_limit), plc_v_limit);
	};

	auto write_v_limit = [&](const double* values) -> bool
	{
		return ads.ADSWrite(AdsSymbol::v_limit, sizeof(plc_v_limit), const_cast<double*>(values));
	};

	auto load_pos_from_actual = [&]()
	{
		// 以当前实际位置为基线重建一帧 refer，避免直接使用旧参考引发跳变。
		copy_positions(plc_act_pos, pos, 7);
	};

	auto from_left_to_abs = [&](int axis_index, double from_left_mm) -> double
	{
		// 左限位参考坐标 -> PLC 绝对坐标。
		return plc_leftlimit[axis_index] + from_left_mm;
	};

	auto from_left_to_rel = [&](int axis_index, double from_left_mm) -> double
	{
		// 左限位参考坐标 -> 上位机相对坐标（refer 使用的坐标系）。
		return from_left_to_abs(axis_index, from_left_mm) - plc_init_pos[axis_index];
	};

	auto read_axis_return_status = [&](const AxisReturnAdsSymbols& symbols, AxisReturnStatus& status) -> bool
	{
		// 轮询 PLC 计划回退命令状态位（Busy/Done/Error/ErrorId）。
		bool ok = true;
		ok = ads.ADSRead(symbols.busy, sizeof(status.busy), &status.busy) && ok;
		ok = ads.ADSRead(symbols.done, sizeof(status.done), &status.done) && ok;
		ok = ads.ADSRead(symbols.error, sizeof(status.error), &status.error) && ok;
		ok = ads.ADSRead(symbols.error_id, sizeof(status.error_id), &status.error_id) && ok;
		return ok;
	};

	auto clear_axis_return_request = [&](const AxisReturnAdsSymbols& symbols) -> bool
	{
		// 清除单轴计划回退触发位 Req。
		bool req = false;
		return ads.ADSWrite(symbols.req, sizeof(req), &req);
	};

	auto request_axis_return = [&](const AxisReturnAdsSymbols& symbols,
		double target_abs,
		double velocity,
		double acc,
		double dec,
		double jerk) -> bool
	{
		// 下发一次计划回退参数并置位 Req：
		// target_abs(mm), velocity(mm/s), acc/dec(mm/s^2), jerk(mm/s^3)。
		bool req = true;
		bool ok = true;
		ok = ads.ADSWrite(symbols.target_abs, sizeof(target_abs), &target_abs) && ok;
		ok = ads.ADSWrite(symbols.velocity, sizeof(velocity), &velocity) && ok;
		ok = ads.ADSWrite(symbols.acc, sizeof(acc), &acc) && ok;
		ok = ads.ADSWrite(symbols.dec, sizeof(dec), &dec) && ok;
		ok = ads.ADSWrite(symbols.jerk, sizeof(jerk), &jerk) && ok;
		ok = ads.ADSWrite(symbols.req, sizeof(req), &req) && ok;
		return ok;
	};

	auto clear_axis1_group_return_requests = [&]() -> bool
	{
		// axis1 相关轴群共用的回退请求清理（1/3/5/6）。
		bool ok = true;
		ok = clear_axis_return_request(AdsSymbol::axis1_return) && ok;
		ok = clear_axis_return_request(AdsSymbol::axis3_return) && ok;
		ok = clear_axis_return_request(AdsSymbol::axis5_return) && ok;
		ok = clear_axis_return_request(AdsSymbol::axis6_return) && ok;
		return ok;
	};

	auto write_axis4_manual_requests = [&](bool forward_req, bool reverse_req) -> bool
	{
		// Axis4 点动请求由 PLC 侧状态机执行，本处仅负责写入请求位。
		bool ok = true;
		ok = ads.ADSWrite(AdsSymbol::axis4_fwd_req, sizeof(forward_req), &forward_req) && ok;
		ok = ads.ADSWrite(AdsSymbol::axis4_rev_req, sizeof(reverse_req), &reverse_req) && ok;
		return ok;
	};

	// 常规导管模式的跟随基准。
	double axis3_base_rel = 0.0;
	double axis5_base_rel = 0.0;
	double axis6_mirror_base_rel = 0.0;
	// 回退流程“进入点/稳定点”缓存（用于回退全流程保持姿态一致）。
	double axis1_return_entry_rel = 0.0;
	double axis1_return_settle_rel = 0.0;
	double axis6_return_entry_rel = 0.0;
	double axis6_return_settle_rel = 0.0;
	// axis6 回退后采用“同方向重入”门控，避免刚回退完就立刻重复触发。
	// axis1 回退期间，镜像轴(3/5)保持在回退触发时刻的值。
	double axis1_return_hold_axis3_rel = 0.0;
	double axis1_return_hold_axis5_rel = 0.0;
	// 导管模式下 axis1 快退联动 axis6 并行快进状态。
	double axis1_fast_entry_abs = 0.0;
	double axis6_fast_entry_abs = 0.0;
	double axis6_coupled_target_abs = 0.0;
	double axis6_coupled_settle_rel = 0.0;
	bool axis6_coupled_active = false;
	bool axis6_coupled_requested = false;
	bool axis6_coupled_done = false;
	bool axis6_coupled_error = false;
	unsigned long axis6_coupled_error_id = 0;
	// 旋转轴保持值（不再使用重接管门控）。
	double axis2_hold_rel = 0.0;
	double axis7_hold_rel = 0.0;
	// 线性增量控制：上一拍滤波值与当前累计目标（绝对坐标）。
	double axis1_prev_linear_filtered = 0.0;
	double axis6_prev_linear_filtered = 0.0;
	double axis1_follow_cmd_abs = 0.0;
	double axis6_follow_cmd_abs = 0.0;
	// 正反切换一次性触发保护：仅在同模式内切换正反时拉起。
	bool axis1_reverse_switch_guard_active = false;
	bool axis6_reverse_switch_guard_active = false;
	// 触发边“入边触发”判定使用的上一拍实际位置。
	double axis1_prev_abs_for_trigger = 0.0;
	double axis6_prev_abs_for_trigger = 0.0;
	bool axis1_prev_abs_valid = false;
	bool axis6_prev_abs_valid = false;

	// 导丝模式缓存：独立模式下冻结导管侧轴位。
	GuidewireMode guidewire_mode = GuidewireMode::None;
	double independent_axis1_hold_rel = 0.0;
	double independent_axis2_hold_rel = 0.0;
	double independent_axis3_hold_rel = 0.0;
	double independent_axis5_hold_rel = 0.0;
	// axis6 窗口锁定（协同/独立切换时保证窗口边界稳定）。
	bool axis6_window_locked = false;
	double axis6_locked_window_start_abs = 0.0;
	double axis6_locked_window_end_abs = 0.0;
	// 协同模式 axis6 增量叠加状态。
	bool axis6_coop_ff_inited = false;
	double axis6_coop_prev_axis1_cmd_abs = 0.0;

	StartupState startup;
	ForceFeedbackState ff;
	ForceLogState force_log;
	CrawlState axis1_crawl;
	CrawlState axis6_crawl;
	axis1_crawl.enabled = true;

	// Axis1 使用固定左限位参考窗口；Axis6 动态捕获其窗口。
	auto axis1_window_left_abs = [&]() -> double
	{
		return from_left_to_abs(0, cfg.axis1_window_left_from_left_mm);
	};

	auto axis1_window_right_abs = [&]() -> double
	{
		return from_left_to_abs(0, cfg.axis1_window_right_from_left_mm);
	};

	// 设置 axis6 独立/协同通用窗口：
	// window_right_abs 为窗口右端，左端按窗口长度回推，并自动钳制到左限位。
	auto set_axis6_independent_window = [&](double window_right_abs, bool log_clamp) -> bool
	{
		const double requested_left_abs = window_right_abs - cfg.axis6_independent_window_size_mm;
		const double clamped_left_abs = (requested_left_abs < plc_leftlimit[5]) ? plc_leftlimit[5] : requested_left_abs;

		axis6_crawl.start_abs = clamped_left_abs;
		axis6_crawl.end_abs = window_right_abs;

		if ((axis6_crawl.end_abs - axis6_crawl.start_abs) < cfg.crawl_arrive_tol_mm)
		{
			if (log_clamp)
			{
				std::cout << "Guidewire mode switch ignored: axis6 independent window is too close to left limit." << std::endl;
			}
			return false;
		}

		if (log_clamp && clamped_left_abs > requested_left_abs)
		{
			std::cout << "Axis6 independent window clamped by left limit." << std::endl;
		}

		return true;
	};

	// 锁定当前 axis6 窗口边界，避免模式切换期间窗口漂移。
	auto lock_axis6_window_from_current = [&]()
	{
		axis6_window_locked = true;
		axis6_locked_window_start_abs = axis6_crawl.start_abs;
		axis6_locked_window_end_abs = axis6_crawl.end_abs;
	};

	// 恢复已锁定的 axis6 窗口边界。
	auto apply_locked_axis6_window = [&]()
	{
		axis6_crawl.start_abs = axis6_locked_window_start_abs;
		axis6_crawl.end_abs = axis6_locked_window_end_abs;
	};

	// 导丝窗口覆盖修正：
	// 当 axis6 远端相对左限位的位置与 axis5 相对左限位的位置过近时，
	// 认为窗口被覆盖，按 axis5 的 from-left 值重建 axis6 两端点。
	auto rebuild_axis6_window_if_covered = [&](const char* reason, bool log_result) -> bool
	{
		const double axis6_start_from_left_mm = axis6_crawl.start_abs - plc_leftlimit[5];
		const double axis6_end_from_left_mm = axis6_crawl.end_abs - plc_leftlimit[5];
		const double axis6_far_from_left_mm =
			(axis6_start_from_left_mm > axis6_end_from_left_mm)
			? axis6_start_from_left_mm
			: axis6_end_from_left_mm;
		const double axis5_from_left_mm =
			(plc_act_pos[4] + plc_init_pos[4]) - plc_leftlimit[4];
		const double delta_mm = axis6_far_from_left_mm - axis5_from_left_mm;
		bool rebuilt = false;

		if (delta_mm < cfg.axis6_window_cover_threshold_mm)
		{
			const double rebuilt_window_left_abs = plc_leftlimit[5] + axis5_from_left_mm;
			const double rebuilt_window_right_abs =
				rebuilt_window_left_abs + cfg.axis6_independent_window_size_mm;
			if (!set_axis6_independent_window(rebuilt_window_right_abs, true))
			{
				return false;
			}
			lock_axis6_window_from_current();
			rebuilt = true;
		}

		if (log_result)
		{
			std::cout
				<< "Axis6 window cover check(" << reason << "): far_from_left="
				<< axis6_far_from_left_mm
				<< " mm, axis5_from_left=" << axis5_from_left_mm
				<< " mm, delta=" << delta_mm
				<< " mm, rebuilt=" << (rebuilt ? "YES" : "NO")
				<< std::endl;
		}
		return true;
	};

	// 仅在模式边沿或一次爬行循环完成后，重同步导管侧爬行状态。
	auto sync_axis1 = [&](int samples, bool wait_rearm, int rearm_dir) -> bool
	{
		(void)wait_rearm;
		(void)rearm_dir;
		const double preserved_axis2_hold_rel = axis2_hold_rel;
		clear_axis1_group_return_requests();

		if (!read_plc_state())
		{
			return false;
		}

		load_pos_from_actual();
		pos[1] = preserved_axis2_hold_rel;
		pos[6] = axis7_hold_rel;

		get_average_handle_pose(handle_axis1, samples, axis1_crawl.handle_ref, axis1_crawl.rot_ref);
		axis1_handle_filter.reset(axis1_crawl.handle_ref, axis1_crawl.rot_ref);
		axis1_prev_linear_filtered = axis1_handle_filter.axis0_filtered;
		axis1_crawl.base_rel = plc_act_pos[0];
		axis1_crawl.rot_base_rel = preserved_axis2_hold_rel;
		axis1_follow_cmd_abs = plc_act_pos[0] + plc_init_pos[0];
		axis1_crawl.start_abs = axis1_window_left_abs();
		axis1_crawl.end_abs = axis1_window_right_abs();
		axis1_crawl.target_abs = axis1_crawl.end_abs;
		axis1_crawl.plc_move_requested = false;
		axis1_crawl.window_active = is_within_range(
			plc_act_pos[0] + plc_init_pos[0],
			axis1_crawl.min_abs(),
		axis1_crawl.max_abs(),
			cfg.crawl_arrive_tol_mm);
		axis1_crawl.phase = CrawlState::Phase::Follow;
		axis1_crawl.phase_t0 = GetTickCount();
		axis1_crawl.wait_rearm = false;
		axis1_crawl.rearm_dir = 0;

		axis2_hold_rel = preserved_axis2_hold_rel;
		axis1_reverse_switch_guard_active = false;
		axis1_prev_abs_for_trigger = plc_act_pos[0] + plc_init_pos[0];
		axis1_prev_abs_valid = true;

		axis3_base_rel = plc_act_pos[2];
		axis5_base_rel = plc_act_pos[4];
		axis6_mirror_base_rel = plc_act_pos[5];
		axis6_coupled_active = false;
		axis6_coupled_requested = false;
		axis6_coupled_done = false;
		axis6_coupled_error = false;
		axis6_coupled_error_id = 0;

		return write_refer();
	};

	// 仅重同步导丝侧爬行状态。进入独立模式时可选择基于
	// 当前 axis6 绝对位置重建窗口。
	auto sync_axis6 = [&](int samples,
		bool capture_window,
		bool wait_rearm,
		int rearm_dir,
		bool check_window_cover,
		bool log_window_cover) -> bool
	{
		(void)wait_rearm;
		(void)rearm_dir;
		const double preserved_axis7_hold_rel = axis7_hold_rel;
		clear_axis_return_request(AdsSymbol::axis6_return);

		if (!read_plc_state())
		{
			return false;
		}

		load_pos_from_actual();
		pos[1] = axis2_hold_rel;
		pos[6] = preserved_axis7_hold_rel;

		get_average_handle_pose(handle_axis6, samples, axis6_crawl.handle_ref, axis6_crawl.rot_ref);
		axis6_handle_filter.reset(axis6_crawl.handle_ref, axis6_crawl.rot_ref);
		axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;
		axis6_crawl.base_rel = plc_act_pos[5];
		axis6_crawl.rot_base_rel = preserved_axis7_hold_rel;
		axis6_follow_cmd_abs = plc_act_pos[5] + plc_init_pos[5];
		if (capture_window || !axis6_crawl.enabled)
		{
			if (!axis6_window_locked)
			{
				if (!set_axis6_independent_window(plc_act_pos[5] + plc_init_pos[5], capture_window))
				{
					return false;
				}
				lock_axis6_window_from_current();
			}
			else
			{
				apply_locked_axis6_window();
			}
		}
		else if (axis6_window_locked)
		{
			apply_locked_axis6_window();
		}
		if (check_window_cover)
		{
			if (!rebuild_axis6_window_if_covered("sync_axis6", log_window_cover))
			{
				return false;
			}
		}
		axis6_crawl.target_abs = axis6_crawl.end_abs;
		axis6_crawl.plc_move_requested = false;
		axis6_crawl.window_active = is_within_range(
			plc_act_pos[5] + plc_init_pos[5],
			axis6_crawl.min_abs(),
		axis6_crawl.max_abs(),
			cfg.crawl_arrive_tol_mm);
		axis6_crawl.phase = CrawlState::Phase::Follow;
		axis6_crawl.phase_t0 = GetTickCount();
		axis6_crawl.wait_rearm = false;
		axis6_crawl.rearm_dir = 0;
		axis6_crawl.enabled = true;
		axis6_coop_ff_inited = false;
		axis6_coop_prev_axis1_cmd_abs = 0.0;

		axis7_hold_rel = preserved_axis7_hold_rel;
		axis6_reverse_switch_guard_active = false;
		axis6_prev_abs_for_trigger = plc_act_pos[5] + plc_init_pos[5];
		axis6_prev_abs_valid = true;

		return write_refer();
	};

	// 全局重同步会将两个手柄及全部镜像基准对齐到当前 PLC 位置。
	auto sync_all = [&](int samples) -> bool
	{
		double preserved_axis2_hold_rel = axis2_hold_rel;
		double preserved_axis7_hold_rel = axis7_hold_rel;
		clear_axis1_group_return_requests();
		clear_axis_return_request(AdsSymbol::axis6_return);
		write_axis4_manual_requests(false, false);

		if (!read_plc_state())
		{
			return false;
		}
		// 使用“当前轴实际位置”重置旋转保持基准，避免复用陈旧 hold 值导致意外回零。
		preserved_axis2_hold_rel = plc_act_pos[1];
		preserved_axis7_hold_rel = plc_act_pos[6];

		load_pos_from_actual();
		pos[1] = preserved_axis2_hold_rel;
		pos[6] = preserved_axis7_hold_rel;
		if (!write_refer())
		{
			return false;
		}

		get_average_dual_pos(
			handle_axis1,
			handle_axis6,
			samples,
			axis1_crawl.handle_ref,
			axis1_crawl.rot_ref,
			axis6_crawl.handle_ref,
			axis6_crawl.rot_ref);
		axis1_handle_filter.reset(axis1_crawl.handle_ref, axis1_crawl.rot_ref);
		axis6_handle_filter.reset(axis6_crawl.handle_ref, axis6_crawl.rot_ref);
		axis1_prev_linear_filtered = axis1_handle_filter.axis0_filtered;
		axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;

		if (!read_plc_state())
		{
			return false;
		}

		load_pos_from_actual();
		pos[1] = preserved_axis2_hold_rel;
		pos[6] = preserved_axis7_hold_rel;
		if (!write_refer())
		{
			return false;
		}

		axis1_crawl.base_rel = plc_act_pos[0];
		axis1_crawl.rot_base_rel = preserved_axis2_hold_rel;
		axis1_crawl.start_abs = axis1_window_left_abs();
		axis1_crawl.end_abs = axis1_window_right_abs();
		axis1_crawl.target_abs = axis1_crawl.end_abs;
		axis1_crawl.plc_move_requested = false;
		axis1_crawl.window_active = is_within_range(
			plc_act_pos[0] + plc_init_pos[0],
			axis1_crawl.min_abs(),
			axis1_crawl.max_abs(),
			cfg.crawl_arrive_tol_mm);
		axis1_crawl.phase = CrawlState::Phase::Follow;
		axis1_crawl.phase_t0 = GetTickCount();
		axis1_crawl.wait_rearm = false;
		axis1_crawl.rearm_dir = 0;
		axis1_crawl.enabled = true;
		axis1_follow_cmd_abs = plc_act_pos[0] + plc_init_pos[0];

		axis2_hold_rel = preserved_axis2_hold_rel;
		axis1_reverse_switch_guard_active = false;
		axis1_prev_abs_for_trigger = axis1_follow_cmd_abs;
		axis1_prev_abs_valid = true;

		axis3_base_rel = plc_act_pos[2];
		axis5_base_rel = plc_act_pos[4];
		axis6_mirror_base_rel = plc_act_pos[5];

		axis6_crawl.base_rel = plc_act_pos[5];
		axis6_crawl.rot_base_rel = preserved_axis7_hold_rel;
		axis6_crawl.start_abs = plc_act_pos[5] + plc_init_pos[5];
		axis6_crawl.end_abs = plc_act_pos[5] + plc_init_pos[5];
		axis6_crawl.target_abs = axis6_crawl.end_abs;
		axis6_crawl.plc_move_requested = false;
		axis6_crawl.window_active = false;
		axis6_crawl.phase = CrawlState::Phase::Follow;
		axis6_crawl.phase_t0 = GetTickCount();
		axis6_crawl.wait_rearm = false;
		axis6_crawl.rearm_dir = 0;
		axis6_crawl.enabled = false;
		axis6_window_locked = false;
		axis6_coop_ff_inited = false;
		axis6_coop_prev_axis1_cmd_abs = 0.0;
		axis6_follow_cmd_abs = plc_act_pos[5] + plc_init_pos[5];
		axis6_reverse_switch_guard_active = false;
		axis6_prev_abs_for_trigger = axis6_follow_cmd_abs;
		axis6_prev_abs_valid = true;
		axis6_coupled_active = false;
		axis6_coupled_requested = false;
		axis6_coupled_done = false;
		axis6_coupled_error = false;
		axis6_coupled_error_id = 0;

		axis7_hold_rel = preserved_axis7_hold_rel;

		return true;
	};

	auto clear_plc_reinit_req = [&]()
	{
		bool clear_val = false;
		ads.ADSWrite(AdsSymbol::handle_reinit_req, sizeof(clear_val), &clear_val);
	};

	// 当 axis1 首次进入爬行窗口时，刷新常规模式跟随基准。
	auto capture_axis1_follow_baseline = [&]()
	{
		axis1_crawl.handle_ref = axis1_handle_filter.axis0_filtered;
		axis1_crawl.rot_ref = axis1_handle_filter.axis1_filtered;
		axis1_crawl.base_rel = plc_act_pos[0];
		axis1_crawl.rot_base_rel = axis2_hold_rel;
		axis1_follow_cmd_abs = plc_act_pos[0] + plc_init_pos[0];
		axis1_crawl.plc_move_requested = false;
		axis3_base_rel = plc_act_pos[2];
		axis5_base_rel = plc_act_pos[4];
		if (!axis6_crawl.enabled)
		{
			axis6_mirror_base_rel = plc_act_pos[5];
		}
	};

	auto apply_axis1_mirror_from_abs = [&](double axis1_abs_cmd, bool include_axis6)
	{
		const double axis1_delta_rel = axis1_abs_cmd - plc_init_pos[0] - axis1_crawl.base_rel;
		pos[2] = axis3_base_rel + axis1_delta_rel;
		pos[4] = axis5_base_rel + axis1_delta_rel;
		if (include_axis6)
		{
			pos[5] = axis6_mirror_base_rel + axis1_delta_rel;
		}
	};

	// 进入导丝独立模式时冻结导管侧各轴在当前
	// 相对位置；Axis7 保持可控并重同步到当前手柄扭转量。
	auto enter_independent_guidewire_mode = [&]() -> bool
	{
		const double preserved_axis7_hold_rel = axis7_hold_rel;

		if (!read_plc_state())
		{
			return false;
		}

		load_pos_from_actual();
		independent_axis1_hold_rel = plc_act_pos[0];
		independent_axis2_hold_rel = axis2_hold_rel;
		independent_axis3_hold_rel = plc_act_pos[2];
		independent_axis5_hold_rel = plc_act_pos[4];

		axis7_hold_rel = preserved_axis7_hold_rel;

		get_average_handle_pose(handle_axis6, 20, axis6_crawl.handle_ref, axis6_crawl.rot_ref);
		axis6_handle_filter.reset(axis6_crawl.handle_ref, axis6_crawl.rot_ref);
		axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;
		axis6_crawl.base_rel = plc_act_pos[5];
		axis6_crawl.rot_base_rel = axis7_hold_rel;
		axis6_follow_cmd_abs = plc_act_pos[5] + plc_init_pos[5];
		if (!axis6_window_locked)
		{
			if (!set_axis6_independent_window(plc_act_pos[5] + plc_init_pos[5], true))
			{
				return false;
			}
			lock_axis6_window_from_current();
		}
		else
		{
			apply_locked_axis6_window();
		}
		if (!rebuild_axis6_window_if_covered("enter_independent", true))
		{
			return false;
		}
		axis6_crawl.target_abs = axis6_crawl.end_abs;
		axis6_crawl.plc_move_requested = false;
		axis6_crawl.phase = CrawlState::Phase::Follow;
		axis6_crawl.phase_t0 = GetTickCount();
		axis6_crawl.wait_rearm = false;
		axis6_crawl.rearm_dir = 0;
		axis6_crawl.window_active = is_within_range(
			plc_act_pos[5] + plc_init_pos[5],
			axis6_crawl.min_abs(),
		axis6_crawl.max_abs(),
		cfg.crawl_arrive_tol_mm);
		axis6_crawl.enabled = true;
		axis6_coop_ff_inited = false;
		axis6_coop_prev_axis1_cmd_abs = 0.0;
		axis6_reverse_switch_guard_active = false;
		axis6_prev_abs_for_trigger = axis6_follow_cmd_abs;
		axis6_prev_abs_valid = true;

		pos[0] = plc_act_pos[0];
		pos[1] = axis2_hold_rel;
		pos[2] = plc_act_pos[2];
		pos[4] = plc_act_pos[4];
		pos[5] = plc_act_pos[5];
		pos[6] = axis7_hold_rel;
		return write_refer();
	};

	// 协同模式复用 axis6 爬行状态机，同时保持导管轴链路激活。
	// 协同模式入口：复用 axis6 的窗口化爬行状态机，不冻结导管轴链路。
	auto enter_cooperative_guidewire_mode = [&]() -> bool
	{
		return sync_axis6(20, true, false, 0, true, true);
	};

	// 导丝模式入模门限检查：
	// 仅在“尝试进入模式”瞬间检查 |axis6-leftlimit| < max，不在运行中强制退出。
	auto check_axis6_guidewire_entry_gate = [&](double& axis6_from_left_mm) -> bool
	{
		if (!read_plc_state())
		{
			return false;
		}
		const double axis6_abs = plc_act_pos[5] + plc_init_pos[5];
		axis6_from_left_mm = std::abs(axis6_abs - plc_leftlimit[5]);
		return axis6_from_left_mm < cfg.guidewire_entry_axis6_from_left_max_mm;
	};

	auto exit_guidewire_mode_to_normal = [&]() -> bool
	{
		guidewire_mode = GuidewireMode::None;
		axis6_window_locked = false;
		axis6_coop_ff_inited = false;
		axis6_coop_prev_axis1_cmd_abs = 0.0;
		// 退出导丝时用当前实际值刷新旋转保持位，避免沿用陈旧 hold 导致 axis2 偶发回零。
		if (!read_plc_state())
		{
			return false;
		}
		axis2_hold_rel = plc_act_pos[1];
		axis7_hold_rel = plc_act_pos[6];
		axis1_reverse_switch_guard_active = false;
		axis6_reverse_switch_guard_active = false;
		return sync_axis1(20, false, 0);
	};

	// 启动准备是叠加在 PLC 自检之上的上位机流程。
	// 它会临时降低 3/5/6 轴的 v_limit，并在完成后恢复。
	// 启动准备流程（上位机侧）：
	// 在 PLC 自检完成后执行，期间会临时缩放 3/5/6 速度上限并在结束时恢复。
	auto start_startup_sequence = [&]() -> bool
	{
		if (!read_plc_state())
		{
			return false;
		}

		load_pos_from_actual();
		startup.axis1_hold_rel = plc_act_pos[0];
		startup.axis2_hold_rel = axis2_hold_rel;
		startup.axis3_hold_rel = plc_act_pos[2];
		startup.axis5_hold_rel = plc_act_pos[4];
		startup.axis6_hold_rel = plc_act_pos[5];
		startup.axis7_hold_rel = axis7_hold_rel;

		pos[1] = axis2_hold_rel;
		pos[6] = axis7_hold_rel;

		if (!startup.v_limit_scaled)
		{
			if (!read_v_limit())
			{
				return false;
			}

			copy_positions(plc_v_limit, startup.v_limit_backup, 7);
			double scaled_v_limit[7] = { 0 };
			copy_positions(plc_v_limit, scaled_v_limit, 7);
			scaled_v_limit[2] *= cfg.startup_motion_speed_scale;
			scaled_v_limit[4] *= cfg.startup_motion_speed_scale;
			scaled_v_limit[5] *= cfg.startup_motion_speed_scale;
			if (!write_v_limit(scaled_v_limit))
			{
				return false;
			}
			startup.v_limit_scaled = true;
		}

		guidewire_mode = GuidewireMode::None;
		axis6_crawl.enabled = false;
		axis6_window_locked = false;
		axis6_coop_ff_inited = false;
		axis6_coop_prev_axis1_cmd_abs = 0.0;
		startup.phase = StartupPhase::ReleaseClamps;
		startup.phase_t0 = GetTickCount();
		startup.completed = false;
		startup.prompted = false;

		if (!write_refer())
		{
			if (startup.v_limit_scaled)
			{
				write_v_limit(startup.v_limit_backup);
				startup.v_limit_scaled = false;
			}
			return false;
		}

		return true;
	};

	// 在启动完成或启动中断后恢复 PLC 运动限值。
	auto restore_startup_v_limit = [&]() -> bool
	{
		if (!startup.v_limit_scaled)
		{
			return true;
		}

		if (!write_v_limit(startup.v_limit_backup))
		{
			return false;
		}

		startup.v_limit_scaled = false;
		return true;
	};

	auto prompt_startup_mode = [&]()
	{
		if (!startup.prompted)
		{
			std::cout << "Startup mode pending: press C for direct control, or S to run startup preparation first." << std::endl;
			startup.prompted = true;
		}
	};

	// 在交互循环开始前，用初始 PLC 快照初始化各保持位姿。
	if (!read_plc_state())
	{
		std::cout << "Failed to read PLC state." << std::endl;
		handle_axis1.close();
		handle_axis6.close();
		return 0;
	}

	load_pos_from_actual();
	axis2_hold_rel = plc_act_pos[1];
	axis7_hold_rel = plc_act_pos[6];
	axis1_follow_cmd_abs = plc_act_pos[0] + plc_init_pos[0];
	axis6_follow_cmd_abs = plc_act_pos[5] + plc_init_pos[5];
	axis1_prev_linear_filtered = axis1_handle_filter.axis0_filtered;
	axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;
	axis1_prev_abs_for_trigger = axis1_follow_cmd_abs;
	axis6_prev_abs_for_trigger = axis6_follow_cmd_abs;
	axis1_prev_abs_valid = true;
	axis6_prev_abs_valid = true;
	write_refer();

	bool self_check_done = true;
	bool has_self_check_flag = ads.ADSRead(AdsSymbol::self_check_done, sizeof(self_check_done), &self_check_done);

	bool control_active = !has_self_check_flag || self_check_done;
	bool last_self_check_done = self_check_done;
	bool handle_reinit_req = false;
	bool estop_hold_req = false;
	bool estop_hold_active = false;
	bool axis1_push_rearm_after_hold = false;
	bool axis1_delivery_stop_latched = false;
	bool axis1_delivery_stop_prompted = false;
	bool freeze_active = false;
	bool pause_pressed_prev = false;
	bool axis1_reverse_pressed_prev = false;
	bool axis6_effective_reverse_prev = false;
	int axis4_jog_state_prev = 0;
	bool axis4_manual_busy_prev = false;
	bool axis4_manual_error_prev = false;
	unsigned long axis4_manual_error_id_prev = 0;
	GuidewireMode requested_guidewire_mode_prev = GuidewireMode::None;
	bool axis1_fast_return = false; // 轴1快退旁路标志（写入 G.axis1_fast_return）
	bool axis6_fast_retract = false; // 轴6快退旁路标志（写入 G.axis6_fast_retract）
	AxisReturnStatus axis1_return_status;
	AxisReturnStatus axis6_return_status;
	int loop_count = 0;
	DWORD force_log_warn_last_ms = 0;
	startup_smoothing_bypass = false;
	ads.ADSWrite(AdsSymbol::startup_smoothing_bypass, sizeof(startup_smoothing_bypass), &startup_smoothing_bypass);

	unsigned char last_btn_axis1 = 0xFF;
	unsigned char last_btn_axis6 = 0xFF;

	std::cout << "Force feedback: OFF (press F to toggle)" << std::endl;
	apply_cmd_force(0.0);

	// 力感记录默认随 main 启动；文件名包含日期与 24 小时制时间（到秒）。
	force_log.enabled = true;
	force_log.period_ms = cfg.force_log_period_ms;
	const std::string force_log_filename = build_force_log_filename();
	if (force_log.open_file(force_log_filename))
	{
		std::cout << "Force sensor logging: ON -> " << force_log_filename
			<< " (period_ms=" << force_log.period_ms << ")" << std::endl;
	}
	else
	{
		std::cout << "Force sensor logging: OFF (file open failed)." << std::endl;
	}

	if (!has_self_check_flag || self_check_done)
	{
		if (sync_all(30))
		{
			control_active = false;
			startup.phase = StartupPhase::WaitForEnter;
			startup.completed = false;
			startup.prompted = false;
			prompt_startup_mode();
		}
	}

	while (true)
	{
		// 1) 采样两个手柄，并生成按键边沿触发状态。
		handle_axis1.poll();
		handle_axis6.poll();
		axis1_handle_filter.update(
			handle_axis1.fJoints2[0],
			handle_axis1.fJoints2[1],
			cfg.linear_handle_alpha,
			cfg.rotational_handle_alpha);
		axis6_handle_filter.update(
			handle_axis6.fJoints2[0],
			handle_axis6.fJoints2[1],
			cfg.linear_handle_alpha,
			cfg.rotational_handle_alpha);

		++loop_count;
		axis1_fast_return = false; // 每周期先清零，仅在快退状态机阶段置 TRUE
		axis6_fast_retract = false; // 每周期先清零，仅在快退状态机阶段置 TRUE

		// from-left 观测量仅用于监测/门控，不需要每帧刷新。
		// 降频到每 5 帧读取一次，减少 ADS 通信负担。
		if ((loop_count % 5) == 0)
		{
			const char* from_left_symbols[] = {
				AdsSymbol::act_pos_from_left,
				AdsSymbol::refer_from_left
			};
			const unsigned long from_left_lengths[] = {
				static_cast<unsigned long>(sizeof(plc_act_pos_from_left)),
				static_cast<unsigned long>(sizeof(plc_refer_from_left))
			};
			void* from_left_outputs[] = {
				plc_act_pos_from_left,
				plc_refer_from_left
			};
			(void)ads.ADSReadSum(from_left_symbols, from_left_lengths, from_left_outputs, 2);
		}

		const unsigned char axis1_buttons = handle_axis1.buttons2;
		const bool pause_pressed = (axis1_buttons & axis1_pause_button_mask) != 0;
		const bool axis1_reverse_pressed = (axis1_buttons & axis1_reverse_button_mask) != 0;
		const bool axis6_reverse_pressed = (handle_axis6.buttons2 & axis6_reverse_button_mask) != 0;
		const bool guidewire_independent_pressed = (handle_axis6.buttons2 & axis6_independent_button_mask) != 0;
		const bool guidewire_cooperative_pressed = (handle_axis6.buttons2 & axis6_cooperative_button_mask) != 0;
		const bool guidewire_forced_reverse_pressed = guidewire_independent_pressed && guidewire_cooperative_pressed;
		const bool axis4_base_pressed = (axis1_buttons & axis4_buttons_base_mask) == axis4_buttons_base_mask;
		const bool axis4_forward_pressed =
			axis4_base_pressed &&
			((axis1_buttons & axis4_buttons_forward_mask) != 0) &&
			((axis1_buttons & axis4_buttons_reverse_mask) == 0);
		const bool axis4_reverse_pressed =
			axis4_base_pressed &&
			((axis1_buttons & axis4_buttons_reverse_mask) != 0) &&
			((axis1_buttons & axis4_buttons_forward_mask) == 0);
		const int axis4_jog_state = axis4_forward_pressed ? 1 : (axis4_reverse_pressed ? -1 : 0);

		// 导丝模式请求解码（587）：
		// - 仅 b6: 独立模式
		// - 仅 b0: 协同模式
		// - b0+b6: 按当前约定仍归入协同模式
		GuidewireMode requested_guidewire_mode = GuidewireMode::None;
		if (!guidewire_cooperative_pressed && guidewire_independent_pressed)
		{
			requested_guidewire_mode = GuidewireMode::Independent;
		}
		else if (guidewire_cooperative_pressed && !guidewire_independent_pressed)
		{
			requested_guidewire_mode = GuidewireMode::Cooperative;
		}
		else if (guidewire_cooperative_pressed && guidewire_independent_pressed)
		{
			requested_guidewire_mode = GuidewireMode::Cooperative;
		}
		// 反向有效键按模式解释：
		// - 协同：b0+b7 触发反向
		// - 独立：使用 b7 作为反向键
		const bool axis6_effective_reverse_pressed =
			((requested_guidewire_mode == GuidewireMode::Cooperative) && guidewire_forced_reverse_pressed) ||
			((requested_guidewire_mode == GuidewireMode::Independent) && axis6_reverse_pressed);
		const bool startup_sequence_active = startup.is_active();
		const bool axis4_jog_allowed = !freeze_active && !estop_hold_active && !startup_sequence_active;
		const bool axis4_forward_request = axis4_jog_allowed && axis4_forward_pressed;
		const bool axis4_reverse_request = axis4_jog_allowed && axis4_reverse_pressed;
		if (axis4_jog_state != axis4_jog_state_prev)
		{
			std::cout << "Axis4 jog: "
				<< ((axis4_jog_state > 0) ? "FORWARD" : ((axis4_jog_state < 0) ? "REVERSE" : "STOP"))
				<< std::endl;
			axis4_jog_state_prev = axis4_jog_state;
		}

		if (pause_pressed && !pause_pressed_prev)
		{
			freeze_active = true;
			control_active = false;
			apply_cmd_force(0.0);
			std::cout << "Handle582 pause: ON." << std::endl;
		}
		else if (!pause_pressed && pause_pressed_prev)
		{
			freeze_active = false;
			if (startup_sequence_active)
			{
				std::cout << "Handle582 pause: OFF, startup sequence resumed." << std::endl;
			}
			else if (!startup.completed)
			{
				if (!estop_hold_active && sync_all(20))
				{
					control_active = false;
					if (!startup.prompted && (!has_self_check_flag || self_check_done))
					{
						prompt_startup_mode();
					}
					std::cout << "Handle582 pause: OFF, waiting for startup mode selection." << std::endl;
				}
				else if (estop_hold_active)
				{
					std::cout << "Handle582 pause released, waiting for PLC hold to clear." << std::endl;
				}
				else
				{
					std::cout << "Handle582 pause released, resync pending." << std::endl;
				}
			}
			else if (!estop_hold_active && sync_all(20))
			{
				control_active = true;
				std::cout << "Handle582 pause: OFF, control resumed." << std::endl;
			}
			else if (estop_hold_active)
			{
				std::cout << "Handle582 pause released, waiting for PLC hold to clear." << std::endl;
			}
			else
			{
				std::cout << "Handle582 pause released, resync pending." << std::endl;
			}
		}
		pause_pressed_prev = pause_pressed;

		// 2) 正反键切换：启用一次性触发保护，不再执行线性重同步。
		if (!freeze_active && !estop_hold_active && !startup_sequence_active && control_active)
		{
			if (guidewire_mode == GuidewireMode::None && axis1_reverse_pressed != axis1_reverse_pressed_prev)
			{
				const double axis1_abs_now_for_guard = plc_act_pos[0] + plc_init_pos[0];
				const double axis1_new_trigger_edge_abs =
					axis1_reverse_pressed ? axis1_window_right_abs() : axis1_window_left_abs();
				axis1_reverse_switch_guard_active =
					(std::abs(axis1_abs_now_for_guard - axis1_new_trigger_edge_abs) <= cfg.reverse_switch_trigger_guard_mm);
				if (axis1_reverse_switch_guard_active)
				{
					std::cout << "Axis1 catheter retract mode: "
						<< (axis1_reverse_pressed ? "ON" : "OFF")
						<< " (trigger guard armed)." << std::endl;
				}
				else
				{
					std::cout << "Axis1 catheter retract mode: " << (axis1_reverse_pressed ? "ON" : "OFF") << std::endl;
				}
			}

			if ((guidewire_mode == GuidewireMode::Independent || guidewire_mode == GuidewireMode::Cooperative) &&
				(requested_guidewire_mode == guidewire_mode) &&
				axis6_effective_reverse_pressed != axis6_effective_reverse_prev)
			{
				const double axis6_abs_now_for_guard = plc_act_pos[5] + plc_init_pos[5];
				const double axis6_new_trigger_edge_abs =
					axis6_effective_reverse_pressed ? axis6_crawl.end_abs : axis6_crawl.start_abs;
				axis6_reverse_switch_guard_active =
					(std::abs(axis6_abs_now_for_guard - axis6_new_trigger_edge_abs) <= cfg.reverse_switch_trigger_guard_mm);
				if (axis6_reverse_switch_guard_active)
				{
					std::cout << "Axis6 reverse crawl mode: "
						<< (axis6_effective_reverse_pressed ? "ON" : "OFF")
						<< " (trigger guard armed)." << std::endl;
				}
				else
				{
					std::cout << "Axis6 reverse crawl mode: " << (axis6_effective_reverse_pressed ? "ON" : "OFF") << std::endl;
				}
			}
		}
		axis1_reverse_pressed_prev = axis1_reverse_pressed;
		axis6_effective_reverse_prev = axis6_effective_reverse_pressed;

		// 3) 轮询 PLC 侧保持状态，并在保持期间禁用上位机力输出。
		if ((loop_count % 10) == 0)
		{
			if (ads.ADSRead(AdsSymbol::estop_hold_req, sizeof(estop_hold_req), &estop_hold_req))
			{
				if (estop_hold_req)
				{
					if (!estop_hold_active)
					{
						std::cout << "PLC hold: ON" << std::endl;
					}
					estop_hold_active = true;
					control_active = false;
					apply_cmd_force(0.0);
				}
				else
				{
					if (estop_hold_active)
					{
						std::cout << "PLC hold: OFF" << std::endl;
						axis1_push_rearm_after_hold = true;
						std::cout << "Axis1 push locked after hold; pull handle back to re-arm." << std::endl;
					}
					estop_hold_active = false;
				}
			}
		}

		if (freeze_active)
		{
			ff.clear_output();
			apply_cmd_force(0.0);
		}

		// 4) 键盘侧通道：选择直接控制 / 启动准备 / 力反馈开关。
		if (_kbhit())
		{
			const int ch = _getch();
			if (ch == 'c' || ch == 'C')
			{
				if (!startup.completed && startup.phase == StartupPhase::WaitForEnter)
				{
					if (freeze_active)
					{
						std::cout << "Direct control start ignored: Handle582 pause is active." << std::endl;
					}
					else if (estop_hold_active)
					{
						std::cout << "Direct control start ignored: PLC hold is active." << std::endl;
					}
					else if (has_self_check_flag && !self_check_done)
					{
						std::cout << "Direct control start ignored: PLC self check is not done yet." << std::endl;
					}
					else if (!restore_startup_v_limit())
					{
						std::cout << "Direct control start failed: unable to restore startup v_limit." << std::endl;
					}
					else if (sync_all(20))
					{
						startup.phase = StartupPhase::Done;
						startup.completed = true;
						startup.prompted = false;
						control_active = true;
						std::cout << "Direct control started." << std::endl;
					}
					else
					{
						std::cout << "Direct control start failed: ADS sync failed." << std::endl;
					}
				}
			}
			else if (ch == 's' || ch == 'S')
			{
				if (!startup.completed && startup.phase == StartupPhase::WaitForEnter)
				{
					if (freeze_active)
					{
						std::cout << "Startup sequence start ignored: Handle582 pause is active." << std::endl;
					}
					else if (estop_hold_active)
					{
						std::cout << "Startup sequence start ignored: PLC hold is active." << std::endl;
					}
					else if (has_self_check_flag && !self_check_done)
					{
						std::cout << "Startup sequence start ignored: PLC self check is not done yet." << std::endl;
					}
					else if (start_startup_sequence())
					{
						control_active = false;
						std::cout << "Startup preparation sequence started." << std::endl;
					}
					else
					{
						std::cout << "Startup preparation sequence start failed: ADS sync failed." << std::endl;
					}
				}
			}
			else if (ch == '\r')
			{
				if (!startup.completed && startup.phase == StartupPhase::WaitForEnter)
				{
					prompt_startup_mode();
				}
			}
			else if (ch == 'f' || ch == 'F')
			{
				ff.enabled = !ff.enabled;
				ff.reset();
				std::cout << "Force feedback: " << (ff.enabled ? "ON" : "OFF") << std::endl;
				if (!ff.enabled)
				{
					apply_cmd_force(0.0);
				}
			}
			else if (ch == 0 || ch == 224)
			{
				_getch();
			}
		}

		// 5) 导丝模式由 handle 587 的边沿触发（协同=b0，独立=b6），并在切换时重同步。
		if (requested_guidewire_mode != requested_guidewire_mode_prev)
		{
			if (requested_guidewire_mode == GuidewireMode::None)
			{
				if (guidewire_mode != GuidewireMode::None)
				{
					if (!freeze_active && !estop_hold_active && !startup_sequence_active)
					{
						if (exit_guidewire_mode_to_normal())
						{
							std::cout << "Guidewire mode: OFF." << std::endl;
						}
						else
						{
							guidewire_mode = GuidewireMode::None;
							axis6_crawl.enabled = false;
							axis6_window_locked = false;
							axis6_coop_ff_inited = false;
							axis6_coop_prev_axis1_cmd_abs = 0.0;
							std::cout << "Guidewire mode exit failed: ADS sync failed." << std::endl;
						}
					}
					else
					{
						guidewire_mode = GuidewireMode::None;
						axis6_crawl.enabled = false;
						axis6_window_locked = false;
						axis6_coop_ff_inited = false;
						axis6_coop_prev_axis1_cmd_abs = 0.0;
					}
				}
			}
			else if (freeze_active)
			{
				std::cout << "Guidewire mode switch ignored: Handle582 pause is active." << std::endl;
			}
			else if (estop_hold_active)
			{
				std::cout << "Guidewire mode switch ignored: PLC hold is active." << std::endl;
			}
			else if (!startup.completed || startup.phase != StartupPhase::Done)
			{
				std::cout << "Guidewire mode switch ignored: startup sequence is not completed yet." << std::endl;
			}
			else if (!control_active)
			{
				std::cout << "Guidewire mode switch ignored: control is not active yet." << std::endl;
			}
			else
			{
				bool mode_ok = false;
				bool mode_attempted = false;
				double axis6_from_left_mm = 0.0;
				const bool gate_checked = check_axis6_guidewire_entry_gate(axis6_from_left_mm);
				if (!gate_checked)
				{
					std::cout << "Guidewire mode switch failed: unable to read PLC state for axis6 entry gate." << std::endl;
				}
				else if (axis6_from_left_mm >= cfg.guidewire_entry_axis6_from_left_max_mm)
				{
					std::cout
						<< "Guidewire mode switch ignored: axis6 distance from left limit = "
						<< axis6_from_left_mm
						<< " mm, required < "
						<< cfg.guidewire_entry_axis6_from_left_max_mm
						<< " mm."
						<< std::endl;
				}
				else
				{
					mode_attempted = true;
					if (requested_guidewire_mode == GuidewireMode::Independent)
					{
						mode_ok = enter_independent_guidewire_mode();
						if (mode_ok)
						{
							guidewire_mode = GuidewireMode::Independent;
							std::cout << "Guidewire mode: INDEPENDENT." << std::endl;
						}
					}
					else if (requested_guidewire_mode == GuidewireMode::Cooperative)
					{
						mode_ok = enter_cooperative_guidewire_mode();
						if (mode_ok)
						{
							guidewire_mode = GuidewireMode::Cooperative;
							if (guidewire_forced_reverse_pressed)
							{
								std::cout << "Guidewire mode: COOPERATIVE + REVERSE." << std::endl;
							}
							else
							{
								std::cout << "Guidewire mode: COOPERATIVE." << std::endl;
							}
						}
					}
				}

				if (mode_attempted && !mode_ok)
				{
					std::cout << "Guidewire mode switch failed." << std::endl;
				}
			}
		}
		requested_guidewire_mode_prev = requested_guidewire_mode;

		// 6) 周期性响应 PLC 自检完成与 PLC 请求的重同步。
		if (has_self_check_flag && (loop_count % 50) == 0)
		{
			if (ads.ADSRead(AdsSymbol::self_check_done, sizeof(self_check_done), &self_check_done))
			{
				if (!last_self_check_done && self_check_done)
				{
					if (!restore_startup_v_limit())
					{
						std::cout << "Warning: failed to restore startup v_limit after PLC self check transition." << std::endl;
					}
					guidewire_mode = GuidewireMode::None;
					axis6_crawl.enabled = false;
					axis6_window_locked = false;
					axis6_coop_ff_inited = false;
					axis6_coop_prev_axis1_cmd_abs = 0.0;
					startup.phase = StartupPhase::WaitForEnter;
					startup.completed = false;
					startup.prompted = false;
					if (!freeze_active && !estop_hold_active && sync_all(30))
					{
						control_active = false;
						std::cout << "PLC self check completed." << std::endl;
						prompt_startup_mode();
					}
					else
					{
						control_active = false;
					}
				}
				last_self_check_done = self_check_done;
			}

			if (ads.ADSRead(AdsSymbol::handle_reinit_req, sizeof(handle_reinit_req), &handle_reinit_req))
			{
				if (handle_reinit_req)
				{
					if (!freeze_active && !estop_hold_active && !startup.is_active())
					{
						if (sync_all(30))
						{
							control_active = startup.completed && (startup.phase == StartupPhase::Done);
							if (!startup.completed && (!has_self_check_flag || self_check_done) && !startup.prompted)
							{
								prompt_startup_mode();
							}
						}
					}
					clear_plc_reinit_req();
				}
			}
		}

		// 7) 每次 buttons2 变化时输出可读的按键诊断信息。
		if (handle_axis1.buttons2 != last_btn_axis1)
		{
			std::cout << "Handle582 Btns: 0x" << std::hex << static_cast<int>(handle_axis1.buttons2) << std::dec << std::endl;
			last_btn_axis1 = handle_axis1.buttons2;
		}

		if (handle_axis6.buttons2 != last_btn_axis6)
		{
			std::cout << "Handle587 Btns: 0x" << std::hex << static_cast<int>(handle_axis6.buttons2) << std::dec << std::endl;
			last_btn_axis6 = handle_axis6.buttons2;
		}

		process_force_feedback(
			ff,
			ads,
			handle_axis1,
			control_active,
			freeze_active,
			estop_hold_active,
			loop_count,
			cfg.axial_force_axis,
			cfg.axial_force_sign);

		// 8) 当启动已完成但控制未激活时，通过全量重同步恢复。
		const bool motion_startup_active = startup.is_active();
		startup_smoothing_bypass = motion_startup_active;
		if (!control_active && !motion_startup_active && !freeze_active && !estop_hold_active && startup.completed)
		{
			if (sync_all(20))
			{
				std::cout << "Re-synced" << std::endl;
				control_active = true;
			}
		}
		else if (!startup.completed &&
				 !motion_startup_active &&
				 !freeze_active &&
				 !estop_hold_active &&
				 !startup.prompted &&
				 (!has_self_check_flag || self_check_done))
		{
			prompt_startup_mode();
		}

		unsigned short cylinder1_cmd = cyl.cyl1_open;
		unsigned short cylinder2_cmd = cyl.cyl2_clamp;
		unsigned short cylinder3_cmd = cyl.cyl3_follow_release;
		unsigned short cylinder4_cmd = cyl.cyl4_follow_release;
		bool axis4_manual_forward_req = axis4_reverse_request;
		bool axis4_manual_reverse_req = axis4_forward_request;

		// 9) 根据当前顶层模式构建一帧 refer 和一组气缸指令。
		if (!freeze_active && !estop_hold_active && (control_active || motion_startup_active) && read_plc_state())
		{
			load_pos_from_actual();
			pos[1] = axis2_hold_rel;
			pos[6] = axis7_hold_rel;

			const DWORD now_ms = GetTickCount();
			const double axis1_linear_filtered = axis1_handle_filter.axis0_filtered;
			const double axis1_rot_filtered = axis1_handle_filter.axis1_filtered;
			const double axis6_linear_filtered = axis6_handle_filter.axis0_filtered;
			const double axis6_rot_filtered = axis6_handle_filter.axis1_filtered;

			const double axis1_abs = plc_act_pos[0] + plc_init_pos[0]; // 轴1绝对位置(mm)
			const double axis3_abs = plc_act_pos[2] + plc_init_pos[2];
			const double axis1_linear_increment_raw_mm =
				(axis1_linear_filtered - axis1_prev_linear_filtered) * cfg.k_handle_to_mm * cfg.axis_push_sign;
			const double axis1_linear_increment_mm =
				(std::abs(axis1_linear_increment_raw_mm) >= cfg.linear_increment_noise_deadband_mm)
				? axis1_linear_increment_raw_mm
				: 0.0;
			const bool axis1_linear_increment_active = std::abs(axis1_linear_increment_mm) > 0.0;
			const double axis1_window_left_abs_now = axis1_crawl.start_abs;
			const double axis1_window_right_abs_now = axis1_crawl.end_abs;
			const double axis1_min_abs = axis1_crawl.min_abs();
			const double axis1_max_abs = axis1_crawl.max_abs();
			const double axis3_from_left_mm = axis3_abs - plc_leftlimit[2];
			const bool axis3_delivery_stop_active =
				axis3_from_left_mm <= (cfg.axis3_delivery_stop_from_left_mm + cfg.crawl_arrive_tol_mm);

			const double axis6_abs = plc_act_pos[5] + plc_init_pos[5]; // 轴6绝对位置(mm)
			const double axis6_linear_increment_raw_mm =
				(axis6_linear_filtered - axis6_prev_linear_filtered) * cfg.k_handle_to_mm * cfg.axis_push_sign;
			const double axis6_linear_increment_mm =
				(std::abs(axis6_linear_increment_raw_mm) >= cfg.linear_increment_noise_deadband_mm)
				? axis6_linear_increment_raw_mm
				: 0.0;
			const bool axis6_linear_increment_active = std::abs(axis6_linear_increment_mm) > 0.0;
			const double axis6_window_left_abs_now = axis6_crawl.start_abs;
			const double axis6_window_right_abs_now = axis6_crawl.end_abs;

			auto hold_axis1_mirror_axes_for_return = [&]()
			{
				pos[2] = axis1_return_hold_axis3_rel;
				pos[4] = axis1_return_hold_axis5_rel;
			};

			auto compute_axis7_cmd_rel = [&]() -> double
			{
				const double axis7_follow_rel =
					axis6_crawl.rot_base_rel + (axis6_rot_filtered - axis6_crawl.rot_ref) * cfg.axis_rot_scale_deg;
				axis7_hold_rel = axis7_follow_rel;
				return axis7_follow_rel;
			};

			const double axis7_cmd_rel =
				(guidewire_mode == GuidewireMode::None) ? axis7_hold_rel : compute_axis7_cmd_rel();
			pos[6] = axis7_cmd_rel;

			// 独立与协同导丝模式共用的 axis6 爬行状态机（增量式输入）。
			// 参数说明：
			// - axis6_raw_cmd_abs: 本拍按增量累加后的 axis6 绝对目标（未做最终触发处理）
			// - axis6_increment_mm: 本拍 axis6 线性有效增量（mm）
			// - axis6_reverse_mode: 当前是否处于反向爬行判定
			// - axis6_user_increment_active: 587 本人线性通道是否存在有效增量
			// - require_user_increment_for_trigger: 是否要求“触发反弹必须有 587 本人有效增量”
			auto run_axis6_crawl_state = [&](double axis6_raw_cmd_abs,
				double axis6_increment_mm,
				bool axis6_reverse_mode,
				bool axis6_user_increment_active,
				bool require_user_increment_for_trigger)
			{
				if (axis6_crawl.phase == CrawlState::Phase::Follow)
				{
					double axis6_cmd_abs = axis6_follow_cmd_abs;
					const bool axis6_increment_active = std::abs(axis6_increment_mm) > 0.0;
					if (axis6_increment_active)
					{
						axis6_cmd_abs = clamp_double(axis6_raw_cmd_abs, axis6_window_left_abs_now, axis6_window_right_abs_now);
					}

					pos[5] = axis6_cmd_abs - plc_init_pos[5];
					axis6_follow_cmd_abs = axis6_cmd_abs;
					cylinder3_cmd = cyl.cyl3_open;
					cylinder4_cmd = cyl.cyl4_clamp;

					const double axis6_trigger_edge_abs =
						axis6_reverse_mode ? axis6_window_right_abs_now : axis6_window_left_abs_now;
					if (axis6_reverse_switch_guard_active &&
						(std::abs(axis6_abs - axis6_trigger_edge_abs) > cfg.reverse_switch_trigger_guard_mm))
					{
						axis6_reverse_switch_guard_active = false;
					}
					const bool axis6_switch_guard_blocked =
						axis6_reverse_switch_guard_active &&
						(std::abs(axis6_abs - axis6_trigger_edge_abs) <= cfg.reverse_switch_trigger_guard_mm);

					const bool axis6_toward_trigger =
						axis6_reverse_mode ? (axis6_increment_mm > 0.0) : (axis6_increment_mm < 0.0);
					const bool axis6_trigger_user_ok =
						(!require_user_increment_for_trigger) || axis6_user_increment_active;
					const double axis6_prev_abs = axis6_prev_abs_valid ? axis6_prev_abs_for_trigger : axis6_abs;
					const bool axis6_enter_trigger_edge =
						(std::abs(axis6_abs - axis6_trigger_edge_abs) <= cfg.crawl_arrive_tol_mm) &&
						(std::abs(axis6_prev_abs - axis6_trigger_edge_abs) > cfg.crawl_arrive_tol_mm);
					const bool axis6_ready_to_trigger =
						axis6_trigger_user_ok &&
						axis6_increment_active &&
						axis6_toward_trigger &&
						axis6_enter_trigger_edge &&
						!axis6_switch_guard_blocked;
					if (axis6_ready_to_trigger)
					{
						axis6_crawl.target_abs = axis6_reverse_mode
							? axis6_window_left_abs_now
							: axis6_window_right_abs_now;
						axis6_return_entry_rel = plc_act_pos[5];
						axis6_crawl.phase = CrawlState::Phase::SwitchWait;
						axis6_crawl.phase_t0 = now_ms;
						axis6_crawl.rearm_dir = 0;
						axis6_crawl.plc_move_requested = false;
						cylinder3_cmd = cyl.cyl3_clamp;
						cylinder4_cmd = cyl.cyl4_open;
					}
				}
				else if (axis6_crawl.phase == CrawlState::Phase::SwitchWait)
				{
					// 兼容旧状态，直接并入并行快退阶段。
					axis6_fast_retract = true;
					pos[5] = axis6_return_entry_rel;
					cylinder3_cmd = cyl.cyl3_clamp;
					cylinder4_cmd = cyl.cyl4_open;
					if ((now_ms - axis6_crawl.phase_t0) >= cfg.axis6_pre_move_cylinder_wait_ms)
					{
						axis6_crawl.phase = CrawlState::Phase::FastMove;
						axis6_crawl.phase_t0 = now_ms;
						axis6_crawl.plc_move_requested = false;
					}
				}
				else if (axis6_crawl.phase == CrawlState::Phase::FastMove)
				{
					pos[5] = axis6_crawl.target_abs - plc_init_pos[5];
					cylinder3_cmd = cyl.cyl3_clamp;
					cylinder4_cmd = cyl.cyl4_open;
					axis6_fast_retract = true;

					if (!axis6_crawl.plc_move_requested)
					{
						if (request_axis_return(
							AdsSymbol::axis6_return,
							axis6_crawl.target_abs,
							cfg.axis6_return_velocity_mm_s,
							cfg.axis6_return_acc_mm_s2,
							cfg.axis6_return_dec_mm_s2,
							cfg.axis6_return_jerk_mm_s3))
						{
							axis6_crawl.plc_move_requested = true;
						}
					}
					else if (read_axis_return_status(AdsSymbol::axis6_return, axis6_return_status))
					{
						if (axis6_return_status.error)
						{
							clear_axis_return_request(AdsSymbol::axis6_return);
							axis6_crawl.plc_move_requested = false;
							std::cout << "Axis6 planned return error: " << axis6_return_status.error_id << std::endl;
							if (!sync_axis6(3, false, false, 0, false, false))
							{
								axis6_crawl.phase = CrawlState::Phase::Follow;
							}
						}
						else if (axis6_return_status.done)
						{
							clear_axis_return_request(AdsSymbol::axis6_return);
							axis6_crawl.plc_move_requested = false;
							axis6_return_settle_rel = axis6_crawl.target_abs - plc_init_pos[5];
							axis6_crawl.phase = CrawlState::Phase::RestoreWait;
							axis6_crawl.phase_t0 = now_ms;
						}
					}
				}
				else if (axis6_crawl.phase == CrawlState::Phase::RestoreWait)
				{
					axis6_fast_retract = true;
					pos[5] = axis6_return_settle_rel;
					cylinder3_cmd = cyl.cyl3_open;
					cylinder4_cmd = cyl.cyl4_clamp;
					if ((now_ms - axis6_crawl.phase_t0) >= cfg.axis6_post_return_cylinder_wait_ms)
					{
						if (!sync_axis6(3, false, false, 0, false, false))
						{
							std::cout << "Axis6 resync after planned return failed." << std::endl;
							axis6_crawl.phase = CrawlState::Phase::Follow;
						}
					}
				}
			};

			if (motion_startup_active)
			{
				// 启动序列默认固定 axis2/7；axis1 在阶段2会先走到左限位参考准备点。
				pos[0] = startup.axis1_hold_rel;
				pos[1] = startup.axis2_hold_rel;
				pos[2] = startup.axis3_hold_rel;
				pos[4] = startup.axis5_hold_rel;
				pos[5] = startup.axis6_hold_rel;
				pos[6] = startup.axis7_hold_rel;

				const double startup_axis1_ready_abs = from_left_to_abs(0, cfg.startup_axis1_ready_from_left_mm);
				const double startup_axis5_ready_abs = from_left_to_abs(4, cfg.startup_axis5_ready_from_left_mm);
				const double startup_axis6_ready_abs = from_left_to_abs(5, cfg.startup_axis5_ready_from_left_mm + cfg.axis56_ready_gap_mm);
				const double startup_axis3_ready_abs = from_left_to_abs(2, cfg.startup_axis3_ready_from_left_mm);
				const double axis1_abs = plc_act_pos[0] + plc_init_pos[0];
				const double axis5_abs = plc_act_pos[4] + plc_init_pos[4];
				const double axis6_abs_now = plc_act_pos[5] + plc_init_pos[5];
				const double axis3_abs = plc_act_pos[2] + plc_init_pos[2];

				if (startup.phase == StartupPhase::ReleaseClamps)
				{
					// 阶段 1：打开全部夹爪并等待机构稳定。
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_open;
					cylinder3_cmd = cfg.startup_cyl3_open;
					cylinder4_cmd = cfg.startup_cyl4_open;
					if ((now_ms - startup.phase_t0) >= cfg.startup_clamp_settle_delay_ms)
					{
						startup.phase = StartupPhase::MoveAxis56ToLeftReady;
					}
				}
				else if (startup.phase == StartupPhase::MoveAxis56ToLeftReady)
				{
					// 阶段 2：将 1/5/6 轴同步移动到左限位参考的准备点。
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_open;
					cylinder3_cmd = cfg.startup_cyl3_open;
					cylinder4_cmd = cfg.startup_cyl4_open;
					pos[0] = from_left_to_rel(0, cfg.startup_axis1_ready_from_left_mm);
					pos[4] = from_left_to_rel(4, cfg.startup_axis5_ready_from_left_mm);
					pos[5] = from_left_to_rel(5, cfg.startup_axis5_ready_from_left_mm + cfg.axis56_ready_gap_mm);
					if ((std::abs(axis1_abs - startup_axis1_ready_abs) <= cfg.crawl_arrive_tol_mm) &&
						(std::abs(axis5_abs - startup_axis5_ready_abs) <= cfg.crawl_arrive_tol_mm) &&
						(std::abs(axis6_abs_now - startup_axis6_ready_abs) <= cfg.crawl_arrive_tol_mm))
					{
						// 锁存 axis1 阶段2到位值，避免后续阶段回到旧 hold 点。
						startup.axis1_hold_rel = pos[0];
						startup.phase = StartupPhase::ClampCylinder34Wait;
						startup.phase_t0 = now_ms;
					}
				}
				else if (startup.phase == StartupPhase::ClampCylinder34Wait)
				{
					// 阶段 3：在 3/5/6 联动前先闭合导丝侧夹爪对。
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_open;
					cylinder3_cmd = cfg.startup_cyl3_clamp;
					cylinder4_cmd = cfg.startup_cyl4_clamp;
					pos[4] = from_left_to_rel(4, cfg.startup_axis5_ready_from_left_mm);
					pos[5] = from_left_to_rel(5, cfg.startup_axis5_ready_from_left_mm + cfg.axis56_ready_gap_mm);
					if ((now_ms - startup.phase_t0) >= cfg.startup_clamp_settle_delay_ms)
					{
						startup.axis3_move_base_rel = plc_act_pos[2];
						startup.axis5_move_base_rel = plc_act_pos[4];
						startup.axis6_move_base_rel = plc_act_pos[5];
						startup.phase = StartupPhase::MoveAxis356BackToReady;
					}
				}
				else if (startup.phase == StartupPhase::MoveAxis356BackToReady)
				{
					// 阶段 4：axis3 移动到准备点，同时 5/6 轴跟随相同相对增量。
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_open;
					cylinder3_cmd = cfg.startup_cyl3_clamp;
					cylinder4_cmd = cfg.startup_cyl4_clamp;
					const double axis3_target_rel = from_left_to_rel(2, cfg.startup_axis3_ready_from_left_mm);
					const double axis356_delta_rel = axis3_target_rel - startup.axis3_move_base_rel;
					pos[2] = axis3_target_rel;
					pos[4] = startup.axis5_move_base_rel + axis356_delta_rel;
					pos[5] = startup.axis6_move_base_rel + axis356_delta_rel;
					const double axis3_remaining_mm = std::abs(axis3_abs - startup_axis3_ready_abs);
					const double cyl2_clamp_trigger_mm =
						(cfg.startup_axis3_cyl2_clamp_advance_mm > cfg.crawl_arrive_tol_mm)
						? cfg.startup_axis3_cyl2_clamp_advance_mm
						: cfg.crawl_arrive_tol_mm;
					if (axis3_remaining_mm <= cyl2_clamp_trigger_mm)
					{
						startup.phase = StartupPhase::ClampCylinder2AfterAxis3;
						startup.phase_t0 = now_ms;
					}
				}
				else if (startup.phase == StartupPhase::ClampCylinder2AfterAxis3)
				{
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_clamp;
					cylinder3_cmd = cfg.startup_cyl3_clamp;
					cylinder4_cmd = cfg.startup_cyl4_clamp;
					pos[2] = from_left_to_rel(2, cfg.startup_axis3_ready_from_left_mm);
					pos[4] = startup.axis5_move_base_rel + (pos[2] - startup.axis3_move_base_rel);
					pos[5] = startup.axis6_move_base_rel + (pos[2] - startup.axis3_move_base_rel);
					if ((now_ms - startup.phase_t0) >= cfg.startup_clamp_settle_delay_ms)
					{
						if (!restore_startup_v_limit())
						{
							std::cout << "Warning: failed to restore startup v_limit after startup sequence." << std::endl;
						}
						startup.phase = StartupPhase::Done;
						startup.completed = true;
						if (sync_all(30))
						{
							control_active = true;
							std::cout << "Startup preparation sequence completed." << std::endl;
						}
						else
						{
							control_active = false;
							std::cout << "Startup preparation sequence completed, but resync failed." << std::endl;
						}
					}
				}
			}
			else if (guidewire_mode == GuidewireMode::Independent)
			{
				// 导丝独立模式会冻结导管侧各轴，仅运行 axis6/7。
				pos[0] = independent_axis1_hold_rel;
				pos[1] = independent_axis2_hold_rel;
				pos[2] = independent_axis3_hold_rel;
				pos[4] = independent_axis5_hold_rel;
				pos[6] = axis7_cmd_rel;

				axis6_coop_ff_inited = false;
				const double axis6_raw_cmd_abs = axis6_follow_cmd_abs + axis6_linear_increment_mm;
				run_axis6_crawl_state(
					axis6_raw_cmd_abs,
					axis6_linear_increment_mm,
					axis6_effective_reverse_pressed,
					axis6_linear_increment_active,
					false);
			}
			else
			{
				// 常规导管模式：
				// - axis1/2 由 handle 582 控制
				// - 轴 3/5 在 Follow 阶段镜像 axis1 平移
				// - axis6 在导管模式 Follow 阶段保持不动
				// - axis1 触发快退时，axis6 按等距并行快进
				const bool cooperative_mode = (guidewire_mode == GuidewireMode::Cooperative);
				const bool axis1_now_in_window = is_within_range(axis1_abs, axis1_min_abs, axis1_max_abs, cfg.crawl_arrive_tol_mm);
				if (!axis1_crawl.window_active && axis1_now_in_window)
				{
					capture_axis1_follow_baseline();
					axis1_crawl.window_active = true;
					std::cout << "Axis1 entered crawl window; crawl logic enabled." << std::endl;
				}

				if (axis1_crawl.phase == CrawlState::Phase::Follow)
				{
					const double axis1_raw_cmd_abs = axis1_follow_cmd_abs + axis1_linear_increment_mm;
					const bool axis1_follow_enabled = axis1_reverse_pressed || (!axis1_push_rearm_after_hold && !axis1_delivery_stop_latched);
					if (axis3_from_left_mm >
						(cfg.axis3_delivery_stop_from_left_mm + cfg.axis3_delivery_release_hysteresis_mm))
					{
						axis1_delivery_stop_prompted = false;
					}

					if (axis1_push_rearm_after_hold && axis1_linear_increment_mm > 0.0)
					{
						axis1_push_rearm_after_hold = false;
						capture_axis1_follow_baseline();
						std::cout << "Axis1 push re-armed after hold." << std::endl;
					}
					if (axis1_delivery_stop_latched && axis1_reverse_pressed && axis1_linear_increment_active)
					{
						axis1_delivery_stop_latched = false;
						axis1_delivery_stop_prompted = false;
					}

					double axis1_cmd_abs = axis1_follow_cmd_abs;
					if (axis1_linear_increment_active && axis1_follow_enabled)
					{
						axis1_cmd_abs = axis1_crawl.window_active
							? clamp_double(axis1_raw_cmd_abs, axis1_window_left_abs_now, axis1_window_right_abs_now)
							: axis1_raw_cmd_abs;
					}
					if (axis1_crawl.window_active)
					{
						axis1_cmd_abs = clamp_double(axis1_cmd_abs, axis1_window_left_abs_now, axis1_window_right_abs_now);
					}
					if (!axis1_reverse_pressed && axis1_cmd_abs < axis1_window_left_abs_now)
					{
						axis1_cmd_abs = axis1_window_left_abs_now;
					}
					if (axis1_delivery_stop_latched && !axis1_reverse_pressed)
					{
						axis1_cmd_abs = axis1_window_left_abs_now;
					}

					pos[0] = axis1_cmd_abs - plc_init_pos[0]; // 绝对目标 -> refer相对坐标（相对 init_pos）
					axis1_follow_cmd_abs = axis1_cmd_abs;

					pos[1] = axis1_crawl.rot_base_rel +
						(axis1_rot_filtered - axis1_crawl.rot_ref) * cfg.axis_rot_scale_deg;
					axis2_hold_rel = pos[1];

					apply_axis1_mirror_from_abs(axis1_cmd_abs, false);
					if (!cooperative_mode)
					{
						// 导管模式下 axis6 不再随 axis1 跟随，保持在导管基准位。
						pos[5] = axis6_mirror_base_rel;
						cylinder3_cmd = cyl.cyl3_open;
						cylinder4_cmd = cyl.cyl4_clamp;
					}
					else
					{
						cylinder3_cmd = cyl.cyl3_follow_release;
						cylinder4_cmd = cyl.cyl4_follow_release;
					}

					if (axis1_crawl.window_active)
					{
						const double axis1_trigger_edge_abs = axis1_reverse_pressed
							? axis1_window_right_abs_now
							: axis1_window_left_abs_now;
						if (axis1_reverse_switch_guard_active &&
							(std::abs(axis1_abs - axis1_trigger_edge_abs) > cfg.reverse_switch_trigger_guard_mm))
						{
							axis1_reverse_switch_guard_active = false;
						}
						const bool axis1_switch_guard_blocked =
							axis1_reverse_switch_guard_active &&
							(std::abs(axis1_abs - axis1_trigger_edge_abs) <= cfg.reverse_switch_trigger_guard_mm);
						const bool axis1_toward_trigger =
							axis1_reverse_pressed ? (axis1_linear_increment_mm > 0.0) : (axis1_linear_increment_mm < 0.0);
						const double axis1_prev_abs = axis1_prev_abs_valid ? axis1_prev_abs_for_trigger : axis1_abs;
						const bool axis1_enter_trigger_edge =
							(std::abs(axis1_abs - axis1_trigger_edge_abs) <= cfg.crawl_arrive_tol_mm) &&
							(std::abs(axis1_prev_abs - axis1_trigger_edge_abs) > cfg.crawl_arrive_tol_mm);
						const bool axis1_ready_to_trigger =
							axis1_linear_increment_active &&
							axis1_toward_trigger &&
							axis1_follow_enabled &&
							axis1_enter_trigger_edge &&
							!axis1_switch_guard_blocked;
						if (axis1_ready_to_trigger)
						{
							if (!axis1_reverse_pressed && axis3_delivery_stop_active)
							{
								axis1_delivery_stop_latched = true;
								if (!axis1_delivery_stop_prompted)
								{
									std::cout << "Delivery reached: axis3 <= 20mm from left limit. Enable reverse mode to continue." << std::endl;
									axis1_delivery_stop_prompted = true;
								}
							}
							else
							{
								axis1_crawl.target_abs = axis1_reverse_pressed
									? axis1_window_left_abs_now
									: axis1_window_right_abs_now;
								axis1_return_entry_rel = plc_act_pos[0];
								axis1_return_settle_rel = plc_act_pos[0];
								axis1_fast_entry_abs = axis1_abs;
								axis6_fast_entry_abs = axis6_abs;
								axis1_return_hold_axis3_rel = plc_act_pos[2];
								axis1_return_hold_axis5_rel = plc_act_pos[4];
								axis6_coupled_settle_rel = plc_act_pos[5];
								if (!cooperative_mode)
								{
									// 导管模式：axis6 联动方向与 axis1 快退位移取反（按工艺要求镜像反向）。
									axis6_coupled_target_abs =
										axis6_fast_entry_abs - (axis1_crawl.target_abs - axis1_fast_entry_abs);
									axis6_return_entry_rel = plc_act_pos[5];
									axis6_coupled_active = true;
									axis6_coupled_requested = false;
									axis6_coupled_done = false;
									axis6_coupled_error = false;
									axis6_coupled_error_id = 0;
								}
								else
								{
									axis6_coupled_active = false;
									axis6_coupled_requested = false;
									axis6_coupled_done = false;
									axis6_coupled_error = false;
									axis6_coupled_error_id = 0;
								}
								axis1_crawl.phase = CrawlState::Phase::SwitchWait;
								axis1_crawl.phase_t0 = now_ms;
								axis1_crawl.rearm_dir = 0;
								axis1_crawl.plc_move_requested = false;
								cylinder1_cmd = cyl.cyl1_clamp;
								cylinder2_cmd = cyl.cyl2_open;
								if (!cooperative_mode)
								{
									cylinder3_cmd = cyl.cyl3_clamp;
									cylinder4_cmd = cyl.cyl4_open;
								}
							}
						}
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::SwitchWait)
				{
					const DWORD coupled_pre_move_wait_ms =
						axis6_coupled_active
						? ((cfg.axis1_pre_move_cylinder_wait_ms > cfg.axis6_pre_move_cylinder_wait_ms)
							? cfg.axis1_pre_move_cylinder_wait_ms
							: cfg.axis6_pre_move_cylinder_wait_ms)
						: cfg.axis1_pre_move_cylinder_wait_ms;
					axis1_fast_return = true;
					pos[0] = axis1_return_entry_rel;
					hold_axis1_mirror_axes_for_return();
					pos[1] = axis2_hold_rel;
					cylinder1_cmd = cyl.cyl1_clamp;
					cylinder2_cmd = cyl.cyl2_open;
					if (axis6_coupled_active)
					{
						axis6_fast_retract = true;
						pos[5] = axis6_return_entry_rel;
						cylinder3_cmd = cyl.cyl3_clamp;
						cylinder4_cmd = cyl.cyl4_open;
					}
					if ((now_ms - axis1_crawl.phase_t0) >= coupled_pre_move_wait_ms)
					{
						axis1_crawl.phase = CrawlState::Phase::FastMove;
						axis1_crawl.phase_t0 = now_ms;
						axis1_crawl.plc_move_requested = false;
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::FastMove)
				{
					pos[0] = axis1_crawl.target_abs - plc_init_pos[0];
					hold_axis1_mirror_axes_for_return();
					pos[1] = axis2_hold_rel;
					cylinder1_cmd = cyl.cyl1_clamp;
					cylinder2_cmd = cyl.cyl2_open;
					axis1_fast_return = true;
					if (axis6_coupled_active)
					{
						axis6_fast_retract = true;
						// 回退联动阶段持续覆盖为“联动目标点”，避免接管前后端点注入。
						pos[5] = axis6_coupled_target_abs - plc_init_pos[5];
						cylinder3_cmd = cyl.cyl3_clamp;
						cylinder4_cmd = cyl.cyl4_open;
					}
					if (!axis1_crawl.plc_move_requested)
					{
						const bool request_ok = request_axis_return(
							AdsSymbol::axis1_return,
							axis1_crawl.target_abs,
							cfg.axis1_return_velocity_mm_s,
							cfg.axis1_return_acc_mm_s2,
							cfg.axis1_return_dec_mm_s2,
							cfg.axis1_return_jerk_mm_s3);
						if (request_ok)
						{
							axis1_crawl.plc_move_requested = true;
						}
						else
						{
							clear_axis_return_request(AdsSymbol::axis1_return);
						}
					}
					if (axis6_coupled_active &&
						axis1_crawl.plc_move_requested &&
						!axis6_coupled_requested &&
						!axis6_coupled_error)
					{
						if (request_axis_return(
							AdsSymbol::axis6_return,
							axis6_coupled_target_abs,
							cfg.axis1_return_velocity_mm_s,
							cfg.axis1_return_acc_mm_s2,
							cfg.axis1_return_dec_mm_s2,
							cfg.axis1_return_jerk_mm_s3))
						{
							axis6_coupled_requested = true;
						}
						else
						{
							clear_axis_return_request(AdsSymbol::axis6_return);
							axis6_coupled_error = true;
							axis6_coupled_error_id = 0;
						}
					}
					if (axis6_coupled_active && axis6_coupled_requested && !axis6_coupled_done && !axis6_coupled_error)
					{
						if (read_axis_return_status(AdsSymbol::axis6_return, axis6_return_status))
						{
							if (axis6_return_status.error)
							{
								clear_axis_return_request(AdsSymbol::axis6_return);
								axis6_coupled_error = true;
								axis6_coupled_error_id = axis6_return_status.error_id;
							}
							else if (axis6_return_status.done)
							{
								clear_axis_return_request(AdsSymbol::axis6_return);
								axis6_coupled_done = true;
								axis6_coupled_settle_rel = axis6_coupled_target_abs - plc_init_pos[5];
							}
						}
					}
					if (axis1_crawl.plc_move_requested &&
						read_axis_return_status(AdsSymbol::axis1_return, axis1_return_status))
					{
						if (axis1_return_status.error)
						{
							clear_axis_return_request(AdsSymbol::axis1_return);
							axis1_crawl.plc_move_requested = false;
							std::cout
								<< "Axis1 planned return error: "
								<< axis1_return_status.error_id
								<< std::endl;
							clear_axis_return_request(AdsSymbol::axis6_return);
							axis6_coupled_active = false;
							axis6_coupled_requested = false;
							axis6_coupled_done = false;
							axis6_coupled_error = false;
							axis6_coupled_error_id = 0;
							if (!sync_axis1(3, false, 0))
							{
								axis1_crawl.phase = CrawlState::Phase::Follow;
							}
						}
						else if (axis1_return_status.done)
						{
							if (axis6_coupled_active)
							{
								if (axis6_coupled_error)
								{
									std::cout
										<< "Axis6 coupled fast-forward error: "
										<< axis6_coupled_error_id
										<< std::endl;
									clear_axis_return_request(AdsSymbol::axis1_return);
									clear_axis_return_request(AdsSymbol::axis6_return);
									axis1_crawl.plc_move_requested = false;
									axis6_coupled_active = false;
									axis6_coupled_requested = false;
									axis6_coupled_done = false;
									axis6_coupled_error = false;
									axis6_coupled_error_id = 0;
									if (!sync_axis1(3, false, 0))
									{
										axis1_crawl.phase = CrawlState::Phase::Follow;
									}
								}
								else if (axis6_coupled_done)
								{
									clear_axis_return_request(AdsSymbol::axis1_return);
									axis1_crawl.plc_move_requested = false;
									axis1_return_settle_rel = axis1_crawl.target_abs - plc_init_pos[0];
									axis6_return_settle_rel = axis6_coupled_settle_rel;
									axis1_crawl.phase = CrawlState::Phase::RestoreWait;
									axis1_crawl.phase_t0 = now_ms;
								}
							}
							else
							{
								clear_axis_return_request(AdsSymbol::axis1_return);
								axis1_crawl.plc_move_requested = false;
								axis1_return_settle_rel = axis1_crawl.target_abs - plc_init_pos[0];
								axis1_crawl.phase = CrawlState::Phase::RestoreWait;
								axis1_crawl.phase_t0 = now_ms;
							}
						}
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::RestoreWait)
				{
					const DWORD coupled_post_return_wait_ms =
						axis6_coupled_active
						? ((cfg.axis1_post_return_cylinder_wait_ms > cfg.axis6_post_return_cylinder_wait_ms)
							? cfg.axis1_post_return_cylinder_wait_ms
							: cfg.axis6_post_return_cylinder_wait_ms)
						: cfg.axis1_post_return_cylinder_wait_ms;
					axis1_fast_return = true;
					pos[0] = axis1_return_settle_rel;
					hold_axis1_mirror_axes_for_return();
					pos[1] = axis2_hold_rel;
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_clamp;
					if (axis6_coupled_active)
					{
						axis6_fast_retract = true;
						pos[5] = axis6_return_settle_rel;
						cylinder3_cmd = cyl.cyl3_open;
						cylinder4_cmd = cyl.cyl4_clamp;
					}
					if ((now_ms - axis1_crawl.phase_t0) >= coupled_post_return_wait_ms)
					{
						if (!sync_axis1(3, false, 0))
						{
							std::cout << "Axis1 resync after planned return failed." << std::endl;
							axis1_crawl.phase = CrawlState::Phase::Follow;
						}
						axis6_coupled_active = false;
						axis6_coupled_requested = false;
						axis6_coupled_done = false;
						axis6_coupled_error = false;
						axis6_coupled_error_id = 0;
					}
				}

				if (cooperative_mode)
				{
					// 协同模式保持导管侧各轴挂在 axis1 链路上，同时 axis6 运行
					// 增量叠加的爬行状态机：
					// axis6_increment = increment(587) + increment(axis1_cmd)。
					const double axis1_cmd_abs_for_ff = pos[0] + plc_init_pos[0];

					if (axis6_crawl.phase == CrawlState::Phase::Follow)
					{
						if (!axis6_coop_ff_inited)
						{
							axis6_coop_ff_inited = true;
							axis6_coop_prev_axis1_cmd_abs = axis1_cmd_abs_for_ff;
						}
						double axis1_increment_mm = 0.0;
						if (axis1_crawl.phase == CrawlState::Phase::Follow)
						{
							axis1_increment_mm = axis1_cmd_abs_for_ff - axis6_coop_prev_axis1_cmd_abs;
						}
						const double axis6_combined_increment_mm = axis6_linear_increment_mm + axis1_increment_mm;
						axis6_coop_prev_axis1_cmd_abs = axis1_cmd_abs_for_ff;

						if (cfg.cooperative_debug_log && ((loop_count % 50) == 0))
						{
							std::cout
								<< "[COOP] inc587=" << axis6_linear_increment_mm
								<< " inc1=" << axis1_increment_mm
								<< " sum=" << axis6_combined_increment_mm
								<< " axis1_phase=" << static_cast<int>(axis1_crawl.phase)
								<< std::endl;
						}

						const double axis6_raw_cmd_abs = axis6_follow_cmd_abs + axis6_combined_increment_mm;

						run_axis6_crawl_state(
							axis6_raw_cmd_abs,
							axis6_combined_increment_mm,
							axis6_effective_reverse_pressed,
							axis6_linear_increment_active,
							true);
					}
					else
					{
						// 按设计，axis6 回退阶段会忽略 axis1 的贡献。
						axis6_coop_ff_inited = false;
						run_axis6_crawl_state(
							axis6_follow_cmd_abs,
							0.0,
							axis6_effective_reverse_pressed,
							false,
							true);
					}
				}
			}

			axis1_prev_linear_filtered = axis1_linear_filtered;
			axis6_prev_linear_filtered = axis6_linear_filtered;
			axis1_prev_abs_for_trigger = axis1_abs;
			axis6_prev_abs_for_trigger = axis6_abs;
			axis1_prev_abs_valid = true;
			axis6_prev_abs_valid = true;
			write_refer();
		}

		bool axis4_manual_busy_now = false;
		bool axis4_manual_error_now = false;
		unsigned long axis4_manual_error_id_now = 0;
		if ((loop_count % 20) == 0)
		{
			bool axis4_diag_ok = true;
			axis4_diag_ok = ads.ADSRead(AdsSymbol::axis4_manual_busy, sizeof(axis4_manual_busy_now), &axis4_manual_busy_now) && axis4_diag_ok;
			axis4_diag_ok = ads.ADSRead(AdsSymbol::axis4_manual_error, sizeof(axis4_manual_error_now), &axis4_manual_error_now) && axis4_diag_ok;
			axis4_diag_ok = ads.ADSRead(AdsSymbol::axis4_manual_error_id, sizeof(axis4_manual_error_id_now), &axis4_manual_error_id_now) && axis4_diag_ok;
			if (axis4_diag_ok)
			{
				if (axis4_manual_busy_now != axis4_manual_busy_prev)
				{
					std::cout << "Axis4 manual busy: " << (axis4_manual_busy_now ? "ON" : "OFF") << std::endl;
					axis4_manual_busy_prev = axis4_manual_busy_now;
				}
				if (axis4_manual_error_now &&
					(!axis4_manual_error_prev || axis4_manual_error_id_now != axis4_manual_error_id_prev))
				{
					std::cout << "Axis4 manual error: " << axis4_manual_error_id_now << std::endl;
				}
				axis4_manual_error_prev = axis4_manual_error_now;
				axis4_manual_error_id_prev = axis4_manual_error_id_now;
			}
		}

		// 10) 仅在运动激活时驱动气缸；快速回退标志始终会写入。
		if (!freeze_active && (control_active || motion_startup_active))
		{
			ads.ADSWrite(AdsSymbol::cylinder1_value, sizeof(cylinder1_cmd), &cylinder1_cmd);
			ads.ADSWrite(AdsSymbol::cylinder2_value, sizeof(cylinder2_cmd), &cylinder2_cmd);
			ads.ADSWrite(AdsSymbol::cylinder3_value, sizeof(cylinder3_cmd), &cylinder3_cmd);
			ads.ADSWrite(AdsSymbol::cylinder4_value, sizeof(cylinder4_cmd), &cylinder4_cmd);
		}

		write_axis4_manual_requests(axis4_manual_forward_req, axis4_manual_reverse_req);
		ads.ADSWrite(AdsSymbol::startup_smoothing_bypass, sizeof(startup_smoothing_bypass), &startup_smoothing_bypass);
		ads.ADSWrite(AdsSymbol::axis1_fast_return, sizeof(axis1_fast_return), &axis1_fast_return); // 轴1快退平滑旁路
		ads.ADSWrite(AdsSymbol::axis6_fast_retract, sizeof(axis6_fast_retract), &axis6_fast_retract); // 轴6快退平滑旁路

		// 力感数据记录：默认启用，可通过 force_log_period_ms 控制频率。
		const DWORD force_log_now_ms = GetTickCount();
		// 力传感器只用于日志，不参与控制闭环：降频到每 10 帧读取一次。
		if (force_log.should_sample(force_log_now_ms) && ((loop_count % 10) == 0))
		{
			short ft_1_value = 0;
			short fn_1_value = 0;
			short fn_2_value = 0;
			short ft_2_value = 0;
			bool force_log_read_ok = true;
			force_log_read_ok = ads.ADSRead(AdsSymbol::ft_1_value, sizeof(ft_1_value), &ft_1_value) && force_log_read_ok;
			force_log_read_ok = ads.ADSRead(AdsSymbol::fn_1_value, sizeof(fn_1_value), &fn_1_value) && force_log_read_ok;
			force_log_read_ok = ads.ADSRead(AdsSymbol::fn_2_value, sizeof(fn_2_value), &fn_2_value) && force_log_read_ok;
			force_log_read_ok = ads.ADSRead(AdsSymbol::ft_2_value, sizeof(ft_2_value), &ft_2_value) && force_log_read_ok;
			if (force_log_read_ok)
			{
				force_log.append_sample(force_log_now_ms, ft_1_value, fn_1_value, fn_2_value, ft_2_value);
			}
			else if ((force_log_now_ms - force_log_warn_last_ms) >= 1000)
			{
				std::cout << "Force sensor logging warning: ADS read failed." << std::endl;
				force_log_warn_last_ms = force_log_now_ms;
			}
		}
		// 无论本拍是否进入控制分支，都更新线性差分基准，避免暂停/等待期间累积大跳变。
		axis1_prev_linear_filtered = axis1_handle_filter.axis0_filtered;
		axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;
	}

	startup_smoothing_bypass = false;
	ads.ADSWrite(AdsSymbol::startup_smoothing_bypass, sizeof(startup_smoothing_bypass), &startup_smoothing_bypass);
	force_log.close();
	handle_axis1.close();
	handle_axis6.close();
	return 0;
}
