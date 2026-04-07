// 文件职责说明：
// 1) 定义控制程序公共类型、配置结构与轻量工具函数声明。
// 2) 定义运行时上下文 AppContext，供各业务模块共享状态。
// 3) 本文件不承载具体业务流程，仅提供类型与基础接口。
#pragma once

#include "Handle.h"
#include <ADSComm1.h>
#include <fstream>
#include <string>
#include <windows.h>

void setup_console_utf8();
std::string build_force_log_filename();
std::string build_force_log_timestamp();
double clamp_double(double value, double low, double high);
bool is_within_range(double value, double low, double high, double tol = 0.0);
void get_average_handle_pose(Handle& handle, int samples, double& axis0, double& axis1);
void get_average_dual_pos(
	Handle& handle_a,
	Handle& handle_b,
	int samples,
	double& a_axis0,
	double& a_axis1,
	double& b_axis0,
	double& b_axis1);
void copy_positions(const double* src, double* dst, int count);

class TcpForceDaqClient;

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

enum class ForceSampleSource
{
	ADS,
	TCP_DAQ
};

struct ControlConfig
{
	// 手柄运动缩放系数与符号约定。
	double k_handle_to_mm = 500.0 * (75.0 / 50.0); // 手柄线性位移差 -> 轴位移增量(mm)
	double axis_push_sign = -1.0; // 手柄“推/拉”到轴“正/负”方向的映射符号
	double axis_rot_scale_deg = Rad;

	// 力反馈输出配置：当前主路径统一使用 setforce(F,N) 的 axis=0 语义。
	// axial_force_* 保留为兼容旧配置，本轮不再参与映射。
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
	// 轴6窗口整体平移：正值表示远离左限位方向平移（单位 mm）。
	double axis6_window_shift_from_left_mm = 3.0;
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
	// 轴1切缸顺序中两步动作之间的等待（导管正/反向共用）。
	DWORD axis1_cylinder_interstep_wait_ms = 100;
	// 快退完成后切回最终缸态等待（轴1主链路）。
	DWORD axis1_post_return_cylinder_wait_ms = 100;
	double axis6_return_velocity_mm_s = 200.0;
	double axis6_return_acc_mm_s2 = 2400.0;
	double axis6_return_dec_mm_s2 = 2400.0;
	double axis6_return_jerk_mm_s3 = 35000.0;
	// 快退前先切缸等待（轴6链路）。
	DWORD axis6_pre_move_cylinder_wait_ms = 80;
	// 轴6切缸顺序中两步动作之间的等待（导丝正/反向共用）。
	DWORD axis6_cylinder_interstep_wait_ms = 100;
	// 快退完成后切回最终缸态等待（轴6链路）。
	DWORD axis6_post_return_cylinder_wait_ms = 80;

	// 手柄低通滤波。
	double linear_handle_alpha = 0.25;
	double rotational_handle_alpha = 0.20;
	bool cooperative_debug_log = false;
	// CSV 中 ft_1/fn_1 的采样来源：ADS(PLC变量) 或 TCP_DAQ(采集卡直连)。
	ForceSampleSource force_sample_source = ForceSampleSource::ADS;
	// TCP 采集卡连接参数（仅 force_sample_source=TCP_DAQ 时生效）。
	const char* tcp_force_daq_ip = "192.168.1.30";
	unsigned short tcp_force_daq_port = 502;
	// 力感记录周期：0=每循环记录；>0=按毫秒周期记录。
	DWORD force_log_period_ms = 0;

	// 启动准备阶段目标。
	DWORD startup_clamp_settle_delay_ms = 300;
	double startup_motion_speed_scale = 0.5;
	unsigned short startup_cyl3_open = 500;
	unsigned short startup_cyl4_open = 0;
	unsigned short startup_cyl3_clamp = 0;
	unsigned short startup_cyl4_clamp = 1000;
	double startup_axis1_ready_from_left_mm = 20.0;
	double startup_axis5_ready_from_left_mm = 290.0;
	double startup_axis3_ready_from_left_mm = 635.0;
	// 在 axis3 完全到达目标前提前触发 cylinder2 夹紧；现场调参使其领先约 0.5 s。
	double startup_axis3_cyl2_clamp_advance_mm = 50.0;
};

struct CylinderPreset
{
	// 命名遵循当前接线方式：
	// cyl1/cyl2 属于导管侧夹爪对，
	// cyl3/cyl4 属于导丝侧夹爪对。
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
	// 顺序切缸子阶段：
	// 0=未启用/已完成，1=第一步已下发，2=第二步已下发（进入旧等待计时）
	int cyl_seq_stage = 0;
	DWORD cyl_seq_t0 = 0;

	double min_abs() const { return (start_abs < end_abs) ? start_abs : end_abs; }
	double max_abs() const { return (start_abs > end_abs) ? start_abs : end_abs; }
};

struct ForceFeedbackState
{
	// 力反馈开关：F=ON 时允许输出，F=OFF 时强制双手柄归零。
	bool enabled = false;
	// 当前输出命令缓存（用于调试与冻结保持）。
	double force_582_f = 0.0;
	double force_582_n = 0.0;
	double force_587_f = 0.0;
	double force_587_n = 0.0;
	// 导管快进/快退期间冻结 582 输出，保持进入冻结时最后一拍命令。
	bool freeze_582_active = false;
	double freeze_582_f = 0.0;
	double freeze_582_n = 0.0;
	// 调试观测缓存。
	short last_fn_1_raw = 0;
	short last_ft_1_raw = 0;
	short last_fn_2_raw = 0;
	short last_ft_2_raw = 0;
	bool last_fast_move = false;
	GuidewireMode last_mode = GuidewireMode::None;

	void reset()
	{
		force_582_f = 0.0;
		force_582_n = 0.0;
		force_587_f = 0.0;
		force_587_n = 0.0;
		freeze_582_active = false;
		freeze_582_f = 0.0;
		freeze_582_n = 0.0;
		last_fast_move = false;
		last_mode = GuidewireMode::None;
	}

	void clear_output()
	{
		force_582_f = 0.0;
		force_582_n = 0.0;
		force_587_f = 0.0;
		force_587_n = 0.0;
		freeze_582_active = false;
		freeze_582_f = 0.0;
		freeze_582_n = 0.0;
	}
};

struct ForceSampleFrame
{
	short ft_1_value = 0;
	short fn_1_value = 0;
	short fn_2_value = 0;
	short ft_2_value = 0;
	double axis1_pos_rel = 0.0;
	bool valid = false;
	DWORD tick_ms = 0;
};

struct ForceOutputCmd
{
	double force_582_f = 0.0;
	double force_582_n = 0.0;
	double force_587_f = 0.0;
	double force_587_n = 0.0;
};

struct ForceLogState
{
	// 力感记录与采样使能；period_ms=0 表示每个主循环都记录。
	bool enabled = false;
	DWORD period_ms = 0;
	DWORD last_sample_ms = 0;
	DWORD last_buffer_flush_ms = 0;
	std::ofstream file;
	std::string filename;
	std::string line_buffer;
	size_t buffered_lines = 0;

	bool open_file(const std::string& output_name);
	bool should_sample(DWORD now_ms) const;
	void append_sample(
		DWORD now_ms,
		double ft1_value,
		double fn1_value,
		short fn2_value,
		short ft2_value,
		int mode_code,
		int reverse_code,
		int push_pull_code,
		int rot_sign_code,
		double axis1_pos_rel);
	void flush(bool force_flush);
	void close();
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

struct AppContext
{
	// 外设与通信对象。
	CADSComm* ads = nullptr;
	Handle* axis1_input_handle = nullptr;
	Handle* axis6_input_handle = nullptr;
	Handle* handle_axis1 = nullptr;
	Handle* handle_axis6 = nullptr;
	HandleFilterState* axis1_handle_filter = nullptr;
	HandleFilterState* axis6_handle_filter = nullptr;

	// 配置与预设。
	const ControlConfig* cfg = nullptr;
	const CylinderPreset* cyl = nullptr;
	ForceSampleSource force_sample_source = ForceSampleSource::ADS;
	TcpForceDaqClient* tcp_force_daq = nullptr;

	// PLC 镜像缓存。
	double* pos = nullptr;
	double* plc_act_pos = nullptr;
	double* plc_init_pos = nullptr;
	double* plc_leftlimit = nullptr;
	double* plc_act_pos_from_left = nullptr;
	double* plc_refer_from_left = nullptr;
	double* plc_v_limit = nullptr;

	// 模式与状态机。
	GuidewireMode* guidewire_mode = nullptr;
	CrawlState* axis1_crawl = nullptr;
	CrawlState* axis6_crawl = nullptr;
	StartupState* startup = nullptr;
	ForceFeedbackState* ff = nullptr;
	ForceLogState* force_log = nullptr;

	// 常规导管/导丝基线状态。
	double* axis3_base_rel = nullptr;
	double* axis5_base_rel = nullptr;
	double* axis6_mirror_base_rel = nullptr;
	double* axis1_return_entry_rel = nullptr;
	double* axis1_return_settle_rel = nullptr;
	double* axis6_return_entry_rel = nullptr;
	double* axis6_return_settle_rel = nullptr;
	double* axis1_return_hold_axis3_rel = nullptr;
	double* axis1_return_hold_axis5_rel = nullptr;
	double* axis1_fast_entry_abs = nullptr;
	double* axis6_fast_entry_abs = nullptr;
	double* axis6_coupled_target_abs = nullptr;
	double* axis6_coupled_settle_rel = nullptr;
	bool* axis6_coupled_active = nullptr;
	bool* axis6_coupled_requested = nullptr;
	bool* axis6_coupled_done = nullptr;
	bool* axis6_coupled_error = nullptr;
	unsigned long* axis6_coupled_error_id = nullptr;
	double* axis2_hold_rel = nullptr;
	double* axis7_hold_rel = nullptr;
	double* axis1_prev_linear_filtered = nullptr;
	double* axis6_prev_linear_filtered = nullptr;
	double* axis1_prev_rot_filtered = nullptr;
	double* axis6_prev_rot_filtered = nullptr;
	double* axis1_follow_cmd_abs = nullptr;
	double* axis6_follow_cmd_abs = nullptr;
	bool* axis1_reverse_switch_guard_active = nullptr;
	bool* axis6_reverse_switch_guard_active = nullptr;
	double* axis1_prev_abs_for_trigger = nullptr;
	double* axis6_prev_abs_for_trigger = nullptr;
	bool* axis1_prev_abs_valid = nullptr;
	bool* axis6_prev_abs_valid = nullptr;
	double* independent_axis1_hold_rel = nullptr;
	double* independent_axis2_hold_rel = nullptr;
	double* independent_axis3_hold_rel = nullptr;
	double* independent_axis5_hold_rel = nullptr;
	bool* axis6_window_locked = nullptr;
	double* axis6_locked_window_start_abs = nullptr;
	double* axis6_locked_window_end_abs = nullptr;
	bool* axis6_coop_ff_inited = nullptr;
	double* axis6_coop_prev_axis1_cmd_abs = nullptr;
	bool* startup_smoothing_bypass = nullptr;
};
