#include "Handle.h"
#include <ADSComm1.h>

#include <cmath>
#include <conio.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <windows.h>

namespace
{

// ═══════════════════════════════════════════════════════════════════════════════
//  通用工具函数
// ═══════════════════════════════════════════════════════════════════════════════

double clamp_double(double value, double low, double high)
{
	if (value < low) return low;
	if (value > high) return high;
	return value;
}

bool is_within_range(double value, double low, double high, double tol = 0.0)
{
	return (value >= (low - tol)) && (value <= (high + tol));
}

/// 对手柄某一轴做 N 次采样取平均，用于建立零位基准。
/// 每次采样间隔 10 ms，总耗时约 samples × 10 ms。
double get_average_pos(Handle& handle, int axis, int samples)
{
	double sum = 0.0;
	for (int i = 0; i < samples; ++i)
	{
		handle.poll();
		sum += handle.fJoints2[axis];
		Sleep(10);
	}
	return sum / static_cast<double>(samples);
}

/// 同时对两个手柄做 N 次采样取平均。
/// 必须同步采样——若分别采样，先采的手柄零位会比后采的早约 N×10 ms，
/// 进入控制的首帧会产生假位移。
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

// ═══════════════════════════════════════════════════════════════════════════════
//  ADS 符号名常量
//  与 TwinCAT PLC 程序中 G（全局变量表）的变量名一一对应。
//  集中定义避免字符串散落，修改 PLC 变量名时只改这一处。
// ═══════════════════════════════════════════════════════════════════════════════

namespace AdsSymbol
{
	const char* refer             = "G.refer";           // double[7]  位置指令（相对于 init_pos）
	const char* act_pos           = "G.Act_pos";         // double[7]  各轴实际位置（相对于 init_pos）
	const char* init_pos          = "G.init_pos";        // double[7]  各轴初始绝对位置
	const char* rightlimit        = "G.rightlimit";      // double[7]  各轴右限位绝对位置
	const char* v_limit           = "G.v_limit";         // double[7]  各轴速度限制
	const char* cylinder1_value   = "G.cylinder1_value"; // unsigned short  导管前夹持气缸
	const char* cylinder2_value   = "G.cylinder2_value"; // unsigned short  导管后夹持气缸
	const char* cylinder3_value   = "G.cylinder3_value"; // unsigned short  导丝前夹持气缸
	const char* cylinder4_value   = "G.cylinder4_value"; // unsigned short  导丝后夹持气缸
	const char* self_check_done   = "G.self_check_done"; // bool  PLC 自检完成标志
	const char* handle_reinit_req = "G.handle_reinit_req"; // bool  PLC 请求手柄重初始化
	const char* estop_hold_req    = "G.estop_hold_req";  // bool  急停/保持请求
	const char* fn_value          = "G.fn_value";        // short  法向力传感器原始值
	const char* ft_value          = "G.ft_value";        // short  扭矩传感器原始值
	const char* axis1_fast_return = "G.axis1_fast_return";   // bool  轴1快速回退标志
	const char* axis6_fast_retract = "G.axis6_fast_retract"; // bool  轴6快速回撤标志
}

// ═══════════════════════════════════════════════════════════════════════════════
//  控制参数
//  所有运动参数集中定义，方便查阅和调参。实例化后作为 const 使用。
// ═══════════════════════════════════════════════════════════════════════════════

struct ControlConfig
{
	// --- 手柄映射 ---
	// 手柄关节量 → 直线位移 (mm) 的换算系数。
	// 500 是设备基础比例，75/50 是齿轮比校正，最终 = 750.0 mm/单位。
	double k_handle_to_mm = 500.0 * (75.0 / 50.0);
	// 手柄正方向与机器人轴正方向相反，需取反。
	double axis_push_sign = -1.0;
	// 旋转轴：手柄输出弧度，乘以 Rad (= 180/π ≈ 57.296) 转为度。
	double axis_rot_scale_deg = Rad;
	double axis2_rot_reengage_deadband_deg = 1.0;

	// --- 力反馈 ---
	int    axial_force_axis = 1;   // 力反馈施加在手柄的第 1 轴（直线轴）
	double axial_force_sign = -1.0; // 力方向取反（手柄坐标与传感器坐标相反）

	// --- 按钮掩码（手柄 buttons2 的 bit 位）---
	unsigned char btn_b0 = 0x01;
	unsigned char btn_b6 = 0x40;
	unsigned char btn_b7 = 0x80;

	// --- 蠕动窗口参数 ---
	// 蠕动窗口起点：距离右限位 56 mm 处。
	double crawl_window_start_offset_mm = 56.0;
	// 蠕动窗口大小：30 mm（轴1），20 mm（轴6独立模式）。
	double crawl_window_size_mm = 30.0;
	double axis6_independent_window_size_mm = 20.0;
	// 触发蠕动的手柄死区和重臂阈值。
	double crawl_trigger_deadband_mm = 0.3;
	double crawl_rearm_threshold_mm = 0.3;
	// 到位判定容差。
	double crawl_arrive_tol_mm = 0.2;
	// 状态机延时：切换等待 250 ms，夹紧等待 50 ms。
	DWORD  crawl_switch_delay_ms = 250;
	DWORD  crawl_clamp_delay_ms = 50;

	// --- 软限位 ---
	// 反向模式下，禁止超过 rightlimit - margin。
	double axis1_right_soft_margin_mm = 10.0;
	double axis6_right_soft_margin_mm = 10.0;

	// --- 启动准备序列参数 ---
	DWORD  startup_clamp_settle_delay_ms = 300;
	double startup_motion_speed_scale = 0.5;
	unsigned short startup_cyl3_open = 2000;
	// 各轴就位目标：距离右限位的偏移量 (mm)。
	double startup_axis5_ready_from_right_mm = 425.0;
	double startup_axis3_ready_from_right_mm = 250.0;
	double startup_axis3_follow_from_right_mm = 110.0;
	double startup_axis6_follow_from_right_mm = 50.0;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  气缸控制预设值（DAC 输出量）
//  cyl1/cyl2 控制导管前后夹持，cyl3/cyl4 控制导丝前后夹持。
//  open = 松开，clamp = 夹紧，follow_release = 跟随时的微松状态。
// ═══════════════════════════════════════════════════════════════════════════════

struct CylinderPreset
{
	unsigned short cyl1_open   = 1000;
	unsigned short cyl1_clamp  = 100;
	unsigned short cyl2_open   = 0;
	unsigned short cyl2_clamp  = 1000;
	unsigned short cyl3_open   = 1000;
	unsigned short cyl3_clamp  = 100;
	unsigned short cyl4_open   = 0;
	unsigned short cyl4_clamp  = 1000;
	unsigned short cyl3_follow_release = 150;
	unsigned short cyl4_follow_release = 100;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  枚举与状态结构体
// ═══════════════════════════════════════════════════════════════════════════════

/// 导丝操作模式。
/// None:        普通导管模式，轴6镜像跟随轴1。
/// Independent: 独立导丝模式，轴1锁定，轴6由手柄587独立控制。
/// Cooperative: 协同导丝模式，轴1正常蠕动，轴6在轴1快速回退时做反向补偿。
enum class GuidewireMode
{
	None,
	Independent,
	Cooperative
};

/// 启动准备序列阶段。
/// 机器人从初始位置经多步运动到达操作就绪位置。
/// 每步之间有气缸夹持/释放和延时等待。
enum class StartupPhase
{
	WaitForEnter,        // 等待用户选择启动模式
	ReleaseClamps,       // 释放所有气缸
	MoveAxis5ToReady,    // 轴5移到就绪位
	ClampCylinder3Wait,  // 夹紧导丝前夹持，等待稳定
	MoveAxis3ToReady,    // 轴3移到就绪位（轴5联动）
	ClampCylinder2Wait,  // 夹紧导管后夹持，等待稳定
	MoveAxis3AndAxis5,   // 轴3+5联动到跟随位
	MoveAxis3Axis5Axis6, // 轴3+5+6联动到最终位
	Done
};

/// 蠕动状态机。
/// 蠕动(crawl)是导管/导丝推进的核心机制：在一个有限窗口内来回移动，
/// 每次到达窗口边界时切换夹持气缸（前夹后松→后夹前松），快速回位，
/// 再切换回来继续推进，实现"步进式"无限行程推进。
///
/// 状态转换：Follow → SwitchWait → FastMove → ClampWait → RestoreWait → Follow
///
/// Follow:      手柄跟随，实时计算位置指令。
/// SwitchWait:  到达窗口边界，前端夹紧、后端松开，等待气缸动作 (250ms)。
/// FastMove:    快速回退到窗口另一端（PLC 用高速模式）。
/// ClampWait:   到达后，前后都夹紧，等待稳定 (50ms)。
/// RestoreWait: 前端松开、后端夹紧，等待气缸切换 (250ms)，然后重新同步进入 Follow。
struct CrawlState
{
	enum class Phase
	{
		Follow,
		SwitchWait,
		FastMove,
		ClampWait,
		RestoreWait
	};

	bool enabled = false;
	Phase phase = Phase::Follow;
	bool wait_rearm = false;     // 蠕动完成一次后需要操作者"重新推动"才触发下一次
	bool window_active = false;  // 当前位置是否在蠕动窗口内
	int rearm_dir = 0;           // 上次蠕动方向，用于 rearm 检测
	double handle_ref = 0.0;     // 手柄直线轴零位基准
	double rot_ref = 0.0;        // 手柄旋转轴零位基准
	double base_rel = 0.0;       // 同步时 PLC 实际相对位置基准
	double rot_base_rel = 0.0;   // 同步时 PLC 旋转相对位置基准
	double start_abs = 0.0;      // 蠕动窗口起点（绝对坐标）
	double end_abs = 0.0;        // 蠕动窗口终点（绝对坐标）
	double target_abs = 0.0;     // 当前蠕动快速回退目标（绝对坐标）
	DWORD phase_t0 = 0;          // 当前阶段起始时刻 (ms)

	/// 返回窗口的 min/max 绝对坐标。
	double min_abs() const { return (start_abs < end_abs) ? start_abs : end_abs; }
	double max_abs() const { return (start_abs > end_abs) ? start_abs : end_abs; }
};

/// 力反馈状态。
/// 从 PLC 读取法向力(fn)和扭矩(ft)传感器，经滤波后输出到手柄电机。
struct ForceFeedbackState
{
	bool enabled = false;       // 用户按 F 键开关

	short fn_raw = 0;           // 法向力原始读数
	short ft_raw = 0;           // 扭矩原始读数
	double fn_bias = 0.0;       // 法向力自适应零偏
	bool fn_bias_inited = false;
	double fn_force_f = 0.0;    // 法向力低通滤波输出
	double ft_force_f = 0.0;    // 扭矩低通滤波输出
	int last_fn_raw = 0;        // 上帧原始值（用于日志去重）
	int last_ft_raw = 0;
	short fn_last_valid = 0;    // 最后一次有效读数（用于 hold-last-valid）
	short ft_last_valid = 0;
	bool fn_has_valid = false;
	bool ft_has_valid = false;
	int fn_invalid_streak = 0;  // 连续无效读数计数
	int ft_invalid_streak = 0;

	/// 完整重置（切换开关时调用）。
	void reset()
	{
		fn_bias_inited = false;
		fn_force_f = 0.0;
		ft_force_f = 0.0;
		fn_invalid_streak = 0;
		ft_invalid_streak = 0;
	}

	/// 仅清除输出和偏置（暂停时调用，不清 streak 以保留连续性）。
	void clear_output()
	{
		fn_bias_inited = false;
		fn_force_f = 0.0;
		ft_force_f = 0.0;
	}
};

/// 启动准备序列状态。
struct StartupState
{
	StartupPhase phase = StartupPhase::WaitForEnter;
	bool completed = false;   // 启动序列是否已完成
	bool prompted = false;    // 是否已提示用户选择模式
	DWORD phase_t0 = 0;

	// 各轴在启动开始时的锁定位置（相对于 init_pos）
	double axis1_hold_rel = 0.0;
	double axis2_hold_rel = 0.0;
	double axis3_hold_rel = 0.0;
	double axis5_hold_rel = 0.0;
	double axis6_hold_rel = 0.0;
	double axis7_hold_rel = 0.0;

	// 阶段间联动基准——启动过程中多轴需要同步联动，
	// 每进入新阶段时捕获当前位置作为下一段联动的基准。
	double axis3_phase2_base_rel = 0.0;
	double axis5_phase2_base_rel = 0.0;
	double axis3_phase3_base_rel = 0.0;
	double axis5_phase3_base_rel = 0.0;
	double axis6_phase3_base_rel = 0.0;

	// 速度限制：启动序列降速运行，完成后恢复原值
	double v_limit_backup[7] = {};
	bool v_limit_scaled = false;

	bool is_active() const
	{
		return phase != StartupPhase::WaitForEnter && phase != StartupPhase::Done;
	}
};

// ═══════════════════════════════════════════════════════════════════════════════
//  蠕动状态机辅助函数
// ═══════════════════════════════════════════════════════════════════════════════

/// 检测蠕动 rearm 条件。
/// 蠕动完成一次后置 wait_rearm=true，要求操作者沿上一次有效方向再次推过阈值，
/// 防止蠕动在窗口边界反复误触发。反方向位移仅视为回弹/重置手柄行程，不清除 wait_rearm。
void check_crawl_rearm(CrawlState& crawl, double hand_delta_mm, double threshold_mm)
{
	if (!crawl.wait_rearm)
	{
		return;
	}
	const bool same_dir_push    = (crawl.rearm_dir > 0) && (hand_delta_mm > threshold_mm);
	const bool same_dir_pull    = (crawl.rearm_dir < 0) && (hand_delta_mm < -threshold_mm);
	if (same_dir_push || same_dir_pull)
	{
		crawl.wait_rearm = false;
	}
}

/// 蠕动非 Follow 阶段处理结果。
struct CrawlSettleResult
{
	unsigned short cyl_front;  // 前端夹持气缸指令
	unsigned short cyl_rear;   // 后端夹持气缸指令
	bool fast_flag;            // 快速移动标志（写入 PLC）
	bool sync_needed;          // RestoreWait 超时，需要调用方执行同步回 Follow
};

/// 蠕动状态机非 Follow 阶段的通用处理。
///
/// 导管轴1和导丝轴6的 SwitchWait→FastMove→ClampWait→RestoreWait 状态转换结构相同，
/// 区别仅在：
///   1. 操作的气缸编号不同（轴1用cyl1/cyl2，轴6用cyl3/cyl4）。
///   2. 轴6独立模式在 ClampWait/RestoreWait 阶段需要显式保持目标位（hold_in_settle=true），
///      而轴1依赖帧开头 load_pos_from_actual 的值（hold_in_settle=false）。
///   3. 调用方负责不同的 sync 回调。
///
/// @param pos_slot       指向 pos 数组中对应轴的槽（如 &pos[0] 或 &pos[5]）。
/// @param hold_in_settle 若 true，在 ClampWait/RestoreWait 也写目标位到 pos_slot。
CrawlSettleResult process_crawl_settle_phases(
	CrawlState& crawl,
	double axis_abs,
	double plc_init,
	double* pos_slot,
	bool hold_in_settle,
	DWORD now_ms,
	DWORD switch_delay_ms,
	DWORD clamp_delay_ms,
	double arrive_tol_mm,
	unsigned short cyl_clamp,
	unsigned short cyl_open)
{
	CrawlSettleResult r;
	r.cyl_front = cyl_clamp;
	r.cyl_rear = cyl_open;
	r.fast_flag = false;
	r.sync_needed = false;

	const double target_rel = crawl.target_abs - plc_init;

	if (crawl.phase == CrawlState::Phase::SwitchWait)
	{
		// 前端夹紧、后端松开，等待气缸完成动作
		r.cyl_front = cyl_clamp;
		r.cyl_rear = cyl_open;
		if ((now_ms - crawl.phase_t0) >= switch_delay_ms)
		{
			crawl.phase = CrawlState::Phase::FastMove;
			crawl.phase_t0 = now_ms;
		}
	}
	else if (crawl.phase == CrawlState::Phase::FastMove)
	{
		// 快速回退到窗口另一端
		r.cyl_front = cyl_clamp;
		r.cyl_rear = cyl_open;
		r.fast_flag = true;
		*pos_slot = target_rel;
		if (std::abs(axis_abs - crawl.target_abs) <= arrive_tol_mm)
		{
			crawl.phase = CrawlState::Phase::ClampWait;
			crawl.phase_t0 = now_ms;
		}
	}
	else if (crawl.phase == CrawlState::Phase::ClampWait)
	{
		// 前后都夹紧，等待稳定
		if (hold_in_settle) *pos_slot = target_rel;
		r.cyl_front = cyl_clamp;
		r.cyl_rear = cyl_clamp;
		if ((now_ms - crawl.phase_t0) >= clamp_delay_ms)
		{
			crawl.phase = CrawlState::Phase::RestoreWait;
			crawl.phase_t0 = now_ms;
		}
	}
	else if (crawl.phase == CrawlState::Phase::RestoreWait)
	{
		// 前端松开、后端夹紧，恢复正常跟随夹持配置
		if (hold_in_settle) *pos_slot = target_rel;
		r.cyl_front = cyl_open;
		r.cyl_rear = cyl_clamp;
		if ((now_ms - crawl.phase_t0) >= switch_delay_ms)
		{
			r.sync_needed = true;
		}
	}

	return r;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  力反馈处理
// ═══════════════════════════════════════════════════════════════════════════════

/// 力反馈完整处理流程。
///
/// 数据流：PLC 传感器 → ADS 读取 → 有效性检测(hold-last-valid) →
///         偏置自适应 → 死区 → 增益限幅 → 低通滤波 → 手柄电机输出。
///
/// 法向力 fn: 传感器原始值 → 去偏 → 死区(±20) → 增益(1/1000) → 限幅(±6N) → IIR(0.7/0.3)
/// 扭矩 ft:   传感器原始值 → 分段线性映射（死区[-870,-700]，两侧线性）→ 限幅(±1) → IIR(0.7/0.3)
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
		// --- 有效性检测与 hold-last-valid ---
		// 传感器偶发异常值（|raw| > 8000），此时保持上一次有效值。
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

		// --- 偏置自适应 ---
		// 首次读取时直接取当前值作为偏置。
		// 后续若偏差在 ±120 以内，以 0.5% 缓慢跟踪（只修正漂移，不追实际力）。
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

		// --- 法向力：去偏 → 死区 → 增益 → 限幅 ---
		double fn_zeroed = static_cast<double>(ff.fn_raw) - ff.fn_bias;
		if (std::abs(fn_zeroed) < 20.0)
		{
			fn_zeroed = 0.0;
		}

		const double axial_gain = 1.0 / 1000.0;
		const double axial_limit = 6.0;
		const double axial_force = clamp_double(fn_zeroed * axial_gain, -axial_limit, axial_limit);

		// --- 扭矩力：分段线性映射 ---
		// 死区 [-870, -700]（传感器在此区间表示零扭矩）。
		// > -700: 正方向映射，增益 -1/600。
		// < -870: 负方向映射，增益 -1/530。
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

		// --- 低通滤波与输出 ---
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

		// --- 日志（变化时或每 100 帧）---
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
		// ADS 读取失败或力反馈关闭或急停：清零输出
		ff.fn_force_f = 0.0;
		ff.ft_force_f = 0.0;
		if (!freeze_active)
		{
			handle.setforce_axis(0.0, force_axis, 0.0);
		}
	}
}

} // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════════════
//  主程序入口
// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
	const DWORD serial_axis1_handle = 582;
	const DWORD serial_axis6_handle = 587;
	const char* hardcoded_ads_netid = "169.254.119.135.1.1";

	// ═══ CLI 调试模式：--buttons ═══
	// 打印手柄按钮状态的每个 bit，用于硬件调试。
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

	// ═══ CLI 调试模式：--monitor ═══
	// 持续打印手柄的编码器、速度、关节角数据。
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

	// ═══ 参数实例化 ═══

	const ControlConfig cfg;
	const CylinderPreset cyl;

	// 按钮语义别名（基于 ControlConfig 中的 bit 定义）
	const unsigned char axis1_pause_button_mask       = cfg.btn_b6;
	const unsigned char axis1_reverse_button_mask     = cfg.btn_b0;
	const unsigned char axis6_reverse_button_mask     = cfg.btn_b0;
	const unsigned char axis6_independent_button_mask = cfg.btn_b6;
	const unsigned char axis6_cooperative_button_mask = cfg.btn_b7;

	// ═══ 硬件初始化 ═══

	Handle handle_axis1(serial_axis1_handle);
	Handle handle_axis6(serial_axis6_handle);
	CADSComm ads;

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

	// ADS 连接：先尝试本机路由（开发环境），失败则用硬编码远程地址。
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

	// ═══ ADS 读写 lambda ═══
	// 对 ADS 通讯做薄包装，隐藏 sizeof/类型细节。
	// 注意：pos[7] 是与 PLC G.refer 布局一致的 7×double 连续内存，不可拆分。

	auto apply_cmd_force = [&](double cmd_force)
	{
		handle_axis1.setforce_axis(cmd_force * cfg.axial_force_sign, cfg.axial_force_axis, 0.0);
	};

	// pos: 位置指令数组，与 PLC G.refer 对应。
	// 实际使用的轴：pos[0]=轴1直线, pos[1]=轴2旋转, pos[2]=轴3, pos[4]=轴5, pos[5]=轴6, pos[6]=轴7。
	// pos[3] 始终为 0（未使用轴，但必须保留以匹配 PLC 数组布局）。
	double pos[7] = { 0 };
	double plc_act_pos[7] = { 0 };    // PLC 各轴实际位置（相对于 init_pos）
	double plc_init_pos[7] = { 0 };   // PLC 各轴初始绝对位置
	double plc_rightlimit[7] = { 0 }; // PLC 各轴右限位绝对位置
	double plc_v_limit[7] = { 0 };    // PLC 各轴速度限制

	auto read_plc_state = [&]() -> bool
	{
		bool ok = true;
		ok = ads.ADSRead(AdsSymbol::act_pos, sizeof(plc_act_pos), plc_act_pos) && ok;
		ok = ads.ADSRead(AdsSymbol::init_pos, sizeof(plc_init_pos), plc_init_pos) && ok;
		ok = ads.ADSRead(AdsSymbol::rightlimit, sizeof(plc_rightlimit), plc_rightlimit) && ok;
		return ok;
	};

	auto write_refer = [&]() -> bool
	{
		return ads.ADSWrite(AdsSymbol::refer, sizeof(pos), pos);
	};

	auto read_v_limit = [&]() -> bool
	{
		return ads.ADSRead(AdsSymbol::v_limit, sizeof(plc_v_limit), plc_v_limit);
	};

	auto write_v_limit = [&](const double* values) -> bool
	{
		return ads.ADSWrite(AdsSymbol::v_limit, sizeof(plc_v_limit), (void*)values);
	};

	auto load_pos_from_actual = [&]()
	{
		copy_positions(plc_act_pos, pos, 7);
	};

	// ═══ 运动状态变量 ═══

	// 联动基准：轴3、轴5、轴6 在 Follow 模式下需要与轴1同步偏移，
	// 同步时刻捕获各自的 PLC 实际位置作为基准。
	double axis3_base_rel = 0.0;
	double axis5_base_rel = 0.0;
	double axis6_mirror_base_rel = 0.0;
	double axis2_hold_rel = 0.0;
	bool axis2_rot_reengage_required = false;
	double axis2_rot_reengage_ref = 0.0;
	double axis7_hold_rel = 0.0;
	bool axis7_rot_reengage_required = false;
	double axis7_rot_reengage_ref = 0.0;

	GuidewireMode guidewire_mode = GuidewireMode::None;

	// 独立模式下轴1各轴的锁定位置
	double independent_axis1_hold_rel = 0.0;
	double independent_axis2_hold_rel = 0.0;
	double independent_axis3_hold_rel = 0.0;
	double independent_axis5_hold_rel = 0.0;

	// 协同模式下轴6的保持位置
	double cooperative_axis6_hold_rel = 0.0;
	bool axis1_fastmove_prev = false;
	double axis1_fastmove_start_abs = 0.0;

	StartupState startup;
	ForceFeedbackState ff;

	CrawlState axis1_crawl;
	CrawlState axis6_crawl;
	axis1_crawl.enabled = true;

	// ═══ 蠕动窗口计算 ═══
	// 轴1的蠕动窗口由右限位决定（固定偏移），轴6的窗口在进入独立模式时动态捕获。
	// 这是因为轴1有固定的机械行程参考点，而轴6的操作起点取决于当时实际位置。

	auto axis1_start_abs = [&]() -> double
	{
		return plc_rightlimit[0] - cfg.crawl_window_start_offset_mm;
	};

	auto axis1_end_abs = [&]() -> double
	{
		return axis1_start_abs() + cfg.crawl_window_size_mm;
	};

	// ═══ 基线同步 / 模式切换 lambda ═══
	// 这些 lambda 在模式切换、暂停恢复、蠕动完成等事件发生时调用，
	// 重新读取 PLC 状态并建立手柄零位基准，确保恢复跟随时无跳变。

	/// 同步轴1蠕动状态。用于反向按钮切换、蠕动 RestoreWait 完成等场景。
	auto sync_axis1 = [&](int samples, bool wait_rearm, int rearm_dir) -> bool
	{
		const double preserved_axis2_hold_rel = axis2_hold_rel;

		if (!read_plc_state())
		{
			return false;
		}

		load_pos_from_actual();
		pos[1] = preserved_axis2_hold_rel;

		axis1_crawl.handle_ref = get_average_pos(handle_axis1, 0, samples);
		axis1_crawl.rot_ref = handle_axis1.fJoints2[1];
		axis1_crawl.base_rel = plc_act_pos[0];
		axis1_crawl.rot_base_rel = preserved_axis2_hold_rel;
		axis1_crawl.start_abs = axis1_start_abs();
		axis1_crawl.end_abs = axis1_end_abs();
		axis1_crawl.window_active = is_within_range(
			plc_act_pos[0] + plc_init_pos[0],
			axis1_crawl.min_abs(), axis1_crawl.max_abs(),
			cfg.crawl_arrive_tol_mm);
		axis1_crawl.phase = CrawlState::Phase::Follow;
		axis1_crawl.phase_t0 = GetTickCount();
		axis1_crawl.wait_rearm = wait_rearm;
		axis1_crawl.rearm_dir = rearm_dir;
		axis2_hold_rel = preserved_axis2_hold_rel;
		axis2_rot_reengage_required = wait_rearm;
		axis2_rot_reengage_ref = handle_axis1.fJoints2[1];

		axis3_base_rel = plc_act_pos[2];
		axis5_base_rel = plc_act_pos[4];
		axis6_mirror_base_rel = plc_act_pos[5];

		return write_refer();
	};

	/// 同步轴6蠕动状态。用于独立模式下反向切换、蠕动完成等场景。
	auto sync_axis6 = [&](int samples, bool capture_window, bool wait_rearm, int rearm_dir) -> bool
	{
		const double preserved_axis7_hold_rel = axis7_hold_rel;

		if (!read_plc_state())
		{
			return false;
		}

		load_pos_from_actual();
		pos[6] = preserved_axis7_hold_rel;

		axis6_crawl.handle_ref = get_average_pos(handle_axis6, 0, samples);
		axis6_crawl.rot_ref = handle_axis6.fJoints2[1];
		axis6_crawl.base_rel = plc_act_pos[5];
		axis6_crawl.rot_base_rel = preserved_axis7_hold_rel;
		if (capture_window || !axis6_crawl.enabled)
		{
			axis6_crawl.start_abs = plc_act_pos[5] + plc_init_pos[5];
			axis6_crawl.end_abs = axis6_crawl.start_abs + cfg.axis6_independent_window_size_mm;
		}
		axis6_crawl.window_active = is_within_range(
			plc_act_pos[5] + plc_init_pos[5],
			axis6_crawl.min_abs(), axis6_crawl.max_abs(),
			cfg.crawl_arrive_tol_mm);
		axis6_crawl.phase = CrawlState::Phase::Follow;
		axis6_crawl.phase_t0 = GetTickCount();
		axis6_crawl.wait_rearm = wait_rearm;
		axis6_crawl.rearm_dir = rearm_dir;
		axis6_crawl.enabled = true;
		axis7_hold_rel = preserved_axis7_hold_rel;
		axis7_rot_reengage_required = wait_rearm;
		axis7_rot_reengage_ref = handle_axis6.fJoints2[1];

		return write_refer();
	};

	/// 释放轴6回到镜像跟随模式。退出独立/协同模式时调用。
	auto release_axis6_to_follow = [&]() -> bool
	{
		if (!read_plc_state())
		{
			return false;
		}

		load_pos_from_actual();

		axis6_crawl.handle_ref = get_average_pos(handle_axis6, 0, 20);
		axis6_crawl.rot_ref = handle_axis6.fJoints2[1];
		axis6_crawl.base_rel = plc_act_pos[5];
		axis6_crawl.rot_base_rel = plc_act_pos[6];
		axis6_crawl.phase = CrawlState::Phase::Follow;
		axis6_crawl.phase_t0 = GetTickCount();
		axis6_crawl.wait_rearm = false;
		axis6_crawl.rearm_dir = 0;
		axis6_crawl.enabled = false;
		axis6_crawl.window_active = false;
		axis7_hold_rel = plc_act_pos[6];
		axis7_rot_reengage_required = false;
		axis7_rot_reengage_ref = handle_axis6.fJoints2[1];

		// 退出独立控制时，把轴6镜像基准校到当前点。
		// 如果轴1正在 Follow，需要扣除轴1已经产生的偏移量，
		// 否则恢复镜像随动的瞬间轴6会跳变。
		if (axis1_crawl.phase == CrawlState::Phase::Follow)
		{
			const double axis1_base_abs = axis1_crawl.base_rel + plc_init_pos[0];
			const double axis1_hand_delta_mm =
				(handle_axis1.fJoints2[0] - axis1_crawl.handle_ref) * cfg.k_handle_to_mm * cfg.axis_push_sign;
			const double axis1_cmd_abs = clamp_double(
				axis1_base_abs + axis1_hand_delta_mm,
				axis1_crawl.min_abs(), axis1_crawl.max_abs());
			const double axis1_delta_rel = axis1_cmd_abs - plc_init_pos[0] - axis1_crawl.base_rel;
			axis6_mirror_base_rel = plc_act_pos[5] - axis1_delta_rel;
		}
		else
		{
			axis6_mirror_base_rel = plc_act_pos[5];
		}

		pos[5] = plc_act_pos[5];
		pos[6] = plc_act_pos[6];
		return write_refer();
	};

	/// 全轴同步。用于启动、暂停恢复、self-check 完成等全局重置场景。
	auto sync_all = [&](int samples) -> bool
	{
		const double preserved_axis2_hold_rel = axis2_hold_rel;
		const double preserved_axis7_hold_rel = axis7_hold_rel;

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

		// 两个手柄同步取样——先采 582 再采 587 会导致 582 的零位
		// 比真正进入控制早约 samples×10 ms，首帧产生假位移。
		get_average_dual_pos(
			handle_axis1,
			handle_axis6,
			samples,
			axis1_crawl.handle_ref,
			axis1_crawl.rot_ref,
			axis6_crawl.handle_ref,
			axis6_crawl.rot_ref);

		// 取完零位后再读一次 PLC 实际位置，把基准和 refer 一起刷新到最新，
		// 避免校零窗口内手柄或轴状态变化造成启动瞬间跟随跳变。
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
		axis1_crawl.start_abs = axis1_start_abs();
		axis1_crawl.end_abs = axis1_end_abs();
		axis1_crawl.window_active = is_within_range(
			plc_act_pos[0] + plc_init_pos[0],
			axis1_crawl.min_abs(), axis1_crawl.max_abs(),
			cfg.crawl_arrive_tol_mm);
		axis1_crawl.phase = CrawlState::Phase::Follow;
		axis1_crawl.phase_t0 = GetTickCount();
		axis1_crawl.wait_rearm = false;
		axis1_crawl.rearm_dir = 0;
		axis1_crawl.enabled = true;
		axis2_hold_rel = preserved_axis2_hold_rel;
		axis2_rot_reengage_required = true;
		axis2_rot_reengage_ref = handle_axis1.fJoints2[1];

		axis3_base_rel = plc_act_pos[2];
		axis5_base_rel = plc_act_pos[4];
		axis6_mirror_base_rel = plc_act_pos[5];

		axis6_crawl.base_rel = plc_act_pos[5];
		axis6_crawl.rot_base_rel = preserved_axis7_hold_rel;
		axis6_crawl.start_abs = plc_act_pos[5] + plc_init_pos[5];
		axis6_crawl.end_abs = axis6_crawl.start_abs + cfg.axis6_independent_window_size_mm;
		axis6_crawl.window_active = false;
		axis6_crawl.phase = CrawlState::Phase::Follow;
		axis6_crawl.phase_t0 = GetTickCount();
		axis6_crawl.wait_rearm = false;
		axis6_crawl.rearm_dir = 0;
		axis6_crawl.enabled = false;
		axis7_hold_rel = preserved_axis7_hold_rel;
		axis7_rot_reengage_required = true;
		axis7_rot_reengage_ref = handle_axis6.fJoints2[1];

		return true;
	};

	auto clear_plc_reinit_req = [&]()
	{
		const bool clear_val = false;
		ads.ADSWrite(AdsSymbol::handle_reinit_req, sizeof(clear_val), (void*)&clear_val);
	};

	/// 捕获轴1 Follow 基准（无采样，直接取当前值）。
	/// 在蠕动窗口动态进入时调用——此时轴正在运动中，不能停下来做多次采样。
	auto capture_axis1_follow_baseline = [&]()
	{
		axis1_crawl.handle_ref = handle_axis1.fJoints2[0];
		axis1_crawl.rot_ref = handle_axis1.fJoints2[1];
		axis1_crawl.base_rel = plc_act_pos[0];
		// 旋转基准使用当前保持值而非 PLC 实际值，避免因 PLC 执行延迟
		// 导致 rot_base_rel 与 axis2_hold_rel 不一致，产生旋转轴瞬间跳变。
		axis1_crawl.rot_base_rel = axis2_hold_rel;
		// axis2_hold_rel 不重新赋值——保持上一帧的累积旋转位置
		axis3_base_rel = plc_act_pos[2];
		axis5_base_rel = plc_act_pos[4];
		if (!axis6_crawl.enabled)
		{
			axis6_mirror_base_rel = plc_act_pos[5];
		}
	};

	/// 进入独立导丝模式：轴1锁定当前位置，轴6切换为独立蠕动控制。
	auto enter_independent_guidewire_mode = [&]() -> bool
	{
		if (!read_plc_state())
		{
			return false;
		}

		load_pos_from_actual();
		independent_axis1_hold_rel = plc_act_pos[0];
		independent_axis2_hold_rel = plc_act_pos[1];
		independent_axis3_hold_rel = plc_act_pos[2];
		independent_axis5_hold_rel = plc_act_pos[4];
		axis7_hold_rel = plc_act_pos[6];
		axis7_rot_reengage_required = false;
		axis7_rot_reengage_ref = handle_axis6.fJoints2[1];

		axis6_crawl.handle_ref = get_average_pos(handle_axis6, 0, 20);
		axis6_crawl.rot_ref = handle_axis6.fJoints2[1];
		axis6_crawl.base_rel = plc_act_pos[5];
		axis6_crawl.rot_base_rel = plc_act_pos[6];
		axis6_crawl.start_abs = plc_act_pos[5] + plc_init_pos[5];
		axis6_crawl.end_abs = axis6_crawl.start_abs + cfg.axis6_independent_window_size_mm;
		axis6_crawl.target_abs = axis6_crawl.start_abs;
		axis6_crawl.phase = CrawlState::Phase::Follow;
		axis6_crawl.phase_t0 = GetTickCount();
		axis6_crawl.wait_rearm = false;
		axis6_crawl.rearm_dir = 0;
		axis6_crawl.window_active = true;
		axis6_crawl.enabled = true;
		axis1_fastmove_prev = false;

		pos[0] = plc_act_pos[0];
		pos[1] = plc_act_pos[1];
		pos[2] = plc_act_pos[2];
		pos[4] = plc_act_pos[4];
		pos[5] = plc_act_pos[5];
		pos[6] = plc_act_pos[6];
		return write_refer();
	};

	/// 进入协同导丝模式：轴6固定在当前位置，轴1正常蠕动，
	/// 轴1快速回退时轴6做反向补偿以保持导丝末端不动。
	auto enter_cooperative_guidewire_mode = [&]() -> bool
	{
		if (!read_plc_state())
		{
			return false;
		}

		load_pos_from_actual();
		cooperative_axis6_hold_rel = plc_act_pos[5];
		axis7_hold_rel = plc_act_pos[6];
		axis7_rot_reengage_required = false;
		axis7_rot_reengage_ref = handle_axis6.fJoints2[1];
		axis6_crawl.handle_ref = handle_axis6.fJoints2[0];
		axis6_crawl.rot_ref = handle_axis6.fJoints2[1];
		axis6_crawl.base_rel = plc_act_pos[5];
		axis6_crawl.rot_base_rel = plc_act_pos[6];
		axis6_crawl.phase = CrawlState::Phase::Follow;
		axis6_crawl.phase_t0 = GetTickCount();
		axis6_crawl.wait_rearm = false;
		axis6_crawl.rearm_dir = 0;
		axis6_crawl.window_active = false;
		axis6_crawl.enabled = false;
		axis1_fastmove_prev = false;

		pos[5] = plc_act_pos[5];
		pos[6] = plc_act_pos[6];
		return write_refer();
	};

	auto exit_guidewire_mode_to_normal = [&]() -> bool
	{
		guidewire_mode = GuidewireMode::None;
		axis1_fastmove_prev = false;
		return sync_all(20);
	};

	/// 启动准备序列：读取当前位置作为锁定基准，降低速度限制，开始状态机。
	auto start_startup_sequence = [&]() -> bool
	{
		if (!read_plc_state())
		{
			return false;
		}

		load_pos_from_actual();
		startup.axis1_hold_rel = plc_act_pos[0];
		startup.axis2_hold_rel = plc_act_pos[1];
		startup.axis3_hold_rel = plc_act_pos[2];
		startup.axis5_hold_rel = plc_act_pos[4];
		startup.axis6_hold_rel = plc_act_pos[5];
		startup.axis7_hold_rel = plc_act_pos[6];
		if (!startup.v_limit_scaled)
		{
			if (!read_v_limit())
			{
				return false;
			}

			copy_positions(plc_v_limit, startup.v_limit_backup, 7);

			double scaled_v_limit[7] = { 0 };
			copy_positions(plc_v_limit, scaled_v_limit, 7);
			// 启动序列期间轴3/5/6 降速至 50%，确保安全。
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
		axis1_fastmove_prev = false;
		startup.phase = StartupPhase::ReleaseClamps;
		startup.phase_t0 = GetTickCount();
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

	// ═══ 启动前 PLC 状态读取 ═══

	if (!read_plc_state())
	{
		std::cout << "Failed to read PLC state." << std::endl;
		handle_axis1.close();
		handle_axis6.close();
		return 0;
	}

	load_pos_from_actual();
	axis2_hold_rel = plc_act_pos[1];
	axis2_rot_reengage_required = false;
	axis2_rot_reengage_ref = handle_axis1.fJoints2[1];
	axis7_hold_rel = plc_act_pos[6];
	axis7_rot_reengage_required = false;
	axis7_rot_reengage_ref = handle_axis6.fJoints2[1];
	write_refer();

	// 检测 PLC 是否支持 self-check 流程
	bool self_check_done = true;
	bool has_self_check_flag = ads.ADSRead(AdsSymbol::self_check_done, sizeof(self_check_done), &self_check_done);
	if (has_self_check_flag)
	{
		ads.ADSRead(AdsSymbol::self_check_done, sizeof(self_check_done), &self_check_done);
	}

	bool control_active = !has_self_check_flag || self_check_done;
	bool last_self_check_done = self_check_done;
	bool handle_reinit_req = false;
	bool estop_hold_req = false;
	bool estop_hold_active = false;
	bool freeze_active = false;
	bool pause_pressed_prev = false;
	bool axis1_reverse_pressed_prev = false;
	bool axis6_reverse_pressed_prev = false;
	GuidewireMode requested_guidewire_mode_prev = GuidewireMode::None;
	bool axis1_fast_return = false;
	bool axis6_fast_retract = false;
	int loop_count = 0;

	unsigned char last_btn_axis1 = 0xFF;
	unsigned char last_btn_axis6 = 0xFF;

	std::cout << "Force feedback: OFF (press F to toggle)" << std::endl;
	apply_cmd_force(0.0);

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

	// ═══════════════════════════════════════════════════════════════════════════
	//  主控循环
	//  每帧约 10–20 ms（取决于手柄 poll 和 ADS 通讯耗时）。
	// ═══════════════════════════════════════════════════════════════════════════

	while (true)
	{
		handle_axis1.poll();
		handle_axis6.poll();

		++loop_count;
		axis1_fast_return = false;
		axis6_fast_retract = false;

		// --- 按钮状态采样 ---
		const bool pause_pressed = (handle_axis1.buttons2 & axis1_pause_button_mask) != 0;
		const bool axis1_reverse_pressed = (handle_axis1.buttons2 & axis1_reverse_button_mask) != 0;
		const bool axis6_reverse_pressed = (handle_axis6.buttons2 & axis6_reverse_button_mask) != 0;
		const bool guidewire_independent_pressed = (handle_axis6.buttons2 & axis6_independent_button_mask) != 0;
		const bool guidewire_cooperative_pressed = (handle_axis6.buttons2 & axis6_cooperative_button_mask) != 0;
		const GuidewireMode requested_guidewire_mode = guidewire_cooperative_pressed
			? GuidewireMode::Cooperative
			: (guidewire_independent_pressed ? GuidewireMode::Independent : GuidewireMode::None);
		const bool startup_sequence_active = startup.is_active();

		// --- 暂停按钮边沿检测 ---
		// 582 手柄 b6 的上升沿进入暂停（冻结所有输出），下降沿恢复。
		// 恢复时需要 sync_all 重建手柄零位，防止松手期间的漂移导致恢复瞬间跳变。
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

		// --- 反向按钮边沿检测 ---
		// 反向按钮切换时需要 sync 重建基准——因为反向模式改变了位置计算逻辑
		// （正常模式向前推进，反向模式只允许向右限位方向退回），
		// 如果不重新同步，切换瞬间手柄当前偏移会产生错误的位置跳变。
		if (!freeze_active && !estop_hold_active && !startup_sequence_active && control_active)
		{
			if (guidewire_mode == GuidewireMode::None && axis1_reverse_pressed != axis1_reverse_pressed_prev)
			{
				if (sync_axis1(20, false, 0))
				{
					std::cout << "Axis1 reverse-only mode: " << (axis1_reverse_pressed ? "ON" : "OFF") << std::endl;
				}
				else
				{
					std::cout << "Axis1 reverse-only mode sync failed." << std::endl;
				}
			}

			if (guidewire_mode == GuidewireMode::Independent && axis6_reverse_pressed != axis6_reverse_pressed_prev)
			{
				if (sync_axis6(20, false, false, 0))
				{
					std::cout << "Axis6 reverse-only mode: " << (axis6_reverse_pressed ? "ON" : "OFF") << std::endl;
				}
				else
				{
					std::cout << "Axis6 reverse-only mode sync failed." << std::endl;
				}
			}
		}
		axis1_reverse_pressed_prev = axis1_reverse_pressed;
		axis6_reverse_pressed_prev = axis6_reverse_pressed;

		// --- PLC 急停/保持轮询（每 10 帧）---
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
					}
					estop_hold_active = false;
				}
			}
		}

		// --- 暂停期间：清除力反馈状态 ---
		if (freeze_active)
		{
			ff.clear_output();
			apply_cmd_force(0.0);
		}

		// --- 键盘输入处理 ---
		if (_kbhit())
		{
			const int ch = _getch();
			if (ch == 'c' || ch == 'C')
			{
				// 'C': 跳过启动准备，直接进入控制
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
				// 'S': 开始启动准备序列
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
				// 'F': 切换力反馈开关
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
				_getch(); // 消耗功能键第二字节
			}
		}

		// --- 导丝模式切换 ---
		// 587 手柄的 b6/b7 按钮控制导丝模式。
		// 仅在按钮状态变化时处理（边沿触发），且需要满足一系列前置条件。
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
							axis1_fastmove_prev = false;
							std::cout << "Guidewire mode exit failed: ADS sync failed." << std::endl;
						}
					}
					else
					{
						guidewire_mode = GuidewireMode::None;
						axis6_crawl.enabled = false;
						axis1_fastmove_prev = false;
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
				if (requested_guidewire_mode == GuidewireMode::Independent)
				{
					mode_ok = enter_independent_guidewire_mode();
					if (mode_ok)
					{
						guidewire_mode = GuidewireMode::Independent;
						std::cout << "Guidewire mode: INDEPENDENT." << std::endl;
					}
				}
				else
				{
					mode_ok = enter_cooperative_guidewire_mode();
					if (mode_ok)
					{
						guidewire_mode = GuidewireMode::Cooperative;
						std::cout << "Guidewire mode: COOPERATIVE." << std::endl;
					}
				}

				if (!mode_ok)
				{
					std::cout << "Guidewire mode switch failed: ADS sync failed." << std::endl;
				}
			}
		}
		requested_guidewire_mode_prev = requested_guidewire_mode;

		// --- PLC self-check & handle reinit 轮询（每 50 帧）---
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
					axis1_fastmove_prev = false;
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

		// --- 按钮变化日志 ---
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

		// --- 力反馈处理 ---
		process_force_feedback(
			ff, ads, handle_axis1, control_active, freeze_active, estop_hold_active,
			loop_count, cfg.axial_force_axis, cfg.axial_force_sign);

		// --- 自动重同步 ---
		const bool motion_startup_active = startup.is_active();
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

		// ─── 运动指令计算 ───
		// 每帧默认气缸状态：导管前松后夹(正常跟随配置)，导丝微松。
		unsigned short cylinder1_cmd = cyl.cyl1_open;
		unsigned short cylinder2_cmd = cyl.cyl2_clamp;
		unsigned short cylinder3_cmd = cyl.cyl3_follow_release;
		unsigned short cylinder4_cmd = cyl.cyl4_follow_release;

		if (!freeze_active && !estop_hold_active && (control_active || motion_startup_active) && read_plc_state())
		{
			load_pos_from_actual();

			const DWORD now_ms = GetTickCount();

			// 轴1 运动学量（绝对坐标 = 相对位置 + init_pos）
			const double axis1_abs = plc_act_pos[0] + plc_init_pos[0];
			const double axis1_base_abs = axis1_crawl.base_rel + plc_init_pos[0];
			double axis1_hand_delta_mm = (handle_axis1.fJoints2[0] - axis1_crawl.handle_ref) * cfg.k_handle_to_mm * cfg.axis_push_sign;
			// "far" 和 "near"：相对于右限位的远近。
			// 用距离右限位的远近来判断推进方向——rightlimit 是导管/导丝可到达的最远端，
			// "远离 rightlimit" 即向前推进（away），"靠近 rightlimit" 即后退（toward）。
			const double axis1_start_right_dist = std::abs(plc_rightlimit[0] - axis1_crawl.start_abs);
			const double axis1_end_right_dist = std::abs(plc_rightlimit[0] - axis1_crawl.end_abs);
			const double axis1_far_abs = (axis1_start_right_dist >= axis1_end_right_dist) ? axis1_crawl.start_abs : axis1_crawl.end_abs;
			const double axis1_near_abs = (axis1_start_right_dist >= axis1_end_right_dist) ? axis1_crawl.end_abs : axis1_crawl.start_abs;
			const double axis1_right_soft_abs = plc_rightlimit[0] - cfg.axis1_right_soft_margin_mm;
			const double axis1_min_abs = axis1_crawl.min_abs();
			const double axis1_max_abs = axis1_crawl.max_abs();

			// 轴6 运动学量
			const double axis6_abs = plc_act_pos[5] + plc_init_pos[5];
			const double axis6_base_abs = axis6_crawl.base_rel + plc_init_pos[5];
			double axis6_hand_delta_mm = (handle_axis6.fJoints2[0] - axis6_crawl.handle_ref) * cfg.k_handle_to_mm * cfg.axis_push_sign;
			const double axis6_start_right_dist = std::abs(plc_rightlimit[5] - axis6_crawl.start_abs);
			const double axis6_end_right_dist = std::abs(plc_rightlimit[5] - axis6_crawl.end_abs);
			const double axis6_far_abs = (axis6_start_right_dist >= axis6_end_right_dist) ? axis6_crawl.start_abs : axis6_crawl.end_abs;
			const double axis6_near_abs = (axis6_start_right_dist >= axis6_end_right_dist) ? axis6_crawl.end_abs : axis6_crawl.start_abs;
			const double axis6_right_soft_abs = plc_rightlimit[5] - cfg.axis6_right_soft_margin_mm;
			auto compute_axis7_cmd_rel = [&]() -> double
			{
				if (axis7_rot_reengage_required)
				{
					const double axis7_rot_delta_deg =
						(handle_axis6.fJoints2[1] - axis7_rot_reengage_ref) * cfg.axis_rot_scale_deg;
					if (std::abs(axis7_rot_delta_deg) >= cfg.axis2_rot_reengage_deadband_deg)
					{
						axis7_rot_reengage_required = false;
						axis6_crawl.rot_ref = handle_axis6.fJoints2[1];
						axis6_crawl.rot_base_rel = axis7_hold_rel;
					}
					return axis7_hold_rel;
				}

				const double axis7_follow_rel =
					axis6_crawl.rot_base_rel + (handle_axis6.fJoints2[1] - axis6_crawl.rot_ref) * cfg.axis_rot_scale_deg;
				axis7_hold_rel = axis7_follow_rel;
				return axis7_follow_rel;
			};
			const double axis7_cmd_rel =
				(guidewire_mode == GuidewireMode::None) ? axis7_hold_rel : compute_axis7_cmd_rel();

			// ─────────────────────────────────────────────────────────────────
			//  分支一：启动准备状态机
			// ─────────────────────────────────────────────────────────────────
			if (motion_startup_active)
			{
				// 启动序列期间，所有轴保持启动前的锁定位置（除正在运动的轴外）
				pos[0] = startup.axis1_hold_rel;
				pos[1] = startup.axis2_hold_rel;
				pos[2] = startup.axis3_hold_rel;
				pos[4] = startup.axis5_hold_rel;
				pos[5] = startup.axis6_hold_rel;
				pos[6] = startup.axis7_hold_rel;

				const double startup_axis5_ready_abs = plc_rightlimit[4] - cfg.startup_axis5_ready_from_right_mm;
				const double startup_axis3_ready_abs = plc_rightlimit[2] - cfg.startup_axis3_ready_from_right_mm;
				const double startup_axis3_follow_abs = plc_rightlimit[2] - cfg.startup_axis3_follow_from_right_mm;
				const double startup_axis6_follow_abs = plc_rightlimit[5] - cfg.startup_axis6_follow_from_right_mm;
				const double axis5_abs = plc_act_pos[4] + plc_init_pos[4];
				const double axis3_abs = plc_act_pos[2] + plc_init_pos[2];

				// 启动准备各阶段：顺序释放/夹紧气缸，逐步将各轴移到操作就绪位。
				// 每个阶段完成条件是"到位"或"延时到"，然后进入下一阶段。
				if (startup.phase == StartupPhase::ReleaseClamps)
				{
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_open;
					cylinder3_cmd = cfg.startup_cyl3_open;
					cylinder4_cmd = cyl.cyl4_open;
					if ((now_ms - startup.phase_t0) >= cfg.startup_clamp_settle_delay_ms)
					{
						startup.phase = StartupPhase::MoveAxis5ToReady;
					}
				}
				else if (startup.phase == StartupPhase::MoveAxis5ToReady)
				{
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_open;
					cylinder3_cmd = cfg.startup_cyl3_open;
					cylinder4_cmd = cyl.cyl4_open;
					pos[4] = startup_axis5_ready_abs - plc_init_pos[4];
					if (std::abs(axis5_abs - startup_axis5_ready_abs) <= cfg.crawl_arrive_tol_mm)
					{
						startup.phase = StartupPhase::ClampCylinder3Wait;
						startup.phase_t0 = now_ms;
					}
				}
				else if (startup.phase == StartupPhase::ClampCylinder3Wait)
				{
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_open;
					cylinder3_cmd = cyl.cyl3_clamp;
					cylinder4_cmd = cyl.cyl4_open;
					pos[4] = startup_axis5_ready_abs - plc_init_pos[4];
					if ((now_ms - startup.phase_t0) >= cfg.startup_clamp_settle_delay_ms)
					{
						startup.axis3_phase2_base_rel = plc_act_pos[2];
						startup.axis5_phase2_base_rel = plc_act_pos[4];
						startup.phase = StartupPhase::MoveAxis3ToReady;
					}
				}
				else if (startup.phase == StartupPhase::MoveAxis3ToReady)
				{
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_open;
					cylinder3_cmd = cyl.cyl3_clamp;
					cylinder4_cmd = cyl.cyl4_open;
					const double axis3_target_rel = startup_axis3_ready_abs - plc_init_pos[2];
					const double axis35_delta_rel = axis3_target_rel - startup.axis3_phase2_base_rel;
					pos[2] = axis3_target_rel;
					pos[4] = startup.axis5_phase2_base_rel + axis35_delta_rel;
					if (std::abs(axis3_abs - startup_axis3_ready_abs) <= cfg.crawl_arrive_tol_mm)
					{
						startup.phase = StartupPhase::ClampCylinder2Wait;
						startup.phase_t0 = now_ms;
					}
				}
				else if (startup.phase == StartupPhase::ClampCylinder2Wait)
				{
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_clamp;
					cylinder3_cmd = cyl.cyl3_clamp;
					cylinder4_cmd = cyl.cyl4_open;
					const double axis3_target_rel = startup_axis3_ready_abs - plc_init_pos[2];
					const double axis35_delta_rel = axis3_target_rel - startup.axis3_phase2_base_rel;
					pos[2] = axis3_target_rel;
					pos[4] = startup.axis5_phase2_base_rel + axis35_delta_rel;
					if ((now_ms - startup.phase_t0) >= cfg.startup_clamp_settle_delay_ms)
					{
						startup.axis3_phase3_base_rel = plc_act_pos[2];
						startup.axis5_phase3_base_rel = plc_act_pos[4];
						startup.phase = StartupPhase::MoveAxis3AndAxis5;
					}
				}
				else if (startup.phase == StartupPhase::MoveAxis3AndAxis5)
				{
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_clamp;
					cylinder3_cmd = cyl.cyl3_clamp;
					cylinder4_cmd = cyl.cyl4_open;
					const double axis3_target_rel = startup_axis3_follow_abs - plc_init_pos[2];
					const double axis35_delta_rel = axis3_target_rel - startup.axis3_phase3_base_rel;
					pos[2] = axis3_target_rel;
					pos[4] = startup.axis5_phase3_base_rel + axis35_delta_rel;
					if (std::abs(axis3_abs - startup_axis3_follow_abs) <= cfg.crawl_arrive_tol_mm)
					{
						startup.axis3_phase3_base_rel = plc_act_pos[2];
						startup.axis5_phase3_base_rel = plc_act_pos[4];
						startup.axis6_phase3_base_rel = plc_act_pos[5];
						startup.phase = StartupPhase::MoveAxis3Axis5Axis6;
					}
				}
				else if (startup.phase == StartupPhase::MoveAxis3Axis5Axis6)
				{
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_clamp;
					cylinder3_cmd = cyl.cyl3_clamp;
					cylinder4_cmd = cyl.cyl4_open;
					const double axis6_target_rel = startup_axis6_follow_abs - plc_init_pos[5];
					const double axis356_delta_rel = axis6_target_rel - startup.axis6_phase3_base_rel;
					pos[2] = startup.axis3_phase3_base_rel + axis356_delta_rel;
					pos[4] = startup.axis5_phase3_base_rel + axis356_delta_rel;
					pos[5] = axis6_target_rel;
					if (std::abs(axis6_abs - startup_axis6_follow_abs) <= cfg.crawl_arrive_tol_mm)
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
			// ─────────────────────────────────────────────────────────────────
			//  分支二：独立导丝模式
			//  轴1/2/3/5 锁定在进入独立模式时的位置，
			//  轴6 由 587 手柄独立控制，有自己的蠕动窗口。
			//  与普通模式的区别：轴6有独立的 crawl 状态机，气缸3/4做蠕动切换。
			// ─────────────────────────────────────────────────────────────────
			else if (guidewire_mode == GuidewireMode::Independent)
			{
				pos[0] = independent_axis1_hold_rel;
				pos[1] = independent_axis2_hold_rel;
				pos[2] = independent_axis3_hold_rel;
				pos[4] = independent_axis5_hold_rel;
				pos[6] = axis7_cmd_rel;

				// 轴6 单向前进行程重置：同轴1逻辑，回拉时重置手柄基准而非后退。
				if (!axis6_reverse_pressed && axis6_hand_delta_mm > cfg.crawl_trigger_deadband_mm)
				{
					axis6_crawl.handle_ref = handle_axis6.fJoints2[0];
					axis6_hand_delta_mm = 0.0;
				}

				check_crawl_rearm(axis6_crawl, axis6_hand_delta_mm, cfg.crawl_rearm_threshold_mm);

				if (axis6_crawl.phase == CrawlState::Phase::Follow)
				{
					// --- 轴6 Follow：手柄跟随计算 ---
					const double axis6_raw_cmd_abs = axis6_base_abs + axis6_hand_delta_mm;
					const double axis6_base_right_dist = plc_rightlimit[5] - axis6_base_abs;
					const double axis6_raw_right_dist = plc_rightlimit[5] - axis6_raw_cmd_abs;
					const bool axis6_request_away =
						(std::abs(axis6_hand_delta_mm) > cfg.crawl_trigger_deadband_mm) &&
						(axis6_raw_right_dist > axis6_base_right_dist);

					double axis6_cmd_abs = axis6_base_abs;
					if (axis6_reverse_pressed)
					{
						// 反向模式：只允许向右限位方向移动，不超过软限位。
						const double axis6_reverse_cmd_abs = clamp_double(
							axis6_raw_cmd_abs,
							(axis6_base_abs < axis6_right_soft_abs) ? axis6_base_abs : axis6_right_soft_abs,
							(axis6_base_abs > axis6_right_soft_abs) ? axis6_base_abs : axis6_right_soft_abs);
						const double axis6_reverse_right_dist = plc_rightlimit[5] - axis6_reverse_cmd_abs;
						if (axis6_reverse_right_dist <= axis6_base_right_dist)
						{
							axis6_cmd_abs = axis6_reverse_cmd_abs;
						}
					}
					else
					{
						// 正常模式：在蠕动窗口内前进。
						const double axis6_forward_cmd_abs = clamp_double(
							axis6_raw_cmd_abs,
							(axis6_near_abs < axis6_far_abs) ? axis6_near_abs : axis6_far_abs,
							(axis6_near_abs > axis6_far_abs) ? axis6_near_abs : axis6_far_abs);
						const double axis6_forward_right_dist = plc_rightlimit[5] - axis6_forward_cmd_abs;
						if (axis6_forward_right_dist >= axis6_base_right_dist)
						{
							axis6_cmd_abs = axis6_forward_cmd_abs;
						}
					}

					pos[5] = axis6_cmd_abs - plc_init_pos[5];
					cylinder3_cmd = cyl.cyl3_open;
					cylinder4_cmd = cyl.cyl4_clamp;

					// 蠕动触发：到达窗口远端且手柄在推 → 进入 SwitchWait
					if (!axis6_reverse_pressed && !axis6_crawl.wait_rearm)
					{
						if (axis6_request_away && (std::abs(axis6_abs - axis6_far_abs) <= cfg.crawl_arrive_tol_mm))
						{
							axis6_crawl.target_abs = axis6_near_abs;
							axis6_crawl.phase = CrawlState::Phase::SwitchWait;
							axis6_crawl.phase_t0 = now_ms;
							axis6_crawl.rearm_dir = (axis6_hand_delta_mm >= 0.0) ? 1 : -1;
						}
					}
				}
				else
				{
					// --- 轴6 蠕动非 Follow 阶段（SwitchWait/FastMove/ClampWait/RestoreWait）---
					// hold_in_settle=true：轴6 在 ClampWait/RestoreWait 需要显式保持目标位，
					// 因为独立模式下轴6是唯一运动轴，load_from_actual 后 pos[5] 会被覆盖。
					auto sr = process_crawl_settle_phases(
						axis6_crawl, axis6_abs, plc_init_pos[5], &pos[5],
						true, now_ms,
						cfg.crawl_switch_delay_ms, cfg.crawl_clamp_delay_ms,
						cfg.crawl_arrive_tol_mm, cyl.cyl3_clamp, cyl.cyl3_open);
					cylinder3_cmd = sr.cyl_front;
					cylinder4_cmd = sr.cyl_rear;
					axis6_fast_retract = sr.fast_flag;
					if (sr.sync_needed)
					{
						sync_axis6(20, false, true, axis6_crawl.rearm_dir);
					}
				}
			}
			// ─────────────────────────────────────────────────────────────────
			//  分支三：普通导管模式 / 协同导丝模式
			//  轴1 由 582 手柄控制，有蠕动窗口。
			//  普通模式：轴6 镜像跟随轴1（pos[5] = mirror_base + delta）。
			//  协同模式：轴6 在轴1 Follow 时保持不动，在轴1 FastMove 时做反向补偿，
			//            保证导丝末端在导管快速回退时不被拖动。
			// ─────────────────────────────────────────────────────────────────
			else
			{
				// 蠕动窗口动态进入检测：轴1 从窗口外运动进入窗口范围时，
				// 重新捕获基准并激活蠕动逻辑。
				const bool axis1_now_in_window = is_within_range(axis1_abs, axis1_min_abs, axis1_max_abs, cfg.crawl_arrive_tol_mm);
				if (!axis1_crawl.window_active && axis1_now_in_window)
				{
					capture_axis1_follow_baseline();
					axis1_crawl.window_active = true;
					std::cout << "Axis1 entered crawl window; crawl logic enabled." << std::endl;
				}

				// 单向前进行程重置：操作者把手柄往回拉（hand_delta > 0，即靠近右限位方向）时，
				// 不让轴1后退，而是把手柄基准重置到当前位置，使后续从新位置继续推时仍能前进。
				// 注意：hand_delta > 0 对应物理上的"回拉"——因为 axis_push_sign = -1，
				// 手柄物理正方向（推）→ 负 delta（远离右限位），
				// 手柄物理负方向（拉）→ 正 delta（靠近右限位）。
				if (guidewire_mode == GuidewireMode::None &&
					!axis1_reverse_pressed &&
					axis1_hand_delta_mm > cfg.crawl_trigger_deadband_mm)
				{
					axis1_crawl.handle_ref = handle_axis1.fJoints2[0];
					axis1_hand_delta_mm = 0.0;
				}

				check_crawl_rearm(axis1_crawl, axis1_hand_delta_mm, cfg.crawl_rearm_threshold_mm);

				if (axis1_crawl.phase == CrawlState::Phase::Follow)
				{
					// --- 轴1 Follow：手柄跟随 + 多轴联动计算 ---
					const double axis1_raw_cmd_abs = axis1_base_abs + axis1_hand_delta_mm;
					const bool axis1_normal_mode = guidewire_mode == GuidewireMode::None;
					const double axis1_base_right_dist = plc_rightlimit[0] - axis1_base_abs;
					const double axis1_raw_right_dist = plc_rightlimit[0] - axis1_raw_cmd_abs;
					// "request away" = 手柄在推且指令点比基准更远离右限位（即向前推进）
					const bool axis1_request_away =
						axis1_normal_mode &&
						(std::abs(axis1_hand_delta_mm) > cfg.crawl_trigger_deadband_mm) &&
						(axis1_raw_right_dist > axis1_base_right_dist);
					const bool axis1_push_request = axis1_hand_delta_mm > cfg.crawl_trigger_deadband_mm;
					const bool axis1_pull_request = axis1_hand_delta_mm < -cfg.crawl_trigger_deadband_mm;

					double axis1_cmd_abs = axis1_base_abs;
					if (axis1_normal_mode && axis1_reverse_pressed)
					{
						// 反向回退模式：限制在 [base, right_soft] 之间
						const double axis1_reverse_cmd_abs = clamp_double(
							axis1_raw_cmd_abs,
							(axis1_base_abs < axis1_right_soft_abs) ? axis1_base_abs : axis1_right_soft_abs,
							(axis1_base_abs > axis1_right_soft_abs) ? axis1_base_abs : axis1_right_soft_abs);
						const double axis1_reverse_right_dist = plc_rightlimit[0] - axis1_reverse_cmd_abs;
						if (axis1_reverse_right_dist <= axis1_base_right_dist)
						{
							axis1_cmd_abs = axis1_reverse_cmd_abs;
						}
					}
					else if (axis1_normal_mode)
					{
						// 普通前进模式：窗口内受限，窗口外自由
						const double axis1_forward_cmd_abs = axis1_crawl.window_active
							? clamp_double(
								axis1_raw_cmd_abs,
								(axis1_near_abs < axis1_far_abs) ? axis1_near_abs : axis1_far_abs,
								(axis1_near_abs > axis1_far_abs) ? axis1_near_abs : axis1_far_abs)
							: axis1_raw_cmd_abs;
						const double axis1_forward_right_dist = plc_rightlimit[0] - axis1_forward_cmd_abs;
						if ((loop_count % 50) == 0 && !axis1_reverse_pressed)
						{
							std::cout << "[DIR] R=" << plc_rightlimit[0]
								<< " B=" << axis1_base_abs
								<< " C=" << axis1_forward_cmd_abs
								<< " delta=" << axis1_hand_delta_mm
								<< " base_dist=" << axis1_base_right_dist
								<< " cmd_dist=" << std::abs(plc_rightlimit[0] - axis1_forward_cmd_abs)
								<< " pass=" << (axis1_forward_right_dist >= axis1_base_right_dist)
								<< " win=" << axis1_crawl.window_active
								<< std::endl;
						}
						if (axis1_forward_right_dist >= axis1_base_right_dist)
						{
							axis1_cmd_abs = axis1_forward_cmd_abs;
						}
					}
					else
					{
						// 协同模式下轴1的跟随（无反向限制，但有窗口限制）
						axis1_cmd_abs = axis1_crawl.window_active
							? clamp_double(axis1_raw_cmd_abs, axis1_min_abs, axis1_max_abs)
							: axis1_raw_cmd_abs;
					}

					// 计算位置指令：所有轴使用相对位置(rel)写入 pos[]，
					// PLC 内部会加上 init_pos 得到绝对位置。
					const double axis1_delta_rel = axis1_cmd_abs - plc_init_pos[0] - axis1_crawl.base_rel;
					pos[0] = axis1_cmd_abs - plc_init_pos[0];
					if (axis2_rot_reengage_required)
					{
						pos[1] = axis2_hold_rel;
						const double axis2_rot_delta_deg =
							(handle_axis1.fJoints2[1] - axis2_rot_reengage_ref) * cfg.axis_rot_scale_deg;
						if (std::abs(axis2_rot_delta_deg) >= cfg.axis2_rot_reengage_deadband_deg)
						{
							axis2_rot_reengage_required = false;
							axis1_crawl.rot_ref = handle_axis1.fJoints2[1];
							axis1_crawl.rot_base_rel = axis2_hold_rel;
						}
					}
					else
					{
						pos[1] = axis1_crawl.rot_base_rel + (handle_axis1.fJoints2[1] - axis1_crawl.rot_ref) * cfg.axis_rot_scale_deg;
						axis2_hold_rel = pos[1];
					}
					// 轴3、轴5 与轴1 联动——导管体跟随导管头同步平移
					pos[2] = axis3_base_rel + axis1_delta_rel;
					pos[4] = axis5_base_rel + axis1_delta_rel;

					if (guidewire_mode == GuidewireMode::Cooperative)
					{
						// 协同模式 Follow 阶段：轴6 保持不动，轴7 跟旋转手柄
						pos[5] = cooperative_axis6_hold_rel;
						pos[6] = axis7_cmd_rel;
						cylinder3_cmd = cyl.cyl3_open;
						cylinder4_cmd = cyl.cyl4_clamp;
					}
					else
					{
						// 普通模式：轴6 镜像跟随轴1 的平移量
						pos[5] = axis6_mirror_base_rel + axis1_delta_rel;
						cylinder3_cmd = cyl.cyl3_follow_release;
						cylinder4_cmd = cyl.cyl4_follow_release;
					}

					// 蠕动触发判定
					if (axis1_crawl.window_active && !axis1_crawl.wait_rearm)
					{
						if (axis1_normal_mode)
						{
							// 普通模式：到达远端且手柄推离 rightlimit → 蠕动
							if (!axis1_reverse_pressed && axis1_request_away && (std::abs(axis1_abs - axis1_far_abs) <= cfg.crawl_arrive_tol_mm))
							{
								axis1_crawl.target_abs = axis1_near_abs;
								axis1_crawl.phase = CrawlState::Phase::SwitchWait;
								axis1_crawl.phase_t0 = now_ms;
								axis1_crawl.rearm_dir = (axis1_hand_delta_mm >= 0.0) ? 1 : -1;
							}
						}
						// 协同模式：到达 end → 回 start，到达 start → 回 end
						else if (axis1_push_request && (std::abs(axis1_abs - axis1_crawl.end_abs) <= cfg.crawl_arrive_tol_mm))
						{
							axis1_crawl.target_abs = axis1_crawl.start_abs;
							axis1_crawl.phase = CrawlState::Phase::SwitchWait;
							axis1_crawl.phase_t0 = now_ms;
							axis1_crawl.rearm_dir = 1;
						}
						else if (axis1_pull_request && (std::abs(axis1_abs - axis1_crawl.start_abs) <= cfg.crawl_arrive_tol_mm))
						{
							axis1_crawl.target_abs = axis1_crawl.end_abs;
							axis1_crawl.phase = CrawlState::Phase::SwitchWait;
							axis1_crawl.phase_t0 = now_ms;
							axis1_crawl.rearm_dir = -1;
						}
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::SwitchWait)
				{
					// 轴1的时序恢复为备份版的显式实现：
					// 1. 电缸1夹紧、电缸2松开；
					// 2. 等待切换延时结束后，下一步才允许轴1进入快速回退。
					pos[1] = axis2_hold_rel;
					cylinder1_cmd = cyl.cyl1_clamp;
					cylinder2_cmd = cyl.cyl2_open;
					if ((now_ms - axis1_crawl.phase_t0) >= cfg.crawl_switch_delay_ms)
					{
						axis1_crawl.phase = CrawlState::Phase::FastMove;
						axis1_crawl.phase_t0 = now_ms;
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::FastMove)
				{
					pos[1] = axis2_hold_rel;
					cylinder1_cmd = cyl.cyl1_clamp;
					cylinder2_cmd = cyl.cyl2_open;
					axis1_fast_return = true;
					pos[0] = axis1_crawl.target_abs - plc_init_pos[0];
					if (std::abs(axis1_abs - axis1_crawl.target_abs) <= cfg.crawl_arrive_tol_mm)
					{
						axis1_crawl.phase = CrawlState::Phase::ClampWait;
						axis1_crawl.phase_t0 = now_ms;
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::ClampWait)
				{
					pos[1] = axis2_hold_rel;
					cylinder1_cmd = cyl.cyl1_clamp;
					cylinder2_cmd = cyl.cyl2_clamp;
					if ((now_ms - axis1_crawl.phase_t0) >= cfg.crawl_clamp_delay_ms)
					{
						axis1_crawl.phase = CrawlState::Phase::RestoreWait;
						axis1_crawl.phase_t0 = now_ms;
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::RestoreWait)
				{
					pos[1] = axis2_hold_rel;
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_clamp;
					if ((now_ms - axis1_crawl.phase_t0) >= cfg.crawl_switch_delay_ms)
					{
						sync_axis1(20, true, axis1_crawl.rearm_dir);
					}
				}

				// --- 协同模式下轴6的 FastMove 补偿 ---
				// 当轴1进入 FastMove（快速回退），导管体在移动但夹持切换后导丝不应跟着动。
				// 为此轴6做反向补偿：pos[5] += (轴1 FastMove 起点 - 轴1 当前位置)。
				// 进入 FastMove 时记录起点，退出时把补偿量固化到 hold_rel 中。
				const bool axis1_fastmove_now = axis1_crawl.phase == CrawlState::Phase::FastMove;
				if (guidewire_mode == GuidewireMode::Cooperative)
				{
					if (axis1_fastmove_now && !axis1_fastmove_prev)
					{
						axis1_fastmove_start_abs = axis1_abs;
					}

					if (!axis1_fastmove_now && axis1_fastmove_prev)
					{
						cooperative_axis6_hold_rel += (axis1_fastmove_start_abs - axis1_abs);
					}

					if (axis1_fastmove_now)
					{
						const double axis1_comp_delta = axis1_fastmove_start_abs - axis1_abs;
						pos[5] = cooperative_axis6_hold_rel + axis1_comp_delta;
						pos[6] = axis7_cmd_rel;
						cylinder3_cmd = cyl.cyl3_clamp;
						cylinder4_cmd = cyl.cyl4_open;
						axis6_fast_retract = true;
					}
					else
					{
						pos[5] = cooperative_axis6_hold_rel;
						pos[6] = axis7_cmd_rel;
						// SwitchWait 阶段：导丝前夹后松（准备快速回退）
						if (axis1_crawl.phase == CrawlState::Phase::SwitchWait)
						{
							cylinder3_cmd = cyl.cyl3_clamp;
							cylinder4_cmd = cyl.cyl4_open;
						}
						// ClampWait/RestoreWait：前后都夹或前松后夹
						else if (axis1_crawl.phase == CrawlState::Phase::ClampWait ||
								 axis1_crawl.phase == CrawlState::Phase::RestoreWait)
						{
							cylinder3_cmd = cyl.cyl3_clamp;
							cylinder4_cmd = cyl.cyl4_clamp;
						}
					}

					axis1_fastmove_prev = axis1_fastmove_now;
				}
				else
				{
					axis1_fastmove_prev = false;
				}
			}

			write_refer();
		}

		// --- 输出写入 ---
		if (!freeze_active && (control_active || motion_startup_active))
		{
			ads.ADSWrite(AdsSymbol::cylinder1_value, sizeof(cylinder1_cmd), &cylinder1_cmd);
			ads.ADSWrite(AdsSymbol::cylinder2_value, sizeof(cylinder2_cmd), &cylinder2_cmd);
			ads.ADSWrite(AdsSymbol::cylinder3_value, sizeof(cylinder3_cmd), &cylinder3_cmd);
			ads.ADSWrite(AdsSymbol::cylinder4_value, sizeof(cylinder4_cmd), &cylinder4_cmd);
		}

		ads.ADSWrite(AdsSymbol::axis1_fast_return, sizeof(axis1_fast_return), &axis1_fast_return);
		ads.ADSWrite(AdsSymbol::axis6_fast_retract, sizeof(axis6_fast_retract), &axis6_fast_retract);
	}

	handle_axis1.close();
	handle_axis6.close();
	return 0;
}
