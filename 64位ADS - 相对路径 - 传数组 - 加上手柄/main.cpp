#include "control_types.h"
#include "force_feedback.h"
#include "force_logger.h"
#include "guidewire_mode.h"
#include "motion_sync.h"
#include "plc_io.h"
#include "startup_sequence.h"
#include "tcp_force_daq.h"

#include <cmath>
#include <conio.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <windows.h>

// 文件职责说明：
// 1) 本文件是 ADS 控制程序入口与主循环调度层。
// 2) 运动同步、导丝模式、启动流程、力反馈与 ADS 读写已拆分到独立模块。
// 3) 本文件保留原有业务时序与按键行为，不改变控制策略。

int main(int argc, char* argv[])
{
	setup_console_utf8();

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
			std::cout << "手柄初始化失败，序列号: " << test_serial << std::endl;
			return 0;
		}

		std::cout << "=== 按键测试模式 ===" << std::endl;
		std::cout << "序列号: " << test_serial << std::endl;
		std::cout << "请按下按键查看位掩码。" << std::endl;
		std::cout << "按 ESC 或 q 退出。" << std::endl;

		unsigned char last_btn = 0xFF;
		while (true)
		{
			test_handle.poll();
			const unsigned char cur_btn = test_handle.buttons2;

			if (cur_btn != last_btn)
			{
				std::cout << "按键: 0x" << std::hex << static_cast<int>(cur_btn) << std::dec << " | 位: ";
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
			std::cout << "手柄初始化失败，序列号: " << test_serial << std::endl;
			return 0;
		}

		std::cout << "=== 手柄监视模式 ===" << std::endl;
		std::cout << "序列号: " << test_serial << std::endl;
		std::cout << "按 ESC 或 q 退出。" << std::endl;

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
	const unsigned char axis6_independent_button_mask = cfg.btn_b6;
	const unsigned char axis6_cooperative_button_mask = cfg.btn_b0;
	// Handle587（本轮重映射约定）：
	// - b6: 导丝模式主键（独立）
	// - b0: 导丝反向键（仅在 b6 未按下时作为反向判定）
	// - b7: 本轮不参与导丝模式与反向判定
	// Handle587 在基值 0x06 下的状态：
	// 0x46 -> 独立导丝（正向）
	// 0x07 -> 独立导丝（反向）
	// 0x47 -> 独立导丝（取消“同时按 b6+b0 进入协同模式”）
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
	TcpForceDaqClient tcp_force_daq;
	ForceLogger force_logger;
	HandleFilterState axis1_handle_filter;
	HandleFilterState axis6_handle_filter;
	const bool axis1_handle_ready = handle_axis1.init();
	if (!axis1_handle_ready)
	{
		std::cout << "手柄初始化失败，序列号: " << serial_axis1_handle << std::endl;
	}
	const bool axis6_handle_ready = handle_axis6.init();
	if (!axis6_handle_ready)
	{
		std::cout << "手柄初始化失败，序列号: " << serial_axis6_handle << std::endl;
	}
	if (!axis1_handle_ready && !axis6_handle_ready)
	{
		return 0;
	}

	bool single_handle_mode = false;
	GuidewireMode single_handle_requested_mode = GuidewireMode::None;
	Handle* axis1_input_handle = &handle_axis1;
	Handle* axis6_input_handle = &handle_axis6;
	if (!(axis1_handle_ready && axis6_handle_ready))
	{
		Handle* single_handle = axis1_handle_ready ? &handle_axis1 : &handle_axis6;
		const DWORD single_serial = axis1_handle_ready ? serial_axis1_handle : serial_axis6_handle;
		std::cout << "仅识别到一个手柄（序列号: " << single_serial << "）。" << std::endl;
		std::cout << "按回车继续单手柄控制，按 ESC 或 q 退出。" << std::endl;
		while (true)
		{
			if (_kbhit())
			{
				const int ch = _getch();
				if (ch == '\r')
				{
					single_handle_mode = true;
					axis1_input_handle = single_handle;
					axis6_input_handle = single_handle;
					single_handle_requested_mode = GuidewireMode::None;
					std::cout << "已进入单手柄模式（默认导管/582语义）。" << std::endl;
					std::cout << "数字1：导管(582语义)，数字2：导丝(587语义)。" << std::endl;
					std::cout << "按键说明：b0方向，b6 Y阀，b7/b5 轴4点动。" << std::endl;
					break;
				}
				if (ch == 27 || ch == 'q' || ch == 'Q')
				{
					handle_axis1.close();
					handle_axis6.close();
					return 0;
				}
			}
			Sleep(20);
		}
	}

	Sleep(1000);
	if (axis1_input_handle == axis6_input_handle)
	{
		axis1_input_handle->poll();
	}
	else
	{
		axis1_input_handle->poll();
		axis6_input_handle->poll();
	}
	axis1_handle_filter.reset(axis1_input_handle->fJoints2[0], axis1_input_handle->fJoints2[1]);
	axis6_handle_filter.reset(axis6_input_handle->fJoints2[0], axis6_input_handle->fJoints2[1]);

	if (ads.OpenComm_inside())
	{
		std::cout << "ADS 已连接：本地 AMS 路由，端口 851。" << std::endl;
	}
	else
	{
		// 本地路由失败时尝试远端 NetId；仅当两者都失败再输出错误，避免无效告警干扰。
		if (ads.OpenComm())
		{
			std::cout << "ADS 已连接：远端 AMS NetId " << hardcoded_ads_netid << "，端口 851。" << std::endl;
		}
		else
		{
			std::cout << "ADS 连接失败，本地与远端路由均不可用，错误码: " << ads.GetLastError() << std::endl;
			handle_axis1.close();
			handle_axis6.close();
			return 0;
		}
	}

	// 诊断：打印当前 ADS 实际连接到的 PLC 应用名，排查“连错实例/端口”问题。
	char plc_app_name[64] = { 0 };
	if (ads.ADSRead(AdsSymbol::app_name, sizeof(plc_app_name), plc_app_name))
	{
		plc_app_name[sizeof(plc_app_name) - 1] = '\0';
		std::cout << "ADS 目标 PLC 应用: " << plc_app_name << std::endl;
	}
	else
	{
		std::cout << "警告：读取 PLC 应用名失败，错误: " << ads.GetLastError() << std::endl;
	}

	// 力输出统一入口：按 setforce(F,N) 语义分别下发到 582/587。
	auto apply_force_output = [&](double force_582_f, double force_582_n, double force_587_f, double force_587_n)
	{
		handle_axis1.setforce(force_582_f, force_582_n);
		handle_axis6.setforce(force_587_f, force_587_n);
	};

	// 双手柄力输出清零（用于暂停、急停保持、F=OFF 等场景）。
	auto clear_force_output = [&]()
	{
		apply_force_output(0.0, 0.0, 0.0, 0.0);
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

	// 主循环共享上下文：集中收敛模块调用所需的运行态对象与缓存指针。
	AppContext ctx;
	ctx.ads = &ads;
	ctx.axis1_input_handle = axis1_input_handle;
	ctx.axis6_input_handle = axis6_input_handle;
	ctx.handle_axis1 = &handle_axis1;
	ctx.handle_axis6 = &handle_axis6;
	ctx.axis1_handle_filter = &axis1_handle_filter;
	ctx.axis6_handle_filter = &axis6_handle_filter;
	ctx.cfg = &cfg;
	ctx.cyl = &cyl;
	ctx.force_sample_source = cfg.force_sample_source;
	ctx.tcp_force_daq = &tcp_force_daq;
	ctx.pos = pos;
	ctx.plc_act_pos = plc_act_pos;
	ctx.plc_init_pos = plc_init_pos;
	ctx.plc_leftlimit = plc_leftlimit;
	ctx.plc_act_pos_from_left = plc_act_pos_from_left;
	ctx.plc_refer_from_left = plc_refer_from_left;
	ctx.plc_v_limit = plc_v_limit;
	ctx.startup_smoothing_bypass = &startup_smoothing_bypass;

	auto read_plc_state = [&]() -> bool { return plc_io::read_plc_state(ctx); };
	auto read_force_sample = [&](ForceSampleFrame& sample) -> bool { return plc_io::read_force_sample(ctx, sample); };
	auto write_refer = [&]() -> bool { return plc_io::write_refer(ctx); };
	auto read_v_limit = [&]() -> bool { return plc_io::read_v_limit(ctx); };
	auto write_v_limit = [&](const double* values) -> bool { return plc_io::write_v_limit(ctx, values); };
	auto load_pos_from_actual = [&]() { plc_io::load_pos_from_actual(ctx); };
	auto from_left_to_abs = [&](int axis_index, double from_left_mm) -> double
	{
		return motion_sync::from_left_to_abs(ctx, axis_index, from_left_mm);
	};
	auto from_left_to_rel = [&](int axis_index, double from_left_mm) -> double
	{
		return motion_sync::from_left_to_rel(ctx, axis_index, from_left_mm);
	};
	auto read_axis_return_status = [&](const AxisReturnAdsSymbols& symbols, AxisReturnStatus& status) -> bool
	{
		return plc_io::read_axis_return_status(ctx, symbols, status);
	};
	auto clear_axis_return_request = [&](const AxisReturnAdsSymbols& symbols) -> bool
	{
		return plc_io::clear_axis_return_request(ctx, symbols);
	};
	auto request_axis_return = [&](const AxisReturnAdsSymbols& symbols,
		double target_abs,
		double velocity,
		double acc,
		double dec,
		double jerk) -> bool
	{
		return plc_io::request_axis_return(ctx, symbols, target_abs, velocity, acc, dec, jerk);
	};
	auto clear_axis1_group_return_requests = [&]() -> bool
	{
		return plc_io::clear_axis1_group_return_requests(ctx);
	};
	auto write_axis4_manual_requests = [&](bool forward_req, bool reverse_req) -> bool
	{
		return plc_io::write_axis4_manual_requests(ctx, forward_req, reverse_req);
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
	double axis1_prev_rot_filtered = 0.0;
	double axis6_prev_rot_filtered = 0.0;
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
	ForceCalibrationConfig cal_cfg;
	ForceCalibrationState cal_state;
	ForceLogState force_log;
	CrawlState axis1_crawl;
	CrawlState axis6_crawl;
	axis1_crawl.enabled = true;

	// 绑定上下文中的长生命周期状态，替代大段 lambda capture。
	ctx.guidewire_mode = &guidewire_mode;
	ctx.axis1_crawl = &axis1_crawl;
	ctx.axis6_crawl = &axis6_crawl;
	ctx.startup = &startup;
	ctx.ff = &ff;
	ctx.force_log = &force_log;
	ctx.axis3_base_rel = &axis3_base_rel;
	ctx.axis5_base_rel = &axis5_base_rel;
	ctx.axis6_mirror_base_rel = &axis6_mirror_base_rel;
	ctx.axis1_return_entry_rel = &axis1_return_entry_rel;
	ctx.axis1_return_settle_rel = &axis1_return_settle_rel;
	ctx.axis6_return_entry_rel = &axis6_return_entry_rel;
	ctx.axis6_return_settle_rel = &axis6_return_settle_rel;
	ctx.axis1_return_hold_axis3_rel = &axis1_return_hold_axis3_rel;
	ctx.axis1_return_hold_axis5_rel = &axis1_return_hold_axis5_rel;
	ctx.axis1_fast_entry_abs = &axis1_fast_entry_abs;
	ctx.axis6_fast_entry_abs = &axis6_fast_entry_abs;
	ctx.axis6_coupled_target_abs = &axis6_coupled_target_abs;
	ctx.axis6_coupled_settle_rel = &axis6_coupled_settle_rel;
	ctx.axis6_coupled_active = &axis6_coupled_active;
	ctx.axis6_coupled_requested = &axis6_coupled_requested;
	ctx.axis6_coupled_done = &axis6_coupled_done;
	ctx.axis6_coupled_error = &axis6_coupled_error;
	ctx.axis6_coupled_error_id = &axis6_coupled_error_id;
	ctx.axis2_hold_rel = &axis2_hold_rel;
	ctx.axis7_hold_rel = &axis7_hold_rel;
	ctx.axis1_prev_linear_filtered = &axis1_prev_linear_filtered;
	ctx.axis6_prev_linear_filtered = &axis6_prev_linear_filtered;
	ctx.axis1_prev_rot_filtered = &axis1_prev_rot_filtered;
	ctx.axis6_prev_rot_filtered = &axis6_prev_rot_filtered;
	ctx.axis1_follow_cmd_abs = &axis1_follow_cmd_abs;
	ctx.axis6_follow_cmd_abs = &axis6_follow_cmd_abs;
	ctx.axis1_reverse_switch_guard_active = &axis1_reverse_switch_guard_active;
	ctx.axis6_reverse_switch_guard_active = &axis6_reverse_switch_guard_active;
	ctx.axis1_prev_abs_for_trigger = &axis1_prev_abs_for_trigger;
	ctx.axis6_prev_abs_for_trigger = &axis6_prev_abs_for_trigger;
	ctx.axis1_prev_abs_valid = &axis1_prev_abs_valid;
	ctx.axis6_prev_abs_valid = &axis6_prev_abs_valid;
	ctx.independent_axis1_hold_rel = &independent_axis1_hold_rel;
	ctx.independent_axis2_hold_rel = &independent_axis2_hold_rel;
	ctx.independent_axis3_hold_rel = &independent_axis3_hold_rel;
	ctx.independent_axis5_hold_rel = &independent_axis5_hold_rel;
	ctx.axis6_window_locked = &axis6_window_locked;
	ctx.axis6_locked_window_start_abs = &axis6_locked_window_start_abs;
	ctx.axis6_locked_window_end_abs = &axis6_locked_window_end_abs;
	ctx.axis6_coop_ff_inited = &axis6_coop_ff_inited;
	ctx.axis6_coop_prev_axis1_cmd_abs = &axis6_coop_prev_axis1_cmd_abs;

	auto axis1_window_left_abs = [&]() -> double { return motion_sync::axis1_window_left_abs(ctx); };
	auto axis1_window_right_abs = [&]() -> double { return motion_sync::axis1_window_right_abs(ctx); };
	auto set_axis6_independent_window = [&](double window_right_abs, bool log_clamp) -> bool
	{
		return motion_sync::set_axis6_independent_window(ctx, window_right_abs, log_clamp);
	};
	auto lock_axis6_window_from_current = [&]() { motion_sync::lock_axis6_window_from_current(ctx); };
	auto apply_locked_axis6_window = [&]() { motion_sync::apply_locked_axis6_window(ctx); };
	auto rebuild_axis6_window_if_covered = [&](const char* reason, bool log_result) -> bool
	{
		return motion_sync::rebuild_axis6_window_if_covered(ctx, reason, log_result);
	};
	auto sync_axis1 = [&](int samples, bool wait_rearm, int rearm_dir) -> bool
	{
		return motion_sync::sync_axis1(ctx, samples, wait_rearm, rearm_dir);
	};
	auto sync_axis6 = [&](int samples,
		bool capture_window,
		bool wait_rearm,
		int rearm_dir,
		bool check_window_cover,
		bool log_window_cover) -> bool
	{
		return motion_sync::sync_axis6(
			ctx,
			samples,
			capture_window,
			wait_rearm,
			rearm_dir,
			check_window_cover,
			log_window_cover);
	};
	auto sync_all = [&](int samples) -> bool { return motion_sync::sync_all(ctx, samples); };
	auto clear_plc_reinit_req = [&]() { plc_io::clear_plc_reinit_req(ctx); };
	auto capture_axis1_follow_baseline = [&]() { motion_sync::capture_axis1_follow_baseline(ctx); };
	auto apply_axis1_mirror_from_abs = [&](double axis1_abs_cmd, bool include_axis6)
	{
		motion_sync::apply_axis1_mirror_from_abs(ctx, axis1_abs_cmd, include_axis6);
	};
	auto enter_independent_guidewire_mode = [&]() -> bool
	{
		return guidewire_mode_ctrl::enter_independent_guidewire_mode(ctx);
	};
	auto enter_cooperative_guidewire_mode = [&]() -> bool
	{
		return guidewire_mode_ctrl::enter_cooperative_guidewire_mode(ctx);
	};
	auto check_axis6_guidewire_entry_gate = [&](double& axis6_from_left_mm) -> bool
	{
		return guidewire_mode_ctrl::check_axis6_guidewire_entry_gate(ctx, axis6_from_left_mm);
	};
	auto exit_guidewire_mode_to_normal = [&]() -> bool
	{
		return guidewire_mode_ctrl::exit_guidewire_mode_to_normal(ctx);
	};
	auto start_startup_sequence = [&]() -> bool { return startup_sequence::start_startup_sequence(ctx); };
	auto restore_startup_v_limit = [&]() -> bool { return startup_sequence::restore_startup_v_limit(ctx); };
	auto prompt_startup_mode = [&]() { startup_sequence::prompt_startup_mode(ctx); };

	// 在交互循环开始前，用初始 PLC 快照初始化各保持位姿。
	if (!read_plc_state())
	{
		std::cout << "读取 PLC 状态失败。" << std::endl;
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
	axis1_prev_rot_filtered = axis1_handle_filter.axis1_filtered;
	axis6_prev_rot_filtered = axis6_handle_filter.axis1_filtered;
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
	bool axis4_manual_error_prev = false;
	unsigned long axis4_manual_error_id_prev = 0;
	GuidewireMode requested_guidewire_mode_prev = GuidewireMode::None;
	bool axis1_fast_return = false; // 轴1快退旁路标志（写入 G.axis1_fast_return）
	bool axis6_fast_retract = false; // 轴6快退旁路标志（写入 G.axis6_fast_retract）
	AxisReturnStatus axis1_return_status;
	AxisReturnStatus axis6_return_status;
	ForceSampleFrame force_sample;
	int loop_count = 0;
	DWORD force_log_warn_last_ms = 0;
	bool cylinder5_diag_edge_pending = false;
	DWORD cylinder5_diag_last_log_ms = 0;
	startup_smoothing_bypass = false;
	ads.ADSWrite(AdsSymbol::startup_smoothing_bypass, sizeof(startup_smoothing_bypass), &startup_smoothing_bypass);

	std::cout << "力反馈：关闭（按 F 键切换）。" << std::endl;
	clear_force_output();

	// 力感记录改为在用户选择 C/S 后再启动；文件名包含日期与 24 小时制时间（到秒）。
	// 旧 ForceLogState 路径已停用（保留结构以兼容其他模块对 ctx.force_log 的引用）。
	force_log.period_ms = cfg.force_log_period_ms;
	force_log.enabled = false;
	bool force_log_started = false;
	auto ensure_force_log_started = [&]()
	{
		if (force_log_started)
		{
			return;
		}
		if (ctx.force_sample_source == ForceSampleSource::TCP_DAQ)
		{
			if (tcp_force_daq.start(cfg.tcp_force_daq_ip, cfg.tcp_force_daq_port))
			{
				std::cout << "CSV采样源：TCP_DAQ（" << cfg.tcp_force_daq_ip << ":" << cfg.tcp_force_daq_port
					<< "），AIN0/AIN1 原始电压按传感器频率落盘。" << std::endl;
			}
			else
			{
				std::cout << "CSV采样源：TCP_DAQ 启动失败，日志将等待 TCP 有效帧后再写入。" << std::endl;
			}
		}
		else
		{
			std::cout << "CSV采样源：ADS（仅主循环节拍粗采样，未启用高频记录）。" << std::endl;
		}
		if (force_logger.start("."))
		{
			tcp_force_daq.set_on_sample([&](std::uint64_t tick_ms, const double v[6])
			{
				force_logger.on_sensor_sample(tick_ms, v);
			});
			std::cout << "高频传感数据记录器已启动（CSV 文件位于工作目录）。" << std::endl;
		}
		else
		{
			std::cout << "高频传感数据记录器启动失败：CSV 文件打开失败。" << std::endl;
		}
		force_log_started = true;
	};

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

	bool cylinder_manual_open_override[4] = { false, false, false, false };

	while (true)
	{
		// 1) 采样逻辑手柄输入，并生成按键边沿触发状态。
		if (axis1_input_handle == axis6_input_handle)
		{
			axis1_input_handle->poll();
		}
		else
		{
			axis1_input_handle->poll();
			axis6_input_handle->poll();
		}
		axis1_handle_filter.update(
			axis1_input_handle->fJoints2[0],
			axis1_input_handle->fJoints2[1],
			cfg.linear_handle_alpha,
			cfg.rotational_handle_alpha);
		axis6_handle_filter.update(
			axis6_input_handle->fJoints2[0],
			axis6_input_handle->fJoints2[1],
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

		const unsigned char axis1_buttons = axis1_input_handle->buttons2;
		const unsigned char axis6_buttons = axis6_input_handle->buttons2;
		const bool pause_pressed = (axis1_buttons & axis1_pause_button_mask) != 0;
		const bool axis1_reverse_pressed = (axis1_buttons & axis1_reverse_button_mask) != 0;
		const bool guidewire_independent_pressed = (axis6_buttons & axis6_independent_button_mask) != 0;
		const bool guidewire_cooperative_pressed = (axis6_buttons & axis6_cooperative_button_mask) != 0;
		const bool guidewire_reverse_pressed = guidewire_cooperative_pressed && !guidewire_independent_pressed;
		const bool axis4_base_pressed = (axis1_buttons & axis4_buttons_base_mask) == axis4_buttons_base_mask;
		const bool axis4_forward_pressed =
			axis4_base_pressed &&
			((axis1_buttons & axis4_buttons_forward_mask) != 0) &&
			((axis1_buttons & axis4_buttons_reverse_mask) == 0);
		const bool axis4_reverse_pressed =
			axis4_base_pressed &&
			((axis1_buttons & axis4_buttons_reverse_mask) != 0) &&
			((axis1_buttons & axis4_buttons_forward_mask) == 0);

		// 导丝模式请求解码（587）：
		// - b6=1,b0=0 -> Independent（0x46）
		// - b6=0,b0=1 -> Independent + Reverse（0x07）
		// - b6=1,b0=1 -> Independent（0x47，协同模式入口已取消）
		// - b6=0,b0=0 -> None（0x06）
		GuidewireMode requested_guidewire_mode = GuidewireMode::None;
		if (single_handle_mode)
		{
			requested_guidewire_mode = single_handle_requested_mode;
		}
		else if (guidewire_independent_pressed || guidewire_cooperative_pressed)
		{
			requested_guidewire_mode = GuidewireMode::Independent;
		}
		// 反向有效键仅在“独立且 b0 按下（0x07）”时生效。
		const bool axis6_effective_reverse_pressed = single_handle_mode ? axis1_reverse_pressed : guidewire_reverse_pressed;
		const bool startup_sequence_active = startup.is_active();
		// 正式控制阶段：启动流程已完成，b6 从“暂停键”切换为“电缸5开关键”。
		const bool formal_control_stage = startup.completed && (startup.phase == StartupPhase::Done);
		const bool axis4_jog_allowed = !freeze_active && !estop_hold_active && !startup_sequence_active;
		const bool axis4_forward_request = axis4_jog_allowed && axis4_forward_pressed;
		const bool axis4_reverse_request = axis4_jog_allowed && axis4_reverse_pressed;

		if (!formal_control_stage)
		{
			if (pause_pressed && !pause_pressed_prev)
			{
				freeze_active = true;
				control_active = false;
				clear_force_output();
				std::cout << "582 暂停：开启。" << std::endl;
			}
			else if (!pause_pressed && pause_pressed_prev)
			{
				freeze_active = false;
				if (startup_sequence_active)
				{
					std::cout << "582 暂停：关闭，启动流程继续。" << std::endl;
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
						std::cout << "582 暂停：关闭，等待选择启动方式。" << std::endl;
					}
					else if (estop_hold_active)
					{
						std::cout << "582 暂停已释放，等待 PLC 保持解除。" << std::endl;
					}
					else
					{
						std::cout << "582 暂停已释放，等待重同步完成。" << std::endl;
					}
				}
				else if (!estop_hold_active && sync_all(20))
				{
					control_active = true;
					std::cout << "582 暂停：关闭，控制已恢复。" << std::endl;
				}
				else if (estop_hold_active)
				{
					std::cout << "582 暂停已释放，等待 PLC 保持解除。" << std::endl;
				}
				else
				{
					std::cout << "582 暂停已释放，等待重同步完成。" << std::endl;
				}
			}
		}
		else if (pause_pressed != pause_pressed_prev)
		{
			// 正式控制阶段下，b6 仅用于切换电缸5，不再触发 freeze/pause。
			cylinder5_diag_edge_pending = true;
			std::cout << "582 b6："
				<< (pause_pressed ? "按下，电缸5 -> 0。" : "松开，电缸5 -> 2000。")
				<< std::endl;
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
					std::cout << "轴1导管回退模式："
						<< (axis1_reverse_pressed ? "开启" : "关闭")
						<< "（触发保护已激活）。" << std::endl;
				}
				else
				{
					std::cout << "轴1导管回退模式：" << (axis1_reverse_pressed ? "开启" : "关闭") << std::endl;
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
					std::cout
						<< (axis6_effective_reverse_pressed ? "导丝模式：反向取出" : "导丝模式：正向输送")
						<< "（触发保护已激活）。" << std::endl;
				}
				else
				{
					std::cout << (axis6_effective_reverse_pressed ? "导丝模式：反向取出。" : "导丝模式：正向输送。") << std::endl;
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
						std::cout << "PLC 保持：开启。" << std::endl;
					}
					estop_hold_active = true;
					control_active = false;
					clear_force_output();
				}
				else
				{
					if (estop_hold_active)
					{
						std::cout << "PLC 保持：关闭。" << std::endl;
						axis1_push_rearm_after_hold = true;
						std::cout << "轴1推送已锁定，请先反向回拉手柄完成重接管。" << std::endl;
					}
					estop_hold_active = false;
				}
			}
		}

		if (freeze_active)
		{
			ff.clear_output();
			clear_force_output();
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
						std::cout << "直接控制启动已忽略：582 暂停处于开启状态。" << std::endl;
					}
					else if (estop_hold_active)
					{
						std::cout << "直接控制启动已忽略：PLC 保持处于开启状态。" << std::endl;
					}
					else if (has_self_check_flag && !self_check_done)
					{
						std::cout << "直接控制启动已忽略：PLC 自检尚未完成。" << std::endl;
					}
					else if (!restore_startup_v_limit())
					{
						std::cout << "直接控制启动失败：无法恢复启动期速度限制参数。" << std::endl;
					}
					else if (sync_all(20))
					{
						startup.phase = StartupPhase::Done;
						startup.completed = true;
						startup.prompted = false;
						control_active = true;
						ensure_force_log_started();
						std::cout << "已进入直接控制。" << std::endl;
					}
					else
					{
						std::cout << "直接控制启动失败：ADS 重同步失败。" << std::endl;
					}
				}
			}
			else if (ch == 's' || ch == 'S')
			{
				if (!startup.completed && startup.phase == StartupPhase::WaitForEnter)
				{
					if (freeze_active)
					{
						std::cout << "启动准备已忽略：582 暂停处于开启状态。" << std::endl;
					}
					else if (estop_hold_active)
					{
						std::cout << "启动准备已忽略：PLC 保持处于开启状态。" << std::endl;
					}
					else if (has_self_check_flag && !self_check_done)
					{
						std::cout << "启动准备已忽略：PLC 自检尚未完成。" << std::endl;
					}
					else if (start_startup_sequence())
					{
						control_active = false;
						ensure_force_log_started();
						std::cout << "启动准备流程已开始。" << std::endl;
					}
					else
					{
						std::cout << "启动准备流程启动失败：ADS 重同步失败。" << std::endl;
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
				std::cout << "力反馈：" << (ff.enabled ? "开启" : "关闭") << std::endl;
				if (!ff.enabled)
				{
					clear_force_output();
				}
			}
			else if (ch == 'z' || ch == 'Z')
			{
				double raw_v[6] = { 0 };
				std::uint64_t ts = 0;
				if (tcp_force_daq.get_latest_raw(raw_v, ts))
				{
					cal_state.ft_zero = raw_v[0];
					cal_state.f_zero = raw_v[1];
					cal_state.zeroed = true;
					std::cout << "力传感器零点已采集：ft_zero=" << cal_state.ft_zero
						<< " V, f_zero=" << cal_state.f_zero << " V" << std::endl;
				}
				else
				{
					std::cout << "零点采集失败：TCP DAQ 无有效帧。" << std::endl;
				}
			}
			else if (single_handle_mode && ch == '1')
			{
				single_handle_requested_mode = GuidewireMode::None;
				std::cout << "单手柄模式：已切换到导管(582语义)。" << std::endl;
				std::cout << "按键说明：b0方向，b6 Y阀，b7/b5 轴4点动，数字2切导丝。" << std::endl;
			}
			else if (single_handle_mode && ch == '2')
			{
				single_handle_requested_mode = GuidewireMode::Independent;
				std::cout << "单手柄模式：已切换到导丝(587语义)。" << std::endl;
				std::cout << "按键说明：b0方向，b6 Y阀，b7/b5 轴4点动，数字1回导管。" << std::endl;
			}
			else if (ch == 'q' || ch == 'Q')
			{
				cylinder_manual_open_override[0] = !cylinder_manual_open_override[0];
				std::cout << "电缸1 手动开覆盖：" << (cylinder_manual_open_override[0] ? "开启" : "关闭") << std::endl;
			}
			else if (ch == 'w' || ch == 'W')
			{
				cylinder_manual_open_override[1] = !cylinder_manual_open_override[1];
				std::cout << "电缸2 手动开覆盖：" << (cylinder_manual_open_override[1] ? "开启" : "关闭") << std::endl;
			}
			else if (ch == 'e' || ch == 'E')
			{
				cylinder_manual_open_override[2] = !cylinder_manual_open_override[2];
				std::cout << "电缸3 手动开覆盖：" << (cylinder_manual_open_override[2] ? "开启" : "关闭") << std::endl;
			}
			else if (ch == 'r' || ch == 'R')
			{
				cylinder_manual_open_override[3] = !cylinder_manual_open_override[3];
				std::cout << "电缸4 手动开覆盖：" << (cylinder_manual_open_override[3] ? "开启" : "关闭") << std::endl;
			}
			else if (ch == 0 || ch == 224)
			{
				_getch();
			}
		}

		// 5) 导丝模式切换：双手柄时由 587 按键触发；单手柄时由键盘 1/2 触发，并复用同一重同步链路。
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
							std::cout << "导丝模式：关闭。" << std::endl;
						}
						else
						{
							guidewire_mode = GuidewireMode::None;
							axis6_crawl.enabled = false;
							axis6_window_locked = false;
							axis6_coop_ff_inited = false;
							axis6_coop_prev_axis1_cmd_abs = 0.0;
							std::cout << "导丝模式退出失败：ADS 重同步失败。" << std::endl;
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
				std::cout << "导丝模式切换已忽略：582 暂停处于开启状态。" << std::endl;
			}
			else if (estop_hold_active)
			{
				std::cout << "导丝模式切换已忽略：PLC 保持处于开启状态。" << std::endl;
			}
			else if (!startup.completed || startup.phase != StartupPhase::Done)
			{
				std::cout << "导丝模式切换已忽略：启动准备尚未完成。" << std::endl;
			}
			else if (!control_active)
			{
				std::cout << "导丝模式切换已忽略：控制尚未激活。" << std::endl;
			}
			else
			{
				bool mode_ok = false;
				bool mode_attempted = false;
				double axis6_from_left_mm = 0.0;
				const bool gate_checked = check_axis6_guidewire_entry_gate(axis6_from_left_mm);
				if (!gate_checked)
				{
					std::cout << "导丝模式切换失败：无法读取 axis6 进入门控的 PLC 状态。" << std::endl;
				}
				else if (axis6_from_left_mm >= cfg.guidewire_entry_axis6_from_left_max_mm)
				{
					std::cout
						<< "导丝模式切换已忽略：axis6 距左限位 = "
						<< axis6_from_left_mm
						<< " mm，要求 < "
						<< cfg.guidewire_entry_axis6_from_left_max_mm
						<< " mm。"
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
							std::cout << (axis6_effective_reverse_pressed ? "导丝模式：反向取出。" : "导丝模式：正向输送。") << std::endl;
						}
					}
					else if (requested_guidewire_mode == GuidewireMode::Cooperative)
					{
						mode_ok = enter_cooperative_guidewire_mode();
						if (mode_ok)
						{
							guidewire_mode = GuidewireMode::Cooperative;
							std::cout << "导丝模式：协同。" << std::endl;
						}
					}
				}

				if (mode_attempted && !mode_ok)
				{
					std::cout << "导丝模式切换失败。" << std::endl;
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
						std::cout << "警告：PLC 自检切换后恢复启动期速度限制参数失败。" << std::endl;
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
						std::cout << "PLC 自检已完成。" << std::endl;
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

		// 8) 当启动已完成但控制未激活时，通过全量重同步恢复。
		const bool motion_startup_active = startup.is_active();
		startup_smoothing_bypass = motion_startup_active;
		if (!control_active && !motion_startup_active && !freeze_active && !estop_hold_active && startup.completed)
		{
			if (sync_all(20))
			{
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
		// 力感 CSV 元数据编码：
		// mode_code: 0=导管, 1=导丝
		// reverse_code: 活动模式对应的正反向键状态(0/1)
		// push_pull_code: +1/-1/0（活动模式线性增量方向）
		// rot_sign_code: +1/-1/0（活动模式旋转关节增量符号）
		int force_mode_code = (guidewire_mode == GuidewireMode::None) ? 0 : 1;
		int force_reverse_code =
			(guidewire_mode == GuidewireMode::None)
			? (axis1_reverse_pressed ? 1 : 0)
			: (axis6_effective_reverse_pressed ? 1 : 0);
		int force_push_pull_code = 0;
		int force_rot_sign_code = 0;
		const double force_rot_sign_deadband_rad = 1e-4;

		unsigned short cylinder1_cmd = cyl.cyl1_open;
		unsigned short cylinder2_cmd = cyl.cyl2_clamp;
		unsigned short cylinder3_cmd = cyl.cyl3_follow_release;
		unsigned short cylinder4_cmd = cyl.cyl4_follow_release;
		// 电缸5默认维持初始化值；正式控制阶段由 582 b6 实时切换。
		unsigned short cylinder5_cmd = 2000;
		if (formal_control_stage)
		{
			cylinder5_cmd = pause_pressed ? static_cast<unsigned short>(0) : static_cast<unsigned short>(2000);
		}
		bool axis4_manual_forward_req = axis4_reverse_request;
		bool axis4_manual_reverse_req = axis4_forward_request;

		// 9) 根据当前顶层模式构建一帧 refer 和一组气缸指令。
		if (!freeze_active && !estop_hold_active && (control_active || motion_startup_active) && read_plc_state())
		{
			load_pos_from_actual();
			pos[1] = axis2_hold_rel;
			pos[6] = axis7_hold_rel;

			// 发布最新轴位置给高频日志线程读取（绝对坐标 mm）。
			force_logger.publish_axis_snapshot(
				plc_act_pos[0] + plc_init_pos[0],
				plc_act_pos[1] + plc_init_pos[1],
				plc_act_pos[5] + plc_init_pos[5],
				plc_act_pos[6] + plc_init_pos[6]);

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
			const bool force_is_catheter_mode = (guidewire_mode == GuidewireMode::None);
			const double active_linear_increment_mm =
				force_is_catheter_mode ? axis1_linear_increment_mm : axis6_linear_increment_mm;
			if (active_linear_increment_mm > 0.0)
			{
				force_push_pull_code = 1;
			}
			else if (active_linear_increment_mm < 0.0)
			{
				force_push_pull_code = -1;
			}

			const double axis1_rot_increment_rad = axis1_rot_filtered - axis1_prev_rot_filtered;
			const double axis6_rot_increment_rad = axis6_rot_filtered - axis6_prev_rot_filtered;
			const double active_rot_increment_rad =
				force_is_catheter_mode ? axis1_rot_increment_rad : axis6_rot_increment_rad;
			if (active_rot_increment_rad > force_rot_sign_deadband_rad)
			{
				force_rot_sign_code = 1;
			}
			else if (active_rot_increment_rad < -force_rot_sign_deadband_rad)
			{
				force_rot_sign_code = -1;
			}

			auto hold_axis1_mirror_axes_for_return = [&]()
			{
				pos[2] = axis1_return_hold_axis3_rel;
				pos[4] = axis1_return_hold_axis5_rel;
			};

			// 错峰切缸：避免一对夹爪同周期同步翻转造成器械瞬间双开释放。
			// t<stagger_ms 仅下发 close 侧；t>=stagger_ms 才下发 open 侧。
			// 调用方需在阶段入口将 seq_t0 置为 now_ms。
			auto staggered_pair = [&](unsigned short& close_cmd, unsigned short close_val,
				unsigned short& open_cmd, unsigned short open_val,
				DWORD seq_t0, DWORD stagger_ms)
			{
				close_cmd = close_val;
				if ((now_ms - seq_t0) >= stagger_ms)
				{
					open_cmd = open_val;
				}
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
				bool require_user_increment_for_trigger,
				bool use_sequential_cylinder)
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
						axis6_crawl.cyl_seq_stage = 0;
						axis6_crawl.cyl_seq_t0 = now_ms;
						axis6_crawl.rearm_dir = 0;
						axis6_crawl.plc_move_requested = false;
						staggered_pair(cylinder3_cmd, cyl.cyl3_clamp,
							cylinder4_cmd, cyl.cyl4_open,
							axis6_crawl.cyl_seq_t0, cfg.axis6_cylinder_interstep_wait_ms);
					}
				}
				else if (axis6_crawl.phase == CrawlState::Phase::SwitchWait)
				{
					axis6_fast_retract = true;
					pos[5] = axis6_return_entry_rel;
					staggered_pair(cylinder3_cmd, cyl.cyl3_clamp,
						cylinder4_cmd, cyl.cyl4_open,
						axis6_crawl.cyl_seq_t0, cfg.axis6_cylinder_interstep_wait_ms);
					if ((now_ms - axis6_crawl.phase_t0) >=
						(cfg.axis6_cylinder_interstep_wait_ms + cfg.axis6_pre_move_cylinder_wait_ms))
					{
						axis6_crawl.phase = CrawlState::Phase::FastMove;
						axis6_crawl.phase_t0 = now_ms;
						axis6_crawl.plc_move_requested = false;
						axis6_crawl.cyl_seq_stage = 0;
					}
				}
				else if (axis6_crawl.phase == CrawlState::Phase::FastMove)
				{
					pos[5] = axis6_crawl.target_abs - plc_init_pos[5];
					staggered_pair(cylinder3_cmd, cyl.cyl3_clamp,
						cylinder4_cmd, cyl.cyl4_open,
						axis6_crawl.cyl_seq_t0, cfg.axis6_cylinder_interstep_wait_ms);
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
							std::cout << "轴6 计划回退报错，错误码: " << axis6_return_status.error_id << std::endl;
							if (!sync_axis6(3, false, false, 0, false, false))
							{
								std::cout << "轴6 计划回退报错后重同步失败。" << std::endl;
							}
							axis6_crawl.phase = CrawlState::Phase::Follow;
							axis6_crawl.cyl_seq_stage = 0;
						}
						else if (axis6_return_status.done)
						{
							clear_axis_return_request(AdsSymbol::axis6_return);
							axis6_crawl.plc_move_requested = false;
							axis6_return_settle_rel = axis6_crawl.target_abs - plc_init_pos[5];
							axis6_crawl.phase = CrawlState::Phase::RestoreWait;
							axis6_crawl.phase_t0 = now_ms;
							axis6_crawl.cyl_seq_stage = 0;
							axis6_crawl.cyl_seq_t0 = now_ms;
						}
					}
					if (std::abs(axis6_abs - axis6_crawl.target_abs) <= cfg.crawl_arrive_tol_mm)
					{
						clear_axis_return_request(AdsSymbol::axis6_return);
						axis6_crawl.plc_move_requested = false;
						axis6_return_settle_rel = axis6_crawl.target_abs - plc_init_pos[5];
						axis6_crawl.phase = CrawlState::Phase::RestoreWait;
						axis6_crawl.phase_t0 = now_ms;
						axis6_crawl.cyl_seq_stage = 0;
						axis6_crawl.cyl_seq_t0 = now_ms;
					}
				}
				else if (axis6_crawl.phase == CrawlState::Phase::RestoreWait)
				{
					axis6_fast_retract = true;
					pos[5] = axis6_return_settle_rel;
					staggered_pair(cylinder4_cmd, cyl.cyl4_clamp,
						cylinder3_cmd, cyl.cyl3_open,
						axis6_crawl.cyl_seq_t0, cfg.axis6_cylinder_interstep_wait_ms);
					if ((now_ms - axis6_crawl.phase_t0) >=
						(cfg.axis6_cylinder_interstep_wait_ms + cfg.axis6_post_return_cylinder_wait_ms))
					{
						if (!sync_axis6(3, false, false, 0, false, false))
						{
							std::cout << "轴6 计划回退后重同步失败。" << std::endl;
						}
						axis6_crawl.phase = CrawlState::Phase::Follow;
						axis6_crawl.cyl_seq_stage = 0;
						axis6_crawl.plc_move_requested = false;
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
							std::cout << "警告：启动准备完成后恢复启动期速度限制参数失败。" << std::endl;
						}
						startup.phase = StartupPhase::Done;
						startup.completed = true;
						if (sync_all(30))
						{
							control_active = true;
							std::cout << "启动准备流程已完成。" << std::endl;
						}
						else
						{
							control_active = false;
							std::cout << "启动准备流程已完成，但重同步失败。" << std::endl;
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
					false,
					true);
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
						std::cout << "PLC 保持解除后，轴1推送已重接管。" << std::endl;
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
						cylinder3_cmd = cyl.cyl3_follow_release;
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
									std::cout << "已到达投送停止位：axis3 距左限位 <= 20mm。请切换反向模式后继续。" << std::endl;
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
								axis1_crawl.cyl_seq_stage = 1;
								axis1_crawl.cyl_seq_t0 = now_ms;
								axis1_crawl.rearm_dir = 0;
								axis1_crawl.plc_move_requested = false;
								staggered_pair(cylinder1_cmd, cyl.cyl1_clamp,
									cylinder2_cmd, cyl.cyl2_open,
									axis1_crawl.cyl_seq_t0, cfg.axis1_cylinder_interstep_wait_ms);
								if (!cooperative_mode)
								{
									staggered_pair(cylinder3_cmd, cyl.cyl3_clamp,
										cylinder4_cmd, cyl.cyl4_open,
										axis1_crawl.cyl_seq_t0, cfg.axis6_cylinder_interstep_wait_ms);
								}
							}
						}
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::SwitchWait)
				{
					const DWORD pair_pre_move_wait_ms =
						cfg.axis1_cylinder_interstep_wait_ms + cfg.axis1_pre_move_cylinder_wait_ms;
					const DWORD axis6_pair_pre_move_wait_ms =
						cfg.axis6_cylinder_interstep_wait_ms + cfg.axis6_pre_move_cylinder_wait_ms;
					const DWORD coupled_pre_move_wait_ms =
						axis6_coupled_active
						? ((pair_pre_move_wait_ms > axis6_pair_pre_move_wait_ms)
							? pair_pre_move_wait_ms
							: axis6_pair_pre_move_wait_ms)
						: pair_pre_move_wait_ms;
					axis1_fast_return = true;
					pos[0] = axis1_return_entry_rel;
					hold_axis1_mirror_axes_for_return();
					pos[1] = axis2_hold_rel;
					staggered_pair(cylinder1_cmd, cyl.cyl1_clamp,
						cylinder2_cmd, cyl.cyl2_open,
						axis1_crawl.cyl_seq_t0, cfg.axis1_cylinder_interstep_wait_ms);
					if (axis6_coupled_active)
					{
						axis6_fast_retract = true;
						pos[5] = axis6_return_entry_rel;
						staggered_pair(cylinder3_cmd, cyl.cyl3_clamp,
							cylinder4_cmd, cyl.cyl4_open,
							axis1_crawl.cyl_seq_t0, cfg.axis6_cylinder_interstep_wait_ms);
					}
					if ((now_ms - axis1_crawl.phase_t0) >= coupled_pre_move_wait_ms)
					{
						axis1_crawl.phase = CrawlState::Phase::FastMove;
						axis1_crawl.phase_t0 = now_ms;
						axis1_crawl.plc_move_requested = false;
						axis1_crawl.cyl_seq_stage = 0;
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::FastMove)
				{
					pos[0] = axis1_crawl.target_abs - plc_init_pos[0];
					hold_axis1_mirror_axes_for_return();
					pos[1] = axis2_hold_rel;
					staggered_pair(cylinder1_cmd, cyl.cyl1_clamp,
						cylinder2_cmd, cyl.cyl2_open,
						axis1_crawl.cyl_seq_t0, cfg.axis1_cylinder_interstep_wait_ms);
					axis1_fast_return = true;
					if (axis6_coupled_active)
					{
						axis6_fast_retract = true;
						pos[5] = axis6_coupled_target_abs - plc_init_pos[5];
						staggered_pair(cylinder3_cmd, cyl.cyl3_clamp,
							cylinder4_cmd, cyl.cyl4_open,
							axis1_crawl.cyl_seq_t0, cfg.axis6_cylinder_interstep_wait_ms);
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
					if (axis6_coupled_active && !axis6_coupled_done && !axis6_coupled_error &&
						(std::abs(axis6_abs - axis6_coupled_target_abs) <= cfg.crawl_arrive_tol_mm))
					{
						clear_axis_return_request(AdsSymbol::axis6_return);
						axis6_coupled_done = true;
						axis6_coupled_settle_rel = axis6_coupled_target_abs - plc_init_pos[5];
					}
					if (axis1_crawl.plc_move_requested &&
						read_axis_return_status(AdsSymbol::axis1_return, axis1_return_status))
					{
						if (axis1_return_status.error)
						{
							clear_axis_return_request(AdsSymbol::axis1_return);
							axis1_crawl.plc_move_requested = false;
							std::cout
								<< "轴1 计划回退报错，错误码: "
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
								std::cout << "轴1 计划回退报错后重同步失败。" << std::endl;
							}
							axis1_crawl.phase = CrawlState::Phase::Follow;
							axis1_crawl.cyl_seq_stage = 0;
						}
						else if (axis1_return_status.done)
						{
							if (axis6_coupled_active)
							{
								if (axis6_coupled_error)
								{
									std::cout
										<< "轴6 协同快进报错，错误码: "
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
										std::cout << "轴6 协同快进报错后重同步失败。" << std::endl;
									}
									axis1_crawl.phase = CrawlState::Phase::Follow;
									axis1_crawl.cyl_seq_stage = 0;
								}
								else if (axis6_coupled_done)
								{
									clear_axis_return_request(AdsSymbol::axis1_return);
									axis1_crawl.plc_move_requested = false;
									axis1_return_settle_rel = axis1_crawl.target_abs - plc_init_pos[0];
									axis6_return_settle_rel = axis6_coupled_settle_rel;
									axis1_crawl.phase = CrawlState::Phase::RestoreWait;
									axis1_crawl.phase_t0 = now_ms;
									axis1_crawl.cyl_seq_stage = 1;
									axis1_crawl.cyl_seq_t0 = now_ms;
								}
							}
							else
							{
								clear_axis_return_request(AdsSymbol::axis1_return);
								axis1_crawl.plc_move_requested = false;
								axis1_return_settle_rel = axis1_crawl.target_abs - plc_init_pos[0];
								axis1_crawl.phase = CrawlState::Phase::RestoreWait;
								axis1_crawl.phase_t0 = now_ms;
								axis1_crawl.cyl_seq_stage = 1;
								axis1_crawl.cyl_seq_t0 = now_ms;
							}
						}
					}
					const bool axis1_fast_at_target =
						std::abs(axis1_abs - axis1_crawl.target_abs) <= cfg.crawl_arrive_tol_mm;
					if (axis1_fast_at_target && (!axis6_coupled_active || axis6_coupled_done || axis6_coupled_error))
					{
						clear_axis_return_request(AdsSymbol::axis1_return);
						if (axis6_coupled_active)
						{
							clear_axis_return_request(AdsSymbol::axis6_return);
						}
						axis1_crawl.plc_move_requested = false;
						axis1_return_settle_rel = axis1_crawl.target_abs - plc_init_pos[0];
						axis6_return_settle_rel = axis6_coupled_settle_rel;
						axis1_crawl.phase = CrawlState::Phase::RestoreWait;
						axis1_crawl.phase_t0 = now_ms;
						axis1_crawl.cyl_seq_stage = 0;
						axis1_crawl.cyl_seq_t0 = now_ms;
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::RestoreWait)
				{
					const DWORD pair_post_return_wait_ms =
						cfg.axis1_cylinder_interstep_wait_ms + cfg.axis1_post_return_cylinder_wait_ms;
					const DWORD axis6_pair_post_return_wait_ms =
						cfg.axis6_cylinder_interstep_wait_ms + cfg.axis6_post_return_cylinder_wait_ms;
					const DWORD coupled_post_return_wait_ms =
						axis6_coupled_active
						? ((pair_post_return_wait_ms > axis6_pair_post_return_wait_ms)
							? pair_post_return_wait_ms
							: axis6_pair_post_return_wait_ms)
						: pair_post_return_wait_ms;
					axis1_fast_return = true;
					pos[0] = axis1_return_settle_rel;
					hold_axis1_mirror_axes_for_return();
					pos[1] = axis2_hold_rel;
					// RestoreWait 角色互换：电缸2先合，电缸1后开。
					staggered_pair(cylinder2_cmd, cyl.cyl2_clamp,
						cylinder1_cmd, cyl.cyl1_open,
						axis1_crawl.cyl_seq_t0, cfg.axis1_cylinder_interstep_wait_ms);
					if (axis6_coupled_active)
					{
						axis6_fast_retract = true;
						pos[5] = axis6_return_settle_rel;
						staggered_pair(cylinder4_cmd, cyl.cyl4_clamp,
							cylinder3_cmd, cyl.cyl3_open,
							axis1_crawl.cyl_seq_t0, cfg.axis6_cylinder_interstep_wait_ms);
					}
					if ((now_ms - axis1_crawl.phase_t0) >= coupled_post_return_wait_ms)
					{
						if (!sync_axis1(3, false, 0))
						{
							std::cout << "轴1 计划回退后重同步失败。" << std::endl;
						}
						axis1_crawl.phase = CrawlState::Phase::Follow;
						axis1_crawl.cyl_seq_stage = 0;
						axis1_crawl.plc_move_requested = false;
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

						const double axis6_raw_cmd_abs = axis6_follow_cmd_abs + axis6_combined_increment_mm;

						run_axis6_crawl_state(
							axis6_raw_cmd_abs,
							axis6_combined_increment_mm,
							axis6_effective_reverse_pressed,
							axis6_linear_increment_active,
							true,
							false);
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
							true,
							false);
					}
				}
			}

			axis1_prev_linear_filtered = axis1_linear_filtered;
			axis6_prev_linear_filtered = axis6_linear_filtered;
			axis1_prev_rot_filtered = axis1_rot_filtered;
			axis6_prev_rot_filtered = axis6_rot_filtered;
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
				if (axis4_manual_error_now &&
					(!axis4_manual_error_prev || axis4_manual_error_id_now != axis4_manual_error_id_prev))
				{
					std::cout << "轴4 手动控制报错，错误码: " << axis4_manual_error_id_now << std::endl;
				}
				axis4_manual_error_prev = axis4_manual_error_now;
				axis4_manual_error_id_prev = axis4_manual_error_id_now;
			}
		}

		// 10) 仅在运动激活时驱动气缸；快速回退标志始终会写入。
		bool cylinder5_write_attempted = false;
		bool cylinder5_write_ok = false;
		bool cylinder5_req = formal_control_stage ? pause_pressed : false;
		if (!freeze_active && (control_active || motion_startup_active))
		{
			if (cylinder_manual_open_override[0]) cylinder1_cmd = cyl.cyl1_open;
			if (cylinder_manual_open_override[1]) cylinder2_cmd = cyl.cyl2_open;
			if (cylinder_manual_open_override[2]) cylinder3_cmd = cyl.cyl3_open;
			if (cylinder_manual_open_override[3]) cylinder4_cmd = cyl.cyl4_open;
			ads.ADSWrite(AdsSymbol::cylinder1_value, sizeof(cylinder1_cmd), &cylinder1_cmd);
			ads.ADSWrite(AdsSymbol::cylinder2_value, sizeof(cylinder2_cmd), &cylinder2_cmd);
			ads.ADSWrite(AdsSymbol::cylinder3_value, sizeof(cylinder3_cmd), &cylinder3_cmd);
			ads.ADSWrite(AdsSymbol::cylinder4_value, sizeof(cylinder4_cmd), &cylinder4_cmd);
			cylinder5_write_attempted = true;
			// 电缸5改为写 BOOL 请求，再由 PLC handle 周期映射为 0/2000。
			cylinder5_write_ok = ads.ADSWrite(AdsSymbol::cylinder5_press_req, sizeof(cylinder5_req), &cylinder5_req);
		}

		const DWORD cylinder5_diag_now_ms = GetTickCount();
		// const bool cylinder5_need_diag = cylinder5_diag_edge_pending || (cylinder5_write_attempted && !cylinder5_write_ok);
		const bool cylinder5_need_diag = false;
		if (cylinder5_need_diag &&
			(cylinder5_diag_edge_pending || (cylinder5_diag_now_ms - cylinder5_diag_last_log_ms) >= 500))
		{
			bool cylinder5_req_readback = false;
			const bool cylinder5_req_read_ok =
				ads.ADSRead(AdsSymbol::cylinder5_press_req, sizeof(cylinder5_req_readback), &cylinder5_req_readback);
			unsigned short cylinder5_cmd_readback = 0;
			const bool cylinder5_cmd_read_ok =
				ads.ADSRead(AdsSymbol::cylinder5_cmd, sizeof(cylinder5_cmd_readback), &cylinder5_cmd_readback);
			unsigned short cylinder5_out_readback = 0;
			const bool cylinder5_out_read_ok =
				ads.ADSRead(AdsSymbol::cylinder5_value, sizeof(cylinder5_out_readback), &cylinder5_out_readback);
			unsigned short gen_state_now = 0;
			const bool gen_state_read_ok =
				ads.ADSRead(AdsSymbol::gen_state, sizeof(gen_state_now), &gen_state_now);
			bool self_check_done_now = false;
			const bool self_check_read_ok =
				ads.ADSRead(AdsSymbol::self_check_done, sizeof(self_check_done_now), &self_check_done_now);
			std::cout << "电缸5诊断：cmd=" << cylinder5_cmd
				<< "，write_attempt=" << (cylinder5_write_attempted ? "Y" : "N")
				<< "，write_ok=" << (cylinder5_write_ok ? "Y" : "N")
				<< "，req=" << (cylinder5_req ? "1" : "0")
				<< "，req_read_ok=" << (cylinder5_req_read_ok ? "Y" : "N")
				<< "，req_readback=" << (cylinder5_req_read_ok ? (cylinder5_req_readback ? "1" : "0") : "N/A")
				<< "，cmd_read_ok=" << (cylinder5_cmd_read_ok ? "Y" : "N")
				<< "，cmd_readback=" << (cylinder5_cmd_read_ok ? std::to_string(cylinder5_cmd_readback) : "N/A")
				<< "，out_read_ok=" << (cylinder5_out_read_ok ? "Y" : "N")
				<< "，out_readback=" << (cylinder5_out_read_ok ? std::to_string(cylinder5_out_readback) : "N/A")
				<< "，gen_state=" << (gen_state_read_ok ? std::to_string(gen_state_now) : "N/A")
				<< "，self_check_done=" << (self_check_read_ok ? (self_check_done_now ? "1" : "0") : "N/A")
				<< "，formal=" << (formal_control_stage ? "Y" : "N")
				<< "，pause=" << (pause_pressed ? "1" : "0")
				<< "，freeze=" << (freeze_active ? "Y" : "N")
				<< "，estop_hold=" << (estop_hold_active ? "Y" : "N")
				<< "，control_active=" << (control_active ? "Y" : "N")
				<< "，startup_active=" << (motion_startup_active ? "Y" : "N");
			if (cylinder5_write_attempted && !cylinder5_write_ok)
			{
				std::cout << "，write_err=" << ads.GetLastError();
			}
			if (!cylinder5_req_read_ok || !cylinder5_cmd_read_ok || !cylinder5_out_read_ok)
			{
				std::cout << "，read_err=" << ads.GetLastError();
			}
			std::cout << std::endl;
			cylinder5_diag_last_log_ms = cylinder5_diag_now_ms;
		}
		cylinder5_diag_edge_pending = false;

		write_axis4_manual_requests(axis4_manual_forward_req, axis4_manual_reverse_req);
		ads.ADSWrite(AdsSymbol::startup_smoothing_bypass, sizeof(startup_smoothing_bypass), &startup_smoothing_bypass);
		ads.ADSWrite(AdsSymbol::axis1_fast_return, sizeof(axis1_fast_return), &axis1_fast_return); // 轴1快退平滑旁路
		ads.ADSWrite(AdsSymbol::axis6_fast_retract, sizeof(axis6_fast_retract), &axis6_fast_retract); // 轴6快退平滑旁路

		// 力传感器采样与 CSV 写入：仅受 force_log_period_ms 控制。
		const DWORD force_log_now_ms = GetTickCount();
		if (force_log.should_sample(force_log_now_ms))
		{
			// 采样节拍按 period_ms 统一推进，读失败也不打乱节拍。
			force_log.last_sample_ms = force_log_now_ms;
			ForceSampleFrame sampled_frame;
			if (read_force_sample(sampled_frame))
			{
				force_sample = sampled_frame;
				double log_ft1_value = static_cast<double>(force_sample.ft_1_value);
				double log_fn1_value = static_cast<double>(force_sample.fn_1_value);
				bool ready_to_log = true;

				// TCP DAQ 电压填入 force_sample 供标定力反馈使用。
				{
					double tcp_raw_v[6] = { 0 };
					std::uint64_t tcp_ts = 0;
					if (tcp_force_daq.get_latest_raw(tcp_raw_v, tcp_ts))
					{
						force_sample.ft_1_value_v = tcp_raw_v[0];
						force_sample.fn_1_value_v = tcp_raw_v[1];
					}
				}

				// 仅替换 CSV 中 ft_1/fn_1 的来源：TCP 第0/1通道。
				if (ctx.force_sample_source == ForceSampleSource::TCP_DAQ)
				{
					double tcp_ft1_value = 0.0;
					double tcp_fn1_value = 0.0;
					std::uint64_t tcp_tick_ms = 0;
					if (tcp_force_daq.get_latest_ft1_fn1(tcp_ft1_value, tcp_fn1_value, tcp_tick_ms))
					{
						log_ft1_value = tcp_ft1_value;
						log_fn1_value = tcp_fn1_value;
					}
					else
					{
						ready_to_log = false;
						if ((force_log_now_ms - force_log_warn_last_ms) >= 1000)
						{
							std::cout << "力传感器告警：TCP_DAQ 当前无有效采样帧，已跳过本周期 CSV 行写入。" << std::endl;
							force_log_warn_last_ms = force_log_now_ms;
						}
					}
				}

				if (ready_to_log)
				{
					force_log.append_sample(
						force_log_now_ms,
						log_ft1_value,
						log_fn1_value,
						force_sample.fn_2_value,
						force_sample.ft_2_value,
						force_mode_code,
						force_reverse_code,
						force_push_pull_code,
						force_rot_sign_code,
						force_sample.axis1_pos_rel);
				}
			}
			else if ((force_log_now_ms - force_log_warn_last_ms) >= 1000)
			{
				std::cout << "力传感器告警：ft_1/fn_1/fn_2/ft_2 与 axis1_pos_rel 的 ADSReadSum 读取失败。" << std::endl;
				force_log_warn_last_ms = force_log_now_ms;
			}
		}

		process_force_feedback(
			ff,
			force_sample,
			handle_axis1,
			handle_axis6,
			guidewire_mode,
			control_active,
			freeze_active,
			estop_hold_active,
			axis1_fast_return,
			axis6_fast_retract,
			loop_count,
			cal_cfg,
			cal_state);

		// 无论本拍是否进入控制分支，都更新线性差分基准，避免暂停/等待期间累积大跳变。
		axis1_prev_linear_filtered = axis1_handle_filter.axis0_filtered;
		axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;
		axis1_prev_rot_filtered = axis1_handle_filter.axis1_filtered;
		axis6_prev_rot_filtered = axis6_handle_filter.axis1_filtered;
	}

	startup_smoothing_bypass = false;
	ads.ADSWrite(AdsSymbol::startup_smoothing_bypass, sizeof(startup_smoothing_bypass), &startup_smoothing_bypass);
	clear_force_output();
	force_log.close();
	tcp_force_daq.stop();
	force_logger.stop();
	handle_axis1.close();
	handle_axis6.close();
	return 0;
}
