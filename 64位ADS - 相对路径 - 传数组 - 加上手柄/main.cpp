#include "control_types.h"
#include "delivery_tracking.h"
#include "delivery_tracking_logger.h"
#include "force_calibration.h"
#include "force_feedback.h"
#include "force_logger.h"
#include "force_transition_experiment.h"
#include "force_transition_logger.h"
#include "guidewire_mode.h"
#include "motion_sync.h"
#include "plc_io.h"
#include "startup_sequence.h"
#include "tcp_force_daq.h"
#include "vis_server.h"

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
// 3) 新增调试模式通过独立状态接管，退出后重建原有业务链路基准。

int main(int argc, char* argv[])
{
	setup_console_utf8();

	constexpr DWORD physical_handle_582_serial = 582;
	constexpr DWORD physical_handle_587_serial = 587;
	// TRUE：交换两只物理手柄的角色，用 587 承担导管/axis1/582语义，用 582 承担导丝/axis6/587语义。
	constexpr bool swap_handle_roles = true;
	const DWORD serial_axis1_handle = swap_handle_roles ? physical_handle_587_serial : physical_handle_582_serial;
	const DWORD serial_axis6_handle = swap_handle_roles ? physical_handle_582_serial : physical_handle_587_serial;
	const char* hardcoded_ads_netid = "169.254.119.135.1.1";

	std::cout << "手柄角色映射：导管/axis1/582语义 -> 物理 SN " << serial_axis1_handle
		<< "，导丝/axis6/587语义 -> 物理 SN " << serial_axis6_handle
		<< (swap_handle_roles ? "（已交换）。" : "（未交换）。") << std::endl;

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

	ControlConfig cfg;
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
	ForceTransitionLogger ft_logger;
	DeliveryTrackingController tracking_controller;
	DeliveryTrackingLogger tracking_logger;
	ForceTransitionExperiment ft_exp;
	VisServer vis_server;
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
		single_handle_mode = true;
		axis1_input_handle = single_handle;
		axis6_input_handle = single_handle;
		single_handle_requested_mode = GuidewireMode::None;
		std::cout << "单手柄模式已自动启用（序列号: " << single_serial << "）。" << std::endl;
	}
	// 协同递送仅允许在两个逻辑角色均由独立物理手柄承担时进入。
	// 当前运行中不支持热插拔，第二只手柄接入后需要重启本程序重新初始化。
	const bool dual_handle_ready = axis1_handle_ready && axis6_handle_ready && !single_handle_mode;

	// 力反馈按“导管/导丝语义”下发；单手柄时导管力输出跟随唯一可用物理手柄。
	Handle* catheter_force_output_handle = &handle_axis1;
	Handle* guidewire_force_output_handle = &handle_axis6;
	if (single_handle_mode)
	{
		if (axis1_handle_ready)
		{
			catheter_force_output_handle = &handle_axis1;
			guidewire_force_output_handle = &handle_axis6;
		}
		else
		{
			catheter_force_output_handle = &handle_axis6;
			guidewire_force_output_handle = &handle_axis1;
		}
		std::cout << "单手柄力反馈：导管/582语义输出 -> 物理 SN "
			<< catheter_force_output_handle->serial() << "。" << std::endl;
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

	vis_server.start();

	// 力输出统一入口：轴向力按配置的 SDK force axis 下发，力矩仍走 torque 参数。
	auto apply_force_output = [&](double force_582_f, double force_582_n, double force_587_f, double force_587_n)
	{
		handle_axis1.setforce_axis(force_582_f, cfg.axial_force_axis, force_582_n);
		handle_axis6.setforce_axis(force_587_f, cfg.axial_force_axis, force_587_n);
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
	// 协同方向分为 UI 请求值和已激活值。入口或方向切换失败时保留已激活方向，
	// 避免失败请求改变正在运行的运动方向。
	CooperativeDirection cooperative_direction_requested = CooperativeDirection::None;
	CooperativeDirection cooperative_direction = CooperativeDirection::None;
	CooperativeReturnOwner cooperative_return_owner = CooperativeReturnOwner::None;

	StartupState startup;
	ForceFeedbackState ff;
	ForceCalibrationConfig cal_cfg;
	ForceCalibrationState cal_state;
	ForceLogState force_log;
	CrawlState axis1_crawl;
	CrawlState axis6_crawl;
	SpacingRecoveryState spacing_recovery;
	axis1_crawl.enabled = true;
	startup.final_axis1_from_left_mm = cfg.startup_axis1_ready_from_left_mm;
	startup.final_axis3_from_left_mm = cfg.startup_axis3_ready_from_left_mm;
	startup.final_axis5_from_left_mm = cfg.startup_axis5_ready_from_left_mm;
	startup.final_axis6_from_left_mm = cfg.startup_axis5_ready_from_left_mm + cfg.axis56_ready_gap_mm;

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
	auto sync_axis1 = [&](int samples) -> bool
	{
		return motion_sync::sync_axis1(ctx, samples);
	};
	auto sync_axis6 = [&](int samples,
		bool rebuild_window,
		bool log_window_rebuild) -> bool
	{
		return motion_sync::sync_axis6(
			ctx,
			samples,
			rebuild_window,
			log_window_rebuild);
	};
	auto sync_cooperative_guidewire = [&](int samples, bool log_window_rebuild) -> bool
	{
		return motion_sync::sync_cooperative_guidewire(ctx, samples, log_window_rebuild);
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
	bool return_ads_fault_hold = false; // 回退 ADS 故障后保持停控，重启上位机并重新检查后解除
	// 普通导管正向递送中，轴1计划回退完成后的首次前推单独先行状态。
	bool axis1_post_return_lead_armed = false;
	bool axis1_post_return_lead_active = false;
	double axis1_post_return_lead_target_abs = 0.0;
	// axis6 内部软限位为保护锁存：预测目标越过上限后，不再允许相关手柄链路继续运动。
	bool axis6_soft_limit_hold = false;
	double axis6_soft_limit_blocked_target_from_left_mm = 0.0;
	AxisReturnStatus axis1_return_status;
	AxisReturnStatus axis6_return_status;
	ForceSampleFrame force_sample;
	int loop_count = 0;
	DWORD force_sample_last_sample_ms = 0;
	DWORD tracking_log_last_sample_ms = 0;
	std::string force_feedback_diag_reason;
	std::string force_sample_diag_reason;
	// 协同模式请求不能穿越暂停、自检重建或 ADS 故障继续保留。
	// 此处只撤销上位机模式接管；已经被 PLC 消费的回退命令仍由 PLC 安全链路处理。
	auto cancel_cooperative_delivery = [&](bool leave_active_mode)
	{
		cooperative_direction_requested = CooperativeDirection::None;
		cooperative_direction = CooperativeDirection::None;
		cooperative_return_owner = CooperativeReturnOwner::None;
		axis1_post_return_lead_armed = false;
		axis1_post_return_lead_active = false;
		if (leave_active_mode && guidewire_mode == GuidewireMode::Cooperative)
		{
			guidewire_mode = GuidewireMode::None;
			axis6_crawl.enabled = false;
			axis6_crawl.phase = CrawlState::Phase::Follow;
			axis6_crawl.plc_move_requested = false;
			axis6_crawl.cyl_seq_stage = 0;
			axis6_window_locked = false;
			axis6_coop_ff_inited = false;
			axis6_coop_prev_axis1_cmd_abs = 0.0;
		}
	};
	auto reset_axis1_post_return_lead = [&]()
	{
		axis1_post_return_lead_armed = false;
		axis1_post_return_lead_active = false;
		axis1_post_return_lead_target_abs = 0.0;
	};
	auto axis6_from_left_mm = [&](double axis6_abs) -> double
	{
		return axis6_abs - plc_leftlimit[5];
	};
	auto axis6_target_exceeds_soft_limit = [&](double target_abs) -> bool
	{
		return axis6_from_left_mm(target_abs) >
			(cfg.axis6_soft_limit_from_left_mm + 1e-6);
	};
	auto engage_axis6_soft_limit_hold = [&](double target_abs, const char* source)
	{
		if (axis6_soft_limit_hold)
		{
			return;
		}

		axis6_soft_limit_hold = true;
		axis6_soft_limit_blocked_target_from_left_mm = axis6_from_left_mm(target_abs);
		reset_axis1_post_return_lead();
		// 预测越限时撤销尚未完成的上位机回退请求，避免 PLC 继续接受超限目标。
		(void)clear_axis1_group_return_requests();
		(void)clear_axis_return_request(AdsSymbol::axis6_return);
		axis1_crawl.phase = CrawlState::Phase::Follow;
		axis1_crawl.plc_move_requested = false;
		axis6_crawl.phase = CrawlState::Phase::Follow;
		axis6_crawl.plc_move_requested = false;
		axis6_coupled_active = false;
		axis6_coupled_requested = false;
		axis6_coupled_done = false;
		axis6_coupled_error = false;
		axis6_coupled_error_id = 0;
		axis6_coop_ff_inited = false;
		axis1_fast_return = false;
		axis6_fast_retract = false;
		clear_force_output();
		std::cout << "警告：axis6 软件限位锁止（" << source
			<< " 预测目标距左限位 " << axis6_soft_limit_blocked_target_from_left_mm
			<< " mm > " << cfg.axis6_soft_limit_from_left_mm
			<< " mm）。未下发越限回退，相关手柄链路已冻结。" << std::endl;
	};
	auto cooperative_direction_text = [](CooperativeDirection direction) -> const char*
	{
		switch (direction)
		{
		case CooperativeDirection::Delivery: return "协同递送";
		case CooperativeDirection::Retraction: return "协同撤出";
		default: return "协同模式";
		}
	};
	auto reset_cooperative_direction_guards = [&](CooperativeDirection direction)
	{
		const bool retraction = direction == CooperativeDirection::Retraction;
		const double axis1_abs_now = plc_act_pos[0] + plc_init_pos[0];
		const double axis6_abs_now = plc_act_pos[5] + plc_init_pos[5];
		const double axis1_trigger_edge_abs = retraction
			? axis1_window_right_abs()
			: axis1_window_left_abs();
		const double axis6_trigger_edge_abs = retraction
			? axis6_crawl.max_abs()
			: axis6_crawl.min_abs();
		axis1_reverse_switch_guard_active =
			std::abs(axis1_abs_now - axis1_trigger_edge_abs) <= cfg.reverse_switch_trigger_guard_mm;
		axis6_reverse_switch_guard_active =
			std::abs(axis6_abs_now - axis6_trigger_edge_abs) <= cfg.reverse_switch_trigger_guard_mm;
		axis1_prev_abs_for_trigger = axis1_abs_now;
		axis6_prev_abs_for_trigger = axis6_abs_now;
		axis1_prev_abs_valid = true;
		axis6_prev_abs_valid = true;
		axis1_delivery_stop_latched = false;
		axis1_delivery_stop_prompted = false;
		axis6_coop_ff_inited = false;
		axis6_coop_prev_axis1_cmd_abs = 0.0;
	};
	auto validate_cooperative_entry = [&](CooperativeDirection direction) -> bool
	{
		const char* mode_name = cooperative_direction_text(direction);
		if (direction == CooperativeDirection::None)
		{
			std::cout << "协同模式进入被拒绝：未指定运动方向。" << std::endl;
			return false;
		}
		if (!dual_handle_ready)
		{
			std::cout << mode_name << "进入被拒绝：需要在程序启动时成功连接两只手柄。" << std::endl;
			return false;
		}
		if (!startup.completed || startup.phase != StartupPhase::Done || !control_active ||
			freeze_active || estop_hold_active || return_ads_fault_hold || axis6_soft_limit_hold)
		{
			std::cout << mode_name << "进入被拒绝：控制尚未处于可用状态。" << std::endl;
			return false;
		}
		if (spacing_recovery.active() || spacing_recovery.requested || ft_exp.active())
		{
			std::cout << mode_name << "进入被拒绝：当前有调试模式或力过渡实验在接管。" << std::endl;
			return false;
		}
		if (axis1_crawl.phase != CrawlState::Phase::Follow ||
			axis6_crawl.phase != CrawlState::Phase::Follow ||
			axis1_crawl.plc_move_requested || axis6_crawl.plc_move_requested ||
			axis6_coupled_active ||
			cooperative_return_owner != CooperativeReturnOwner::None)
		{
			std::cout << mode_name << "进入被拒绝：轴1或轴6仍在执行回退或夹爪切换。" << std::endl;
			return false;
		}

		AxisReturnStatus entry_axis1_return;
		AxisReturnStatus entry_axis6_return;
		if (!read_axis_return_status(AdsSymbol::axis1_return, entry_axis1_return) ||
			!read_axis_return_status(AdsSymbol::axis6_return, entry_axis6_return))
		{
			std::cout << mode_name << "进入被拒绝：无法读取 PLC 计划回退状态。" << std::endl;
			return false;
		}
		if (entry_axis1_return.busy || entry_axis6_return.busy)
		{
			std::cout << mode_name << "进入被拒绝：PLC 计划回退仍处于 Busy。" << std::endl;
			return false;
		}
		if (!read_plc_state())
		{
			std::cout << mode_name << "进入被拒绝：无法读取 PLC 实际位置。" << std::endl;
			return false;
		}

		const double axis6_abs = plc_act_pos[5] + plc_init_pos[5];
		const double axis6_from_left_mm = std::abs(axis6_abs - plc_leftlimit[5]);
		if (direction == CooperativeDirection::Delivery &&
			axis6_from_left_mm >= cfg.guidewire_entry_axis6_from_left_max_mm)
		{
			std::cout << mode_name << "进入被拒绝：axis6 距左限位 = "
				<< axis6_from_left_mm << " mm，要求 < "
				<< cfg.guidewire_entry_axis6_from_left_max_mm << " mm。" << std::endl;
			return false;
		}

		const double axis5_from_left_mm =
			(plc_act_pos[4] + plc_init_pos[4]) - plc_leftlimit[4];
		const double axis56_gap_mm = axis6_from_left_mm - axis5_from_left_mm;
		const double axis56_max_gap_mm =
			cfg.axis6_window_min_gap_from_axis5_mm + cfg.axis6_window_size_mm;
		if (axis56_gap_mm < (cfg.axis6_window_min_gap_from_axis5_mm - cfg.crawl_arrive_tol_mm) ||
			axis56_gap_mm > (axis56_max_gap_mm + cfg.crawl_arrive_tol_mm))
		{
			std::cout << mode_name << "进入被拒绝：axis6 相对 axis5 不在 ["
				<< cfg.axis6_window_min_gap_from_axis5_mm << ", "
				<< axis56_max_gap_mm << "] mm 安全窗口内。" << std::endl;
			return false;
		}
		return true;
	};
	startup_smoothing_bypass = false;
	ads.ADSWrite(AdsSymbol::startup_smoothing_bypass, sizeof(startup_smoothing_bypass), &startup_smoothing_bypass);

	std::cout << "力反馈：关闭（按 F 键切换）。" << std::endl;
	clear_force_output();

	// 力感高频记录在用户选择 C/S 后启动；force_log.period_ms 仅控制力采样节拍。
	force_log.period_ms = cfg.force_log_period_ms;
	force_log.enabled = false;
	bool force_log_started = false;
	bool force_tcp_zero_wait_logged = false;
	DWORD force_feedback_value_log_last_ms = 0;
	auto ensure_force_log_started = [&]()
	{
		if (force_log_started)
		{
			return;
		}
		if (ctx.force_sample_source == ForceSampleSource::TCP_DAQ)
		{
			if (tcp_force_daq.start(cfg.tcp_force_daq_ip, cfg.tcp_force_daq_port, cfg.tcp_force_daq_local_ip))
			{
				std::cout << "CSV采样源：TCP_DAQ（" << cfg.tcp_force_daq_ip << ":" << cfg.tcp_force_daq_port
					<< "，本机绑定 " << cfg.tcp_force_daq_local_ip
					<< "），fn_1/ft_1 原始电压按传感器频率落盘。" << std::endl;
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
			force_logger.publish_force_zero(cal_state.f_zero, cal_state.ft_zero);
			if (ctx.force_sample_source == ForceSampleSource::TCP_DAQ)
			{
				tcp_force_daq.set_on_sample([&](std::uint64_t tick_ms, const double v[6])
				{
					force_logger.on_sensor_sample(tick_ms, v);
				});
			}
			else
			{
				tcp_force_daq.set_on_sample(nullptr);
			}
			std::cout << "高频传感数据记录器已启动（CSV 文件位于工作目录）。" << std::endl;
		}
		else
		{
			std::cout << "高频传感数据记录器启动失败：CSV 文件打开失败。" << std::endl;
		}
		force_log_started = true;
	};

	// 主从位移研究记录从上位机进程启动即创建会话，不再依赖 UI 先点击“开始记录”。
	// 记录器仅异步写 CSV，不会阻塞运动控制循环；程序退出时统一 flush 并关闭。
	if (tracking_logger.start_session("."))
	{
		tracking_controller.start_session();
		tracking_log_last_sample_ms = 0;
		force_sample_last_sample_ms = 0;
		std::cout << "主从位移实验记录已自动启动（20 Hz，CSV 位于工作目录）。" << std::endl;
	}
	else
	{
		std::cout << "主从位移实验记录自动启动失败：CSV 文件打开失败；位移补偿不可用。" << std::endl;
	}

	auto zero_force_sensor = [&](const char* source) -> bool
	{
		if (ctx.force_sample_source == ForceSampleSource::TCP_DAQ)
		{
			double raw_v[6] = { 0 };
			std::uint64_t ts = 0;
			if (tcp_force_daq.get_latest_raw(raw_v, ts))
			{
				cal_state.f_zero = raw_v[0];
				cal_state.ft_zero = raw_v[1];
				cal_state.theta0_deg = plc_act_pos[1];
				cal_state.zeroed = true;
				force_logger.publish_force_zero(cal_state.f_zero, cal_state.ft_zero);
				force_tcp_zero_wait_logged = false;
				std::cout << source << "力传感器零点已采集：AI0/f_zero=" << cal_state.f_zero
					<< " V, AI1/ft_zero=" << cal_state.ft_zero
					<< " V, axis2/theta0=" << cal_state.theta0_deg << " deg" << std::endl;
				return true;
			}
			if (!force_tcp_zero_wait_logged)
			{
				std::cout << source << "零点采集失败：当前采样源为 TCP_DAQ，但尚无采集卡有效帧；不会回退到 ADS 零点。" << std::endl;
				force_tcp_zero_wait_logged = true;
			}
			return false;
		}

		// ADS 为默认源时必须同步读取一帧，不能依赖旧采样节拍或力反馈开关。
		ForceSampleFrame sampled_frame;
		if (read_force_sample(sampled_frame))
		{
			force_sample = sampled_frame;
			cal_state.ft_zero = sampled_frame.ft_1_value_v;
			cal_state.f_zero = sampled_frame.fn_1_value_v;
			cal_state.theta0_deg = sampled_frame.axis2_pos_rel;
			cal_state.zeroed = true;
			force_logger.publish_force_zero(cal_state.f_zero, cal_state.ft_zero);
			std::cout << source << "力传感器零点已采集（ADS 当前采样）：ft_zero=" << cal_state.ft_zero
				<< " V, f_zero=" << cal_state.f_zero << " V"
				<< ", axis2/theta0=" << cal_state.theta0_deg << " deg" << std::endl;
			return true;
		}

		std::cout << source << "零点采集失败：ADS 力采样读取失败。" << std::endl;
		return false;
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

	enum class CylinderManualMode : unsigned char
	{
		Automatic,
		Open,
		Closed
	};
	CylinderManualMode cylinder_manual_mode[4] = {
		CylinderManualMode::Automatic,
		CylinderManualMode::Automatic,
		CylinderManualMode::Automatic,
		CylinderManualMode::Automatic
	};
	bool vis_reverse_override_active = false;
	bool vis_reverse_override_value = false;
	int vis_reverse_override_target = 0;

	struct PendingStartupParams {
		double axis1_from_left_mm = 20.0;
		double axis3_from_left_mm = 635.0;
		double axis5_from_left_mm = 290.0;
		double axis6_from_left_mm = 310.0;
		double axis2_deg = 0.0;
		double axis7_deg = 0.0;
		double speed_scale = 0.005;
	} pending_startup;
	bool tracking_manual_cylinder_override = false;

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
		if (return_ads_fault_hold)
		{
			control_active = false;
			cancel_cooperative_delivery(true);
			clear_force_output();
		}

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
		const bool axis1_reverse_button_pressed = (axis1_buttons & axis1_reverse_button_mask) != 0;
		// 只有实际进入协同模式后才固定为其选定方向。入口请求尚未通过门控时，
		// 必须继续沿用当前模式的方向，避免被拒绝的请求造成单拍反向跳变。
		bool cooperative_mode_active =
			guidewire_mode == GuidewireMode::Cooperative &&
			cooperative_direction != CooperativeDirection::None;
		bool cooperative_retraction_active =
			cooperative_mode_active && cooperative_direction == CooperativeDirection::Retraction;
		bool axis1_reverse_pressed = cooperative_mode_active
			? cooperative_retraction_active
			: ((vis_reverse_override_active && vis_reverse_override_target == 0)
				? vis_reverse_override_value
				: axis1_reverse_button_pressed);
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

		// 导丝模式请求解码（物理 SN 582，axis6 逻辑角色）：
		// - b6=1,b0=0 -> Independent（0x46）
		// - b6=0,b0=1 -> Independent + Reverse（0x07）
		// - b6=1,b0=1 -> Independent（0x47，协同模式入口已取消）
		// - b6=0,b0=0 -> None（0x06）
		GuidewireMode requested_guidewire_mode = GuidewireMode::None;
		if (cooperative_direction_requested != CooperativeDirection::None && dual_handle_ready)
		{
			// 协同方向由 UI 显式进入，期间忽略导丝手柄的模式/方向按键。
			requested_guidewire_mode = GuidewireMode::Cooperative;
		}
		else if (vis_reverse_override_active)
		{
			requested_guidewire_mode =
				(vis_reverse_override_target == 0) ? GuidewireMode::None : GuidewireMode::Independent;
		}
		else if (single_handle_mode)
		{
			requested_guidewire_mode = single_handle_requested_mode;
		}
		else if (guidewire_independent_pressed || guidewire_cooperative_pressed)
		{
			requested_guidewire_mode = GuidewireMode::Independent;
		}
		if (spacing_recovery.active())
		{
			// 恢复模式接管轴3/5/6期间忽略物理导丝模式按键。
			requested_guidewire_mode = GuidewireMode::None;
			cancel_cooperative_delivery(true);
		}
		// 反向有效键仅在“独立且 b0 按下（0x07）”时生效。
		bool axis6_effective_reverse_pressed = cooperative_mode_active
			? cooperative_retraction_active
			: ((vis_reverse_override_active && vis_reverse_override_target == 1)
				? vis_reverse_override_value
				: (single_handle_mode ? axis1_reverse_pressed : guidewire_reverse_pressed));
		const bool startup_sequence_active = startup.is_active();
		// axis1 单独先行只属于普通导管正向递送的紧邻回退后阶段，不能穿越模式或安全状态。
		if (guidewire_mode != GuidewireMode::None || axis1_reverse_pressed ||
			freeze_active || estop_hold_active || return_ads_fault_hold ||
			spacing_recovery.active() || spacing_recovery.requested || ft_exp.active() ||
			startup_sequence_active)
		{
			reset_axis1_post_return_lead();
		}
		// 正式控制阶段：启动流程已完成，b6 从“暂停键”切换为“电缸5开关键”。
		const bool formal_control_stage = startup.completed && (startup.phase == StartupPhase::Done);
		const bool axis4_jog_allowed = !freeze_active && !estop_hold_active && !startup_sequence_active &&
			!spacing_recovery.active() && !spacing_recovery.requested;
		const bool axis4_forward_request = axis4_jog_allowed && axis4_forward_pressed;
		const bool axis4_reverse_request = axis4_jog_allowed && axis4_reverse_pressed;

		if (!formal_control_stage)
		{
			if (pause_pressed && !pause_pressed_prev)
			{
				freeze_active = true;
				control_active = false;
				cancel_cooperative_delivery(true);
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
			std::cout << "582 b6："
				<< (pause_pressed ? "按下，电缸5 -> 0。" : "松开，电缸5 -> 2000。")
				<< std::endl;
		}
		pause_pressed_prev = pause_pressed;

		// 2) 正反键切换：启用一次性触发保护，不再执行线性重同步。
		if (!freeze_active && !estop_hold_active && !startup_sequence_active && control_active &&
			!spacing_recovery.active())
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
					cancel_cooperative_delivery(true);
					clear_force_output();
				}
				else
				{
					if (estop_hold_active)
					{
						std::cout << "PLC 保持：关闭。" << std::endl;
						if (formal_control_stage)
						{
							axis1_push_rearm_after_hold = true;
							std::cout << "轴1推送已锁定，请先反向回拉手柄完成重接管。" << std::endl;
						}
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
		if ((freeze_active || estop_hold_active || return_ads_fault_hold) &&
			(spacing_recovery.active() || spacing_recovery.requested))
		{
			spacing_recovery.reset();
			clear_force_output();
			std::cout << "屈曲恢复已由暂停、急停或 ADS 故障终止。" << std::endl;
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
				zero_force_sensor("");
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
				cylinder_manual_mode[0] = cylinder_manual_mode[0] == CylinderManualMode::Open
					? CylinderManualMode::Automatic : CylinderManualMode::Open;
				std::cout << "电缸1 手动开覆盖：" << (cylinder_manual_mode[0] == CylinderManualMode::Open ? "开启" : "关闭") << std::endl;
			}
			else if (ch == 'w' || ch == 'W')
			{
				cylinder_manual_mode[1] = cylinder_manual_mode[1] == CylinderManualMode::Open
					? CylinderManualMode::Automatic : CylinderManualMode::Open;
				std::cout << "电缸2 手动开覆盖：" << (cylinder_manual_mode[1] == CylinderManualMode::Open ? "开启" : "关闭") << std::endl;
			}
			else if (ch == 'e' || ch == 'E')
			{
				cylinder_manual_mode[2] = cylinder_manual_mode[2] == CylinderManualMode::Open
					? CylinderManualMode::Automatic : CylinderManualMode::Open;
				std::cout << "电缸3 手动开覆盖：" << (cylinder_manual_mode[2] == CylinderManualMode::Open ? "开启" : "关闭") << std::endl;
			}
			else if (ch == 'r' || ch == 'R')
			{
				cylinder_manual_mode[3] = cylinder_manual_mode[3] == CylinderManualMode::Open
					? CylinderManualMode::Automatic : CylinderManualMode::Open;
				std::cout << "电缸4 手动开覆盖：" << (cylinder_manual_mode[3] == CylinderManualMode::Open ? "开启" : "关闭") << std::endl;
			}
			else if (ch == 0 || ch == 224)
			{
				_getch();
			}
		}

		// 5) 导丝模式切换：双手柄时由 587 按键进入独立导丝；协同递送仅由 UI 显式请求。
		bool cooperative_transition_failed = false;
		if (requested_guidewire_mode != requested_guidewire_mode_prev)
		{
			if (requested_guidewire_mode == GuidewireMode::None)
			{
				cooperative_direction = CooperativeDirection::None;
				cooperative_direction_requested = CooperativeDirection::None;
				if (guidewire_mode != GuidewireMode::None)
				{
					if (!freeze_active && !estop_hold_active && !startup_sequence_active)
					{
						if (exit_guidewire_mode_to_normal())
						{
							cooperative_return_owner = CooperativeReturnOwner::None;
							std::cout << "导丝模式：关闭。" << std::endl;
						}
						else
						{
							guidewire_mode = GuidewireMode::None;
							axis6_crawl.enabled = false;
							axis6_window_locked = false;
							axis6_coop_ff_inited = false;
							axis6_coop_prev_axis1_cmd_abs = 0.0;
							cooperative_return_owner = CooperativeReturnOwner::None;
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
						cooperative_return_owner = CooperativeReturnOwner::None;
					}
				}
			}
			else if (freeze_active)
			{
				std::cout << "导丝模式切换已忽略：582 暂停处于开启状态。" << std::endl;
				cooperative_transition_failed = (requested_guidewire_mode == GuidewireMode::Cooperative);
			}
			else if (estop_hold_active)
			{
				std::cout << "导丝模式切换已忽略：PLC 保持处于开启状态。" << std::endl;
				cooperative_transition_failed = (requested_guidewire_mode == GuidewireMode::Cooperative);
			}
			else if (!startup.completed || startup.phase != StartupPhase::Done)
			{
				std::cout << "导丝模式切换已忽略：启动准备尚未完成。" << std::endl;
				cooperative_transition_failed = (requested_guidewire_mode == GuidewireMode::Cooperative);
			}
			else if (!control_active || return_ads_fault_hold)
			{
				std::cout << "导丝模式切换已忽略：控制尚未激活。" << std::endl;
				cooperative_transition_failed = (requested_guidewire_mode == GuidewireMode::Cooperative);
			}
			else
			{
				bool mode_ok = false;
				bool mode_attempted = false;
				bool mode_rejected = false;
				const bool cooperative_request = (requested_guidewire_mode == GuidewireMode::Cooperative);
				if (cooperative_request)
				{
					mode_rejected = !validate_cooperative_entry(cooperative_direction_requested);
				}
				else
				{
					double axis6_from_left_mm = 0.0;
					const bool gate_checked = check_axis6_guidewire_entry_gate(axis6_from_left_mm);
					if (!gate_checked)
					{
						std::cout << "导丝模式切换失败：无法读取 axis6 进入门控的 PLC 状态。" << std::endl;
						mode_rejected = true;
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
						mode_rejected = true;
					}
				}

				if (!mode_rejected)
				{
					mode_attempted = true;
					if (requested_guidewire_mode == GuidewireMode::Independent)
					{
						mode_ok = enter_independent_guidewire_mode();
						if (mode_ok)
						{
							guidewire_mode = GuidewireMode::Independent;
							cooperative_direction = CooperativeDirection::None;
							cooperative_return_owner = CooperativeReturnOwner::None;
							std::cout << (axis6_effective_reverse_pressed ? "导丝模式：反向取出。" : "导丝模式：正向输送。") << std::endl;
						}
					}
					else if (requested_guidewire_mode == GuidewireMode::Cooperative)
					{
						mode_ok = enter_cooperative_guidewire_mode();
						if (mode_ok)
						{
							guidewire_mode = GuidewireMode::Cooperative;
							// 本拍在切换前已经采样过 b0；成功进入后立即覆盖为
							// 固定协同方向，避免首次控制带入旧模式方向。
							cooperative_direction = cooperative_direction_requested;
							cooperative_mode_active = true;
							cooperative_retraction_active =
								cooperative_direction == CooperativeDirection::Retraction;
							axis1_reverse_pressed = cooperative_retraction_active;
							axis6_effective_reverse_pressed = cooperative_retraction_active;
							cooperative_return_owner = CooperativeReturnOwner::None;
							reset_cooperative_direction_guards(cooperative_direction);
							axis1_reverse_pressed_prev = axis1_reverse_pressed;
							axis6_effective_reverse_prev = axis6_effective_reverse_pressed;
							clear_force_output();
							std::cout << cooperative_direction_text(cooperative_direction)
								<< "：已进入，物理 SN 587 控制导管，物理 SN 582 控制导丝。" << std::endl;
						}
					}
				}

				if (mode_attempted && !mode_ok)
				{
					std::cout << "导丝模式切换失败。" << std::endl;
				}
				if (cooperative_request && !mode_ok)
				{
					// 入口失败后取消 UI 请求，下一帧按此前的普通 UI/物理模式恢复，
					// 防止因持续重试影响原模式。
					cooperative_direction_requested = CooperativeDirection::None;
					cooperative_direction = CooperativeDirection::None;
					cooperative_return_owner = CooperativeReturnOwner::None;
					cooperative_transition_failed = true;
				}
			}
		}
		requested_guidewire_mode_prev = cooperative_transition_failed
			? guidewire_mode
			: requested_guidewire_mode;

		// 两个协同方向之间允许一键安全切换。切换仅在两轴均未进入换手、
		// PLC 无残留回退且入口几何条件重新通过时执行；同步采样不会产生位移命令。
		if (!cooperative_transition_failed &&
			guidewire_mode == GuidewireMode::Cooperative &&
			cooperative_direction != CooperativeDirection::None &&
			cooperative_direction_requested != CooperativeDirection::None &&
			cooperative_direction_requested != cooperative_direction)
		{
			const CooperativeDirection requested_direction = cooperative_direction_requested;
			if (!validate_cooperative_entry(requested_direction))
			{
				// 切换失败时继续保持原协同方向，WPF 依据状态快照自动校正选中项。
				cooperative_direction_requested = cooperative_direction;
			}
			else if (!sync_cooperative_guidewire(20, true))
			{
				std::cout << cooperative_direction_text(requested_direction)
					<< "切换被拒绝：双手柄重同步失败，已保持原协同方向。" << std::endl;
				cooperative_direction_requested = cooperative_direction;
			}
			else
			{
				cooperative_direction = requested_direction;
				cooperative_retraction_active =
					cooperative_direction == CooperativeDirection::Retraction;
				axis1_reverse_pressed = cooperative_retraction_active;
				axis6_effective_reverse_pressed = cooperative_retraction_active;
				reset_cooperative_direction_guards(cooperative_direction);
				axis1_reverse_pressed_prev = axis1_reverse_pressed;
				axis6_effective_reverse_prev = axis6_effective_reverse_pressed;
				clear_force_output();
				std::cout << cooperative_direction_text(cooperative_direction)
					<< "：已完成双手柄重同步，等待新的手柄增量。" << std::endl;
			}
		}

		// 6) 周期性响应 PLC 自检完成与 PLC 请求的重同步。
		if (has_self_check_flag && (loop_count % 50) == 0)
		{
			if (ads.ADSRead(AdsSymbol::self_check_done, sizeof(self_check_done), &self_check_done))
			{
				if (last_self_check_done && !self_check_done)
				{
					cancel_cooperative_delivery(true);
					control_active = false;
					clear_force_output();
					std::cout << "PLC 自检重新开始，已退出协同递送。" << std::endl;
				}
				if (!last_self_check_done && self_check_done)
				{
					spacing_recovery.reset();
					cancel_cooperative_delivery(true);
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
					spacing_recovery.reset();
					cancel_cooperative_delivery(true);
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
		if (!control_active && !return_ads_fault_hold && !motion_startup_active &&
			!freeze_active && !estop_hold_active && startup.completed)
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
		// mode_code: 0=导管, 1=独立导丝, 2=协同递送, 3=协同撤出
		// reverse_code: 活动模式对应的正反向键状态(0/1)
		// push_pull_code: +1/-1/0（活动模式线性增量方向）
		// rot_sign_code: +1/-1/0（活动模式旋转关节增量符号）
		int force_mode_code =
			(guidewire_mode == GuidewireMode::None) ? 0 :
			((guidewire_mode == GuidewireMode::Independent) ? 1 :
				(cooperative_direction == CooperativeDirection::Retraction ? 3 : 2));
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
		// 轴4 接线方向与按键语义相反，此处交换映射。
		bool axis4_manual_forward_req = axis4_reverse_request;
		bool axis4_manual_reverse_req = axis4_forward_request;
		tracking_manual_cylinder_override = false;
		for (int cylinder_index = 0; cylinder_index < 4; ++cylinder_index)
		{
			if (cylinder_manual_mode[cylinder_index] != CylinderManualMode::Automatic)
			{
				tracking_manual_cylinder_override = true;
				break;
			}
		}

		// 9) 根据当前顶层模式构建一帧 refer 和一组气缸指令。
		if (!return_ads_fault_hold && !freeze_active && !estop_hold_active &&
			(control_active || motion_startup_active) && read_plc_state())
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
			const double axis5_abs = plc_act_pos[4] + plc_init_pos[4];
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
			if (!axis6_soft_limit_hold && axis6_target_exceeds_soft_limit(axis6_abs))
			{
				engage_axis6_soft_limit_hold(axis6_abs, "axis6 实际位置");
			}
			const double axis6_linear_increment_raw_mm =
				(axis6_linear_filtered - axis6_prev_linear_filtered) * cfg.k_handle_to_mm * cfg.axis_push_sign;
			const double axis6_linear_increment_mm =
				(std::abs(axis6_linear_increment_raw_mm) >= cfg.linear_increment_noise_deadband_mm)
				? axis6_linear_increment_raw_mm
				: 0.0;
			const bool axis6_linear_increment_active = std::abs(axis6_linear_increment_mm) > 0.0;

			// 主从位移实验门控：只认可普通导管递送 axis1 与独立导丝递送 axis6。
			// 夹持成立仅是命令保持 150 ms 的实验假设，并不替代物理到位传感器。
			const bool tracking_no_return_active =
				axis1_crawl.phase == CrawlState::Phase::Follow &&
				axis6_crawl.phase == CrawlState::Phase::Follow &&
				!axis1_crawl.plc_move_requested &&
				!axis6_crawl.plc_move_requested &&
				!axis6_coupled_active &&
				!axis1_post_return_lead_armed &&
				!axis1_post_return_lead_active &&
				!axis6_soft_limit_hold;
			auto tracking_reason_for = [&](bool forward_mode) -> TrackingInvalidReason
			{
				if (!tracking_logger.is_running()) return TrackingInvalidReason::NotLogging;
				if (!forward_mode) return TrackingInvalidReason::NotForwardDelivery;
				if (!control_active) return TrackingInvalidReason::ControlInactive;
				if (motion_startup_active) return TrackingInvalidReason::StartupActive;
				if (freeze_active) return TrackingInvalidReason::Paused;
				if (estop_hold_active) return TrackingInvalidReason::PlcHold;
				if (return_ads_fault_hold) return TrackingInvalidReason::AdsReturnFault;
				if (spacing_recovery.active() || spacing_recovery.requested) return TrackingInvalidReason::SpacingRecovery;
				if (ft_exp.active()) return TrackingInvalidReason::ForceTransitionExperiment;
				if (tracking_manual_cylinder_override) return TrackingInvalidReason::ManualCylinderOverride;
				if (!tracking_no_return_active) return TrackingInvalidReason::CrawlReturnActive;
				return TrackingInvalidReason::None;
			};
			const TrackingInvalidReason axis1_tracking_reason = tracking_reason_for(
				guidewire_mode == GuidewireMode::None && !axis1_reverse_pressed);
			const TrackingInvalidReason axis6_tracking_reason = tracking_reason_for(
				guidewire_mode == GuidewireMode::Independent && !axis6_effective_reverse_pressed);
			// 先按当前 Follow 的预期夹持状态推进控制；本拍末尾还会按最终命令复核。
			const bool axis1_grip_command_active =
				cylinder2_cmd == cyl.cyl2_clamp && cylinder1_cmd == cyl.cyl1_open;
			const bool axis6_grip_command_expected =
				guidewire_mode == GuidewireMode::Independent &&
				axis6_crawl.phase == CrawlState::Phase::Follow &&
				!tracking_manual_cylinder_override;
			tracking_controller.update_gate(
				DeliveryTrackingAxis::Axis1,
				axis1_tracking_reason == TrackingInvalidReason::None,
				axis1_grip_command_active,
				now_ms,
				axis1_abs,
				axis1_tracking_reason);
			tracking_controller.update_gate(
				DeliveryTrackingAxis::Axis6,
				axis6_tracking_reason == TrackingInvalidReason::None,
				axis6_grip_command_expected,
				now_ms,
				axis6_abs,
				axis6_tracking_reason);
			tracking_controller.begin_cycle(
				DeliveryTrackingAxis::Axis1,
				axis1_input_handle->fJoints2[0],
				axis1_linear_filtered,
				axis1_linear_increment_raw_mm,
				plc_act_pos[0]);
			tracking_controller.begin_cycle(
				DeliveryTrackingAxis::Axis6,
				axis6_input_handle->fJoints2[0],
				axis6_linear_filtered,
				axis6_linear_increment_raw_mm,
				plc_act_pos[5]);
			const bool axis1_forward_tracking_mode =
				guidewire_mode == GuidewireMode::None && !axis1_reverse_pressed;
			const bool axis6_forward_tracking_mode =
				guidewire_mode == GuidewireMode::Independent && !axis6_effective_reverse_pressed;
			const bool axis1_handover_active = axis1_forward_tracking_mode &&
				(axis1_crawl.phase != CrawlState::Phase::Follow ||
					axis1_crawl.plc_move_requested || axis6_coupled_active);
			const bool axis6_handover_active = axis6_forward_tracking_mode &&
				(axis6_crawl.phase != CrawlState::Phase::Follow || axis6_crawl.plc_move_requested);
			if (axis1_handover_active)
			{
				// 正向递送坐标朝左为负；换手时只把持续前推转换成正欠账，回拉复位不计入。
				tracking_controller.record_handover_forward_increment(
					DeliveryTrackingAxis::Axis1,
					(std::max)(0.0, -axis1_linear_increment_mm));
			}
			if (axis6_handover_active)
			{
				tracking_controller.record_handover_forward_increment(
					DeliveryTrackingAxis::Axis6,
					(std::max)(0.0, -axis6_linear_increment_mm));
			}
			const bool tracking_segment_active =
				tracking_controller.snapshot(DeliveryTrackingAxis::Axis1).segment_active ||
				tracking_controller.snapshot(DeliveryTrackingAxis::Axis6).segment_active;
			const DeliveryTrackingAxisSnapshot axis1_tracking_gate_snapshot =
				tracking_controller.snapshot(DeliveryTrackingAxis::Axis1);
			const DeliveryTrackingAxisSnapshot axis6_tracking_gate_snapshot =
				tracking_controller.snapshot(DeliveryTrackingAxis::Axis6);
			const bool tracking_handover_or_regrip =
				axis1_tracking_reason == TrackingInvalidReason::CrawlReturnActive ||
				axis6_tracking_reason == TrackingInvalidReason::CrawlReturnActive ||
				axis1_tracking_reason == TrackingInvalidReason::GripSettling ||
				axis6_tracking_reason == TrackingInvalidReason::GripSettling ||
				axis1_tracking_gate_snapshot.invalid_reason ==
					static_cast<int>(TrackingInvalidReason::CrawlReturnActive) ||
				axis6_tracking_gate_snapshot.invalid_reason ==
					static_cast<int>(TrackingInvalidReason::CrawlReturnActive) ||
				axis1_tracking_gate_snapshot.invalid_reason ==
					static_cast<int>(TrackingInvalidReason::GripSettling) ||
				axis6_tracking_gate_snapshot.invalid_reason ==
					static_cast<int>(TrackingInvalidReason::GripSettling);
			if (tracking_controller.compensation_enabled() && !tracking_segment_active &&
				!tracking_handover_or_regrip)
			{
				tracking_controller.disable_compensation();
				std::cout << "主从位移补偿已关闭：已离开正向递送/换手恢复范围。" << std::endl;
			}
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

			const bool cooperative_axis7_locked =
				guidewire_mode == GuidewireMode::Cooperative &&
				cooperative_return_owner == CooperativeReturnOwner::Axis1;
			const double axis7_cmd_rel =
				(guidewire_mode == GuidewireMode::None || cooperative_axis7_locked)
				? axis7_hold_rel
				: compute_axis7_cmd_rel();
			pos[6] = axis7_cmd_rel;

			auto sync_axis6_after_return = [&](const char* failure_message) -> bool
			{
				const bool cooperative_axis6_return =
					guidewire_mode == GuidewireMode::Cooperative &&
					cooperative_return_owner == CooperativeReturnOwner::Axis6;
				const bool synced = cooperative_axis6_return
					? sync_cooperative_guidewire(3, false)
					: sync_axis6(3, false, false);
				if (!synced)
				{
					std::cout << failure_message << std::endl;
					if (cooperative_axis6_return)
					{
						cancel_cooperative_delivery(true);
						return_ads_fault_hold = true;
						control_active = false;
						clear_force_output();
						std::cout << "协同模式已因重同步失败停止上位机运动控制。" << std::endl;
					}
					return false;
				}
				if (cooperative_axis6_return)
				{
					cooperative_return_owner = CooperativeReturnOwner::None;
				}
				return true;
			};

			// axis6 软限位锁止时，相关导管/导丝链路统一保持实际位置，
			// 防止 axis6 已停而 axis1/3/5 或旋转轴继续造成相对拉扯。
				auto hold_axis6_related_axes = [&]()
				{
					pos[0] = plc_act_pos[0];
					pos[1] = plc_act_pos[1];
					pos[2] = plc_act_pos[2];
				pos[4] = plc_act_pos[4];
				pos[5] = plc_act_pos[5];
				pos[6] = plc_act_pos[6];
				axis1_follow_cmd_abs = axis1_abs;
				axis6_follow_cmd_abs = axis6_abs;
					axis2_hold_rel = plc_act_pos[1];
					axis7_hold_rel = plc_act_pos[6];
					axis6_coop_ff_inited = false;
					reset_axis1_post_return_lead();
					// 软件锁止时保持两组夹爪的稳定抓持组合，避免默认 Follow
					// 命令把已经冻结的器械意外释放。
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_clamp;
					cylinder3_cmd = cyl.cyl3_open;
					cylinder4_cmd = cyl.cyl4_clamp;
				};

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
				if (axis6_soft_limit_hold)
				{
					hold_axis6_related_axes();
					return;
				}

				const bool cooperative_axis6_mode = guidewire_mode == GuidewireMode::Cooperative;
				const double axis6_window_left_abs_now = axis6_crawl.min_abs();
				const double axis6_window_right_abs_now = axis6_crawl.max_abs();
				if (axis6_crawl.phase == CrawlState::Phase::Follow)
				{
					double axis6_cmd_abs = axis6_follow_cmd_abs;
					const bool axis6_increment_active = std::abs(axis6_increment_mm) > 0.0;
					if (axis6_increment_active)
					{
						axis6_cmd_abs = clamp_double(axis6_raw_cmd_abs, axis6_window_left_abs_now, axis6_window_right_abs_now);
						if (axis6_target_exceeds_soft_limit(axis6_cmd_abs))
						{
							engage_axis6_soft_limit_hold(axis6_cmd_abs, "axis6 手柄目标");
							hold_axis6_related_axes();
							return;
						}
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
					// 按运动方向判断是否到达或跨过触发边，避免两个 ADS 采样点跨过窄容差窗时漏触发。
					const bool axis6_enter_trigger_edge = axis6_reverse_mode
						? ((axis6_prev_abs < (axis6_trigger_edge_abs - cfg.crawl_arrive_tol_mm)) &&
							(axis6_abs >= (axis6_trigger_edge_abs - cfg.crawl_arrive_tol_mm)))
						: ((axis6_prev_abs > (axis6_trigger_edge_abs + cfg.crawl_arrive_tol_mm)) &&
							(axis6_abs <= (axis6_trigger_edge_abs + cfg.crawl_arrive_tol_mm)));
					const bool axis6_ready_to_trigger =
						axis6_trigger_user_ok &&
						axis6_increment_active &&
						axis6_toward_trigger &&
						axis6_enter_trigger_edge &&
						!axis6_switch_guard_blocked;
					if (axis6_ready_to_trigger &&
						(!cooperative_axis6_mode ||
							cooperative_return_owner == CooperativeReturnOwner::None))
					{
						const double axis6_return_target_abs = axis6_reverse_mode
							? axis6_window_left_abs_now
							: axis6_window_right_abs_now;
						if (axis6_target_exceeds_soft_limit(axis6_return_target_abs))
						{
							engage_axis6_soft_limit_hold(axis6_return_target_abs, "axis6 计划回退");
							hold_axis6_related_axes();
							return;
						}
						axis6_crawl.target_abs = axis6_return_target_abs;
						axis6_return_entry_rel = plc_act_pos[5];
						axis6_crawl.phase = CrawlState::Phase::SwitchWait;
						if (cooperative_axis6_mode)
						{
							cooperative_return_owner = CooperativeReturnOwner::Axis6;
							// 本拍 axis1 Follow 已先完成计算；axis6 一旦取得回退所有权，
							// 立即撤销尚未写出的导管链路目标，保证另一条链路从触发瞬间起保持。
							pos[0] = plc_act_pos[0];
							pos[1] = plc_act_pos[1];
							pos[2] = plc_act_pos[2];
							pos[4] = plc_act_pos[4];
							axis1_follow_cmd_abs = plc_act_pos[0] + plc_init_pos[0];
							axis2_hold_rel = plc_act_pos[1];
						}
						axis6_crawl.phase_t0 = now_ms;
						axis6_crawl.cyl_seq_stage = 0;
						axis6_crawl.cyl_seq_t0 = now_ms;
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
						else
						{
							// ADS 下发失败时立即保持实际位置，禁止快速旁路继续追赶远端目标。
							clear_axis_return_request(AdsSymbol::axis6_return);
							load_pos_from_actual();
							axis6_fast_retract = false;
							axis6_crawl.plc_move_requested = false;
							axis6_crawl.phase = CrawlState::Phase::Follow;
							axis6_follow_cmd_abs = plc_act_pos[5] + plc_init_pos[5];
							if (cooperative_axis6_mode)
							{
								cancel_cooperative_delivery(true);
								clear_force_output();
							}
							return_ads_fault_hold = true;
							control_active = false;
							std::cout << "轴6 计划回退 ADS 下发失败，已保持当前位置并停止上位机运动控制。" << std::endl;
							return;
						}
					}
					else if (read_axis_return_status(AdsSymbol::axis6_return, axis6_return_status))
					{
						if (axis6_return_status.error)
						{
							clear_axis_return_request(AdsSymbol::axis6_return);
							axis6_crawl.plc_move_requested = false;
							std::cout << "轴6 计划回退报错，错误码: " << axis6_return_status.error_id << std::endl;
							(void)sync_axis6_after_return("轴6 计划回退报错后重同步失败。");
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
					if (axis6_crawl.phase == CrawlState::Phase::FastMove &&
						std::abs(axis6_abs - axis6_crawl.target_abs) <= cfg.crawl_arrive_tol_mm)
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
						(void)sync_axis6_after_return("轴6 计划回退后重同步失败。");
						axis6_crawl.phase = CrawlState::Phase::Follow;
						axis6_crawl.cyl_seq_stage = 0;
						axis6_crawl.plc_move_requested = false;
					}
				}
			};

			auto calculate_spacing_recovery_remaining = [&]() -> double
			{
				double remaining = cfg.spacing_recovery_axis3_max_from_left_mm -
					((spacing_recovery.axis3_cmd_rel + plc_init_pos[2]) - plc_leftlimit[2]);
				const double axis5_remaining = cfg.spacing_recovery_axis5_max_from_left_mm -
					((spacing_recovery.axis5_cmd_rel + plc_init_pos[4]) - plc_leftlimit[4]);
				const double axis6_remaining = cfg.spacing_recovery_axis6_max_from_left_mm -
					((spacing_recovery.axis6_cmd_rel + plc_init_pos[5]) - plc_leftlimit[5]);
				if (axis5_remaining < remaining) remaining = axis5_remaining;
				if (axis6_remaining < remaining) remaining = axis6_remaining;
				return remaining > 0.0 ? remaining : 0.0;
			};

			if (spacing_recovery.active() && !spacing_recovery.requested &&
				spacing_recovery.phase != SpacingRecoveryPhase::ExitSync)
			{
				spacing_recovery.phase = SpacingRecoveryPhase::ExitSync;
				spacing_recovery.phase_t0 = now_ms;
				spacing_recovery.pending_delta_mm = 0.0;
				std::cout << "屈曲恢复：正在停止并重建跟随基准。" << std::endl;
			}

			if (spacing_recovery.requested && !spacing_recovery.active())
			{
				const double axis5_from_left_mm = axis5_abs - plc_leftlimit[4];
				const double axis6_from_left_mm = axis6_abs - plc_leftlimit[5];
				const double axis56_gap_mm = axis6_from_left_mm - axis5_from_left_mm;
				const double axis56_max_gap_mm =
					cfg.axis6_window_min_gap_from_axis5_mm + cfg.axis6_window_size_mm;
				const bool prerequisites_ok =
					startup.completed && startup.phase == StartupPhase::Done &&
					control_active && guidewire_mode == GuidewireMode::None &&
					!axis1_reverse_pressed && !ft_exp.active() && !axis6_soft_limit_hold &&
					axis1_crawl.phase == CrawlState::Phase::Follow &&
					axis6_crawl.phase == CrawlState::Phase::Follow &&
					!axis1_crawl.plc_move_requested && !axis6_crawl.plc_move_requested &&
					!axis6_coupled_active;

				spacing_recovery.axis1_hold_rel = plc_act_pos[0];
				spacing_recovery.axis2_hold_rel = plc_act_pos[1];
				spacing_recovery.axis3_cmd_rel = plc_act_pos[2];
				spacing_recovery.axis5_cmd_rel = plc_act_pos[4];
				spacing_recovery.axis6_cmd_rel = plc_act_pos[5];
				spacing_recovery.axis7_hold_rel = plc_act_pos[6];
				spacing_recovery.remaining_mm = calculate_spacing_recovery_remaining();

				if (!prerequisites_ok)
				{
					std::cout << "屈曲恢复进入被拒绝：需处于已启动的导管递送 Follow 状态，且无回退或力过渡实验。" << std::endl;
					spacing_recovery.reset();
				}
				else if (axis5_from_left_mm + cfg.crawl_arrive_tol_mm < axis3_from_left_mm ||
					axis56_gap_mm < (cfg.axis6_window_min_gap_from_axis5_mm - cfg.crawl_arrive_tol_mm) ||
					axis56_gap_mm > (axis56_max_gap_mm + cfg.crawl_arrive_tol_mm))
				{
					std::cout << "屈曲恢复进入被拒绝：轴3/5/6当前相对位置不满足安全窗口。" << std::endl;
					spacing_recovery.reset();
				}
				else if (spacing_recovery.remaining_mm <= cfg.crawl_arrive_tol_mm)
				{
					std::cout << "屈曲恢复进入被拒绝：轴3/5/6已无可用恢复行程。" << std::endl;
					spacing_recovery.reset();
				}
				else
				{
					clear_axis1_group_return_requests();
					clear_axis_return_request(AdsSymbol::axis6_return);
					for (int cylinder_index = 0; cylinder_index < 4; ++cylinder_index)
					{
						cylinder_manual_mode[cylinder_index] = CylinderManualMode::Automatic;
					}
					spacing_recovery.phase = SpacingRecoveryPhase::ClampWait;
					spacing_recovery.phase_t0 = now_ms;
					spacing_recovery.last_motion_tick = now_ms;
					spacing_recovery.pending_delta_mm = 0.0;
					spacing_recovery.moved_mm = 0.0;
					spacing_recovery.limit_logged = false;
					clear_force_output();
					std::cout << "屈曲恢复：夹爪准备中，轴1保持；恢复速度上限 "
						<< cfg.spacing_recovery_speed_limit_mm_s << " mm/s。" << std::endl;
				}
			}

			if (axis6_soft_limit_hold)
			{
				hold_axis6_related_axes();
			}
			else if (spacing_recovery.active())
			{
				// 固定导管侧，释放随轴3/5/6移动的夹爪；电缸4保留轻限位，防止导丝脱出。
				cylinder1_cmd = cyl.cyl1_open;
				cylinder2_cmd = cyl.cyl2_clamp;
				cylinder3_cmd = cyl.cyl3_open;
				cylinder4_cmd = cfg.spacing_recovery_cyl4_release;
				pos[0] = spacing_recovery.axis1_hold_rel;
				pos[1] = spacing_recovery.axis2_hold_rel;
				pos[6] = spacing_recovery.axis7_hold_rel;

				if (spacing_recovery.phase == SpacingRecoveryPhase::ClampWait)
				{
					spacing_recovery.pending_delta_mm = 0.0;
					if ((now_ms - spacing_recovery.phase_t0) >= cfg.spacing_recovery_clamp_settle_ms)
					{
						spacing_recovery.phase = SpacingRecoveryPhase::Active;
						spacing_recovery.last_motion_tick = now_ms;
						std::cout << "屈曲恢复：已激活，582 手柄仅控制轴3/5/6向远离左限位方向移动。" << std::endl;
					}
				}
				else if (spacing_recovery.phase == SpacingRecoveryPhase::Active)
				{
					DWORD elapsed_ms = now_ms - spacing_recovery.last_motion_tick;
					spacing_recovery.last_motion_tick = now_ms;
					if (elapsed_ms > 50) elapsed_ms = 50;

					if (axis1_linear_increment_mm > 0.0)
					{
						spacing_recovery.pending_delta_mm += axis1_linear_increment_mm;
					}
					else
					{
						// 不累计反方向或已经停止的手柄输入，避免停止推拉后继续运动。
						spacing_recovery.pending_delta_mm = 0.0;
					}

					spacing_recovery.remaining_mm = calculate_spacing_recovery_remaining();
					const double max_step_mm = cfg.spacing_recovery_speed_limit_mm_s *
						(static_cast<double>(elapsed_ms) / 1000.0);
					double step_mm = spacing_recovery.pending_delta_mm;
					if (step_mm > max_step_mm) step_mm = max_step_mm;
					if (step_mm > spacing_recovery.remaining_mm) step_mm = spacing_recovery.remaining_mm;

					if (step_mm > 0.0)
					{
						spacing_recovery.axis3_cmd_rel += step_mm;
						spacing_recovery.axis5_cmd_rel += step_mm;
						spacing_recovery.axis6_cmd_rel += step_mm;
						spacing_recovery.pending_delta_mm -= step_mm;
						spacing_recovery.moved_mm += step_mm;
						spacing_recovery.remaining_mm = calculate_spacing_recovery_remaining();
					}
					if (spacing_recovery.remaining_mm <= cfg.crawl_arrive_tol_mm)
					{
						spacing_recovery.pending_delta_mm = 0.0;
						if (!spacing_recovery.limit_logged)
						{
							std::cout << "屈曲恢复：已到达轴3/5/6共同安全行程上限。" << std::endl;
							spacing_recovery.limit_logged = true;
						}
					}
				}

				pos[2] = spacing_recovery.axis3_cmd_rel;
				pos[4] = spacing_recovery.axis5_cmd_rel;
				pos[5] = spacing_recovery.axis6_cmd_rel;
			}
			else if (motion_startup_active)
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
				const double startup_final_axis1_abs = from_left_to_abs(0, startup.final_axis1_from_left_mm);
				const double startup_final_axis3_abs = from_left_to_abs(2, startup.final_axis3_from_left_mm);
				const double startup_final_axis5_abs = from_left_to_abs(4, startup.final_axis5_from_left_mm);
				const double startup_final_axis6_abs = from_left_to_abs(5, startup.final_axis6_from_left_mm);
				const double axis1_abs = plc_act_pos[0] + plc_init_pos[0];
				const double axis5_abs = plc_act_pos[4] + plc_init_pos[4];
				const double axis6_abs_now = plc_act_pos[5] + plc_init_pos[5];
				const double axis3_abs = plc_act_pos[2] + plc_init_pos[2];
				const double axis2_rel = plc_act_pos[1];
				const double axis7_rel = plc_act_pos[6];

				auto apply_startup_final_targets = [&]()
				{
					pos[0] = from_left_to_rel(0, startup.final_axis1_from_left_mm);
					pos[1] = startup.final_axis2_deg;
					pos[2] = from_left_to_rel(2, startup.final_axis3_from_left_mm);
					pos[4] = from_left_to_rel(4, startup.final_axis5_from_left_mm);
					pos[5] = from_left_to_rel(5, startup.final_axis6_from_left_mm);
					pos[6] = startup.final_axis7_deg;
				};
				auto startup_final_targets_reached = [&]() -> bool
				{
					return (std::abs(axis1_abs - startup_final_axis1_abs) <= cfg.crawl_arrive_tol_mm) &&
						(std::abs(axis3_abs - startup_final_axis3_abs) <= cfg.crawl_arrive_tol_mm) &&
						(std::abs(axis5_abs - startup_final_axis5_abs) <= cfg.crawl_arrive_tol_mm) &&
						(std::abs(axis6_abs_now - startup_final_axis6_abs) <= cfg.crawl_arrive_tol_mm) &&
						(std::abs(axis2_rel - startup.final_axis2_deg) <= cfg.startup_rot_arrive_tol_deg) &&
						(std::abs(axis7_rel - startup.final_axis7_deg) <= cfg.startup_rot_arrive_tol_deg);
				};

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
					// 阶段 3：在 axis3 移动前先闭合导丝侧夹爪对。
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
						std::cout << "启动准备：阶段4按 UI 最终目标移动 1/2/3/5/6/7 轴。" << std::endl;
					}
				}
				else if (startup.phase == StartupPhase::MoveAxis356BackToReady)
				{
					// 阶段 4：所有 UI 最终目标在此阶段生效，前置阶段仍保持固定准备位。
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_open;
					cylinder3_cmd = cfg.startup_cyl3_clamp;
					cylinder4_cmd = cfg.startup_cyl4_clamp;
					apply_startup_final_targets();
					if (startup_final_targets_reached())
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
					apply_startup_final_targets();
					if ((now_ms - startup.phase_t0) >= cfg.startup_clamp_settle_delay_ms &&
						startup_final_targets_reached())
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
				const double axis6_follow_start_abs = axis6_follow_cmd_abs;
				double axis6_nominal_cmd_abs = axis6_follow_start_abs;
				if (axis6_crawl.phase == CrawlState::Phase::Follow && axis6_linear_increment_active)
				{
					axis6_nominal_cmd_abs = clamp_double(
						axis6_follow_start_abs + axis6_linear_increment_mm,
						axis6_crawl.min_abs(),
						axis6_crawl.max_abs());
				}
				const double axis6_nominal_delta_axis_mm = axis6_nominal_cmd_abs - axis6_follow_start_abs;
				const double axis6_nominal_forward_mm =
					(!axis6_effective_reverse_pressed && axis6_nominal_delta_axis_mm < 0.0)
					? -axis6_nominal_delta_axis_mm : 0.0;
				double axis6_requested_forward_mm = axis6_nominal_forward_mm;
				double axis6_raw_cmd_abs = axis6_follow_start_abs + axis6_linear_increment_mm;
				double axis6_increment_for_state_mm = axis6_linear_increment_mm;
				if (axis6_nominal_forward_mm > 0.0)
				{
					axis6_requested_forward_mm = tracking_controller.request_compensated_forward_increment(
						DeliveryTrackingAxis::Axis6,
						axis6_nominal_forward_mm,
						now_ms);
					axis6_raw_cmd_abs = axis6_follow_start_abs - axis6_requested_forward_mm;
					axis6_increment_for_state_mm = -axis6_requested_forward_mm;
				}
				run_axis6_crawl_state(
					axis6_raw_cmd_abs,
					axis6_increment_for_state_mm,
					axis6_effective_reverse_pressed,
					axis6_linear_increment_active,
					false);
				const double axis6_effective_delta_axis_mm = axis6_follow_cmd_abs - axis6_follow_start_abs;
				const double axis6_effective_forward_mm =
					(!axis6_effective_reverse_pressed && axis6_effective_delta_axis_mm < 0.0)
					? -axis6_effective_delta_axis_mm : 0.0;
				const bool axis6_tracking_output_clamped = axis6_nominal_forward_mm > 0.0 &&
					std::abs(axis6_effective_forward_mm - axis6_requested_forward_mm) > 1e-6;
				tracking_controller.commit_command(
					DeliveryTrackingAxis::Axis6,
					axis6_nominal_forward_mm,
					axis6_requested_forward_mm,
					axis6_effective_forward_mm,
					axis6_tracking_output_clamped);
			}
			else
			{
				// 常规导管模式：
				// - axis1/2 由 handle 582 控制
				// - 轴 3/5 在 Follow 阶段镜像 axis1 平移
				// - axis6 在导管模式 Follow 阶段保持不动
				// - axis1 触发快退时，axis6 按反向等位移联动，并限制在轴5相对窗口内
				const bool cooperative_mode = (guidewire_mode == GuidewireMode::Cooperative);
				// 协同回退只允许一条链路拥有 PLC 计划动作。若运行态在异常边沿进入
				// 非 Follow，相同周期内优先判定 axis1，避免两个 return_cmd 并发下发。
				if (cooperative_mode && cooperative_return_owner == CooperativeReturnOwner::None)
				{
					if (axis1_crawl.phase != CrawlState::Phase::Follow)
					{
						cooperative_return_owner = CooperativeReturnOwner::Axis1;
					}
					else if (axis6_crawl.phase != CrawlState::Phase::Follow)
					{
						cooperative_return_owner = CooperativeReturnOwner::Axis6;
					}
				}
				const bool cooperative_axis1_locked =
					cooperative_mode && cooperative_return_owner == CooperativeReturnOwner::Axis6;
				double axis6_catheter_window_start_abs = axis6_abs;
				double axis6_catheter_window_end_abs = axis6_abs;
				if (!cooperative_mode)
				{
					// 导管模式下窗口随轴5移动，仅用于约束轴1快退时的轴6联动目标。
					motion_sync::calculate_axis6_window_from_axis5(
						ctx,
						axis6_catheter_window_start_abs,
						axis6_catheter_window_end_abs);
					axis6_crawl.start_abs = axis6_catheter_window_start_abs;
					axis6_crawl.end_abs = axis6_catheter_window_end_abs;
					axis6_crawl.window_active = is_within_range(
						axis6_abs,
						axis6_crawl.min_abs(),
						axis6_crawl.max_abs(),
						cfg.crawl_arrive_tol_mm);
				}
				const bool axis1_now_in_window = is_within_range(axis1_abs, axis1_min_abs, axis1_max_abs, cfg.crawl_arrive_tol_mm);
				if (!cooperative_axis1_locked && !axis1_crawl.window_active && axis1_now_in_window)
				{
					capture_axis1_follow_baseline();
					axis1_crawl.window_active = true;
				}

				if (cooperative_axis1_locked)
				{
					// axis6 回退期间冻结导管链路；本拍采样差分会在循环末尾丢弃，
					// 不会在回退结束后累积成一次目标跳变。
					pos[0] = plc_act_pos[0];
					pos[1] = axis2_hold_rel;
					pos[2] = plc_act_pos[2];
					pos[4] = plc_act_pos[4];
					axis6_coop_ff_inited = false;
				}
				else if (!cooperative_mode &&
					!axis1_reverse_pressed &&
					(axis1_post_return_lead_armed || axis1_post_return_lead_active))
				{
					// 普通导管正向递送中，每次计划回退完成后的首次有效前推，
					// 先只让 axis1 走设定的短位移，用于补偿轴1与轴3的循环位移差。
					// 此阶段 axis3/5/6 与两个旋转轴均保持当前实际位置，不能镜像跟随。
					pos[0] = plc_act_pos[0];
					pos[1] = plc_act_pos[1];
					pos[2] = plc_act_pos[2];
					pos[4] = plc_act_pos[4];
					pos[5] = plc_act_pos[5];
					pos[6] = plc_act_pos[6];
					axis2_hold_rel = plc_act_pos[1];
					axis7_hold_rel = plc_act_pos[6];
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_clamp;
					cylinder3_cmd = cyl.cyl3_follow_release;
					cylinder4_cmd = cyl.cyl4_clamp;

					if (axis1_post_return_lead_armed && !axis1_post_return_lead_active)
					{
						const double configured_lead_mm = cfg.axis1_post_return_lead_mm;
						if (axis3_delivery_stop_active || axis1_delivery_stop_latched ||
							std::abs(configured_lead_mm) <= 1e-6)
						{
							// 先行不得绕过原有轴3投送停止位；零先行量等价于取消本次武装。
							reset_axis1_post_return_lead();
						}
						else if (axis1_linear_increment_mm < -cfg.linear_increment_noise_deadband_mm)
						{
							const double lead_target_abs = clamp_double(
								axis1_follow_cmd_abs - configured_lead_mm,
								axis1_window_left_abs_now,
								axis1_window_right_abs_now);

							// 触发先行的这一次手柄增量不再叠加到后续 Follow，
							// 先行完成后还会用 sync_axis1 重建完整基准。
							axis1_prev_linear_filtered = axis1_linear_filtered;
							axis1_prev_rot_filtered = axis1_rot_filtered;
							if (std::abs(lead_target_abs - axis1_abs) <= cfg.crawl_arrive_tol_mm)
							{
								reset_axis1_post_return_lead();
								axis1_follow_cmd_abs = axis1_abs;
								std::cout << "轴1 回退后先行已跳过：当前窗口内无可用先行行程。" << std::endl;
							}
							else
							{
								axis1_post_return_lead_armed = false;
								axis1_post_return_lead_active = true;
								axis1_post_return_lead_target_abs = lead_target_abs;
								std::cout << "轴1 回退后先行已触发："
									<< configured_lead_mm << " mm。" << std::endl;
							}
						}
					}

					if (axis1_post_return_lead_active)
					{
						pos[0] = axis1_post_return_lead_target_abs - plc_init_pos[0];
						axis1_follow_cmd_abs = axis1_post_return_lead_target_abs;
						if (std::abs(axis1_abs - axis1_post_return_lead_target_abs) <=
							cfg.crawl_arrive_tol_mm)
						{
							if (sync_axis1(3))
							{
								reset_axis1_post_return_lead();
								std::cout << "轴1 回退后先行完成，已重建手柄与镜像轴基准。" << std::endl;
							}
							else
							{
								reset_axis1_post_return_lead();
								return_ads_fault_hold = true;
								control_active = false;
								clear_force_output();
								std::cout << "轴1 回退后先行完成，但重同步失败，已停止上位机运动控制。" << std::endl;
							}
						}
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::Follow)
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

					const double axis1_follow_start_abs = axis1_follow_cmd_abs;
					auto constrain_axis1_follow_cmd_abs = [&](double candidate_abs) -> double
					{
						double command_abs = axis1_follow_start_abs;
						if (axis1_linear_increment_active && axis1_follow_enabled)
						{
							command_abs = axis1_crawl.window_active
								? clamp_double(candidate_abs, axis1_window_left_abs_now, axis1_window_right_abs_now)
								: candidate_abs;
						}
						if (axis1_crawl.window_active)
						{
							command_abs = clamp_double(command_abs, axis1_window_left_abs_now, axis1_window_right_abs_now);
						}
						if (!axis1_reverse_pressed && command_abs < axis1_window_left_abs_now)
						{
							command_abs = axis1_window_left_abs_now;
						}
						if (axis1_delivery_stop_latched && !axis1_reverse_pressed)
						{
							command_abs = axis1_window_left_abs_now;
						}
						return command_abs;
					};

					const double axis1_nominal_cmd_abs = constrain_axis1_follow_cmd_abs(axis1_raw_cmd_abs);
					const double axis1_nominal_delta_axis_mm = axis1_nominal_cmd_abs - axis1_follow_start_abs;
					const double axis1_nominal_forward_mm =
						(!axis1_reverse_pressed && axis1_nominal_delta_axis_mm < 0.0)
						? -axis1_nominal_delta_axis_mm : 0.0;
					double axis1_cmd_abs = axis1_nominal_cmd_abs;
					double axis1_requested_forward_mm = axis1_nominal_forward_mm;
					bool axis1_tracking_output_clamped = false;
					if (axis1_nominal_forward_mm > 0.0)
					{
						axis1_requested_forward_mm = tracking_controller.request_compensated_forward_increment(
							DeliveryTrackingAxis::Axis1,
							axis1_nominal_forward_mm,
							now_ms);
						const double axis1_compensated_target_abs =
							axis1_follow_start_abs - axis1_requested_forward_mm;
						axis1_cmd_abs = constrain_axis1_follow_cmd_abs(axis1_compensated_target_abs);
						axis1_tracking_output_clamped =
							std::abs(axis1_cmd_abs - axis1_compensated_target_abs) > 1e-6;
					}
					const double axis1_effective_delta_axis_mm = axis1_cmd_abs - axis1_follow_start_abs;
					const double axis1_effective_forward_mm =
						(!axis1_reverse_pressed && axis1_effective_delta_axis_mm < 0.0)
						? -axis1_effective_delta_axis_mm : 0.0;
					tracking_controller.commit_command(
						DeliveryTrackingAxis::Axis1,
						axis1_nominal_forward_mm,
						axis1_requested_forward_mm,
						axis1_effective_forward_mm,
						axis1_tracking_output_clamped);

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
						// 按运动方向判断是否到达或跨过触发边，避免高速时越过 ±tol 后漏掉回退。
						const bool axis1_enter_trigger_edge = axis1_reverse_pressed
							? ((axis1_prev_abs < (axis1_trigger_edge_abs - cfg.crawl_arrive_tol_mm)) &&
								(axis1_abs >= (axis1_trigger_edge_abs - cfg.crawl_arrive_tol_mm)))
							: ((axis1_prev_abs > (axis1_trigger_edge_abs + cfg.crawl_arrive_tol_mm)) &&
								(axis1_abs <= (axis1_trigger_edge_abs + cfg.crawl_arrive_tol_mm)));
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
								const double axis1_return_target_abs = axis1_reverse_pressed
									? axis1_window_left_abs_now
									: axis1_window_right_abs_now;
								double axis6_coupled_target_raw_abs = axis6_abs;
								double candidate_axis6_coupled_target_abs = axis6_abs;
								bool axis6_coupled_target_safe = true;
								if (!cooperative_mode)
								{
									// 在切缸、进入 SwitchWait、下发 PLC 回退之前先验证
									// axis1 回退引起的 axis6 联动终点，防止下一次换手越过 670 mm 软限位。
									axis6_coupled_target_raw_abs =
										axis6_abs - (axis1_return_target_abs - axis1_abs);
									candidate_axis6_coupled_target_abs = clamp_double(
										axis6_coupled_target_raw_abs,
										axis6_catheter_window_start_abs,
										axis6_catheter_window_end_abs);
									if (axis6_target_exceeds_soft_limit(candidate_axis6_coupled_target_abs))
									{
										engage_axis6_soft_limit_hold(
											candidate_axis6_coupled_target_abs,
											"axis1 回退时 axis6 联动目标");
										hold_axis6_related_axes();
										axis6_coupled_target_safe = false;
									}
								}

								if (axis6_coupled_target_safe)
								{
								if (cooperative_mode)
								{
									cooperative_return_owner = CooperativeReturnOwner::Axis1;
									axis7_hold_rel = plc_act_pos[6];
									axis6_coop_ff_inited = false;
								}
								axis1_crawl.target_abs = axis1_return_target_abs;
								axis1_return_entry_rel = plc_act_pos[0];
								axis1_return_settle_rel = plc_act_pos[0];
								axis1_fast_entry_abs = axis1_abs;
								axis6_fast_entry_abs = axis6_abs;
								axis1_return_hold_axis3_rel = plc_act_pos[2];
								axis1_return_hold_axis5_rel = plc_act_pos[4];
								axis6_coupled_settle_rel = plc_act_pos[5];
								if (!cooperative_mode)
								{
									// 导管模式：axis6 与 axis1 快退位移镜像反向，目标受轴5相对窗口限制。
									axis6_coupled_target_abs = candidate_axis6_coupled_target_abs;
									if (std::abs(axis6_coupled_target_abs - axis6_coupled_target_raw_abs) >
										cfg.crawl_arrive_tol_mm)
									{
										std::cout
											<< "轴6 导管联动目标已限制在 axis5 相对窗口内：["
											<< (axis6_catheter_window_start_abs - plc_leftlimit[5])
											<< ", "
											<< (axis6_catheter_window_end_abs - plc_leftlimit[5])
											<< "] mm。"
											<< std::endl;
									}
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
						// PLC 确认接收联动请求前保持入口位置，避免 ADS 失败时由 refer 直接驱动轴6。
						axis6_fast_retract = axis6_coupled_requested;
						pos[5] = axis6_coupled_requested
							? (axis6_coupled_target_abs - plc_init_pos[5])
							: axis6_return_entry_rel;
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
							clear_axis_return_request(AdsSymbol::axis6_return);
							load_pos_from_actual();
							axis1_fast_return = false;
							axis6_fast_retract = false;
							axis1_crawl.phase = CrawlState::Phase::Follow;
							axis1_crawl.plc_move_requested = false;
							axis1_follow_cmd_abs = plc_act_pos[0] + plc_init_pos[0];
							axis6_coupled_active = false;
							axis6_coupled_requested = false;
							axis6_coupled_done = false;
							axis6_coupled_error = false;
							axis6_coupled_error_id = 0;
							if (cooperative_mode && cooperative_return_owner == CooperativeReturnOwner::Axis1)
							{
								cancel_cooperative_delivery(true);
								clear_force_output();
							}
							return_ads_fault_hold = true;
							control_active = false;
							std::cout << "轴1 计划回退 ADS 下发失败，已保持当前位置并停止上位机运动控制。" << std::endl;
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
							axis6_fast_retract = true;
							pos[5] = axis6_coupled_target_abs - plc_init_pos[5];
						}
						else
						{
							clear_axis_return_request(AdsSymbol::axis6_return);
							pos[5] = plc_act_pos[5];
							axis6_fast_retract = false;
							axis6_coupled_active = false;
							axis6_coupled_requested = false;
							axis6_coupled_done = false;
							axis6_coupled_error = false;
							axis6_coupled_error_id = 0;
							return_ads_fault_hold = true;
							control_active = false;
							std::cout << "轴6 联动回退 ADS 下发失败，已保持轴6当前位置并停止上位机运动控制。" << std::endl;
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
							const bool cooperative_axis1_return =
								cooperative_mode && cooperative_return_owner == CooperativeReturnOwner::Axis1;
							const bool synced = cooperative_axis1_return
								? sync_cooperative_guidewire(3, false)
								: sync_axis1(3);
							if (!synced)
							{
								std::cout << "轴1 计划回退报错后重同步失败。" << std::endl;
								if (cooperative_axis1_return)
								{
									cancel_cooperative_delivery(true);
									return_ads_fault_hold = true;
									control_active = false;
									clear_force_output();
									std::cout << "协同模式已因重同步失败停止上位机运动控制。" << std::endl;
								}
							}
							else if (cooperative_axis1_return)
							{
								cooperative_return_owner = CooperativeReturnOwner::None;
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
									if (!sync_axis1(3))
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
					if (axis1_crawl.phase == CrawlState::Phase::FastMove &&
						axis1_fast_at_target && (!axis6_coupled_active || axis6_coupled_done || axis6_coupled_error))
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
						const bool cooperative_axis1_return =
							cooperative_mode && cooperative_return_owner == CooperativeReturnOwner::Axis1;
						const bool synced = cooperative_axis1_return
							? sync_cooperative_guidewire(3, false)
							: sync_axis1(3);
						if (!synced)
						{
							std::cout << "轴1 计划回退后重同步失败。" << std::endl;
							if (cooperative_axis1_return)
							{
								cancel_cooperative_delivery(true);
								return_ads_fault_hold = true;
								control_active = false;
								clear_force_output();
								std::cout << "协同模式已因重同步失败停止上位机运动控制。" << std::endl;
							}
						}
						else if (cooperative_axis1_return)
						{
							cooperative_return_owner = CooperativeReturnOwner::None;
						}
						if (synced && !cooperative_axis1_return &&
							guidewire_mode == GuidewireMode::None &&
							!axis1_reverse_pressed &&
							std::abs(cfg.axis1_post_return_lead_mm) > 1e-6)
						{
							axis1_post_return_lead_armed = true;
							axis1_post_return_lead_active = false;
							axis1_post_return_lead_target_abs = 0.0;
							std::cout << "轴1 计划回退完成：已武装回退后先行 "
								<< cfg.axis1_post_return_lead_mm << " mm。" << std::endl;
						}
						else
						{
							reset_axis1_post_return_lead();
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
					if (cooperative_return_owner == CooperativeReturnOwner::Axis1)
					{
						// 导管回退期间导丝链路不运行状态机，也不接受线性/旋转手柄输入。
						// 保持导丝侧正常 Follow 夹爪状态，等待导管侧恢复后统一重同步。
						axis6_coop_ff_inited = false;
						pos[5] = plc_act_pos[5];
						pos[6] = axis7_hold_rel;
						cylinder3_cmd = cyl.cyl3_open;
						cylinder4_cmd = cyl.cyl4_clamp;
					}
					else
					{
						const bool cooperative_follow_active =
							cooperative_return_owner == CooperativeReturnOwner::None &&
							axis1_crawl.phase == CrawlState::Phase::Follow &&
							axis6_crawl.phase == CrawlState::Phase::Follow;
						if (cooperative_follow_active)
						{
							// 两种协同方向的窗口都随当期 axis5 命令位置移动，而不是沿用入模时的固定窗口。
							const double axis5_cmd_abs = pos[4] + plc_init_pos[4];
							double axis6_window_left_abs = 0.0;
							double axis6_window_right_abs = 0.0;
							motion_sync::calculate_axis6_window_from_axis5_abs(
								ctx,
								axis5_cmd_abs,
								axis6_window_left_abs,
								axis6_window_right_abs);
							axis6_crawl.start_abs = axis6_window_left_abs;
							axis6_crawl.end_abs = axis6_window_right_abs;
							axis6_crawl.window_active = is_within_range(
								axis6_abs,
								axis6_crawl.min_abs(),
								axis6_crawl.max_abs(),
								cfg.crawl_arrive_tol_mm);
						}

						if (cooperative_follow_active)
						{
							// axis6 目标 = 582 导管链路已接受的命令增量 + 587 本人的命令增量。
							// 两项均保留原坐标符号，因此递送与撤出都能维持 axis6 相对 axis5 的间距。
							const double axis1_cmd_abs_for_ff = pos[0] + plc_init_pos[0];
							if (!axis6_coop_ff_inited)
							{
								axis6_coop_ff_inited = true;
								axis6_coop_prev_axis1_cmd_abs = axis1_cmd_abs_for_ff;
							}
							const double axis1_increment_mm =
								axis1_cmd_abs_for_ff - axis6_coop_prev_axis1_cmd_abs;
							const double axis6_combined_increment_mm = axis6_linear_increment_mm + axis1_increment_mm;
							axis6_coop_prev_axis1_cmd_abs = axis1_cmd_abs_for_ff;
							const double axis6_raw_cmd_abs = axis6_follow_cmd_abs + axis6_combined_increment_mm;

							run_axis6_crawl_state(
								axis6_raw_cmd_abs,
								axis6_combined_increment_mm,
								cooperative_retraction_active,
								axis6_linear_increment_active,
								true);
						}
						else
						{
							// 导丝拥有回退时仅推进其自身状态机；导管侧输入已在上方冻结。
							axis6_coop_ff_inited = false;
							run_axis6_crawl_state(
								axis6_follow_cmd_abs,
								0.0,
								cooperative_retraction_active,
								false,
								true);
						}
					}
				}
			}

			const bool tracking_return_started_this_cycle =
				axis1_crawl.phase != CrawlState::Phase::Follow ||
				axis6_crawl.phase != CrawlState::Phase::Follow ||
				axis1_crawl.plc_move_requested ||
				axis6_crawl.plc_move_requested ||
				axis6_coupled_active;
			if (tracking_return_started_this_cycle)
			{
				// 本拍触发 SwitchWait/回退时立即结束实验段并冻结 PI；
				// 换手前向欠账保留，待恢复正向 Follow 后继续由受限映射逐步补偿。
				tracking_controller.invalidate_all(TrackingInvalidReason::CrawlReturnActive);
			}
			else
			{
				// 以状态机构最终生成的命令复核 150 ms 夹持假设。
				// 这里再次调用不会重复累计实际位移，因为本拍实际位置尚未改变。
				const bool axis1_final_grip_command =
					cylinder2_cmd == cyl.cyl2_clamp && cylinder1_cmd == cyl.cyl1_open;
				const bool axis6_final_grip_command =
					cylinder4_cmd == cyl.cyl4_clamp && cylinder3_cmd == cyl.cyl3_open;
				tracking_controller.update_gate(
					DeliveryTrackingAxis::Axis1,
					axis1_tracking_reason == TrackingInvalidReason::None,
					axis1_final_grip_command,
					now_ms,
					axis1_abs,
					axis1_tracking_reason);
				tracking_controller.update_gate(
					DeliveryTrackingAxis::Axis6,
					axis6_tracking_reason == TrackingInvalidReason::None,
					axis6_final_grip_command,
					now_ms,
					axis6_abs,
					axis6_tracking_reason);
			}
			tracking_controller.finish_cycle(DeliveryTrackingAxis::Axis1, pos[0], plc_act_pos[0]);
			tracking_controller.finish_cycle(DeliveryTrackingAxis::Axis6, pos[5], plc_act_pos[5]);

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
		else
		{
			TrackingInvalidReason inactive_reason = TrackingInvalidReason::NotForwardDelivery;
			if (!tracking_logger.is_running()) inactive_reason = TrackingInvalidReason::NotLogging;
			else if (return_ads_fault_hold) inactive_reason = TrackingInvalidReason::AdsReturnFault;
			else if (motion_startup_active) inactive_reason = TrackingInvalidReason::StartupActive;
			else if (freeze_active) inactive_reason = TrackingInvalidReason::Paused;
			else if (estop_hold_active) inactive_reason = TrackingInvalidReason::PlcHold;
			else if (spacing_recovery.active() || spacing_recovery.requested) inactive_reason = TrackingInvalidReason::SpacingRecovery;
			else if (ft_exp.active()) inactive_reason = TrackingInvalidReason::ForceTransitionExperiment;
			else if (!control_active) inactive_reason = TrackingInvalidReason::ControlInactive;
			tracking_controller.invalidate_all(inactive_reason);
			if (tracking_controller.compensation_enabled())
			{
				tracking_controller.disable_compensation();
				std::cout << "主从位移补偿已关闭：运动控制当前不可用。" << std::endl;
			}
		}

		const DWORD spacing_recovery_exit_elapsed_ms =
			GetTickCount() - spacing_recovery.phase_t0;
		const bool spacing_recovery_targets_settled =
			std::abs(plc_act_pos[2] - spacing_recovery.axis3_cmd_rel) <= cfg.crawl_arrive_tol_mm &&
			std::abs(plc_act_pos[4] - spacing_recovery.axis5_cmd_rel) <= cfg.crawl_arrive_tol_mm &&
			std::abs(plc_act_pos[5] - spacing_recovery.axis6_cmd_rel) <= cfg.crawl_arrive_tol_mm;
		const bool spacing_recovery_exit_ready =
			spacing_recovery_exit_elapsed_ms >= cfg.spacing_recovery_exit_settle_ms &&
			(spacing_recovery_targets_settled ||
				spacing_recovery_exit_elapsed_ms >= cfg.spacing_recovery_exit_timeout_ms);
		if (spacing_recovery.phase == SpacingRecoveryPhase::ExitSync && spacing_recovery_exit_ready &&
			!freeze_active && !estop_hold_active && !return_ads_fault_hold)
		{
			if (!spacing_recovery_targets_settled)
			{
				std::cout << "屈曲恢复退出等待超时，将按轴3/5/6当前实际位置重同步。" << std::endl;
			}
			const double recovered_mm = spacing_recovery.moved_mm;
			if (sync_all(20))
			{
				const double recovered_axis3_from_left_mm =
					(plc_act_pos[2] + plc_init_pos[2]) - plc_leftlimit[2];
				if (recovered_axis3_from_left_mm >
					(cfg.axis3_delivery_stop_from_left_mm + cfg.axis3_delivery_release_hysteresis_mm))
				{
					axis1_delivery_stop_latched = false;
					axis1_delivery_stop_prompted = false;
				}
				axis1_push_rearm_after_hold = false;
				spacing_recovery.reset();
				control_active = true;
				std::cout << "屈曲恢复已退出并完成重同步，本次共同移动 "
					<< recovered_mm << " mm。" << std::endl;
			}
			else
			{
				spacing_recovery.reset();
				return_ads_fault_hold = true;
				control_active = false;
				clear_force_output();
				std::cout << "屈曲恢复退出重同步失败，已停止上位机运动控制。" << std::endl;
			}
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
		bool cylinder5_req = formal_control_stage ? pause_pressed : false;
		if (!freeze_active && (control_active || motion_startup_active))
		{
			if (!spacing_recovery.active())
			{
				auto apply_cylinder_manual_mode = [](CylinderManualMode mode, unsigned short& command,
					unsigned short open_value, unsigned short closed_value)
				{
					if (mode == CylinderManualMode::Open) command = open_value;
					else if (mode == CylinderManualMode::Closed) command = closed_value;
				};
				apply_cylinder_manual_mode(cylinder_manual_mode[0], cylinder1_cmd, cyl.cyl1_open, cyl.cyl1_clamp);
				apply_cylinder_manual_mode(cylinder_manual_mode[1], cylinder2_cmd, cyl.cyl2_open, cyl.cyl2_clamp);
				apply_cylinder_manual_mode(cylinder_manual_mode[2], cylinder3_cmd, cyl.cyl3_open, cyl.cyl3_clamp);
				apply_cylinder_manual_mode(cylinder_manual_mode[3], cylinder4_cmd, cyl.cyl4_open, cyl.cyl4_clamp);
			}
			ads.ADSWrite(AdsSymbol::cylinder1_value, sizeof(cylinder1_cmd), &cylinder1_cmd);
			ads.ADSWrite(AdsSymbol::cylinder2_value, sizeof(cylinder2_cmd), &cylinder2_cmd);
			ads.ADSWrite(AdsSymbol::cylinder3_value, sizeof(cylinder3_cmd), &cylinder3_cmd);
			ads.ADSWrite(AdsSymbol::cylinder4_value, sizeof(cylinder4_cmd), &cylinder4_cmd);
			// 电缸5写 BOOL 请求，由 PLC handle 周期映射为 0/2000。
			ads.ADSWrite(AdsSymbol::cylinder5_press_req, sizeof(cylinder5_req), &cylinder5_req);
		}

		write_axis4_manual_requests(axis4_manual_forward_req, axis4_manual_reverse_req);
		ads.ADSWrite(AdsSymbol::startup_smoothing_bypass, sizeof(startup_smoothing_bypass), &startup_smoothing_bypass);
		ads.ADSWrite(AdsSymbol::axis1_fast_return, sizeof(axis1_fast_return), &axis1_fast_return); // 轴1快退平滑旁路
		ads.ADSWrite(AdsSymbol::axis6_fast_retract, sizeof(axis6_fast_retract), &axis6_fast_retract); // 轴6快退平滑旁路

		// 力传感器采样独立于旧 CSV 开关，避免 force_log.enabled=false 阻断力反馈。
		const DWORD force_log_now_ms = GetTickCount();
		const bool tracking_log_active = tracking_logger.is_running();
		const bool force_sampling_active = force_log_started || ff.enabled || tracking_log_active;
		// 仅为主从实验记录提供力样本时保持 20 Hz；旧高频记录或力反馈仍沿用原采样节拍。
		const DWORD force_sampling_period_ms = (force_log_started || ff.enabled)
			? force_log.period_ms
			: 50;
		const bool should_sample_force =
			force_sampling_active &&
			((force_sampling_period_ms == 0) ||
				(force_sample_last_sample_ms == 0) ||
				((force_log_now_ms - force_sample_last_sample_ms) >= force_sampling_period_ms));
		if (!force_sampling_active)
		{
			force_sample.valid = false;
		}
		if (should_sample_force)
		{
			// 采样节拍按 period_ms 统一推进，读失败也不打乱节拍。
			force_sample_last_sample_ms = force_log_now_ms;
			ForceSampleFrame sampled_frame;
			const bool ads_sample_ok = read_force_sample(sampled_frame);
			bool ready_to_log = false;
			double log_ft1_value = 0.0;
			double log_fn1_value = 0.0;

			if (ctx.force_sample_source == ForceSampleSource::TCP_DAQ)
			{
				double tcp_raw_v[6] = { 0 };
				std::uint64_t tcp_ts = 0;
				if (tcp_force_daq.get_latest_raw(tcp_raw_v, tcp_ts))
				{
					force_sample = ads_sample_ok ? sampled_frame : ForceSampleFrame{};
					if (!ads_sample_ok)
					{
						force_sample.axis1_pos_rel = plc_act_pos[0];
						force_sample.tick_ms = force_log_now_ms;
					}
					force_sample.axis2_pos_rel = plc_act_pos[1];
					force_sample.fn_1_value_v = tcp_raw_v[0];
					force_sample.ft_1_value_v = tcp_raw_v[1];
					force_sample.valid = true;
					log_ft1_value = force_sample.ft_1_value_v;
					log_fn1_value = force_sample.fn_1_value_v;
					ready_to_log = true;
					force_sample_diag_reason.clear();
				}
				else
				{
					force_sample.valid = false;
					const std::string reason = "力传感器告警：当前采样源为 TCP_DAQ，但尚无采集卡有效帧。";
					if (force_sample_diag_reason != reason)
					{
						std::cout << reason << std::endl;
					}
					force_sample_diag_reason = reason;
				}
			}
			else if (ads_sample_ok)
			{
				force_sample = sampled_frame;
				force_sample.axis2_pos_rel = plc_act_pos[1];
				log_ft1_value = force_sample.ft_1_value_v;
				log_fn1_value = force_sample.fn_1_value_v;
				ready_to_log = true;
				// ADS 高速日志没有独立采样线程；将统一单位后的本拍样本送入旧写入器。
				if (force_logger.is_running())
				{
					double ads_v[6] = { force_sample.fn_1_value_v, force_sample.ft_1_value_v, 0.0, 0.0, 0.0, 0.0 };
					force_logger.on_sensor_sample(force_sample.tick_ms, ads_v);
				}
				force_sample_diag_reason.clear();
			}
			else
			{
				force_sample = ForceSampleFrame{};
				force_sample.tick_ms = force_log_now_ms;
				const std::string reason = "力传感器告警：ft_1/fn_1/fn_2/ft_2 与 axis1_pos_rel 的 ADSReadSum 读取失败。";
				if (force_sample_diag_reason != reason)
				{
					std::cout << reason << std::endl;
				}
				force_sample_diag_reason = reason;
			}

			if (force_log.enabled && ready_to_log)
			{
				force_log.last_sample_ms = force_log_now_ms;
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

		// 主从实验不依赖力反馈开关，但 ADS 力输入失效时关闭补偿，禁止在无健康采样的状态继续放大手柄位移。
		if (tracking_logger.is_running() && !force_sample.valid && tracking_controller.compensation_enabled())
		{
			tracking_controller.disable_compensation();
			std::cout << "主从位移补偿已关闭：当前没有有效力采样。" << std::endl;
		}
		if (tracking_logger.is_running() &&
			(tracking_log_last_sample_ms == 0 || (force_log_now_ms - tracking_log_last_sample_ms) >= 50))
		{
			tracking_log_last_sample_ms = force_log_now_ms;
			DeliveryTrackingLogger::Row tracking_row{};
			tracking_row.tick_ms = force_log_now_ms;
			tracking_row.session_id = tracking_logger.session_id();
			tracking_row.guidewire_mode = static_cast<int>(guidewire_mode);
			tracking_row.axis1_phase = static_cast<int>(axis1_crawl.phase);
			tracking_row.axis6_phase = static_cast<int>(axis6_crawl.phase);
			tracking_row.cylinder_cmd[0] = cylinder1_cmd;
			tracking_row.cylinder_cmd[1] = cylinder2_cmd;
			tracking_row.cylinder_cmd[2] = cylinder3_cmd;
			tracking_row.cylinder_cmd[3] = cylinder4_cmd;
			tracking_row.compensation_enabled = tracking_controller.compensation_enabled();
			tracking_row.force_sample_valid = force_sample.valid;
			tracking_row.calibration_zeroed = cal_state.zeroed;
			tracking_row.freeze_active = freeze_active;
			tracking_row.plc_hold_active = estop_hold_active;
			tracking_row.ads_return_fault = return_ads_fault_hold;
			tracking_row.spacing_recovery_active = spacing_recovery.active() || spacing_recovery.requested;
			tracking_row.force_transition_active = ft_exp.active();
			tracking_row.manual_cylinder_override = tracking_manual_cylinder_override;
			tracking_row.fn_1_raw_v = force_sample.fn_1_value_v;
			tracking_row.ft_1_raw_v = force_sample.ft_1_value_v;
			tracking_row.fn_1_zero_v = cal_state.f_zero;
			tracking_row.ft_1_zero_v = cal_state.ft_zero;
			if (force_sample.valid && cal_state.zeroed)
			{
				const CalibratedForce tracking_force = calibrate_force(
					force_sample.fn_1_value_v,
					force_sample.ft_1_value_v,
					force_sample.axis2_pos_rel,
					cal_cfg,
					cal_state);
				tracking_row.calibrated_force_n = tracking_force.f_feedback_n;
				tracking_row.calibrated_torque_nm = tracking_force.t_feedback_nm;
			}
			tracking_row.logger_dropped = tracking_logger.dropped_count();
			tracking_row.axis1 = tracking_controller.snapshot(DeliveryTrackingAxis::Axis1);
			tracking_row.axis6 = tracking_controller.snapshot(DeliveryTrackingAxis::Axis6);
			tracking_logger.enqueue(tracking_row);
		}

		// 力反馈阻断原因仅在状态变化时输出一次，避免控制台被周期性等待提示淹没。
		std::string current_force_feedback_reason;
		if (ff.enabled)
		{
			if (!cal_state.zeroed) current_force_feedback_reason = "力反馈等待：尚未完成力传感器零点采集。";
			else if (!force_sample.valid) current_force_feedback_reason = "力反馈等待：当前没有有效力采样。";
			else if (!control_active) current_force_feedback_reason = "力反馈等待：控制尚未激活。";
			else if (freeze_active) current_force_feedback_reason = "力反馈等待：582 暂停处于开启状态。";
			else if (estop_hold_active) current_force_feedback_reason = "力反馈等待：PLC 保持处于开启状态。";
			else if (spacing_recovery.active()) current_force_feedback_reason = "力反馈等待：当前为屈曲恢复模式。";
			else if (guidewire_mode != GuidewireMode::None) current_force_feedback_reason = "力反馈等待：当前为导丝模式，582 导管力反馈输出被置零。";
		}
		if (!current_force_feedback_reason.empty() && current_force_feedback_reason != force_feedback_diag_reason)
		{
			std::cout << current_force_feedback_reason << std::endl;
		}
		force_feedback_diag_reason = current_force_feedback_reason;

		// 力过渡决定性预实验：每拍 tick；激活时接管 axis1 refer 与可选的 v_limit / axis1_fast_return。
		// 调用必须在 process_force_feedback 之前，让 axis1_fast_return 边沿能进入既有冻结链路。
		const bool ft_exp_was_active = ft_exp.active();
		if (ft_exp.active())
		{
			const std::uint32_t now_tick_ms = GetTickCount();
			const bool exp_taking_over = ft_exp.tick(
				ctx,
				now_tick_ms,
				control_active,
				freeze_active,
				estop_hold_active,
				cal_state.zeroed,
				guidewire_mode);
			if (exp_taking_over)
			{
				pos[0] = ft_exp.current_axis1_refer();
				if (ft_exp.axis1_fast_return_request())
				{
					axis1_fast_return = true;
				}
				if (ft_exp.wants_v_limit_override())
				{
					// ft_exp 内部维护 start 时的 v_limit 快照，主循环不需要每拍读 PLC。
					double v_limit_local[7];
					ft_exp.fill_v_limit_override(v_limit_local);
					plc_io::write_v_limit(ctx, v_limit_local);
				}
			}
		}
		// 实验由 active → 非 active（Done/Abort）的边沿：关闭专用 CSV 会话，flush 落盘。
		if (ft_exp_was_active && !ft_exp.active() && ft_logger.is_running())
		{
			ft_logger.stop_session();
			std::cout << (ft_exp.aborted() ? "力过渡实验已异常终止，CSV 已落盘。"
				: "力过渡实验已完成，CSV 已落盘。") << std::endl;
		}

		process_force_feedback(
			ff,
			force_sample,
			*catheter_force_output_handle,
			*guidewire_force_output_handle,
			guidewire_mode,
			control_active && !spacing_recovery.active(),
			freeze_active,
			estop_hold_active,
			axis1_fast_return,
			axis6_fast_retract,
			loop_count,
			cfg,
			cal_cfg,
			cal_state);

		// 力过渡决定性预实验（论文 §6.1）：在 process_force_feedback 之后采样，
		// 把"实验当前接管阶段、实际 axis1 位置、标定后力、原始电压、相位上下文"打包入队。
		// 仅在实验 active 且 logger 运行时入队；其他情况由 ForceLogger 自行处理。
		if (ft_exp.active() && ft_logger.is_running())
		{
			ForceTransitionLogger::Row r{};
			r.trial_id = ft_exp.current_trial_id();
			r.velocity_level = ft_exp.current_velocity_level();
			r.repeat_in_level = ft_exp.current_repeat_in_level();
			r.phase_code = static_cast<int>(ft_exp.current_phase());
			r.tick_ms = GetTickCount();
			// GetTickCount 49 天回卷，但单次实验最长几十分钟，回卷期间差值仍正确（DWORD 自然回绕）。
			r.dt_ms_from_phase_start = static_cast<std::uint64_t>(
				static_cast<std::uint32_t>(r.tick_ms) - ft_exp.current_phase_t0_ms());
			r.axis1_act_pos_mm = plc_act_pos[0];
			r.axis1_refer_mm = pos[0];
			r.axis1_v_limit_used = plc_v_limit[0];
			r.fn_1_raw_v = force_sample.fn_1_value_v;
			r.ft_1_raw_v = force_sample.ft_1_value_v;
			r.fn_1_zero_v = cal_state.f_zero;
			r.ft_1_zero_v = cal_state.ft_zero;
			r.f_feedback_n = ff.force_582_theory_f;
			r.t_feedback_nm = ff.force_582_theory_n;
			r.ff_enabled = ff.enabled;
			r.cal_zeroed = cal_state.zeroed;
			r.freeze_active = freeze_active;
			r.fast_active = axis1_fast_return;
			r.guidewire_mode = static_cast<int>(guidewire_mode);
			r.axis1_reverse = axis1_reverse_pressed;
			r.cyl1_cmd = cylinder1_cmd;
			r.cyl2_cmd = cylinder2_cmd;
			ft_logger.enqueue(r);
		}


		const DWORD force_feedback_value_log_now_ms = GetTickCount();
		if (ff.enabled &&
			cal_state.zeroed &&
			force_sample.valid &&
			control_active &&
			!freeze_active &&
			!estop_hold_active &&
			guidewire_mode == GuidewireMode::None &&
			(force_feedback_value_log_last_ms == 0 ||
				(force_feedback_value_log_now_ms - force_feedback_value_log_last_ms) >= 1000))
		{
			const double dft = force_sample.ft_1_value_v - cal_state.ft_zero;
			const double df = force_sample.fn_1_value_v - cal_state.f_zero;
			std::cout << "[FF] AI0/f=" << force_sample.fn_1_value_v
				<< " V, AI1/ft=" << force_sample.ft_1_value_v
				<< " V, dft=" << dft
				<< " V, df=" << df
				<< " V -> 导管力反馈(SN " << catheter_force_output_handle->serial()
				<< ").setforce_axis(F=" << ff.force_582_f
				<< " N, axis=" << cfg.axial_force_axis
				<< ", N=" << ff.force_582_n
				<< " N*m), theory(F=" << ff.force_582_theory_f
				<< " N, N=" << ff.force_582_theory_n
				<< " N*m), gravity=" << (cal_cfg.gravity_comp_enabled ? "ON" : "OFF")
				<< ", theta=" << force_sample.axis2_pos_rel
				<< " deg, theta0=" << cal_state.theta0_deg << " deg" << std::endl;
			force_feedback_value_log_last_ms = force_feedback_value_log_now_ms;
		}

		// 无论本拍是否进入控制分支，都更新线性差分基准，避免暂停/等待期间累积大跳变。
		axis1_prev_linear_filtered = axis1_handle_filter.axis0_filtered;
		axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;
		axis1_prev_rot_filtered = axis1_handle_filter.axis1_filtered;
		axis6_prev_rot_filtered = axis6_handle_filter.axis1_filtered;

		// 可视化管道：推送状态快照 + 轮询 UI 命令。
		{
			VisState vs{};
			for (int i = 0; i < 7; ++i)
			{
				vs.axis_pos[i] = plc_act_pos[i];
				vs.axis_pos_from_left[i] = plc_act_pos_from_left[i];
			}
			vs.cylinder_cmd[0] = cylinder1_cmd;
			vs.cylinder_cmd[1] = cylinder2_cmd;
			vs.cylinder_cmd[2] = cylinder3_cmd;
			vs.cylinder_cmd[3] = cylinder4_cmd;
			vs.guidewire_mode = static_cast<int>(guidewire_mode);
			vs.axis1_phase = static_cast<int>(axis1_crawl.phase);
			vs.axis6_phase = static_cast<int>(axis6_crawl.phase);
			vs.startup_phase = static_cast<int>(startup.phase);
			vs.control_active = control_active;
			vs.freeze_active = freeze_active;
			vs.estop_hold = estop_hold_active;
			vs.axis1_fast_return = axis1_fast_return;
			vs.axis6_fast_retract = axis6_fast_retract;
			vs.self_check_done = self_check_done;
			vs.ff_enabled = ff.enabled;
			vs.cal_zeroed = cal_state.zeroed;
			vs.axis1_reverse = axis1_reverse_pressed;
			vs.axis6_reverse = axis6_effective_reverse_pressed;
			vs.force_log_running = force_logger.is_running();
			vs.startup_waiting = (!startup.completed && startup.phase == StartupPhase::WaitForEnter);
			vs.startup_completed = startup.completed;
			vs.ft_1_v = force_sample.ft_1_value_v;
			vs.fn_1_v = force_sample.fn_1_value_v;
			vs.force_582_f = ff.force_582_f;
			vs.force_582_n = ff.force_582_n;
			vs.force_587_f = ff.force_587_f;
			vs.force_587_n = ff.force_587_n;
			vs.loop_count = loop_count;
			vs.tick_ms = GetTickCount();
			vs.force_582_theory_f = ff.force_582_theory_f;
			vs.force_582_theory_n = ff.force_582_theory_n;
			vs.gravity_comp_enabled = cal_cfg.gravity_comp_enabled;
			vs.ft_exp_phase = static_cast<int>(ft_exp.current_phase());
			vs.ft_exp_velocity_level = ft_exp.current_velocity_level();
			vs.ft_exp_trial_id = ft_exp.current_trial_id();
			vs.ft_exp_repeat_in_lvl = ft_exp.current_repeat_in_level();
			vs.ft_exp_v_ratio_curr = ft_exp.current_v_ratio();
			vs.ft_exp_axis1_target = ft_exp.current_axis1_target();
			vs.ft_exp_active = ft_exp.active();
			vs.ft_exp_aborted = ft_exp.aborted();
			vs.spacing_recovery_phase = static_cast<int>(spacing_recovery.phase);
			vs.spacing_recovery_moved_mm = spacing_recovery.moved_mm;
			vs.spacing_recovery_remaining_mm = spacing_recovery.remaining_mm;
			vs.dual_handle_ready = dual_handle_ready;
			vs.cooperative_return_owner = static_cast<int>(cooperative_return_owner);
			const DeliveryTrackingAxisSnapshot axis1_tracking_snapshot =
				tracking_controller.snapshot(DeliveryTrackingAxis::Axis1);
			const DeliveryTrackingAxisSnapshot axis6_tracking_snapshot =
				tracking_controller.snapshot(DeliveryTrackingAxis::Axis6);
			vs.tracking_log_running = tracking_logger.is_running();
			vs.tracking_compensation_enabled = tracking_controller.compensation_enabled();
			vs.axis1_tracking_error_mm = axis1_tracking_snapshot.tracking_error_mm;
			vs.axis6_tracking_error_mm = axis6_tracking_snapshot.tracking_error_mm;
			vs.axis1_compensation_gain = axis1_tracking_snapshot.compensation_gain;
			vs.axis6_compensation_gain = axis6_tracking_snapshot.compensation_gain;
			vs.tracking_log_dropped = tracking_logger.dropped_count();
			vs.cooperative_direction = static_cast<int>(cooperative_direction);
			vs.axis6_soft_limit_hold = axis6_soft_limit_hold;
			vis_server.push_state(vs);
		}

		{
			VisCommand vcmd;
			while (vis_server.poll_command(vcmd))
			{
				switch (vcmd.type)
				{
				case VisCommandType::SetCylinderManualOpen:
					if (!spacing_recovery.active() && !spacing_recovery.requested &&
						vcmd.param1 >= 0 && vcmd.param1 < 4)
						cylinder_manual_mode[vcmd.param1] = CylinderManualMode::Open;
					break;
				case VisCommandType::SetCylinderManualClosed:
					if (!spacing_recovery.active() && !spacing_recovery.requested &&
						vcmd.param1 >= 0 && vcmd.param1 < 4)
						cylinder_manual_mode[vcmd.param1] = CylinderManualMode::Closed;
					break;
				case VisCommandType::RequestModeSwitch:
					if (single_handle_mode)
						single_handle_requested_mode = static_cast<GuidewireMode>(vcmd.param1);
					break;
				case VisCommandType::ZeroForceSensor:
				{
					zero_force_sensor("UI：");
					break;
				}
				case VisCommandType::ToggleForceFeedback:
					ff.enabled = !ff.enabled;
					ff.reset();
					std::cout << "UI：力反馈：" << (ff.enabled ? "开启" : "关闭") << std::endl;
					if (!ff.enabled) clear_force_output();
					break;
				case VisCommandType::SetReverseMode:
				{
					if (cooperative_return_owner != CooperativeReturnOwner::None)
					{
						std::cout << "UI：模式切换已忽略，协同计划回退尚未完成。" << std::endl;
						break;
					}
					if (spacing_recovery.active() || spacing_recovery.requested)
					{
						std::cout << "UI：模式切换已忽略，请先退出屈曲恢复。" << std::endl;
						break;
					}
					// param1: 0=catheter mode+direction, 1=guidewire mode+direction
					// param2: 0=forward(递送), 1=reverse(撤出)
					GuidewireMode target_mode = (vcmd.param1 == 0) ? GuidewireMode::None : GuidewireMode::Independent;
					if (single_handle_mode)
						single_handle_requested_mode = target_mode;
					// 任何普通模式选择均明确退出协同方向请求。
					cooperative_direction_requested = CooperativeDirection::None;
					vis_reverse_override_active = true;
					vis_reverse_override_target = vcmd.param1;
					vis_reverse_override_value = (vcmd.param2 != 0);
					break;
				}
				case VisCommandType::ToggleForceLog:
					if (force_logger.is_running())
					{
						tcp_force_daq.set_on_sample(nullptr);
						force_logger.stop();
						std::cout << "高频力数据记录已停止。" << std::endl;
					}
					else
					{
						if (force_logger.start("."))
						{
							force_logger.publish_force_zero(cal_state.f_zero, cal_state.ft_zero);
							if (ctx.force_sample_source == ForceSampleSource::TCP_DAQ)
							{
								tcp_force_daq.set_on_sample([&](std::uint64_t tick_ms, const double v[6]) {
									force_logger.on_sensor_sample(tick_ms, v);
								});
							}
							else
							{
								tcp_force_daq.set_on_sample(nullptr);
							}
							std::cout << "高频力数据记录已启动。" << std::endl;
						}
					}
					break;
				case VisCommandType::SetTrackingLog:
					// 记录会话由进程启动和退出统一管理，避免换手研究数据被 UI 误停。
					std::cout << "主从位移实验记录已由程序自动管理，忽略手动开始/停止请求。" << std::endl;
					break;
				case VisCommandType::SetTrackingCompensation:
					if (vcmd.param1 == 0)
					{
						if (tracking_controller.compensation_enabled())
						{
							tracking_controller.disable_compensation();
							std::cout << "主从位移补偿已关闭。" << std::endl;
						}
						break;
					}
					{
						DeliveryTrackingAxis tracking_axis = DeliveryTrackingAxis::Axis1;
						bool forward_tracking_mode = false;
						if (guidewire_mode == GuidewireMode::None && !axis1_reverse_pressed)
						{
							tracking_axis = DeliveryTrackingAxis::Axis1;
							forward_tracking_mode = true;
						}
						else if (guidewire_mode == GuidewireMode::Independent && !axis6_effective_reverse_pressed)
						{
							tracking_axis = DeliveryTrackingAxis::Axis6;
							forward_tracking_mode = true;
						}
						if (!tracking_logger.is_running())
						{
							std::cout << "主从位移补偿开启被拒绝：自动记录会话未运行，请检查 CSV 文件创建权限。" << std::endl;
						}
						else if (!forward_tracking_mode || !force_sample.valid)
						{
							std::cout << "主从位移补偿开启被拒绝：需处于正向 Follow 递送段且 ADS 力采样有效。" << std::endl;
						}
						else if (!tracking_controller.set_compensation_enabled(true, tracking_axis))
						{
							std::cout << "主从位移补偿开启被拒绝：夹持假设尚未稳定或 PI 参数未满足安全范围。" << std::endl;
						}
						else
						{
							std::cout << "主从位移补偿已开启。" << std::endl;
						}
					}
					break;
				case VisCommandType::SetTrackingCompensationParam:
					if (vcmd.param1 < static_cast<int>(TrackingParameterField::Axis1Kp) ||
						vcmd.param1 > static_cast<int>(TrackingParameterField::Axis6MaxError) ||
						!tracking_controller.set_parameter(
							static_cast<TrackingParameterField>(vcmd.param1),
							static_cast<double>(vcmd.param2) / 1000.0))
					{
						std::cout << "主从位移 PI 参数已忽略：补偿开启时不可修改，或数值超出安全范围。" << std::endl;
					}
					break;
				case VisCommandType::SetAxis1PostReturnLead:
				{
					const double lead_mm = static_cast<double>(vcmd.param1) / 1000.0;
					if (lead_mm < -cfg.axis1_post_return_lead_limit_mm ||
						lead_mm > cfg.axis1_post_return_lead_limit_mm)
					{
						std::cout << "UI：axis1 回退后先行量已忽略，范围为 [-10, 10] mm。" << std::endl;
					}
					else
					{
						cfg.axis1_post_return_lead_mm = lead_mm;
						std::cout << "UI：axis1 回退后先行量已更新为 "
							<< lead_mm << " mm（正值沿递送方向）。" << std::endl;
					}
					break;
				}
				case VisCommandType::SetStartupAxisPos:
				{
					double pos_mm = vcmd.param2 / 100.0;
					switch (vcmd.param1)
					{
					case 1: pending_startup.axis1_from_left_mm = pos_mm; break;
					case 3: pending_startup.axis3_from_left_mm = pos_mm; break;
					case 5: pending_startup.axis5_from_left_mm = pos_mm; break;
					case 6: pending_startup.axis6_from_left_mm = pos_mm; break;
					}
					break;
				}
				case VisCommandType::SetStartupAxisDeg:
				{
					double deg = vcmd.param2 / 100.0;
					if (vcmd.param1 == 2) pending_startup.axis2_deg = deg;
					else if (vcmd.param1 == 7) pending_startup.axis7_deg = deg;
					break;
				}
				case VisCommandType::SetStartupSpeed:
					pending_startup.speed_scale = vcmd.param1 / 100000.0;
					break;
				case VisCommandType::ExecuteStartup:
				{
					bool valid = true;
					if (pending_startup.axis1_from_left_mm < 5.0 || pending_startup.axis1_from_left_mm > 95.0) valid = false;
					if (pending_startup.axis3_from_left_mm < 10.0 || pending_startup.axis3_from_left_mm > 650.0) valid = false;
					if (pending_startup.axis5_from_left_mm < 10.0 || pending_startup.axis5_from_left_mm > 670.0) valid = false;
					if (pending_startup.axis6_from_left_mm < 10.0 || pending_startup.axis6_from_left_mm > 670.0) valid = false;
					if (pending_startup.axis6_from_left_mm < pending_startup.axis5_from_left_mm) valid = false;
					if (pending_startup.axis5_from_left_mm < pending_startup.axis3_from_left_mm) valid = false;
					if (pending_startup.axis3_from_left_mm < pending_startup.axis1_from_left_mm) valid = false;
					if (pending_startup.speed_scale < 0.00001 || pending_startup.speed_scale > 0.5) valid = false;
					if (valid && !startup.completed && startup.phase == StartupPhase::WaitForEnter &&
						!freeze_active && !estop_hold_active)
					{
						startup.final_axis1_from_left_mm = pending_startup.axis1_from_left_mm;
						startup.final_axis3_from_left_mm = pending_startup.axis3_from_left_mm;
						startup.final_axis5_from_left_mm = pending_startup.axis5_from_left_mm;
						startup.final_axis6_from_left_mm = pending_startup.axis6_from_left_mm;
						startup.final_axis2_deg = pending_startup.axis2_deg;
						startup.final_axis7_deg = pending_startup.axis7_deg;
						cfg.startup_motion_speed_scale = pending_startup.speed_scale;
						if (start_startup_sequence())
						{
							control_active = false;
							ensure_force_log_started();
							std::cout << "启动准备流程已开始（UI 参数）。" << std::endl;
						}
					}
					break;
				}
				case VisCommandType::SelectDirectControl:
				{
					if (!startup.completed &&
						startup.phase == StartupPhase::WaitForEnter &&
						!freeze_active &&
						!estop_hold_active &&
						(!has_self_check_flag || self_check_done))
					{
						if (restore_startup_v_limit() && sync_all(20))
						{
							startup.phase = StartupPhase::Done;
							startup.completed = true;
							startup.prompted = false;
							control_active = true;
							ensure_force_log_started();
							std::cout << "已进入直接控制（UI 触发）。" << std::endl;
						}
					}
					break;
				}
				case VisCommandType::SetGravityCompensation:
				{
					const bool enabled = (vcmd.param1 != 0);
					if (cal_cfg.gravity_comp_enabled != enabled)
					{
						cal_cfg.gravity_comp_enabled = enabled;
						std::cout << "UI：重力补偿：" << (enabled ? "开启" : "关闭")
							<< "，theta0=" << cal_state.theta0_deg << " deg" << std::endl;
					}
					break;
				}
				case VisCommandType::SetFtExpParamA:
					ft_exp.set_param_a(vcmd.param1, vcmd.param2);
					break;
				case VisCommandType::SetFtExpParamB:
					ft_exp.set_param_b(vcmd.param1, vcmd.param2);
					break;
				case VisCommandType::SetSpacingRecovery:
					if (vcmd.param1 != 0 && cooperative_return_owner != CooperativeReturnOwner::None)
					{
						std::cout << "UI：屈曲恢复已忽略，协同计划回退尚未完成。" << std::endl;
						break;
					}
					if (vcmd.param1 != 0)
					{
						cancel_cooperative_delivery(true);
					}
					spacing_recovery.requested = (vcmd.param1 != 0);
					break;
				case VisCommandType::SetCooperativeDelivery:
				case VisCommandType::SetCooperativeRetraction:
				{
					const CooperativeDirection requested_direction =
						vcmd.type == VisCommandType::SetCooperativeRetraction
						? CooperativeDirection::Retraction
						: CooperativeDirection::Delivery;
					const char* mode_name = cooperative_direction_text(requested_direction);
					if (vcmd.param1 != 0)
					{
						if (!dual_handle_ready)
						{
							std::cout << "UI：" << mode_name
								<< "已忽略，程序启动时未成功连接两只手柄。" << std::endl;
						}
						else if (spacing_recovery.active() || spacing_recovery.requested ||
							cooperative_return_owner != CooperativeReturnOwner::None)
						{
							std::cout << "UI：" << mode_name
								<< "已忽略，当前有恢复流程或协同换手在执行。" << std::endl;
						}
						else
						{
							// 入口被拒绝或方向切换失败时，主循环会以实际激活方向校正 WPF 选中项。
							cooperative_direction_requested = requested_direction;
						}
					}
					else if (cooperative_return_owner != CooperativeReturnOwner::None)
					{
						std::cout << "UI：" << mode_name
							<< "退出已忽略，请等待当前计划换手完成。" << std::endl;
					}
					else
					{
						cooperative_direction_requested = CooperativeDirection::None;
						// 退出协同后固定回到普通导管递送，避免手柄按键立即切到其他模式。
						vis_reverse_override_active = true;
						vis_reverse_override_target = 0;
						vis_reverse_override_value = false;
					}
					break;
				}
				case VisCommandType::StartForceTransitionExperiment:
				{
					if (!ft_exp.active())
					{
						const bool prerequisites_ok =
							control_active &&
							cal_state.zeroed &&
							!freeze_active &&
							!estop_hold_active &&
							!axis6_soft_limit_hold &&
							guidewire_mode == GuidewireMode::None &&
							!spacing_recovery.active() && !spacing_recovery.requested &&
							startup.completed;
						if (!prerequisites_ok)
						{
							std::cout << "UI：力过渡实验启动被拒绝：前置条件未满足（需 控制激活 + 已标零 + 非暂停 + 非急停 + 导管模式 + 启动完成）。" << std::endl;
						}
						else if (!ft_logger.is_running() && !ft_logger.start_session("."))
						{
							std::cout << "UI：力过渡实验启动被拒绝：CSV 文件创建失败。" << std::endl;
						}
						else if (!ft_exp.start(ctx, ft_exp.pending_cfg(), &ft_logger))
						{
							std::cout << "UI：力过渡实验启动被拒绝：" << ft_exp.last_error() << std::endl;
							ft_logger.stop_session();
						}
						else
						{
							cooperative_direction_requested = CooperativeDirection::None;
							std::cout << "UI：力过渡实验已启动。" << std::endl;
						}
					}
					break;
				}
				case VisCommandType::StopForceTransitionExperiment:
				{
					if (ft_exp.active())
					{
						ft_exp.abort(ctx, "UI stop");
						std::cout << "UI：力过渡实验已停止。" << std::endl;
					}
					if (ft_logger.is_running())
					{
						ft_logger.stop_session();
					}
					break;
				}
				default:
					break;
				}
			}
		}
	}

	startup_smoothing_bypass = false;
	ads.ADSWrite(AdsSymbol::startup_smoothing_bypass, sizeof(startup_smoothing_bypass), &startup_smoothing_bypass);
	clear_force_output();
	tracking_controller.stop_session();
	tracking_logger.stop_session();
	force_log.close();
	tcp_force_daq.stop();
	force_logger.stop();
	vis_server.stop();
	handle_axis1.close();
	handle_axis6.close();
	return 0;
}
