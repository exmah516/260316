#include "control_types.h"
#include "ads_communication.h"
#include "arm_manual_ads_service.h"
#include "delivery_tracking.h"
#include "experiment_recorder.h"
#include "force_calibration.h"
#include "force_feedback.h"
#include "force_transition_experiment.h"
#include "guidewire_mode.h"
#include "motion_sync.h"
#include "plc_io.h"
#include "sensor_calibration_experiment.h"
#include "startup_sequence.h"
#include "tcp_force_daq.h"
#include "vis_server.h"

#include <cmath>
#include <conio.h>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <windows.h>

// 文件职责说明：
// 1) 本文件是 ADS 控制程序入口与主循环调度层。
// 2) 运动同步、导丝模式、启动流程、力反馈与 ADS 读写已拆分到独立模块。
// 3) 新增调试模式通过独立状态接管，退出后重建原有业务链路基准。

int main(int argc, char* argv[])
{
	setup_console_utf8();
	if (sensor_calibration_experiment::is_command(argc, argv))
	{
		return sensor_calibration_experiment::run(argc, argv);
	}

	constexpr DWORD physical_handle_582_serial = 582;
	constexpr DWORD physical_handle_587_serial = 587;
	// 主循环允许通信线程最多 300 ms 产出下一帧，避免低频 ADS 请求占用期间误触发连接保持。
	constexpr DWORD ads_snapshot_wait_timeout_ms = 300;
	// WPF 状态只用于显示；降低推送频率不会改变 100 Hz ADS、运动或实验记录节拍。
	constexpr std::int64_t vis_publish_rate_hz = 15;
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

	// B7 选择当前物理手柄语义，选中后对应手柄的 B6 电平决定递送或撤出。
	// 启动尚未完成时仍保留 main 原有的 B6 暂停安全语义；正式控制阶段 B6 只作为方向电平。
	const unsigned char axis1_pause_button_mask = cfg.btn_b6;

	// 长生命周期运行时对象。
	Handle handle_axis1(serial_axis1_handle);
	Handle handle_axis6(serial_axis6_handle);
	CADSComm ads;
	TcpForceDaqClient tcp_force_daq;
	DeliveryTrackingController tracking_controller;
	ExperimentRecorder experiment_recorder;
	ForceTransitionExperiment ft_exp;
	VisServer vis_server;
	HandleFilterState axis1_handle_filter;
	HandleFilterState axis6_handle_filter;
	// 首位启动 ADS 100 Hz 后台通信服务与辅助服务，彻底消除主线程直连并发竞争。
	AdsCommunicationService ads_communication(ads);
	ArmManualAdsService arm_manual_ads(ads_communication);
	if (!ads_communication.start(nullptr, nullptr))
	{
		std::cout << "警告：ADS 100 Hz 通信后台服务启动失败。" << std::endl;
	}
	if (!arm_manual_ads.start())
	{
		std::cout << "定位臂低频 ADS 服务启动失败，定位臂 UI 将保持不可用。" << std::endl;
	}
	vis_server.start();

	bool axis1_handle_ready = handle_axis1.init();
	if (!axis1_handle_ready)
	{
		std::cout << "手柄初始化未就绪，序列号: " << serial_axis1_handle << "，将在后台持续重试。" << std::endl;
	}
	const bool axis6_handle_ready_init = handle_axis6.init();
	bool axis6_handle_ready = axis6_handle_ready_init;
	if (!axis6_handle_ready)
	{
		std::cout << "手柄初始化未就绪，序列号: " << serial_axis6_handle << "，将在后台持续重试。" << std::endl;
	}

	bool handle_startup_locked = false;
	bool single_handle_mode = false;
	bool dual_handle_ready = false;
	GuidewireMode single_handle_requested_mode = GuidewireMode::None;
	Handle* axis1_input_handle = &handle_axis1;
	Handle* axis6_input_handle = &handle_axis6;
	Handle* catheter_force_output_handle = &handle_axis1;
	Handle* guidewire_force_output_handle = &handle_axis6;

	auto lock_handle_mode = [&](bool axis1_ok, bool axis6_ok)
	{
		if (handle_startup_locked) return;
		if (axis1_ok && axis6_ok)
		{
			single_handle_mode = false;
			dual_handle_ready = true;
			axis1_input_handle = &handle_axis1;
			axis6_input_handle = &handle_axis6;
			catheter_force_output_handle = &handle_axis1;
			guidewire_force_output_handle = &handle_axis6;
			handle_startup_locked = true;
			std::cout << "双手柄模式已锁定（582/导管 + 587/导丝）。" << std::endl;
		}
		else if (axis1_ok || axis6_ok)
		{
			single_handle_mode = true;
			dual_handle_ready = false;
			Handle* single_handle = axis1_ok ? &handle_axis1 : &handle_axis6;
			const DWORD single_serial = axis1_ok ? serial_axis1_handle : serial_axis6_handle;
			axis1_input_handle = single_handle;
			axis6_input_handle = single_handle;
			catheter_force_output_handle = single_handle;
			guidewire_force_output_handle = single_handle;
			single_handle_requested_mode = GuidewireMode::None;
			handle_startup_locked = true;
			std::cout << "单手柄模式已锁定（序列号: " << single_serial << "）。" << std::endl;
		}
	};

	if (axis1_handle_ready || axis6_handle_ready)
	{
		lock_handle_mode(axis1_handle_ready, axis6_handle_ready);
	}
	else
	{
		std::cout << "两只手柄均未连接，程序进入等待连接状态（每隔 1 秒重试）..." << std::endl;
	}
	ForceFeedbackState ff;

	// 力输出统一入口：轴向力按配置的 SDK force axis 下发，力矩仍走 torque 参数。
	auto apply_force_output = [&](double force_582_f, double force_582_n, double force_587_f, double force_587_n)
	{
		handle_axis1.setforce_axis(force_582_f, cfg.axial_force_axis, force_582_n);
		handle_axis6.setforce_axis(force_587_f, cfg.axial_force_axis, force_587_n);
	};

	// 双手柄力输出清零（用于暂停、急停保持、F=OFF 等场景）。
	auto clear_force_output = [&]()
	{
		// 立即清除闭爪保持状态，避免安全事件后的下一拍继续复用旧力值。
		ff.clear_clamp_holds();
		apply_force_output(0.0, 0.0, 0.0, 0.0);
	};

	// PLC 镜像状态数组：在生成新的 refer 帧前会先通过 ADS 刷新。
	double pos[7] = { 0 }; // 上位机本周期目标（将写入 G.refer，坐标系：相对 init_pos）
	double plc_act_pos[7] = { 0 }; // PLC 当前相对位置（G.Act_pos）
	double plc_init_pos[7] = { 0 }; // PLC 相对零点偏置（G.init_pos）
	double plc_leftlimit[7] = { 0 }; // 左限位绝对位置（G.leftlimit）
	double plc_act_pos_from_left[7] = { 0 };
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
	ctx.plc_v_limit = plc_v_limit;
	ctx.startup_smoothing_bypass = &startup_smoothing_bypass;
	// 绑定上下文中的 ADS 通信后台服务指针。
	ctx.ads_service = &ads_communication;
	// 手柄模式可能在启动等待阶段才锁定，后续同步模块始终读取当前锁定的手柄指针。
	auto update_handle_context = [&]()
	{
		ctx.axis1_input_handle = axis1_input_handle;
		ctx.axis6_input_handle = axis6_input_handle;
	};
	update_handle_context();

	auto read_plc_state = [&]() -> bool { return plc_io::read_plc_state(ctx); };
	auto read_force_sample = [&](ForceSampleFrame& sample) -> bool { return plc_io::read_force_sample(ctx, sample); };
	auto load_pos_from_actual = [&]() { plc_io::load_pos_from_actual(ctx); };
	auto from_left_to_abs = [&](int axis_index, double from_left_mm) -> double
	{
		return motion_sync::from_left_to_abs(ctx, axis_index, from_left_mm);
	};
	auto from_left_to_rel = [&](int axis_index, double from_left_mm) -> double
	{
		return motion_sync::from_left_to_rel(ctx, axis_index, from_left_mm);
	};
	auto clear_axis_return_request = [&](const AxisReturnAdsSymbols& symbols) -> bool
	{
		return plc_io::clear_axis_return_request(ctx, symbols);
	};
	auto clear_axis_return_requests = [&](const AxisReturnAdsSymbols* const* symbols, int count) -> bool
	{
		return plc_io::clear_axis_return_requests(ctx, symbols, count);
	};
	auto clear_axis1_group_return_requests = [&]() -> bool
	{
		return plc_io::clear_axis1_group_return_requests(ctx);
	};
	// 常规导管模式的跟随基准。
	double axis3_base_rel = 0.0;
	double axis5_base_rel = 0.0;
	double axis6_mirror_base_rel = 0.0;
	// axis6 回退后采用“同方向重入”门控，避免刚回退完就立刻重复触发。
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
	// 普通导管递送在 axis1 计划回退完成后，前 10 mm 手柄输入的比例映射状态。
	bool axis1_delivery_mapping_active = false;
	double axis1_delivery_mapping_progress_mm = 0.0;
	double axis1_delivery_mapping_applied_extra_mm = 0.0;
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
	// 协同模式 axis5 命令增量差分基准，用于判断 axis6 相对窗口运动方向。
	bool axis6_coop_ff_inited = false;
	double axis6_coop_prev_axis1_cmd_abs = 0.0;
	// 协同方向分为 UI 请求值和已激活值。入口或方向切换失败时保留已激活方向，
	// 避免失败请求改变正在运行的运动方向。
	CooperativeDirection cooperative_direction_requested = CooperativeDirection::None;
	CooperativeDirection cooperative_direction = CooperativeDirection::None;

	StartupState startup;
	ForceCalibrationConfig cal_cfg;
	ForceCalibrationState cal_state;
	CrawlState axis1_crawl;
	CrawlState axis6_crawl;
	PlannedReturnCoordinator planned_return;
	SpacingRecoveryState spacing_recovery;
	axis1_crawl.enabled = true;
	// 最终启动默认姿态与标准启动的中间夹持位置分开设置。
	// 635/635/640 可保持缩短后的 0 mm、1 mm 相对差，同时让完整
	// axis6 运行窗口 [axis5+1, axis5+21] 不超过 670 mm 软上限。
	startup.final_axis1_from_left_mm = cfg.startup_final_axis1_default_from_left_mm;
	startup.final_axis3_from_left_mm = cfg.startup_final_axis3_default_from_left_mm;
	startup.final_axis5_from_left_mm = cfg.startup_final_axis5_default_from_left_mm;
	startup.final_axis6_from_left_mm = cfg.startup_final_axis6_default_from_left_mm;

	// 绑定上下文中的长生命周期状态，替代大段 lambda capture。
	ctx.guidewire_mode = &guidewire_mode;
	ctx.axis1_crawl = &axis1_crawl;
	ctx.axis6_crawl = &axis6_crawl;
	ctx.startup = &startup;
	ctx.ff = &ff;
	ctx.axis3_base_rel = &axis3_base_rel;
	ctx.axis5_base_rel = &axis5_base_rel;
	ctx.axis6_mirror_base_rel = &axis6_mirror_base_rel;
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
	auto capture_axis1_follow_baseline = [&]() { motion_sync::capture_axis1_follow_baseline(ctx); };
	auto clear_axis1_delivery_mapping = [&]()
	{
		axis1_delivery_mapping_active = false;
		axis1_delivery_mapping_progress_mm = 0.0;
		axis1_delivery_mapping_applied_extra_mm = 0.0;
	};
	auto arm_axis1_delivery_mapping = [&]()
	{
		axis1_delivery_mapping_active = cfg.axis1_post_return_lead_mm > 1e-6;
		axis1_delivery_mapping_progress_mm = 0.0;
		axis1_delivery_mapping_applied_extra_mm = 0.0;
	};
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
	auto consume_startup_loading_ready = [&]() -> bool { return startup_sequence::consume_startup_loading_ready(ctx); };
	auto restore_startup_v_limit = [&]() -> bool { return startup_sequence::restore_startup_v_limit(ctx); };
	auto prompt_startup_mode = [&]() { startup_sequence::prompt_startup_mode(ctx); };
	// 递送只接受手柄“推”产生的负轴向增量，撤出只接受“拉”产生的正轴向增量。
	// 被拒绝的增量仍会在循环末尾刷新采样基准，因此不会积压到后续控制拍。
	auto gate_linear_increment_for_mode = [](double increment_mm, bool reverse_mode) -> double
	{
		if ((!reverse_mode && increment_mm < 0.0) || (reverse_mode && increment_mm > 0.0))
		{
			return increment_mm;
		}
		return 0.0;
	};

	// 启动阶段不做同步 ADS 读写；等待后台服务发布首个有效快照后再初始化位置和参考。
	// ADS 或 PLC 尚未就绪时只进入保持/重连状态，不直接退出。
	load_pos_from_actual();
	axis2_hold_rel = 0.0;
	axis7_hold_rel = 0.0;
	axis1_follow_cmd_abs = 0.0;
	axis6_follow_cmd_abs = 0.0;
	clear_axis1_delivery_mapping();
	axis1_prev_abs_valid = false;
	axis6_prev_abs_valid = false;

	bool self_check_done = true;
	bool has_self_check_flag = false;

	bool control_active = !has_self_check_flag || self_check_done;
	bool last_self_check_done = self_check_done;
	bool handle_reinit_req = false;
	bool last_handle_reinit_req = false;
	bool estop_hold_req = false;
	bool estop_hold_active = false;
	bool axis1_push_rearm_after_hold = false;
	bool axis1_delivery_stop_latched = false;
	bool axis1_delivery_stop_prompted = false;
	bool freeze_active = false;
	bool pause_pressed_prev = false;
	bool axis1_reverse_pressed_prev = false;
	bool axis6_effective_reverse_prev = false;
	bool catheter_mode_button_pressed_prev =
		(axis1_input_handle->buttons2 & cfg.btn_b7) != 0;
	bool guidewire_mode_button_pressed_prev =
		(axis6_input_handle->buttons2 & cfg.btn_b7) != 0;
	bool axis4_manual_error_prev = false;
	unsigned long axis4_manual_error_id_prev = 0;
	bool axis4_ui_forward_pressed = false;
	bool axis4_ui_reverse_pressed = false;
	ULONGLONG axis4_ui_jog_deadline_ms = 0;
	constexpr ULONGLONG axis4_ui_jog_lease_ms = 300;
	bool y_valve_open = false;
	int injector_ui_direction[2] = {};
	ULONGLONG injector_ui_jog_deadline_ms[2] = {};
	constexpr ULONGLONG injector_ui_jog_lease_ms = 300;
	GuidewireMode requested_guidewire_mode_prev = GuidewireMode::None;
	bool axis1_fast_return = false; // 轴1快退旁路标志（写入 G.axis1_fast_return）
	bool axis6_fast_retract = false; // 轴6快退旁路标志（写入 G.axis6_fast_retract）
	bool return_ads_fault_hold = false; // 回退 ADS 故障后保持停控，重启上位机并重新检查后解除
	// PLC ErrorId 可能比 Error 边沿晚一个 Notification 到达；短暂保留本次错误上下文用于补报。
	int pending_return_error_axis = -1;
	std::uint64_t pending_return_error_event_sequence = 0;
	ULONGLONG pending_return_error_deadline_ms = 0;
	// axis6 软限位只在当前危险动作或实际越限时阻断，不把一次被拒绝的换手永久锁死。
	enum class Axis6SoftLimitReason : unsigned char
	{
		None,
		ActualPosition,
		HandleTarget,
		PlannedReturn,
		Axis1CoupledReturn
	};
	bool axis6_soft_limit_hold = false;
	Axis6SoftLimitReason axis6_soft_limit_reason = Axis6SoftLimitReason::None;
	bool axis6_soft_limit_warning_active = false;
	Axis6SoftLimitReason axis6_soft_limit_warning_reason = Axis6SoftLimitReason::None;
	double axis6_soft_limit_blocked_target_from_left_mm = 0.0;
	ForceSampleFrame force_sample;
	int loop_count = 0;
	DWORD force_sample_last_sample_ms = 0;
	bool clean_force_monitor_enabled = false;
	std::string force_feedback_diag_reason;
	std::string force_sample_diag_reason;
	// 统一回退的完成、重试和取消均以同一份100 Hz快照序号判定“之后的新鲜帧”。
	std::uint64_t ads_snapshot_sequence = 0;
	auto planned_return_phase_for_axis = [&](int axis_index) -> int
	{
		return planned_return.compatibility_phase_for_axis(axis_index);
	};
	auto current_cooperative_return_owner = [&]() -> CooperativeReturnOwner
	{
		return planned_return.cooperative_owner();
	};
	auto clear_planned_return_ads_command_tracking = [&]()
	{
		planned_return.ads_command_purpose = PlannedReturnAdsCommandPurpose::None;
		planned_return.ads_command_sequence = 0;
		planned_return.ads_command_submit_ms = 0;
	};
	auto planned_return_active_axis_mask = [&]() -> std::uint8_t
	{
		std::uint8_t mask = 0;
		for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
		{
			const PlannedReturnLeg& leg = planned_return.legs[leg_index];
			if (!leg.active) continue;
			if (leg.axis_index == 0) mask |= 0x01u;
			else if (leg.axis_index == 5) mask |= 0x02u;
		}
		return mask;
	};
	auto latch_planned_return_possibly_started = [&](std::uint8_t mask)
	{
		for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
		{
			PlannedReturnLeg& leg = planned_return.legs[leg_index];
			const std::uint8_t axis_mask = leg.axis_index == 0 ? 0x01u :
				(leg.axis_index == 5 ? 0x02u : 0x00u);
			if ((mask & axis_mask) != 0) leg.possibly_started = true;
		}
	};
	auto submit_planned_return_ads_command = [&](AdsPlannedReturnOperation operation,
		PlannedReturnAdsCommandPurpose purpose,
		bool replace_pending) -> bool
	{
		if (planned_return.leg_count <= 0) return false;
		if (!replace_pending && planned_return.ads_command_sequence != 0) return false;

		AdsPlannedReturnCommand command{};
		command.operation = operation;
		for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
		{
			const PlannedReturnLeg& leg = planned_return.legs[leg_index];
			if (!leg.active || (leg.axis_index != 0 && leg.axis_index != 5)) return false;
			AdsPlannedReturnLegCommand& command_leg = command.legs[command.leg_count++];
			command_leg.axis_index = leg.axis_index;
			command_leg.target_abs = leg.target_abs;
			const bool is_axis1 = leg.axis_index == 0;
			command_leg.velocity = is_axis1
				? cfg.axis1_return_velocity_mm_s : cfg.axis6_return_velocity_mm_s;
			command_leg.acc = is_axis1
				? cfg.axis1_return_acc_mm_s2 : cfg.axis6_return_acc_mm_s2;
			command_leg.dec = is_axis1
				? cfg.axis1_return_dec_mm_s2 : cfg.axis6_return_dec_mm_s2;
			command_leg.jerk = is_axis1
				? cfg.axis1_return_jerk_mm_s3 : cfg.axis6_return_jerk_mm_s3;
		}

		const std::uint64_t sequence = ads_communication.submit_planned_return_command(command);
		if (sequence == 0) return false;
		planned_return.ads_command_purpose = purpose;
		planned_return.ads_command_sequence = sequence;
		planned_return.ads_command_submit_ms = GetTickCount64();
		return true;
	};
	enum class PlannedReturnAdsPoll : unsigned char
	{
		Pending,
		Success,
		Failure
	};
	auto poll_planned_return_ads_command = [&]() -> PlannedReturnAdsPoll
	{
		if (planned_return.ads_command_sequence == 0) return PlannedReturnAdsPoll::Failure;
		bool completed = false;
		bool success = false;
		std::uint8_t possibly_started_mask = 0;
		if (!ads_communication.planned_return_command_result(
			planned_return.ads_command_sequence,
			completed,
			success,
			possibly_started_mask))
		{
			if (planned_return.ads_command_purpose == PlannedReturnAdsCommandPurpose::Commit)
			{
				// 结果槽不可判定时，不能证明Req没有到达PLC；按全部任务腿保守锁存。
				latch_planned_return_possibly_started(planned_return_active_axis_mask());
			}
			return PlannedReturnAdsPoll::Failure;
		}
		if (!completed) return PlannedReturnAdsPoll::Pending;
		if (!success &&
			planned_return.ads_command_purpose == PlannedReturnAdsCommandPurpose::Commit)
		{
			latch_planned_return_possibly_started(possibly_started_mask);
		}
		return success ? PlannedReturnAdsPoll::Success : PlannedReturnAdsPoll::Failure;
	};
	auto planned_return_ads_command_timed_out = [&](ULONGLONG now_ms) -> bool
	{
		return planned_return.ads_command_sequence != 0 &&
			planned_return.ads_command_submit_ms != 0 &&
			(now_ms - planned_return.ads_command_submit_ms) >=
			cfg.planned_return_ack_timeout_ms;
	};
	auto reset_planned_return = [&]()
	{
		planned_return.reset();
		axis1_fast_return = false;
		axis6_fast_retract = false;
	};
	// 自动换手不能穿越暂停、自检重建或 ADS 故障继续保留。
	// Req清除成功只表示命令已送达；必须继续等待PLC Busy/Error清零和后续新鲜快照，
	// 才能重建基准并把协调器恢复为Follow，避免本地先于PLC伪装完成。
	auto cancel_active_return_motion = [&](bool clear_plc_requests) -> bool
	{
		const bool had_active_return = planned_return.active();
		if (!had_active_return) return true;
		clear_axis1_delivery_mapping();
		if (planned_return.phase == PlannedReturnPhase::CancelWait)
		{
			return planned_return.cancel_clear_confirmed;
		}

		const PlannedReturnPhase interrupted_phase = planned_return.phase;
		const bool handoff_clear_already_applied =
			(interrupted_phase == PlannedReturnPhase::AwaitHandoffApplied ||
				interrupted_phase == PlannedReturnPhase::PostHandoffClampSettle) &&
			planned_return.handoff_generation != 0 &&
			ads_communication.applied_output_generation() >=
			planned_return.handoff_generation;
		if (handoff_clear_already_applied)
		{
			// 交接代次成功意味着本任务Req=FALSE、实际位置保持和夹爪恢复
			// 已在同一次Sum Write中写入；取消链不得再等待一个不会发生的重复通知。
			planned_return.completion_clear_confirmed = true;
		}
		const bool plc_request_can_be_active = clear_plc_requests &&
			interrupted_phase != PlannedReturnPhase::ClampSettle &&
			interrupted_phase != PlannedReturnPhase::PostHandoffClampSettle &&
			!planned_return.completion_clear_confirmed;
		const AdsEventState cancel_events = ads_communication.event_state();
		for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
		{
			PlannedReturnLeg& leg = planned_return.legs[leg_index];
			const bool axis1 = leg.axis_index == 0;
			const bool busy = axis1
				? cancel_events.axis1_return_busy : cancel_events.axis6_return_busy;
			leg.cancel_event_sequence = axis1
				? cancel_events.axis1_return_event_sequence
				: cancel_events.axis6_return_event_sequence;
			leg.cancel_event_required = plc_request_can_be_active && (leg.started || busy);
		}
		axis6_coop_ff_inited = false;
		axis6_coop_prev_axis1_cmd_abs = 0.0;
		axis1_fast_return = false;
		axis6_fast_retract = false;
		load_pos_from_actual();
		planned_return.phase = PlannedReturnPhase::CancelWait;
		planned_return.cancel_t0_ms = GetTickCount64();
		planned_return.cancel_timeout_reported = false;
		planned_return.cancel_rebased = false;
		planned_return.cancel_hold_generation = 0;
		planned_return.cancel_clear_confirmed = !plc_request_can_be_active;
		planned_return.cancel_snapshot_sequence = ads_snapshot_sequence;
		clear_planned_return_ads_command_tracking();
		bool clear_submitted = true;
		if (plc_request_can_be_active)
		{
			clear_submitted = submit_planned_return_ads_command(
				AdsPlannedReturnOperation::Clear,
				PlannedReturnAdsCommandPurpose::CancelClear,
				true);
			if (!clear_submitted)
			{
				planned_return.cancel_clear_retry_after_ms = GetTickCount64() + 250;
				std::cout << "自动换手取消：清Req命令暂未进入通信队列，将在ADS可用后重试。"
					<< std::endl;
			}
		}
		return clear_submitted;
	};
	auto cancel_cooperative_delivery = [&](bool leave_active_mode)
	{
		(void)cancel_active_return_motion(true);
		cooperative_direction_requested = CooperativeDirection::None;
		cooperative_direction = CooperativeDirection::None;
		if (leave_active_mode && guidewire_mode == GuidewireMode::Cooperative)
		{
			guidewire_mode = GuidewireMode::None;
			axis6_crawl.enabled = false;
			axis6_window_locked = false;
			axis6_coop_ff_inited = false;
			axis6_coop_prev_axis1_cmd_abs = 0.0;
		}
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
	auto engage_axis6_soft_limit_hold = [&](double target_abs,
		Axis6SoftLimitReason reason,
		const char* source)
	{
		// 实际位置越限的冻结优先级高于任何预测原因，避免同一拍的
		// axis1 联动检查覆盖 ActualPosition，下一拍误把真实越限解锁。
		if (axis6_soft_limit_reason == Axis6SoftLimitReason::ActualPosition &&
			reason != Axis6SoftLimitReason::ActualPosition)
		{
			axis6_soft_limit_hold = true;
			return;
		}
		axis6_soft_limit_hold = true;
		axis6_soft_limit_reason = reason;
		axis6_soft_limit_blocked_target_from_left_mm = axis6_from_left_mm(target_abs);
		const bool new_warning = !axis6_soft_limit_warning_active ||
			axis6_soft_limit_warning_reason != reason;
		if (new_warning)
		{
			axis6_soft_limit_warning_active = true;
			axis6_soft_limit_warning_reason = reason;
			// 预测或实际越限时精确取消当前统一回退任务，避免PLC继续接受超限目标。
			if (planned_return.active())
			{
				(void)cancel_active_return_motion(true);
			}
		}
		axis6_coop_ff_inited = false;
		axis1_fast_return = false;
		axis6_fast_retract = false;
		clear_force_output();
		if (new_warning)
		{
			std::cout << "警告：axis6 软件限位阻断（" << source
				<< "，目标距左限位 " << axis6_soft_limit_blocked_target_from_left_mm
				<< " mm > " << cfg.axis6_soft_limit_from_left_mm
				<< " mm）。本拍不下发越限运动；松手、改变模式或回到安全窗口后重新评估。" << std::endl;
		}
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
		if (planned_return.active())
		{
			std::cout << mode_name << "进入被拒绝：轴1或轴6仍在执行回退或夹爪切换。" << std::endl;
			return false;
		}

		if (ads_communication.stats().state != AdsConnectionState::Running)
		{
			std::cout << mode_name << "进入被拒绝：ADS 通信服务未运行。" << std::endl;
			return false;
		}
		const AdsEventState entry_events = ads_communication.event_state();
		if (entry_events.axis1_return_busy || entry_events.axis6_return_busy)
		{
			std::cout << mode_name << "进入被拒绝：PLC 计划回退仍处于 Busy。" << std::endl;
			return false;
		}
		if (entry_events.axis1_return_error || entry_events.axis6_return_error)
		{
			std::cout << mode_name << "进入被拒绝：PLC 计划回退仍有未清除错误。" << std::endl;
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

	std::cout << "力反馈：关闭（按 F 键切换）。" << std::endl;
	clear_force_output();

	bool force_tcp_zero_wait_logged = false;
	// TCP 采集卡是力输入源，不再与任何磁盘记录开关绑定。
	if (ctx.force_sample_source == ForceSampleSource::TCP_DAQ)
	{
		if (tcp_force_daq.start(cfg.tcp_force_daq_ip, cfg.tcp_force_daq_port, cfg.tcp_force_daq_local_ip))
		{
			std::cout << "力采样源：TCP_DAQ（" << cfg.tcp_force_daq_ip << ":" << cfg.tcp_force_daq_port
				<< "，本机绑定 " << cfg.tcp_force_daq_local_ip << "）。" << std::endl;
		}
		else
		{
			std::cout << "力采样源：TCP_DAQ 后台线程启动失败。" << std::endl;
		}
	}

	auto zero_force_sensor = [&](const char* source) -> bool
	{
		if (ctx.force_sample_source == ForceSampleSource::TCP_DAQ)
		{
			if (!force_tcp_zero_wait_logged)
			{
				std::cout << source
					<< "零点采集失败：本次F_direct仅由ADS/PLC计数标定，TCP_DAQ同源性尚未确认；"
					<< "不会对未验证的TCP数据启用力反馈。" << std::endl;
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
			cal_state.fn_2_zero = sampled_frame.fn_2_value_v;
			cal_state.ft_2_zero = sampled_frame.ft_2_value_v;
			cal_state.theta0_deg = sampled_frame.axis2_pos_rel;
			cal_state.zeroed = true;
			ff.reset();
			clear_force_output();
			std::cout << source << "力传感器零点已采集（ADS 当前采样）：ft_zero=" << cal_state.ft_zero
				<< " V, f_zero=" << cal_state.f_zero << " V"
				<< ", fn_2_zero=" << cal_state.fn_2_zero << " V"
				<< ", ft_2_zero=" << cal_state.ft_2_zero << " V"
				<< ", axis2/theta0=" << cal_state.theta0_deg << " deg" << std::endl;
			return true;
		}

		std::cout << source << "零点采集失败：ADS 力采样读取失败。" << std::endl;
		return false;
	};

	bool initial_sync_done = false;
	bool plc_app_name_read = false;
	ULONGLONG handle1_next_retry_ms = 0;
	ULONGLONG handle6_next_retry_ms = 0;
	bool handle1_reconnect_pending_poll = false;
	bool handle6_reconnect_pending_poll = false;
	bool handle_soft_hold_active = true;
	bool connection_hold_active_prev = true;

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
	// 手动电缸只用于调试。自动运动开始接管时必须撤销全部覆盖，避免末尾写入压住换手状态机命令。
	auto clear_cylinder_manual_overrides = [&]()
	{
		for (int cylinder_index = 0; cylinder_index < 4; ++cylinder_index)
		{
			if (cylinder_manual_mode[cylinder_index] != CylinderManualMode::Automatic)
			{
				cylinder_manual_mode[cylinder_index] = CylinderManualMode::Automatic;
			}
		}
	};
	bool vis_reverse_override_active = false;
	bool vis_reverse_override_value = false;
	int vis_reverse_override_target = 0;
	enum class PhysicalModeSource : unsigned char
	{
		None,
		Catheter,
		Guidewire
	};
	PhysicalModeSource physical_mode_source = PhysicalModeSource::None;
	PhysicalModeSource pending_physical_mode_source = PhysicalModeSource::None;
	bool physical_mode_request_pending = false;
	PhysicalModeSource physical_mode_source_before_request = PhysicalModeSource::None;
	bool vis_reverse_override_active_before_request = false;
	bool vis_reverse_override_value_before_request = false;
	int vis_reverse_override_target_before_request = 0;
	enum class ModeSelection : int
	{
		None = 0,
		CatheterDelivery = 1,
		CatheterRetraction = 2,
		GuidewireDelivery = 3,
		GuidewireRetraction = 4,
		CooperativeDelivery = 5,
		CooperativeRetraction = 6
	};
	ModeSelection pending_mode_selection = ModeSelection::None;
	std::uint32_t physical_button_event_counter = 0;
	int physical_button_event_code = 0;

	auto apply_mode_selection = [&](ModeSelection selection,
		PhysicalModeSource mode_source = PhysicalModeSource::None)
	{
		clear_axis1_delivery_mapping();
		// UI/键盘选择清除物理模式源；B7选择保留对应手柄作为模式源。
		physical_mode_source = mode_source;
		if (selection == ModeSelection::CooperativeDelivery ||
			selection == ModeSelection::CooperativeRetraction)
		{
			cooperative_direction_requested =
				selection == ModeSelection::CooperativeRetraction
				? CooperativeDirection::Retraction
				: CooperativeDirection::Delivery;
			clear_cylinder_manual_overrides();
			return;
		}

		const bool guidewire =
			selection == ModeSelection::GuidewireDelivery ||
			selection == ModeSelection::GuidewireRetraction;
		const bool reverse =
			selection == ModeSelection::CatheterRetraction ||
			selection == ModeSelection::GuidewireRetraction;
		if (single_handle_mode)
		{
			single_handle_requested_mode = guidewire ? GuidewireMode::Independent : GuidewireMode::None;
		}
		cooperative_direction_requested = CooperativeDirection::None;
		clear_cylinder_manual_overrides();
		vis_reverse_override_active = true;
		vis_reverse_override_target = guidewire ? 1 : 0;
		vis_reverse_override_value = reverse;
	};

	auto request_mode_selection = [&](ModeSelection selection,
		const char* source,
		PhysicalModeSource mode_source = PhysicalModeSource::None)
	{
		if (mode_source == PhysicalModeSource::None)
		{
			// UI/键盘选择即使被恢复流程延后，也立即退出物理模式源。
			physical_mode_source = PhysicalModeSource::None;
		}
		if (spacing_recovery.active() || spacing_recovery.requested)
		{
			pending_mode_selection = selection;
			pending_physical_mode_source = mode_source;
			spacing_recovery.requested = false;
			std::cout << source << "：正在退出屈曲恢复并重同步，随后切换目标模式。" << std::endl;
			return;
		}
		const bool automatic_handoff_active = planned_return.active();
		if (automatic_handoff_active)
		{
			std::cout << source << "：模式/方向切换已忽略，请等待当前换手完成。"
				<< std::endl;
			return;
		}
		pending_mode_selection = ModeSelection::None;
		pending_physical_mode_source = PhysicalModeSource::None;
		apply_mode_selection(selection, mode_source);
	};

	struct PendingStartupParams {
		double axis1_from_left_mm = 0.0;
		double axis3_from_left_mm = 0.0;
		double axis5_from_left_mm = 0.0;
		double axis6_from_left_mm = 0.0;
		double axis2_deg = 0.0;
		double axis7_deg = 0.0;
		double speed_scale = 0.0;
	} pending_startup;
	pending_startup.axis1_from_left_mm = startup.final_axis1_from_left_mm;
	pending_startup.axis3_from_left_mm = startup.final_axis3_from_left_mm;
	pending_startup.axis5_from_left_mm = startup.final_axis5_from_left_mm;
	pending_startup.axis6_from_left_mm = startup.final_axis6_from_left_mm;
	pending_startup.speed_scale = cfg.startup_motion_speed_scale;
	bool tracking_manual_cylinder_override = false;
	ads_snapshot_sequence = 0;
	std::uint64_t handled_plc_restart_count = 0;
	std::uint64_t handled_reconnect_count = 0;
	bool ads_soft_hold_active = true;
	bool plc_restart_recovery_latched = false;
	std::vector<AdsFastSnapshot> drained_ads_snapshots;
	struct HandleRecordSnapshot
	{
		std::int64_t qpc_ticks = 0;
		double axis1_linear_raw = 0.0;
		double axis1_linear_filtered = 0.0;
		double axis1_rotation_raw = 0.0;
		double axis1_rotation_filtered = 0.0;
		double axis6_linear_raw = 0.0;
		double axis6_linear_filtered = 0.0;
		double axis6_rotation_raw = 0.0;
		double axis6_rotation_filtered = 0.0;
		bool axis1_valid = false;
		bool axis6_valid = false;
	};
	std::deque<HandleRecordSnapshot> handle_record_history;
	std::uint64_t force_sample_ads_sequence = 0;
	bool ft_v_limit_last_valid = false;
	double ft_v_limit_last[7] = {};
	LARGE_INTEGER vis_qpc_frequency{};
	QueryPerformanceFrequency(&vis_qpc_frequency);
	std::int64_t next_vis_publish_qpc = 0;

	while (true)
	{
		AdsFastSnapshot ads_snapshot{};
		const bool has_new_ads_snapshot = ads_communication.wait_for_snapshot(
			ads_snapshot_sequence, ads_snapshot_wait_timeout_ms, ads_snapshot);
		if (has_new_ads_snapshot)
		{
			ads_snapshot_sequence = ads_snapshot.attempt_sequence;
			if (ads_snapshot.position_valid)
			{
				(void)read_plc_state();
			}
		}
		ads_communication.drain_snapshots(drained_ads_snapshots);
		const AdsCommunicationStats ads_stats = ads_communication.stats();
		const AdsEventState ads_events = ads_communication.event_state();
		if (pending_return_error_axis >= 0)
		{
			std::uint64_t error_id_sequence = 0;
			std::uint32_t error_id = 0;
			if (pending_return_error_axis == 0)
			{
				error_id_sequence = ads_events.axis1_return_last_nonzero_error_id_sequence;
				error_id = ads_events.axis1_return_last_nonzero_error_id;
			}
			else if (pending_return_error_axis == 5)
			{
				error_id_sequence = ads_events.axis6_return_last_nonzero_error_id_sequence;
				error_id = ads_events.axis6_return_last_nonzero_error_id;
			}
			if (error_id != 0 && error_id_sequence >= pending_return_error_event_sequence)
			{
				std::cout << "axis" << (pending_return_error_axis + 1)
					<< " 计划回退最终错误码: " << error_id << std::endl;
				pending_return_error_axis = -1;
				pending_return_error_event_sequence = 0;
				pending_return_error_deadline_ms = 0;
			}
			else if (pending_return_error_deadline_ms != 0 &&
				GetTickCount64() >= pending_return_error_deadline_ms)
			{
				pending_return_error_axis = -1;
				pending_return_error_event_sequence = 0;
				pending_return_error_deadline_ms = 0;
			}
		}
		self_check_done = ads_events.self_check_done;
		handle_reinit_req = ads_events.handle_reinit_req;
		estop_hold_req = ads_events.estop_hold_req || ads_events.host_comm_timeout;
		if (ads_stats.reconnect_count != handled_reconnect_count)
		{
			handled_reconnect_count = ads_stats.reconnect_count;
			ads_communication.request_coordinate_refresh();
			ads_communication.request_watchdog_recovery();
			spacing_recovery.reset();
			if (ft_exp.active()) ft_exp.abort(ctx, "ADS reconnect");
			tracking_controller.disable_compensation();
			(void)cancel_active_return_motion(true);
			std::cout << "ADS 重连：已取消中断的回退、屈曲恢复、力过渡和 PI 瞬态，保留稳定模式与方向。" << std::endl;
		}
		// 运动控制只依赖位置快照；力数据质量单独由 force_valid 传给力反馈和记录器。
		const bool ads_motion_cycle_valid = has_new_ads_snapshot && ads_snapshot.position_valid &&
			ads_stats.state == AdsConnectionState::Running;
		if (ads_motion_cycle_valid && !has_self_check_flag)
		{
			// ADS 服务的连接初始化已完成这些符号的解析和初值读取，
			// 首个有效快照到达后再启用自检/启动流程状态，不访问未就绪的 ADS。
			has_self_check_flag = true;
			last_self_check_done = self_check_done;
			startup.loading_ready_symbol_available = true;
			startup.loading_ready_plc = ads_events.startup_loading_ready;
		}
		if (planned_return.phase == PlannedReturnPhase::CancelWait)
		{
			const ULONGLONG cancel_now_ms = GetTickCount64();
			if (planned_return.ads_command_purpose == PlannedReturnAdsCommandPurpose::CancelClear)
			{
				const PlannedReturnAdsPoll poll = poll_planned_return_ads_command();
				if (poll == PlannedReturnAdsPoll::Success)
				{
					clear_planned_return_ads_command_tracking();
					planned_return.cancel_clear_confirmed = true;
					planned_return.cancel_snapshot_sequence = ads_snapshot_sequence;
					planned_return.cancel_clear_retry_after_ms = 0;
				}
				else if (poll == PlannedReturnAdsPoll::Failure)
				{
					clear_planned_return_ads_command_tracking();
					planned_return.cancel_clear_retry_after_ms = cancel_now_ms + 250;
					std::cout << "PLC计划回退Req清除失败，将按250 ms间隔继续重试。"
						<< std::endl;
				}
				else if (planned_return_ads_command_timed_out(cancel_now_ms))
				{
					// ADS调用本身有短超时；这里再限制队列结果等待，防止通信线程异常时
					// CancelWait永久卡在一个Pending序号。旧Clear即使稍后执行也仍是幂等的。
					clear_planned_return_ads_command_tracking();
					planned_return.cancel_clear_retry_after_ms = cancel_now_ms + 250;
					std::cout << "PLC计划回退Req清除等待超过250 ms，将继续异步重试。"
						<< std::endl;
				}
			}
			if (!planned_return.cancel_clear_confirmed &&
				planned_return.ads_command_purpose == PlannedReturnAdsCommandPurpose::None &&
				ads_motion_cycle_valid && cancel_now_ms >= planned_return.cancel_clear_retry_after_ms)
			{
				if (!submit_planned_return_ads_command(
					AdsPlannedReturnOperation::Clear,
					PlannedReturnAdsCommandPurpose::CancelClear,
					false))
				{
					planned_return.cancel_clear_retry_after_ms = cancel_now_ms + 250;
				}
			}
			if (!planned_return.cancel_timeout_reported && ads_motion_cycle_valid &&
				planned_return.cancel_t0_ms != 0 &&
				(cancel_now_ms - planned_return.cancel_t0_ms) >=
				cfg.planned_return_execution_timeout_ms)
			{
				planned_return.cancel_timeout_reported = true;
				return_ads_fault_hold = true;
				control_active = false;
				clear_force_output();
				std::cout << "计划回退取消等待超过5 s，继续保持实际位置并要求人工重新启动。"
					<< std::endl;
			}
		}
		const ULONGLONG loop_now_ms = GetTickCount64();

		// 启动阶段未锁定模式时持续重试
		if (!handle_startup_locked)
		{
			if (loop_now_ms >= handle1_next_retry_ms)
			{
				handle1_next_retry_ms = loop_now_ms + 1000;
				axis1_handle_ready = handle_axis1.init();
			}
			if (loop_now_ms >= handle6_next_retry_ms)
			{
				handle6_next_retry_ms = loop_now_ms + 1000;
				axis6_handle_ready = handle_axis6.init();
			}
			if (axis1_handle_ready || axis6_handle_ready)
			{
				lock_handle_mode(axis1_handle_ready, axis6_handle_ready);
				update_handle_context();
			}
		}
		else
		{
			if (!handle_axis1.is_open() && loop_now_ms >= handle1_next_retry_ms)
			{
				handle1_next_retry_ms = loop_now_ms + 1000;
				if (handle_axis1.init())
				{
					handle1_reconnect_pending_poll = true;
					std::cout << "导管手柄 (SN " << serial_axis1_handle << ") 重新打开成功，等待首帧有效采样..." << std::endl;
				}
			}
			if (!handle_axis6.is_open() && loop_now_ms >= handle6_next_retry_ms)
			{
				handle6_next_retry_ms = loop_now_ms + 1000;
				if (handle_axis6.init())
				{
					handle6_reconnect_pending_poll = true;
					std::cout << "导丝手柄 (SN " << serial_axis6_handle << ") 重新打开成功，等待首帧有效采样..." << std::endl;
				}
			}
		}

		bool handle1_poll_ok = false;
		bool handle6_poll_ok = false;
		if (handle_startup_locked)
		{
			if (single_handle_mode)
			{
				Handle* active_handle = axis1_input_handle;
				if (active_handle != nullptr && active_handle->is_open())
				{
					if (active_handle->poll())
					{
						handle1_poll_ok = true;
						handle6_poll_ok = true;
						if (handle1_reconnect_pending_poll || handle6_reconnect_pending_poll)
						{
							handle1_reconnect_pending_poll = false;
							handle6_reconnect_pending_poll = false;
							axis1_handle_filter.reset(active_handle->fJoints2[0], active_handle->fJoints2[1]);
							axis6_handle_filter.reset(active_handle->fJoints2[0], active_handle->fJoints2[1]);
							axis1_prev_linear_filtered = axis1_handle_filter.axis0_filtered;
							axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;
							axis1_prev_rot_filtered = axis1_handle_filter.axis1_filtered;
							axis6_prev_rot_filtered = axis6_handle_filter.axis1_filtered;
							catheter_mode_button_pressed_prev = (active_handle->buttons2 & cfg.btn_b7) != 0;
							guidewire_mode_button_pressed_prev = (active_handle->buttons2 & cfg.btn_b7) != 0;
							std::cout << "单手柄重连后取得首帧有效采样，基准已重新建立。" << std::endl;
						}
					}
					else
					{
						active_handle->close();
						if (active_handle == &handle_axis1) handle1_next_retry_ms = loop_now_ms + 1000;
						if (active_handle == &handle_axis6) handle6_next_retry_ms = loop_now_ms + 1000;
						std::cout << "单手柄轮询失败并已断开，进入保持状态。" << std::endl;
					}
				}
			}
			else
			{
				if (handle_axis1.is_open())
				{
					if (handle_axis1.poll())
					{
						handle1_poll_ok = true;
						if (handle1_reconnect_pending_poll)
						{
							handle1_reconnect_pending_poll = false;
							axis1_handle_filter.reset(handle_axis1.fJoints2[0], handle_axis1.fJoints2[1]);
							axis1_prev_linear_filtered = axis1_handle_filter.axis0_filtered;
							axis1_prev_rot_filtered = axis1_handle_filter.axis1_filtered;
							catheter_mode_button_pressed_prev = (handle_axis1.buttons2 & cfg.btn_b7) != 0;
							std::cout << "导管手柄 (SN " << serial_axis1_handle << ") 取得首帧有效采样，基准已重新建立。" << std::endl;
						}
					}
					else
					{
						handle_axis1.close();
						handle1_next_retry_ms = loop_now_ms + 1000;
						std::cout << "导管手柄 (SN " << serial_axis1_handle << ") 轮询失败并已断开。" << std::endl;
					}
				}
				if (handle_axis6.is_open())
				{
					if (handle_axis6.poll())
					{
						handle6_poll_ok = true;
						if (handle6_reconnect_pending_poll)
						{
							handle6_reconnect_pending_poll = false;
							axis6_handle_filter.reset(handle_axis6.fJoints2[0], handle_axis6.fJoints2[1]);
							axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;
							axis6_prev_rot_filtered = axis6_handle_filter.axis1_filtered;
							guidewire_mode_button_pressed_prev = (handle_axis6.buttons2 & cfg.btn_b7) != 0;
							std::cout << "导丝手柄 (SN " << serial_axis6_handle << ") 取得首帧有效采样，基准已重新建立。" << std::endl;
						}
					}
					else
					{
						handle_axis6.close();
						handle6_next_retry_ms = loop_now_ms + 1000;
						std::cout << "导丝手柄 (SN " << serial_axis6_handle << ") 轮询失败并已断开。" << std::endl;
					}
				}
			}
		}

		handle_soft_hold_active = single_handle_mode
			? (!handle1_poll_ok && !handle6_poll_ok)
			: (!handle1_poll_ok || !handle6_poll_ok);

		if (!ads_motion_cycle_valid)
		{
			if (!ads_soft_hold_active)
			{
				ads_soft_hold_active = true;
				clear_force_output();
				std::cout << "ADS 快照中断：保持最后参考并丢弃故障期间手柄增量。" << std::endl;
			}
		}
		else if (ads_soft_hold_active && !planned_return.active())
		{
			if (!handle_soft_hold_active)
			{
				axis1_handle_filter.reset(axis1_input_handle->fJoints2[0], axis1_input_handle->fJoints2[1]);
				axis6_handle_filter.reset(axis6_input_handle->fJoints2[0], axis6_input_handle->fJoints2[1]);
				axis1_prev_linear_filtered = axis1_handle_filter.axis0_filtered;
				axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;
				axis1_prev_rot_filtered = axis1_handle_filter.axis1_filtered;
				axis6_prev_rot_filtered = axis6_handle_filter.axis1_filtered;
			}
			load_pos_from_actual();
			bool baseline_rebuilt = true;
			if (!plc_restart_recovery_latched && !handle_soft_hold_active)
			{
				if (guidewire_mode == GuidewireMode::Cooperative)
				{
					baseline_rebuilt = sync_cooperative_guidewire(1, false);
				}
				else if (guidewire_mode == GuidewireMode::Independent)
				{
					baseline_rebuilt = sync_axis6(1, false, false);
				}
				else
				{
					baseline_rebuilt = sync_axis1(1);
				}
			}
			ads_soft_hold_active = !baseline_rebuilt;
			if (baseline_rebuilt)
			{
				axis1_fast_return = false;
				axis6_fast_retract = false;
				std::cout << "ADS 快照已恢复：实际位置、手柄基准与当前稳定模式已重建。" << std::endl;
			}
		}

		const bool connection_hold_active = !initial_sync_done || ads_soft_hold_active || handle_soft_hold_active;
		const bool connection_hold_enter_edge = connection_hold_active && !connection_hold_active_prev;
		const bool connection_hold_exit_edge = !connection_hold_active && connection_hold_active_prev;
		connection_hold_active_prev = connection_hold_active;

		if (connection_hold_enter_edge)
		{
			if (planned_return.active())
			{
				(void)cancel_active_return_motion(true);
			}
			tracking_controller.disable_compensation();
			spacing_recovery.reset();
			if (ft_exp.active()) ft_exp.abort(ctx, "connection hold");
			clear_force_output();
			std::cout << "连接保持激活：已中断回退与瞬态任务，保持最后参考位并清零力反馈。" << std::endl;
		}
		if (connection_hold_exit_edge)
		{
			load_pos_from_actual();
			axis1_fast_return = false;
			axis6_fast_retract = false;
			std::cout << "连接保持解除：实际位置已重新加载并恢复控制。" << std::endl;
		}

		if (!initial_sync_done)
		{
			if (ads_motion_cycle_valid && !handle_soft_hold_active && handle_startup_locked)
			{
				(void)read_plc_state();
				if (sync_all(30))
				{
					initial_sync_done = true;
					control_active = false;
					startup.phase = StartupPhase::WaitForEnter;
					startup.completed = false;
					startup.prompted = false;
					prompt_startup_mode();
					std::cout << "系统初始同步完成，进入就绪状态。" << std::endl;
				}
			}
		}

		if (ads_motion_cycle_valid && !plc_app_name_read)
		{
			char app_name_buf[64] = { 0 };
			if (ads_communication.read(AdsSymbol::app_name, sizeof(app_name_buf), app_name_buf, 50))
			{
				app_name_buf[sizeof(app_name_buf) - 1] = '\0';
				std::cout << "ADS 目标 PLC 应用: " << app_name_buf << std::endl;
				plc_app_name_read = true;
			}
		}

		if (ads_stats.plc_restart_count != handled_plc_restart_count)
		{
			handled_plc_restart_count = ads_stats.plc_restart_count;
			plc_restart_recovery_latched = true;
			cal_state.zeroed = false;
			ff.enabled = false;
			tracking_controller.disable_compensation();
			spacing_recovery.reset();
			if (ft_exp.active()) ft_exp.abort(ctx, "PLC restart");
			cancel_cooperative_delivery(true);
			control_active = false;
			startup.completed = false;
			startup.phase = StartupPhase::WaitForEnter;
			clear_force_output();
			std::cout << "检测到 PLC 重启或应用重载：已清除力零点并锁定控制，必须重新自检、调零和人工启动。" << std::endl;
		}
		// 1) 采样逻辑手柄输入，并生成按键边沿触发状态。
		if (!handle_soft_hold_active)
		{
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
		}
		LARGE_INTEGER handle_record_qpc{};
		QueryPerformanceCounter(&handle_record_qpc);
		HandleRecordSnapshot handle_record{};
		handle_record.qpc_ticks = handle_record_qpc.QuadPart;
		handle_record.axis1_valid = axis1_input_handle != nullptr;
		handle_record.axis6_valid = axis6_input_handle != nullptr;
		handle_record.axis1_linear_raw = axis1_input_handle->fJoints2[0];
		handle_record.axis1_linear_filtered = axis1_handle_filter.axis0_filtered;
		handle_record.axis1_rotation_raw = axis1_input_handle->fJoints2[1];
		handle_record.axis1_rotation_filtered = axis1_handle_filter.axis1_filtered;
		handle_record.axis6_linear_raw = axis6_input_handle->fJoints2[0];
		handle_record.axis6_linear_filtered = axis6_handle_filter.axis0_filtered;
		handle_record.axis6_rotation_raw = axis6_input_handle->fJoints2[1];
		handle_record.axis6_rotation_filtered = axis6_handle_filter.axis1_filtered;
		handle_record_history.push_back(handle_record);
		const std::int64_t handle_history_ticks = std::max<std::int64_t>(1, vis_qpc_frequency.QuadPart / 10);
		while (!handle_record_history.empty() &&
			handle_record.qpc_ticks - handle_record_history.front().qpc_ticks > handle_history_ticks)
		{
			handle_record_history.pop_front();
		}

		if (planned_return.phase == PlannedReturnPhase::CancelWait &&
			!planned_return.cancel_rebased && planned_return.cancel_clear_confirmed &&
			ads_motion_cycle_valid && ads_snapshot_sequence > planned_return.cancel_snapshot_sequence)
		{
			bool plc_return_idle = true;
			for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
			{
				const PlannedReturnLeg& leg = planned_return.legs[leg_index];
				if (!leg.active) continue;
				if (leg.axis_index == 0)
				{
					plc_return_idle = plc_return_idle &&
						!ads_events.axis1_return_busy && !ads_events.axis1_return_error &&
						(!leg.cancel_event_required ||
							ads_events.axis1_return_event_sequence > leg.cancel_event_sequence);
				}
				else if (leg.axis_index == 5)
				{
					plc_return_idle = plc_return_idle &&
						!ads_events.axis6_return_busy && !ads_events.axis6_return_error &&
						(!leg.cancel_event_required ||
							ads_events.axis6_return_event_sequence > leg.cancel_event_sequence);
				}
			}
			if (plc_return_idle)
			{
				// 只使用已经发布到内存的同一份新鲜快照和本拍滤波手柄值；
				// 不轮询外设、不Sleep，也不在取消链中追加同步ADS读取。
				bool rebased = false;
				switch (planned_return.rebase_scope)
				{
				case PlannedReturnRebaseScope::Axis1:
					rebased = motion_sync::rebase_axis1_after_return(ctx);
					break;
				case PlannedReturnRebaseScope::Axis6:
					rebased = motion_sync::rebase_axis6_after_return(ctx);
					break;
				case PlannedReturnRebaseScope::Cooperative:
					rebased = motion_sync::rebase_cooperative_after_return(ctx);
					break;
				}
				if (!rebased)
				{
					load_pos_from_actual();
					return_ads_fault_hold = true;
					control_active = false;
					clear_force_output();
					std::cout << "计划回退取消后基准重建失败，继续保持上位机运动控制锁止。"
						<< std::endl;
				}
				planned_return.cancel_rebased = true;
				planned_return.cancel_hold_generation = 0;
			}
		}
		if (planned_return.phase == PlannedReturnPhase::CancelWait &&
			planned_return.cancel_rebased && planned_return.cancel_hold_generation != 0 &&
			ads_communication.applied_motion_output_generation() >=
			planned_return.cancel_hold_generation)
		{
			const bool fault_hold_remains = return_ads_fault_hold;
			reset_planned_return();
			if (fault_hold_remains)
			{
				std::cout << "实际位置保持已写入；回退故障保持继续生效，需人工重新启动。"
					<< std::endl;
			}
		}

		++loop_count;
		axis1_fast_return = false; // 每周期先清零，仅在快退状态机阶段置 TRUE
		axis6_fast_retract = false; // 每周期先清零，仅在快退状态机阶段置 TRUE
		// 预测目标造成的阻断只覆盖当前危险动作；下一拍重新读取位置和手柄输入。
		// 实际已经越过 670 mm 时保留冻结，直到实际位置回到安全侧。
		if (axis6_soft_limit_hold &&
			axis6_soft_limit_reason != Axis6SoftLimitReason::ActualPosition)
		{
			axis6_soft_limit_hold = false;
			axis6_soft_limit_reason = Axis6SoftLimitReason::None;
		}
		if (return_ads_fault_hold)
		{
			control_active = false;
			clear_force_output();
		}

		const unsigned char axis1_buttons = axis1_input_handle->buttons2;
		const unsigned char axis6_buttons = axis6_input_handle->buttons2;
		const bool pause_pressed = (axis1_buttons & axis1_pause_button_mask) != 0;
		const bool catheter_b6_pressed = (axis1_buttons & cfg.btn_b6) != 0;
		const bool guidewire_b6_pressed = (axis6_buttons & cfg.btn_b6) != 0;
		const bool catheter_b7_pressed = (axis1_buttons & cfg.btn_b7) != 0;
		const bool guidewire_b7_pressed = (axis6_buttons & cfg.btn_b7) != 0;
		const bool catheter_b7_press_edge =
			catheter_b7_pressed && !catheter_mode_button_pressed_prev;
		const bool guidewire_b7_press_edge =
			guidewire_b7_pressed && !guidewire_mode_button_pressed_prev;

		// 屈曲恢复正常退出并完成重同步后，再落实恢复期间点击的目标模式。
		if (pending_mode_selection != ModeSelection::None &&
			!ads_soft_hold_active && !spacing_recovery.active() && !spacing_recovery.requested)
		{
			const ModeSelection selection = pending_mode_selection;
			const PhysicalModeSource mode_source = pending_physical_mode_source;
			pending_mode_selection = ModeSelection::None;
			pending_physical_mode_source = PhysicalModeSource::None;
			apply_mode_selection(selection, mode_source);
			std::cout << "屈曲恢复退出后的目标模式切换请求已生效。" << std::endl;
		}

		const bool physical_mode_switch_allowed =
			!connection_hold_active &&
			!return_ads_fault_hold &&
			!single_handle_mode &&
			!freeze_active &&
			!estop_hold_active &&
			startup.completed &&
			startup.phase == StartupPhase::Done &&
			control_active &&
			!spacing_recovery.active() &&
			!spacing_recovery.requested &&
			!ft_exp.active() &&
			guidewire_mode != GuidewireMode::Cooperative &&
			cooperative_direction_requested == CooperativeDirection::None &&
			!planned_return.active();
		const bool physical_b7_conflict =
			(catheter_b7_pressed && guidewire_b7_pressed) &&
			(catheter_b7_press_edge || guidewire_b7_press_edge);
		if (physical_mode_switch_allowed && physical_b7_conflict)
		{
			std::cout << "物理模式切换冲突：SN 587 与 SN 582 的 B7 同时按下，本次切换已忽略。"
				<< std::endl;
			++physical_button_event_counter;
			physical_button_event_code = 7;
		}
		else if (physical_mode_switch_allowed && catheter_b7_press_edge)
		{
			const ModeSelection selection = catheter_b6_pressed
				? ModeSelection::CatheterDelivery : ModeSelection::CatheterRetraction;
			physical_mode_source_before_request = physical_mode_source;
			vis_reverse_override_active_before_request = vis_reverse_override_active;
			vis_reverse_override_value_before_request = vis_reverse_override_value;
			vis_reverse_override_target_before_request = vis_reverse_override_target;
			physical_mode_request_pending = guidewire_mode != GuidewireMode::None;
			request_mode_selection(selection, "物理 SN 587 B7 按下沿", PhysicalModeSource::Catheter);
			++physical_button_event_counter;
			physical_button_event_code = static_cast<int>(selection);
		}
		else if (physical_mode_switch_allowed && guidewire_b7_press_edge)
		{
			const ModeSelection selection = guidewire_b6_pressed
				? ModeSelection::GuidewireDelivery : ModeSelection::GuidewireRetraction;
			physical_mode_source_before_request = physical_mode_source;
			vis_reverse_override_active_before_request = vis_reverse_override_active;
			vis_reverse_override_value_before_request = vis_reverse_override_value;
			vis_reverse_override_target_before_request = vis_reverse_override_target;
			physical_mode_request_pending = guidewire_mode != GuidewireMode::Independent;
			request_mode_selection(selection, "物理 SN 582 B7 按下沿", PhysicalModeSource::Guidewire);
			++physical_button_event_counter;
			physical_button_event_code = static_cast<int>(selection);
		}
		// B7 松开不触发动作，持续按住也不会重复触发。
		catheter_mode_button_pressed_prev = catheter_b7_pressed;
		guidewire_mode_button_pressed_prev = guidewire_b7_pressed;

		// 只有实际进入协同模式后才固定为其选定方向。入口请求尚未通过门控时，
		// 必须继续沿用当前模式的方向，避免被拒绝的请求造成单拍反向跳变。
		bool cooperative_mode_active =
			guidewire_mode == GuidewireMode::Cooperative &&
			cooperative_direction != CooperativeDirection::None;
		bool cooperative_retraction_active =
			cooperative_mode_active && cooperative_direction == CooperativeDirection::Retraction;
		bool axis1_reverse_pressed = cooperative_mode_active
			? cooperative_retraction_active
			: (physical_mode_source == PhysicalModeSource::Catheter
				? !catheter_b6_pressed
				: ((vis_reverse_override_active && vis_reverse_override_target == 0)
					? vis_reverse_override_value
					: false));
		// B7选中的物理手柄作为当前模式源；UI/键盘选择会清除该物理模式源。
		GuidewireMode requested_guidewire_mode = GuidewireMode::None;
		if (cooperative_direction_requested != CooperativeDirection::None && dual_handle_ready)
		{
			// 协同方向由 UI 显式进入，期间忽略导丝手柄的模式/方向按键。
			requested_guidewire_mode = GuidewireMode::Cooperative;
		}
		else if (physical_mode_source == PhysicalModeSource::Guidewire)
		{
			requested_guidewire_mode = GuidewireMode::Independent;
		}
		else if (physical_mode_source == PhysicalModeSource::Catheter)
		{
			requested_guidewire_mode = GuidewireMode::None;
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
		if (spacing_recovery.active())
		{
			// 恢复模式接管轴3/5/6期间忽略物理导丝模式按键。
			physical_mode_source = PhysicalModeSource::None;
			requested_guidewire_mode = GuidewireMode::None;
			cancel_cooperative_delivery(true);
		}
		// 物理模式源下 B6 按当前电平决定方向；没有物理模式源时沿用UI/单手柄方向。
		bool axis6_effective_reverse_pressed = cooperative_mode_active
			? cooperative_retraction_active
			: (physical_mode_source == PhysicalModeSource::Guidewire
				? !guidewire_b6_pressed
				: ((vis_reverse_override_active && vis_reverse_override_target == 1)
					? vis_reverse_override_value
					: (single_handle_mode ? axis1_reverse_pressed : false)));
		const bool startup_sequence_active = startup.is_active();
		// 该映射只属于一次“导管正向递送回退完成后的前 10 mm”过程。
		// 只要离开普通导管正向 Follow，立即清除，避免撤出/暂停/保持后再次接管时
		// 把旧的附加量带入新的运动段。
		if (guidewire_mode != GuidewireMode::None ||
			cooperative_mode_active ||
			axis1_reverse_pressed ||
			cfg.axis1_post_return_lead_mm <= 1e-6 ||
			!control_active ||
			connection_hold_active ||
			ads_soft_hold_active ||
			return_ads_fault_hold ||
			freeze_active ||
			estop_hold_active ||
			startup_sequence_active ||
			spacing_recovery.active() ||
			spacing_recovery.requested ||
			ft_exp.active() ||
			planned_return.active() ||
			axis6_soft_limit_hold)
		{
			clear_axis1_delivery_mapping();
		}
		// 统一回退任务不能穿越暂停、急停、通信保持或其它接管状态。
		if (planned_return.active() && planned_return.phase != PlannedReturnPhase::CancelWait &&
			(!control_active || connection_hold_active ||
			freeze_active || estop_hold_active || return_ads_fault_hold ||
			spacing_recovery.active() || spacing_recovery.requested || ft_exp.active() ||
			startup_sequence_active || axis6_soft_limit_hold))
		{
			(void)cancel_active_return_motion(true);
		}
		// 正式控制阶段：启动流程已完成，B6 不再承担暂停/电缸5语义，而只作为所选手柄的方向电平。
		const bool formal_control_stage = startup.completed && (startup.phase == StartupPhase::Done);
		if ((axis4_ui_forward_pressed || axis4_ui_reverse_pressed) &&
			axis4_ui_jog_deadline_ms != 0 &&
			GetTickCount64() >= axis4_ui_jog_deadline_ms)
		{
			axis4_ui_forward_pressed = false;
			axis4_ui_reverse_pressed = false;
			axis4_ui_jog_deadline_ms = 0;
		}
		for (int injector_index = 0; injector_index < 2; ++injector_index)
		{
			if (injector_ui_direction[injector_index] != 0 &&
				injector_ui_jog_deadline_ms[injector_index] != 0 &&
				GetTickCount64() >= injector_ui_jog_deadline_ms[injector_index])
			{
				injector_ui_direction[injector_index] = 0;
				injector_ui_jog_deadline_ms[injector_index] = 0;
			}
		}
		const bool axis4_jog_allowed = ads_motion_cycle_valid && !freeze_active && !estop_hold_active && !startup_sequence_active &&
			!spacing_recovery.active() && !spacing_recovery.requested;
		// 轴4只保留UI点动，物理手柄按键不再映射到轴4。
		const bool axis4_forward_semantic = axis4_ui_forward_pressed;
		const bool axis4_reverse_semantic = axis4_ui_reverse_pressed;
		const bool axis4_direction_conflict = axis4_forward_semantic && axis4_reverse_semantic;
		const bool axis4_forward_request = axis4_jog_allowed && axis4_forward_semantic && !axis4_direction_conflict;
		const bool axis4_reverse_request = axis4_jog_allowed && axis4_reverse_semantic && !axis4_direction_conflict;
		bool injector_push_request[2] = {};
		bool injector_pull_request[2] = {};
		for (int injector_index = 0; injector_index < 2; ++injector_index)
		{
			injector_push_request[injector_index] =
				axis4_jog_allowed && injector_ui_direction[injector_index] > 0;
			injector_pull_request[injector_index] =
				axis4_jog_allowed && injector_ui_direction[injector_index] < 0;
		}

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
				// 模式切换不应把切换前遗留的低通滤波增量当成新手柄输入。
				axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;
				axis6_prev_rot_filtered = axis6_handle_filter.axis1_filtered;
				axis6_prev_abs_for_trigger = axis6_abs_now_for_guard;
				axis6_prev_abs_valid = true;
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

		// 3) 保持状态由 ADS Notification 更新，只在状态边沿执行安全处理。
		if (estop_hold_req != estop_hold_active)
		{
			if (estop_hold_req)
			{
				std::cout << "PLC 保持：开启。" << std::endl;
				estop_hold_active = true;
				control_active = false;
				cancel_cooperative_delivery(true);
				clear_force_output();
			}
			else
			{
				std::cout << "PLC 保持：关闭。" << std::endl;
				if (formal_control_stage)
				{
					axis1_push_rearm_after_hold = true;
					std::cout << "轴1推送已锁定，请先反向回拉手柄完成重接管。" << std::endl;
				}
				estop_hold_active = false;
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
			pending_mode_selection = ModeSelection::None;
			pending_physical_mode_source = PhysicalModeSource::None;
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
					else if (ads_soft_hold_active)
					{
						std::cout << "直接控制启动已忽略：ADS 当前处于软保持或重连状态。" << std::endl;
					}
					else if (has_self_check_flag && !self_check_done)
					{
						std::cout << "直接控制启动已忽略：PLC 自检尚未完成。" << std::endl;
					}
					else if (plc_restart_recovery_latched && !cal_state.zeroed)
					{
						std::cout << "直接控制启动已忽略：PLC 重启后必须先重新力感调零。" << std::endl;
					}
					else if (!restore_startup_v_limit())
					{
						std::cout << "直接控制启动失败：无法恢复启动期速度限制参数。" << std::endl;
					}
					else if (!consume_startup_loading_ready())
					{
						std::cout << "直接控制启动失败：无法清除 PLC 装卸位就绪标志。" << std::endl;
					}
					else if (sync_all(20))
					{
						clear_cylinder_manual_overrides();
						startup.recovery_mode = false;
						startup.phase = StartupPhase::Done;
						startup.completed = true;
						startup.prompted = false;
						control_active = true;
						plc_restart_recovery_latched = false;
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
					else if (ads_soft_hold_active)
					{
						std::cout << "启动准备已忽略：ADS 当前处于软保持或重连状态。" << std::endl;
					}
					else if (has_self_check_flag && !self_check_done)
					{
						std::cout << "启动准备已忽略：PLC 自检尚未完成。" << std::endl;
					}
					else if (plc_restart_recovery_latched && !cal_state.zeroed)
					{
						std::cout << "启动准备已忽略：PLC 重启后必须先重新力感调零。" << std::endl;
					}
					else if (start_startup_sequence())
					{
						clear_cylinder_manual_overrides();
						control_active = false;
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
				if (planned_return.active())
				{
					std::cout << "单手柄模式切换已忽略：当前计划换手尚未完成。" << std::endl;
				}
				else
				{
					physical_mode_source = PhysicalModeSource::None;
					single_handle_requested_mode = GuidewireMode::None;
					std::cout << "单手柄模式：已切换到导管语义。" << std::endl;
					std::cout << "按键说明：数字2切导丝；双手柄时可用对应手柄 B7 选择模式。" << std::endl;
				}
			}
			else if (single_handle_mode && ch == '2')
			{
				if (planned_return.active())
				{
					std::cout << "单手柄模式切换已忽略：当前计划换手尚未完成。" << std::endl;
				}
				else
				{
					physical_mode_source = PhysicalModeSource::None;
					single_handle_requested_mode = GuidewireMode::Independent;
					std::cout << "单手柄模式：已切换到导丝语义。" << std::endl;
					std::cout << "按键说明：数字1回导管；双手柄时可用对应手柄 B7 选择模式。" << std::endl;
				}
			}
			else if (ch == 'q' || ch == 'Q')
			{
				if (!planned_return.active())
				{
					cylinder_manual_mode[0] = cylinder_manual_mode[0] == CylinderManualMode::Open
						? CylinderManualMode::Automatic : CylinderManualMode::Open;
					std::cout << "电缸1 手动开覆盖：" << (cylinder_manual_mode[0] == CylinderManualMode::Open ? "开启" : "关闭") << std::endl;
				}
			}
			else if (ch == 'w' || ch == 'W')
			{
				if (!planned_return.active())
				{
					cylinder_manual_mode[1] = cylinder_manual_mode[1] == CylinderManualMode::Open
						? CylinderManualMode::Automatic : CylinderManualMode::Open;
					std::cout << "电缸2 手动开覆盖：" << (cylinder_manual_mode[1] == CylinderManualMode::Open ? "开启" : "关闭") << std::endl;
				}
			}
			else if (ch == 'e' || ch == 'E')
			{
				if (!planned_return.active())
				{
					cylinder_manual_mode[2] = cylinder_manual_mode[2] == CylinderManualMode::Open
						? CylinderManualMode::Automatic : CylinderManualMode::Open;
					std::cout << "电缸3 手动开覆盖：" << (cylinder_manual_mode[2] == CylinderManualMode::Open ? "开启" : "关闭") << std::endl;
				}
			}
			else if (ch == 'r' || ch == 'R')
			{
				if (!planned_return.active())
				{
					cylinder_manual_mode[3] = cylinder_manual_mode[3] == CylinderManualMode::Open
						? CylinderManualMode::Automatic : CylinderManualMode::Open;
					std::cout << "电缸4 手动开覆盖：" << (cylinder_manual_mode[3] == CylinderManualMode::Open ? "开启" : "关闭") << std::endl;
				}
			}
			else if (ch == 0 || ch == 224)
			{
				_getch();
			}
		}

		// 5) 导丝模式切换：双手柄时由对应手柄 B7 进入，协同模式仅由 UI 显式请求。
		bool cooperative_transition_failed = false;
		bool physical_mode_transition_rejected = false;
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
							std::cout << "导丝模式：关闭。" << std::endl;
						}
						else
						{
							if (!physical_mode_request_pending)
							{
								guidewire_mode = GuidewireMode::None;
								axis6_crawl.enabled = false;
								axis6_window_locked = false;
								axis6_coop_ff_inited = false;
								axis6_coop_prev_axis1_cmd_abs = 0.0;
							}
							// B7 请求在退出导丝时的 ADS 重同步失败，必须进入统一失败回滚；
							// 否则物理模式源已切到导管，但请求会永久挂起。
							physical_mode_transition_rejected = physical_mode_request_pending;
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
				// 从协同模式退出后，本拍立即采用普通导管最终方向。
				// 退出同步已经消费当前手柄采样；下一拍仍会按方向变化建立触发保护。
				cooperative_mode_active = false;
				cooperative_retraction_active = false;
				axis1_reverse_pressed =
					(vis_reverse_override_active && vis_reverse_override_target == 0)
					? vis_reverse_override_value
					: false;
			}
			else if (freeze_active)
			{
				std::cout << "导丝模式切换已忽略：582 暂停处于开启状态。" << std::endl;
				cooperative_transition_failed = (requested_guidewire_mode == GuidewireMode::Cooperative);
				physical_mode_transition_rejected = physical_mode_request_pending;
			}
			else if (estop_hold_active)
			{
				std::cout << "导丝模式切换已忽略：PLC 保持处于开启状态。" << std::endl;
				cooperative_transition_failed = (requested_guidewire_mode == GuidewireMode::Cooperative);
				physical_mode_transition_rejected = physical_mode_request_pending;
			}
			else if (!startup.completed || startup.phase != StartupPhase::Done)
			{
				std::cout << "导丝模式切换已忽略：启动准备尚未完成。" << std::endl;
				cooperative_transition_failed = (requested_guidewire_mode == GuidewireMode::Cooperative);
				physical_mode_transition_rejected = physical_mode_request_pending;
			}
			else if (!control_active || return_ads_fault_hold)
			{
				std::cout << "导丝模式切换已忽略：控制尚未激活。" << std::endl;
				cooperative_transition_failed = (requested_guidewire_mode == GuidewireMode::Cooperative);
				physical_mode_transition_rejected = physical_mode_request_pending;
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
							clear_cylinder_manual_overrides();
							guidewire_mode = GuidewireMode::Independent;
							cooperative_direction = CooperativeDirection::None;
							std::cout << (axis6_effective_reverse_pressed ? "导丝模式：反向取出。" : "导丝模式：正向输送。") << std::endl;
						}
					}
					else if (requested_guidewire_mode == GuidewireMode::Cooperative)
					{
						mode_ok = enter_cooperative_guidewire_mode();
						if (mode_ok)
						{
							clear_cylinder_manual_overrides();
							guidewire_mode = GuidewireMode::Cooperative;
							// 本拍在切换前已经采样过物理按键；成功进入后立即覆盖为
							// 固定协同方向，避免首次控制带入旧模式方向。
							cooperative_direction = cooperative_direction_requested;
							cooperative_mode_active = true;
							cooperative_retraction_active =
								cooperative_direction == CooperativeDirection::Retraction;
							axis1_reverse_pressed = cooperative_retraction_active;
							axis6_effective_reverse_pressed = cooperative_retraction_active;
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
				if ((!mode_attempted && mode_rejected) || (mode_attempted && !mode_ok))
				{
					physical_mode_transition_rejected = physical_mode_request_pending;
				}
				if (cooperative_request && !mode_ok)
				{
					// 入口失败后取消 UI 请求，下一帧按此前的普通 UI/物理模式恢复，
					// 防止因持续重试影响原模式。
					cooperative_direction_requested = CooperativeDirection::None;
					cooperative_direction = CooperativeDirection::None;
					cooperative_transition_failed = true;
				}
			}
		}
		if (physical_mode_request_pending)
		{
			const bool physical_mode_request_succeeded =
				(requested_guidewire_mode == GuidewireMode::None && guidewire_mode == GuidewireMode::None) ||
				(requested_guidewire_mode == GuidewireMode::Independent && guidewire_mode == GuidewireMode::Independent);
			if (physical_mode_request_succeeded)
			{
				physical_mode_request_pending = false;
			}
			else if (physical_mode_transition_rejected)
			{
				// B7 请求被门控或重同步拒绝时恢复原模式来源，避免 B6 突然解释为另一只手柄。
				physical_mode_source = physical_mode_source_before_request;
				vis_reverse_override_active = vis_reverse_override_active_before_request;
				vis_reverse_override_value = vis_reverse_override_value_before_request;
				vis_reverse_override_target = vis_reverse_override_target_before_request;
				physical_mode_request_pending = false;
				requested_guidewire_mode_prev = guidewire_mode;
				std::cout << "物理 B7 模式切换未通过，已保留原模式来源。" << std::endl;
			}
		}
		requested_guidewire_mode_prev = (cooperative_transition_failed || physical_mode_transition_rejected)
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

		// 6) 自检和重同步请求均由 Notification 驱动，不再做周期单读。
		if (has_self_check_flag && self_check_done != last_self_check_done)
		{
			if (last_self_check_done && !self_check_done)
			{
				cancel_cooperative_delivery(true);
				control_active = false;
				clear_force_output();
				std::cout << "PLC 自检重新开始，已退出协同递送。" << std::endl;
			}
			else if (!last_self_check_done && self_check_done)
			{
				const bool coordinates_refreshed = ads_communication.refresh_coordinates();
				if (!coordinates_refreshed)
				{
					ads_communication.request_coordinate_refresh();
					std::cout << "警告：PLC 自检完成后的坐标缓存同步失败，已排队重试；本拍不下发重同步参考。" << std::endl;
				}
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
				if (coordinates_refreshed && !freeze_active && !estop_hold_active &&
					!ads_soft_hold_active && sync_all(30))
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
		}
		last_self_check_done = self_check_done;

		if (handle_reinit_req && !last_handle_reinit_req)
		{
			spacing_recovery.reset();
			cancel_cooperative_delivery(true);
			const bool coordinates_refreshed = ads_communication.refresh_coordinates();
			if (!coordinates_refreshed) ads_communication.request_coordinate_refresh();
			if (coordinates_refreshed && !freeze_active && !estop_hold_active &&
				!ads_soft_hold_active && !startup.is_active() &&
				sync_all(30))
			{
				control_active = startup.completed && (startup.phase == StartupPhase::Done);
				if (!startup.completed && (!has_self_check_flag || self_check_done) && !startup.prompted)
				{
					prompt_startup_mode();
				}
			}
		}
		last_handle_reinit_req = handle_reinit_req;

		// 8) 当启动已完成但控制未激活时，通过全量重同步恢复。
		const bool motion_startup_active = startup.is_active();
		startup_smoothing_bypass = motion_startup_active;
		if (!control_active && !return_ads_fault_hold && !motion_startup_active &&
			!freeze_active && !estop_hold_active && !ads_soft_hold_active && startup.completed)
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
				 !ads_soft_hold_active &&
				 !startup.prompted &&
				 (!has_self_check_flag || self_check_done))
		{
			prompt_startup_mode();
		}
		unsigned short cylinder1_cmd = cyl.cyl1_open;
		unsigned short cylinder2_cmd = cyl.cyl2_clamp;
		unsigned short cylinder3_cmd = cyl.cyl3_follow_release;
		unsigned short cylinder4_cmd = cyl.cyl4_follow_release;
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
		if (!ads_soft_hold_active && !return_ads_fault_hold && !freeze_active && !estop_hold_active &&
			(control_active || motion_startup_active) && read_plc_state())
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
			const double axis5_abs = plc_act_pos[4] + plc_init_pos[4];
			const double axis1_linear_increment_raw_mm =
				(axis1_linear_filtered - axis1_prev_linear_filtered) * cfg.k_handle_to_mm * cfg.axis_push_sign;
			const double axis1_linear_increment_mm =
				(std::abs(axis1_linear_increment_raw_mm) >= cfg.linear_increment_noise_deadband_mm)
				? axis1_linear_increment_raw_mm
				: 0.0;
			const double axis1_directional_increment_mm =
				gate_linear_increment_for_mode(axis1_linear_increment_mm, axis1_reverse_pressed);
			const bool axis1_directional_increment_active =
				std::abs(axis1_directional_increment_mm) > 0.0;
			const double axis1_window_left_abs_now = axis1_crawl.start_abs;
			const double axis1_window_right_abs_now = axis1_crawl.end_abs;
			const double axis1_min_abs = axis1_crawl.min_abs();
			const double axis1_max_abs = axis1_crawl.max_abs();
			const double axis3_from_left_mm = axis3_abs - plc_leftlimit[2];
			const bool axis3_delivery_stop_active =
				axis3_from_left_mm <= (cfg.axis3_delivery_stop_from_left_mm + cfg.crawl_arrive_tol_mm);

			const double axis6_abs = plc_act_pos[5] + plc_init_pos[5]; // 轴6绝对位置(mm)
			if (axis6_soft_limit_reason == Axis6SoftLimitReason::ActualPosition &&
				!axis6_target_exceeds_soft_limit(axis6_abs))
			{
				// 实际位置已经回到限制内，允许后续正常状态机重新接管。
				axis6_soft_limit_hold = false;
				axis6_soft_limit_reason = Axis6SoftLimitReason::None;
			}
			if (axis6_target_exceeds_soft_limit(axis6_abs))
			{
				engage_axis6_soft_limit_hold(
					axis6_abs,
					Axis6SoftLimitReason::ActualPosition,
					"axis6 实际位置");
			}
			const double axis6_linear_increment_raw_mm =
				(axis6_linear_filtered - axis6_prev_linear_filtered) * cfg.k_handle_to_mm * cfg.axis_push_sign;
			const double axis6_linear_increment_mm =
				(std::abs(axis6_linear_increment_raw_mm) >= cfg.linear_increment_noise_deadband_mm)
				? axis6_linear_increment_raw_mm
				: 0.0;
			const double axis6_directional_increment_mm =
				gate_linear_increment_for_mode(axis6_linear_increment_mm, axis6_effective_reverse_pressed);
			const bool axis6_directional_increment_active =
				std::abs(axis6_directional_increment_mm) > 0.0;

			// 主从位移实验门控：只认可普通导管递送 axis1 与独立导丝递送 axis6。
			// 夹持成立仅是命令保持 150 ms 的实验假设，并不替代物理到位传感器。
			const bool tracking_no_return_active =
				!planned_return.active() && !axis6_soft_limit_hold;
			auto tracking_reason_for = [&](bool forward_mode) -> TrackingInvalidReason
			{
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
				!planned_return.active() &&
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
				planned_return.active() && planned_return.contains_axis(0);
			const bool axis6_handover_active = axis6_forward_tracking_mode &&
				planned_return.active() && planned_return.contains_axis(5);
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
			struct PlannedReturnEventView
			{
				bool busy = false;
				bool done = false;
				bool error = false;
				std::uint32_t error_id = 0;
				std::uint64_t event_sequence = 0;
				std::uint64_t busy_true_sequence = 0;
				std::uint64_t done_false_sequence = 0;
				std::uint64_t done_true_sequence = 0;
				std::uint64_t error_true_sequence = 0;
				std::uint32_t last_nonzero_error_id = 0;
				std::uint64_t last_nonzero_error_id_sequence = 0;
			};
			auto planned_return_event_view = [&](int axis_index) -> PlannedReturnEventView
			{
				PlannedReturnEventView view{};
				if (axis_index == 0)
				{
					view.busy = ads_events.axis1_return_busy;
					view.done = ads_events.axis1_return_done;
					view.error = ads_events.axis1_return_error;
					view.error_id = ads_events.axis1_return_error_id;
					view.event_sequence = ads_events.axis1_return_event_sequence;
					view.busy_true_sequence = ads_events.axis1_return_busy_true_sequence;
					view.done_false_sequence = ads_events.axis1_return_done_false_sequence;
					view.done_true_sequence = ads_events.axis1_return_done_true_sequence;
					view.error_true_sequence = ads_events.axis1_return_error_true_sequence;
					view.last_nonzero_error_id = ads_events.axis1_return_last_nonzero_error_id;
					view.last_nonzero_error_id_sequence =
						ads_events.axis1_return_last_nonzero_error_id_sequence;
				}
				else if (axis_index == 5)
				{
					view.busy = ads_events.axis6_return_busy;
					view.done = ads_events.axis6_return_done;
					view.error = ads_events.axis6_return_error;
					view.error_id = ads_events.axis6_return_error_id;
					view.event_sequence = ads_events.axis6_return_event_sequence;
					view.busy_true_sequence = ads_events.axis6_return_busy_true_sequence;
					view.done_false_sequence = ads_events.axis6_return_done_false_sequence;
					view.done_true_sequence = ads_events.axis6_return_done_true_sequence;
					view.error_true_sequence = ads_events.axis6_return_error_true_sequence;
					view.last_nonzero_error_id = ads_events.axis6_return_last_nonzero_error_id;
					view.last_nonzero_error_id_sequence =
						ads_events.axis6_return_last_nonzero_error_id_sequence;
				}
				return view;
			};

			auto refresh_planned_return_leg = [&](PlannedReturnLeg& leg)
			{
				// 初始化/prepare阶段的Done=FALSE属于历史初值，只有本次commit已捕获
				// 事件基线后，才允许把后续Busy/Done/Error边沿解释为本任务状态。
				if (!leg.active || !leg.request_armed) return;
				const PlannedReturnEventView events = planned_return_event_view(leg.axis_index);
				std::uint64_t first_ack_sequence = 0;
				if (events.busy_true_sequence > leg.request_event_sequence)
				{
					first_ack_sequence = events.busy_true_sequence;
				}
				if (events.done_false_sequence > leg.request_event_sequence &&
					(first_ack_sequence == 0 || events.done_false_sequence < first_ack_sequence))
				{
					first_ack_sequence = events.done_false_sequence;
				}
				if (!leg.acknowledged && first_ack_sequence != 0)
				{
					leg.acknowledged = true;
					leg.started = true;
					leg.ack_event_sequence = first_ack_sequence;
					if (planned_return.execution_t0_ms == 0)
					{
						planned_return.execution_t0_ms = now_ms;
					}
				}

				if (events.error_true_sequence > leg.request_event_sequence)
				{
					leg.error_event_sequence = events.error_true_sequence;
					if (events.last_nonzero_error_id_sequence > leg.request_event_sequence)
					{
						leg.reported_error_id = events.last_nonzero_error_id;
					}
					else
					{
						leg.reported_error_id = events.error_id;
					}
				}

				const bool completed_cycle = leg.acknowledged &&
					events.done_true_sequence > leg.ack_event_sequence &&
					events.done && !events.busy;
				if (completed_cycle)
				{
					leg.done = true;
				}
			};

			auto planned_return_any_started = [&]() -> bool
			{
				for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
				{
					const PlannedReturnLeg& leg = planned_return.legs[leg_index];
					if (leg.started || leg.possibly_started) return true;
				}
				return false;
			};

			auto planned_return_all_acknowledged = [&]() -> bool
			{
				if (planned_return.leg_count <= 0) return false;
				for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
				{
					if (!planned_return.legs[leg_index].acknowledged) return false;
				}
				return true;
			};

			auto planned_return_all_done = [&]() -> bool
			{
				if (planned_return.leg_count <= 0) return false;
				for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
				{
					if (!planned_return.legs[leg_index].done) return false;
				}
				return true;
			};

			auto planned_return_has_error = [&]() -> const PlannedReturnLeg*
			{
				for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
				{
					const PlannedReturnLeg& leg = planned_return.legs[leg_index];
					if (leg.error_event_sequence > leg.request_event_sequence) return &leg;
				}
				return nullptr;
			};

			auto planned_return_targets_safe = [&]() -> bool
			{
				for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
				{
					const PlannedReturnLeg& leg = planned_return.legs[leg_index];
					if (leg.axis_index == 0)
					{
						if (!is_within_range(
							leg.target_abs,
							axis1_crawl.min_abs(),
							axis1_crawl.max_abs(),
							cfg.crawl_arrive_tol_mm))
						{
							return false;
						}
					}
					else if (leg.axis_index == 5)
					{
						if (axis6_target_exceeds_soft_limit(leg.target_abs)) return false;
						double window_left_abs = axis6_crawl.min_abs();
						double window_right_abs = axis6_crawl.max_abs();
						if (planned_return.mode == PlannedReturnMode::CatheterDelivery ||
							planned_return.mode == PlannedReturnMode::CatheterRetraction ||
							planned_return.cooperative())
						{
							motion_sync::calculate_axis6_window_from_axis5_abs(
								ctx,
								axis5_abs,
								window_left_abs,
								window_right_abs);
						}
						if (!is_within_range(
							leg.target_abs,
							window_left_abs,
							window_right_abs,
							cfg.crawl_arrive_tol_mm))
						{
							return false;
						}
					}
				}
				return true;
			};

			auto rebase_planned_return = [&]() -> bool
			{
				switch (planned_return.rebase_scope)
				{
				case PlannedReturnRebaseScope::Axis1:
					return motion_sync::rebase_axis1_after_return(ctx);
				case PlannedReturnRebaseScope::Axis6:
					return motion_sync::rebase_axis6_after_return(ctx);
				case PlannedReturnRebaseScope::Cooperative:
					return motion_sync::rebase_cooperative_after_return(ctx);
				}
				return false;
			};

			auto fail_planned_return = [&](const char* reason,
				const PlannedReturnLeg* failed_leg,
				bool attempt_request_clear)
			{
				(void)attempt_request_clear;
				return_ads_fault_hold = true;
				control_active = false;
				clear_force_output();
				std::cout << reason;
				if (failed_leg != nullptr && failed_leg->reported_error_id != 0)
				{
					std::cout << "，错误码: " << failed_leg->reported_error_id;
					pending_return_error_axis = -1;
					pending_return_error_event_sequence = 0;
					pending_return_error_deadline_ms = 0;
				}
				else if (failed_leg != nullptr &&
					failed_leg->error_event_sequence > failed_leg->request_event_sequence)
				{
					pending_return_error_axis = failed_leg->axis_index;
					pending_return_error_event_sequence = failed_leg->error_event_sequence;
					pending_return_error_deadline_ms = GetTickCount64() + 2000;
					std::cout << "，错误码等待PLC后续Notification";
				}
				std::cout << "。已停止上位机运动控制，并异步清Req、等待PLC受控停稳。"
					<< std::endl;
				// 即使本次故障来自清Req失败，也必须保留任务并继续按250 ms重试；
				// 不能先把本地状态伪装成Follow。
				(void)cancel_active_return_motion(true);
			};

		auto begin_planned_return = [&](PlannedReturnMode mode,
				PlannedReturnRebaseScope rebase_scope,
				bool axis1_active,
				double axis1_target_abs,
				bool axis6_active,
				double axis6_target_abs) -> bool
			{
				// 力过渡实验和统一换手都会占用 axis1 的 PLC 计划回退命令，
				// 两者必须双向互斥，不能只依赖本拍末尾的输出覆盖顺序。
				if (planned_return.active() || ft_exp.active()) return false;
				planned_return.reset();
				planned_return.mode = mode;
				planned_return.rebase_scope = rebase_scope;
				planned_return.phase = PlannedReturnPhase::ClampSettle;
				clear_axis1_delivery_mapping();
				planned_return.phase_t0_ms = 0;
				planned_return.hold_axis3_rel = plc_act_pos[2];
				planned_return.hold_axis5_rel = plc_act_pos[4];
				if (axis1_active)
				{
					PlannedReturnLeg& leg = planned_return.legs[planned_return.leg_count++];
					leg.active = true;
					leg.axis_index = 0;
					leg.target_abs = axis1_target_abs;
					leg.entry_rel = plc_act_pos[0];
				}
				if (axis6_active)
				{
					PlannedReturnLeg& leg = planned_return.legs[planned_return.leg_count++];
					leg.active = true;
					leg.axis_index = 5;
					leg.target_abs = axis6_target_abs;
					leg.entry_rel = plc_act_pos[5];
				}
				if (planned_return.leg_count <= 0 || !planned_return_targets_safe())
				{
					planned_return.reset();
					return false;
				}
				clear_cylinder_manual_overrides();
				return true;
			};

			auto planned_return_clamp_settle_ms = [&]() -> DWORD
			{
				DWORD clamp_wait_ms = 0;
				if (planned_return.contains_axis(0))
				{
					clamp_wait_ms = cfg.axis1_pre_move_cylinder_wait_ms;
				}
				if (planned_return.contains_axis(5))
				{
					clamp_wait_ms = (std::max)(clamp_wait_ms, cfg.axis6_pre_move_cylinder_wait_ms);
				}
				return clamp_wait_ms;
			};

			auto apply_planned_return_outputs = [&]()
			{
				const bool axis1_leg_active = planned_return.contains_axis(0);
				const bool axis6_leg_active = planned_return.contains_axis(5);
				const bool restore_clamps =
					planned_return.phase == PlannedReturnPhase::CancelWait ||
					planned_return.phase == PlannedReturnPhase::AwaitFreshSnapshot ||
					planned_return.phase == PlannedReturnPhase::PublishHandoff ||
					planned_return.phase == PlannedReturnPhase::AwaitHandoffApplied ||
					planned_return.phase == PlannedReturnPhase::PostHandoffClampSettle;
				const bool request_may_start =
					planned_return.phase == PlannedReturnPhase::AwaitAccepted ||
					planned_return.phase == PlannedReturnPhase::AwaitDone ||
					(planned_return.phase == PlannedReturnPhase::SubmitRequest &&
						planned_return.request_prepared) ||
					planned_return.ads_command_purpose == PlannedReturnAdsCommandPurpose::Commit;

				axis1_fast_return = false;
				axis6_fast_retract = false;
				if (axis1_leg_active)
				{
					const PlannedReturnLeg& leg = planned_return.legs[0];
					const bool move_active = leg.started && !restore_clamps;
					pos[0] = restore_clamps
						? plc_act_pos[0]
						: (move_active ? (leg.target_abs - plc_init_pos[0]) : leg.entry_rel);
					pos[1] = axis2_hold_rel;
					pos[2] = planned_return.hold_axis3_rel;
					pos[4] = planned_return.hold_axis5_rel;
					axis1_fast_return = move_active || request_may_start ||
						planned_return.phase == PlannedReturnPhase::AwaitFreshSnapshot;
					if (restore_clamps)
					{
						cylinder1_cmd = cyl.cyl1_open;
						cylinder2_cmd = cyl.cyl2_clamp;
					}
					else
					{
						cylinder1_cmd = cyl.cyl1_clamp;
						cylinder2_cmd = cyl.cyl2_open;
					}
				}

				if (axis6_leg_active)
				{
					const PlannedReturnLeg& leg = planned_return.legs[axis1_leg_active ? 1 : 0];
					const bool move_active = leg.started && !restore_clamps;
					pos[5] = restore_clamps
						? plc_act_pos[5]
						: (move_active ? (leg.target_abs - plc_init_pos[5]) : leg.entry_rel);
					pos[6] = axis7_hold_rel;
					axis6_fast_retract = move_active || request_may_start ||
						planned_return.phase == PlannedReturnPhase::AwaitFreshSnapshot;
					if (restore_clamps)
					{
						cylinder3_cmd = cyl.cyl3_open;
						cylinder4_cmd = cyl.cyl4_clamp;
					}
					else
					{
						cylinder3_cmd = cyl.cyl3_clamp;
						cylinder4_cmd = cyl.cyl4_open;
					}
				}

				if (!axis1_leg_active)
				{
					pos[0] = (guidewire_mode == GuidewireMode::Independent)
						? independent_axis1_hold_rel : plc_act_pos[0];
					pos[1] = (guidewire_mode == GuidewireMode::Independent)
						? independent_axis2_hold_rel : axis2_hold_rel;
					pos[2] = (guidewire_mode == GuidewireMode::Independent)
						? independent_axis3_hold_rel : plc_act_pos[2];
					pos[4] = (guidewire_mode == GuidewireMode::Independent)
						? independent_axis5_hold_rel : plc_act_pos[4];
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_clamp;
				}
				if (!axis6_leg_active)
				{
					pos[5] = plc_act_pos[5];
					pos[6] = axis7_hold_rel;
					cylinder3_cmd = cyl.cyl3_open;
					cylinder4_cmd = cyl.cyl4_clamp;
				}
			};

			auto enter_planned_return_retry = [&](const char* reason)
			{
				for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
				{
					refresh_planned_return_leg(planned_return.legs[leg_index]);
				}
				if (planned_return.retry_count >= 1 || planned_return_any_started())
				{
					fail_planned_return(reason, planned_return_has_error(), true);
					return;
				}
				planned_return.request_prepared = false;
				planned_return.commit_guard_generation = 0;
				axis1_fast_return = false;
				axis6_fast_retract = false;
				planned_return.phase = PlannedReturnPhase::RetryClear;
				planned_return.retry_clear_t0_ms = now_ms;
				planned_return.retry_clear_issued = false;
				planned_return.retry_clear_snapshot_sequence = ads_snapshot_sequence;
				std::cout << reason << "；PLC尚未启动动作，将清除请求并安全重试一次。"
					<< std::endl;
			};

			auto step_planned_return = [&]()
			{
				if (!planned_return.active()) return;
				for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
				{
					refresh_planned_return_leg(planned_return.legs[leg_index]);
				}
				apply_planned_return_outputs();
				if (planned_return.phase == PlannedReturnPhase::CancelWait)
				{
					return;
				}

				if (planned_return.phase == PlannedReturnPhase::ClampSettle)
				{
					const DWORD clamp_wait_ms = planned_return_clamp_settle_ms();
					if (planned_return.clamp_output_generation != 0 &&
						!planned_return.clamp_output_applied &&
						ads_communication.applied_output_generation() >=
						planned_return.clamp_output_generation)
					{
						planned_return.clamp_output_applied = true;
						planned_return.phase_t0_ms = now_ms;
					}
					if (planned_return.clamp_output_applied &&
						(now_ms - planned_return.phase_t0_ms) >= clamp_wait_ms)
					{
						planned_return.phase = PlannedReturnPhase::SubmitRequest;
					}
				}
				else if (planned_return.phase == PlannedReturnPhase::SubmitRequest)
				{
					bool existing_busy = false;
					for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
					{
						PlannedReturnLeg& leg = planned_return.legs[leg_index];
						const PlannedReturnEventView events = planned_return_event_view(leg.axis_index);
						if (events.busy)
						{
							leg.started = true;
							existing_busy = true;
						}
					}

					if (existing_busy &&
						planned_return.ads_command_purpose != PlannedReturnAdsCommandPurpose::Commit)
					{
						fail_planned_return(
							"计划回退提交前检测到PLC已有Busy，禁止覆盖正在执行的动作",
							nullptr,
							true);
					}
					else if (!planned_return.request_prepared)
					{
						if (planned_return.ads_command_purpose == PlannedReturnAdsCommandPurpose::None)
						{
							if (!submit_planned_return_ads_command(
								AdsPlannedReturnOperation::Prepare,
								PlannedReturnAdsCommandPurpose::Prepare,
								false))
							{
								enter_planned_return_retry("计划回退参数命令未能进入通信队列");
							}
						}
						else if (planned_return.ads_command_purpose == PlannedReturnAdsCommandPurpose::Prepare)
						{
							const PlannedReturnAdsPoll poll = poll_planned_return_ads_command();
							if (poll == PlannedReturnAdsPoll::Failure)
							{
								clear_planned_return_ads_command_tracking();
								enter_planned_return_retry("计划回退参数批量写入失败");
							}
							else if (poll == PlannedReturnAdsPoll::Success)
							{
								clear_planned_return_ads_command_tracking();
								planned_return.request_prepared = true;
								planned_return.commit_guard_generation = 0;
								planned_return.request_t0_ms = now_ms;
								apply_planned_return_outputs();
							}
							else if (planned_return_ads_command_timed_out(now_ms))
							{
								clear_planned_return_ads_command_tracking();
								enter_planned_return_retry("计划回退参数命令等待超过250 ms");
							}
						}
						else
						{
							clear_planned_return_ads_command_tracking();
							enter_planned_return_retry("计划回退参数阶段出现未知通信命令");
						}
					}
					else
					{
						if (planned_return.ads_command_purpose == PlannedReturnAdsCommandPurpose::Commit)
						{
							const PlannedReturnAdsPoll poll = poll_planned_return_ads_command();
							if (poll == PlannedReturnAdsPoll::Failure)
							{
								clear_planned_return_ads_command_tracking();
								planned_return.request_prepared = false;
								planned_return.commit_guard_generation = 0;
								enter_planned_return_retry("计划回退请求批量置位失败");
							}
							else if (poll == PlannedReturnAdsPoll::Success)
							{
								clear_planned_return_ads_command_tracking();
								planned_return.request_prepared = false;
								planned_return.commit_guard_generation = 0;
								planned_return.request_t0_ms = now_ms;
								planned_return.phase = PlannedReturnPhase::AwaitAccepted;
								apply_planned_return_outputs();
							}
							else if (planned_return_ads_command_timed_out(now_ms))
							{
								// Commit仍在队列/ADS调用中时无法证明Req未生效，禁止清理后整组重发。
								latch_planned_return_possibly_started(
									planned_return_active_axis_mask());
								clear_planned_return_ads_command_tracking();
								planned_return.request_prepared = false;
								planned_return.commit_guard_generation = 0;
								enter_planned_return_retry("计划回退请求命令等待超过250 ms");
							}
						}
						else if (planned_return.commit_guard_generation == 0 ||
							ads_communication.applied_output_generation() <
							planned_return.commit_guard_generation)
						{
							// 必须先确认包含快退旁路和冻结参考的输出代次已经写入，
							// 才允许Req=TRUE进入队列；因此通信线程不可能把Commit与旧输出配对。
							if ((now_ms - planned_return.request_t0_ms) >=
								cfg.planned_return_ack_timeout_ms)
							{
								planned_return.request_prepared = false;
								planned_return.commit_guard_generation = 0;
								enter_planned_return_retry("计划回退提交保护输出确认超时");
							}
						}
						else
						{
							bool idle_state_ready = true;
							for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
							{
								const PlannedReturnEventView events =
									planned_return_event_view(planned_return.legs[leg_index].axis_index);
								idle_state_ready = idle_state_ready && !events.busy &&
									!events.done && !events.error;
							}
							if (idle_state_ready)
							{
								for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
								{
									PlannedReturnLeg& leg = planned_return.legs[leg_index];
									const PlannedReturnEventView events = planned_return_event_view(leg.axis_index);
									leg.request_event_sequence = events.event_sequence;
									leg.error_event_sequence = events.event_sequence;
									// 命令一旦进入通信队列就可能在下一PLC扫描启动；先武装边沿检测。
									leg.request_armed = true;
								}
								if (!submit_planned_return_ads_command(
									AdsPlannedReturnOperation::Commit,
									PlannedReturnAdsCommandPurpose::Commit,
									false))
								{
									for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
									{
										planned_return.legs[leg_index].request_armed = false;
									}
									planned_return.request_prepared = false;
									enter_planned_return_retry("计划回退请求未能进入通信队列");
								}
								else
								{
									// commit等待通信线程完成期间立即拉起快退旁路/力反馈冻结，
									// refer仍保持各腿entry位置直到观察到本次ACK。
									apply_planned_return_outputs();
								}
							}
							else if ((now_ms - planned_return.request_t0_ms) >=
								cfg.planned_return_ack_timeout_ms)
							{
								planned_return.request_prepared = false;
								enter_planned_return_retry("计划回退提交前PLC状态清理超时");
							}
						}
					}
				}
				else if (planned_return.phase == PlannedReturnPhase::AwaitAccepted)
				{
					const PlannedReturnLeg* error_leg = planned_return_has_error();
					if (error_leg != nullptr)
					{
						enter_planned_return_retry("计划回退在PLC启动前报告错误");
					}
					else if (planned_return_all_acknowledged())
					{
						planned_return.phase = PlannedReturnPhase::AwaitDone;
					}
					else if ((now_ms - planned_return.request_t0_ms) >=
						cfg.planned_return_ack_timeout_ms)
					{
						enter_planned_return_retry("计划回退请求确认超时");
					}
				}
				else if (planned_return.phase == PlannedReturnPhase::AwaitDone)
				{
					const PlannedReturnLeg* error_leg = planned_return_has_error();
					if (error_leg != nullptr)
					{
						fail_planned_return("计划回退运动报错", error_leg, true);
					}
					else if (planned_return_all_done())
					{
						// Done事件可能落在当前ADS读快照之后，因此先记录序号并等待下一份
						// 有效100 Hz快照；随后把清Req和交接输出合并到同一次Sum Write。
						planned_return.done_snapshot_sequence = ads_snapshot_sequence;
						planned_return.completion_clear_confirmed = false;
						axis1_fast_return = false;
						axis6_fast_retract = false;
						planned_return.phase = PlannedReturnPhase::AwaitFreshSnapshot;
						apply_planned_return_outputs();
					}
					else if (planned_return.execution_t0_ms != 0 &&
						(now_ms - planned_return.execution_t0_ms) >=
						cfg.planned_return_execution_timeout_ms)
					{
						fail_planned_return("计划回退执行超时", nullptr, true);
					}
				}
				else if (planned_return.phase == PlannedReturnPhase::RetryClear)
				{
					if (planned_return_any_started())
					{
						fail_planned_return("清理待重试请求期间检测到PLC已经启动动作", nullptr, true);
					}
					else if (!planned_return.retry_clear_issued)
					{
						if (planned_return.ads_command_purpose == PlannedReturnAdsCommandPurpose::None)
						{
							if (!submit_planned_return_ads_command(
								AdsPlannedReturnOperation::Clear,
								PlannedReturnAdsCommandPurpose::RetryClear,
								false))
							{
								fail_planned_return("安全重试前清Req命令未能进入通信队列", nullptr, true);
							}
						}
						else if (planned_return.ads_command_purpose ==
							PlannedReturnAdsCommandPurpose::RetryClear)
						{
							const PlannedReturnAdsPoll poll = poll_planned_return_ads_command();
							if (poll == PlannedReturnAdsPoll::Failure)
							{
								clear_planned_return_ads_command_tracking();
								fail_planned_return("安全重试前清除PLC请求失败", nullptr, true);
							}
							else if (poll == PlannedReturnAdsPoll::Success)
							{
								clear_planned_return_ads_command_tracking();
								planned_return.retry_clear_issued = true;
								planned_return.retry_clear_t0_ms = now_ms;
								planned_return.retry_clear_snapshot_sequence = ads_snapshot_sequence;
							}
							else if (planned_return_ads_command_timed_out(now_ms))
							{
								clear_planned_return_ads_command_tracking();
								fail_planned_return("安全重试前清Req命令等待超过250 ms", nullptr, true);
							}
						}
					}
					else
					{
						bool reset_state_ready = ads_motion_cycle_valid &&
							ads_snapshot_sequence > planned_return.retry_clear_snapshot_sequence;
						for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
						{
							const PlannedReturnEventView events =
								planned_return_event_view(planned_return.legs[leg_index].axis_index);
							reset_state_ready = reset_state_ready && !events.busy && !events.error;
						}
						if (reset_state_ready)
						{
							if (!planned_return_targets_safe())
							{
								fail_planned_return("安全重试前目标重新校验失败", nullptr, true);
							}
							else
							{
								++planned_return.retry_count;
								for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
								{
									PlannedReturnLeg& leg = planned_return.legs[leg_index];
									leg.acknowledged = false;
									leg.started = false;
									leg.possibly_started = false;
									leg.done = false;
									leg.request_armed = false;
									leg.request_event_sequence = 0;
									leg.ack_event_sequence = 0;
									leg.error_event_sequence = 0;
									leg.reported_error_id = 0;
									leg.cancel_event_sequence = 0;
									leg.cancel_event_required = false;
									leg.entry_rel = plc_act_pos[leg.axis_index];
								}
								planned_return.request_prepared = false;
								planned_return.commit_guard_generation = 0;
						planned_return.execution_t0_ms = 0;
						planned_return.phase = PlannedReturnPhase::SubmitRequest;
					}
						}
						else if ((now_ms - planned_return.retry_clear_t0_ms) >=
							cfg.planned_return_retry_clear_timeout_ms)
						{
							fail_planned_return("安全重试前PLC状态复位超时", nullptr, true);
						}
					}
				}
				else if (planned_return.phase == PlannedReturnPhase::AwaitFreshSnapshot)
				{
					if (ads_motion_cycle_valid && ads_snapshot_sequence > planned_return.done_snapshot_sequence)
					{
						if (!rebase_planned_return())
						{
							fail_planned_return("计划回退完成后的非阻塞基准重建失败", nullptr, true);
						}
						else
						{
							axis1_fast_return = false;
							axis6_fast_retract = false;
							planned_return.phase = PlannedReturnPhase::PublishHandoff;
						}
					}
				}
				else if (planned_return.phase == PlannedReturnPhase::AwaitHandoffApplied)
				{
					if (planned_return.handoff_generation != 0 &&
						ads_communication.applied_output_generation() >=
						planned_return.handoff_generation)
					{
						// handoff generation对应的同一Sum Write同时携带各腿Req=FALSE，
						// 因而该代次成功即可同时确认清Req、实际位置保持和夹爪恢复。
						planned_return.completion_clear_confirmed = true;
						planned_return.phase_t0_ms = now_ms;
						planned_return.phase = PlannedReturnPhase::PostHandoffClampSettle;
					}
				}
				else if (planned_return.phase == PlannedReturnPhase::PostHandoffClampSettle)
				{
					if ((now_ms - planned_return.phase_t0_ms) >= planned_return_clamp_settle_ms() &&
						ads_motion_cycle_valid)
					{
						if (!rebase_planned_return())
						{
							fail_planned_return("回退后夹爪稳定完成后的基准重建失败", nullptr, true);
						}
						else
						{
							if (planned_return.mode == PlannedReturnMode::CatheterDelivery)
							{
								// 回退完成后的附加映射只在后续正常导管前推中生效。
								arm_axis1_delivery_mapping();
							}
							reset_planned_return();
						}
					}
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
				current_cooperative_return_owner() == CooperativeReturnOwner::Axis1;
			const double axis7_cmd_rel =
				(guidewire_mode == GuidewireMode::None || cooperative_axis7_locked)
				? axis7_hold_rel
				: compute_axis7_cmd_rel();
			pos[6] = axis7_cmd_rel;

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
			// - axis6_user_increment_active: 导丝手柄线性通道是否存在有效增量
			// - require_user_increment_for_trigger: 是否要求“触发反弹必须有导丝手柄本人有效增量”
			// - cooperative_axis5_increment_mm: 协同模式下 axis5 本拍已接受增量，用于判断相对窗口运动方向
			auto run_axis6_crawl_state = [&](double axis6_raw_cmd_abs,
				double axis6_increment_mm,
				bool axis6_reverse_mode,
				bool axis6_user_increment_active,
				bool require_user_increment_for_trigger,
				double cooperative_axis5_increment_mm)
			{
				if (axis6_soft_limit_hold)
				{
					hold_axis6_related_axes();
					return;
				}

				const bool cooperative_axis6_mode = guidewire_mode == GuidewireMode::Cooperative;
				const double axis6_window_left_abs_now = axis6_crawl.min_abs();
				const double axis6_window_right_abs_now = axis6_crawl.max_abs();
				// 该函数只在统一回退协调器空闲时调用，因此这里只保留Follow与触发计算。
				{
					double axis6_cmd_abs = axis6_follow_cmd_abs;
					const bool axis6_increment_active = std::abs(axis6_increment_mm) > 0.0;
					if (axis6_increment_active)
					{
						axis6_cmd_abs = clamp_double(axis6_raw_cmd_abs, axis6_window_left_abs_now, axis6_window_right_abs_now);
						if (axis6_target_exceeds_soft_limit(axis6_cmd_abs))
						{
							engage_axis6_soft_limit_hold(
								axis6_cmd_abs,
								Axis6SoftLimitReason::HandleTarget,
								"axis6 手柄目标");
							hold_axis6_related_axes();
							return;
						}
					}

					pos[5] = axis6_cmd_abs - plc_init_pos[5];
					axis6_follow_cmd_abs = axis6_cmd_abs;
					cylinder3_cmd = cyl.cyl3_open;
					cylinder4_cmd = cyl.cyl4_clamp;

					const bool cooperative_relative_window_control = cooperative_axis6_mode;
					bool cooperative_trigger_from_far_edge = false;
					bool axis6_ready_to_trigger = false;
					if (cooperative_relative_window_control)
					{
						// 两种协同方向下，两只手柄分别控制绝对位移；相对增量决定轴6正在靠近哪一侧动态窗口。
						const double relative_increment_mm =
							axis6_increment_mm - cooperative_axis5_increment_mm;
						const bool cooperative_input_active =
							std::abs(axis6_increment_mm) > 0.0 ||
							std::abs(cooperative_axis5_increment_mm) > 0.0;
						const bool at_near_edge =
							axis6_abs <= (axis6_window_left_abs_now + cfg.crawl_arrive_tol_mm);
						const bool at_far_edge =
							axis6_abs >= (axis6_window_right_abs_now - cfg.crawl_arrive_tol_mm);
						const bool trigger_from_near_edge =
							at_near_edge && cooperative_input_active && relative_increment_mm <= 0.0;
						cooperative_trigger_from_far_edge =
							at_far_edge && cooperative_input_active && relative_increment_mm >= 0.0;
						axis6_ready_to_trigger =
							trigger_from_near_edge || cooperative_trigger_from_far_edge;
					}
					else
					{
						const double axis6_trigger_edge_abs =
							axis6_reverse_mode ? axis6_window_right_abs_now : axis6_window_left_abs_now;
						if (axis6_reverse_switch_guard_active &&
							(std::abs(axis6_abs - axis6_trigger_edge_abs) > cfg.reverse_switch_trigger_guard_mm))
						{
							axis6_reverse_switch_guard_active = false;
						}
						const bool axis6_toward_trigger =
							axis6_reverse_mode ? (axis6_increment_mm > 0.0) : (axis6_increment_mm < 0.0);
						const bool axis6_trigger_user_ok =
							(!require_user_increment_for_trigger) || axis6_user_increment_active;
						const bool axis6_at_trigger_edge =
							std::abs(axis6_abs - axis6_trigger_edge_abs) <= cfg.crawl_arrive_tol_mm;
						const double axis6_prev_abs = axis6_prev_abs_valid ? axis6_prev_abs_for_trigger : axis6_abs;
						// 按运动方向判断是否到达或跨过触发边，避免两个 ADS 采样点跨过窄容差窗时漏触发。
						const bool axis6_enter_trigger_edge = axis6_reverse_mode
							? ((axis6_prev_abs < (axis6_trigger_edge_abs - cfg.crawl_arrive_tol_mm)) &&
								(axis6_abs >= (axis6_trigger_edge_abs - cfg.crawl_arrive_tol_mm)))
							: ((axis6_prev_abs > (axis6_trigger_edge_abs + cfg.crawl_arrive_tol_mm)) &&
								(axis6_abs <= (axis6_trigger_edge_abs + cfg.crawl_arrive_tol_mm)));
						// 在窗口端点切换递送/撤出时，正确方向的手柄输入会被窗口夹住，
						// 无法满足“从窗口内部进入”的普通触发条件。此处仅允许切换后的
						// 首次有效同向输入消耗保护并触发一次换手；切换本身不会产生运动。
						const bool axis6_guarded_edge_input =
							axis6_reverse_switch_guard_active &&
							axis6_at_trigger_edge &&
							axis6_increment_active &&
							axis6_toward_trigger &&
							axis6_trigger_user_ok;
						const bool axis6_switch_guard_blocked =
							axis6_reverse_switch_guard_active && !axis6_guarded_edge_input &&
							(std::abs(axis6_abs - axis6_trigger_edge_abs) <= cfg.reverse_switch_trigger_guard_mm);
						if (axis6_guarded_edge_input)
						{
							axis6_reverse_switch_guard_active = false;
						}
						// 独立导丝每个方向只有一个换手触发边：必须由窗口内部实际进入该边。
						// 不能因停在边界后持续推手柄重复触发；方向切换后的一次性边界输入除外。
						axis6_ready_to_trigger =
							axis6_trigger_user_ok &&
							axis6_increment_active &&
							axis6_toward_trigger &&
							(axis6_enter_trigger_edge || axis6_guarded_edge_input) &&
							!axis6_switch_guard_blocked;
					}
					if (axis6_ready_to_trigger && !planned_return.active())
					{
						double axis6_return_target_abs = axis6_reverse_mode
							? axis6_window_left_abs_now
							: axis6_window_right_abs_now;
						if (cooperative_relative_window_control)
						{
							const double half_window_mm =
								(axis6_window_right_abs_now - axis6_window_left_abs_now) * 0.5;
							const double reset_inset_mm = clamp_double(
								cfg.cooperative_axis6_reset_inset_mm,
								0.0,
								half_window_mm);
							axis6_return_target_abs = cooperative_trigger_from_far_edge
								? axis6_window_left_abs_now + reset_inset_mm
								: axis6_window_right_abs_now - reset_inset_mm;
						}
						if (axis6_target_exceeds_soft_limit(axis6_return_target_abs))
						{
							engage_axis6_soft_limit_hold(
								axis6_return_target_abs,
								Axis6SoftLimitReason::PlannedReturn,
								"axis6 计划回退");
							hold_axis6_related_axes();
							return;
						}
						const PlannedReturnMode return_mode = cooperative_axis6_mode
							? (axis6_reverse_mode
								? PlannedReturnMode::CooperativeRetractionAxis6
								: PlannedReturnMode::CooperativeDeliveryAxis6)
							: (axis6_reverse_mode
								? PlannedReturnMode::GuidewireRetraction
								: PlannedReturnMode::GuidewireDelivery);
						const PlannedReturnRebaseScope rebase_scope = cooperative_axis6_mode
							? PlannedReturnRebaseScope::Cooperative
							: PlannedReturnRebaseScope::Axis6;
						if (begin_planned_return(
							return_mode,
							rebase_scope,
							false,
							0.0,
							true,
							axis6_return_target_abs))
						{
							// 本拍立即冻结另一条链路并下发换手夹爪目标；电机请求仍等待50 ms。
							apply_planned_return_outputs();
						}
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
					!planned_return.active();

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

			if (axis6_soft_limit_hold && !motion_startup_active)
			{
				// 实际越限时继续冻结；启动准备的安全目标允许把 axis6 拉回限制内。
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
				// 标准启动中 axis2/7 从第一拍执行目标；中断恢复由恢复阶段覆盖 axis1/6 等目标。
				pos[0] = startup.axis1_hold_rel;
				pos[1] = startup.final_axis2_deg;
				pos[2] = startup.axis3_hold_rel;
				pos[4] = startup.axis5_hold_rel;
				pos[5] = startup.axis6_hold_rel;
				pos[6] = startup.final_axis7_deg;

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
				auto complete_startup_sequence = [&]()
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
						plc_restart_recovery_latched = false;
						std::cout << (startup.recovery_mode ? "中断恢复启动已完成。" : "启动准备流程已完成。") << std::endl;
					}
					else
					{
						control_active = false;
						std::cout << (startup.recovery_mode
							? "中断恢复启动已完成，但重同步失败。"
							: "启动准备流程已完成，但重同步失败。") << std::endl;
					}
				};

				if (startup.recovery_mode)
				{
					// 上位机中断恢复：全程保持正常抓持组合，不再执行标准装卸夹爪阶段。
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_clamp;
					cylinder3_cmd = cyl.cyl3_open;
					cylinder4_cmd = cyl.cyl4_clamp;
					pos[0] = from_left_to_rel(0, startup.final_axis1_from_left_mm);
					pos[5] = from_left_to_rel(5, startup.final_axis6_from_left_mm);

					if (startup.phase == StartupPhase::RecoveryMoveAxis1267)
					{
						if ((now_ms - startup.phase_t0) >= cfg.startup_recovery_stage_delay_ms)
						{
							startup.phase = StartupPhase::RecoveryMoveAxis5;
							startup.phase_t0 = now_ms;
							std::cout << "中断恢复启动：axis1/2/6/7 已启动 2 秒，开始移动 axis5。" << std::endl;
						}
					}
					else if (startup.phase == StartupPhase::RecoveryMoveAxis5)
					{
						pos[4] = from_left_to_rel(4, startup.final_axis5_from_left_mm);
						if ((now_ms - startup.phase_t0) >= cfg.startup_recovery_stage_delay_ms)
						{
							startup.phase = StartupPhase::RecoveryMoveAxis3;
							startup.phase_t0 = now_ms;
							std::cout << "中断恢复启动：axis5 已启动 2 秒，开始移动 axis3。" << std::endl;
						}
					}
					else if (startup.phase == StartupPhase::RecoveryMoveAxis3)
					{
						pos[4] = from_left_to_rel(4, startup.final_axis5_from_left_mm);
						pos[2] = from_left_to_rel(2, startup.final_axis3_from_left_mm);
						if (startup_final_targets_reached())
						{
							complete_startup_sequence();
						}
					}
				}
				else if (startup.phase == StartupPhase::ReleaseClamps)
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
						std::cout << "启动准备：阶段4按 UI 最终目标移动 1/3/5/6 轴，axis2/7 继续保持启动目标。" << std::endl;
					}
				}
				else if (startup.phase == StartupPhase::MoveAxis356BackToReady)
				{
					// 阶段 4：直线轴 UI 最终目标在此阶段生效；旋转轴目标已从阶段1开始执行。
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
						complete_startup_sequence();
					}
				}
			}
			else if (planned_return.active())
			{
				// 六种模式的计划回退和交接统一由单一协调器推进。
				step_planned_return();
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
				if (axis6_directional_increment_active)
				{
					axis6_nominal_cmd_abs = clamp_double(
						axis6_follow_start_abs + axis6_directional_increment_mm,
						axis6_crawl.min_abs(),
						axis6_crawl.max_abs());
				}
				const double axis6_nominal_delta_axis_mm = axis6_nominal_cmd_abs - axis6_follow_start_abs;
				const double axis6_nominal_forward_mm =
					(!axis6_effective_reverse_pressed && axis6_nominal_delta_axis_mm < 0.0)
					? -axis6_nominal_delta_axis_mm : 0.0;
				double axis6_requested_forward_mm = axis6_nominal_forward_mm;
				double axis6_raw_cmd_abs = axis6_follow_start_abs + axis6_directional_increment_mm;
				double axis6_increment_for_state_mm = axis6_directional_increment_mm;
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
					axis6_directional_increment_active,
					false,
					0.0);
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
				const bool axis1_return_couples_axis6 = !cooperative_mode;
				const bool cooperative_axis1_locked =
					cooperative_mode &&
					current_cooperative_return_owner() == CooperativeReturnOwner::Axis6;
				double axis6_catheter_window_start_abs = axis6_abs;
				double axis6_catheter_window_end_abs = axis6_abs;
				if (axis1_return_couples_axis6)
				{
					// 普通导管按轴5实际位置约束轴1回退时的轴6联动目标。
					motion_sync::calculate_axis6_window_from_axis5(
						ctx,
						axis6_catheter_window_start_abs,
						axis6_catheter_window_end_abs);
					if (!cooperative_mode)
					{
						axis6_crawl.start_abs = axis6_catheter_window_start_abs;
						axis6_crawl.end_abs = axis6_catheter_window_end_abs;
						axis6_crawl.window_active = is_within_range(
							axis6_abs,
							axis6_crawl.min_abs(),
							axis6_crawl.max_abs(),
							cfg.crawl_arrive_tol_mm);
					}
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
				else
				{
					const double axis1_raw_cmd_abs = axis1_follow_cmd_abs + axis1_directional_increment_mm;
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
					if (axis1_delivery_stop_latched && axis1_reverse_pressed && axis1_directional_increment_active)
					{
						axis1_delivery_stop_latched = false;
						axis1_delivery_stop_prompted = false;
					}

					const double axis1_follow_start_abs = axis1_follow_cmd_abs;
					auto constrain_axis1_follow_cmd_abs = [&](double candidate_abs) -> double
					{
						double command_abs = axis1_follow_start_abs;
						if (axis1_directional_increment_active && axis1_follow_enabled)
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
					// 计划回退完成后，映射只作用于普通导管正向 Follow 的前 10 mm。
					// 进度使用本拍名义手柄增量累计，不能从已附加映射的绝对目标反推，
					// 否则每拍都会把附加量再次当成新的名义输入。
					double axis1_mapping_extra_requested_mm = 0.0;
					if (axis1_delivery_mapping_active &&
						!cooperative_mode &&
						!axis1_reverse_pressed &&
						axis1_nominal_forward_mm > 0.0 &&
						cfg.axis1_post_return_mapping_span_mm > 1e-6)
					{
						const double mapping_span_mm = cfg.axis1_post_return_mapping_span_mm;
						const double progress_before_mm = clamp_double(
							axis1_delivery_mapping_progress_mm,
							0.0,
							mapping_span_mm);
						const double progress_after_mm = (std::min)(
							mapping_span_mm,
							progress_before_mm + axis1_nominal_forward_mm);
						const double target_extra_mm = clamp_double(
							cfg.axis1_post_return_lead_mm * progress_after_mm / mapping_span_mm,
							0.0,
							cfg.axis1_post_return_lead_limit_mm);
						axis1_mapping_extra_requested_mm = (std::max)(
							0.0,
							target_extra_mm - axis1_delivery_mapping_applied_extra_mm);
						const double mapped_target_abs = axis1_cmd_abs - axis1_mapping_extra_requested_mm;
						const double mapped_cmd_abs = constrain_axis1_follow_cmd_abs(mapped_target_abs);
						const double mapping_extra_applied_mm = (std::max)(
							0.0,
							axis1_cmd_abs - mapped_cmd_abs);
						axis1_cmd_abs = mapped_cmd_abs;
						axis1_delivery_mapping_progress_mm = progress_after_mm;
						axis1_delivery_mapping_applied_extra_mm += mapping_extra_applied_mm;
						if (progress_after_mm >= mapping_span_mm - 1e-6)
						{
							axis1_delivery_mapping_active = false;
						}
					}
					const double axis1_effective_delta_axis_mm = axis1_cmd_abs - axis1_follow_start_abs;
					const double axis1_effective_forward_mm =
						(!axis1_reverse_pressed && axis1_effective_delta_axis_mm < 0.0)
						? -axis1_effective_delta_axis_mm : 0.0;
					// 映射附加量不计入 PI 的实际补偿量；只有 PI 请求范围内的运动
					// 才用于清理主从补偿欠账，避免映射被误显示为 PI 增益。
					const double axis1_tracking_effective_forward_mm = (std::min)(
						axis1_effective_forward_mm,
						axis1_requested_forward_mm);
					if (axis1_requested_forward_mm > 1e-6 &&
						axis1_tracking_effective_forward_mm + 1e-6 < axis1_requested_forward_mm)
					{
						axis1_tracking_output_clamped = true;
					}
					tracking_controller.commit_command(
						DeliveryTrackingAxis::Axis1,
						axis1_nominal_forward_mm,
						axis1_requested_forward_mm,
						axis1_tracking_effective_forward_mm,
						axis1_tracking_output_clamped);

					pos[0] = axis1_cmd_abs - plc_init_pos[0]; // 绝对目标 -> refer相对坐标（相对 init_pos）
					axis1_follow_cmd_abs = axis1_cmd_abs;

					pos[1] = axis1_crawl.rot_base_rel +
						(axis1_rot_filtered - axis1_crawl.rot_ref) * cfg.axis_rot_scale_deg;
					axis2_hold_rel = pos[1];

					// 轴3/5镜像跟随名义目标（扣除轴1已累计的先行附加量），使轴1相对后方各轴产生真正先行效果
					apply_axis1_mirror_from_abs(axis1_cmd_abs + axis1_delivery_mapping_applied_extra_mm, false);
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
							axis1_reverse_pressed ? (axis1_directional_increment_mm > 0.0) : (axis1_directional_increment_mm < 0.0);
						const double axis1_prev_abs = axis1_prev_abs_valid ? axis1_prev_abs_for_trigger : axis1_abs;
						// 按运动方向判断是否到达或跨过触发边，避免高速时越过 ±tol 后漏掉回退。
						const bool axis1_enter_trigger_edge = axis1_reverse_pressed
							? ((axis1_prev_abs < (axis1_trigger_edge_abs - cfg.crawl_arrive_tol_mm)) &&
								(axis1_abs >= (axis1_trigger_edge_abs - cfg.crawl_arrive_tol_mm)))
							: ((axis1_prev_abs > (axis1_trigger_edge_abs + cfg.crawl_arrive_tol_mm)) &&
								(axis1_abs <= (axis1_trigger_edge_abs + cfg.crawl_arrive_tol_mm)));
						const bool axis1_at_or_past_trigger_edge = axis1_reverse_pressed
							? (axis1_abs >= (axis1_trigger_edge_abs - cfg.crawl_arrive_tol_mm))
							: (axis1_abs <= (axis1_trigger_edge_abs + cfg.crawl_arrive_tol_mm));
						// 上位机重启、重同步或跟踪误差可能让轴停在边界上或略微
						// 越过边界；有效同向输入仍作为重新武装信号。
						const bool axis1_retrigger_from_edge =
							axis1_at_or_past_trigger_edge &&
							axis1_toward_trigger &&
							axis1_directional_increment_active;
						const bool axis1_ready_to_trigger =
							axis1_directional_increment_active &&
							axis1_toward_trigger &&
							axis1_follow_enabled &&
							(axis1_enter_trigger_edge || axis1_retrigger_from_edge) &&
							(!axis1_switch_guard_blocked || axis1_retrigger_from_edge);
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
								if (axis1_return_couples_axis6)
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
											Axis6SoftLimitReason::Axis1CoupledReturn,
											"axis1 回退时 axis6 联动目标");
										hold_axis6_related_axes();
										axis6_coupled_target_safe = false;
									}
								}

								if (axis6_coupled_target_safe)
								{
								if (cooperative_mode)
								{
									axis7_hold_rel = plc_act_pos[6];
									axis6_coop_ff_inited = false;
								}
								if (axis1_return_couples_axis6)
								{
									// 普通导管：axis6 与 axis1 回退位移镜像反向快进。
								}
								const PlannedReturnMode return_mode = cooperative_mode
									? (axis1_reverse_pressed
										? PlannedReturnMode::CooperativeRetractionAxis1
										: PlannedReturnMode::CooperativeDeliveryAxis1)
									: (axis1_reverse_pressed
										? PlannedReturnMode::CatheterRetraction
										: PlannedReturnMode::CatheterDelivery);
								const PlannedReturnRebaseScope rebase_scope = cooperative_mode
									? PlannedReturnRebaseScope::Cooperative
									: PlannedReturnRebaseScope::Axis1;
								if (begin_planned_return(
									return_mode,
									rebase_scope,
									true,
									axis1_return_target_abs,
									axis1_return_couples_axis6,
									candidate_axis6_coupled_target_abs))
								{
									// 本拍先完成夹爪切换和跨链路冻结，电机请求仍等待50 ms。
									apply_planned_return_outputs();
								}
								}
							}
						}
					}
				}
				if (cooperative_mode)
				{
					if (current_cooperative_return_owner() == CooperativeReturnOwner::Axis1)
					{
						// 导管换手期间不再运行 axis6 自身状态机，也不接受线性/旋转手柄输入。
						axis6_coop_ff_inited = false;
						pos[5] = plc_act_pos[5];
						pos[6] = axis7_hold_rel;
						cylinder3_cmd = cyl.cyl3_open;
						cylinder4_cmd = cyl.cyl4_clamp;
					}
					else
					{
						const bool cooperative_follow_active =
							current_cooperative_return_owner() == CooperativeReturnOwner::None &&
							!planned_return.active();
						if (cooperative_follow_active)
						{
							// 两种协同方向都按 axis5 实际位置建立物理相对窗口，避免命令超前导致提前切夹爪。
							const double axis5_window_abs = axis5_abs;
							double axis6_window_left_abs = 0.0;
							double axis6_window_right_abs = 0.0;
							motion_sync::calculate_axis6_window_from_axis5_abs(
								ctx,
								axis5_window_abs,
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
							const double axis5_cmd_abs_for_relative = pos[4] + plc_init_pos[4];
							if (!axis6_coop_ff_inited)
							{
								axis6_coop_ff_inited = true;
								axis6_coop_prev_axis1_cmd_abs = axis5_cmd_abs_for_relative;
							}
							const double axis5_increment_mm =
								axis5_cmd_abs_for_relative - axis6_coop_prev_axis1_cmd_abs;
							axis6_coop_prev_axis1_cmd_abs = axis5_cmd_abs_for_relative;
							// 两种协同方向的线性链路都相互独立：axis6 只累加导丝手柄增量，
							// axis5 增量仅参与相对窗口方向和双边触发判断。
							run_axis6_crawl_state(
								axis6_follow_cmd_abs + axis6_directional_increment_mm,
								axis6_directional_increment_mm,
								cooperative_retraction_active,
								axis6_directional_increment_active,
								true,
								axis5_increment_mm);
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
								true,
								0.0);
						}
					}
				}
			}

			if (!axis6_soft_limit_hold && axis6_soft_limit_warning_active)
			{
				std::cout << "axis6 软件限位阻断已解除：当前动作未再请求越过 "
					<< cfg.axis6_soft_limit_from_left_mm
					<< " mm，控制链路重新按实际位置接管。" << std::endl;
				axis6_soft_limit_warning_active = false;
				axis6_soft_limit_warning_reason = Axis6SoftLimitReason::None;
			}

			const bool tracking_return_started_this_cycle = planned_return.active();
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
		}
		else
		{
			TrackingInvalidReason inactive_reason = TrackingInvalidReason::NotForwardDelivery;
			if (return_ads_fault_hold) inactive_reason = TrackingInvalidReason::AdsReturnFault;
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

		const bool axis4_manual_error_now = ads_events.axis4_manual_error;
		const unsigned long axis4_manual_error_id_now = ads_events.axis4_manual_error_id;
		if (axis4_manual_error_now &&
			(!axis4_manual_error_prev || axis4_manual_error_id_now != axis4_manual_error_id_prev))
		{
			std::cout << "轴4 手动控制报错，错误码: " << axis4_manual_error_id_now << std::endl;
		}
		axis4_manual_error_prev = axis4_manual_error_now;
		axis4_manual_error_id_prev = axis4_manual_error_id_now;

		// 10) 构建本拍离散输出；与 refer 一起交给 100 Hz 通信线程。
		bool cylinder5_req = y_valve_open;
		const bool cylinder_output_enabled = !connection_hold_active && !freeze_active &&
			!estop_hold_active && (control_active || motion_startup_active);
		if (cylinder_output_enabled)
		{
			if (!spacing_recovery.active() && !axis6_soft_limit_hold)
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
		}

		// 记录器直接消费 100 Hz ADS 快照序号；无效快照仍保留时间槽并由写入器归一化为 NaN。
		experiment_recorder.update_ads_communication_stats(ads_stats);
		experiment_recorder.poll_health();
		const std::int64_t recorder_now_qpc = experiment_recorder.clock().now_qpc();
		const DWORD force_sample_now_ms = GetTickCount();
		const bool force_data_consumer_active = experiment_recorder.is_recording() || ff.enabled ||
			tracking_controller.compensation_enabled() || clean_force_monitor_enabled || ft_exp.active();
		if (ctx.force_sample_source == ForceSampleSource::ADS)
		{
			if (has_new_ads_snapshot)
			{
				ForceSampleFrame sampled_frame{};
				sampled_frame.ft_1_value = ads_snapshot.ft_1_value;
				sampled_frame.fn_1_value = ads_snapshot.fn_1_value;
				sampled_frame.fn_2_value = ads_snapshot.fn_2_value;
				sampled_frame.ft_2_value = ads_snapshot.ft_2_value;
				sampled_frame.ft_1_value_v = static_cast<double>(ads_snapshot.ft_1_value) / 1000.0;
				sampled_frame.fn_1_value_v = static_cast<double>(ads_snapshot.fn_1_value) / 1000.0;
				sampled_frame.fn_2_value_v = static_cast<double>(ads_snapshot.fn_2_value) / 1000.0;
				sampled_frame.ft_2_value_v = static_cast<double>(ads_snapshot.ft_2_value) / 1000.0;
				sampled_frame.axis1_pos_rel = ads_snapshot.act_pos_rel[0];
				sampled_frame.axis2_pos_rel = ads_snapshot.act_pos_rel[1];
				sampled_frame.tick_ms = force_sample_now_ms;
				sampled_frame.qpc_ticks = ads_snapshot.qpc_ticks;
				sampled_frame.valid = ads_snapshot.force_valid;
				force_sample = sampled_frame;
				force_sample_ads_sequence = ads_snapshot.attempt_sequence;
			}
			else if (ads_soft_hold_active)
			{
				force_sample.valid = false;
			}
		}
		else
		{
			DWORD force_sampling_period_ms = 20;
			if (tracking_controller.compensation_enabled() && !ff.enabled &&
				!clean_force_monitor_enabled && !ft_exp.active() && !experiment_recorder.is_recording())
			{
				force_sampling_period_ms = 50;
			}
			const bool should_sample_force = force_data_consumer_active &&
				(has_new_ads_snapshot || force_sample_last_sample_ms == 0 ||
				 (force_sample_now_ms - force_sample_last_sample_ms) >= force_sampling_period_ms);
			if (should_sample_force)
			{
				force_sample_last_sample_ms = force_sample_now_ms;
				ForceSampleFrame sampled_frame{};
				double tcp_raw_v[6] = {};
				std::uint64_t tcp_tick_ms = 0;
				std::int64_t tcp_qpc_ticks = 0;
				bool sample_ok = false;
				if (tcp_force_daq.get_latest_raw(tcp_raw_v, tcp_tick_ms, &tcp_qpc_ticks))
				{
					const std::int64_t max_age_ticks = experiment_recorder.clock().frequency() / 2;
					if (tcp_qpc_ticks > 0 && recorder_now_qpc - tcp_qpc_ticks <= max_age_ticks)
					{
						sampled_frame.fn_1_value_v = tcp_raw_v[0];
						sampled_frame.ft_1_value_v = tcp_raw_v[1];
						sampled_frame.axis1_pos_rel = plc_act_pos[0];
						sampled_frame.axis2_pos_rel = plc_act_pos[1];
						sampled_frame.tick_ms = static_cast<DWORD>(tcp_tick_ms);
						sampled_frame.qpc_ticks = tcp_qpc_ticks;
						sampled_frame.valid = true;
						sample_ok = true;
					}
				}
				force_sample = sampled_frame;
				force_sample.valid = sample_ok;
				force_sample_ads_sequence = 0;
			}
		}

		if (force_data_consumer_active && !force_sample.valid)
		{
			const std::string reason = ctx.force_sample_source == ForceSampleSource::TCP_DAQ
				? "力传感器告警：TCP_DAQ 尚无新鲜有效帧。"
				: "力数据无效：仅暂停力反馈并标记记录，位置控制继续。";
			if (force_sample_diag_reason != reason) std::cout << reason << std::endl;
			force_sample_diag_reason = reason;
		}
		else
		{
			force_sample_diag_reason.clear();
		}

		const double csv_nan = std::numeric_limits<double>::quiet_NaN();
		const bool clean_force_valid = force_sample.valid && cal_state.zeroed;
		const CleanForce clean_force = clean_force_valid
			? calculate_clean_force(
				force_sample.fn_1_value_v, force_sample.ft_1_value_v, cal_cfg, cal_state)
			: CleanForce{};
		if (experiment_recorder.is_recording())
		{
			const std::int64_t max_handle_delta_ticks =
				std::max<std::int64_t>(1, experiment_recorder.clock().frequency() / 100);
			for (const AdsFastSnapshot& record_snapshot : drained_ads_snapshots)
			{
				ForceCsvRow force_row{};
				force_row.ads_snapshot_sequence = record_snapshot.attempt_sequence;
				force_row.source_qpc_ticks = record_snapshot.qpc_ticks;
				if (ctx.force_sample_source == ForceSampleSource::ADS)
				{
					force_row.sample_valid = record_snapshot.force_valid;
					force_row.fn_1_raw_v = static_cast<double>(record_snapshot.fn_1_value) / 1000.0;
					force_row.ft_1_raw_v = static_cast<double>(record_snapshot.ft_1_value) / 1000.0;
					force_row.fn_2_raw_v = static_cast<double>(record_snapshot.fn_2_value) / 1000.0;
					force_row.ft_2_raw_v = static_cast<double>(record_snapshot.ft_2_value) / 1000.0;
				}
				else
				{
					const std::int64_t delta = force_sample.qpc_ticks >= record_snapshot.qpc_ticks
						? force_sample.qpc_ticks - record_snapshot.qpc_ticks
						: record_snapshot.qpc_ticks - force_sample.qpc_ticks;
					force_row.sample_valid = force_sample.valid && delta <= max_handle_delta_ticks;
					force_row.fn_1_raw_v = force_sample.fn_1_value_v;
					force_row.ft_1_raw_v = force_sample.ft_1_value_v;
					force_row.fn_2_raw_v = force_sample.fn_2_value_v;
					force_row.ft_2_raw_v = force_sample.ft_2_value_v;
				}
				force_row.calibrated_valid = force_row.sample_valid && cal_state.zeroed;
				const CleanForce row_clean_force = force_row.calibrated_valid
					? calculate_clean_force(
						force_row.fn_1_raw_v, force_row.ft_1_raw_v, cal_cfg, cal_state)
					: CleanForce{};
				const CleanForce row_clean_force_2 = force_row.calibrated_valid
					? calculate_clean_guidewire_force(
						force_row.fn_2_raw_v, force_row.ft_2_raw_v, cal_cfg, cal_state)
					: CleanForce{};
				force_row.fn_1_zero_v = force_row.calibrated_valid ? cal_state.f_zero : csv_nan;
				force_row.ft_1_zero_v = force_row.calibrated_valid ? cal_state.ft_zero : csv_nan;
				force_row.clean_force_n = force_row.calibrated_valid ? row_clean_force.force_n : csv_nan;
				force_row.clean_handle_torque_nm = force_row.calibrated_valid
					? row_clean_force.handle_torque_nm : csv_nan;
				force_row.fn_2_zero_v = force_row.calibrated_valid ? cal_state.fn_2_zero : csv_nan;
				force_row.ft_2_zero_v = force_row.calibrated_valid ? cal_state.ft_2_zero : csv_nan;
				force_row.clean_force_2_n = force_row.calibrated_valid ? row_clean_force_2.force_n : csv_nan;
				force_row.clean_handle_torque_2_nm = force_row.calibrated_valid
					? row_clean_force_2.handle_torque_nm : csv_nan;
				(void)experiment_recorder.enqueue_force(force_row);

				// 100 Hz ADS 序号的偶数帧形成稳定 50 Hz 运动表，不做历史突发补写。
				if ((record_snapshot.attempt_sequence & 1ULL) != 0ULL) continue;
				MotionCsvRow motion_row{};
				motion_row.ads_snapshot_sequence = record_snapshot.attempt_sequence;
				motion_row.source_qpc_ticks = record_snapshot.qpc_ticks;
				motion_row.position_valid = record_snapshot.position_valid;
				for (int axis = 0; axis < 7; ++axis)
				{
					motion_row.axis_from_left_mm[axis] = record_snapshot.act_pos_from_left[axis];
				}
				const HandleRecordSnapshot* nearest_handle = nullptr;
				std::int64_t nearest_delta = std::numeric_limits<std::int64_t>::max();
				for (const HandleRecordSnapshot& candidate : handle_record_history)
				{
					const std::int64_t delta = candidate.qpc_ticks >= record_snapshot.qpc_ticks
						? candidate.qpc_ticks - record_snapshot.qpc_ticks
						: record_snapshot.qpc_ticks - candidate.qpc_ticks;
					if (delta < nearest_delta)
					{
						nearest_delta = delta;
						nearest_handle = &candidate;
					}
				}
				if (nearest_handle != nullptr && nearest_delta <= max_handle_delta_ticks)
				{
					motion_row.axis1_handle_valid = nearest_handle->axis1_valid;
					motion_row.axis1_handle_linear_raw = nearest_handle->axis1_linear_raw;
					motion_row.axis1_handle_linear_filtered = nearest_handle->axis1_linear_filtered;
					motion_row.axis1_handle_rotation_raw = nearest_handle->axis1_rotation_raw;
					motion_row.axis1_handle_rotation_filtered = nearest_handle->axis1_rotation_filtered;
					motion_row.axis6_handle_valid = nearest_handle->axis6_valid;
					motion_row.axis6_handle_linear_raw = nearest_handle->axis6_linear_raw;
					motion_row.axis6_handle_linear_filtered = nearest_handle->axis6_linear_filtered;
					motion_row.axis6_handle_rotation_raw = nearest_handle->axis6_rotation_raw;
					motion_row.axis6_handle_rotation_filtered = nearest_handle->axis6_rotation_filtered;
				}
				(void)experiment_recorder.enqueue_motion(motion_row);
			}
		}

		// PI 与磁盘记录无关；仅在 PI 自身运行时保留 20 Hz 力健康门控。
		if (tracking_controller.compensation_enabled() && !force_sample.valid)
		{
			tracking_controller.disable_compensation();
			std::cout << "主从位移补偿已关闭：当前没有有效力采样。" << std::endl;
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
					// ft_exp 内部维护 start 时的 v_limit 快照；只在覆写值变化时写一次。
					double v_limit_local[7];
					ft_exp.fill_v_limit_override(v_limit_local);
					if (!ft_v_limit_last_valid ||
						std::memcmp(ft_v_limit_last, v_limit_local, sizeof(ft_v_limit_last)) != 0)
					{
						if (plc_io::write_v_limit(ctx, v_limit_local))
						{
							std::memcpy(ft_v_limit_last, v_limit_local, sizeof(ft_v_limit_last));
							ft_v_limit_last_valid = true;
						}
					}
				}
				else
				{
					ft_v_limit_last_valid = false;
				}
			}
		}
		if (!ft_exp.active() || !ft_exp.wants_v_limit_override()) ft_v_limit_last_valid = false;
		// 实验由 active -> 非 active（Done/Abort）的边沿：仅关闭当前会话内的专用 CSV。
		if (ft_exp_was_active && !ft_exp.active() && experiment_recorder.force_transition_log_running())
		{
			experiment_recorder.stop_force_transition_log();
			std::cout << (ft_exp.aborted() ? "力过渡实验已异常终止，CSV 已落盘。"
				: "力过渡实验已完成，CSV 已落盘。") << std::endl;
		}

		// refer、快退、气缸、axis4 与平滑旁路由同一份命令快照发布。
		// 放在力过渡 tick 之后，确保实验本拍产生的 refer/快退请求能够实际下发。
		const bool cancel_hold_output_enabled =
			planned_return.phase == PlannedReturnPhase::CancelWait &&
			planned_return.cancel_rebased && ads_motion_cycle_valid && !estop_hold_active;
		if (cancel_hold_output_enabled)
		{
			// 暂停或回退故障时常规motion_enabled会关闭；取消链仍需单独发送一次
			// “当前实际位置＋关闭快退旁路”，并等待该代次真正写入后才允许Follow。
			load_pos_from_actual();
			axis1_fast_return = false;
			axis6_fast_retract = false;
		}
		AdsOutputCommand ads_output{};
		const bool normal_motion_output_enabled = ads_motion_cycle_valid && !connection_hold_active &&
			!return_ads_fault_hold && !freeze_active && !estop_hold_active &&
			(control_active || motion_startup_active);
		ads_output.motion_enabled = normal_motion_output_enabled || cancel_hold_output_enabled;
		for (int axis = 0; axis < 7; ++axis) ads_output.refer[axis] = pos[axis];
		ads_output.axis1_fast_return = ads_output.motion_enabled && axis1_fast_return;
		ads_output.axis6_fast_retract = ads_output.motion_enabled && axis6_fast_retract;
		if ((planned_return.phase == PlannedReturnPhase::PublishHandoff ||
			planned_return.phase == PlannedReturnPhase::AwaitHandoffApplied) &&
			!planned_return.completion_clear_confirmed)
		{
			// 交接代次持续携带各回退腿的Req=FALSE，直到同一Sum Write成功。
			// 这样不会因desired_output与专用命令队列的并发取样而拆成两拍。
			for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
			{
				const PlannedReturnLeg& leg = planned_return.legs[leg_index];
				if (!leg.active) continue;
				if (leg.axis_index == 0) ads_output.planned_return_clear_mask |= 0x01u;
				else if (leg.axis_index == 5) ads_output.planned_return_clear_mask |= 0x02u;
			}
		}
		ads_output.cylinder[0] = cylinder1_cmd;
		ads_output.cylinder[1] = cylinder2_cmd;
		ads_output.cylinder[2] = cylinder3_cmd;
		ads_output.cylinder[3] = cylinder4_cmd;
		ads_output.cylinder_valid = cylinder_output_enabled;
		ads_output.cylinder5_press_req = cylinder_output_enabled && cylinder5_req;
		ads_output.axis4_forward_req = ads_motion_cycle_valid && axis4_manual_forward_req;
		ads_output.axis4_reverse_req = ads_motion_cycle_valid && axis4_manual_reverse_req;
		for (int injector_index = 0; injector_index < 2; ++injector_index)
		{
			ads_output.inject_push_req[injector_index] = injector_push_request[injector_index];
			ads_output.inject_pull_req[injector_index] = injector_pull_request[injector_index];
		}
		ads_output.startup_smoothing_bypass = ads_output.motion_enabled && startup_smoothing_bypass;
		const std::uint64_t output_generation = ads_communication.publish_output(ads_output);
		if (planned_return.phase == PlannedReturnPhase::SubmitRequest &&
			planned_return.request_prepared &&
			planned_return.commit_guard_generation == 0 &&
			ads_output.motion_enabled)
		{
			bool guard_output_complete = true;
			for (int leg_index = 0; leg_index < planned_return.leg_count; ++leg_index)
			{
				const PlannedReturnLeg& leg = planned_return.legs[leg_index];
				if (!leg.active) continue;
				if (leg.axis_index == 0) guard_output_complete =
					guard_output_complete && ads_output.axis1_fast_return;
				else if (leg.axis_index == 5) guard_output_complete =
					guard_output_complete && ads_output.axis6_fast_retract;
			}
			if (guard_output_complete)
			{
				planned_return.commit_guard_generation = output_generation;
			}
		}
		if (planned_return.phase == PlannedReturnPhase::ClampSettle &&
			planned_return.clamp_output_generation == 0 &&
			ads_output.motion_enabled && ads_output.cylinder_valid)
		{
			// 50 ms 从夹爪命令实际经 Sum Write 应用后开始计算，而不是从任务创建时开始。
			planned_return.clamp_output_generation = output_generation;
		}
		bool clamp_hold_582_trigger = false;
		bool clamp_hold_587_trigger = false;
		if (planned_return.phase == PlannedReturnPhase::PublishHandoff &&
			ads_output.motion_enabled && !ads_output.axis1_fast_return &&
			!ads_output.axis6_fast_retract)
		{
			// 本次 Sum Write 正在发布恢复夹爪组合；只在这一拍触发一次 200 ms 保持。
			clamp_hold_582_trigger = planned_return.contains_axis(0);
			clamp_hold_587_trigger = planned_return.contains_axis(5);
			planned_return.handoff_generation = output_generation;
			planned_return.phase = PlannedReturnPhase::AwaitHandoffApplied;
		}
		if (cancel_hold_output_enabled && planned_return.cancel_hold_generation == 0)
		{
			planned_return.cancel_hold_generation = output_generation;
		}

		process_force_feedback(
			ff,
			force_sample,
			*catheter_force_output_handle,
			*guidewire_force_output_handle,
			guidewire_mode,
			control_active && !spacing_recovery.active() &&
				!ads_soft_hold_active && !return_ads_fault_hold,
			freeze_active,
			estop_hold_active,
			axis1_fast_return,
			axis6_fast_retract,
			clamp_hold_582_trigger,
			clamp_hold_587_trigger,
			GetTickCount64(),
			loop_count,
			cfg,
			cal_cfg,
			cal_state);

		// 力过渡专用表只保存复现实验所需的精简字段，纯净力与统一 force.csv 语义一致。
		if (ft_exp.active() && experiment_recorder.force_transition_log_running())
		{
			ForceTransitionCsvRow r{};
			const std::int64_t transition_qpc = force_sample.qpc_ticks != 0
				? force_sample.qpc_ticks
				: experiment_recorder.clock().now_qpc();
			r.force_snapshot_sequence = force_sample_ads_sequence;
			r.source_qpc_ticks = transition_qpc;
			r.valid = clean_force_valid;
			r.trial_id = ft_exp.current_trial_id();
			r.velocity_level = ft_exp.current_velocity_level();
			r.repeat_in_level = ft_exp.current_repeat_in_level();
			r.phase_code = static_cast<int>(ft_exp.current_phase());
			// GetTickCount 49 天回卷，但单次实验最长几十分钟，回卷期间差值仍正确（DWORD 自然回绕）。
			r.phase_elapsed_ms = static_cast<std::uint64_t>(
				GetTickCount() - ft_exp.current_phase_t0_ms());
			r.v_ratio = ft_exp.current_v_ratio();
			r.axis1_from_left_mm = plc_act_pos[0] + plc_init_pos[0] - plc_leftlimit[0];
			r.clean_force_n = clean_force_valid ? clean_force.force_n : csv_nan;
			r.clean_handle_torque_nm = clean_force_valid ? clean_force.handle_torque_nm : csv_nan;
			experiment_recorder.enqueue_force_transition(r);
		}

		// 无论本拍是否进入控制分支，都更新线性差分基准，避免暂停/等待期间累积大跳变。
		axis1_prev_linear_filtered = axis1_handle_filter.axis0_filtered;
		axis6_prev_linear_filtered = axis6_handle_filter.axis0_filtered;
		axis1_prev_rot_filtered = axis1_handle_filter.axis1_filtered;
		axis6_prev_rot_filtered = axis6_handle_filter.axis1_filtered;

		// 可视化状态限制为约 15 Hz，命令轮询仍每个控制拍执行。
		LARGE_INTEGER vis_now_qpc{};
		QueryPerformanceCounter(&vis_now_qpc);
		const std::int64_t vis_period_ticks =
			std::max<std::int64_t>(1, vis_qpc_frequency.QuadPart / vis_publish_rate_hz);
		if (next_vis_publish_qpc == 0) next_vis_publish_qpc = vis_now_qpc.QuadPart;
		if (vis_now_qpc.QuadPart >= next_vis_publish_qpc)
		{
			next_vis_publish_qpc += vis_period_ticks;
			if (vis_now_qpc.QuadPart - next_vis_publish_qpc > vis_period_ticks)
			{
				next_vis_publish_qpc = vis_now_qpc.QuadPart + vis_period_ticks;
			}
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
			vs.axis1_phase = planned_return_phase_for_axis(0);
			vs.axis6_phase = planned_return_phase_for_axis(5);
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
			vs.cooperative_return_owner = static_cast<int>(current_cooperative_return_owner());
			const DeliveryTrackingAxisSnapshot axis1_tracking_snapshot =
				tracking_controller.snapshot(DeliveryTrackingAxis::Axis1);
			const DeliveryTrackingAxisSnapshot axis6_tracking_snapshot =
				tracking_controller.snapshot(DeliveryTrackingAxis::Axis6);
			vs.tracking_compensation_enabled = tracking_controller.compensation_enabled();
			vs.axis1_tracking_error_mm = axis1_tracking_snapshot.tracking_error_mm;
			vs.axis6_tracking_error_mm = axis6_tracking_snapshot.tracking_error_mm;
			vs.axis1_compensation_gain = axis1_tracking_snapshot.compensation_gain;
			vs.axis6_compensation_gain = axis6_tracking_snapshot.compensation_gain;
			vs.cooperative_direction = static_cast<int>(cooperative_direction);
			vs.axis6_soft_limit_hold = axis6_soft_limit_hold;
			const ExperimentRecorderSnapshot recorder_snapshot = experiment_recorder.snapshot();
			vs.recording_state = static_cast<int>(recorder_snapshot.state);
			vs.recording_error = static_cast<int>(recorder_snapshot.error);
			vs.recording_elapsed_us = recorder_snapshot.elapsed_us;
			vs.force_writer_dropped = recorder_snapshot.force_dropped;
			vs.motion_writer_dropped = recorder_snapshot.motion_dropped;
			vs.force_schedule_missed = recorder_snapshot.force_missed;
			vs.motion_schedule_missed = recorder_snapshot.motion_missed;
			vs.force_sample_valid = force_sample.valid;
			vs.clean_force_valid = clean_force_valid;
			vs.clean_force_n = clean_force_valid ? clean_force.force_n : csv_nan;
			vs.clean_handle_torque_nm = clean_force_valid ? clean_force.handle_torque_nm : csv_nan;
			vs.camera_state = static_cast<int>(recorder_snapshot.camera.state);
			vs.camera_input_format = static_cast<int>(recorder_snapshot.camera.input_format);
			vs.camera_width = recorder_snapshot.camera.width;
			vs.camera_height = recorder_snapshot.camera.height;
			vs.camera_fps_numerator = recorder_snapshot.camera.fps_numerator;
			vs.camera_fps_denominator = recorder_snapshot.camera.fps_denominator;
			vs.camera_preview_enabled = recorder_snapshot.camera.preview_enabled;
			vs.camera_recording = recorder_snapshot.camera.recording;
			vs.camera_recording_elapsed_us = recorder_snapshot.camera.recording_elapsed_us;
			vs.camera_frame_count = recorder_snapshot.camera.frame_count;
			vs.camera_dropped_frames = recorder_snapshot.camera.dropped_frames;
			vs.camera_error_code = recorder_snapshot.camera.error_code;
			vs.physical_button_event_counter = physical_button_event_counter;
			vs.physical_button_event_code = physical_button_event_code;
			vs.ads_state = static_cast<int>(ads_stats.state);
			vs.ads_actual_hz = ads_stats.actual_hz;
			vs.ads_snapshot_age_us = ads_stats.latest_snapshot_age_us;
			vs.ads_rtt_us = ads_stats.latest_rtt_us;
			vs.ads_failed_cycles = ads_stats.failed_cycles;
			vs.ads_reconnect_count = ads_stats.reconnect_count;
			vs.plc_restart_count = ads_stats.plc_restart_count;
			vs.host_comm_timeout = ads_events.host_comm_timeout;
			const ArmManualSnapshot arm_snapshot = arm_manual_ads.snapshot();
			vs.arm_manual_enable = arm_snapshot.manual_enable;
			for (int i = 0; i < 5; ++i)
			{
				vs.arm_enable_req[i] = arm_snapshot.enable_req[i];
				vs.arm_power_done[i] = arm_snapshot.power_done[i];
				vs.arm_power_busy[i] = arm_snapshot.power_busy[i];
				vs.arm_power_active[i] = arm_snapshot.power_active[i];
				vs.arm_power_error[i] = arm_snapshot.power_error[i];
				vs.arm_power_error_id[i] = arm_snapshot.power_error_id[i];
				vs.arm_reset_done[i] = arm_snapshot.reset_done[i];
				vs.arm_reset_busy[i] = arm_snapshot.reset_busy[i];
				vs.arm_reset_active[i] = arm_snapshot.reset_active[i];
				vs.arm_reset_error[i] = arm_snapshot.reset_error[i];
				vs.arm_reset_error_id[i] = arm_snapshot.reset_error_id[i];
				vs.arm_act_pos[i] = arm_snapshot.act_pos[i];
				vs.arm_act_vel[i] = arm_snapshot.act_vel[i];
				vs.arm_motion_busy[i] = arm_snapshot.motion_busy[i];
				vs.arm_motion_done[i] = arm_snapshot.motion_done[i];
				vs.arm_motion_error[i] = arm_snapshot.motion_error[i];
				vs.arm_motion_error_id[i] = arm_snapshot.motion_error_id[i];
				vs.arm_cmd_dir[i] = arm_snapshot.cmd_dir[i];
				vs.arm_cmd_conflict[i] = arm_snapshot.cmd_conflict[i];
				vs.arm_jog_velocity[i] = arm_snapshot.jog_velocity[i];
				vs.arm_jog_acc[i] = arm_snapshot.jog_acc[i];
				vs.arm_jog_dec[i] = arm_snapshot.jog_dec[i];
				vs.arm_jog_jerk[i] = arm_snapshot.jog_jerk[i];
			}
			vs.axis4_manual_busy = ads_events.axis4_manual_busy;
			vs.axis4_manual_done = ads_events.axis4_manual_done;
			vs.axis4_manual_error = ads_events.axis4_manual_error;
			vs.axis4_manual_error_id = ads_events.axis4_manual_error_id;
			vs.force_feedback_hold_enabled = ff.clamp_hold_enabled;
			vs.force_feedback_hold_active = ff.clamp_hold_enabled && ff.clamp_hold_owner() != 0;
			vs.force_feedback_hold_owner = ff.clamp_hold_owner();
			vis_server.push_state(vs);
		}

		{
			VisCommand vcmd;
			while (vis_server.poll_command(vcmd))
			{
				switch (vcmd.type)
				{
				case VisCommandType::SetCylinderManualOpen:
				case VisCommandType::SetCylinderManualClosed:
				{
					const bool valid_cylinder_index = vcmd.param1 >= 0 && vcmd.param1 < 4;
					const bool automatic_cylinder_sequence_active =
						motion_startup_active ||
						axis6_soft_limit_hold ||
						planned_return.active();
					if (!spacing_recovery.active() && !spacing_recovery.requested &&
						valid_cylinder_index && !automatic_cylinder_sequence_active)
					{
						cylinder_manual_mode[vcmd.param1] =
							vcmd.type == VisCommandType::SetCylinderManualOpen
							? CylinderManualMode::Open
							: CylinderManualMode::Closed;
					}
					else if (valid_cylinder_index && automatic_cylinder_sequence_active)
					{
						std::cout << "UI：自动换手、启动准备或 axis6 软限位正在接管，已忽略电缸手动覆盖请求。" << std::endl;
					}
					break;
				}
				case VisCommandType::RequestModeSwitch:
					if (single_handle_mode && !ads_soft_hold_active && !planned_return.active())
						single_handle_requested_mode = static_cast<GuidewireMode>(vcmd.param1);
					else if (single_handle_mode && planned_return.active())
						std::cout << "UI：单手柄模式切换已忽略，当前计划换手尚未完成。" << std::endl;
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
				case VisCommandType::SetForceFeedbackHold:
					ff.clamp_hold_enabled = vcmd.param1 != 0;
					ff.clear_clamp_holds();
					std::cout << "UI：力反馈-保持："
						<< (ff.clamp_hold_enabled ? "开启（自动换手闭爪后保持 200 ms）" : "关闭")
						<< std::endl;
					break;
				case VisCommandType::SetReverseMode:
				{
					if (ads_soft_hold_active)
					{
						std::cout << "UI：ADS 软保持或重连期间已忽略模式切换。" << std::endl;
						break;
					}
					if (planned_return.active())
					{
						std::cout << "UI：模式切换已忽略，计划回退尚未完成。" << std::endl;
						break;
					}
					// param1: 0=catheter mode+direction, 1=guidewire mode+direction
					// param2: 0=forward(递送), 1=reverse(撤出)
					const ModeSelection selection = vcmd.param1 == 0
						? (vcmd.param2 != 0 ? ModeSelection::CatheterRetraction : ModeSelection::CatheterDelivery)
						: (vcmd.param2 != 0 ? ModeSelection::GuidewireRetraction : ModeSelection::GuidewireDelivery);
					request_mode_selection(selection, "UI 模式按钮");
					break;
				}
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
						const std::int64_t gate_now_qpc = experiment_recorder.clock().now_qpc();
						const std::int64_t gate_max_age = experiment_recorder.clock().frequency() / 2;
						const bool force_sample_fresh =
							force_sample.valid &&
							force_sample.qpc_ticks > 0 &&
							gate_now_qpc >= force_sample.qpc_ticks &&
							gate_now_qpc - force_sample.qpc_ticks <= gate_max_age;
						if (!force_sample_fresh)
						{
							ForceSampleFrame gate_sample{};
							bool gate_sample_ok = false;
							if (ctx.force_sample_source == ForceSampleSource::TCP_DAQ)
							{
								double tcp_raw_v[6] = {};
								std::uint64_t tcp_tick_ms = 0;
								std::int64_t tcp_qpc_ticks = 0;
								if (tcp_force_daq.get_latest_raw(tcp_raw_v, tcp_tick_ms, &tcp_qpc_ticks) &&
									tcp_qpc_ticks > 0 && gate_now_qpc >= tcp_qpc_ticks &&
									gate_now_qpc - tcp_qpc_ticks <= gate_max_age)
								{
									gate_sample.fn_1_value_v = tcp_raw_v[0];
									gate_sample.ft_1_value_v = tcp_raw_v[1];
									gate_sample.axis1_pos_rel = plc_act_pos[0];
									gate_sample.axis2_pos_rel = plc_act_pos[1];
									gate_sample.tick_ms = static_cast<DWORD>(tcp_tick_ms);
									gate_sample.qpc_ticks = tcp_qpc_ticks;
									gate_sample.valid = true;
									gate_sample_ok = true;
								}
							}
							else
							{
								gate_sample_ok = read_force_sample(gate_sample);
							}
							force_sample = gate_sample;
							force_sample.valid = gate_sample_ok;
						}
						if (!forward_tracking_mode || !force_sample.valid)
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
				case VisCommandType::StartExperimentRecording:
				{
					const ExperimentRecorderSnapshot before_start = experiment_recorder.snapshot();
					if (before_start.state != ExperimentRecordingState::Idle &&
						before_start.state != ExperimentRecordingState::Error)
					{
						break;
					}
					ExperimentStartInfo start_info{};
					start_info.axis1_handle_serial = axis1_input_handle != nullptr
						? axis1_input_handle->serial()
						: 0;
					start_info.axis6_handle_serial = axis6_input_handle != nullptr
						? axis6_input_handle->serial()
						: 0;
					start_info.single_handle_mode = single_handle_mode;
					if (experiment_recorder.start(vcmd.payload_utf8, start_info))
					{
						std::cout << "UI：统一实验记录已启动。" << std::endl;
					}
					else
					{
						const ExperimentRecorderSnapshot failed_snapshot = experiment_recorder.snapshot();
						std::cout << "UI：统一实验记录启动失败，错误码："
							<< static_cast<int>(failed_snapshot.error) << "。" << std::endl;
					}
					break;
				}
				case VisCommandType::StopExperimentRecording:
					experiment_recorder.stop_async("user_stop");
					break;
				case VisCommandType::SetCameraPreview:
					experiment_recorder.set_camera_preview(vcmd.param1 != 0);
					break;
				case VisCommandType::SetCleanForceMonitor:
					clean_force_monitor_enabled = vcmd.param1 != 0;
					break;
				case VisCommandType::SetArmManualEnable:
					arm_manual_ads.set_manual_enable(vcmd.param1 != 0);
					break;
				case VisCommandType::SetArmAxisEnable:
					if (vcmd.param1 >= 1 && vcmd.param1 <= 5)
						arm_manual_ads.set_axis_enable(vcmd.param1, vcmd.param2 != 0);
					break;
				case VisCommandType::RequestArmAxisReset:
					if (vcmd.param1 >= 1 && vcmd.param1 <= 5)
						arm_manual_ads.request_reset(vcmd.param1);
					break;
				case VisCommandType::SetArmAxisJog:
					if (vcmd.param1 >= 1 && vcmd.param1 <= 5 && vcmd.param2 >= -1 && vcmd.param2 <= 1)
						arm_manual_ads.set_jog_direction(vcmd.param1, vcmd.param2);
					break;
				case VisCommandType::SetArmJogParameter:
				{
					const int axis_one_based = (vcmd.param1 / 4) + 1;
					const int parameter_kind = vcmd.param1 % 4;
					const double value = static_cast<double>(vcmd.param2) / 1000.0;
					if (vcmd.param1 < 0 || vcmd.param1 >= 20 ||
						!arm_manual_ads.set_jog_parameter(axis_one_based, parameter_kind, value))
					{
						std::cout << "定位臂点动参数已忽略：轴号、字段或数值超出范围。" << std::endl;
					}
					break;
				}
				case VisCommandType::SetAxis4ManualJog:
					if (vcmd.param1 >= -1 && vcmd.param1 <= 1)
					{
						axis4_ui_forward_pressed = vcmd.param1 > 0;
						axis4_ui_reverse_pressed = vcmd.param1 < 0;
						axis4_ui_jog_deadline_ms = vcmd.param1 == 0
							? 0
							: GetTickCount64() + axis4_ui_jog_lease_ms;
					}
					break;
				case VisCommandType::SetYValveOpen:
					y_valve_open = vcmd.param1 != 0;
					std::cout << "UI：Y阀：" << (y_valve_open ? "打开。" : "关闭。") << std::endl;
					break;
				case VisCommandType::SetInjectorManualJog:
					if (vcmd.param1 >= 1 && vcmd.param1 <= 2 &&
						vcmd.param2 >= -1 && vcmd.param2 <= 1)
					{
						const int injector_index = vcmd.param1 - 1;
						injector_ui_direction[injector_index] = vcmd.param2;
						injector_ui_jog_deadline_ms[injector_index] = vcmd.param2 == 0
							? 0
							: GetTickCount64() + injector_ui_jog_lease_ms;
					}
					break;
				case VisCommandType::SetAxis1PostReturnLead:
				{
					const double lead_mm = static_cast<double>(vcmd.param1) / 1000.0;
					if (lead_mm < 0.0 ||
						lead_mm > cfg.axis1_post_return_lead_limit_mm)
					{
						std::cout << "UI：axis1 前10 mm比例映射量已忽略，范围为 [0, 5] mm。"
							<< std::endl;
					}
					else
					{
						cfg.axis1_post_return_lead_mm = lead_mm;
						// 参数变更从下一段回退后的递送重新计量，避免旧映射进度
						// 继续套用到新参数。
						clear_axis1_delivery_mapping();
						std::cout << "UI：axis1 前10 mm比例映射量已更新为 "
							<< lead_mm << " mm。" << std::endl;
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
					if (plc_restart_recovery_latched && !cal_state.zeroed)
					{
						std::cout << "UI：PLC 重启后必须先重新力感调零，启动准备已忽略。" << std::endl;
						break;
					}
					bool valid = true;
					if (pending_startup.axis1_from_left_mm < 5.0 || pending_startup.axis1_from_left_mm > 95.0) valid = false;
					if (pending_startup.axis3_from_left_mm < 10.0 || pending_startup.axis3_from_left_mm > 650.0) valid = false;
					if (pending_startup.axis5_from_left_mm < 10.0 || pending_startup.axis5_from_left_mm > 670.0) valid = false;
					if (pending_startup.axis6_from_left_mm < 10.0 || pending_startup.axis6_from_left_mm > 670.0) valid = false;
					if (pending_startup.axis2_deg < -360.0 || pending_startup.axis2_deg > 360.0) valid = false;
					if (pending_startup.axis7_deg < -360.0 || pending_startup.axis7_deg > 360.0) valid = false;
					if (pending_startup.axis6_from_left_mm < pending_startup.axis5_from_left_mm) valid = false;
					if (pending_startup.axis5_from_left_mm < pending_startup.axis3_from_left_mm) valid = false;
					if (pending_startup.axis3_from_left_mm < pending_startup.axis1_from_left_mm) valid = false;
					if (pending_startup.speed_scale < 0.00001 || pending_startup.speed_scale > 0.5) valid = false;
					if (!valid)
					{
						std::cout << "UI：启动准备参数已拒绝，请检查直线轴范围、axis2/7 的 [-360, 360] deg 范围及速度比例。" << std::endl;
						break;
					}
					if (!startup.completed && startup.phase == StartupPhase::WaitForEnter &&
						!freeze_active && !estop_hold_active && !ads_soft_hold_active &&
						(!has_self_check_flag || self_check_done))
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
							clear_cylinder_manual_overrides();
							control_active = false;
							std::cout << "启动准备流程已开始（UI 参数）。" << std::endl;
						}
					}
					break;
				}
				case VisCommandType::SelectDirectControl:
				{
					if (plc_restart_recovery_latched && !cal_state.zeroed)
					{
						std::cout << "UI：PLC 重启后必须先重新力感调零，直接控制已忽略。" << std::endl;
						break;
					}
					if (!startup.completed &&
						startup.phase == StartupPhase::WaitForEnter &&
						!freeze_active &&
						!estop_hold_active &&
						!ads_soft_hold_active &&
						(!has_self_check_flag || self_check_done))
					{
						if (restore_startup_v_limit() &&
							consume_startup_loading_ready() &&
							sync_all(20))
						{
						clear_cylinder_manual_overrides();
							startup.recovery_mode = false;
							startup.phase = StartupPhase::Done;
							startup.completed = true;
							startup.prompted = false;
							control_active = true;
							plc_restart_recovery_latched = false;
							std::cout << "已进入直接控制（UI 触发）。" << std::endl;
						}
					}
					break;
				}
				case VisCommandType::SetGravityCompensation:
				{
					const bool requested_enabled = (vcmd.param1 != 0);
					const bool enabled = requested_enabled && cal_cfg.gravity_comp_validated;
					if (requested_enabled && !cal_cfg.gravity_comp_validated)
					{
						std::cout << "UI：重力补偿启用已拒绝：返工后的F_direct尚未完成旋转重力复标。"
							<< std::endl;
					}
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
				{
					if (vcmd.param1 != 0 && ads_soft_hold_active)
					{
						std::cout << "UI：ADS 软保持或重连期间已忽略屈曲恢复请求。" << std::endl;
						break;
					}
					const bool automatic_return_active = planned_return.active();
					if (vcmd.param1 != 0 && automatic_return_active)
					{
						std::cout << "UI：屈曲恢复已忽略，请等待当前换手完成。"
							<< std::endl;
						break;
					}
					if (vcmd.param1 != 0)
					{
						cancel_cooperative_delivery(true);
						pending_mode_selection = ModeSelection::None;
						pending_physical_mode_source = PhysicalModeSource::None;
					}
					spacing_recovery.requested = (vcmd.param1 != 0);
					break;
				}
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
						if (ads_soft_hold_active)
						{
							std::cout << "UI：ADS 软保持或重连期间已忽略协同模式切换。" << std::endl;
						}
						else if (!dual_handle_ready)
						{
							std::cout << "UI：" << mode_name
								<< "已忽略，程序启动时未成功连接两只手柄。" << std::endl;
						}
						else if (planned_return.active())
						{
							std::cout << "UI：" << mode_name
								<< "已忽略，当前协同换手仍在执行。" << std::endl;
						}
						else
						{
							const ModeSelection selection =
								requested_direction == CooperativeDirection::Retraction
								? ModeSelection::CooperativeRetraction
								: ModeSelection::CooperativeDelivery;
							// 恢复中先退出并重同步；正常状态下立即提交协同入口请求。
							request_mode_selection(selection, "UI 协同模式按钮");
						}
					}
					else if (planned_return.active())
					{
						std::cout << "UI：" << mode_name
							<< "退出已忽略，请等待当前计划换手完成。" << std::endl;
					}
					else
					{
						cooperative_direction_requested = CooperativeDirection::None;
						physical_mode_source = PhysicalModeSource::None;
						pending_physical_mode_source = PhysicalModeSource::None;
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
							!ads_soft_hold_active &&
							!axis6_soft_limit_hold &&
							guidewire_mode == GuidewireMode::None &&
							!spacing_recovery.active() && !spacing_recovery.requested &&
							!planned_return.active() &&
							startup.completed;
						if (!prerequisites_ok)
						{
							std::cout << "UI：力过渡实验启动被拒绝：前置条件未满足（需 控制激活 + 已标零 + ADS新鲜 + 非暂停 + 非急停 + 导管Follow + 无换手任务 + 启动完成）。" << std::endl;
						}
						else
						{
							const bool transition_log_started = experiment_recorder.is_recording()
								&& experiment_recorder.start_force_transition_log();
							if (!ft_exp.start(ctx, ft_exp.pending_cfg()))
							{
								std::cout << "UI：力过渡实验启动被拒绝：" << ft_exp.last_error() << std::endl;
								if (transition_log_started) experiment_recorder.stop_force_transition_log();
							}
							else
							{
								cooperative_direction_requested = CooperativeDirection::None;
								std::cout << "UI：力过渡实验已启动"
									<< (transition_log_started ? "，专用 CSV 写入当前会话。" : "（当前无统一记录会话，不写专用 CSV）。")
									<< std::endl;
							}
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
					if (experiment_recorder.force_transition_log_running())
					{
						experiment_recorder.stop_force_transition_log();
					}
					break;
				}
				default:
					break;
				}
			}
		}
	}

	AdsOutputCommand shutdown_output{};
	shutdown_output.motion_enabled = false;
	shutdown_output.cylinder_valid = false;
	shutdown_output.cylinder5_press_req = false;
	shutdown_output.axis4_forward_req = false;
	shutdown_output.axis4_reverse_req = false;
	shutdown_output.startup_smoothing_bypass = false;
	ads_communication.publish_output(shutdown_output);
	AdsFastSnapshot shutdown_snapshot{};
	(void)ads_communication.wait_for_snapshot(ads_snapshot_sequence, 50, shutdown_snapshot);
	clear_force_output();
	experiment_recorder.stop_and_wait("program_exit");
	tcp_force_daq.stop();
	arm_manual_ads.stop();
	ads_communication.stop();
	vis_server.stop();
	handle_axis1.close();
	handle_axis6.close();
	return 0;
}
