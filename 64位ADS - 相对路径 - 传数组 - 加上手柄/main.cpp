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

// Historical project notes still mention right-limit based coordinates.
// The current runtime path uses G.leftlimit and derives all startup/crawl
// windows from left-limit referenced absolute positions.

// Clamp a scalar into a closed interval.
double clamp_double(double value, double low, double high)
{
	if (value < low) return low;
	if (value > high) return high;
	return value;
}

// Tolerance-aware range check used for window activation and arrival checks.
bool is_within_range(double value, double low, double high, double tol = 0.0)
{
	return (value >= (low - tol)) && (value <= (high + tol));
}

// Capture one handle pose in the same short window so translation/rotation baselines stay aligned.
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

// Capture both handles in the same time window so the shared resync baseline is coherent.
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

// Small helper for PLC mirrored arrays and refer buffers.
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
	const char* axis1_fast_return = "G.axis1_fast_return";
	const char* axis6_fast_retract = "G.axis6_fast_retract";
	const char* axis4_fwd_req = "G.axis4_fwd_req";
	const char* axis4_rev_req = "G.axis4_rev_req";
	const char* axis4_manual_busy = "G.axis4_manual_busy";
	const char* axis4_manual_done = "G.axis4_manual_done";
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
	// Hand motion scaling and sign conventions.
	double k_handle_to_mm = 500.0 * (75.0 / 50.0);
	double axis_push_sign = -1.0;
	double axis_rot_scale_deg = Rad;
	double axis2_rot_reengage_deadband_deg = 1.0;

	// Force feedback is only applied on handle 582.
	int axial_force_axis = 1;
	double axial_force_sign = -1.0;

	// Button mapping derived from buttons2 bit masks.
	unsigned char btn_b0 = 0x01;
	unsigned char btn_b5 = 0x20;
	unsigned char btn_b6 = 0x40;
	unsigned char btn_b7 = 0x80;

	// Left-limit referenced crawl windows.
	double axis1_window_left_from_left_mm = 6.0;
	double axis1_window_right_from_left_mm = 26.0;
	double axis6_independent_window_size_mm = 20.0;
	double axis56_ready_gap_mm = 20.0;
	double axis3_delivery_stop_from_left_mm = 20.0;
	double axis3_delivery_release_hysteresis_mm = 2.0;
	double guidewire_entry_axis6_from_left_max_mm = 665.0;

	// Crawl trigger / arrival thresholds.
	double crawl_trigger_deadband_mm = 0.3;
	double crawl_rearm_threshold_mm = 0.3;
	double crawl_arrive_tol_mm = 0.2;
	double hold_recover_rearm_mm = 0.6;

	// Clamp action timing in the upper computer state machine.
	DWORD crawl_cylinder_action_delay_ms = 350;
	DWORD crawl_both_clamp_settle_ms = 50;

	// PLC-planned fast return tuning.
	double axis1_return_velocity_mm_s = 100.0;
	double axis1_return_acc_mm_s2 = 400.0;
	double axis1_return_dec_mm_s2 = 1200.0;
	double axis1_return_jerk_mm_s3 = 2000.0;
	DWORD axis1_return_settle_hold_ms = 80;
	double axis6_return_velocity_mm_s = 50.0;
	double axis6_return_acc_mm_s2 = 200.0;
	double axis6_return_dec_mm_s2 = 200.0;
	double axis6_return_jerk_mm_s3 = 2000.0;
	DWORD axis6_return_settle_hold_ms = 80;

	// Axis4 manual velocity mode.
	double axis4_manual_velocity_rad_s = 1.0;
	double axis4_manual_acc_rad_s2 = 5.0;
	double axis4_manual_dec_rad_s2 = 5.0;
	double axis4_manual_jerk_rad_s3 = 20.0;

	// Handle low-pass filtering.
	double linear_handle_alpha = 0.25;
	double rotational_handle_alpha = 0.20;
	bool cooperative_debug_log = false;

	// Startup preparation targets.
	DWORD startup_clamp_settle_delay_ms = 300;
	double startup_motion_speed_scale = 0.5;
	unsigned short startup_cyl3_open = 500;
	unsigned short startup_cyl4_open = 0;
	unsigned short startup_cyl3_clamp = 0;
	unsigned short startup_cyl4_clamp = 1000;
	double startup_axis5_ready_from_left_mm = 290.0;
	double startup_axis3_ready_from_left_mm = 635.0;
	// Trigger cylinder2 clamp before axis3 is fully at target; tune on site to match ~0.5 s lead.
	double startup_axis3_cyl2_clamp_advance_mm = 60.0;
};

struct CylinderPreset
{
	// The naming follows the current wiring:
	// cyl1/cyl2 belong to the catheter-side crawl pair,
	// cyl3/cyl4 belong to the guidewire-side crawl pair.
	unsigned short cyl1_open = 1000;
	unsigned short cyl1_clamp = 100;
	unsigned short cyl2_open = 0;
	unsigned short cyl2_clamp = 1000;
	unsigned short cyl3_open = 500;
	unsigned short cyl3_clamp = 0;
	unsigned short cyl4_open = 0;
	unsigned short cyl4_clamp = 500;
	unsigned short cyl3_follow_release = 150;
	unsigned short cyl4_follow_release = 100;
};

enum class GuidewireMode
{
	None,
	Independent,
	Cooperative
};

// Startup preparation is an explicit upper-computer sequence entered after self-check.
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
	// Follow: direct hand mapping inside the active window.
	// Switch/Clamp/Restore: clamp timing wrapper around a fast retract/advance move.
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
	double handle_ref = 0.0;
	double rot_ref = 0.0;
	double base_rel = 0.0;
	double rot_base_rel = 0.0;
	double start_abs = 0.0;
	double end_abs = 0.0;
	double target_abs = 0.0;
	bool plc_move_requested = false;
	DWORD phase_t0 = 0;

	double min_abs() const { return (start_abs < end_abs) ? start_abs : end_abs; }
	double max_abs() const { return (start_abs > end_abs) ? start_abs : end_abs; }
};

struct ForceFeedbackState
{
	// Keep filtering and validity state local so force can be disabled without
	// polluting the motion control state.
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

struct StartupState
{
	// Hold positions are captured once when the startup sequence begins.
	// move_base_rel is the follow baseline for the second startup motion segment.
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

// Require the operator to re-apply motion in the same direction before the next crawl cycle.
void check_crawl_rearm(CrawlState& crawl, double hand_delta_mm, double threshold_mm)
{
	if (!crawl.wait_rearm)
	{
		return;
	}

	const bool same_dir_push = (crawl.rearm_dir > 0) && (hand_delta_mm > threshold_mm);
	const bool same_dir_pull = (crawl.rearm_dir < 0) && (hand_delta_mm < -threshold_mm);
	if (same_dir_push || same_dir_pull)
	{
		crawl.wait_rearm = false;
	}
}

// Force feedback is intentionally independent from motion generation. When motion is frozen,
// held or not yet active, the force output is forced back to zero without touching motion state.
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

} // namespace

int main(int argc, char* argv[])
{
	const DWORD serial_axis1_handle = 582;
	const DWORD serial_axis6_handle = 587;
	const char* hardcoded_ads_netid = "169.254.119.135.1.1";

	// Utility mode: inspect button bit masks without opening the motion loop.
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

	// Utility mode: dump raw handle state continuously for diagnostics.
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
	// Handle587 states with base 0x06:
	// 0x07 -> cooperative mode entry
	// 0x47 -> cooperative mode reverse
	// Axis4 jog target states on handle 582:
	// forward  ~= 0x86 (b7 on, b5 off, base 0x06)
	// reverse  ~= 0x26 (b5 on, b7 off, base 0x06)
	// release  ~= 0x06
	// b0 can coexist (0x87/0x27) and should not block jog.
	const unsigned char axis4_buttons_base_mask = 0x06;
	const unsigned char axis4_buttons_forward_mask = 0x80;
	const unsigned char axis4_buttons_reverse_mask = 0x20;

	// Long-running runtime objects.
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

	// Force is only rendered on the catheter-side handle.
	auto apply_cmd_force = [&](double cmd_force)
	{
		handle_axis1.setforce_axis(cmd_force * cfg.axial_force_sign, cfg.axial_force_axis, 0.0);
	};

	// PLC-mirrored state arrays. These are refreshed from ADS before generating a new refer frame.
	double pos[7] = { 0 };
	double plc_act_pos[7] = { 0 };
	double plc_init_pos[7] = { 0 };
	double plc_leftlimit[7] = { 0 };
	double plc_act_pos_from_left[7] = { 0 };
	double plc_refer_from_left[7] = { 0 };
	double plc_v_limit[7] = { 0 };

	// Read the minimum state required by the upper-computer motion loop.
	auto read_plc_state = [&]() -> bool
	{
		bool ok = true;
		ok = ads.ADSRead(AdsSymbol::act_pos, sizeof(plc_act_pos), plc_act_pos) && ok;
		ok = ads.ADSRead(AdsSymbol::init_pos, sizeof(plc_init_pos), plc_init_pos) && ok;
		ok = ads.ADSRead(AdsSymbol::leftlimit, sizeof(plc_leftlimit), plc_leftlimit) && ok;
		ok = ads.ADSRead(AdsSymbol::act_pos_from_left, sizeof(plc_act_pos_from_left), plc_act_pos_from_left) && ok;
		ok = ads.ADSRead(AdsSymbol::refer_from_left, sizeof(plc_refer_from_left), plc_refer_from_left) && ok;
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
		return ads.ADSWrite(AdsSymbol::v_limit, sizeof(plc_v_limit), const_cast<double*>(values));
	};

	auto load_pos_from_actual = [&]()
	{
		copy_positions(plc_act_pos, pos, 7);
	};

	auto from_left_to_abs = [&](int axis_index, double from_left_mm) -> double
	{
		return plc_leftlimit[axis_index] + from_left_mm;
	};

	auto from_left_to_rel = [&](int axis_index, double from_left_mm) -> double
	{
		return from_left_to_abs(axis_index, from_left_mm) - plc_init_pos[axis_index];
	};

	auto read_axis_return_status = [&](const AxisReturnAdsSymbols& symbols, AxisReturnStatus& status) -> bool
	{
		bool ok = true;
		ok = ads.ADSRead(symbols.busy, sizeof(status.busy), &status.busy) && ok;
		ok = ads.ADSRead(symbols.done, sizeof(status.done), &status.done) && ok;
		ok = ads.ADSRead(symbols.error, sizeof(status.error), &status.error) && ok;
		ok = ads.ADSRead(symbols.error_id, sizeof(status.error_id), &status.error_id) && ok;
		return ok;
	};

	auto clear_axis_return_request = [&](const AxisReturnAdsSymbols& symbols) -> bool
	{
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
		bool ok = true;
		ok = clear_axis_return_request(AdsSymbol::axis1_return) && ok;
		ok = clear_axis_return_request(AdsSymbol::axis3_return) && ok;
		ok = clear_axis_return_request(AdsSymbol::axis5_return) && ok;
		ok = clear_axis_return_request(AdsSymbol::axis6_return) && ok;
		return ok;
	};

	auto write_axis4_manual_requests = [&](bool forward_req, bool reverse_req) -> bool
	{
		bool ok = true;
		ok = ads.ADSWrite(AdsSymbol::axis4_fwd_req, sizeof(forward_req), &forward_req) && ok;
		ok = ads.ADSWrite(AdsSymbol::axis4_rev_req, sizeof(reverse_req), &reverse_req) && ok;
		return ok;
	};

	// Follow baselines for the normal catheter mode.
	double axis3_base_rel = 0.0;
	double axis5_base_rel = 0.0;
	double axis6_mirror_base_rel = 0.0;
	double axis1_return_entry_rel = 0.0;
	double axis1_return_settle_rel = 0.0;
	double axis6_return_entry_rel = 0.0;
	double axis6_return_settle_rel = 0.0;
	bool axis6_follow_recenter_required = false;
	double axis1_return_hold_axis3_rel = 0.0;
	double axis1_return_hold_axis5_rel = 0.0;
	double axis1_return_hold_axis6_rel = 0.0;
	double axis2_hold_rel = 0.0;
	bool axis2_rot_reengage_required = false;
	double axis2_rot_reengage_ref = 0.0;
	double axis7_hold_rel = 0.0;
	bool axis7_rot_reengage_required = false;
	double axis7_rot_reengage_ref = 0.0;

	GuidewireMode guidewire_mode = GuidewireMode::None;
	double independent_axis1_hold_rel = 0.0;
	double independent_axis2_hold_rel = 0.0;
	double independent_axis3_hold_rel = 0.0;
	double independent_axis5_hold_rel = 0.0;
	bool axis6_window_locked = false;
	double axis6_locked_window_start_abs = 0.0;
	double axis6_locked_window_end_abs = 0.0;
	bool axis6_coop_ff_inited = false;
	double axis6_coop_ff_offset_mm = 0.0;
	double axis6_coop_prev_hand_delta_mm = 0.0;
	double axis6_coop_prev_axis1_cmd_abs = 0.0;
	DWORD axis6_coop_prev_t_ms = 0;

	StartupState startup;
	ForceFeedbackState ff;
	CrawlState axis1_crawl;
	CrawlState axis6_crawl;
	axis1_crawl.enabled = true;

	// Axis1 uses a fixed left-limit referenced window. Axis6 captures its window dynamically.
	auto axis1_window_left_abs = [&]() -> double
	{
		return from_left_to_abs(0, cfg.axis1_window_left_from_left_mm);
	};

	auto axis1_window_right_abs = [&]() -> double
	{
		return from_left_to_abs(0, cfg.axis1_window_right_from_left_mm);
	};

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

	auto lock_axis6_window_from_current = [&]()
	{
		axis6_window_locked = true;
		axis6_locked_window_start_abs = axis6_crawl.start_abs;
		axis6_locked_window_end_abs = axis6_crawl.end_abs;
	};

	auto apply_locked_axis6_window = [&]()
	{
		axis6_crawl.start_abs = axis6_locked_window_start_abs;
		axis6_crawl.end_abs = axis6_locked_window_end_abs;
	};

	// Resync only the catheter-side crawl state after a mode edge or a completed crawl cycle.
	auto sync_axis1 = [&](int samples, bool wait_rearm, int rearm_dir) -> bool
	{
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
		axis1_crawl.wait_rearm = wait_rearm;
		axis1_crawl.rearm_dir = rearm_dir;

		axis2_hold_rel = preserved_axis2_hold_rel;
		axis2_rot_reengage_required = wait_rearm;
		axis2_rot_reengage_ref = axis1_handle_filter.axis1_filtered;

		axis3_base_rel = plc_act_pos[2];
		axis5_base_rel = plc_act_pos[4];
		axis6_mirror_base_rel = plc_act_pos[5];

		return write_refer();
	};

	// Resync only the guidewire-side crawl state. The window can optionally be rebuilt from the
	// current axis6 absolute position when entering independent mode.
	auto sync_axis6 = [&](int samples, bool capture_window, bool wait_rearm, int rearm_dir) -> bool
	{
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
		axis6_crawl.base_rel = plc_act_pos[5];
		axis6_crawl.rot_base_rel = preserved_axis7_hold_rel;
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
		axis6_crawl.target_abs = axis6_crawl.end_abs;
		axis6_crawl.plc_move_requested = false;
		axis6_crawl.window_active = is_within_range(
			plc_act_pos[5] + plc_init_pos[5],
			axis6_crawl.min_abs(),
			axis6_crawl.max_abs(),
			cfg.crawl_arrive_tol_mm);
		axis6_crawl.phase = CrawlState::Phase::Follow;
		axis6_crawl.phase_t0 = GetTickCount();
		axis6_crawl.wait_rearm = wait_rearm;
		axis6_crawl.rearm_dir = rearm_dir;
		axis6_follow_recenter_required = wait_rearm;
		axis6_crawl.enabled = true;
		axis6_coop_ff_inited = false;
		axis6_coop_ff_offset_mm = 0.0;
		axis6_coop_prev_hand_delta_mm = 0.0;
		axis6_coop_prev_axis1_cmd_abs = 0.0;
		axis6_coop_prev_t_ms = 0;

		axis7_hold_rel = preserved_axis7_hold_rel;
		axis7_rot_reengage_required = wait_rearm;
		axis7_rot_reengage_ref = axis6_handle_filter.axis1_filtered;

		return write_refer();
	};

	// Global resync aligns both handles and all mirrored baselines to the current PLC position.
	auto sync_all = [&](int samples) -> bool
	{
		const double preserved_axis2_hold_rel = axis2_hold_rel;
		const double preserved_axis7_hold_rel = axis7_hold_rel;
		clear_axis1_group_return_requests();
		clear_axis_return_request(AdsSymbol::axis6_return);
		write_axis4_manual_requests(false, false);

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

		axis2_hold_rel = preserved_axis2_hold_rel;
		axis2_rot_reengage_required = true;
		axis2_rot_reengage_ref = axis1_handle_filter.axis1_filtered;

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
		axis6_follow_recenter_required = false;
		axis6_window_locked = false;
		axis6_coop_ff_inited = false;
		axis6_coop_ff_offset_mm = 0.0;
		axis6_coop_prev_hand_delta_mm = 0.0;
		axis6_coop_prev_axis1_cmd_abs = 0.0;
		axis6_coop_prev_t_ms = 0;

		axis7_hold_rel = preserved_axis7_hold_rel;
		axis7_rot_reengage_required = true;
		axis7_rot_reengage_ref = axis6_handle_filter.axis1_filtered;

		return true;
	};

	auto clear_plc_reinit_req = [&]()
	{
		bool clear_val = false;
		ads.ADSWrite(AdsSymbol::handle_reinit_req, sizeof(clear_val), &clear_val);
	};

	// Refresh the normal-mode follow baseline when axis1 first enters its crawl window.
	auto capture_axis1_follow_baseline = [&]()
	{
		axis1_crawl.handle_ref = axis1_handle_filter.axis0_filtered;
		axis1_crawl.rot_ref = axis1_handle_filter.axis1_filtered;
		axis1_crawl.base_rel = plc_act_pos[0];
		axis1_crawl.rot_base_rel = axis2_hold_rel;
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

	// Enter independent guidewire mode while freezing the catheter-side axes at their current
	// relative positions. Axis7 stays active and is resynced to the current hand twist.
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
		axis7_rot_reengage_required = false;
		axis7_rot_reengage_ref = axis6_handle_filter.axis1_filtered;

		get_average_handle_pose(handle_axis6, 20, axis6_crawl.handle_ref, axis6_crawl.rot_ref);
		axis6_handle_filter.reset(axis6_crawl.handle_ref, axis6_crawl.rot_ref);
		axis6_crawl.base_rel = plc_act_pos[5];
		axis6_crawl.rot_base_rel = axis7_hold_rel;
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
		axis6_follow_recenter_required = false;
		axis6_coop_ff_inited = false;
		axis6_coop_ff_offset_mm = 0.0;
		axis6_coop_prev_hand_delta_mm = 0.0;
		axis6_coop_prev_axis1_cmd_abs = 0.0;
		axis6_coop_prev_t_ms = 0;

		pos[0] = plc_act_pos[0];
		pos[1] = axis2_hold_rel;
		pos[2] = plc_act_pos[2];
		pos[4] = plc_act_pos[4];
		pos[5] = plc_act_pos[5];
		pos[6] = axis7_hold_rel;
		return write_refer();
	};

	// Cooperative mode reuses the axis6 crawl state machine but keeps the catheter chain active.
	auto enter_cooperative_guidewire_mode = [&]() -> bool
	{
		return sync_axis6(20, true, false, 0);
	};

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
		axis6_coop_ff_offset_mm = 0.0;
		axis6_coop_prev_hand_delta_mm = 0.0;
		axis6_coop_prev_axis1_cmd_abs = 0.0;
		axis6_coop_prev_t_ms = 0;
		return sync_all(20);
	};

	// Startup preparation is an upper-computer sequence layered on top of PLC self-check.
	// It temporarily scales down v_limit on axes 3/5/6 and restores it after completion.
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
		axis6_follow_recenter_required = false;
		axis6_window_locked = false;
		axis6_coop_ff_inited = false;
		axis6_coop_ff_offset_mm = 0.0;
		axis6_coop_prev_hand_delta_mm = 0.0;
		axis6_coop_prev_axis1_cmd_abs = 0.0;
		axis6_coop_prev_t_ms = 0;
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

	// Restore PLC motion limits after startup or after an interrupted startup path.
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

	// Initial PLC snapshot seeds the hold positions before the interactive loop begins.
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
	axis2_rot_reengage_ref = axis1_handle_filter.axis1_filtered;
	axis7_hold_rel = plc_act_pos[6];
	axis7_rot_reengage_required = false;
	axis7_rot_reengage_ref = axis6_handle_filter.axis1_filtered;
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
	bool axis1_fast_return = false;
	bool axis6_fast_retract = false;
	AxisReturnStatus axis1_return_status;
	AxisReturnStatus axis6_return_status;
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

	while (true)
	{
		// 1) Sample both handles and derive edge-triggered button states.
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
		axis1_fast_return = false;
		axis6_fast_retract = false;

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

		// 2) Resync crawl baselines when reverse-crawl buttons change state.
		if (!freeze_active && !estop_hold_active && !startup_sequence_active && control_active)
		{
			if (guidewire_mode == GuidewireMode::None && axis1_reverse_pressed != axis1_reverse_pressed_prev)
			{
				if (sync_axis1(20, false, 0))
				{
					std::cout << "Axis1 catheter retract mode: " << (axis1_reverse_pressed ? "ON" : "OFF") << std::endl;
				}
				else
				{
					std::cout << "Axis1 catheter retract mode sync failed." << std::endl;
				}
			}

			if ((guidewire_mode == GuidewireMode::Independent || guidewire_mode == GuidewireMode::Cooperative) &&
				axis6_effective_reverse_pressed != axis6_effective_reverse_prev)
			{
				if (sync_axis6(20, false, false, 0))
				{
					std::cout << "Axis6 reverse crawl mode: " << (axis6_effective_reverse_pressed ? "ON" : "OFF") << std::endl;
				}
				else
				{
					std::cout << "Axis6 reverse crawl mode sync failed." << std::endl;
				}
			}
		}
		axis1_reverse_pressed_prev = axis1_reverse_pressed;
		axis6_effective_reverse_prev = axis6_effective_reverse_pressed;

		// 3) Poll PLC-side hold state and keep upper-computer force output disabled during holds.
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

		// 4) Keyboard side channel: choose direct control / startup preparation / force toggle.
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

		// 5) Guidewire mode is edge-triggered from handle 587 (cooperative=b0, independent=b6) and re-synced on transitions.
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
							axis6_follow_recenter_required = false;
							axis6_window_locked = false;
							axis6_coop_ff_inited = false;
							axis6_coop_ff_offset_mm = 0.0;
							axis6_coop_prev_hand_delta_mm = 0.0;
							axis6_coop_prev_axis1_cmd_abs = 0.0;
							axis6_coop_prev_t_ms = 0;
							std::cout << "Guidewire mode exit failed: ADS sync failed." << std::endl;
						}
					}
					else
					{
						guidewire_mode = GuidewireMode::None;
						axis6_crawl.enabled = false;
						axis6_follow_recenter_required = false;
						axis6_window_locked = false;
						axis6_coop_ff_inited = false;
						axis6_coop_ff_offset_mm = 0.0;
						axis6_coop_prev_hand_delta_mm = 0.0;
						axis6_coop_prev_axis1_cmd_abs = 0.0;
						axis6_coop_prev_t_ms = 0;
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

		// 6) Periodically react to PLC self-check completion and PLC-requested resync.
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
					axis6_follow_recenter_required = false;
					axis6_window_locked = false;
					axis6_coop_ff_inited = false;
					axis6_coop_ff_offset_mm = 0.0;
					axis6_coop_prev_hand_delta_mm = 0.0;
					axis6_coop_prev_axis1_cmd_abs = 0.0;
					axis6_coop_prev_t_ms = 0;
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

		// 7) Human-readable button diagnostics stay on every buttons2 change.
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

		// 8) When startup has already completed but control is not active, recover with a full resync.
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

		unsigned short cylinder1_cmd = cyl.cyl1_open;
		unsigned short cylinder2_cmd = cyl.cyl2_clamp;
		unsigned short cylinder3_cmd = cyl.cyl3_follow_release;
		unsigned short cylinder4_cmd = cyl.cyl4_follow_release;
		bool axis4_manual_forward_req = axis4_reverse_request;
		bool axis4_manual_reverse_req = axis4_forward_request;

		// 9) Build one refer frame and one cylinder command set from the current top-level mode.
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

			const double axis1_abs = plc_act_pos[0] + plc_init_pos[0];
			const double axis3_abs = plc_act_pos[2] + plc_init_pos[2];
			const double axis1_base_abs = axis1_crawl.base_rel + plc_init_pos[0];
			const double axis1_hand_delta_mm =
				(axis1_linear_filtered - axis1_crawl.handle_ref) * cfg.k_handle_to_mm * cfg.axis_push_sign;
			const double axis1_window_left_abs_now = axis1_crawl.start_abs;
			const double axis1_window_right_abs_now = axis1_crawl.end_abs;
			const double axis1_min_abs = axis1_crawl.min_abs();
			const double axis1_max_abs = axis1_crawl.max_abs();
			const double axis3_from_left_mm = axis3_abs - plc_leftlimit[2];
			const bool axis3_delivery_stop_active =
				axis3_from_left_mm <= (cfg.axis3_delivery_stop_from_left_mm + cfg.crawl_arrive_tol_mm);

			const double axis6_abs = plc_act_pos[5] + plc_init_pos[5];
			const double axis6_base_abs = axis6_crawl.base_rel + plc_init_pos[5];
			const double axis6_hand_delta_mm =
				(axis6_linear_filtered - axis6_crawl.handle_ref) * cfg.k_handle_to_mm * cfg.axis_push_sign;
			const double axis6_window_left_abs_now = axis6_crawl.start_abs;
			const double axis6_window_right_abs_now = axis6_crawl.end_abs;

			auto hold_axis1_mirror_axes_for_return = [&]()
			{
				pos[2] = axis1_return_hold_axis3_rel;
				pos[4] = axis1_return_hold_axis5_rel;
				pos[5] = axis1_return_hold_axis6_rel;
			};

			auto compute_axis7_cmd_rel = [&]() -> double
			{
				// After a resync or mode edge, ignore small hand twists until the operator
				// intentionally exceeds the re-engage deadband.
				if (axis7_rot_reengage_required)
				{
					const double axis7_rot_delta_deg =
						(axis6_rot_filtered - axis7_rot_reengage_ref) * cfg.axis_rot_scale_deg;
					if (std::abs(axis7_rot_delta_deg) >= cfg.axis2_rot_reengage_deadband_deg)
					{
						axis7_rot_reengage_required = false;
						axis6_crawl.rot_ref = axis6_rot_filtered;
						axis6_crawl.rot_base_rel = axis7_hold_rel;
					}
					return axis7_hold_rel;
				}

				const double axis7_follow_rel =
					axis6_crawl.rot_base_rel + (axis6_rot_filtered - axis6_crawl.rot_ref) * cfg.axis_rot_scale_deg;
				axis7_hold_rel = axis7_follow_rel;
				return axis7_follow_rel;
			};

			const double axis7_cmd_rel =
				(guidewire_mode == GuidewireMode::None) ? axis7_hold_rel : compute_axis7_cmd_rel();
			pos[6] = axis7_cmd_rel;

			// Shared axis6 crawl state machine used by both independent and cooperative guidewire modes.
			auto run_axis6_crawl_state = [&](double axis6_raw_cmd_abs, double axis6_delta_mm, bool axis6_push_request, bool axis6_pull_request, bool axis6_reverse_mode)
			{
				if (axis6_crawl.phase == CrawlState::Phase::Follow)
				{
					double axis6_cmd_abs = axis6_base_abs;

					// After a planned return resync, require the operator to recenter the guidewire handle
					// before allowing follow commands to affect axis6 again.
					if (axis6_follow_recenter_required)
					{
						if (std::abs(axis6_delta_mm) <= cfg.crawl_trigger_deadband_mm)
						{
							axis6_follow_recenter_required = false;
						}
					}
					else
					{
						if (axis6_reverse_mode)
						{
							if (axis6_pull_request)
							{
								axis6_cmd_abs = clamp_double(axis6_raw_cmd_abs, axis6_window_left_abs_now, axis6_window_right_abs_now);
							}
						}
						else if (axis6_push_request)
						{
							axis6_cmd_abs = clamp_double(axis6_raw_cmd_abs, axis6_window_left_abs_now, axis6_window_right_abs_now);
						}
					}

					pos[5] = axis6_cmd_abs - plc_init_pos[5];
					cylinder3_cmd = cyl.cyl3_open;
					cylinder4_cmd = cyl.cyl4_clamp;

					if (!axis6_follow_recenter_required && !axis6_crawl.wait_rearm)
					{
						if (!axis6_reverse_mode &&
							axis6_push_request &&
							(std::abs(axis6_abs - axis6_window_left_abs_now) <= cfg.crawl_arrive_tol_mm))
						{
							axis6_crawl.target_abs = axis6_window_right_abs_now;
							axis6_return_entry_rel = plc_act_pos[5];
							axis6_crawl.phase = CrawlState::Phase::SwitchWait;
							axis6_crawl.phase_t0 = now_ms;
							axis6_crawl.rearm_dir = -1;
							axis6_crawl.plc_move_requested = false;
						}
						else if (axis6_reverse_mode &&
							axis6_pull_request &&
							(std::abs(axis6_abs - axis6_window_right_abs_now) <= cfg.crawl_arrive_tol_mm))
						{
							axis6_crawl.target_abs = axis6_window_left_abs_now;
							axis6_return_entry_rel = plc_act_pos[5];
							axis6_crawl.phase = CrawlState::Phase::SwitchWait;
							axis6_crawl.phase_t0 = now_ms;
							axis6_crawl.rearm_dir = 1;
							axis6_crawl.plc_move_requested = false;
						}
					}
				}
				else if (axis6_crawl.phase == CrawlState::Phase::SwitchWait)
				{
					axis6_fast_retract = true;
					pos[5] = axis6_return_entry_rel;
					cylinder3_cmd = cyl.cyl3_clamp;
					cylinder4_cmd = cyl.cyl4_open;
					if ((now_ms - axis6_crawl.phase_t0) >= cfg.crawl_cylinder_action_delay_ms)
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
							if (!sync_axis6(20, false, true, axis6_crawl.rearm_dir))
							{
								axis6_crawl.phase = CrawlState::Phase::Follow;
								axis6_crawl.wait_rearm = true;
								axis6_follow_recenter_required = true;
							}
						}
						else if (axis6_return_status.done)
						{
							clear_axis_return_request(AdsSymbol::axis6_return);
							axis6_crawl.plc_move_requested = false;
							axis6_return_settle_rel = axis6_crawl.target_abs - plc_init_pos[5];
							axis6_crawl.phase = CrawlState::Phase::SettleHold;
							axis6_crawl.phase_t0 = now_ms;
						}
					}
				}
				else if (axis6_crawl.phase == CrawlState::Phase::SettleHold)
				{
					axis6_fast_retract = true;
					pos[5] = axis6_return_settle_rel;
					cylinder3_cmd = cyl.cyl3_clamp;
					cylinder4_cmd = cyl.cyl4_open;
					if ((now_ms - axis6_crawl.phase_t0) >= cfg.axis6_return_settle_hold_ms)
					{
						axis6_crawl.phase = CrawlState::Phase::ClampWait;
						axis6_crawl.phase_t0 = now_ms;
					}
				}
				else if (axis6_crawl.phase == CrawlState::Phase::ClampWait)
				{
					axis6_fast_retract = true;
					pos[5] = axis6_return_settle_rel;
					cylinder3_cmd = cyl.cyl3_clamp;
					cylinder4_cmd = cyl.cyl4_clamp;
					if ((now_ms - axis6_crawl.phase_t0) >= cfg.crawl_both_clamp_settle_ms)
					{
						axis6_crawl.phase = CrawlState::Phase::RestoreWait;
						axis6_crawl.phase_t0 = now_ms;
					}
				}
				else if (axis6_crawl.phase == CrawlState::Phase::RestoreWait)
				{
					axis6_fast_retract = true;
					pos[5] = axis6_return_settle_rel;
					cylinder3_cmd = cyl.cyl3_open;
					cylinder4_cmd = cyl.cyl4_clamp;
					if ((now_ms - axis6_crawl.phase_t0) >= cfg.crawl_cylinder_action_delay_ms)
					{
						if (!sync_axis6(20, false, true, axis6_crawl.rearm_dir))
						{
							std::cout << "Axis6 resync after planned return failed." << std::endl;
							axis6_crawl.phase = CrawlState::Phase::Follow;
							axis6_crawl.wait_rearm = true;
							axis6_follow_recenter_required = true;
						}
					}
				}
			};

			if (motion_startup_active)
			{
				// Startup sequence keeps axis1/2/7 fixed and only moves 3/5/6 through the preparation stages.
				pos[0] = startup.axis1_hold_rel;
				pos[1] = startup.axis2_hold_rel;
				pos[2] = startup.axis3_hold_rel;
				pos[4] = startup.axis5_hold_rel;
				pos[5] = startup.axis6_hold_rel;
				pos[6] = startup.axis7_hold_rel;

				const double startup_axis5_ready_abs = from_left_to_abs(4, cfg.startup_axis5_ready_from_left_mm);
				const double startup_axis6_ready_abs = from_left_to_abs(5, cfg.startup_axis5_ready_from_left_mm + cfg.axis56_ready_gap_mm);
				const double startup_axis3_ready_abs = from_left_to_abs(2, cfg.startup_axis3_ready_from_left_mm);
				const double axis5_abs = plc_act_pos[4] + plc_init_pos[4];
				const double axis6_abs_now = plc_act_pos[5] + plc_init_pos[5];
				const double axis3_abs = plc_act_pos[2] + plc_init_pos[2];

				if (startup.phase == StartupPhase::ReleaseClamps)
				{
					// Stage 1: open all clamps and wait for the mechanism to settle.
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
					// Stage 2: move axes 5/6 to their left-limit referenced preparation points.
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_open;
					cylinder3_cmd = cfg.startup_cyl3_open;
					cylinder4_cmd = cfg.startup_cyl4_open;
					pos[4] = from_left_to_rel(4, cfg.startup_axis5_ready_from_left_mm);
					pos[5] = from_left_to_rel(5, cfg.startup_axis5_ready_from_left_mm + cfg.axis56_ready_gap_mm);
					if ((std::abs(axis5_abs - startup_axis5_ready_abs) <= cfg.crawl_arrive_tol_mm) &&
						(std::abs(axis6_abs_now - startup_axis6_ready_abs) <= cfg.crawl_arrive_tol_mm))
					{
						startup.phase = StartupPhase::ClampCylinder34Wait;
						startup.phase_t0 = now_ms;
					}
				}
				else if (startup.phase == StartupPhase::ClampCylinder34Wait)
				{
					// Stage 3: close the guidewire-side clamp pair before the coupled 3/5/6 move.
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
					// Stage 4: axis3 moves to its ready point while axes 5/6 follow the same relative delta.
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
				// Independent guidewire mode freezes the catheter-side axes and runs axis6/7 only.
				pos[0] = independent_axis1_hold_rel;
				pos[1] = independent_axis2_hold_rel;
				pos[2] = independent_axis3_hold_rel;
				pos[4] = independent_axis5_hold_rel;
				pos[6] = axis7_cmd_rel;

				axis6_coop_ff_inited = false;
				axis6_coop_ff_offset_mm = 0.0;
				check_crawl_rearm(axis6_crawl, axis6_hand_delta_mm, cfg.crawl_rearm_threshold_mm);
				const double axis6_raw_cmd_abs = axis6_base_abs + axis6_hand_delta_mm;
				const bool axis6_push_request = axis6_hand_delta_mm < -cfg.crawl_trigger_deadband_mm;
				const bool axis6_pull_request = axis6_hand_delta_mm > cfg.crawl_trigger_deadband_mm;
				run_axis6_crawl_state(
					axis6_raw_cmd_abs,
					axis6_hand_delta_mm,
					axis6_push_request,
					axis6_pull_request,
					axis6_effective_reverse_pressed);
			}
			else
			{
				// Normal catheter mode:
				// - axis1/2 are controlled by handle 582
				// - axes 3/5 mirror the normal segment of axis1 translation
				// - axis6 mirrors axis1 only when not in cooperative guidewire mode
				// - fast retract phases are handled explicitly by the crawl state machine
				const bool cooperative_mode = (guidewire_mode == GuidewireMode::Cooperative);
				const bool axis1_now_in_window = is_within_range(axis1_abs, axis1_min_abs, axis1_max_abs, cfg.crawl_arrive_tol_mm);
				if (!axis1_crawl.window_active && axis1_now_in_window)
				{
					capture_axis1_follow_baseline();
					axis1_crawl.window_active = true;
					std::cout << "Axis1 entered crawl window; crawl logic enabled." << std::endl;
				}

				check_crawl_rearm(axis1_crawl, axis1_hand_delta_mm, cfg.crawl_rearm_threshold_mm);

				if (axis1_crawl.phase == CrawlState::Phase::Follow)
				{
					const double axis1_raw_cmd_abs = axis1_base_abs + axis1_hand_delta_mm;
					const bool axis1_push_request = axis1_hand_delta_mm < -cfg.crawl_trigger_deadband_mm;
					const bool axis1_pull_request = axis1_hand_delta_mm > cfg.crawl_trigger_deadband_mm;
					const bool axis1_drive_request = axis1_reverse_pressed ? axis1_pull_request : axis1_push_request;
					const bool axis1_follow_enabled =
						axis1_reverse_pressed || (!axis1_push_rearm_after_hold && !axis1_delivery_stop_latched);
					if (axis3_from_left_mm >
						(cfg.axis3_delivery_stop_from_left_mm + cfg.axis3_delivery_release_hysteresis_mm))
					{
						axis1_delivery_stop_prompted = false;
					}

					if (axis1_push_rearm_after_hold && axis1_hand_delta_mm > cfg.hold_recover_rearm_mm)
					{
						axis1_push_rearm_after_hold = false;
						capture_axis1_follow_baseline();
						std::cout << "Axis1 push re-armed after hold." << std::endl;
					}
					if (axis1_delivery_stop_latched && axis1_reverse_pressed && axis1_pull_request)
					{
						axis1_delivery_stop_latched = false;
						axis1_delivery_stop_prompted = false;
					}

					double axis1_cmd_abs = axis1_base_abs;
					if (axis1_drive_request && axis1_follow_enabled)
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

					pos[0] = axis1_cmd_abs - plc_init_pos[0];

					if (axis2_rot_reengage_required)
					{
						pos[1] = axis2_hold_rel;
						const double axis2_rot_delta_deg =
							(axis1_rot_filtered - axis2_rot_reengage_ref) * cfg.axis_rot_scale_deg;
						if (std::abs(axis2_rot_delta_deg) >= cfg.axis2_rot_reengage_deadband_deg)
						{
							axis2_rot_reengage_required = false;
							axis1_crawl.rot_ref = axis1_rot_filtered;
							axis1_crawl.rot_base_rel = axis2_hold_rel;
						}
					}
					else
					{
						pos[1] = axis1_crawl.rot_base_rel +
							(axis1_rot_filtered - axis1_crawl.rot_ref) * cfg.axis_rot_scale_deg;
						axis2_hold_rel = pos[1];
					}

					apply_axis1_mirror_from_abs(axis1_cmd_abs, !cooperative_mode);
					cylinder3_cmd = cyl.cyl3_follow_release;
					cylinder4_cmd = cyl.cyl4_follow_release;

					if (axis1_crawl.window_active && !axis1_crawl.wait_rearm)
					{
						const double axis1_trigger_edge_abs = axis1_reverse_pressed
							? axis1_window_right_abs_now
							: axis1_window_left_abs_now;
						const bool axis1_ready_to_trigger = axis1_drive_request &&
							(axis1_reverse_pressed || (!axis1_push_rearm_after_hold && !axis1_delivery_stop_latched)) &&
							(std::abs(axis1_abs - axis1_trigger_edge_abs) <= cfg.crawl_arrive_tol_mm);
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
							axis1_return_hold_axis3_rel = plc_act_pos[2];
							axis1_return_hold_axis5_rel = plc_act_pos[4];
							axis1_return_hold_axis6_rel = plc_act_pos[5];
							axis1_crawl.phase = CrawlState::Phase::SwitchWait;
							axis1_crawl.phase_t0 = now_ms;
								axis1_crawl.rearm_dir = axis1_reverse_pressed ? 1 : -1;
							axis1_crawl.plc_move_requested = false;
						}
					}
				}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::SwitchWait)
				{
					axis1_fast_return = true;
					pos[0] = axis1_return_entry_rel;
					hold_axis1_mirror_axes_for_return();
					pos[1] = axis2_hold_rel;
					cylinder1_cmd = cyl.cyl1_clamp;
					cylinder2_cmd = cyl.cyl2_open;
					if ((now_ms - axis1_crawl.phase_t0) >= cfg.crawl_cylinder_action_delay_ms)
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
					else
					{
						if (read_axis_return_status(AdsSymbol::axis1_return, axis1_return_status))
						{
							if (axis1_return_status.error)
							{
								clear_axis_return_request(AdsSymbol::axis1_return);
								axis1_crawl.plc_move_requested = false;
								std::cout
									<< "Axis1 planned return error: "
									<< axis1_return_status.error_id
									<< std::endl;
								if (!sync_axis1(20, true, axis1_crawl.rearm_dir))
								{
									axis1_crawl.phase = CrawlState::Phase::Follow;
									axis1_crawl.wait_rearm = true;
								}
							}
							else if (axis1_return_status.done)
							{
								clear_axis_return_request(AdsSymbol::axis1_return);
								axis1_crawl.plc_move_requested = false;
								axis1_return_settle_rel = axis1_crawl.target_abs - plc_init_pos[0];
								axis1_crawl.phase = CrawlState::Phase::SettleHold;
								axis1_crawl.phase_t0 = now_ms;
							}
						}
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::SettleHold)
				{
					// Hold a short stable window at return endpoint before switching clamp states.
					axis1_fast_return = true;
					pos[0] = axis1_return_settle_rel;
					hold_axis1_mirror_axes_for_return();
					pos[1] = axis2_hold_rel;
					cylinder1_cmd = cyl.cyl1_clamp;
					cylinder2_cmd = cyl.cyl2_open;
					if ((now_ms - axis1_crawl.phase_t0) >= cfg.axis1_return_settle_hold_ms)
					{
						axis1_crawl.phase = CrawlState::Phase::ClampWait;
						axis1_crawl.phase_t0 = now_ms;
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::ClampWait)
				{
					axis1_fast_return = true;
					pos[0] = axis1_return_settle_rel;
					hold_axis1_mirror_axes_for_return();
					pos[1] = axis2_hold_rel;
					cylinder1_cmd = cyl.cyl1_clamp;
					cylinder2_cmd = cyl.cyl2_clamp;
					if ((now_ms - axis1_crawl.phase_t0) >= cfg.crawl_both_clamp_settle_ms)
					{
						axis1_crawl.phase = CrawlState::Phase::RestoreWait;
						axis1_crawl.phase_t0 = now_ms;
					}
				}
				else if (axis1_crawl.phase == CrawlState::Phase::RestoreWait)
				{
					axis1_fast_return = true;
					pos[0] = axis1_return_settle_rel;
					hold_axis1_mirror_axes_for_return();
					pos[1] = axis2_hold_rel;
					cylinder1_cmd = cyl.cyl1_open;
					cylinder2_cmd = cyl.cyl2_clamp;
					if ((now_ms - axis1_crawl.phase_t0) >= cfg.crawl_cylinder_action_delay_ms)
					{
						if (!sync_axis1(20, true, axis1_crawl.rearm_dir))
						{
							std::cout << "Axis1 resync after planned return failed." << std::endl;
							axis1_crawl.phase = CrawlState::Phase::Follow;
							axis1_crawl.wait_rearm = true;
						}
					}
				}

				if (cooperative_mode)
				{
					// Cooperative mode keeps catheter axes on the axis1 chain, while axis6 runs
					// its own crawl state machine with velocity feedforward:
					// v6 = v587 + v1 (v1 only in axis1 Follow phase).
					const double axis1_cmd_abs_for_ff = pos[0] + plc_init_pos[0];

					if (axis6_crawl.phase == CrawlState::Phase::Follow)
					{
						if (!axis6_coop_ff_inited)
						{
							axis6_coop_ff_inited = true;
							axis6_coop_ff_offset_mm = 0.0;
							axis6_coop_prev_hand_delta_mm = axis6_hand_delta_mm;
							axis6_coop_prev_axis1_cmd_abs = axis1_cmd_abs_for_ff;
							axis6_coop_prev_t_ms = now_ms;
						}
						else
						{
							const DWORD dt_ms = now_ms - axis6_coop_prev_t_ms;
							if (dt_ms > 0 && dt_ms <= 200)
							{
								const double dt_s = static_cast<double>(dt_ms) / 1000.0;
								const double v587_mm_s =
									(axis6_hand_delta_mm - axis6_coop_prev_hand_delta_mm) / dt_s;
								double v1_mm_s = 0.0;
								if (axis1_crawl.phase == CrawlState::Phase::Follow)
								{
									v1_mm_s = (axis1_cmd_abs_for_ff - axis6_coop_prev_axis1_cmd_abs) / dt_s;
								}

								axis6_coop_ff_offset_mm += (v587_mm_s + v1_mm_s) * dt_s;

								if (cfg.cooperative_debug_log && ((loop_count % 50) == 0))
								{
									std::cout
										<< "[COOP] v587=" << v587_mm_s
										<< " v1=" << v1_mm_s
										<< " off=" << axis6_coop_ff_offset_mm
										<< " axis1_phase=" << static_cast<int>(axis1_crawl.phase)
										<< std::endl;
								}
							}
						}

						axis6_coop_prev_hand_delta_mm = axis6_hand_delta_mm;
						axis6_coop_prev_axis1_cmd_abs = axis1_cmd_abs_for_ff;
						axis6_coop_prev_t_ms = now_ms;

						const double axis6_combined_delta_mm = axis6_coop_ff_offset_mm;
						check_crawl_rearm(axis6_crawl, axis6_combined_delta_mm, cfg.crawl_rearm_threshold_mm);
						const double axis6_raw_cmd_abs = axis6_base_abs + axis6_combined_delta_mm;
						const bool axis6_push_request = axis6_combined_delta_mm < -cfg.crawl_trigger_deadband_mm;
						const bool axis6_pull_request = axis6_combined_delta_mm > cfg.crawl_trigger_deadband_mm;

						run_axis6_crawl_state(
							axis6_raw_cmd_abs,
							axis6_combined_delta_mm,
							axis6_push_request,
							axis6_pull_request,
							axis6_effective_reverse_pressed);
					}
					else
					{
						// axis6 return phases ignore axis1 contribution by design.
						axis6_coop_ff_inited = false;
						run_axis6_crawl_state(
							axis6_base_abs + axis6_coop_ff_offset_mm,
							axis6_coop_ff_offset_mm,
							false,
							false,
							axis6_effective_reverse_pressed);
					}
				}
			}

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

		// 10) Cylinders are only driven while motion is active; the fast-return flags are always written.
		if (!freeze_active && (control_active || motion_startup_active))
		{
			ads.ADSWrite(AdsSymbol::cylinder1_value, sizeof(cylinder1_cmd), &cylinder1_cmd);
			ads.ADSWrite(AdsSymbol::cylinder2_value, sizeof(cylinder2_cmd), &cylinder2_cmd);
			ads.ADSWrite(AdsSymbol::cylinder3_value, sizeof(cylinder3_cmd), &cylinder3_cmd);
			ads.ADSWrite(AdsSymbol::cylinder4_value, sizeof(cylinder4_cmd), &cylinder4_cmd);
		}

		write_axis4_manual_requests(axis4_manual_forward_req, axis4_manual_reverse_req);
		ads.ADSWrite(AdsSymbol::axis1_fast_return, sizeof(axis1_fast_return), &axis1_fast_return);
		ads.ADSWrite(AdsSymbol::axis6_fast_retract, sizeof(axis6_fast_retract), &axis6_fast_retract);
	}

	handle_axis1.close();
	handle_axis6.close();
	return 0;
}
