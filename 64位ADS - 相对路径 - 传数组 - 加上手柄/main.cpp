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

int hand_dir_from_delta(double delta_mm, double deadband_mm)
{
    if (delta_mm > deadband_mm) return 1;
    if (delta_mm < -deadband_mm) return -1;
    return 0;
}

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
    DWORD phase_t0 = 0;
};

bool phase_is_follow(const CrawlState& state)
{
    return state.phase == CrawlState::Phase::Follow;
}

void copy_positions(const double* src, double* dst, int count)
{
    for (int i = 0; i < count; ++i)
    {
        dst[i] = src[i];
    }
}
}

int main(int argc, char* argv[])
{
    const DWORD serial_axis1_handle = 582;
    const DWORD serial_axis6_handle = 587;
    const char* hardcoded_ads_netid = "169.254.119.135.1.1";

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

    const int axial_force_axis = 1;
    const double axial_force_sign = -1.0;

    const double k_handle_to_mm = 500.0 * (75.0 / 50.0);
    const double axis_push_sign = -1.0;
    const double axis_rot_scale_deg = Rad;
    const unsigned char button_mask_b0 = 0x01;
    const unsigned char button_mask_b6 = 0x40;
    const unsigned char axis1_pause_button_mask = button_mask_b6 | button_mask_b0;
    const unsigned char axis6_arm_button_mask = button_mask_b6 | button_mask_b0;

    const double crawl_window_start_offset_mm = 56.0;
    const double crawl_window_size_mm = 30.0;
    const double crawl_trigger_deadband_mm = 0.3;
    const double crawl_rearm_threshold_mm = 0.3;
    const double crawl_arrive_tol_mm = 0.2;
    const DWORD crawl_switch_delay_ms = 250;
    const DWORD crawl_clamp_delay_ms = 50;

    const unsigned short cyl1_open = 1000;
    const unsigned short cyl1_clamp = 100;
    const unsigned short cyl2_open = 0;
    const unsigned short cyl2_clamp = 1000;
    const unsigned short cyl3_open = 1000;
    const unsigned short cyl3_clamp = 100;
    const unsigned short cyl4_open = 0;
    const unsigned short cyl4_clamp = 1000;
    const unsigned short cyl3_follow_release = 150;
    const unsigned short cyl4_follow_release = 100;

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

    auto apply_cmd_force = [&](double cmd_force)
    {
        handle_axis1.setforce_axis(cmd_force * axial_force_sign, axial_force_axis, 0.0);
    };

    double pos[7] = { 0 };
    double plc_act_pos[7] = { 0 };
    double plc_init_pos[7] = { 0 };
    double plc_rightlimit[7] = { 0 };

    auto read_plc_state = [&]() -> bool
    {
        bool ok = true;
        ok = ads.ADSRead((char*)"G.Act_pos", sizeof(plc_act_pos), plc_act_pos) && ok;
        ok = ads.ADSRead((char*)"G.init_pos", sizeof(plc_init_pos), plc_init_pos) && ok;
        ok = ads.ADSRead((char*)"G.rightlimit", sizeof(plc_rightlimit), plc_rightlimit) && ok;
        return ok;
    };

    auto write_refer = [&]() -> bool
    {
        return ads.ADSWrite((char*)"G.refer", sizeof(pos), pos);
    };

    auto load_pos_from_actual = [&]()
    {
        copy_positions(plc_act_pos, pos, 7);
    };

    double axis3_base_rel = 0.0;
    double axis5_base_rel = 0.0;
    double axis6_mirror_base_rel = 0.0;

    CrawlState axis1_crawl;
    CrawlState axis6_crawl;
    axis1_crawl.enabled = true;

    auto axis1_start_abs = [&]() -> double
    {
        return plc_rightlimit[0] - crawl_window_start_offset_mm;
    };

    auto axis1_end_abs = [&]() -> double
    {
        // 留出修改接口：终点 = 起点 + 运动窗口大小，当前窗口为 30 mm。
        return axis1_start_abs() + crawl_window_size_mm;
    };

    auto sync_axis1 = [&](int samples, bool wait_rearm, int rearm_dir) -> bool
    {
        if (!read_plc_state())
        {
            return false;
        }

        load_pos_from_actual();

        axis1_crawl.handle_ref = get_average_pos(handle_axis1, 0, samples);
        axis1_crawl.rot_ref = handle_axis1.fJoints2[1];
        axis1_crawl.base_rel = plc_act_pos[0];
        axis1_crawl.rot_base_rel = plc_act_pos[1];
        axis1_crawl.start_abs = axis1_start_abs();
        axis1_crawl.end_abs = axis1_end_abs();
        axis1_crawl.window_active = is_within_range(
            plc_act_pos[0] + plc_init_pos[0],
            (axis1_crawl.start_abs < axis1_crawl.end_abs) ? axis1_crawl.start_abs : axis1_crawl.end_abs,
            (axis1_crawl.start_abs > axis1_crawl.end_abs) ? axis1_crawl.start_abs : axis1_crawl.end_abs,
            crawl_arrive_tol_mm);
        axis1_crawl.phase = CrawlState::Phase::Follow;
        axis1_crawl.phase_t0 = GetTickCount();
        axis1_crawl.wait_rearm = wait_rearm;
        axis1_crawl.rearm_dir = rearm_dir;

        axis3_base_rel = plc_act_pos[2];
        axis5_base_rel = plc_act_pos[4];
        axis6_mirror_base_rel = plc_act_pos[5];

        return write_refer();
    };

    auto sync_axis6 = [&](int samples, bool capture_window, bool wait_rearm, int rearm_dir) -> bool
    {
        if (!read_plc_state())
        {
            return false;
        }

        load_pos_from_actual();

        axis6_crawl.handle_ref = get_average_pos(handle_axis6, 0, samples);
        axis6_crawl.rot_ref = handle_axis6.fJoints2[1];
        axis6_crawl.base_rel = plc_act_pos[5];
        axis6_crawl.rot_base_rel = plc_act_pos[6];
        if (capture_window || !axis6_crawl.enabled)
        {
            axis6_crawl.start_abs = plc_act_pos[5] + plc_init_pos[5];
            axis6_crawl.end_abs = axis6_crawl.start_abs + crawl_window_size_mm;
        }
        axis6_crawl.window_active = true;
        axis6_crawl.phase = CrawlState::Phase::Follow;
        axis6_crawl.phase_t0 = GetTickCount();
        axis6_crawl.wait_rearm = wait_rearm;
        axis6_crawl.rearm_dir = rearm_dir;
        axis6_crawl.enabled = true;

        return write_refer();
    };

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

        if (axis1_crawl.phase == CrawlState::Phase::Follow)
        {
            const double axis1_base_abs = axis1_crawl.base_rel + plc_init_pos[0];
            const double axis1_min_abs = (axis1_crawl.start_abs < axis1_crawl.end_abs) ? axis1_crawl.start_abs : axis1_crawl.end_abs;
            const double axis1_max_abs = (axis1_crawl.start_abs > axis1_crawl.end_abs) ? axis1_crawl.start_abs : axis1_crawl.end_abs;
            const double axis1_hand_delta_mm =
                (handle_axis1.fJoints2[0] - axis1_crawl.handle_ref) * k_handle_to_mm * axis_push_sign;
            const double axis1_cmd_abs = clamp_double(axis1_base_abs + axis1_hand_delta_mm, axis1_min_abs, axis1_max_abs);
            const double axis1_delta_rel = axis1_cmd_abs - plc_init_pos[0] - axis1_crawl.base_rel;

            // 退出独立控制时，把轴6镜像基准校到当前点，避免恢复随动瞬间跳变。
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

    auto sync_all = [&](int samples) -> bool
    {
        if (!read_plc_state())
        {
            return false;
        }

        load_pos_from_actual();
        if (!write_refer())
        {
            return false;
        }

        // 两个手柄同步取样，避免像之前那样先采 582、再采 587，
        // 导致 582 的零位比真正进入控制早约 300 ms，首帧产生假位移。
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
        if (!write_refer())
        {
            return false;
        }

        axis1_crawl.base_rel = plc_act_pos[0];
        axis1_crawl.rot_base_rel = plc_act_pos[1];
        axis1_crawl.start_abs = axis1_start_abs();
        axis1_crawl.end_abs = axis1_end_abs();
        axis1_crawl.window_active = is_within_range(
            plc_act_pos[0] + plc_init_pos[0],
            (axis1_crawl.start_abs < axis1_crawl.end_abs) ? axis1_crawl.start_abs : axis1_crawl.end_abs,
            (axis1_crawl.start_abs > axis1_crawl.end_abs) ? axis1_crawl.start_abs : axis1_crawl.end_abs,
            crawl_arrive_tol_mm);
        axis1_crawl.phase = CrawlState::Phase::Follow;
        axis1_crawl.phase_t0 = GetTickCount();
        axis1_crawl.wait_rearm = false;
        axis1_crawl.rearm_dir = 0;
        axis1_crawl.enabled = true;

        axis3_base_rel = plc_act_pos[2];
        axis5_base_rel = plc_act_pos[4];
        axis6_mirror_base_rel = plc_act_pos[5];

        axis6_crawl.base_rel = plc_act_pos[5];
        axis6_crawl.rot_base_rel = plc_act_pos[6];
        axis6_crawl.start_abs = plc_act_pos[5] + plc_init_pos[5];
        axis6_crawl.end_abs = axis6_crawl.start_abs + crawl_window_size_mm;
        axis6_crawl.window_active = false;
        axis6_crawl.phase = CrawlState::Phase::Follow;
        axis6_crawl.phase_t0 = GetTickCount();
        axis6_crawl.wait_rearm = false;
        axis6_crawl.rearm_dir = 0;
        axis6_crawl.enabled = false;

        return true;
    };

    auto clear_plc_reinit_req = [&]()
    {
        const bool clear_val = false;
        ads.ADSWrite((char*)"G.handle_reinit_req", sizeof(clear_val), (void*)&clear_val);
    };

    auto capture_axis1_follow_baseline = [&]()
    {
        axis1_crawl.handle_ref = handle_axis1.fJoints2[0];
        axis1_crawl.rot_ref = handle_axis1.fJoints2[1];
        axis1_crawl.base_rel = plc_act_pos[0];
        axis1_crawl.rot_base_rel = plc_act_pos[1];
        axis3_base_rel = plc_act_pos[2];
        axis5_base_rel = plc_act_pos[4];
        if (!axis6_crawl.enabled)
        {
            axis6_mirror_base_rel = plc_act_pos[5];
        }
    };

    if (!read_plc_state())
    {
        std::cout << "Failed to read PLC state." << std::endl;
        handle_axis1.close();
        handle_axis6.close();
        return 0;
    }

    load_pos_from_actual();
    write_refer();

    bool self_check_done = true;
    bool has_self_check_flag = ads.ADSRead((char*)"G.self_check_done", sizeof(self_check_done), &self_check_done);
    if (has_self_check_flag)
    {
        ads.ADSRead((char*)"G.self_check_done", sizeof(self_check_done), &self_check_done);
    }

    bool control_active = !has_self_check_flag || self_check_done;
    bool last_self_check_done = self_check_done;
    bool handle_reinit_req = false;
    bool estop_hold_req = false;
    bool estop_hold_active = false;
    bool freeze_active = false;
    bool pause_pressed_prev = false;
    bool axis6_arm_pressed_prev = false;
    bool axis1_fast_return = false;
    bool axis6_fast_retract = false;
    int loop_count = 0;

    unsigned char last_btn_axis1 = 0xFF;
    unsigned char last_btn_axis6 = 0xFF;

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
    bool force_feedback_enabled = false;

    std::cout << "Force feedback: OFF (press Enter to toggle)" << std::endl;
    apply_cmd_force(0.0);

    if (!has_self_check_flag || self_check_done)
    {
        if (sync_all(30))
        {
            control_active = true;
        }
    }

    while (true)
    {
        handle_axis1.poll();
        handle_axis6.poll();

        ++loop_count;
        axis1_fast_return = false;
        axis6_fast_retract = false;

        const bool pause_pressed = (handle_axis1.buttons2 & axis1_pause_button_mask) != 0;
        const bool axis6_arm_pressed = (handle_axis6.buttons2 & axis6_arm_button_mask) != 0;

        if (axis6_arm_pressed && !axis6_arm_pressed_prev)
        {
            if (freeze_active)
            {
                std::cout << "Axis6 crawl arm ignored: freeze is active." << std::endl;
            }
            else if (estop_hold_active)
            {
                std::cout << "Axis6 crawl arm ignored: PLC hold is active." << std::endl;
            }
            else if (!control_active)
            {
                std::cout << "Axis6 crawl arm ignored: control is not active yet." << std::endl;
            }
            else if (sync_axis6(20, true, false, 0))
            {
                std::cout << "Axis6 crawl armed at current position." << std::endl;
            }
            else
            {
                std::cout << "Axis6 crawl arm failed: ADS sync failed." << std::endl;
            }
        }
        else if (!axis6_arm_pressed && axis6_arm_pressed_prev && axis6_crawl.enabled)
        {
            if (release_axis6_to_follow())
            {
                std::cout << "Axis6 crawl released to follow mode." << std::endl;
            }
        }
        axis6_arm_pressed_prev = axis6_arm_pressed;

        if (pause_pressed && !pause_pressed_prev)
        {
            freeze_active = true;
            control_active = false;
            std::cout << "Handle582 pause: ON." << std::endl;
        }
        else if (!pause_pressed && pause_pressed_prev)
        {
            freeze_active = false;
            if (!estop_hold_active && sync_all(20))
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

        if ((loop_count % 10) == 0)
        {
            if (ads.ADSRead((char*)"G.estop_hold_req", sizeof(estop_hold_req), &estop_hold_req))
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

        if (freeze_active)
        {
            fn_bias_inited = false;
            fn_force_f = 0.0;
            ft_force_f = 0.0;
            apply_cmd_force(0.0);
        }

        if (_kbhit())
        {
            const int ch = _getch();
            if (ch == '\r')
            {
                force_feedback_enabled = !force_feedback_enabled;
                fn_bias_inited = false;
                fn_force_f = 0.0;
                ft_force_f = 0.0;
                fn_invalid_streak = 0;
                ft_invalid_streak = 0;
                std::cout << "Force feedback: " << (force_feedback_enabled ? "ON" : "OFF") << std::endl;
                if (!force_feedback_enabled)
                {
                    apply_cmd_force(0.0);
                }
            }
            else if (ch == 0 || ch == 224)
            {
                _getch();
            }
        }

        if (has_self_check_flag && (loop_count % 50) == 0)
        {
            if (ads.ADSRead((char*)"G.self_check_done", sizeof(self_check_done), &self_check_done))
            {
                if (!last_self_check_done && self_check_done)
                {
                    if (!freeze_active && !estop_hold_active && sync_all(30))
                    {
                        control_active = true;
                    }
                }
                last_self_check_done = self_check_done;
            }

            if (ads.ADSRead((char*)"G.handle_reinit_req", sizeof(handle_reinit_req), &handle_reinit_req))
            {
                if (handle_reinit_req)
                {
                    if (!freeze_active && !estop_hold_active)
                    {
                        sync_all(30);
                    }
                    clear_plc_reinit_req();
                }
            }
        }

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

        if (!estop_hold_active &&
            force_feedback_enabled &&
            ads.ADSRead((char*)"G.fn_value", sizeof(fn_raw), &fn_raw) &&
            ads.ADSRead((char*)"G.ft_value", sizeof(ft_raw), &ft_raw))
        {
            const bool fn_valid = (std::abs(static_cast<int>(fn_raw)) <= 8000);
            const bool ft_valid = (std::abs(static_cast<int>(ft_raw)) <= 8000);

            if (fn_valid)
            {
                fn_last_valid = fn_raw;
                fn_has_valid = true;
                fn_invalid_streak = 0;
            }
            else
            {
                ++fn_invalid_streak;
            }

            if (ft_valid)
            {
                ft_last_valid = ft_raw;
                ft_has_valid = true;
                ft_invalid_streak = 0;
            }
            else
            {
                ++ft_invalid_streak;
            }

            if (fn_has_valid)
            {
                fn_raw = fn_last_valid;
            }
            if (ft_has_valid)
            {
                ft_raw = ft_last_valid;
            }

            if (!fn_bias_inited)
            {
                fn_bias = static_cast<double>(fn_raw);
                fn_bias_inited = true;
            }
            else
            {
                const double fn_err = static_cast<double>(fn_raw) - fn_bias;
                if (std::abs(fn_err) <= 120.0)
                {
                    fn_bias = fn_bias * 0.995 + static_cast<double>(fn_raw) * 0.005;
                }
            }

            double fn_zeroed = static_cast<double>(fn_raw) - fn_bias;
            if (std::abs(fn_zeroed) < 20.0)
            {
                fn_zeroed = 0.0;
            }

            const double axial_gain = 1.0 / 1000.0;
            const double axial_limit = 6.0;
            const double axial_force = clamp_double(fn_zeroed * axial_gain, -axial_limit, axial_limit);

            double torque_force = 0.0;
            if (ft_raw >= -870 && ft_raw <= -700)
            {
                torque_force = 0.0;
            }
            else if (ft_raw > -700)
            {
                torque_force = (static_cast<double>(ft_raw) + 700.0) * (-1.0 / 600.0);
            }
            else
            {
                torque_force = (static_cast<double>(ft_raw) + 870.0) / (-530.0);
            }
            torque_force = clamp_double(torque_force, -1.0, 1.0);

            if (control_active)
            {
                fn_force_f = fn_force_f * 0.7 + (axial_force * 0.3);
                ft_force_f = ft_force_f * 0.7 + (torque_force * 0.3);
                handle_axis1.setforce_axis(fn_force_f * axial_force_sign, axial_force_axis, ft_force_f);
            }
            else
            {
                apply_cmd_force(0.0);
            }

            if ((loop_count % 100) == 0 || fn_raw != last_fn_raw || ft_raw != last_ft_raw)
            {
                last_fn_raw = fn_raw;
                last_ft_raw = ft_raw;
                std::cout
                    << "fn_raw=" << fn_raw << " fn_bias=" << fn_bias
                    << " fn_cmd=" << fn_force_f
                    << " | ft_raw=" << ft_raw << " ft_cmd=" << ft_force_f
                    << " | fn_inv=" << fn_invalid_streak << " ft_inv=" << ft_invalid_streak
                    << std::endl;
            }
        }
        else
        {
            fn_force_f = 0.0;
            ft_force_f = 0.0;
            if (!freeze_active)
            {
                apply_cmd_force(0.0);
            }
        }

        if (!control_active && !freeze_active && !estop_hold_active)
        {
            if (sync_all(20))
            {
                std::cout << "Re-synced" << std::endl;
                control_active = true;
            }
        }

        unsigned short cylinder1_cmd = cyl1_open;
        unsigned short cylinder2_cmd = cyl2_clamp;
        unsigned short cylinder3_cmd = cyl3_follow_release;
        unsigned short cylinder4_cmd = cyl4_follow_release;

        if (control_active && read_plc_state())
        {
            load_pos_from_actual();

            const DWORD now_ms = GetTickCount();

            const double axis1_abs = plc_act_pos[0] + plc_init_pos[0];
            const double axis1_min_abs = (axis1_crawl.start_abs < axis1_crawl.end_abs) ? axis1_crawl.start_abs : axis1_crawl.end_abs;
            const double axis1_max_abs = (axis1_crawl.start_abs > axis1_crawl.end_abs) ? axis1_crawl.start_abs : axis1_crawl.end_abs;
            const bool axis1_now_in_window = is_within_range(axis1_abs, axis1_min_abs, axis1_max_abs, crawl_arrive_tol_mm);
            if (!axis1_crawl.window_active && axis1_now_in_window)
            {
                capture_axis1_follow_baseline();
                axis1_crawl.window_active = true;
                std::cout << "Axis1 entered crawl window; crawl logic enabled." << std::endl;
            }
            const double axis1_base_abs = axis1_crawl.base_rel + plc_init_pos[0];
            const double axis1_hand_delta_mm = (handle_axis1.fJoints2[0] - axis1_crawl.handle_ref) * k_handle_to_mm * axis_push_sign;
            const int axis1_hand_dir = hand_dir_from_delta(axis1_hand_delta_mm, crawl_trigger_deadband_mm);
            const bool axis6_independent = axis6_crawl.enabled;

            if (axis1_crawl.wait_rearm)
            {
                const bool same_dir_push = (axis1_crawl.rearm_dir > 0) && (axis1_hand_delta_mm > crawl_rearm_threshold_mm);
                const bool same_dir_pull = (axis1_crawl.rearm_dir < 0) && (axis1_hand_delta_mm < -crawl_rearm_threshold_mm);
                const bool reverse_dir_push = (axis1_crawl.rearm_dir < 0) && (axis1_hand_delta_mm > crawl_rearm_threshold_mm);
                const bool reverse_dir_pull = (axis1_crawl.rearm_dir > 0) && (axis1_hand_delta_mm < -crawl_rearm_threshold_mm);
                if (same_dir_push || same_dir_pull || reverse_dir_push || reverse_dir_pull)
                {
                    axis1_crawl.wait_rearm = false;
                }
            }

            if (axis1_crawl.phase == CrawlState::Phase::Follow)
            {
                const double axis1_raw_cmd_abs = axis1_base_abs + axis1_hand_delta_mm;
                const double axis1_cmd_abs = axis1_crawl.window_active
                    ? clamp_double(axis1_raw_cmd_abs, axis1_min_abs, axis1_max_abs)
                    : axis1_raw_cmd_abs;
                const double axis1_delta_rel = axis1_cmd_abs - plc_init_pos[0] - axis1_crawl.base_rel;
                pos[0] = axis1_cmd_abs - plc_init_pos[0];
                pos[1] = axis1_crawl.rot_base_rel + (handle_axis1.fJoints2[1] - axis1_crawl.rot_ref) * axis_rot_scale_deg;
                pos[2] = axis3_base_rel + axis1_delta_rel;
                pos[4] = axis5_base_rel + axis1_delta_rel;

                if (!axis6_independent)
                {
                    pos[5] = axis6_mirror_base_rel + axis1_delta_rel;
                    cylinder3_cmd = cyl3_follow_release;
                    cylinder4_cmd = cyl4_follow_release;
                }

                if (axis1_crawl.window_active && !axis1_crawl.wait_rearm)
                {
                    if ((axis1_hand_dir > 0) && (std::abs(axis1_abs - axis1_crawl.end_abs) <= crawl_arrive_tol_mm))
                    {
                        axis1_crawl.target_abs = axis1_crawl.start_abs;
                        axis1_crawl.phase = CrawlState::Phase::SwitchWait;
                        axis1_crawl.phase_t0 = now_ms;
                        axis1_crawl.rearm_dir = 1;
                    }
                    else if ((axis1_hand_dir < 0) && (std::abs(axis1_abs - axis1_crawl.start_abs) <= crawl_arrive_tol_mm))
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
                cylinder1_cmd = cyl1_clamp;
                cylinder2_cmd = cyl2_open;
                if ((now_ms - axis1_crawl.phase_t0) >= crawl_switch_delay_ms)
                {
                    axis1_crawl.phase = CrawlState::Phase::FastMove;
                    axis1_crawl.phase_t0 = now_ms;
                }
            }
            else if (axis1_crawl.phase == CrawlState::Phase::FastMove)
            {
                cylinder1_cmd = cyl1_clamp;
                cylinder2_cmd = cyl2_open;
                axis1_fast_return = true;
                pos[0] = axis1_crawl.target_abs - plc_init_pos[0];
                if (std::abs(axis1_abs - axis1_crawl.target_abs) <= crawl_arrive_tol_mm)
                {
                    axis1_crawl.phase = CrawlState::Phase::ClampWait;
                    axis1_crawl.phase_t0 = now_ms;
                }
            }
            else if (axis1_crawl.phase == CrawlState::Phase::ClampWait)
            {
                cylinder1_cmd = cyl1_clamp;
                cylinder2_cmd = cyl2_clamp;
                if ((now_ms - axis1_crawl.phase_t0) >= crawl_clamp_delay_ms)
                {
                    axis1_crawl.phase = CrawlState::Phase::RestoreWait;
                    axis1_crawl.phase_t0 = now_ms;
                }
            }
            else if (axis1_crawl.phase == CrawlState::Phase::RestoreWait)
            {
                cylinder1_cmd = cyl1_open;
                cylinder2_cmd = cyl2_clamp;
                if ((now_ms - axis1_crawl.phase_t0) >= crawl_switch_delay_ms)
                {
                    sync_axis1(20, true, axis1_crawl.rearm_dir);
                }
            }

            const double axis6_abs = plc_act_pos[5] + plc_init_pos[5];
            const double axis6_min_abs = (axis6_crawl.start_abs < axis6_crawl.end_abs) ? axis6_crawl.start_abs : axis6_crawl.end_abs;
            const double axis6_max_abs = (axis6_crawl.start_abs > axis6_crawl.end_abs) ? axis6_crawl.start_abs : axis6_crawl.end_abs;
            const double axis6_base_abs = axis6_crawl.base_rel + plc_init_pos[5];
            const double axis6_hand_delta_mm = (handle_axis6.fJoints2[0] - axis6_crawl.handle_ref) * k_handle_to_mm * axis_push_sign;
            const int axis6_hand_dir = hand_dir_from_delta(axis6_hand_delta_mm, crawl_trigger_deadband_mm);

            if (axis6_crawl.enabled)
            {
                if (axis6_crawl.wait_rearm)
                {
                    const bool same_dir_push = (axis6_crawl.rearm_dir > 0) && (axis6_hand_delta_mm > crawl_rearm_threshold_mm);
                    const bool same_dir_pull = (axis6_crawl.rearm_dir < 0) && (axis6_hand_delta_mm < -crawl_rearm_threshold_mm);
                    const bool reverse_dir_push = (axis6_crawl.rearm_dir < 0) && (axis6_hand_delta_mm > crawl_rearm_threshold_mm);
                    const bool reverse_dir_pull = (axis6_crawl.rearm_dir > 0) && (axis6_hand_delta_mm < -crawl_rearm_threshold_mm);
                    if (same_dir_push || same_dir_pull || reverse_dir_push || reverse_dir_pull)
                    {
                        axis6_crawl.wait_rearm = false;
                    }
                }

                if (axis6_crawl.phase == CrawlState::Phase::Follow)
                {
                    const double axis6_cmd_abs = clamp_double(axis6_base_abs + axis6_hand_delta_mm, axis6_min_abs, axis6_max_abs);
                    pos[5] = axis6_cmd_abs - plc_init_pos[5];
                    pos[6] = axis6_crawl.rot_base_rel + (handle_axis6.fJoints2[1] - axis6_crawl.rot_ref) * axis_rot_scale_deg;
                    cylinder3_cmd = cyl3_open;
                    cylinder4_cmd = cyl4_clamp;

                    if (!axis6_crawl.wait_rearm)
                    {
                        if ((axis6_hand_dir > 0) && (std::abs(axis6_abs - axis6_crawl.end_abs) <= crawl_arrive_tol_mm))
                        {
                            axis6_crawl.target_abs = axis6_crawl.start_abs;
                            axis6_crawl.phase = CrawlState::Phase::SwitchWait;
                            axis6_crawl.phase_t0 = now_ms;
                            axis6_crawl.rearm_dir = 1;
                        }
                        else if ((axis6_hand_dir < 0) && (std::abs(axis6_abs - axis6_crawl.start_abs) <= crawl_arrive_tol_mm))
                        {
                            axis6_crawl.target_abs = axis6_crawl.end_abs;
                            axis6_crawl.phase = CrawlState::Phase::SwitchWait;
                            axis6_crawl.phase_t0 = now_ms;
                            axis6_crawl.rearm_dir = -1;
                        }
                    }
                }
                else if (axis6_crawl.phase == CrawlState::Phase::SwitchWait)
                {
                    cylinder3_cmd = cyl3_clamp;
                    cylinder4_cmd = cyl4_open;
                    if ((now_ms - axis6_crawl.phase_t0) >= crawl_switch_delay_ms)
                    {
                        axis6_crawl.phase = CrawlState::Phase::FastMove;
                        axis6_crawl.phase_t0 = now_ms;
                    }
                }
                else if (axis6_crawl.phase == CrawlState::Phase::FastMove)
                {
                    cylinder3_cmd = cyl3_clamp;
                    cylinder4_cmd = cyl4_open;
                    axis6_fast_retract = true;
                    pos[5] = axis6_crawl.target_abs - plc_init_pos[5];
                    if (std::abs(axis6_abs - axis6_crawl.target_abs) <= crawl_arrive_tol_mm)
                    {
                        axis6_crawl.phase = CrawlState::Phase::ClampWait;
                        axis6_crawl.phase_t0 = now_ms;
                    }
                }
                else if (axis6_crawl.phase == CrawlState::Phase::ClampWait)
                {
                    cylinder3_cmd = cyl3_clamp;
                    cylinder4_cmd = cyl4_clamp;
                    if ((now_ms - axis6_crawl.phase_t0) >= crawl_clamp_delay_ms)
                    {
                        axis6_crawl.phase = CrawlState::Phase::RestoreWait;
                        axis6_crawl.phase_t0 = now_ms;
                    }
                }
                else if (axis6_crawl.phase == CrawlState::Phase::RestoreWait)
                {
                    cylinder3_cmd = cyl3_open;
                    cylinder4_cmd = cyl4_clamp;
                    if ((now_ms - axis6_crawl.phase_t0) >= crawl_switch_delay_ms)
                    {
                        sync_axis6(20, false, true, axis6_crawl.rearm_dir);
                    }
                }
            }

            write_refer();
        }

        if (!freeze_active)
        {
            ads.ADSWrite((char*)"G.cylinder1_value", sizeof(cylinder1_cmd), &cylinder1_cmd);
            ads.ADSWrite((char*)"G.cylinder2_value", sizeof(cylinder2_cmd), &cylinder2_cmd);
            ads.ADSWrite((char*)"G.cylinder3_value", sizeof(cylinder3_cmd), &cylinder3_cmd);
            ads.ADSWrite((char*)"G.cylinder4_value", sizeof(cylinder4_cmd), &cylinder4_cmd);
        }

        ads.ADSWrite((char*)"G.axis1_fast_return", sizeof(axis1_fast_return), &axis1_fast_return);
        ads.ADSWrite((char*)"G.axis6_fast_retract", sizeof(axis6_fast_retract), &axis6_fast_retract);
    }

    handle_axis1.close();
    handle_axis6.close();
    return 0;
}
