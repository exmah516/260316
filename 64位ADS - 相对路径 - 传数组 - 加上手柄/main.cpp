#include "Handle.h"
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <conio.h>
#include <ADSComm1.h>

// Helper to get average position
double get_average_pos(Handle& h, int axis, int samples) {
    double sum = 0;
    for (int i = 0; i < samples; ++i) {
        h.showinfo();
        sum += h.fJoints2[axis];
        Sleep(10);
    }
    return sum / samples;
}

static double clamp_double(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int main(int argc, char* argv[])
{
    // --- Button Test Mode ---
    if (argc > 1 && (std::string(argv[1]) == "--buttons" || std::string(argv[1]) == "--btn")) {
        Handle h;
        if (!h.init()) {
            std::cout << "Handle Init Failed." << std::endl;
            return 0;
        }
        std::cout << "=== Button Test Mode ===" << std::endl;
        std::cout << "Press buttons to see their bits." << std::endl;
        std::cout << "Press ESC or 'q' to exit." << std::endl;
        
        unsigned char last_btn = 0;
        while (true) {
            h.showinfo();
            unsigned char cur_btn = h.buttons2;
            
            if (cur_btn != last_btn) {
                std::cout << "Btns: 0x" << std::hex << (int)cur_btn << std::dec << " | Bits: ";
                for(int i=0; i<8; ++i) std::cout << ((cur_btn >> i) & 1);
                std::cout << std::endl;
                last_btn = cur_btn;
            }
            
            if (_kbhit()) {
                int ch = _getch();
                if (ch == 27 || ch == 'q') break;
            }
            Sleep(10);
        }
        h.close();
        return 0;
    }

    const int axial_force_axis = 1;
    const double axial_force_sign = -1.0;

    Handle handle;
    CADSComm a;

    if (!handle.init()) {
         std::cout << "Handle Init Failed." << std::endl;
         return 0;
    }

    Sleep(1000);
    handle.showinfo();

    if (!a.OpenComm_inside())
    {
        std::cout << "ADS Open Failed (Local). Error: " << a.GetLastError() << std::endl;
        if (!a.OpenComm())
        {
            std::cout << "ADS Open Failed (Hardcoded). Error: " << a.GetLastError() << std::endl;
            return 0;
        }
    }

    // --- Force helper ---
    auto apply_cmd_force = [&](double cmd_force) {
        handle.setforce_axis(cmd_force * axial_force_sign, axial_force_axis, 0.0);
    };
    
    // --- Initialization ---
    double pos[7] = { 0 };
    double plc_act_pos[7] = { 0 };
    
    // Read Initial Motor Position from PLC to avoid jump
    // We assume index 5 of array corresponds to the controlled axis (based on previous pos[5] usage)
    // Note: G.Act_pos is 1-based in PLC (1..7). In C++ Read, if we read array:
    // element 0 -> Axis 1 ... element 4 -> Axis 5 ... element 5 -> Axis 6? 
    // Previous code used pos[5] to write. pos[5] is the 6th element. 
    // Let's assume ADS mapping is consistent 0->1. So pos[5] -> Axis 6?.
    // User said "Value 1 for Axis 5". 
    // Let's read the WHOLE G.Act_pos array.
    a.ADSRead((char*)"G.Act_pos", sizeof(plc_act_pos), &plc_act_pos);
    for (int i = 0; i < 7; ++i) pos[i] = plc_act_pos[i];
    a.ADSWrite((char*)"G.refer", sizeof(pos), &pos);
    
    double handle_ref = get_average_pos(handle, 0, 30);
    double handle_rot_ref = handle.fJoints2[1];

    double axis1_base_rel = plc_act_pos[0];
    double axis2_base_rel = plc_act_pos[1];
    double axis3_base_rel = plc_act_pos[2];
    double axis5_base_rel = plc_act_pos[4];
    double axis6_base_rel = plc_act_pos[5];

    // Cylinder Vars
    unsigned short cylinder1_cmd = 0;
    unsigned short cylinder2_cmd = 0;
    unsigned short cylinder3_cmd = 0;
    unsigned short cylinder4_cmd = 0;
    a.ADSRead((char*)"G.cylinder1_value", sizeof(cylinder1_cmd), &cylinder1_cmd);
    a.ADSRead((char*)"G.cylinder2_value", sizeof(cylinder2_cmd), &cylinder2_cmd);
    a.ADSRead((char*)"G.cylinder3_value", sizeof(cylinder3_cmd), &cylinder3_cmd);
    a.ADSRead((char*)"G.cylinder4_value", sizeof(cylinder4_cmd), &cylinder4_cmd);

    bool self_check_done = true;
    bool has_self_check_flag = a.ADSRead((char*)"G.self_check_done", sizeof(self_check_done), &self_check_done);
    if (has_self_check_flag) {
        a.ADSRead((char*)"G.self_check_done", sizeof(self_check_done), &self_check_done);
    }

    bool control_active = !has_self_check_flag || self_check_done;
    unsigned char last_btn = 0xFF;
    int loop_count = 0;
    bool last_self_check_done = self_check_done;
    bool handle_reinit_req = false;
    bool estop_hold_req = false;
    bool estop_hold_active = false;

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

    double plc_init_pos[7] = { 0 };
    double plc_rightlimit[7] = { 0 };

    const double k_handle_to_mm = 500.0 * (75.0 / 50.0);
    const double axis1_push_sign = -1.0;
    const double axis1_start_offset_mm = 56.0;
    const double axis1_end_offset_mm = 36.0;
    const double axis1_tol_mm = 0.2;
    const double axis1_trigger_deadband_mm = 0.5;
    const DWORD axis1_clamp_delay_ms = 50;
    const DWORD axis1_open_delay_ms = 250;

    const unsigned short cyl1_open = 1000;
    const unsigned short cyl1_clamp = 100;
    const unsigned short cyl2_open = 0;
    const unsigned short cyl2_clamp = 1000;
    const unsigned short cyl3_open = 1000;
    const unsigned short cyl3_clamp = 100;
    const unsigned short cyl4_open = 0;
    const unsigned short cyl4_clamp = 1000;

    cylinder2_cmd = cyl2_clamp;
    cylinder4_cmd = cyl4_open;
    a.ADSWrite((char*)"G.cylinder2_value", sizeof(cylinder2_cmd), &cylinder2_cmd);
    a.ADSWrite((char*)"G.cylinder4_value", sizeof(cylinder4_cmd), &cylinder4_cmd);

    bool axis1_fast_return = false;
    bool axis1_pause_mirror = false;
    enum class Axis1Phase { Follow, Open2Wait, FastMove, Clamp2Wait, Open1Wait };
    Axis1Phase axis1_phase = Axis1Phase::Follow;
    DWORD axis1_phase_t0 = GetTickCount();
    double axis1_target_abs = 0.0;

    bool pause_pressed_prev = false;
    bool freeze_active = false;
    int axis1_expected_dir = 1;

    class ControlSync {
    public:
        ControlSync(
            CADSComm& ads,
            Handle& handle,
            double* pos,
            double* act_pos,
            double* init_pos,
            double* rightlimit,
            double& handle_ref,
            double& handle_rot_ref,
            double& axis1_base_rel,
            double& axis2_base_rel,
            double& axis3_base_rel,
            double& axis5_base_rel,
            double& axis6_base_rel,
            bool& axis1_pause_mirror,
            Axis1Phase& axis1_phase,
            DWORD& axis1_phase_t0,
            double& axis1_target_abs,
            double axis1_start_offset_mm,
            double axis1_end_offset_mm,
            int& axis1_expected_dir)
            : ads_(ads),
              handle_(handle),
              pos_(pos),
              act_pos_(act_pos),
              init_pos_(init_pos),
              rightlimit_(rightlimit),
              handle_ref_(handle_ref),
              handle_rot_ref_(handle_rot_ref),
              axis1_base_rel_(axis1_base_rel),
              axis2_base_rel_(axis2_base_rel),
              axis3_base_rel_(axis3_base_rel),
              axis5_base_rel_(axis5_base_rel),
              axis6_base_rel_(axis6_base_rel),
              axis1_pause_mirror_(axis1_pause_mirror),
              axis1_phase_(axis1_phase),
              axis1_phase_t0_(axis1_phase_t0),
              axis1_target_abs_(axis1_target_abs),
              axis1_start_offset_mm_(axis1_start_offset_mm),
              axis1_end_offset_mm_(axis1_end_offset_mm),
              axis1_expected_dir_(axis1_expected_dir) {}

        bool Sync(int handle_samples) {
            if (!ads_.ADSRead((char*)"G.Act_pos", sizeof(double) * 7, act_pos_)) return false;
            for (int i = 0; i < 7; ++i) pos_[i] = act_pos_[i];
            ads_.ADSWrite((char*)"G.refer", sizeof(double) * 7, pos_);
            ads_.ADSRead((char*)"G.init_pos", sizeof(double) * 7, init_pos_);
            ads_.ADSRead((char*)"G.rightlimit", sizeof(double) * 7, rightlimit_);

            handle_ref_ = get_average_pos(handle_, 0, handle_samples);
            handle_rot_ref_ = handle_.fJoints2[1];
            axis1_base_rel_ = act_pos_[0];
            axis2_base_rel_ = act_pos_[1];
            axis3_base_rel_ = act_pos_[2];
            axis5_base_rel_ = act_pos_[4];
            axis6_base_rel_ = act_pos_[5];

            axis1_pause_mirror_ = false;
            axis1_phase_ = Axis1Phase::Follow;
            axis1_phase_t0_ = GetTickCount();
            axis1_target_abs_ = rightlimit_[0] - axis1_start_offset_mm_;
            const double start_abs = rightlimit_[0] - axis1_start_offset_mm_;
            const double end_abs = rightlimit_[0] - axis1_end_offset_mm_;
            const double axis1_abs = act_pos_[0] + init_pos_[0];
            if (std::abs(axis1_abs - start_abs) <= std::abs(axis1_abs - end_abs)) {
                axis1_expected_dir_ = (end_abs >= start_abs) ? 1 : -1;
            } else {
                axis1_expected_dir_ = (start_abs >= end_abs) ? 1 : -1;
            }
            return true;
        }

        bool RequestPlcReinitAndSync(int handle_samples, DWORD timeout_ms) {
            const bool req = true;
            ads_.ADSWrite((char*)"G.handle_reinit_req", sizeof(req), (void*)&req);
            const DWORD t0 = GetTickCount();
            bool flag = true;
            bool done = false;
            const bool can_read_done = ads_.ADSRead((char*)"G.handle_reinit_done", sizeof(done), &done);
            while ((GetTickCount() - t0) < timeout_ms) {
                if (can_read_done) {
                    if (!ads_.ADSRead((char*)"G.handle_reinit_done", sizeof(done), &done)) break;
                    if (done) break;
                } else {
                    if (!ads_.ADSRead((char*)"G.handle_reinit_req", sizeof(flag), &flag)) break;
                    if (!flag) break;
                }
                Sleep(10);
            }
            return Sync(handle_samples);
        }

    private:
        CADSComm& ads_;
        Handle& handle_;
        double* pos_;
        double* act_pos_;
        double* init_pos_;
        double* rightlimit_;
        double& handle_ref_;
        double& handle_rot_ref_;
        double& axis1_base_rel_;
        double& axis2_base_rel_;
        double& axis3_base_rel_;
        double& axis5_base_rel_;
        double& axis6_base_rel_;
        bool& axis1_pause_mirror_;
        Axis1Phase& axis1_phase_;
        DWORD& axis1_phase_t0_;
        double& axis1_target_abs_;
        double axis1_start_offset_mm_;
        double axis1_end_offset_mm_;
        int& axis1_expected_dir_;
    };

    ControlSync sync(
        a,
        handle,
        pos,
        plc_act_pos,
        plc_init_pos,
        plc_rightlimit,
        handle_ref,
        handle_rot_ref,
        axis1_base_rel,
        axis2_base_rel,
        axis3_base_rel,
        axis5_base_rel,
        axis6_base_rel,
        axis1_pause_mirror,
        axis1_phase,
        axis1_phase_t0,
        axis1_target_abs,
        axis1_start_offset_mm, 
        axis1_end_offset_mm,
        axis1_expected_dir);

    if (!has_self_check_flag || self_check_done) {
        if (sync.RequestPlcReinitAndSync(30, 1500)) {
            control_active = true;
        }
    }

    while (1)
    {
        handle.showinfo();
        ++loop_count;
        axis1_fast_return = false;

        const bool b0_pressed = (handle.buttons2 & 0x01) != 0;
        const bool b6_pause_pressed = (handle.buttons2 & 0x40) != 0;
        const bool pause_pressed = b0_pressed || b6_pause_pressed;

        if (pause_pressed && !pause_pressed_prev) {
            freeze_active = true;
            control_active = false;
        } else if (!pause_pressed && pause_pressed_prev) {
            freeze_active = false;
            if (!estop_hold_active) {
                if (sync.RequestPlcReinitAndSync(20, 1500)) {
                    control_active = true;
                }
            }
        }
        pause_pressed_prev = pause_pressed;

        if ((loop_count % 10) == 0) {
            if (a.ADSRead((char*)"G.estop_hold_req", sizeof(estop_hold_req), &estop_hold_req)) {
                if (estop_hold_req) {
                    if (!estop_hold_active) {
                        std::cout << "PLC hold: ON" << std::endl;
                        handle.setforce_axis(0.0, axial_force_axis, 0.0);
                    }
                    estop_hold_active = true;
                    control_active = false;
                } else {
                    if (estop_hold_active) {
                        std::cout << "PLC hold: OFF" << std::endl;
                    }
                    estop_hold_active = false;
                }
            }
        }

        if (freeze_active) {
            fn_bias_inited = false;
            fn_force_f = 0.0;
            ft_force_f = 0.0;
            handle.setforce_axis(0.0, axial_force_axis, 0.0);
        }

        if (_kbhit()) {
            const int ch = _getch();
            if (ch == '\r') {
                force_feedback_enabled = !force_feedback_enabled;
                fn_bias_inited = false;
                fn_force_f = 0.0;
                ft_force_f = 0.0;
                fn_invalid_streak = 0;
                ft_invalid_streak = 0;
                std::cout << "Force feedback: " << (force_feedback_enabled ? "ON" : "OFF") << std::endl;
                if (!force_feedback_enabled) {
                    handle.setforce_axis(0.0, axial_force_axis, 0.0);
                }
            } else if (ch == 0 || ch == 224) {
                _getch();
            }
        }

        if (has_self_check_flag && (loop_count % 50) == 0) {
            if (a.ADSRead((char*)"G.self_check_done", sizeof(self_check_done), &self_check_done)) {
                if (!last_self_check_done && self_check_done) {
                    if (!freeze_active && !estop_hold_active && sync.RequestPlcReinitAndSync(30, 1500)) {
                        control_active = true;
                    }
                }
                last_self_check_done = self_check_done;
            }

            if (a.ADSRead((char*)"G.handle_reinit_req", sizeof(handle_reinit_req), &handle_reinit_req)) {
                if (handle_reinit_req) {
                    if (!freeze_active && !estop_hold_active) {
                        sync.Sync(30);
                    }
                    const bool clear_val = false;
                    a.ADSWrite((char*)"G.handle_reinit_req", sizeof(clear_val), (void*)&clear_val);
                }
            }
        }

        {
            
            // 0. Handle State Transition (Re-indexing)
            if (!control_active && !estop_hold_active) {
                if (!freeze_active && sync.Sync(20)) {
                    std::cout << "Re-synced" << std::endl;
                    control_active = true;
                }
            }

            // 1. Cylinder Actions
            cylinder1_cmd = cyl1_open;
            cylinder2_cmd = cyl2_clamp;
            
            // 2. Normal haptic force: from PLC sensors (fn/ft)
            if (!estop_hold_active &&
                force_feedback_enabled &&
                a.ADSRead((char*)"G.fn_value", sizeof(fn_raw), &fn_raw) &&
                a.ADSRead((char*)"G.ft_value", sizeof(ft_raw), &ft_raw))
            {
                const bool fn_valid = (std::abs((int)fn_raw) <= 8000);
                const bool ft_valid = (std::abs((int)ft_raw) <= 8000);

                if (fn_valid) {
                    fn_last_valid = fn_raw;
                    fn_has_valid = true;
                    fn_invalid_streak = 0;
                } else {
                    fn_invalid_streak++;
                }

                if (ft_valid) {
                    ft_last_valid = ft_raw;
                    ft_has_valid = true;
                    ft_invalid_streak = 0;
                } else {
                    ft_invalid_streak++;
                }

                if (fn_has_valid) {
                    fn_raw = fn_last_valid;
                }
                if (ft_has_valid) {
                    ft_raw = ft_last_valid;
                }

                if (!fn_bias_inited) {
                    fn_bias = (double)fn_raw;
                    fn_bias_inited = true;
                } else {
                    const double fn_err = ((double)fn_raw) - fn_bias;
                    const bool fn_near_zero = (std::abs(fn_err) <= 120.0);
                    if (fn_near_zero) {
                        fn_bias = fn_bias * 0.995 + ((double)fn_raw) * 0.005;
                    }
                }

                double fn_zeroed = (double)fn_raw - fn_bias;
                if (std::abs(fn_zeroed) < 20.0) {
                    fn_zeroed = 0.0;
                }

                const double axial_gain = 1.0 / 1000.0;
                const double axial_limit = 6.0;
                const double axial_force = clamp_double(fn_zeroed * axial_gain, -axial_limit, axial_limit);

                double torque_force = 0.0;
                if (ft_raw >= -870 && ft_raw <= -700) {
                    torque_force = 0.0;
                } else if (ft_raw > -700) {
                    torque_force = ((double)ft_raw + 700.0) * (-1.0 / 600.0);
                } else {
                    torque_force = ((double)ft_raw + 870.0) / (-530.0);
                }
                torque_force = clamp_double(torque_force, -1.0, 1.0);

                if (control_active) {
                    fn_force_f = fn_force_f * 0.7 + (axial_force * 0.3);
                    ft_force_f = ft_force_f * 0.7 + (torque_force * 0.3);
                    handle.setforce_axis(fn_force_f * axial_force_sign, axial_force_axis, ft_force_f);
                } else {
                    apply_cmd_force(0.0);
                }

                if ((loop_count % 100) == 0 || fn_raw != last_fn_raw || ft_raw != last_ft_raw) {
                    last_fn_raw = fn_raw;
                    last_ft_raw = ft_raw;
                    std::cout
                        << "fn_raw=" << fn_raw << " fn_bias=" << fn_bias
                        << " fn_cmd=" << fn_force_f
                        << " | ft_raw=" << ft_raw << " ft_cmd=" << ft_force_f
                        << " | axis=" << axial_force_axis << " sign=" << axial_force_sign
                        << " | fn_inv=" << fn_invalid_streak << " ft_inv=" << ft_invalid_streak
                        << std::endl;
                }
            }
            else
            {
                fn_force_f = 0.0;
                ft_force_f = 0.0;
                if (control_active && !estop_hold_active) {
                    handle.setforce_axis(0.0, axial_force_axis, 0.0);
                } else {
                    apply_cmd_force(0.0);
                }
            }
        }

        // Debug Buttons
        if (handle.buttons2 != last_btn) {
             std::cout << "Btns: 0x" << std::hex << (int)handle.buttons2 << std::dec << std::endl;
             last_btn = handle.buttons2;
        }

        // Motion Control
        if (freeze_active)
        {
            control_active = false;
        }
        else if (control_active)
        {
            const DWORD now_ms = GetTickCount();

            a.ADSRead((char*)"G.Act_pos", sizeof(plc_act_pos), &plc_act_pos);

            const double axis1_abs = plc_act_pos[0] + plc_init_pos[0];
            const double axis1_start_abs = plc_rightlimit[0] - axis1_start_offset_mm;
            const double axis1_end_abs = plc_rightlimit[0] - axis1_end_offset_mm;
            const double axis1_min_abs = (axis1_start_abs < axis1_end_abs) ? axis1_start_abs : axis1_end_abs;
            const double axis1_max_abs = (axis1_start_abs > axis1_end_abs) ? axis1_start_abs : axis1_end_abs;

            if (axis1_phase == Axis1Phase::Follow)
            {
                axis1_pause_mirror = false;

                const double hand_delta_mm = (handle.fJoints2[0] - handle_ref) * k_handle_to_mm * axis1_push_sign;
                int hand_dir = 0;
                if (hand_delta_mm > axis1_trigger_deadband_mm) hand_dir = 1;
                else if (hand_delta_mm < -axis1_trigger_deadband_mm) hand_dir = -1;

                const double axis1_base_abs = axis1_base_rel + plc_init_pos[0];
                double cmd_abs = axis1_base_abs + hand_delta_mm;
                if (cmd_abs < axis1_min_abs) cmd_abs = axis1_min_abs;
                if (cmd_abs > axis1_max_abs) cmd_abs = axis1_max_abs;
                pos[0] = cmd_abs - plc_init_pos[0];

                const double axis2_delta_deg = (handle.fJoints2[1] - handle_rot_ref) * Rad;
                pos[1] = axis2_base_rel + axis2_delta_deg;

                if (!axis1_pause_mirror) {
                    pos[2] = axis3_base_rel + (pos[0] - axis1_base_rel);
                    pos[4] = axis5_base_rel + (pos[2] - axis3_base_rel);
                    pos[5] = axis6_base_rel + (pos[2] - axis3_base_rel);
                }

                if ((hand_delta_mm > axis1_trigger_deadband_mm) &&
                    (std::abs(axis1_abs - axis1_end_abs) <= axis1_tol_mm)) {
                    axis1_target_abs = axis1_start_abs;
                    axis1_phase = Axis1Phase::Open2Wait;
                    axis1_phase_t0 = now_ms;
                    axis1_pause_mirror = true;
                } else if ((hand_delta_mm < -axis1_trigger_deadband_mm) &&
                    (std::abs(axis1_abs - axis1_start_abs) <= axis1_tol_mm)) {
                    axis1_target_abs = axis1_end_abs;
                    axis1_phase = Axis1Phase::Open2Wait;
                    axis1_phase_t0 = now_ms;
                    axis1_pause_mirror = true;
                }
            }

            if (axis1_phase == Axis1Phase::Open2Wait) {
                cylinder1_cmd = cyl1_clamp;
                cylinder2_cmd = cyl2_open;
                axis1_pause_mirror = true;
                if ((now_ms - axis1_phase_t0) >= axis1_open_delay_ms) {
                    axis1_phase = Axis1Phase::FastMove;
                    axis1_phase_t0 = now_ms;
                }
            } else if (axis1_phase == Axis1Phase::FastMove) {
                cylinder1_cmd = cyl1_clamp;
                cylinder2_cmd = cyl2_open;
                axis1_pause_mirror = true;
                axis1_fast_return = true;

                pos[0] = axis1_target_abs - plc_init_pos[0];

                a.ADSRead((char*)"G.Act_pos", sizeof(plc_act_pos), &plc_act_pos);
                const double axis1_abs2 = plc_act_pos[0] + plc_init_pos[0];
                if (std::abs(axis1_abs2 - axis1_target_abs) < axis1_tol_mm) {
                    axis1_phase = Axis1Phase::Clamp2Wait;
                    axis1_phase_t0 = now_ms;
                }
            } else if (axis1_phase == Axis1Phase::Clamp2Wait) {
                cylinder1_cmd = cyl1_clamp;
                cylinder2_cmd = cyl2_clamp;
                axis1_pause_mirror = true;
                if ((now_ms - axis1_phase_t0) >= axis1_clamp_delay_ms) {
                    axis1_phase = Axis1Phase::Open1Wait;
                    axis1_phase_t0 = now_ms;
                }
            } else if (axis1_phase == Axis1Phase::Open1Wait) {
                cylinder1_cmd = cyl1_open;
                cylinder2_cmd = cyl2_clamp;
                axis1_pause_mirror = true;
                if ((now_ms - axis1_phase_t0) >= axis1_open_delay_ms) {
                    sync.Sync(20);
                }
            }

            a.ADSWrite((char*)"G.refer", sizeof(pos), &pos);
        }

        // Write IO (after state machine updates)
        if (!freeze_active) {
            cylinder3_cmd = (cylinder1_cmd == cyl1_open) ? cyl3_clamp : cyl3_open;
            cylinder4_cmd = (cylinder2_cmd == cyl2_clamp) ? cyl4_open : cyl4_clamp;
            a.ADSWrite((char*)"G.cylinder1_value", sizeof(cylinder1_cmd), &cylinder1_cmd);
            a.ADSWrite((char*)"G.cylinder2_value", sizeof(cylinder2_cmd), &cylinder2_cmd);
            a.ADSWrite((char*)"G.cylinder3_value", sizeof(cylinder3_cmd), &cylinder3_cmd);
            a.ADSWrite((char*)"G.cylinder4_value", sizeof(cylinder4_cmd), &cylinder4_cmd);
        }
        a.ADSWrite((char*)"G.axis1_fast_return", sizeof(axis1_fast_return), &axis1_fast_return);
    }

    handle.close();
    return 0;
}
