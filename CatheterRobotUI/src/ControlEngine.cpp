#include "ControlEngine.h"

#include <cmath>

namespace
{
double clampDouble(double value, double low, double high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

bool isWithinRange(double value, double low, double high, double tol = 0.0)
{
    return (value >= (low - tol)) && (value <= (high + tol));
}

double getAveragePos(Handle& handle, int axis, int samples)
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

void getAverageDualPos(
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

int handDirFromDelta(double delta_mm, double deadband_mm)
{
    if (delta_mm > deadband_mm)
    {
        return 1;
    }
    if (delta_mm < -deadband_mm)
    {
        return -1;
    }
    return 0;
}
}

ControlEngine::ControlEngine(SharedState& shared)
    : shared_(shared)
{
    axis1_crawl_.enabled = true;
}

ControlEngine::~ControlEngine()
{
    shutdown();
}

bool ControlEngine::init()
{
    handle_axis1_ = new Handle(kSerialAxis1Handle);
    handle_axis6_ = new Handle(kSerialAxis6Handle);

    if (!handle_axis1_->init())
    {
        publishState();
        shutdown();
        return false;
    }
    if (!handle_axis6_->init())
    {
        publishState();
        shutdown();
        return false;
    }

    Sleep(1000);
    handle_axis1_->poll();
    handle_axis6_->poll();

    ads_ = new CADSComm();
    if (!ads_->OpenComm_inside() && !ads_->OpenComm())
    {
        publishState();
        shutdown();
        return false;
    }

    if (!readPlcState())
    {
        publishState();
        shutdown();
        return false;
    }

    loadPosFromActual();
    writeRefer();

    has_self_check_flag_ = ads_->ADSRead("G.self_check_done", sizeof(self_check_done_), &self_check_done_);
    if (has_self_check_flag_)
    {
        ads_->ADSRead("G.self_check_done", sizeof(self_check_done_), &self_check_done_);
    }

    control_active_ = !has_self_check_flag_ || self_check_done_;
    last_self_check_done_ = self_check_done_;
    estop_hold_req_ = false;
    handle_reinit_req_ = false;
    applyCmdForce(0.0);

    if (!has_self_check_flag_ || self_check_done_)
    {
        if (syncAll(30))
        {
            control_active_ = true;
        }
    }

    last_rate_tick_ = GetTickCount();
    rate_counter_ = 0;
    publishState();
    return true;
}

bool ControlEngine::tick()
{
    const UIToControl cmd = shared_.readUICommands();
    applyUICommands(cmd);

    if (cmd.cmd_quit)
    {
        return false;
    }

    if (handle_axis1_ != nullptr)
    {
        handle_axis1_->poll();
    }
    if (handle_axis6_ != nullptr)
    {
        handle_axis6_->poll();
    }

    ++loop_count_;
    axis1_fast_return_ = false;
    axis6_fast_retract_ = false;

    updateControlMode();

    const bool pause_pressed = (handle_axis1_ != nullptr) &&
        ((handle_axis1_->buttons2 & kAxis1PauseButtonMask) != 0);
    const bool axis6_arm_pressed = (handle_axis6_ != nullptr) &&
        ((handle_axis6_->buttons2 & kAxis6ArmButtonMask) != 0);
    const bool freeze_requested = cmd.cmd_pause || pause_pressed;

    if (axis6_arm_pressed && !axis6_arm_pressed_prev_)
    {
        if (!freeze_active_ && !estop_hold_active_ && control_active_)
        {
            syncAxis6(20, true, false, 0);
        }
    }
    else if (!axis6_arm_pressed && axis6_arm_pressed_prev_ && axis6_crawl_.enabled)
    {
        releaseAxis6ToFollow();
    }
    axis6_arm_pressed_prev_ = axis6_arm_pressed;

    if (freeze_requested && !freeze_active_)
    {
        freeze_active_ = true;
        control_active_ = false;
        applyCmdForce(0.0);
    }
    else if (!freeze_requested && freeze_active_)
    {
        freeze_active_ = false;
        if (!estop_hold_active_ && syncAll(20))
        {
            control_active_ = true;
        }
    }

    if ((loop_count_ % 10) == 0 && ads_ != nullptr)
    {
        bool estop_hold_req = estop_hold_req_;
        if (ads_->ADSRead("G.estop_hold_req", sizeof(estop_hold_req), &estop_hold_req))
        {
            estop_hold_req_ = estop_hold_req;
            if (estop_hold_req_)
            {
                estop_hold_active_ = true;
                control_active_ = false;
                applyCmdForce(0.0);
            }
            else
            {
                estop_hold_active_ = false;
            }
        }
    }

    if (freeze_active_)
    {
        fn_bias_inited_ = false;
        fn_force_f_ = 0.0;
        ft_force_f_ = 0.0;
        applyCmdForce(0.0);
    }

    if (has_self_check_flag_ && (loop_count_ % 50) == 0 && ads_ != nullptr)
    {
        bool self_check_done = self_check_done_;
        if (ads_->ADSRead("G.self_check_done", sizeof(self_check_done), &self_check_done))
        {
            self_check_done_ = self_check_done;
            if (!last_self_check_done_ && self_check_done_)
            {
                if (!freeze_active_ && !estop_hold_active_ && syncAll(30))
                {
                    control_active_ = true;
                }
            }
            last_self_check_done_ = self_check_done_;
        }

        bool handle_reinit_req = handle_reinit_req_;
        if (ads_->ADSRead("G.handle_reinit_req", sizeof(handle_reinit_req), &handle_reinit_req))
        {
            handle_reinit_req_ = handle_reinit_req;
            if (handle_reinit_req_)
            {
                if (!freeze_active_ && !estop_hold_active_)
                {
                    syncAll(30);
                }
                clearPlcReinitReq();
            }
        }
    }

    if (!estop_hold_active_ &&
        force_feedback_enabled_ &&
        ads_ != nullptr &&
        ads_->ADSRead("G.fn_value", sizeof(fn_raw_), &fn_raw_) &&
        ads_->ADSRead("G.ft_value", sizeof(ft_raw_), &ft_raw_))
    {
        const bool fn_valid = (std::abs(static_cast<int>(fn_raw_)) <= 8000);
        const bool ft_valid = (std::abs(static_cast<int>(ft_raw_)) <= 8000);

        if (fn_valid)
        {
            fn_last_valid_ = fn_raw_;
            fn_has_valid_ = true;
            fn_invalid_streak_ = 0;
        }
        else
        {
            ++fn_invalid_streak_;
        }

        if (ft_valid)
        {
            ft_last_valid_ = ft_raw_;
            ft_has_valid_ = true;
            ft_invalid_streak_ = 0;
        }
        else
        {
            ++ft_invalid_streak_;
        }

        if (fn_has_valid_)
        {
            fn_raw_ = fn_last_valid_;
        }
        if (ft_has_valid_)
        {
            ft_raw_ = ft_last_valid_;
        }

        if (!fn_bias_inited_)
        {
            fn_bias_ = static_cast<double>(fn_raw_);
            fn_bias_inited_ = true;
        }
        else
        {
            const double fn_err = static_cast<double>(fn_raw_) - fn_bias_;
            if (std::abs(fn_err) <= 120.0)
            {
                fn_bias_ = fn_bias_ * 0.995 + static_cast<double>(fn_raw_) * 0.005;
            }
        }

        double fn_zeroed = static_cast<double>(fn_raw_) - fn_bias_;
        if (std::abs(fn_zeroed) < 20.0)
        {
            fn_zeroed = 0.0;
        }

        const double axial_gain = (1.0 / 1000.0) * force_gain_axial_;
        const double axial_limit = 6.0;
        const double axial_force = clampDouble(fn_zeroed * axial_gain, -axial_limit, axial_limit);

        double torque_force = 0.0;
        if (ft_raw_ >= -870 && ft_raw_ <= -700)
        {
            torque_force = 0.0;
        }
        else if (ft_raw_ > -700)
        {
            torque_force = (static_cast<double>(ft_raw_) + 700.0) * (-1.0 / 600.0);
        }
        else
        {
            torque_force = (static_cast<double>(ft_raw_) + 870.0) / (-530.0);
        }
        torque_force = clampDouble(torque_force * force_gain_torque_, -1.0, 1.0);

        if (control_active_)
        {
            fn_force_f_ = fn_force_f_ * 0.7 + (axial_force * 0.3);
            ft_force_f_ = ft_force_f_ * 0.7 + (torque_force * 0.3);
            handle_axis1_->setforce_axis(fn_force_f_ * kAxialForceSign, kAxialForceAxis, ft_force_f_);
        }
        else
        {
            applyCmdForce(0.0);
        }

        last_fn_raw_ = fn_raw_;
        last_ft_raw_ = ft_raw_;
    }
    else
    {
        fn_force_f_ = 0.0;
        ft_force_f_ = 0.0;
        if (!freeze_active_)
        {
            applyCmdForce(0.0);
        }
    }

    if (recording_)
    {
        ForceSample sample;
        sample.timestamp_sec = (GetTickCount() - record_start_tick_) / 1000.0;
        sample.fn_raw = fn_raw_;
        sample.ft_raw = ft_raw_;
        sample.fn_filtered = fn_force_f_;
        sample.ft_filtered = ft_force_f_;
        shared_.pushForceSample(sample);
    }

    if (!control_active_ && !freeze_active_ && !estop_hold_active_)
    {
        if (syncAll(20))
        {
            control_active_ = true;
        }
    }

    cylinder_cmds_[0] = kCyl1Open;
    cylinder_cmds_[1] = kCyl2Clamp;
    cylinder_cmds_[2] = kCyl3FollowRelease;
    cylinder_cmds_[3] = kCyl4FollowRelease;

    if (control_active_ && readPlcState())
    {
        loadPosFromActual();

        const DWORD now_ms = GetTickCount();
        const double effective_k = kHandleToMmBase * handle_sensitivity_;

        const double axis1_abs = plc_act_pos_[0] + plc_init_pos_[0];
        const double axis1_min_abs = (axis1_crawl_.start_abs < axis1_crawl_.end_abs) ? axis1_crawl_.start_abs : axis1_crawl_.end_abs;
        const double axis1_max_abs = (axis1_crawl_.start_abs > axis1_crawl_.end_abs) ? axis1_crawl_.start_abs : axis1_crawl_.end_abs;
        const bool axis1_now_in_window = isWithinRange(axis1_abs, axis1_min_abs, axis1_max_abs, kCrawlArriveTolMm);
        if (!axis1_crawl_.window_active && axis1_now_in_window)
        {
            captureAxis1FollowBaseline();
            axis1_crawl_.window_active = true;
        }
        const double axis1_base_abs = axis1_crawl_.base_rel + plc_init_pos_[0];
        const double axis1_hand_delta_mm =
            (handle_axis1_->fJoints2[0] - axis1_crawl_.handle_ref) * effective_k * kAxisPushSign;
        const int axis1_hand_dir = handDirFromDelta(axis1_hand_delta_mm, kCrawlTriggerDeadbandMm);
        const bool axis6_independent = axis6_crawl_.enabled;

        if (axis1_crawl_.wait_rearm)
        {
            const bool same_dir_push = (axis1_crawl_.rearm_dir > 0) && (axis1_hand_delta_mm > kCrawlRearmThresholdMm);
            const bool same_dir_pull = (axis1_crawl_.rearm_dir < 0) && (axis1_hand_delta_mm < -kCrawlRearmThresholdMm);
            const bool reverse_dir_push = (axis1_crawl_.rearm_dir < 0) && (axis1_hand_delta_mm > kCrawlRearmThresholdMm);
            const bool reverse_dir_pull = (axis1_crawl_.rearm_dir > 0) && (axis1_hand_delta_mm < -kCrawlRearmThresholdMm);
            if (same_dir_push || same_dir_pull || reverse_dir_push || reverse_dir_pull)
            {
                axis1_crawl_.wait_rearm = false;
            }
        }

        if (axis1_crawl_.phase == CrawlPhase::Follow)
        {
            const double axis1_raw_cmd_abs = axis1_base_abs + axis1_hand_delta_mm;
            const double axis1_cmd_abs = axis1_crawl_.window_active
                ? clampDouble(axis1_raw_cmd_abs, axis1_min_abs, axis1_max_abs)
                : axis1_raw_cmd_abs;
            const double axis1_delta_rel = axis1_cmd_abs - plc_init_pos_[0] - axis1_crawl_.base_rel;
            pos_[0] = axis1_cmd_abs - plc_init_pos_[0];
            pos_[1] = axis1_crawl_.rot_base_rel + (handle_axis1_->fJoints2[1] - axis1_crawl_.rot_ref) * kAxisRotScaleDeg;
            pos_[2] = axis3_base_rel_ + axis1_delta_rel;
            pos_[4] = axis5_base_rel_ + axis1_delta_rel;

            if (!axis6_independent)
            {
                pos_[5] = axis6_mirror_base_rel_ + axis1_delta_rel;
                cylinder_cmds_[2] = kCyl3FollowRelease;
                cylinder_cmds_[3] = kCyl4FollowRelease;
            }

            if (axis1_crawl_.window_active && !axis1_crawl_.wait_rearm)
            {
                if ((axis1_hand_dir > 0) && (std::abs(axis1_abs - axis1_crawl_.end_abs) <= kCrawlArriveTolMm))
                {
                    axis1_crawl_.target_abs = axis1_crawl_.start_abs;
                    axis1_crawl_.phase = CrawlPhase::SwitchWait;
                    axis1_crawl_.phase_t0 = now_ms;
                    axis1_crawl_.rearm_dir = 1;
                }
                else if ((axis1_hand_dir < 0) && (std::abs(axis1_abs - axis1_crawl_.start_abs) <= kCrawlArriveTolMm))
                {
                    axis1_crawl_.target_abs = axis1_crawl_.end_abs;
                    axis1_crawl_.phase = CrawlPhase::SwitchWait;
                    axis1_crawl_.phase_t0 = now_ms;
                    axis1_crawl_.rearm_dir = -1;
                }
            }
        }
        else if (axis1_crawl_.phase == CrawlPhase::SwitchWait)
        {
            cylinder_cmds_[0] = kCyl1Clamp;
            cylinder_cmds_[1] = kCyl2Open;
            if ((now_ms - axis1_crawl_.phase_t0) >= kCrawlSwitchDelayMs)
            {
                axis1_crawl_.phase = CrawlPhase::FastMove;
                axis1_crawl_.phase_t0 = now_ms;
            }
        }
        else if (axis1_crawl_.phase == CrawlPhase::FastMove)
        {
            cylinder_cmds_[0] = kCyl1Clamp;
            cylinder_cmds_[1] = kCyl2Open;
            axis1_fast_return_ = true;
            pos_[0] = axis1_crawl_.target_abs - plc_init_pos_[0];
            if (std::abs(axis1_abs - axis1_crawl_.target_abs) <= kCrawlArriveTolMm)
            {
                axis1_crawl_.phase = CrawlPhase::ClampWait;
                axis1_crawl_.phase_t0 = now_ms;
            }
        }
        else if (axis1_crawl_.phase == CrawlPhase::ClampWait)
        {
            cylinder_cmds_[0] = kCyl1Clamp;
            cylinder_cmds_[1] = kCyl2Clamp;
            if ((now_ms - axis1_crawl_.phase_t0) >= kCrawlClampDelayMs)
            {
                axis1_crawl_.phase = CrawlPhase::RestoreWait;
                axis1_crawl_.phase_t0 = now_ms;
            }
        }
        else if (axis1_crawl_.phase == CrawlPhase::RestoreWait)
        {
            cylinder_cmds_[0] = kCyl1Open;
            cylinder_cmds_[1] = kCyl2Clamp;
            if ((now_ms - axis1_crawl_.phase_t0) >= kCrawlSwitchDelayMs)
            {
                syncAxis1(20, true, axis1_crawl_.rearm_dir);
            }
        }

        const double axis6_abs = plc_act_pos_[5] + plc_init_pos_[5];
        const double axis6_min_abs = (axis6_crawl_.start_abs < axis6_crawl_.end_abs) ? axis6_crawl_.start_abs : axis6_crawl_.end_abs;
        const double axis6_max_abs = (axis6_crawl_.start_abs > axis6_crawl_.end_abs) ? axis6_crawl_.start_abs : axis6_crawl_.end_abs;
        const double axis6_base_abs = axis6_crawl_.base_rel + plc_init_pos_[5];
        const double axis6_hand_delta_mm =
            (handle_axis6_->fJoints2[0] - axis6_crawl_.handle_ref) * effective_k * kAxisPushSign;
        const int axis6_hand_dir = handDirFromDelta(axis6_hand_delta_mm, kCrawlTriggerDeadbandMm);

        if (axis6_crawl_.enabled)
        {
            if (axis6_crawl_.wait_rearm)
            {
                const bool same_dir_push = (axis6_crawl_.rearm_dir > 0) && (axis6_hand_delta_mm > kCrawlRearmThresholdMm);
                const bool same_dir_pull = (axis6_crawl_.rearm_dir < 0) && (axis6_hand_delta_mm < -kCrawlRearmThresholdMm);
                const bool reverse_dir_push = (axis6_crawl_.rearm_dir < 0) && (axis6_hand_delta_mm > kCrawlRearmThresholdMm);
                const bool reverse_dir_pull = (axis6_crawl_.rearm_dir > 0) && (axis6_hand_delta_mm < -kCrawlRearmThresholdMm);
                if (same_dir_push || same_dir_pull || reverse_dir_push || reverse_dir_pull)
                {
                    axis6_crawl_.wait_rearm = false;
                }
            }

            if (axis6_crawl_.phase == CrawlPhase::Follow)
            {
                const double axis6_cmd_abs = clampDouble(axis6_base_abs + axis6_hand_delta_mm, axis6_min_abs, axis6_max_abs);
                pos_[5] = axis6_cmd_abs - plc_init_pos_[5];
                pos_[6] = axis6_crawl_.rot_base_rel + (handle_axis6_->fJoints2[1] - axis6_crawl_.rot_ref) * kAxisRotScaleDeg;
                cylinder_cmds_[2] = kCyl3Open;
                cylinder_cmds_[3] = kCyl4Clamp;

                if (!axis6_crawl_.wait_rearm)
                {
                    if ((axis6_hand_dir > 0) && (std::abs(axis6_abs - axis6_crawl_.end_abs) <= kCrawlArriveTolMm))
                    {
                        axis6_crawl_.target_abs = axis6_crawl_.start_abs;
                        axis6_crawl_.phase = CrawlPhase::SwitchWait;
                        axis6_crawl_.phase_t0 = now_ms;
                        axis6_crawl_.rearm_dir = 1;
                    }
                    else if ((axis6_hand_dir < 0) && (std::abs(axis6_abs - axis6_crawl_.start_abs) <= kCrawlArriveTolMm))
                    {
                        axis6_crawl_.target_abs = axis6_crawl_.end_abs;
                        axis6_crawl_.phase = CrawlPhase::SwitchWait;
                        axis6_crawl_.phase_t0 = now_ms;
                        axis6_crawl_.rearm_dir = -1;
                    }
                }
            }
            else if (axis6_crawl_.phase == CrawlPhase::SwitchWait)
            {
                cylinder_cmds_[2] = kCyl3Clamp;
                cylinder_cmds_[3] = kCyl4Open;
                if ((now_ms - axis6_crawl_.phase_t0) >= kCrawlSwitchDelayMs)
                {
                    axis6_crawl_.phase = CrawlPhase::FastMove;
                    axis6_crawl_.phase_t0 = now_ms;
                }
            }
            else if (axis6_crawl_.phase == CrawlPhase::FastMove)
            {
                cylinder_cmds_[2] = kCyl3Clamp;
                cylinder_cmds_[3] = kCyl4Open;
                axis6_fast_retract_ = true;
                pos_[5] = axis6_crawl_.target_abs - plc_init_pos_[5];
                if (std::abs(axis6_abs - axis6_crawl_.target_abs) <= kCrawlArriveTolMm)
                {
                    axis6_crawl_.phase = CrawlPhase::ClampWait;
                    axis6_crawl_.phase_t0 = now_ms;
                }
            }
            else if (axis6_crawl_.phase == CrawlPhase::ClampWait)
            {
                cylinder_cmds_[2] = kCyl3Clamp;
                cylinder_cmds_[3] = kCyl4Clamp;
                if ((now_ms - axis6_crawl_.phase_t0) >= kCrawlClampDelayMs)
                {
                    axis6_crawl_.phase = CrawlPhase::RestoreWait;
                    axis6_crawl_.phase_t0 = now_ms;
                }
            }
            else if (axis6_crawl_.phase == CrawlPhase::RestoreWait)
            {
                cylinder_cmds_[2] = kCyl3Open;
                cylinder_cmds_[3] = kCyl4Clamp;
                if ((now_ms - axis6_crawl_.phase_t0) >= kCrawlSwitchDelayMs)
                {
                    syncAxis6(20, false, true, axis6_crawl_.rearm_dir);
                }
            }
        }

        writeRefer();
    }

    if (!freeze_active_ && ads_ != nullptr)
    {
        unsigned short cylinder1_cmd = cylinder_cmds_[0];
        unsigned short cylinder2_cmd = cylinder_cmds_[1];
        unsigned short cylinder3_cmd = cylinder_cmds_[2];
        unsigned short cylinder4_cmd = cylinder_cmds_[3];
        ads_->ADSWrite("G.cylinder1_value", sizeof(cylinder1_cmd), &cylinder1_cmd);
        ads_->ADSWrite("G.cylinder2_value", sizeof(cylinder2_cmd), &cylinder2_cmd);
        ads_->ADSWrite("G.cylinder3_value", sizeof(cylinder3_cmd), &cylinder3_cmd);
        ads_->ADSWrite("G.cylinder4_value", sizeof(cylinder4_cmd), &cylinder4_cmd);
    }

    if (ads_ != nullptr)
    {
        ads_->ADSWrite("G.axis1_fast_return", sizeof(axis1_fast_return_), &axis1_fast_return_);
        ads_->ADSWrite("G.axis6_fast_retract", sizeof(axis6_fast_retract_), &axis6_fast_retract_);
    }

    computeLoopRate();
    publishState();
    return true;
}

void ControlEngine::shutdown()
{
    if (handle_axis1_ != nullptr)
    {
        handle_axis1_->close();
        delete handle_axis1_;
        handle_axis1_ = nullptr;
    }
    if (handle_axis6_ != nullptr)
    {
        handle_axis6_->close();
        delete handle_axis6_;
        handle_axis6_ = nullptr;
    }
    if (ads_ != nullptr)
    {
        ads_->CloseComm();
        delete ads_;
        ads_ = nullptr;
    }
}

bool ControlEngine::readPlcState()
{
    if (ads_ == nullptr || !ads_->IsCommOpen())
    {
        return false;
    }

    bool ok = true;
    ok = ads_->ADSRead("G.Act_pos", sizeof(plc_act_pos_), plc_act_pos_) && ok;
    ok = ads_->ADSRead("G.init_pos", sizeof(plc_init_pos_), plc_init_pos_) && ok;
    ok = ads_->ADSRead("G.rightlimit", sizeof(plc_rightlimit_), plc_rightlimit_) && ok;
    ok = ads_->ADSRead("G.gen_state", sizeof(plc_state_raw_), &plc_state_raw_) && ok;
    return ok;
}

bool ControlEngine::writeRefer()
{
    if (ads_ == nullptr || !ads_->IsCommOpen())
    {
        return false;
    }
    return ads_->ADSWrite("G.refer", sizeof(pos_), pos_);
}

void ControlEngine::loadPosFromActual()
{
    for (int i = 0; i < 7; ++i)
    {
        pos_[i] = plc_act_pos_[i];
    }
}

void ControlEngine::applyUICommands(const UIToControl& cmd)
{
    const bool force_feedback_changed = (force_feedback_enabled_ != cmd.force_feedback_on);
    force_feedback_enabled_ = cmd.force_feedback_on;
    force_gain_axial_ = cmd.force_gain_axial;
    force_gain_torque_ = cmd.force_gain_torque;
    axis4_velocity_ = cmd.axis4_velocity;
    double preset_scale = 1.0;
    switch (cmd.speed_preset)
    {
    case UIToControl::SpeedPreset::Fine:
        preset_scale = 0.3;
        break;
    case UIToControl::SpeedPreset::Normal:
        preset_scale = 1.0;
        break;
    case UIToControl::SpeedPreset::Fast:
        preset_scale = 2.0;
        break;
    }
    handle_sensitivity_ = cmd.handle_sensitivity * preset_scale;

    if (force_feedback_changed)
    {
        fn_bias_inited_ = false;
        fn_force_f_ = 0.0;
        ft_force_f_ = 0.0;
        fn_invalid_streak_ = 0;
        ft_invalid_streak_ = 0;
        if (!force_feedback_enabled_)
        {
            applyCmdForce(0.0);
        }
    }

    if (cmd.force_record_start && !recording_)
    {
        recording_ = true;
        record_start_tick_ = GetTickCount();
    }
    if (cmd.force_record_stop && recording_)
    {
        recording_ = false;
    }

    if (cmd.cmd_redo_selfcheck && ads_ != nullptr)
    {
        bool reset = true;
        ads_->ADSWrite("G.selfcheck_reset_req", sizeof(reset), &reset);
        control_active_ = false;
    }

    if (cmd.cmd_estop && ads_ != nullptr)
    {
        bool request = true;
        ads_->ADSWrite("G.estop_hold_req", sizeof(request), &request);
        estop_hold_active_ = true;
        control_active_ = false;
        applyCmdForce(0.0);
    }

    if (ads_ != nullptr && ads_->IsCommOpen())
    {
        double v4 = axis4_velocity_;
        ads_->ADSWrite("G.v_limit[4]", sizeof(v4), &v4);
    }
}

void ControlEngine::applyCmdForce(double cmd_force)
{
    if (handle_axis1_ != nullptr && handle_axis1_->is_open())
    {
        handle_axis1_->setforce_axis(cmd_force * kAxialForceSign, kAxialForceAxis, 0.0);
    }
}

void ControlEngine::updateControlMode()
{
    if (handle_axis6_ == nullptr)
    {
        current_mode_ = ControlMode::Catheter;
        return;
    }

    const unsigned char btn587 = handle_axis6_->buttons2;
    if ((btn587 & kButtonMaskB7) != 0)
    {
        current_mode_ = ControlMode::GuidewireCatheter;
    }
    else if ((btn587 & kButtonMaskB6) != 0)
    {
        current_mode_ = ControlMode::Guidewire;
    }
    else
    {
        current_mode_ = ControlMode::Catheter;
    }
}

void ControlEngine::publishState()
{
    ControlToUI state;

    state.handle582_connected = (handle_axis1_ != nullptr) && handle_axis1_->is_open();
    state.handle587_connected = (handle_axis6_ != nullptr) && handle_axis6_->is_open();
    state.ads_connected = (ads_ != nullptr) && ads_->IsCommOpen();

    state.plc_state = toPlcState(plc_state_raw_);
    state.self_check_done = self_check_done_;
    state.control_active = control_active_;
    state.estop_hold_active = estop_hold_active_;
    state.freeze_active = freeze_active_;

    for (int i = 0; i < 7; ++i)
    {
        state.act_pos[i] = plc_act_pos_[i];
        state.init_pos[i] = plc_init_pos_[i];
        state.rightlimit[i] = plc_rightlimit_[i];
        state.refer[i] = pos_[i];
    }

    state.axis1_crawl_phase = axis1_crawl_.phase;
    state.axis6_crawl_phase = axis6_crawl_.phase;
    state.axis1_crawl_enabled = axis1_crawl_.enabled;
    state.axis6_crawl_enabled = axis6_crawl_.enabled;

    state.current_mode = current_mode_;

    state.fn_raw = fn_raw_;
    state.ft_raw = ft_raw_;
    state.fn_filtered = fn_force_f_;
    state.ft_filtered = ft_force_f_;
    state.fn_bias = fn_bias_;

    if (handle_axis1_ != nullptr)
    {
        state.handle582_buttons = handle_axis1_->buttons2;
        state.handle582_joints = { handle_axis1_->fJoints2[0], handle_axis1_->fJoints2[1] };
    }
    if (handle_axis6_ != nullptr)
    {
        state.handle587_buttons = handle_axis6_->buttons2;
        state.handle587_joints = { handle_axis6_->fJoints2[0], handle_axis6_->fJoints2[1] };
    }

    state.cylinder_cmds = cylinder_cmds_;
    state.loop_rate_hz = current_hz_;

    shared_.writeControlState(state);
}

void ControlEngine::computeLoopRate()
{
    ++rate_counter_;
    const DWORD now = GetTickCount();
    const DWORD elapsed = now - last_rate_tick_;
    if (elapsed >= 1000)
    {
        current_hz_ = rate_counter_ * 1000.0 / elapsed;
        rate_counter_ = 0;
        last_rate_tick_ = now;
    }
}

bool ControlEngine::syncAll(int samples)
{
    if (!readPlcState())
    {
        return false;
    }

    loadPosFromActual();
    if (!writeRefer())
    {
        return false;
    }

    getAverageDualPos(
        *handle_axis1_,
        *handle_axis6_,
        samples,
        axis1_crawl_.handle_ref,
        axis1_crawl_.rot_ref,
        axis6_crawl_.handle_ref,
        axis6_crawl_.rot_ref);

    if (!readPlcState())
    {
        return false;
    }

    loadPosFromActual();
    if (!writeRefer())
    {
        return false;
    }

    axis1_crawl_.base_rel = plc_act_pos_[0];
    axis1_crawl_.rot_base_rel = plc_act_pos_[1];
    axis1_crawl_.start_abs = axis1StartAbs();
    axis1_crawl_.end_abs = axis1EndAbs();
    axis1_crawl_.window_active = isWithinRange(
        plc_act_pos_[0] + plc_init_pos_[0],
        (axis1_crawl_.start_abs < axis1_crawl_.end_abs) ? axis1_crawl_.start_abs : axis1_crawl_.end_abs,
        (axis1_crawl_.start_abs > axis1_crawl_.end_abs) ? axis1_crawl_.start_abs : axis1_crawl_.end_abs,
        kCrawlArriveTolMm);
    axis1_crawl_.phase = CrawlPhase::Follow;
    axis1_crawl_.phase_t0 = GetTickCount();
    axis1_crawl_.wait_rearm = false;
    axis1_crawl_.rearm_dir = 0;
    axis1_crawl_.enabled = true;

    axis3_base_rel_ = plc_act_pos_[2];
    axis5_base_rel_ = plc_act_pos_[4];
    axis6_mirror_base_rel_ = plc_act_pos_[5];

    axis6_crawl_.base_rel = plc_act_pos_[5];
    axis6_crawl_.rot_base_rel = plc_act_pos_[6];
    axis6_crawl_.start_abs = plc_act_pos_[5] + plc_init_pos_[5];
    axis6_crawl_.end_abs = axis6_crawl_.start_abs + kCrawlWindowSizeMm;
    axis6_crawl_.window_active = false;
    axis6_crawl_.phase = CrawlPhase::Follow;
    axis6_crawl_.phase_t0 = GetTickCount();
    axis6_crawl_.wait_rearm = false;
    axis6_crawl_.rearm_dir = 0;
    axis6_crawl_.enabled = false;

    return true;
}

bool ControlEngine::syncAxis1(int samples, bool wait_rearm, int rearm_dir)
{
    if (!readPlcState())
    {
        return false;
    }

    loadPosFromActual();

    axis1_crawl_.handle_ref = getAveragePos(*handle_axis1_, 0, samples);
    axis1_crawl_.rot_ref = handle_axis1_->fJoints2[1];
    axis1_crawl_.base_rel = plc_act_pos_[0];
    axis1_crawl_.rot_base_rel = plc_act_pos_[1];
    axis1_crawl_.start_abs = axis1StartAbs();
    axis1_crawl_.end_abs = axis1EndAbs();
    axis1_crawl_.window_active = isWithinRange(
        plc_act_pos_[0] + plc_init_pos_[0],
        (axis1_crawl_.start_abs < axis1_crawl_.end_abs) ? axis1_crawl_.start_abs : axis1_crawl_.end_abs,
        (axis1_crawl_.start_abs > axis1_crawl_.end_abs) ? axis1_crawl_.start_abs : axis1_crawl_.end_abs,
        kCrawlArriveTolMm);
    axis1_crawl_.phase = CrawlPhase::Follow;
    axis1_crawl_.phase_t0 = GetTickCount();
    axis1_crawl_.wait_rearm = wait_rearm;
    axis1_crawl_.rearm_dir = rearm_dir;

    axis3_base_rel_ = plc_act_pos_[2];
    axis5_base_rel_ = plc_act_pos_[4];
    axis6_mirror_base_rel_ = plc_act_pos_[5];

    return writeRefer();
}

bool ControlEngine::syncAxis6(int samples, bool capture_window, bool wait_rearm, int rearm_dir)
{
    if (!readPlcState())
    {
        return false;
    }

    loadPosFromActual();

    axis6_crawl_.handle_ref = getAveragePos(*handle_axis6_, 0, samples);
    axis6_crawl_.rot_ref = handle_axis6_->fJoints2[1];
    axis6_crawl_.base_rel = plc_act_pos_[5];
    axis6_crawl_.rot_base_rel = plc_act_pos_[6];
    if (capture_window || !axis6_crawl_.enabled)
    {
        axis6_crawl_.start_abs = plc_act_pos_[5] + plc_init_pos_[5];
        axis6_crawl_.end_abs = axis6_crawl_.start_abs + kCrawlWindowSizeMm;
    }
    axis6_crawl_.window_active = true;
    axis6_crawl_.phase = CrawlPhase::Follow;
    axis6_crawl_.phase_t0 = GetTickCount();
    axis6_crawl_.wait_rearm = wait_rearm;
    axis6_crawl_.rearm_dir = rearm_dir;
    axis6_crawl_.enabled = true;

    return writeRefer();
}

bool ControlEngine::releaseAxis6ToFollow()
{
    if (!readPlcState())
    {
        return false;
    }

    loadPosFromActual();

    axis6_crawl_.handle_ref = getAveragePos(*handle_axis6_, 0, 20);
    axis6_crawl_.rot_ref = handle_axis6_->fJoints2[1];
    axis6_crawl_.base_rel = plc_act_pos_[5];
    axis6_crawl_.rot_base_rel = plc_act_pos_[6];
    axis6_crawl_.phase = CrawlPhase::Follow;
    axis6_crawl_.phase_t0 = GetTickCount();
    axis6_crawl_.wait_rearm = false;
    axis6_crawl_.rearm_dir = 0;
    axis6_crawl_.enabled = false;
    axis6_crawl_.window_active = false;

    if (axis1_crawl_.phase == CrawlPhase::Follow)
    {
        const double axis1_base_abs = axis1_crawl_.base_rel + plc_init_pos_[0];
        const double axis1_min_abs = (axis1_crawl_.start_abs < axis1_crawl_.end_abs) ? axis1_crawl_.start_abs : axis1_crawl_.end_abs;
        const double axis1_max_abs = (axis1_crawl_.start_abs > axis1_crawl_.end_abs) ? axis1_crawl_.start_abs : axis1_crawl_.end_abs;
        const double axis1_hand_delta_mm =
            (handle_axis1_->fJoints2[0] - axis1_crawl_.handle_ref) * (kHandleToMmBase * handle_sensitivity_) * kAxisPushSign;
        const double axis1_cmd_abs = clampDouble(axis1_base_abs + axis1_hand_delta_mm, axis1_min_abs, axis1_max_abs);
        const double axis1_delta_rel = axis1_cmd_abs - plc_init_pos_[0] - axis1_crawl_.base_rel;
        axis6_mirror_base_rel_ = plc_act_pos_[5] - axis1_delta_rel;
    }
    else
    {
        axis6_mirror_base_rel_ = plc_act_pos_[5];
    }

    pos_[5] = plc_act_pos_[5];
    pos_[6] = plc_act_pos_[6];
    return writeRefer();
}

void ControlEngine::captureAxis1FollowBaseline()
{
    axis1_crawl_.handle_ref = handle_axis1_->fJoints2[0];
    axis1_crawl_.rot_ref = handle_axis1_->fJoints2[1];
    axis1_crawl_.base_rel = plc_act_pos_[0];
    axis1_crawl_.rot_base_rel = plc_act_pos_[1];
    axis3_base_rel_ = plc_act_pos_[2];
    axis5_base_rel_ = plc_act_pos_[4];
    if (!axis6_crawl_.enabled)
    {
        axis6_mirror_base_rel_ = plc_act_pos_[5];
    }
}

void ControlEngine::clearPlcReinitReq()
{
    if (ads_ == nullptr)
    {
        return;
    }
    bool clear_val = false;
    ads_->ADSWrite("G.handle_reinit_req", sizeof(clear_val), &clear_val);
    handle_reinit_req_ = false;
}

double ControlEngine::axis1StartAbs() const
{
    return plc_rightlimit_[0] - kCrawlWindowStartOffsetMm;
}

double ControlEngine::axis1EndAbs() const
{
    return axis1StartAbs() + kCrawlWindowSizeMm;
}

PlcState ControlEngine::toPlcState(int raw)
{
    switch (raw)
    {
    case 0:
        return PlcState::Init;
    case 1:
        return PlcState::Jog1;
    case 2:
        return PlcState::Jog2;
    case 3:
        return PlcState::Jog3;
    case 4:
        return PlcState::Trans;
    case 5:
        return PlcState::TransWait;
    case 6:
        return PlcState::Handle;
    case 7:
        return PlcState::SelfCheck;
    case 8:
        return PlcState::ClearErr;
    case 9:
        return PlcState::Err;
    case 10:
        return PlcState::Reset;
    default:
        return PlcState::Init;
    }
}
