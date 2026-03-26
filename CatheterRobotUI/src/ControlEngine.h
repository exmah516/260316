#pragma once

#include "SharedState.h"
#include "legacy/ADSComm1.h"
#include "legacy/Handle.h"

class ControlEngine
{
public:
    explicit ControlEngine(SharedState& shared);
    ~ControlEngine();

    bool init();
    bool tick();
    void shutdown();

private:
    struct CrawlState
    {
        bool enabled = false;
        CrawlPhase phase = CrawlPhase::Follow;
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

    static constexpr DWORD kSerialAxis1Handle = 582;
    static constexpr DWORD kSerialAxis6Handle = 587;
    static constexpr int kAxialForceAxis = 1;
    static constexpr double kAxialForceSign = -1.0;
    static constexpr double kHandleToMmBase = 500.0 * (75.0 / 50.0);
    static constexpr double kAxisPushSign = -1.0;
    static constexpr double kAxisRotScaleDeg = 57.29577951308232;
    static constexpr unsigned char kButtonMaskB0 = 0x01;
    static constexpr unsigned char kButtonMaskB6 = 0x40;
    static constexpr unsigned char kButtonMaskB7 = 0x80;
    static constexpr unsigned char kAxis1PauseButtonMask = kButtonMaskB6 | kButtonMaskB0;
    static constexpr unsigned char kAxis6ArmButtonMask = kButtonMaskB6 | kButtonMaskB0;
    static constexpr double kCrawlWindowStartOffsetMm = 56.0;
    static constexpr double kCrawlWindowSizeMm = 30.0;
    static constexpr double kCrawlTriggerDeadbandMm = 0.3;
    static constexpr double kCrawlRearmThresholdMm = 0.3;
    static constexpr double kCrawlArriveTolMm = 0.2;
    static constexpr DWORD kCrawlSwitchDelayMs = 250;
    static constexpr DWORD kCrawlClampDelayMs = 50;
    static constexpr unsigned short kCyl1Open = 1000;
    static constexpr unsigned short kCyl1Clamp = 100;
    static constexpr unsigned short kCyl2Open = 0;
    static constexpr unsigned short kCyl2Clamp = 1000;
    static constexpr unsigned short kCyl3Open = 1000;
    static constexpr unsigned short kCyl3Clamp = 100;
    static constexpr unsigned short kCyl4Open = 0;
    static constexpr unsigned short kCyl4Clamp = 1000;
    static constexpr unsigned short kCyl3FollowRelease = 150;
    static constexpr unsigned short kCyl4FollowRelease = 100;

    SharedState& shared_;

    Handle* handle_axis1_ = nullptr;
    Handle* handle_axis6_ = nullptr;
    CADSComm* ads_ = nullptr;

    double pos_[7] = {};
    double plc_act_pos_[7] = {};
    double plc_init_pos_[7] = {};
    double plc_rightlimit_[7] = {};

    CrawlState axis1_crawl_;
    CrawlState axis6_crawl_;
    double axis3_base_rel_ = 0.0;
    double axis5_base_rel_ = 0.0;
    double axis6_mirror_base_rel_ = 0.0;

    bool has_self_check_flag_ = false;
    bool control_active_ = false;
    bool freeze_active_ = false;
    bool estop_hold_active_ = false;
    bool estop_hold_req_ = false;
    bool self_check_done_ = false;
    bool last_self_check_done_ = false;
    bool handle_reinit_req_ = false;
    bool axis6_arm_pressed_prev_ = false;
    bool axis1_fast_return_ = false;
    bool axis6_fast_retract_ = false;
    int loop_count_ = 0;
    short plc_state_raw_ = 0;

    ControlMode current_mode_ = ControlMode::Catheter;

    bool force_feedback_enabled_ = false;
    double force_gain_axial_ = 1.0;
    double force_gain_torque_ = 1.0;
    short fn_raw_ = 0;
    short ft_raw_ = 0;
    double fn_bias_ = 0.0;
    bool fn_bias_inited_ = false;
    double fn_force_f_ = 0.0;
    double ft_force_f_ = 0.0;
    int last_fn_raw_ = 0;
    int last_ft_raw_ = 0;
    short fn_last_valid_ = 0;
    short ft_last_valid_ = 0;
    bool fn_has_valid_ = false;
    bool ft_has_valid_ = false;
    int fn_invalid_streak_ = 0;
    int ft_invalid_streak_ = 0;

    bool recording_ = false;
    DWORD record_start_tick_ = 0;

    double axis4_velocity_ = 1.5;
    double handle_sensitivity_ = 1.0;
    std::array<unsigned short, 4> cylinder_cmds_ = {
        kCyl1Open, kCyl2Clamp, kCyl3FollowRelease, kCyl4FollowRelease
    };

    DWORD last_rate_tick_ = 0;
    int rate_counter_ = 0;
    double current_hz_ = 0.0;

    bool readPlcState();
    bool writeRefer();
    void loadPosFromActual();
    void applyUICommands(const UIToControl& cmd);
    void applyCmdForce(double cmd_force);
    void publishState();
    void updateControlMode();
    void computeLoopRate();
    bool syncAll(int samples);
    bool syncAxis1(int samples, bool wait_rearm, int rearm_dir);
    bool syncAxis6(int samples, bool capture_window, bool wait_rearm, int rearm_dir);
    bool releaseAxis6ToFollow();
    void captureAxis1FollowBaseline();
    void clearPlcReinitReq();
    double axis1StartAbs() const;
    double axis1EndAbs() const;
    static PlcState toPlcState(int raw);
};
