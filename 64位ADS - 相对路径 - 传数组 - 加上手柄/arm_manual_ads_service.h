#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <condition_variable>
#include "arm_cartesian_motion.h"

class AdsCommunicationService;

struct ArmManualSnapshot
{
	bool valid = false;
	bool manual_enable = false;
	std::uint32_t curve_sequence = 0;
	std::uint32_t curve_state = 0;
	std::uint32_t curve_error = 0;
	double curve_progress = 0;
	int cartesian_status = 0;
	int cartesian_error = 0;
	bool at_program_zero = false;
	std::uint32_t home_request_id = 0;
	std::array<bool, 5> enable_req{};
	std::array<bool, 5> power_done{};
	std::array<bool, 5> power_busy{};
	std::array<bool, 5> power_active{};
	std::array<bool, 5> power_error{};
	std::array<std::uint32_t, 5> power_error_id{};
	std::array<bool, 5> reset_done{};
	std::array<bool, 5> reset_busy{};
	std::array<bool, 5> reset_active{};
	std::array<bool, 5> reset_error{};
	std::array<std::uint32_t, 5> reset_error_id{};
	std::array<double, 5> act_pos{};
	std::array<double, 5> act_vel{};
	std::array<bool, 5> motion_busy{};
	std::array<bool, 5> motion_done{};
	std::array<bool, 5> motion_error{};
	std::array<std::uint32_t, 5> motion_error_id{};
	std::array<std::int8_t, 5> cmd_dir{};
	std::array<bool, 5> cmd_conflict{};
	std::array<double, 5> jog_velocity{ { 5.0, 5.0, 5.0, 5.0, 5.0 } };
	std::array<double, 5> jog_acc{ { 50.0, 50.0, 50.0, 50.0, 50.0 } };
	std::array<double, 5> jog_dec{ { 50.0, 50.0, 50.0, 50.0, 50.0 } };
	std::array<double, 5> jog_jerk{ { 500.0, 500.0, 500.0, 500.0, 500.0 } };
};

// 定位臂只通过 ADS 低频队列读写，避免进入介入机器人 7 轴 100 Hz 控制包。
class ArmManualAdsService
{
public:
	explicit ArmManualAdsService(AdsCommunicationService& ads_service);
	~ArmManualAdsService();

	bool start();
	void stop();

	ArmManualSnapshot snapshot() const;
	void set_manual_enable(bool enabled);
	void set_axis_enable(int axis_one_based, bool enabled);
	void request_reset(int axis_one_based);
	void set_jog_direction(int axis_one_based, int direction);
	bool set_jog_parameter(int axis_one_based, int parameter_kind, double value);
	void set_cartesian_jog(int mode, double speed);
	void request_program_zero();
	void stop_cartesian();
	void keep_cartesian_alive();
	void set_cartesian_parameter(int field, double value);

private:
	void run();
	bool write_dirty_request();
	bool read_snapshot();
	void clear_requests_locked(bool clear_manual_enable);
	void update_auto_enable_gate();
	void expire_jog_deadman();
	void update_cartesian();

	AdsCommunicationService& ads_service_;
	mutable std::mutex mutex_;
	std::condition_variable wake_cv_;
	std::thread worker_;
	bool running_ = false;
	bool stop_requested_ = false;
	bool user_disabled_manual_enable_ = false;

	bool desired_manual_enable_ = false;
	std::array<bool, 5> desired_enable_req_{};
	std::array<bool, 5> desired_reset_req_{};
	std::array<bool, 5> desired_jog_pos_req_{};
	std::array<bool, 5> desired_jog_neg_req_{};
	std::array<std::chrono::steady_clock::time_point, 5> jog_deadline_{};
	std::array<double, 5> desired_jog_velocity_{ { 5.0, 5.0, 5.0, 5.0, 5.0 } };
	std::array<double, 5> desired_jog_acc_{ { 50.0, 50.0, 50.0, 50.0, 50.0 } };
	std::array<double, 5> desired_jog_dec_{ { 50.0, 50.0, 50.0, 50.0, 50.0 } };
	std::array<double, 5> desired_jog_jerk_{ { 500.0, 500.0, 500.0, 500.0, 500.0 } };
	bool manual_dirty_ = true;
	bool enable_dirty_ = true;
	bool reset_dirty_ = false;
	bool jog_pos_dirty_ = true;
	bool jog_neg_dirty_ = true;
	bool params_dirty_ = true;
	ArmManualSnapshot snapshot_{};
	ArmCartesianSettings cartesian_settings_;
	int cartesian_mode_ = 0;
	double cartesian_speed_ = 0;
	bool home_requested_ = false;
	bool cartesian_stop_ = false;
	bool cartesian_blocked_ = false;
	bool curve_pending_ = false;
	std::uint32_t sent_curve_sequence_ = 0;
	std::uint32_t curve_heartbeat_ = 0;
	std::uint64_t cartesian_epoch_ = 0;
	std::chrono::steady_clock::time_point cartesian_deadline_{};
	std::chrono::steady_clock::time_point curve_sent_at_{};
	std::chrono::steady_clock::time_point snapshot_at_{};
};
