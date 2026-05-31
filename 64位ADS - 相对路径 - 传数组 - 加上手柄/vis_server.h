#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#pragma pack(push, 1)
struct VisState
{
	double axis_pos[7];
	double axis_pos_from_left[7];
	unsigned short cylinder_cmd[4];
	int guidewire_mode;
	int axis1_phase;
	int axis6_phase;
	int startup_phase;
	bool control_active;
	bool freeze_active;
	bool estop_hold;
	bool axis1_fast_return;
	bool axis6_fast_retract;
	bool self_check_done;
	bool ff_enabled;
	bool cal_zeroed;
	bool axis1_reverse;
	bool axis6_reverse;
	bool force_log_running;
	bool startup_waiting;
	bool startup_completed;
	double ft_1_v;
	double fn_1_v;
	double force_582_f;
	double force_582_n;
	double force_587_f;
	double force_587_n;
	int loop_count;
	DWORD tick_ms;
};
#pragma pack(pop)

enum class VisCommandType : int
{
	None = 0,
	SetCylinderOverride = 1,
	ClearCylinderOverride = 2,
	RequestModeSwitch = 3,
	ZeroForceSensor = 4,
	ToggleForceFeedback = 5,
	SetReverseMode = 6,
	ToggleForceLog = 7,
	SetStartupAxisPos = 8,
	SetStartupAxisDeg = 9,
	SetStartupSpeed = 10,
	ExecuteStartup = 11,
	SelectDirectControl = 12,
};

#pragma pack(push, 1)
struct VisCommand
{
	VisCommandType type = VisCommandType::None;
	int param1 = 0;
	int param2 = 0;
};
#pragma pack(pop)

class VisServer
{
public:
	VisServer() = default;
	~VisServer();

	bool start();
	void stop();

	void push_state(const VisState& state);
	bool poll_command(VisCommand& cmd);

private:
	void server_loop();

	std::atomic<bool> running_{ false };
	std::atomic<bool> stop_requested_{ false };
	std::thread server_thread_;

	CRITICAL_SECTION state_cs_;
	VisState latest_state_{};
	bool state_dirty_ = false;

	CRITICAL_SECTION cmd_cs_;
	static constexpr int kCmdQueueSize = 16;
	VisCommand cmd_queue_[kCmdQueueSize]{};
	int cmd_head_ = 0;
	int cmd_tail_ = 0;
};
