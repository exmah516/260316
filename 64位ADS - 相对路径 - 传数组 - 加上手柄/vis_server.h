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
	double force_582_theory_f;
	double force_582_theory_n;
	bool gravity_comp_enabled;
	// 力过渡决定性预实验（论文 §6.1）状态字段。末尾追加保持二进制兼容。
	int ft_exp_phase;
	int ft_exp_velocity_level;
	int ft_exp_trial_id;
	int ft_exp_repeat_in_lvl;
	double ft_exp_v_ratio_curr;
	double ft_exp_axis1_target;
	bool ft_exp_active;
	bool ft_exp_aborted;
	// 手动屈曲/间距恢复状态。末尾追加保持既有字段布局不变。
	int spacing_recovery_phase;
	double spacing_recovery_moved_mm;
	double spacing_recovery_remaining_mm;
	// 协同递送状态。仅由上位机内部状态发布，不增加 PLC ADS 契约。
	bool dual_handle_ready;
	int cooperative_return_owner;
};
#pragma pack(pop)
static_assert(sizeof(VisState) == 281, "VisState 管道布局发生变化，请同步更新 WPF 协议结构。");

enum class VisCommandType : int
{
	None = 0,
	SetCylinderManualOpen = 1,
	SetCylinderManualClosed = 2,
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
	SetGravityCompensation = 13,
	// 力过渡决定性预实验（论文 §6.1）控制命令。
	StartForceTransitionExperiment = 14,
	StopForceTransitionExperiment = 15,
	SetFtExpParamA = 16, // param1=field_id, param2=int_val
	SetFtExpParamB = 17, // param1=field_id, param2=fixed-point val (×1000)
	SetSpacingRecovery = 18, // param1: 0=退出，1=进入
	SetCooperativeDelivery = 19, // param1: 0=退出，1=进入
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
