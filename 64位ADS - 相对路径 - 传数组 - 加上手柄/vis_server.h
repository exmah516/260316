#pragma once

#include <atomic>
#include <cstdint>
#include <string>
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
	// 主从位移补偿只发布控制状态，不再持有任何磁盘会话状态。
	bool tracking_compensation_enabled;
	double axis1_tracking_error_mm;
	double axis6_tracking_error_mm;
	double axis1_compensation_gain;
	double axis6_compensation_gain;
	// 协同方向：0=None，1=Delivery，2=Retraction。末尾追加要求 C++ 与 WPF 同步更新。
	int cooperative_direction;
	// axis6 当前软限位阻断状态。仅来自上位机，不增加 PLC ADS 契约。
	bool axis6_soft_limit_hold;

	// 统一实验记录、纯净力与 Action 4 状态。
	int recording_state;
	int recording_error;
	std::uint64_t recording_elapsed_us;
	std::uint64_t force_writer_dropped;
	std::uint64_t motion_writer_dropped;
	std::uint64_t force_schedule_missed;
	std::uint64_t motion_schedule_missed;
	bool force_sample_valid;
	bool clean_force_valid;
	double clean_force_n;
	double clean_handle_torque_nm;
	int camera_state;
	int camera_input_format;
	int camera_width;
	int camera_height;
	int camera_fps_numerator;
	int camera_fps_denominator;
	bool camera_preview_enabled;
	bool camera_recording;
	std::uint64_t camera_recording_elapsed_us;
	std::uint64_t camera_frame_count;
	std::uint64_t camera_dropped_frames;
	int camera_error_code;
	// 物理 B0/B6 的有效按下沿事件；counter 变化时 WPF 显示 event_code 对应提示。
	std::uint32_t physical_button_event_counter;
	int physical_button_event_code;
	// ADS 通信诊断。状态码：0=未启动，1=连接中，2=正常，3=软保持，4=重连中，5=PLC重启，6=错误。
	int ads_state;
	double ads_actual_hz;
	std::uint64_t ads_snapshot_age_us;
	std::uint64_t ads_rtt_us;
	std::uint64_t ads_failed_cycles;
	std::uint64_t ads_reconnect_count;
	std::uint64_t plc_restart_count;
	bool host_comm_timeout;

	// 定位臂独立低频 ADS 状态，末尾追加以保持既有字段偏移不变。
	bool arm_manual_enable;
	bool arm_enable_req[5];
	bool arm_power_done[5];
	bool arm_power_busy[5];
	bool arm_power_active[5];
	bool arm_power_error[5];
	std::uint32_t arm_power_error_id[5];
	bool arm_reset_done[5];
	bool arm_reset_busy[5];
	bool arm_reset_active[5];
	bool arm_reset_error[5];
	std::uint32_t arm_reset_error_id[5];
	double arm_act_pos[5];
	double arm_act_vel[5];
	bool arm_motion_busy[5];
	bool arm_motion_done[5];
	bool arm_motion_error[5];
	std::uint32_t arm_motion_error_id[5];
	std::int8_t arm_cmd_dir[5];
	bool arm_cmd_conflict[5];
	double arm_jog_velocity[5];
	double arm_jog_acc[5];
	double arm_jog_dec[5];
	double arm_jog_jerk[5];

	// Axis4 手动点动状态来自既有 PLC Notification。
	bool axis4_manual_busy;
	bool axis4_manual_done;
	bool axis4_manual_error;
	std::uint32_t axis4_manual_error_id;
};
#pragma pack(pop)
static_assert(sizeof(VisState) == 877, "VisState 管道布局发生变化，请同步更新 WPF 协议结构。");

enum class VisCommandType : int
{
	None = 0,
	SetCylinderManualOpen = 1,
	SetCylinderManualClosed = 2,
	RequestModeSwitch = 3,
	ZeroForceSensor = 4,
	ToggleForceFeedback = 5,
	SetReverseMode = 6,
	// 7 为已删除的旧力记录命令，保留数值空洞。
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
	// 20 为已删除的旧位移记录命令，保留数值空洞。
	SetTrackingCompensation = 21, // param1: 0=关闭，1=开启
	SetTrackingCompensationParam = 22, // param1=TrackingParameterField，param2=数值×1000
	SetCooperativeRetraction = 23, // param1: 0=退出，1=进入
	SetAxis1PostReturnLead = 24, // param1=轴1回退后先行量，单位 mm×1000，范围 [-10, 10]
	StartExperimentRecording = 25, // UTF-8 负载为实验名称
	StopExperimentRecording = 26,
	SetCameraPreview = 27, // param1: 0=关闭，1=打开
	SetCleanForceMonitor = 28, // param1: 0=关闭，1=打开
	SetArmManualEnable = 29, // param1: 0=关闭，1=开启
	SetArmAxisEnable = 30, // param1: 轴号1..5，param2: 0=断电，1=上电
	RequestArmAxisReset = 31, // param1: 轴号1..5
	SetArmAxisJog = 32, // param1: 轴号1..5，param2: -1/0/1
	SetArmJogParameter = 33, // param1=(轴号-1)*4+参数号，param2=值×1000
	SetAxis4ManualJog = 34, // param1: -1=后退，0=停止，1=前进（物理语义）
};

#pragma pack(push, 1)
struct VisWireCommandHeader
{
	std::uint32_t magic = 0x31434D56; // "VMC1"
	std::uint16_t version = 1;
	std::uint16_t header_size = sizeof(VisWireCommandHeader);
	VisCommandType type = VisCommandType::None;
	int param1 = 0;
	int param2 = 0;
	std::uint32_t payload_size = 0;
};
#pragma pack(pop)
static_assert(sizeof(VisWireCommandHeader) == 24, "可视化命令头布局发生变化，请同步更新 WPF 协议结构。");

struct VisCommand
{
	VisCommandType type = VisCommandType::None;
	int param1 = 0;
	int param2 = 0;
	std::string payload_utf8;
};

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
