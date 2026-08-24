#pragma once

#include <array>
#include <cstdint>
#include <string>

enum class DualClampPhase : int
{
	Idle = 0,
	SelfCheck = 1,
	Baseline = 3,
	FixedHold = 4,
	ReleaseMoving = 5,
	ReturnMoving = 6,
	ReclampMoving = 7,
	RecoverHold = 8,
	RecoverMove = 9,
	Completed = 10,
	Aborted = 11,
	Error = 12,
	SelfCheckDone = 13,
	SetupMove = 14,
	ReadyForClamp = 15
};

enum class DualClampRecoveryMode : int
{
	Hold = 0,
	Move = 1
};

struct DualClampConfig
{
	int moving_axis = 1; // 1 或 6；另一端自动作为固定端。
	double axis1_distance_from_left_mm = 96.0;
	double axis6_distance_from_left_mm = 580.0;
	double axis2_angle_abs_deg = 0.0;
	double axis7_angle_abs_deg = 0.0;
	double return_retract_distance_mm = 10.0;
	double return_velocity_mm_s = 20.0;
	double return_acceleration_mm_s2 = 100.0;
	double return_deceleration_mm_s2 = 100.0;
	double return_jerk_mm_s3 = 1000.0;
	unsigned short clamp_axis1_word = 600;
	unsigned short release_axis1_word = 0;
	unsigned short clamp_axis6_word = 500;
	unsigned short release_axis6_word = 0;
	unsigned short cylinder1_open_word = 1000;
	unsigned short cylinder3_open_word = 1000;
	unsigned int baseline_ms = 1000;
	unsigned int clamp_settle_ms = 150;
	// 实验流程默认重新夹紧后恢复到实验开始位置，继续输送。
	DualClampRecoveryMode recovery_mode = DualClampRecoveryMode::Move;
	std::string experiment_name = "dual_clamp";
	std::string note;
	std::string record_suffix = "dual_clamp";
};

struct DualClampLiveFrame
{
	std::int64_t host_qpc = 0;
	std::uint64_t sample_sequence = 0;
	double axis1_pos_abs_mm = 0.0;
	double axis1_velocity_mm_s = 0.0;
	double axis1_acceleration_mm_s2 = 0.0;
	double axis2_angle_abs_deg = 0.0;
	double axis6_pos_abs_mm = 0.0;
	double axis6_velocity_mm_s = 0.0;
	double axis6_acceleration_mm_s2 = 0.0;
	double axis7_angle_abs_deg = 0.0;
	short ft_1_raw = 0;
	short fn_1_raw = 0;
	short ft_2_raw = 0;
	short fn_2_raw = 0;
	unsigned short cylinder2_cmd = 0;
	unsigned short cylinder4_cmd = 0;
	bool valid = false;
	unsigned short plc_phase = 0;
	bool selfcheck_done = false;
	bool selfcheck_busy = false;
	bool selfcheck_error = false;
	bool leftlimit_valid = false;
	bool setup_busy = false;
	bool setup_done = false;
	std::uint32_t status_error_id = 0;
	double leftlimit_axis1_abs_mm = 0.0;
	double leftlimit_axis6_abs_mm = 0.0;
	double setup_target_axis1_abs_mm = 0.0;
	double setup_target_axis6_abs_mm = 0.0;
	double return_target_abs_mm = 0.0;
	bool recording = false;
	bool recording_overflow = false;
	std::uint32_t recording_sample_count = 0;
	std::uint32_t recording_error_id = 0;
	bool zero_busy = false;
	bool zero_done = false;
	std::array<double, 4> zero_values{};
};

struct DualClampSample
{
	std::uint32_t sample_index = 0;
	std::uint64_t plc_time_us = 0;
	std::uint8_t phase = 0;
	std::uint32_t event_sequence = 0;
	double axis1_pos_abs_mm = 0.0;
	double axis1_velocity_mm_s = 0.0;
	double axis1_acceleration_mm_s2 = 0.0;
	double axis6_pos_abs_mm = 0.0;
	double axis6_velocity_mm_s = 0.0;
	double axis6_acceleration_mm_s2 = 0.0;
	short ft_1_raw = 0;
	short fn_1_raw = 0;
	short ft_2_raw = 0;
	short fn_2_raw = 0;
	unsigned short cylinder2_cmd = 0;
	unsigned short cylinder4_cmd = 0;
};

const char* dual_clamp_phase_name(DualClampPhase phase);
