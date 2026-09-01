#pragma once

#include <array>
#include <cstdint>
#include <string>

enum class ProgrammedDeliveryMode : std::uint8_t
{
	Legacy = 0,
	Catheter = 1,
	Guidewire = 2
};

enum class ProgrammedDeliveryPhase : std::uint8_t
{
	Idle = 0,
	SetupMove = 1,
	Ready = 2,
	Baseline = 3,
	ForwardToTrigger = 4,
	ReleaseMovingClamp = 5,
	ReturnMoving = 6,
	ReclampMovingClamp = 7,
	CycleDecision = 8,
	FinalForward = 9,
	Completed = 10,
	Aborted = 11,
	Error = 12
};

struct ProgrammedDeliveryConfig
{
	ProgrammedDeliveryMode mode = ProgrammedDeliveryMode::Catheter;
	double axis1_prepare_from_left_mm = 23.0;
	double axis1_trigger_from_left_mm = 3.0;
	double axis5_from_left_mm = 430.0;
	double axis6_prepare_from_left_mm = 451.0;
	double axis6_trigger_from_left_mm = 431.0;
	double axis2_angle_deg = 0.0;
	double axis7_angle_deg = 0.0;
	std::uint16_t cycle_count = 1;
	double final_forward_distance_mm = 10.0;
	bool cylinder1_coupling_enabled = true;
	bool cylinder3_coupling_enabled = true;
	// 运动端电缸的释放值和夹紧值：导管使用电缸2，导丝使用电缸4。
	std::uint16_t cylinder2_open_word = 0;
	std::uint16_t cylinder2_close_word = 600;
	std::uint16_t cylinder4_open_word = 0;
	std::uint16_t cylinder4_close_word = 500;
	std::uint32_t release_wait_ms = 150;
	std::uint32_t reclamp_wait_ms = 150;
	double forward_velocity_mm_s = 20.0;
	double forward_acceleration_mm_s2 = 100.0;
	double forward_deceleration_mm_s2 = 100.0;
	double forward_jerk_mm_s3 = 1000.0;
	double return_velocity_mm_s = 20.0;
	double return_acceleration_mm_s2 = 100.0;
	double return_deceleration_mm_s2 = 100.0;
	double return_jerk_mm_s3 = 1000.0;
	std::string record_suffix = "program_test";
};

struct ProgrammedDeliveryLiveFrame
{
	ProgrammedDeliveryMode mode = ProgrammedDeliveryMode::Legacy;
	ProgrammedDeliveryPhase phase = ProgrammedDeliveryPhase::Idle;
	std::uint16_t cycle_index = 0;
	std::uint16_t cycle_total = 0;
	bool setup_busy = false;
	bool setup_done = false;
	bool selfcheck_done = false;
	bool selfcheck_busy = false;
	std::uint32_t status_error_id = 0;
	std::uint8_t wait_action = 0;
	std::uint8_t error_source = 0;
	std::uint8_t error_axis = 0;
	std::uint8_t error_phase = 0;
	double error_target_abs_mm = 0.0;
	double error_target_from_left_mm = 0.0;
	double leftlimit_axis1_abs_mm = 0.0;
	double leftlimit_axis5_abs_mm = 0.0;
	double leftlimit_axis6_abs_mm = 0.0;
	double target_axis1_abs_mm = 0.0;
	double target_axis5_abs_mm = 0.0;
	double target_axis6_abs_mm = 0.0;
	double target_axis2_deg = 0.0;
	double target_axis7_deg = 0.0;
	double trigger_target_abs_mm = 0.0;
	double return_target_abs_mm = 0.0;
	double final_target_abs_mm = 0.0;
	double axis1_pos = 0.0, axis1_vel = 0.0, axis1_acc = 0.0;
	double axis2_pos = 0.0, axis2_vel = 0.0, axis2_acc = 0.0;
	double axis5_pos = 0.0, axis5_vel = 0.0, axis5_acc = 0.0;
	double axis6_pos = 0.0, axis6_vel = 0.0, axis6_acc = 0.0;
	double axis7_pos = 0.0, axis7_vel = 0.0, axis7_acc = 0.0;
	short fn1 = 0, ft1 = 0, fn2 = 0, ft2 = 0;
	std::uint16_t cylinder1 = 0, cylinder2 = 0, cylinder3 = 0, cylinder4 = 0;
	bool valid = false;
	bool recording = false;
	bool recording_overflow = false;
	std::uint32_t recording_sample_count = 0;
	std::uint32_t recording_error_id = 0;
	bool zero_busy = false;
	bool zero_done = false;
	std::array<double, 4> zero_values{};
};

struct ProgrammedDeliverySample
{
	std::uint32_t sample_index = 0;
	std::uint64_t plc_time_us = 0;
	std::uint8_t phase = 0;
	std::uint32_t event_sequence = 0;
	std::uint16_t cycle_index = 0;
	double axis1_pos = 0.0, axis1_vel = 0.0, axis1_acc = 0.0;
	double axis2_pos = 0.0, axis2_vel = 0.0, axis2_acc = 0.0;
	double axis5_pos = 0.0, axis5_vel = 0.0, axis5_acc = 0.0;
	double axis6_pos = 0.0, axis6_vel = 0.0, axis6_acc = 0.0;
	double axis7_pos = 0.0, axis7_vel = 0.0, axis7_acc = 0.0;
	std::uint16_t cylinder1 = 0, cylinder2 = 0, cylinder3 = 0, cylinder4 = 0;
	short fn1 = 0, ft1 = 0, fn2 = 0, ft2 = 0;
};

const char* programmed_delivery_mode_name(ProgrammedDeliveryMode mode);
const char* programmed_delivery_phase_name(ProgrammedDeliveryPhase phase);
