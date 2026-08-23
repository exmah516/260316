#include "DualClampAds.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <thread>

namespace
{
	// 位置读取使用NC绝对实际位置；实验目标由PLC根据左限位计算。
	constexpr const char* kAxis1ActPos = "G.axis[1].NcToPlc.ActPos";
	constexpr const char* kAxis1ActVelo = "G.axis[1].NcToPlc.ActVelo";
	constexpr const char* kAxis1ActAcc = "G.axis[1].NcToPlc.ActAcc";
	constexpr const char* kAxis2ActPos = "G.axis[2].NcToPlc.ActPos";
	constexpr const char* kAxis6ActPos = "G.axis[6].NcToPlc.ActPos";
	constexpr const char* kAxis6ActVelo = "G.axis[6].NcToPlc.ActVelo";
	constexpr const char* kAxis6ActAcc = "G.axis[6].NcToPlc.ActAcc";
	constexpr const char* kAxis7ActPos = "G.axis[7].NcToPlc.ActPos";
	constexpr const char* kFt1 = "G.ft_1_value";
	constexpr const char* kFn1 = "G.fn_1_value";
	constexpr const char* kFt2 = "G.ft_2_value";
	constexpr const char* kFn2 = "G.fn_2_value";
	constexpr const char* kCylinder2 = "G.cylinder2_value";
	constexpr const char* kCylinder4 = "G.cylinder4_value";
	constexpr const char* kPhase = "G.dual_clamp_phase";
	constexpr const char* kEventSequence = "G.dual_clamp_event_sequence";
	constexpr const char* kSelfcheckReq = "G.dual_clamp_selfcheck_req";
	constexpr const char* kSelfcheckDone = "G.dual_clamp_selfcheck_done";
	constexpr const char* kSelfcheckBusy = "G.dual_clamp_selfcheck_busy";
	constexpr const char* kSelfcheckError = "G.dual_clamp_selfcheck_error";
	constexpr const char* kSelfcheckErrorId = "G.dual_clamp_selfcheck_error_id";
	constexpr const char* kLeftlimitValid = "G.dual_clamp_leftlimit_valid";
	constexpr const char* kLeftlimitAxis1 = "G.dual_clamp_leftlimit_axis1_abs";
	constexpr const char* kLeftlimitAxis6 = "G.dual_clamp_leftlimit_axis6_abs";
	constexpr const char* kSetupReq = "G.dual_clamp_setup_req";
	constexpr const char* kSetupBusy = "G.dual_clamp_setup_busy";
	constexpr const char* kSetupDone = "G.dual_clamp_setup_done";
	constexpr const char* kStartReq = "G.dual_clamp_start_req";
	constexpr const char* kAbortReq = "G.dual_clamp_abort_req";
	constexpr const char* kStatusErrorId = "G.dual_clamp_status_error_id";
	constexpr const char* kMovingAxis = "G.dual_clamp_moving_axis";
	constexpr const char* kAxis1Distance = "G.dual_clamp_axis1_distance_from_left";
	constexpr const char* kAxis6Distance = "G.dual_clamp_axis6_distance_from_left";
	constexpr const char* kAxis2Angle = "G.dual_clamp_axis2_angle_abs";
	constexpr const char* kAxis7Angle = "G.dual_clamp_axis7_angle_abs";
	constexpr const char* kReturnRetractDistance = "G.dual_clamp_return_retract_distance";
	constexpr const char* kReturnVelocity = "G.dual_clamp_return_velocity";
	constexpr const char* kReturnAcceleration = "G.dual_clamp_return_acceleration";
	constexpr const char* kReturnDeceleration = "G.dual_clamp_return_deceleration";
	constexpr const char* kReturnJerk = "G.dual_clamp_return_jerk";
	constexpr const char* kRecoveryMode = "G.dual_clamp_recovery_mode";
	constexpr const char* kSetupTargetAxis1 = "G.dual_clamp_setup_target_axis1_abs";
	constexpr const char* kSetupTargetAxis6 = "G.dual_clamp_setup_target_axis6_abs";
	constexpr const char* kReturnTarget = "G.dual_clamp_return_target_abs";
	constexpr const char* kStartTargetAxis1 = "G.dual_clamp_start_target_axis1_abs";
	constexpr const char* kStartTargetAxis6 = "G.dual_clamp_start_target_axis6_abs";
	constexpr const char* kCylinder2Cmd = "G.dual_clamp_cylinder2_cmd";
	constexpr const char* kCylinder4Cmd = "G.dual_clamp_cylinder4_cmd";
	constexpr const char* kSampleArm = "G.dual_clamp_sample_arm";
	constexpr const char* kSampleClear = "G.dual_clamp_sample_clear";
	constexpr const char* kSampleCount = "G.dual_clamp_sample_count";
	constexpr const char* kSampleOverflow = "G.dual_clamp_sample_overflow";
	constexpr std::uint32_t kSampleCapacity = 32768;
	constexpr std::uint32_t kSampleChunkSize = 512;

	bool read_sum(CADSComm& comm, const char* const* symbols, const unsigned long* lengths, void* const* outputs, unsigned long count)
	{
		return comm.ADSReadSum(symbols, lengths, outputs, count);
	}
}

DualClampAds::DualClampAds() = default;

DualClampAds::~DualClampAds()
{
	close();
}

bool DualClampAds::open()
{
	if (comm_.IsCommOpen()) return true;
	// 首次建立 AMS 路由连接时给予 1000ms 充分握手时间，并重试最多 3 次，避免报 1861 超时
	for (int attempt = 0; attempt < 3; ++attempt)
	{
		comm_.SetTimeout(1000);
		if (comm_.OpenCommInsideReadOnly() || comm_.OpenCommReadOnly())
		{
			comm_.SetTimeout(1000);
			unsigned short ads_state = 0;
			unsigned short device_state = 0;
			if (comm_.ReadDeviceState(ads_state, device_state))
			{
				// 连接成功后切换为运行期超时 100ms
				comm_.SetTimeout(100);
				return true;
			}
		}
		comm_.CloseComm();
		if (attempt + 1 < 3)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
	}
	return false;
}

void DualClampAds::close()
{
	if (comm_.IsCommOpen()) comm_.CloseComm();
}

bool DualClampAds::is_open() const
{
	return comm_.IsCommOpen();
}

std::string DualClampAds::last_error() const
{
	return comm_.GetLastErrorCopy();
}

bool DualClampAds::read_live(DualClampLiveFrame& frame)
{
	double axis1_pos = 0.0;
	double axis1_vel = 0.0;
	double axis1_acc = 0.0;
	double axis2_pos = 0.0;
	double axis6_pos = 0.0;
	double axis6_vel = 0.0;
	double axis6_acc = 0.0;
	double axis7_pos = 0.0;
	short ft1 = 0;
	short fn1 = 0;
	short ft2 = 0;
	short fn2 = 0;
	unsigned short cyl2 = 0;
	unsigned short cyl4 = 0;
	unsigned short phase = 0;
	bool selfcheck_done = false, selfcheck_busy = false, selfcheck_error = false;
	bool leftlimit_valid = false, setup_busy = false, setup_done = false;
	std::uint32_t selfcheck_error_id = 0, status_error_id = 0;
	double leftlimit1 = 0.0, leftlimit6 = 0.0, setup_target1 = 0.0, setup_target6 = 0.0;
	double return_target = 0.0;
	const char* symbols[] = {
		kAxis1ActPos, kAxis1ActVelo, kAxis1ActAcc, kAxis2ActPos,
		kAxis6ActPos, kAxis6ActVelo, kAxis6ActAcc, kAxis7ActPos,
		kFt1, kFn1, kFt2, kFn2,
		kCylinder2, kCylinder4, kPhase, kSelfcheckDone, kSelfcheckBusy, kSelfcheckError,
		kLeftlimitValid, kSetupBusy, kSetupDone, kSelfcheckErrorId, kStatusErrorId,
		kLeftlimitAxis1, kLeftlimitAxis6, kSetupTargetAxis1, kSetupTargetAxis6, kReturnTarget
	};
	const unsigned long lengths[] = {
		sizeof(axis1_pos), sizeof(axis1_vel), sizeof(axis1_acc), sizeof(axis2_pos),
		sizeof(axis6_pos), sizeof(axis6_vel), sizeof(axis6_acc), sizeof(axis7_pos),
		sizeof(ft1), sizeof(fn1), sizeof(ft2), sizeof(fn2),
		sizeof(cyl2), sizeof(cyl4), sizeof(phase), sizeof(selfcheck_done), sizeof(selfcheck_busy), sizeof(selfcheck_error),
		sizeof(leftlimit_valid), sizeof(setup_busy), sizeof(setup_done), sizeof(selfcheck_error_id), sizeof(status_error_id),
		sizeof(leftlimit1), sizeof(leftlimit6), sizeof(setup_target1), sizeof(setup_target6), sizeof(return_target)
	};
	void* outputs[] = {
		&axis1_pos, &axis1_vel, &axis1_acc, &axis2_pos,
		&axis6_pos, &axis6_vel, &axis6_acc, &axis7_pos,
		&ft1, &fn1, &ft2, &fn2,
		&cyl2, &cyl4, &phase, &selfcheck_done, &selfcheck_busy, &selfcheck_error,
		&leftlimit_valid, &setup_busy, &setup_done, &selfcheck_error_id, &status_error_id,
		&leftlimit1, &leftlimit6, &setup_target1, &setup_target6, &return_target
	};
	if (!read_sum(comm_, symbols, lengths, outputs, static_cast<unsigned long>(std::size(symbols)))) return false;
	frame.axis1_pos_abs_mm = axis1_pos;
	frame.axis1_velocity_mm_s = axis1_vel;
	frame.axis1_acceleration_mm_s2 = axis1_acc;
	frame.axis2_angle_abs_deg = axis2_pos;
	frame.axis6_pos_abs_mm = axis6_pos;
	frame.axis6_velocity_mm_s = axis6_vel;
	frame.axis6_acceleration_mm_s2 = axis6_acc;
	frame.axis7_angle_abs_deg = axis7_pos;
	frame.ft_1_raw = ft1;
	frame.fn_1_raw = fn1;
	frame.ft_2_raw = ft2;
	frame.fn_2_raw = fn2;
	frame.cylinder2_cmd = cyl2;
	frame.cylinder4_cmd = cyl4;
	frame.plc_phase = phase;
	frame.selfcheck_done = selfcheck_done;
	frame.selfcheck_busy = selfcheck_busy;
	frame.selfcheck_error = selfcheck_error;
	frame.leftlimit_valid = leftlimit_valid;
	frame.setup_busy = setup_busy;
	frame.setup_done = setup_done;
	frame.status_error_id = status_error_id != 0 ? status_error_id : selfcheck_error_id;
	frame.leftlimit_axis1_abs_mm = leftlimit1;
	frame.leftlimit_axis6_abs_mm = leftlimit6;
	frame.setup_target_axis1_abs_mm = setup_target1;
	frame.setup_target_axis6_abs_mm = setup_target6;
	frame.return_target_abs_mm = return_target;
	frame.valid = true;
	return true;
}

bool DualClampAds::request_self_check()
{
	const bool request = true;
	return comm_.ADSWrite(kSelfcheckReq, sizeof(request), const_cast<bool*>(&request));
}

bool DualClampAds::write_experiment_config(const DualClampConfig& config, bool setup_request, bool start_request)
{
	const short moving_axis = static_cast<short>(config.moving_axis);
	const bool recovery = config.recovery_mode == DualClampRecoveryMode::Move;
	const bool setup = setup_request;
	const bool start = start_request;
	const char* symbols[] = {
		kMovingAxis, kAxis1Distance, kAxis6Distance, kAxis2Angle, kAxis7Angle,
		kReturnRetractDistance, kReturnVelocity, kReturnAcceleration, kReturnDeceleration, kReturnJerk,
		kRecoveryMode, kSetupReq, kStartReq
	};
	const unsigned long lengths[] = {
		sizeof(moving_axis), sizeof(config.axis1_distance_from_left_mm), sizeof(config.axis6_distance_from_left_mm),
		sizeof(config.axis2_angle_abs_deg), sizeof(config.axis7_angle_abs_deg), sizeof(config.return_retract_distance_mm),
		sizeof(config.return_velocity_mm_s), sizeof(config.return_acceleration_mm_s2), sizeof(config.return_deceleration_mm_s2),
		sizeof(config.return_jerk_mm_s3), sizeof(recovery), sizeof(setup), sizeof(start)
	};
	const void* inputs[] = {
		&moving_axis, &config.axis1_distance_from_left_mm, &config.axis6_distance_from_left_mm,
		&config.axis2_angle_abs_deg, &config.axis7_angle_abs_deg, &config.return_retract_distance_mm,
		&config.return_velocity_mm_s, &config.return_acceleration_mm_s2, &config.return_deceleration_mm_s2,
		&config.return_jerk_mm_s3, &recovery, &setup, &start
	};
	return comm_.ADSWriteSum(symbols, lengths, inputs, static_cast<unsigned long>(std::size(symbols)));
}

bool DualClampAds::request_abort()
{
	const bool request = true;
	return comm_.ADSWrite(kAbortReq, sizeof(request), const_cast<bool*>(&request));
}

bool DualClampAds::clear_sample_buffer()
{
	const bool clear = true;
	// PLC在处理完请求后自行清零，避免 true/false 连续写入被同一PLC周期吞掉。
	return comm_.ADSWrite(kSampleClear, sizeof(clear), const_cast<bool*>(&clear));
}

bool DualClampAds::read_sample_count(std::uint32_t& count, bool& overflow)
{
	if (!comm_.ADSRead(kSampleCount, sizeof(count), &count)) return false;
	if (!comm_.ADSRead(kSampleOverflow, sizeof(overflow), &overflow)) return false;
	count = (std::min)(count, kSampleCapacity);
	return true;
}

bool DualClampAds::read_all_samples(std::uint32_t count, std::vector<DualClampSample>& samples)
{
	count = (std::min)(count, kSampleCapacity);
	samples.clear();
	samples.resize(count);
	if (count == 0) return true;
	for (std::uint32_t offset = 0; offset < count; offset += kSampleChunkSize)
	{
		const std::uint32_t chunk = (std::min)(kSampleChunkSize, count - offset);
		std::vector<std::uint32_t> index(chunk);
		std::vector<std::uint64_t> time_us(chunk);
		std::vector<std::uint8_t> phase(chunk);
		std::vector<std::uint32_t> event_sequence(chunk);
		std::vector<double> axis1_pos(chunk), axis1_vel(chunk), axis1_acc(chunk);
		std::vector<double> axis6_pos(chunk), axis6_vel(chunk), axis6_acc(chunk);
		std::vector<short> ft1(chunk), fn1(chunk), ft2(chunk), fn2(chunk);
		std::vector<unsigned short> cyl2(chunk), cyl4(chunk);

		const std::array<const char*, 16> bases = {
			"G.dual_clamp_sample_index", "G.dual_clamp_sample_time_us", "G.dual_clamp_sample_phase",
			"G.dual_clamp_sample_event_sequence", "G.dual_clamp_sample_axis1_pos", "G.dual_clamp_sample_axis1_vel",
			"G.dual_clamp_sample_axis1_acc", "G.dual_clamp_sample_axis6_pos", "G.dual_clamp_sample_axis6_vel",
			"G.dual_clamp_sample_axis6_acc", "G.dual_clamp_sample_ft1", "G.dual_clamp_sample_fn1",
			"G.dual_clamp_sample_ft2", "G.dual_clamp_sample_fn2", "G.dual_clamp_sample_cylinder2",
			"G.dual_clamp_sample_cylinder4"};
		std::array<unsigned long, 16> lengths{};
		const std::array<unsigned long, 16> element_sizes = {
			sizeof(index[0]), sizeof(time_us[0]), sizeof(phase[0]), sizeof(event_sequence[0]),
			sizeof(axis1_pos[0]), sizeof(axis1_vel[0]), sizeof(axis1_acc[0]), sizeof(axis6_pos[0]),
			sizeof(axis6_vel[0]), sizeof(axis6_acc[0]), sizeof(ft1[0]), sizeof(fn1[0]), sizeof(ft2[0]),
			sizeof(fn2[0]), sizeof(cyl2[0]), sizeof(cyl4[0])};
		std::array<void*, 16> buffers = {
			index.data(), time_us.data(), phase.data(), event_sequence.data(), axis1_pos.data(), axis1_vel.data(),
			axis1_acc.data(), axis6_pos.data(), axis6_vel.data(), axis6_acc.data(), ft1.data(), fn1.data(),
			ft2.data(), fn2.data(), cyl2.data(), cyl4.data()};
		for (std::size_t i = 0; i < bases.size(); ++i)
		{
			lengths[i] = static_cast<unsigned long>(element_sizes[i] * chunk);
			const unsigned long byte_offset = static_cast<unsigned long>(element_sizes[i] * offset);
			if (!comm_.ADSReadSymbolOffset(bases[i], byte_offset, lengths[i], buffers[i])) return false;
		}

		for (std::uint32_t i = 0; i < chunk; ++i)
		{
			DualClampSample& s = samples[offset + i];
			s.sample_index = index[i];
			s.plc_time_us = time_us[i];
			s.phase = phase[i];
			s.event_sequence = event_sequence[i];
			s.axis1_pos_abs_mm = axis1_pos[i];
			s.axis1_velocity_mm_s = axis1_vel[i];
			s.axis1_acceleration_mm_s2 = axis1_acc[i];
			s.axis6_pos_abs_mm = axis6_pos[i];
			s.axis6_velocity_mm_s = axis6_vel[i];
			s.axis6_acceleration_mm_s2 = axis6_acc[i];
			s.ft_1_raw = ft1[i];
			s.fn_1_raw = fn1[i];
			s.ft_2_raw = ft2[i];
			s.fn_2_raw = fn2[i];
			s.cylinder2_cmd = cyl2[i];
			s.cylinder4_cmd = cyl4[i];
		}
	}
	return true;
}
