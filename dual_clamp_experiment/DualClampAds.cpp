#include "DualClampAds.h"
#include "AdsArrayRead.h"

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
	constexpr const char* kSelfcheckReq = "G.selfcheck_start_req";
	constexpr const char* kSelfcheckDone = "G.self_check_done";
	constexpr const char* kSelfcheckBusy = "G.selfcheck_status";
	constexpr const char* kSelfcheckError = "G.dual_clamp_selfcheck_error";
	constexpr const char* kSelfcheckErrorId = "G.dual_clamp_selfcheck_error_id";
	constexpr const char* kLeftlimitValid = "G.self_check_done";
	constexpr const char* kLeftlimitAxis1 = "G.leftlimit[1]";
	constexpr const char* kLeftlimitAxis6 = "G.leftlimit[6]";
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
	if (is_open()) return true;
	close();
	last_error_.clear();
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
				if (ads_state != 5)
				{
					last_error_ = "PLC不在RUN状态";
					close();
					return false;
				}
				host_comm_.SetTimeout(1000);
				if (!(host_comm_.OpenCommInsideReadOnly() || host_comm_.OpenCommReadOnly()) ||
					!host_comm_.SetTimeout(100))
				{
					last_error_ = "独立心跳连接失败：" + host_comm_.GetLastErrorCopy();
					close();
					return false;
				}
				if (!begin_host_session())
				{
					if (last_error_.empty()) last_error_ = comm_.GetLastErrorCopy();
					close();
					return false;
				}
				host_running_.store(true);
				session_ready_.store(true);
				try { host_thread_ = std::thread(&DualClampAds::host_loop, this); }
				catch (const std::exception& error)
				{
					last_error_ = error.what();
					close();
					return false;
				}
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

bool DualClampAds::begin_host_session()
{
	std::uint32_t version = 0;
	if (!comm_.ADSRead("G.program_interface_version", sizeof(version), &version) || version != 20261006)
	{
		last_error_ = "PLC接口缺失或版本不匹配：需要 G.program_interface_version UDINT=20261006；" + comm_.GetLastErrorCopy();
		return false;
	}
	SelfCheckState state;
	if (!read_self_check(host_comm_, state))
	{
		last_error_ = state.error;
		return false;
	}
	if (state.status == 2 || state.start_pending)
	{
		last_error_ = "自检正在运行或请求待消费，拒绝重建会话";
		return false;
	}
	std::uint32_t session = 0;
	std::uint8_t program_phase = 0;
	if (state.done)
	{
		if (!comm_.ADSRead("G.program_test_phase", sizeof(program_phase), &program_phase)) return false;
		if (program_phase >= 1 && program_phase <= 9)
		{
			last_error_ = "程序递送尚未中止，拒绝重建会话";
			return false;
		}
	}
	if (!comm_.ADSRead("G.host_session_id", sizeof(session), &session)) return false;
	bool recover = false;
	if (!comm_.ADSRead("G.host_recover_req", sizeof(recover), &recover) ||
		!host_comm_.ADSRead("G.host_heartbeat_sequence", sizeof(heartbeat_sequence_), &heartbeat_sequence_)) return false;
	if (++session == 0) ++session;
	heartbeat_sequence_ = 1;
	recover = true;
	if (!comm_.ADSWrite("G.host_session_id", sizeof(session), &session) ||
		!comm_.ADSWrite("G.host_heartbeat_sequence", sizeof(heartbeat_sequence_), &heartbeat_sequence_) ||
		!comm_.ADSWrite("G.host_recover_req", sizeof(recover), &recover)) return false;
	std::lock_guard<std::mutex> lock(host_mutex_);
	self_check_ = state;
	host_updated_ = std::chrono::steady_clock::now();
	return true;
}

void DualClampAds::close()
{
	session_ready_.store(false);
	host_running_.store(false);
	if (host_thread_.joinable()) host_thread_.join();
	host_comm_.CloseComm();
	if (comm_.IsCommOpen()) comm_.CloseComm();
	std::lock_guard<std::mutex> lock(host_mutex_);
	self_check_ = {};
}

bool DualClampAds::is_open() const
{
	return session_ready_.load() && comm_.IsCommOpen();
}

std::string DualClampAds::last_error() const
{
	return last_error_.empty() ? comm_.GetLastErrorCopy() : last_error_;
}

bool DualClampAds::read_self_check(CADSComm& comm, SelfCheckState& state)
{
	state = {};
	const char* symbols[] = { kSelfcheckDone, kSelfcheckBusy, kSelfcheckReq, "G.host_comm_timeout", "G.gen_state" };
	const unsigned long lengths[] = { sizeof(state.done), sizeof(state.status), sizeof(state.start_pending), sizeof(state.host_timeout), sizeof(state.gen_state) };
	void* outputs[] = { &state.done, &state.status, &state.start_pending, &state.host_timeout, &state.gen_state };
	if (!comm.ADSReadSum(symbols, lengths, outputs, 5))
	{
		state.error = "自检接口读取失败：" + comm.GetLastErrorCopy();
		return false;
	}
	state.valid = state.status >= 0 && state.status <= 4;
	if (!state.valid) state.error = "G.selfcheck_status 超出DINT状态范围0..4";
	else if (state.gen_state == 9 || (state.status == 2 && state.gen_state != 7))
	{
		state.valid = false;
		state.error = "PLC自检已离开运行状态或进入错误状态，gen_state=" + std::to_string(state.gen_state);
	}
	else if (state.done != (state.status == 4))
	{
		state.valid = false;
		state.error = "PLC自检完成标志与状态不一致";
	}
	return state.valid;
}

void DualClampAds::host_loop()
{
	while (host_running_.load())
	{
		SelfCheckState state;
		++heartbeat_sequence_;
		if (!host_comm_.ADSWrite("G.host_heartbeat_sequence", sizeof(heartbeat_sequence_), &heartbeat_sequence_))
		{
			state.error = "独立心跳写入失败：" + host_comm_.GetLastErrorCopy();
			session_ready_.store(false);
			host_running_.store(false);
		}
		else read_self_check(host_comm_, state);
		{
			std::lock_guard<std::mutex> lock(host_mutex_);
			self_check_ = state;
			host_updated_ = std::chrono::steady_clock::now();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

SelfCheckState DualClampAds::self_check() const
{
	std::lock_guard<std::mutex> lock(host_mutex_);
	SelfCheckState state = self_check_;
	if (!session_ready_.load() || std::chrono::steady_clock::now() - host_updated_ >= std::chrono::milliseconds(100))
	{
		state.valid = false;
		if (state.error.empty()) state.error = "自检状态未知：会话未连接或状态已过期";
	}
	return state;
}

bool DualClampAds::read_live(DualClampLiveFrame& frame)
{
	frame.valid = false;
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
	bool selfcheck_done = false, selfcheck_error = false;
	std::int32_t selfcheck_busy = 0;
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
	frame.selfcheck_busy = selfcheck_busy == 2;
	frame.selfcheck_error = selfcheck_error || selfcheck_busy == 3;
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
	last_error_.clear();
	SelfCheckState state;
	if (!is_open() || !read_self_check(comm_, state))
	{
		last_error_ = state.error.empty() ? "请先连接ADS主机会话" : state.error;
		return false;
	}
	if (state.status == 4 && state.done && !state.start_pending)
	{
		last_error_.clear();
		return true;
	}
	if (state.start_pending || state.gen_state != 7 || (state.status != 1 && state.status != 3))
	{
		last_error_ = "自检启动被拒绝：请求待消费或PLC不在自检等待/拒绝状态，status=" + std::to_string(state.status) + ", gen_state=" + std::to_string(state.gen_state);
		return false;
	}
	bool request = true;
	if (!comm_.ADSWrite(kSelfcheckReq, sizeof(request), &request)) return false;
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
	do
	{
		if (!read_self_check(comm_, state)) break;
		if (!state.start_pending && (state.status == 2 || state.status == 4)) return true;
		if (!state.start_pending && state.status == 3)
		{
			last_error_ = "PLC已消费并拒绝自检请求（status=3）";
			return false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	} while (std::chrono::steady_clock::now() < deadline);
	request = false;
	const bool cancelled = comm_.ADSWrite(kSelfcheckReq, sizeof(request), &request);
	last_error_ = "自检请求未获得PLC消费确认，启动结果未知；" + state.error +
		(cancelled ? "已撤回请求，请核对PLC状态" : "撤回失败，请勿重复启动：" + comm_.GetLastErrorCopy());
	return false;
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
			if (!read_ads_array(comm_, bases[i], offset, chunk,
				static_cast<unsigned long>(element_sizes[i]), buffers[i])) return false;
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
