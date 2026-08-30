#include "ProgrammedDeliveryAds.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <iterator>
#include <thread>

namespace
{
	constexpr std::uint32_t kSampleCapacity = 32768;
	constexpr std::uint32_t kSampleChunkSize = 512;

	bool read_chunk(CADSComm& comm, const char* symbol, std::uint32_t offset,
		std::uint32_t count, std::size_t element_size, void* output)
	{
		return comm.ADSReadSymbolOffset(
			symbol,
			static_cast<unsigned long>(offset * element_size),
			static_cast<unsigned long>(count * element_size),
			output);
	}
}

ProgrammedDeliveryAds::ProgrammedDeliveryAds() = default;

ProgrammedDeliveryAds::~ProgrammedDeliveryAds()
{
	close();
}

bool ProgrammedDeliveryAds::open()
{
	if (comm_.IsCommOpen()) return true;
	for (int attempt = 0; attempt < 3; ++attempt)
	{
		comm_.SetTimeout(1000);
		if (comm_.OpenCommInsideReadOnly() || comm_.OpenCommReadOnly())
		{
			unsigned short ads_state = 0;
			unsigned short device_state = 0;
			if (comm_.ReadDeviceState(ads_state, device_state))
			{
				comm_.SetTimeout(100);
				return true;
			}
		}
		comm_.CloseComm();
		if (attempt + 1 < 3) std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
	return false;
}

void ProgrammedDeliveryAds::close()
{
	if (comm_.IsCommOpen()) comm_.CloseComm();
}

bool ProgrammedDeliveryAds::is_open() const
{
	return comm_.IsCommOpen();
}

std::string ProgrammedDeliveryAds::last_error() const
{
	return comm_.GetLastErrorCopy();
}

bool ProgrammedDeliveryAds::select_mode(ProgrammedDeliveryMode mode)
{
	const std::uint8_t value = static_cast<std::uint8_t>(mode);
	return comm_.ADSWrite("G.program_test_mode", sizeof(value), const_cast<std::uint8_t*>(&value));
}

bool ProgrammedDeliveryAds::read_live(ProgrammedDeliveryLiveFrame& frame)
{
	std::uint8_t mode = 0, phase = 0;
	std::uint16_t cycle_index = 0, cycle_total = 0;
	bool setup_busy = false, setup_done = false, selfcheck_done = false, selfcheck_busy = false;
	std::uint32_t status_error_id = 0;
	std::uint8_t wait_action = 0, error_source = 0, error_axis = 0, error_phase = 0;
	double error_target_abs = 0.0;
	double left1 = 0.0, left5 = 0.0, left6 = 0.0;
	double target1 = 0.0, target5 = 0.0, target6 = 0.0, target2 = 0.0, target7 = 0.0;
	double trigger = 0.0, return_target = 0.0, final_target = 0.0;
	double a1p = 0.0, a1v = 0.0, a1a = 0.0, a2p = 0.0, a2v = 0.0, a2a = 0.0;
	double a5p = 0.0, a5v = 0.0, a5a = 0.0, a6p = 0.0, a6v = 0.0, a6a = 0.0;
	double a7p = 0.0, a7v = 0.0, a7a = 0.0;
	short fn1 = 0, ft1 = 0, fn2 = 0, ft2 = 0;
	std::uint16_t c1 = 0, c2 = 0, c3 = 0, c4 = 0;

	const char* symbols[] = {
		"G.program_test_mode", "G.program_test_phase", "G.program_test_cycle_index", "G.program_test_cycle_total",
		"G.program_test_setup_busy", "G.program_test_setup_done", "G.dual_clamp_selfcheck_done", "G.dual_clamp_selfcheck_busy", "G.program_test_status_error_id",
		"G.program_test_wait_action", "G.program_test_error_source", "G.program_test_error_axis", "G.program_test_error_phase", "G.program_test_error_target_abs",
		"G.leftlimit[1]", "G.leftlimit[5]", "G.leftlimit[6]",
		"G.program_test_target_axis1_abs", "G.program_test_target_axis5_abs", "G.program_test_target_axis6_abs",
		"G.program_test_target_axis2_deg", "G.program_test_target_axis7_deg", "G.program_test_trigger_target_abs",
		"G.program_test_return_target_abs", "G.program_test_final_target_abs",
		"G.axis[1].NcToPlc.ActPos", "G.axis[1].NcToPlc.ActVelo", "G.axis[1].NcToPlc.ActAcc",
		"G.axis[2].NcToPlc.ActPos", "G.axis[2].NcToPlc.ActVelo", "G.axis[2].NcToPlc.ActAcc",
		"G.axis[5].NcToPlc.ActPos", "G.axis[5].NcToPlc.ActVelo", "G.axis[5].NcToPlc.ActAcc",
		"G.axis[6].NcToPlc.ActPos", "G.axis[6].NcToPlc.ActVelo", "G.axis[6].NcToPlc.ActAcc",
		"G.axis[7].NcToPlc.ActPos", "G.axis[7].NcToPlc.ActVelo", "G.axis[7].NcToPlc.ActAcc",
		"G.fn_1_value", "G.ft_1_value", "G.fn_2_value", "G.ft_2_value",
		"G.cylinder1_value", "G.cylinder2_value", "G.cylinder3_value", "G.cylinder4_value"
	};
	const unsigned long lengths[] = {
		sizeof(mode), sizeof(phase), sizeof(cycle_index), sizeof(cycle_total),
		sizeof(setup_busy), sizeof(setup_done), sizeof(selfcheck_done), sizeof(selfcheck_busy), sizeof(status_error_id),
		sizeof(wait_action), sizeof(error_source), sizeof(error_axis), sizeof(error_phase), sizeof(error_target_abs),
		sizeof(left1), sizeof(left5), sizeof(left6),
		sizeof(target1), sizeof(target5), sizeof(target6), sizeof(target2), sizeof(target7), sizeof(trigger),
		sizeof(return_target), sizeof(final_target),
		sizeof(a1p), sizeof(a1v), sizeof(a1a), sizeof(a2p), sizeof(a2v), sizeof(a2a),
		sizeof(a5p), sizeof(a5v), sizeof(a5a), sizeof(a6p), sizeof(a6v), sizeof(a6a),
		sizeof(a7p), sizeof(a7v), sizeof(a7a),
		sizeof(fn1), sizeof(ft1), sizeof(fn2), sizeof(ft2), sizeof(c1), sizeof(c2), sizeof(c3), sizeof(c4)
	};
	void* outputs[] = {
		&mode, &phase, &cycle_index, &cycle_total,
		&setup_busy, &setup_done, &selfcheck_done, &selfcheck_busy, &status_error_id,
		&wait_action, &error_source, &error_axis, &error_phase, &error_target_abs,
		&left1, &left5, &left6,
		&target1, &target5, &target6, &target2, &target7, &trigger, &return_target, &final_target,
		&a1p, &a1v, &a1a, &a2p, &a2v, &a2a,
		&a5p, &a5v, &a5a, &a6p, &a6v, &a6a, &a7p, &a7v, &a7a,
		&fn1, &ft1, &fn2, &ft2, &c1, &c2, &c3, &c4
	};
	static_assert(std::size(symbols) == std::size(lengths) && std::size(symbols) == std::size(outputs));
	if (!comm_.ADSReadSum(symbols, lengths, outputs, static_cast<unsigned long>(std::size(symbols)))) return false;

	frame.mode = static_cast<ProgrammedDeliveryMode>(mode);
	frame.phase = static_cast<ProgrammedDeliveryPhase>(phase);
	frame.cycle_index = cycle_index;
	frame.cycle_total = cycle_total;
	frame.setup_busy = setup_busy;
	frame.setup_done = setup_done;
	frame.selfcheck_done = selfcheck_done;
	frame.selfcheck_busy = selfcheck_busy;
	frame.status_error_id = status_error_id;
	frame.wait_action = wait_action;
	frame.error_source = error_source;
	frame.error_axis = error_axis;
	frame.error_phase = error_phase;
	frame.error_target_abs_mm = error_target_abs;
	if (error_axis == 1) frame.error_target_from_left_mm = error_target_abs - left1;
	else if (error_axis == 5) frame.error_target_from_left_mm = error_target_abs - left5;
	else if (error_axis == 6) frame.error_target_from_left_mm = error_target_abs - left6;
	else frame.error_target_from_left_mm = 0.0;
	frame.leftlimit_axis1_abs_mm = left1;
	frame.leftlimit_axis5_abs_mm = left5;
	frame.leftlimit_axis6_abs_mm = left6;
	frame.target_axis1_abs_mm = target1;
	frame.target_axis5_abs_mm = target5;
	frame.target_axis6_abs_mm = target6;
	frame.target_axis2_deg = target2;
	frame.target_axis7_deg = target7;
	frame.trigger_target_abs_mm = trigger;
	frame.return_target_abs_mm = return_target;
	frame.final_target_abs_mm = final_target;
	frame.axis1_pos = a1p; frame.axis1_vel = a1v; frame.axis1_acc = a1a;
	frame.axis2_pos = a2p; frame.axis2_vel = a2v; frame.axis2_acc = a2a;
	frame.axis5_pos = a5p; frame.axis5_vel = a5v; frame.axis5_acc = a5a;
	frame.axis6_pos = a6p; frame.axis6_vel = a6v; frame.axis6_acc = a6a;
	frame.axis7_pos = a7p; frame.axis7_vel = a7v; frame.axis7_acc = a7a;
	frame.fn1 = fn1; frame.ft1 = ft1; frame.fn2 = fn2; frame.ft2 = ft2;
	frame.cylinder1 = c1; frame.cylinder2 = c2; frame.cylinder3 = c3; frame.cylinder4 = c4;
	frame.valid = true;
	return true;
}

bool ProgrammedDeliveryAds::write_config(const ProgrammedDeliveryConfig& config, bool setup_request)
{
	const std::uint8_t mode = static_cast<std::uint8_t>(config.mode);
	const bool setup = setup_request;
	const char* symbols[] = {
		"G.program_test_mode", "G.program_test_axis1_prepare_from_left_mm", "G.program_test_axis1_trigger_from_left_mm",
		"G.program_test_axis5_from_left_mm", "G.program_test_axis2_angle_deg",
		"G.program_test_axis7_angle_deg", "G.program_test_cycle_count", "G.program_test_final_forward_distance_mm",
		"G.program_test_release_wait_ms", "G.program_test_reclamp_wait_ms",
		"G.program_test_forward_velocity", "G.program_test_forward_acceleration", "G.program_test_forward_deceleration",
		"G.program_test_forward_jerk", "G.program_test_return_velocity", "G.program_test_return_acceleration",
		"G.program_test_return_deceleration", "G.program_test_return_jerk", "G.program_test_setup_req"
	};
	const unsigned long lengths[] = {
		sizeof(mode), sizeof(config.axis1_prepare_from_left_mm), sizeof(config.axis1_trigger_from_left_mm),
		sizeof(config.axis5_from_left_mm), sizeof(config.axis2_angle_deg), sizeof(config.axis7_angle_deg),
		sizeof(config.cycle_count), sizeof(config.final_forward_distance_mm), sizeof(config.release_wait_ms), sizeof(config.reclamp_wait_ms), sizeof(config.forward_velocity_mm_s),
		sizeof(config.forward_acceleration_mm_s2), sizeof(config.forward_deceleration_mm_s2), sizeof(config.forward_jerk_mm_s3),
		sizeof(config.return_velocity_mm_s), sizeof(config.return_acceleration_mm_s2), sizeof(config.return_deceleration_mm_s2),
		sizeof(config.return_jerk_mm_s3), sizeof(setup)
	};
	const void* inputs[] = {
		&mode, &config.axis1_prepare_from_left_mm, &config.axis1_trigger_from_left_mm,
		&config.axis5_from_left_mm, &config.axis2_angle_deg, &config.axis7_angle_deg,
		&config.cycle_count, &config.final_forward_distance_mm, &config.release_wait_ms, &config.reclamp_wait_ms, &config.forward_velocity_mm_s,
		&config.forward_acceleration_mm_s2, &config.forward_deceleration_mm_s2, &config.forward_jerk_mm_s3,
		&config.return_velocity_mm_s, &config.return_acceleration_mm_s2, &config.return_deceleration_mm_s2,
		&config.return_jerk_mm_s3, &setup
	};
	static_assert(std::size(symbols) == std::size(lengths) && std::size(symbols) == std::size(inputs));
	return comm_.ADSWriteSum(symbols, lengths, inputs, static_cast<unsigned long>(std::size(symbols)));
}

bool ProgrammedDeliveryAds::request_start()
{
	const bool request = true;
	return comm_.ADSWrite("G.program_test_start_req", sizeof(request), const_cast<bool*>(&request));
}

bool ProgrammedDeliveryAds::request_abort()
{
	const bool request = true;
	return comm_.ADSWrite("G.program_test_abort_req", sizeof(request), const_cast<bool*>(&request));
}

bool ProgrammedDeliveryAds::clear_sample_buffer()
{
	const bool clear = true;
	return comm_.ADSWrite("G.program_test_sample_clear", sizeof(clear), const_cast<bool*>(&clear));
}

bool ProgrammedDeliveryAds::read_sample_count(std::uint32_t& count, bool& overflow)
{
	if (!comm_.ADSRead("G.program_test_sample_count", sizeof(count), &count)) return false;
	if (!comm_.ADSRead("G.program_test_sample_overflow", sizeof(overflow), &overflow)) return false;
	count = (std::min)(count, kSampleCapacity);
	return true;
}

bool ProgrammedDeliveryAds::read_all_samples(ProgrammedDeliveryMode mode, std::uint32_t count,
	std::vector<ProgrammedDeliverySample>& samples)
{
	count = (std::min)(count, kSampleCapacity);
	samples.assign(count, ProgrammedDeliverySample{});
	for (std::uint32_t offset = 0; offset < count; offset += kSampleChunkSize)
	{
		const std::uint32_t chunk = (std::min)(kSampleChunkSize, count - offset);
		std::vector<std::uint32_t> index(chunk), event(chunk);
		std::vector<std::uint64_t> time(chunk);
		std::vector<std::uint8_t> phase(chunk);
		std::vector<std::uint16_t> cycle(chunk), c1(chunk), c2(chunk), c3(chunk), c4(chunk);
		std::vector<double> a1p(chunk), a1v(chunk), a1a(chunk), a2p(chunk), a2v(chunk), a2a(chunk);
		std::vector<double> a5p(chunk), a5v(chunk), a5a(chunk), a6p(chunk), a6v(chunk), a6a(chunk);
		std::vector<double> a7p(chunk), a7v(chunk), a7a(chunk);
		std::vector<short> fn1(chunk), ft1(chunk), fn2(chunk), ft2(chunk);

		if (!read_chunk(comm_, "G.program_test_sample_index", offset, chunk, sizeof(index[0]), index.data()) ||
			!read_chunk(comm_, "G.program_test_sample_time_us", offset, chunk, sizeof(time[0]), time.data()) ||
			!read_chunk(comm_, "G.program_test_sample_phase", offset, chunk, sizeof(phase[0]), phase.data()) ||
			!read_chunk(comm_, "G.program_test_sample_event_sequence", offset, chunk, sizeof(event[0]), event.data()) ||
			!read_chunk(comm_, "G.program_test_sample_cycle_index", offset, chunk, sizeof(cycle[0]), cycle.data())) return false;

		if (mode == ProgrammedDeliveryMode::Catheter)
		{
			if (!read_chunk(comm_, "G.program_test_sample_axis1_pos", offset, chunk, sizeof(double), a1p.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis1_vel", offset, chunk, sizeof(double), a1v.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis1_acc", offset, chunk, sizeof(double), a1a.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis2_pos", offset, chunk, sizeof(double), a2p.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis2_vel", offset, chunk, sizeof(double), a2v.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis2_acc", offset, chunk, sizeof(double), a2a.data()) ||
				!read_chunk(comm_, "G.program_test_sample_cylinder1", offset, chunk, sizeof(c1[0]), c1.data()) ||
				!read_chunk(comm_, "G.program_test_sample_cylinder2", offset, chunk, sizeof(c2[0]), c2.data()) ||
				!read_chunk(comm_, "G.program_test_sample_fn1", offset, chunk, sizeof(fn1[0]), fn1.data()) ||
				!read_chunk(comm_, "G.program_test_sample_ft1", offset, chunk, sizeof(ft1[0]), ft1.data())) return false;
		}
		else
		{
			if (!read_chunk(comm_, "G.program_test_sample_axis5_pos", offset, chunk, sizeof(double), a5p.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis5_vel", offset, chunk, sizeof(double), a5v.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis5_acc", offset, chunk, sizeof(double), a5a.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis6_pos", offset, chunk, sizeof(double), a6p.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis6_vel", offset, chunk, sizeof(double), a6v.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis6_acc", offset, chunk, sizeof(double), a6a.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis7_pos", offset, chunk, sizeof(double), a7p.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis7_vel", offset, chunk, sizeof(double), a7v.data()) ||
				!read_chunk(comm_, "G.program_test_sample_axis7_acc", offset, chunk, sizeof(double), a7a.data()) ||
				!read_chunk(comm_, "G.program_test_sample_cylinder3", offset, chunk, sizeof(c3[0]), c3.data()) ||
				!read_chunk(comm_, "G.program_test_sample_cylinder4", offset, chunk, sizeof(c4[0]), c4.data()) ||
				!read_chunk(comm_, "G.program_test_sample_fn2", offset, chunk, sizeof(fn2[0]), fn2.data()) ||
				!read_chunk(comm_, "G.program_test_sample_ft2", offset, chunk, sizeof(ft2[0]), ft2.data())) return false;
		}

		for (std::uint32_t i = 0; i < chunk; ++i)
		{
			ProgrammedDeliverySample& s = samples[offset + i];
			s.sample_index = index[i]; s.plc_time_us = time[i]; s.phase = phase[i];
			s.event_sequence = event[i]; s.cycle_index = cycle[i];
			s.axis1_pos = a1p[i]; s.axis1_vel = a1v[i]; s.axis1_acc = a1a[i];
			s.axis2_pos = a2p[i]; s.axis2_vel = a2v[i]; s.axis2_acc = a2a[i];
			s.axis5_pos = a5p[i]; s.axis5_vel = a5v[i]; s.axis5_acc = a5a[i];
			s.axis6_pos = a6p[i]; s.axis6_vel = a6v[i]; s.axis6_acc = a6a[i];
			s.axis7_pos = a7p[i]; s.axis7_vel = a7v[i]; s.axis7_acc = a7a[i];
			s.cylinder1 = c1[i]; s.cylinder2 = c2[i]; s.cylinder3 = c3[i]; s.cylinder4 = c4[i];
			s.fn1 = fn1[i]; s.ft1 = ft1[i]; s.fn2 = fn2[i]; s.ft2 = ft2[i];
		}
	}
	return true;
}
