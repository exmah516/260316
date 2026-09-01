#include "ExperimentStreamAds.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <thread>

namespace
{
	constexpr std::uint16_t kBlockCapacity = 512;
	constexpr std::uint16_t kZeroCapacity = 1000;

	template <typename T>
	bool read_array(CADSComm& comm, const std::string& symbol, std::uint16_t count, std::vector<T>& values)
	{
		values.resize(count);
		if (count == 0) return true;
		return comm.ADSReadSymbolOffset(symbol.c_str(), 0,
			static_cast<unsigned long>(sizeof(T) * count), values.data());
	}

	std::string block_symbol(int slot, const char* field)
	{
		return "G.experiment_record_block" + std::to_string(slot) + "_" + field;
	}
}

ExperimentStreamAds::ExperimentStreamAds() = default;

ExperimentStreamAds::~ExperimentStreamAds()
{
	close();
}

bool ExperimentStreamAds::open()
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

void ExperimentStreamAds::close()
{
	if (comm_.IsCommOpen()) comm_.CloseComm();
}

bool ExperimentStreamAds::is_open() const
{
	return comm_.IsCommOpen();
}

std::string ExperimentStreamAds::last_error() const
{
	return comm_.GetLastErrorCopy();
}

bool ExperimentStreamAds::reset_recording()
{
	const bool value = true;
	return comm_.ADSWrite("G.experiment_record_clear", sizeof(value), const_cast<bool*>(&value));
}

bool ExperimentStreamAds::stop_recording()
{
	const bool value = false;
	return comm_.ADSWrite("G.experiment_record_enable", sizeof(value), const_cast<bool*>(&value));
}

bool ExperimentStreamAds::request_zero()
{
	const bool value = true;
	return comm_.ADSWrite("G.experiment_zero_req", sizeof(value), const_cast<bool*>(&value));
}

bool ExperimentStreamAds::invalidate_zero()
{
	const bool value = false;
	return comm_.ADSWrite("G.experiment_zero_done", sizeof(value), const_cast<bool*>(&value));
}

bool ExperimentStreamAds::read_status(ExperimentStreamStatus& status)
{
	bool ready1 = false, ready2 = false, recording = false, overflow = false;
	std::uint32_t seq1 = 0, seq2 = 0, total = 0, error_id = 0, zero_error = 0;
	std::uint8_t source_mode = 0;
	std::uint16_t count1 = 0, count2 = 0, zero_count = 0;
	bool zero_busy = false, zero_done = false;
	double z[4] = {}, sd[4] = {};
	const char* symbols[] = {
		"G.experiment_record_enable", "G.experiment_record_overflow", "G.experiment_record_total_count", "G.experiment_record_error_id",
		"G.experiment_record_block_ready[1]", "G.experiment_record_block_ready[2]",
		"G.experiment_record_block_sequence[1]", "G.experiment_record_block_sequence[2]",
		"G.experiment_record_block_count[1]", "G.experiment_record_block_count[2]",
		"G.experiment_record_source_mode",
		"G.experiment_zero_busy", "G.experiment_zero_done", "G.experiment_zero_error_id", "G.experiment_zero_sample_count",
		"G.experiment_zero_fn1", "G.experiment_zero_ft1", "G.experiment_zero_fn2", "G.experiment_zero_ft2",
		"G.experiment_zero_fn1_std", "G.experiment_zero_ft1_std", "G.experiment_zero_fn2_std", "G.experiment_zero_ft2_std"
	};
	const unsigned long lengths[] = {
		sizeof(recording), sizeof(overflow), sizeof(total), sizeof(error_id), sizeof(ready1), sizeof(ready2),
		sizeof(seq1), sizeof(seq2), sizeof(count1), sizeof(count2), sizeof(source_mode), sizeof(zero_busy), sizeof(zero_done), sizeof(zero_error), sizeof(zero_count),
		sizeof(z[0]), sizeof(z[1]), sizeof(z[2]), sizeof(z[3]), sizeof(sd[0]), sizeof(sd[1]), sizeof(sd[2]), sizeof(sd[3])
	};
	void* outputs[] = {
		&recording, &overflow, &total, &error_id, &ready1, &ready2, &seq1, &seq2, &count1, &count2,
		&source_mode, &zero_busy, &zero_done, &zero_error, &zero_count, &z[0], &z[1], &z[2], &z[3], &sd[0], &sd[1], &sd[2], &sd[3]
	};
	static_assert(std::size(symbols) == std::size(lengths) && std::size(symbols) == std::size(outputs));
	if (!comm_.ADSReadSum(symbols, lengths, outputs, static_cast<unsigned long>(std::size(symbols)))) return false;
	status.recording = recording;
	status.overflow = overflow;
	status.total_count = total;
	status.error_id = error_id;
	status.source_mode = source_mode;
	status.block_ready = { ready1, ready2 };
	status.block_sequence = { seq1, seq2 };
	status.block_count = { count1, count2 };
	status.zero.busy = zero_busy;
	status.zero.done = zero_done;
	status.zero.error_id = zero_error;
	status.zero.valid = zero_done && zero_error == 0;
	status.zero.sample_count = zero_count;
	for (int i = 0; i < 4; ++i) { status.zero.value[i] = z[i]; status.zero.standard_deviation[i] = sd[i]; }
	return true;
}

bool ExperimentStreamAds::read_block(int slot, std::vector<ExperimentStreamSample>& samples, std::uint32_t& sequence)
{
	if (slot < 0 || slot > 1) return false;
	const int plc_slot = slot + 1;
	bool ready = false;
	std::uint16_t count = 0;
	std::uint32_t initial_sequence = 0;
	if (!comm_.ADSRead(("G.experiment_record_block_ready[" + std::to_string(plc_slot) + "]").c_str(), sizeof(ready), &ready)) return false;
	if (!ready) { samples.clear(); return true; }
	if (!comm_.ADSRead(("G.experiment_record_block_sequence[" + std::to_string(plc_slot) + "]").c_str(), sizeof(initial_sequence), &initial_sequence) ||
		!comm_.ADSRead(("G.experiment_record_block_count[" + std::to_string(plc_slot) + "]").c_str(), sizeof(count), &count)) return false;
	if (count == 0 || count > kBlockCapacity) return false;

	std::vector<std::uint32_t> index, event;
	std::vector<std::uint64_t> time;
	std::vector<std::uint8_t> phase;
	std::vector<std::uint16_t> cycle, c1, c2, c3, c4;
	std::vector<double> a1p, a1v, a1a, a2p, a2v, a2a, a5p, a5v, a5a, a6p, a6v, a6a, a7p, a7v, a7a;
	std::vector<short> fn1, ft1, fn2, ft2;
	const int block = slot;
	index.resize(kBlockCapacity); event.resize(kBlockCapacity); time.resize(kBlockCapacity); phase.resize(kBlockCapacity); cycle.resize(kBlockCapacity);
	a1p.resize(kBlockCapacity); a1v.resize(kBlockCapacity); a1a.resize(kBlockCapacity);
	a2p.resize(kBlockCapacity); a2v.resize(kBlockCapacity); a2a.resize(kBlockCapacity);
	a5p.resize(kBlockCapacity); a5v.resize(kBlockCapacity); a5a.resize(kBlockCapacity);
	a6p.resize(kBlockCapacity); a6v.resize(kBlockCapacity); a6a.resize(kBlockCapacity);
	a7p.resize(kBlockCapacity); a7v.resize(kBlockCapacity); a7a.resize(kBlockCapacity);
	c1.resize(kBlockCapacity); c2.resize(kBlockCapacity); c3.resize(kBlockCapacity); c4.resize(kBlockCapacity);
	fn1.resize(kBlockCapacity); ft1.resize(kBlockCapacity); fn2.resize(kBlockCapacity); ft2.resize(kBlockCapacity);
	const std::array<std::string, 28> names = {
		block_symbol(block, "index"), block_symbol(block, "time_us"), block_symbol(block, "phase"), block_symbol(block, "event_sequence"), block_symbol(block, "cycle_index"),
		block_symbol(block, "axis1_pos"), block_symbol(block, "axis1_vel"), block_symbol(block, "axis1_acc"),
		block_symbol(block, "axis2_pos"), block_symbol(block, "axis2_vel"), block_symbol(block, "axis2_acc"),
		block_symbol(block, "axis5_pos"), block_symbol(block, "axis5_vel"), block_symbol(block, "axis5_acc"),
		block_symbol(block, "axis6_pos"), block_symbol(block, "axis6_vel"), block_symbol(block, "axis6_acc"),
		block_symbol(block, "axis7_pos"), block_symbol(block, "axis7_vel"), block_symbol(block, "axis7_acc"),
		block_symbol(block, "cylinder1"), block_symbol(block, "cylinder2"), block_symbol(block, "cylinder3"), block_symbol(block, "cylinder4"),
		block_symbol(block, "fn1"), block_symbol(block, "ft1"), block_symbol(block, "fn2"), block_symbol(block, "ft2")
	};
	std::array<const char*, 28> symbols{};
	std::array<unsigned long, 28> lengths{};
	std::array<void*, 28> outputs{};
	for (std::size_t i = 0; i < names.size(); ++i) symbols[i] = names[i].c_str();
	const unsigned long bytes = static_cast<unsigned long>(kBlockCapacity);
	lengths = {
		bytes * sizeof(index[0]), bytes * sizeof(time[0]), bytes * sizeof(phase[0]), bytes * sizeof(event[0]), bytes * sizeof(cycle[0]),
		bytes * sizeof(a1p[0]), bytes * sizeof(a1v[0]), bytes * sizeof(a1a[0]), bytes * sizeof(a2p[0]), bytes * sizeof(a2v[0]), bytes * sizeof(a2a[0]),
		bytes * sizeof(a5p[0]), bytes * sizeof(a5v[0]), bytes * sizeof(a5a[0]), bytes * sizeof(a6p[0]), bytes * sizeof(a6v[0]), bytes * sizeof(a6a[0]),
		bytes * sizeof(a7p[0]), bytes * sizeof(a7v[0]), bytes * sizeof(a7a[0]), bytes * sizeof(c1[0]), bytes * sizeof(c2[0]), bytes * sizeof(c3[0]), bytes * sizeof(c4[0]),
		bytes * sizeof(fn1[0]), bytes * sizeof(ft1[0]), bytes * sizeof(fn2[0]), bytes * sizeof(ft2[0])
	};
	outputs = {
		index.data(), time.data(), phase.data(), event.data(), cycle.data(),
		a1p.data(), a1v.data(), a1a.data(), a2p.data(), a2v.data(), a2a.data(), a5p.data(), a5v.data(), a5a.data(),
		a6p.data(), a6v.data(), a6a.data(), a7p.data(), a7v.data(), a7a.data(), c1.data(), c2.data(), c3.data(), c4.data(),
		fn1.data(), ft1.data(), fn2.data(), ft2.data()
	};
	if (!comm_.ADSReadSum(symbols.data(), lengths.data(), outputs.data(), static_cast<unsigned long>(symbols.size()))) return false;

	std::uint32_t final_sequence = 0;
	bool final_ready = false;
	if (!comm_.ADSRead(("G.experiment_record_block_sequence[" + std::to_string(plc_slot) + "]").c_str(), sizeof(final_sequence), &final_sequence) ||
		!comm_.ADSRead(("G.experiment_record_block_ready[" + std::to_string(plc_slot) + "]").c_str(), sizeof(final_ready), &final_ready)) return false;
	if (!final_ready || final_sequence != initial_sequence) return false;

	samples.resize(count);
	for (std::uint16_t i = 0; i < count; ++i)
	{
		auto& s = samples[i];
		s.index = index[i]; s.time_us = time[i]; s.phase = phase[i]; s.event_sequence = event[i]; s.cycle_index = cycle[i];
		s.axis1_pos = a1p[i]; s.axis1_vel = a1v[i]; s.axis1_acc = a1a[i]; s.axis2_pos = a2p[i]; s.axis2_vel = a2v[i]; s.axis2_acc = a2a[i];
		s.axis5_pos = a5p[i]; s.axis5_vel = a5v[i]; s.axis5_acc = a5a[i]; s.axis6_pos = a6p[i]; s.axis6_vel = a6v[i]; s.axis6_acc = a6a[i];
		s.axis7_pos = a7p[i]; s.axis7_vel = a7v[i]; s.axis7_acc = a7a[i];
		s.cylinder1 = c1[i]; s.cylinder2 = c2[i]; s.cylinder3 = c3[i]; s.cylinder4 = c4[i];
		s.fn1 = fn1[i]; s.ft1 = ft1[i]; s.fn2 = fn2[i]; s.ft2 = ft2[i];
	}
	sequence = initial_sequence;
	return true;
}

bool ExperimentStreamAds::acknowledge_block(int slot, std::uint32_t sequence)
{
	if (slot < 0 || slot > 1) return false;
	const std::string symbol = "G.experiment_record_block_ack_sequence[" + std::to_string(slot + 1) + "]";
	return comm_.ADSWrite(symbol.c_str(), sizeof(sequence), &sequence);
}

bool ExperimentStreamAds::read_zero_samples(std::vector<std::array<double, 4>>& samples)
{
	std::vector<short> fn1, ft1, fn2, ft2;
	if (!read_array(comm_, "G.experiment_zero_sample_fn1", kZeroCapacity, fn1) ||
		!read_array(comm_, "G.experiment_zero_sample_ft1", kZeroCapacity, ft1) ||
		!read_array(comm_, "G.experiment_zero_sample_fn2", kZeroCapacity, fn2) ||
		!read_array(comm_, "G.experiment_zero_sample_ft2", kZeroCapacity, ft2)) return false;
	samples.resize(kZeroCapacity);
	for (std::uint16_t i = 0; i < kZeroCapacity; ++i) samples[i] = { static_cast<double>(fn1[i]), static_cast<double>(ft1[i]), static_cast<double>(fn2[i]), static_cast<double>(ft2[i]) };
	return true;
}
