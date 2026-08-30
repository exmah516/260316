#pragma once

#include "DualClampTypes.h"
#include "ProgrammedDeliveryTypes.h"
#include "StandaloneRecordTypes.h"

#include <array>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct ForceZeroState
{
	bool busy = false;
	bool done = false;
	bool valid = false;
	std::uint32_t sample_count = 0;
	std::uint32_t error_id = 0;
	std::array<double, 4> value{};
	std::array<double, 4> standard_deviation{};
	double axis2_angle_deg = 0.0;
	double axis7_angle_deg = 0.0;
};

struct ExperimentStreamSample;

class ExperimentStreamRecorder
{
public:
	ExperimentStreamRecorder();
	~ExperimentStreamRecorder();

	bool begin(const std::string& mode, const std::string& suffix, std::string& error);
	bool begin_standalone(const std::string& suffix, std::uint64_t field_mask, std::string& error);
	// 取零先于正式记录时，允许在尚未写入样本的情况下更新独立记录字段表头。
	bool reconfigure_standalone(std::uint64_t field_mask, std::string& error);
	bool append_dual(const std::vector<DualClampSample>& samples, std::size_t begin_index,
		const ForceZeroState& zero, std::string& error);
	bool append_program(const std::vector<ProgrammedDeliverySample>& samples, std::size_t begin_index,
		ProgrammedDeliveryMode mode, const ForceZeroState& zero, std::string& error);
	bool append_standalone(const std::vector<ExperimentStreamSample>& samples,
		const ForceZeroState& zero, std::uint64_t field_mask, std::string& error);
	bool append_zero(const std::array<double, 4>& raw, std::uint64_t time_us,
		double axis2_angle_deg, double axis7_angle_deg, std::string& error);
	bool begin_zero(std::string& error);
	bool reset_zero_file(std::string& error);
	bool finish_zero(const ForceZeroState& zero, std::string& error);
	bool finalize(const std::string& status, const std::string& reason, const ForceZeroState& zero, std::string& error);

	bool active() const { return active_; }
	bool archived() const { return archived_; }
	bool failed() const;
	std::uint64_t sample_count() const { return sample_count_; }
	const std::string& directory() const { return directory_; }
	std::string last_error() const;

private:
	static std::string sanitize_suffix(const std::string& value);
	static std::string json_escape(const std::string& value);
	static std::string phase_name(std::uint8_t phase);
	bool write_dual_header(std::string& error);
	bool write_program_header(ProgrammedDeliveryMode mode, std::string& error);
	bool write_standalone_header(std::uint64_t field_mask, std::string& error);
	bool write_zero_header(std::string& error);
	bool write_json(const std::string& status, const std::string& reason, const ForceZeroState& zero, std::string& error);
	bool write_event(const std::string& event, std::uint64_t time_us, std::uint32_t cycle, std::uint8_t phase, std::uint32_t sequence, std::string& error);
	bool enqueue_write(int kind, std::string data, std::string& error);
	void writer_loop();
	bool wait_for_queue(std::string& error);
	void stop_writer();

	struct PendingWrite
	{
		int kind = 0; // 0=样本，1=事件，2=取零数据
		std::string data;
	};

	bool active_ = false;
	bool archived_ = false;
	bool program_mode_ = false;
	bool standalone_mode_ = false;
	std::uint64_t standalone_field_mask_ = 0;
	ProgrammedDeliveryMode mode_ = ProgrammedDeliveryMode::Catheter;
	std::string directory_;
	std::string mode_name_;
	std::string last_error_;
	std::uint64_t sample_count_ = 0;
	std::uint64_t zero_sample_count_ = 0;
	std::uint32_t last_event_sequence_ = static_cast<std::uint32_t>(-1);
	std::uint8_t last_phase_ = 0;
	std::string start_time_local_;
	std::string end_time_local_;
	std::string zero_start_time_local_;
	std::string zero_end_time_local_;
	std::uint64_t first_sample_time_us_ = 0;
	std::uint64_t last_sample_time_us_ = 0;
	std::uint64_t block_count_ = 0;
	bool has_sample_time_ = false;
	std::ofstream samples_;
	std::ofstream events_;
	std::ofstream zero_file_;
	std::thread writer_thread_;
	mutable std::mutex writer_mutex_;
	std::condition_variable writer_cv_;
	std::deque<PendingWrite> writer_queue_;
	bool writer_stop_requested_ = false;
	bool writer_busy_ = false;
	bool writer_finished_ = false;
	std::string writer_error_;
};
