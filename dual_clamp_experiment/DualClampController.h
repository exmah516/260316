#pragma once

#include "DualClampAds.h"
#include "ExperimentStreamAds.h"
#include "ExperimentStreamRecorder.h"

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class DualClampController
{
public:
	DualClampController();

	bool prepare(const DualClampConfig& config);
	bool start(const DualClampConfig& config);
	void abort(const std::string& reason);
	void tick(double dt_s);
	bool save_samples(const std::string& directory, std::string& error);
	bool request_zero();
	void invalidate_zero();
	bool set_record_suffix(const std::string& suffix);
	ForceZeroState zero_state() const;
	DualClampPhase phase() const;
	std::string last_error() const;
	std::string recording_directory() const;
	bool recording_archived() const;
	DualClampConfig config() const;
	DualClampLiveFrame live() const;
	std::uint32_t event_sequence() const;

	bool open_ads();
	void close_ads();
	bool is_ads_open() const;

	DualClampAds& ads() { return ads_; }

private:
	bool write_metadata(const std::string& directory, std::string& error) const;
	bool write_csv(const std::string& directory, const std::vector<DualClampSample>& samples, std::string& error) const;
	bool write_events_csv(const std::string& directory, const std::vector<DualClampSample>& samples, std::string& error) const;

	mutable std::mutex mutex_;
	DualClampAds ads_;
	DualClampConfig config_{};
	DualClampLiveFrame live_{};
	DualClampPhase phase_ = DualClampPhase::Idle;
	std::string last_error_;
	std::string abort_reason_;
	std::uint32_t event_sequence_ = 0;
	bool started_ = false;
	bool selfcheck_requested_ = false;
	ExperimentStreamAds stream_ads_;
	ExperimentStreamRecorder recorder_;
	ExperimentStreamStatus stream_status_{};
	std::uint32_t expected_block_sequence_ = 0;
	std::uint32_t expected_sample_index_ = 0;
	bool zero_file_written_ = false;
	void poll_stream_locked();
};
