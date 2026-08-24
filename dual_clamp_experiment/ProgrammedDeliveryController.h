#pragma once

#include "ProgrammedDeliveryAds.h"
#include "ExperimentStreamAds.h"
#include "ExperimentStreamRecorder.h"

#include <mutex>
#include <string>
#include <vector>

class ProgrammedDeliveryController
{
public:
	ProgrammedDeliveryController();

	bool open_ads();
	void close_ads();
	bool is_ads_open() const;
	bool select_mode(ProgrammedDeliveryMode mode);
	bool prepare(const ProgrammedDeliveryConfig& config);
	bool start();
	void abort();
	void tick();
	bool save_samples(const std::string& directory, std::string& error);
	bool request_zero();
	void invalidate_zero();
	bool set_record_suffix(const std::string& suffix);
	ForceZeroState zero_state() const;

	ProgrammedDeliveryConfig config() const;
	ProgrammedDeliveryLiveFrame live() const;
	std::string last_error() const;
	std::string recording_directory() const;
	bool recording_archived() const;

private:
	bool validate_config(const ProgrammedDeliveryConfig& config, std::string& error) const;
	bool write_metadata(const std::string& directory, std::string& error) const;
	bool write_samples_csv(const std::string& directory, const std::vector<ProgrammedDeliverySample>& samples, std::string& error) const;
	bool write_events_csv(const std::string& directory, const std::vector<ProgrammedDeliverySample>& samples, std::string& error) const;

	mutable std::mutex mutex_;
	ProgrammedDeliveryAds ads_;
	ProgrammedDeliveryConfig config_{};
	ProgrammedDeliveryLiveFrame live_{};
	std::string last_error_;
	bool started_ = false;
	ExperimentStreamAds stream_ads_;
	ExperimentStreamRecorder recorder_;
	ExperimentStreamStatus stream_status_{};
	std::uint32_t expected_block_sequence_ = 0;
	std::uint32_t expected_sample_index_ = 0;
	bool zero_file_written_ = false;
	void poll_stream_locked();
};
