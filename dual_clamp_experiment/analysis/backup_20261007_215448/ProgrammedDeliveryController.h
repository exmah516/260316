#pragma once

#include "ProgrammedDeliveryAds.h"
#include "ExperimentStreamAds.h"
#include "ExperimentStreamRecorder.h"
#include "ClampCurveBuffer.h"
#include "ClampDynamics.h"
#include "ExternalValidation.h"
#include "Handle582Feedback.h"

#include <mutex>
#include <future>
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
	std::string curve_response(std::uint64_t after, std::uint64_t generation) const;
	std::string external_curve_response(std::uint64_t after, std::uint64_t generation) const;

private:
	bool open_ads_locked();
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
	struct PendingStreamBlock
	{
		bool ok = false;
		int slot = -1;
		std::uint32_t sequence = 0;
		std::vector<ExperimentStreamSample> samples;
		std::string error;
	};
	std::future<PendingStreamBlock> stream_block_future_;
	bool stream_block_pending_ = false;
	ExperimentStreamRecorder recorder_;
	ExperimentStreamStatus stream_status_{};
	std::uint32_t expected_block_sequence_ = 0;
	std::uint32_t expected_sample_index_ = 0;
	bool zero_file_written_ = false;
	handle582::Handle582Feedback handle_582_{582};
	bool handle_anchor_valid_ = false;
	std::array<double, 2> handle_anchor_{};
	std::array<double, 7> handle_reference_{};
	std::array<double, 7> handle_axis_anchor_{};
	std::array<double, 7> handle_init_pos_{};
	std::uint16_t handle_cycle_ = 0;
	bool handle_phase_active_ = false;
	void poll_stream_locked();
	void reset_model_locked(const char* reason = "restart");
	clampdynamics::Predictor predictor_;
	clampdynamics::OperationGate illustration_gate_;
	clampdynamics::Result last_prediction_;
	std::array<clampdynamics::Config, 5> mode_dynamics_{};
	clampillustration::Generator illustration_;
	clampmodel::CurveBuffer curves_;
	externalvalidation::CurveBuffer external_curves_;
	double model_compute_us_ = 0.0;
	double model_block_span_ms_ = 0.0;
	bool model_zero_valid_ = false;
	forcepulse::Guard pulse_guard_;
	double pulse_compute_us_ = 0.0;
	ProgrammedDeliveryLiveFrame position_reference_{};
};
