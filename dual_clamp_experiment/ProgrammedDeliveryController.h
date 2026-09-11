#pragma once

#include "ProgrammedDeliveryAds.h"
#include "ExperimentStreamAds.h"
#include "ExperimentStreamRecorder.h"
#include "ClampCurveBuffer.h"
#include "ClampDynamics.h"
#include "ExternalValidation.h"

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
	void set_shared_selfcheck_state(bool done, bool busy);
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
	bool shared_selfcheck_done_ = false;
	bool shared_selfcheck_busy_ = false;
	bool shared_selfcheck_valid_ = false;
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
