#pragma once

#include "action4_camera.h"
#include "ads_communication.h"
#include "async_csv_writer.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

enum class ExperimentRecordingState : int
{
	Idle = 0,
	Starting = 1,
	Recording = 2,
	Stopping = 3,
	Error = 4
};

enum class ExperimentRecordingError : int
{
	None = 0,
	InvalidName = 1,
	CreateDirectoryFailed = 2,
	OpenForceCsvFailed = 3,
	OpenMotionCsvFailed = 4,
	ForceWriterFailed = 5,
	MotionWriterFailed = 6,
	StopThreadFailed = 7,
	TransitionWriterFailed = 8,
	VideoFrameWriterFailed = 9
};

struct ExperimentStartInfo
{
	std::uint32_t axis1_handle_serial = 0;
	std::uint32_t axis6_handle_serial = 0;
	bool single_handle_mode = false;
};

class SessionClock
{
public:
	SessionClock();
	void reset();
	std::int64_t now_qpc() const;
	std::uint64_t elapsed_us(std::int64_t qpc_ticks) const;
	std::int64_t anchor_qpc() const { return anchor_qpc_; }
	std::int64_t frequency() const { return frequency_; }
	std::uint64_t utc_start_filetime_100ns() const { return utc_start_filetime_100ns_; }

private:
	std::int64_t frequency_ = 1;
	std::int64_t anchor_qpc_ = 0;
	std::uint64_t utc_start_filetime_100ns_ = 0;
};

struct ForceCsvRow
{
	std::uint64_t sample_index = 0;
	std::uint64_t elapsed_us = 0;
	// 以下两项仅用于会话边界与 ADS 快照去重，不写入 CSV。
	std::uint64_t ads_snapshot_sequence = 0;
	std::int64_t source_qpc_ticks = 0;
	bool sample_valid = false;
	bool calibrated_valid = false;
	double fn_1_raw_v = 0.0;
	double ft_1_raw_v = 0.0;
	double fn_1_zero_v = 0.0;
	double ft_1_zero_v = 0.0;
	double clean_force_n = 0.0;
	double clean_handle_torque_nm = 0.0;
};

struct MotionCsvRow
{
	std::uint64_t sample_index = 0;
	std::uint64_t elapsed_us = 0;
	// 以下两项仅用于会话边界与 ADS 快照去重，不写入 CSV。
	std::uint64_t ads_snapshot_sequence = 0;
	std::int64_t source_qpc_ticks = 0;
	bool position_valid = false;
	double axis_from_left_mm[7] = {};
	bool axis1_handle_valid = false;
	double axis1_handle_linear_raw = 0.0;
	double axis1_handle_linear_filtered = 0.0;
	double axis1_handle_rotation_raw = 0.0;
	double axis1_handle_rotation_filtered = 0.0;
	bool axis6_handle_valid = false;
	double axis6_handle_linear_raw = 0.0;
	double axis6_handle_linear_filtered = 0.0;
	double axis6_handle_rotation_raw = 0.0;
	double axis6_handle_rotation_filtered = 0.0;
};

struct ForceTransitionCsvRow
{
	std::uint64_t elapsed_us = 0;
	// 力过渡表按实际力快照去重，避免主循环重复写入同一帧。
	std::uint64_t force_snapshot_sequence = 0;
	std::int64_t source_qpc_ticks = 0;
	bool valid = false;
	int trial_id = 0;
	int velocity_level = 0;
	int repeat_in_level = 0;
	int phase_code = 0;
	std::uint64_t phase_elapsed_ms = 0;
	double v_ratio = 0.0;
	double axis1_from_left_mm = 0.0;
	double clean_force_n = 0.0;
	double clean_handle_torque_nm = 0.0;
};

struct ExperimentRecorderSnapshot
{
	ExperimentRecordingState state = ExperimentRecordingState::Idle;
	ExperimentRecordingError error = ExperimentRecordingError::None;
	std::uint64_t elapsed_us = 0;
	std::uint64_t force_dropped = 0;
	std::uint64_t motion_dropped = 0;
	// 保留 wire 字段：force_missed 是会话内 ADS 100 Hz 超期数；motion 无独立调度器，固定为 0。
	std::uint64_t force_missed = 0;
	std::uint64_t motion_missed = 0;
	std::uint64_t ads_pre_session_rejected = 0;
	std::uint64_t ads_duplicate_rejected = 0;
	Action4CameraSnapshot camera;
};

class ExperimentRecorder
{
public:
	ExperimentRecorder();
	~ExperimentRecorder();

	bool start(const std::string& experiment_name_utf8, const ExperimentStartInfo& info);
	void stop_async(const char* reason);
	void stop_and_wait(const char* reason);
	void poll_health();

	bool enqueue_force(const ForceCsvRow& row);
	bool enqueue_motion(const MotionCsvRow& row);

	bool start_force_transition_log();
	void stop_force_transition_log();
	bool enqueue_force_transition(const ForceTransitionCsvRow& row);
	bool force_transition_log_running() const { return transition_writer_.is_running(); }
	void update_ads_communication_stats(const AdsCommunicationStats& stats);

	void set_camera_preview(bool enabled);
	Action4CameraSnapshot camera_snapshot() const;
	ExperimentRecorderSnapshot snapshot() const;
	bool is_recording() const { return state_.load(std::memory_order_acquire) == ExperimentRecordingState::Recording; }
	const SessionClock& clock() const { return clock_; }

private:
	static bool write_force_row(std::FILE* fp, const ForceCsvRow& row);
	static bool write_motion_row(std::FILE* fp, const MotionCsvRow& row);
	static bool write_transition_row(std::FILE* fp, const ForceTransitionCsvRow& row);
	static void normalize_force_row(ForceCsvRow& row);
	static void normalize_motion_row(MotionCsvRow& row);
	static void normalize_transition_row(ForceTransitionCsvRow& row);
	static std::wstring sanitize_experiment_name(const std::wstring& raw_name);
	static std::wstring utf8_to_wide(const std::string& text);
	static std::string wide_to_utf8(const std::wstring& text);

	bool create_session_paths(const std::wstring& cleaned_name);
	void rollback_failed_start();
	void stop_worker(std::string reason);
	void write_session_json(bool final, const std::string& stop_reason);
	bool accept_ads_snapshot(
		std::uint64_t sequence,
		std::int64_t qpc_ticks,
		std::uint64_t expected_sequence_step,
		std::uint64_t& first_sequence,
		std::uint64_t& last_sequence,
		std::atomic<std::uint64_t>& accepted,
		std::atomic<std::uint64_t>& invalid,
		bool row_valid,
		std::atomic<std::uint64_t>& pre_session_rejected,
		std::atomic<std::uint64_t>& duplicate_rejected,
		std::atomic<std::uint64_t>& sequence_skipped);
	static std::uint64_t counter_delta(std::uint64_t current, std::uint64_t baseline);

	std::atomic<ExperimentRecordingState> state_{ ExperimentRecordingState::Idle };
	std::atomic<ExperimentRecordingError> error_{ ExperimentRecordingError::None };
	SessionClock clock_;
	ExperimentStartInfo start_info_;
	std::string raw_name_utf8_;
	std::wstring cleaned_name_;
	std::wstring session_directory_;
	std::wstring force_path_;
	std::wstring motion_path_;
	std::wstring video_path_;
	std::wstring video_frames_path_;
	std::wstring metadata_path_;
	std::uint64_t force_sample_index_ = 0;
	std::uint64_t motion_sample_index_ = 0;
	std::atomic<std::uint64_t> stop_elapsed_us_{ 0 };
	std::uint64_t force_first_ads_sequence_ = 0;
	std::uint64_t force_last_ads_sequence_ = 0;
	std::uint64_t motion_first_ads_sequence_ = 0;
	std::uint64_t motion_last_ads_sequence_ = 0;
	std::uint64_t transition_first_force_sequence_ = 0;
	std::uint64_t transition_last_force_sequence_ = 0;
	std::atomic<std::uint64_t> force_ads_accepted_{ 0 };
	std::atomic<std::uint64_t> force_ads_invalid_{ 0 };
	std::atomic<std::uint64_t> force_ads_pre_session_rejected_{ 0 };
	std::atomic<std::uint64_t> force_ads_duplicate_rejected_{ 0 };
	std::atomic<std::uint64_t> force_ads_sequence_skipped_{ 0 };
	std::atomic<std::uint64_t> motion_ads_accepted_{ 0 };
	std::atomic<std::uint64_t> motion_ads_invalid_{ 0 };
	std::atomic<std::uint64_t> motion_ads_pre_session_rejected_{ 0 };
	std::atomic<std::uint64_t> motion_ads_duplicate_rejected_{ 0 };
	std::atomic<std::uint64_t> motion_ads_sequence_skipped_{ 0 };
	std::atomic<std::uint64_t> transition_ads_accepted_{ 0 };
	std::atomic<std::uint64_t> transition_ads_invalid_{ 0 };
	std::atomic<std::uint64_t> transition_ads_pre_session_rejected_{ 0 };
	std::atomic<std::uint64_t> transition_ads_duplicate_rejected_{ 0 };
	std::atomic<std::uint64_t> transition_ads_sequence_skipped_{ 0 };
	std::uint32_t transition_file_index_ = 0;
	std::uint64_t transition_dropped_completed_ = 0;
	bool transition_current_pending_ = false;
	std::atomic<bool> transition_writer_used_{ false };
	mutable std::mutex ads_stats_mutex_;
	AdsCommunicationStats ads_stats_baseline_{};
	AdsCommunicationStats ads_stats_latest_{};
	AdsCommunicationStats ads_stats_session_end_{};
	bool ads_stats_baseline_available_ = false;
	bool ads_stats_latest_available_ = false;
	bool ads_stats_session_end_available_ = false;
	std::uint64_t ads_previous_failed_cycles_ = 0;
	std::uint64_t ads_previous_consecutive_failures_ = 0;
	std::uint64_t ads_pre_session_failure_streak_ = 0;
	std::uint64_t ads_session_max_consecutive_failures_ = 0;
	bool ads_failure_observation_available_ = false;

	AsyncCsvWriter<ForceCsvRow> force_writer_;
	AsyncCsvWriter<MotionCsvRow> motion_writer_;
	AsyncCsvWriter<ForceTransitionCsvRow, 4096> transition_writer_;
	Action4CameraRecorder camera_;
	std::thread stop_thread_;
};
