#pragma once

#include <ADSComm1.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

enum class AdsConnectionState : int
{
	Disconnected = 0,
	Connecting = 1,
	Running = 2,
	SoftHold = 3,
	Reconnecting = 4,
	PlcRestarted = 5,
	Error = 6
};

struct AdsFastSnapshot
{
	std::uint64_t attempt_sequence = 0;
	std::int64_t qpc_ticks = 0;
	std::int64_t plc_dc_task_time = 0;
	std::uint32_t plc_cycle_begin = 0;
	std::uint32_t plc_cycle_end = 0;
	std::uint32_t plc_cycle_span = 0;
	std::uint64_t rtt_us = 0;
	double act_pos_rel[7] = {};
	double act_pos_from_left[7] = {};
	short ft_1_value = 0;
	short fn_1_value = 0;
	short fn_2_value = 0;
	short ft_2_value = 0;
	bool estop_hold_req = false;
	bool host_comm_timeout = false;
	bool position_valid = false;
	bool force_valid = false;
	bool valid = false;
};

struct AdsOutputCommand
{
	bool motion_enabled = false;
	double refer[7] = {};
	bool axis1_fast_return = false;
	bool axis6_fast_retract = false;
	unsigned short cylinder[4] = {};
	bool cylinder_valid = false;
	bool cylinder5_press_req = false;
	bool axis4_forward_req = false;
	bool axis4_reverse_req = false;
	bool startup_smoothing_bypass = false;
};

struct AdsEventState
{
	bool self_check_done = false;
	bool handle_reinit_req = false;
	bool handle_reinit_done = false;
	bool estop_hold_req = false;
	bool host_comm_timeout = false;
	bool startup_loading_ready = false;
	bool axis4_manual_busy = false;
	bool axis4_manual_done = false;
	bool axis4_manual_error = false;
	std::uint32_t axis4_manual_error_id = 0;
	int gen_state = 0;
};

struct AdsCommunicationStats
{
	AdsConnectionState state = AdsConnectionState::Disconnected;
	double actual_hz = 0.0;
	std::uint64_t latest_snapshot_age_us = 0;
	std::uint64_t latest_rtt_us = 0;
	std::uint64_t failed_cycles = 0;
	std::uint64_t consecutive_failures = 0;
	std::uint64_t max_consecutive_failures = 0;
	std::uint64_t reconnect_count = 0;
	std::uint64_t plc_restart_count = 0;
	std::uint64_t missed_deadlines = 0;
	std::uint64_t snapshot_queue_dropped = 0;
};

class AdsCommunicationService
{
public:
	explicit AdsCommunicationService(CADSComm& ads);
	~AdsCommunicationService();

	bool start(const double* initial_init_pos, const double* initial_leftlimit);
	void stop();
	bool wait_for_snapshot(std::uint64_t after_sequence, DWORD timeout_ms, AdsFastSnapshot& snapshot);
	bool latest_snapshot(AdsFastSnapshot& snapshot) const;
	void drain_snapshots(std::vector<AdsFastSnapshot>& snapshots);
	void publish_output(const AdsOutputCommand& command);
	void request_coordinate_refresh();
	bool refresh_coordinates(DWORD timeout_ms = 250);
	void request_watchdog_recovery();
	bool read(const char* symbol, unsigned long length, void* output, DWORD timeout_ms = 250);
	bool write(const char* symbol, unsigned long length, const void* input, DWORD timeout_ms = 250);
	bool read_sum(
		const char* const* symbols,
		const unsigned long* lengths,
		void* const* outputs,
		unsigned long count,
		DWORD timeout_ms = 250);
	bool write_sum(
		const char* const* symbols,
		const unsigned long* lengths,
		const void* const* inputs,
		unsigned long count,
		DWORD timeout_ms = 250);
	AdsEventState event_state() const;
	AdsCommunicationStats stats() const;
	bool coordinate_cache(double* init_pos, double* leftlimit) const;
	std::uint32_t host_session_id() const { return host_session_id_; }

	// ADS DLL 回调线程只调用此入口，入口内部不发起任何 ADS 请求。
	void on_notification(std::uint32_t event_id, const void* data, std::uint32_t size);

private:
	void run();
	bool ensure_connection(bool reconnecting);
	bool initialize_connection();
	bool resolve_fast_handles();
	void clear_fast_handles();
	bool refresh_coordinate_cache_now();
	bool read_fast_snapshot(AdsFastSnapshot& snapshot, bool handles_fresh);
	bool write_output_cycle();
	bool register_notifications();
	void unregister_notifications();
	void clear_runtime_connection_state();
	void mark_plc_restart(bool reconnect_required);
	void process_one_low_frequency_request();
	void fail_queued_low_frequency_requests();
	void publish_snapshot(const AdsFastSnapshot& snapshot);
	void set_connection_state(AdsConnectionState state);
	void update_rate(std::int64_t now_qpc, std::uint64_t rtt_us, bool success);
	void wait_until(std::int64_t qpc_deadline);

	struct NotificationRegistration
	{
		std::uint32_t registration_id = 0;
		unsigned long notification_handle = 0;
	};

	enum class LowFrequencyOperation
	{
		Read,
		Write,
		ReadSum,
		WriteSum
	};

	struct LowFrequencyRequest
	{
		LowFrequencyOperation operation = LowFrequencyOperation::Read;
		std::vector<std::string> symbols;
		std::vector<unsigned long> lengths;
		std::vector<std::vector<unsigned char>> buffers;
		std::mutex mutex;
		std::condition_variable cv;
		bool started = false;
		bool completed = false;
		bool cancelled = false;
		bool success = false;
	};

	bool submit_low_frequency_request(
		const std::shared_ptr<LowFrequencyRequest>& request,
		DWORD timeout_ms);

	CADSComm& ads_;
	mutable std::mutex lifecycle_mutex_;
	std::atomic<bool> running_{ false };
	HANDLE stop_event_ = nullptr;
	std::thread worker_;
	std::int64_t qpc_frequency_ = 1;
	std::uint32_t host_session_id_ = 1;
	std::uint32_t heartbeat_sequence_ = 0;
	std::uint64_t attempt_sequence_ = 0;

	mutable std::mutex snapshot_mutex_;
	std::condition_variable snapshot_cv_;
	AdsFastSnapshot latest_snapshot_{};
	bool has_snapshot_ = false;
	std::deque<AdsFastSnapshot> snapshot_queue_;

	mutable std::mutex output_mutex_;
	AdsOutputCommand desired_output_{};
	AdsOutputCommand last_sent_output_{};
	bool has_desired_output_ = false;
	bool has_last_sent_output_ = false;
	std::atomic<bool> watchdog_recovery_pending_{ false };

	mutable std::mutex event_mutex_;
	AdsEventState event_state_{};
	std::uint32_t notification_update_mask_ = 0;

	mutable std::mutex stats_mutex_;
	AdsCommunicationStats stats_{};
	std::atomic<std::int64_t> latest_valid_qpc_{ 0 };
	std::int64_t rate_window_qpc_ = 0;
	std::uint64_t rate_window_successes_ = 0;
	std::int64_t last_full_success_qpc_ = 0;
	std::int64_t last_plc_dc_time_ = 0;
	std::uint32_t last_plc_cycle_ = 0;
	std::uint32_t stalled_plc_snapshot_count_ = 0;
	std::string last_plc_app_name_;
	bool plc_restart_active_ = false;
	bool restart_reconnect_pending_ = false;

	mutable std::mutex coordinate_mutex_;
	double init_pos_[7] = {};
	double leftlimit_[7] = {};
	bool coordinate_cache_valid_ = false;
	std::atomic<bool> coordinate_refresh_pending_{ false };
	bool use_direct_nc_position_ = true;
	std::array<unsigned long, 16> fast_direct_read_handles_{};
	std::array<unsigned long, 10> fast_fallback_read_handles_{};
	std::array<unsigned long, 14> fast_write_handles_{};
	bool fast_handles_valid_ = false;
	std::vector<NotificationRegistration> notification_registrations_;

	std::mutex low_frequency_mutex_;
	std::deque<std::shared_ptr<LowFrequencyRequest>> low_frequency_requests_;
};
