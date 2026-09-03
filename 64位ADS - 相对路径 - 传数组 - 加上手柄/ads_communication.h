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
	double axis1_act_velocity_mm_s = 0.0;
	short ft_1_value = 0;
	short fn_1_value = 0;
	short fn_2_value = 0;
	short ft_2_value = 0;
	bool estop_hold_req = false;
	bool host_comm_timeout = false;
	// 位置/速度和 PLC 时序可用于运动控制时为 true；不受单独力数据质量影响。
	bool position_valid = false;
	// 本次 Sum Read 通信成功且四路力值均为有限数时为 true；PLC 周期跨度仅用于诊断。
	bool force_valid = false;
	// 完整快照标记，等价于 position_valid && force_valid；不作为运动控制唯一门控。
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
	bool inject_push_req[2] = {};
	bool inject_pull_req[2] = {};
	bool startup_smoothing_bypass = false;
	// 仅供C++内部统一交接：bit0清axis1 Req，bit1清axis6 Req。
	std::uint8_t planned_return_clear_mask = 0;
};

enum class AdsPlannedReturnOperation : unsigned char
{
	Prepare = 0,
	Commit = 1,
	Clear = 2
};

struct AdsPlannedReturnLegCommand
{
	// 沿用控制代码的零基轴号：0 表示 axis1，5 表示 axis6。
	int axis_index = -1;
	double target_abs = 0.0;
	double velocity = 0.0;
	double acc = 0.0;
	double dec = 0.0;
	double jerk = 0.0;
};

struct AdsPlannedReturnCommand
{
	AdsPlannedReturnOperation operation = AdsPlannedReturnOperation::Clear;
	int leg_count = 0;
	AdsPlannedReturnLegCommand legs[2] = {};
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
	bool axis1_return_busy = false;
	bool axis1_return_done = false;
	bool axis1_return_error = false;
	std::uint32_t axis1_return_error_id = 0;
	std::uint64_t axis1_return_event_sequence = 0;
	// 锁存最近一次接收确认边沿，避免Busy短脉冲在两次主循环读取之间丢失。
	std::uint64_t axis1_return_busy_true_sequence = 0;
	std::uint64_t axis1_return_done_false_sequence = 0;
	// 完成和故障必须使用各自的新上升沿，不能由ErrorId等无关通知代替。
	std::uint64_t axis1_return_done_true_sequence = 0;
	std::uint64_t axis1_return_error_true_sequence = 0;
	// ErrorId 可能晚于Error到达并很快被Req清零，额外锁存最近一次非零值。
	std::uint32_t axis1_return_last_nonzero_error_id = 0;
	std::uint64_t axis1_return_last_nonzero_error_id_sequence = 0;
	std::int64_t axis1_return_event_qpc_ticks = 0;
	bool axis6_return_busy = false;
	bool axis6_return_done = false;
	bool axis6_return_error = false;
	std::uint32_t axis6_return_error_id = 0;
	std::uint64_t axis6_return_event_sequence = 0;
	// 与轴1相同，锁存接收确认、完成和故障边沿，避免短脉冲被主循环漏掉。
	std::uint64_t axis6_return_busy_true_sequence = 0;
	std::uint64_t axis6_return_done_false_sequence = 0;
	std::uint64_t axis6_return_done_true_sequence = 0;
	std::uint64_t axis6_return_error_true_sequence = 0;
	// ErrorId 可能晚于Error到达，保留本轮最近一次非零错误码及其事件序号。
	std::uint32_t axis6_return_last_nonzero_error_id = 0;
	std::uint64_t axis6_return_last_nonzero_error_id_sequence = 0;
	std::int64_t axis6_return_event_qpc_ticks = 0;
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
	std::uint64_t publish_output(const AdsOutputCommand& command);
	std::uint64_t applied_output_generation() const;
	// 仅在包含refer的motion_enabled输出成功写入后推进，用于确认实际位置保持语义。
	std::uint64_t applied_motion_output_generation() const;
	// 计划回退命令只复制进固定容量队列；0 表示参数非法、服务未运行或队列已满。
	std::uint64_t submit_planned_return_command(const AdsPlannedReturnCommand& command);
	// 返回 false 表示序号无效或结果历史已被覆盖；返回 true 时其余输出参数有效。
	// possibly_started_mask仅用于失败/未知Commit的保守判定：bit0=axis1，bit1=axis6。
	bool planned_return_command_result(
		std::uint64_t sequence,
		bool& completed,
		bool& success,
		std::uint8_t& possibly_started_mask) const;
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
	bool write_output_cycle(bool& planned_return_processed);
	void fail_queued_planned_return_commands();
	void complete_planned_return_command(
		std::uint64_t sequence,
		bool success,
		std::uint8_t possibly_started_mask = 0);
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

	static constexpr std::size_t kPlannedReturnQueueCapacity = 32;
	static constexpr std::size_t kPlannedReturnResultCapacity = 128;
	static constexpr std::size_t kPlannedReturnFieldsPerAxis = 6;

	struct PlannedReturnQueueSlot
	{
		AdsPlannedReturnCommand command{};
		std::uint64_t sequence = 0;
	};

	enum class PlannedReturnResultState : unsigned char
	{
		Unknown = 0,
		Pending = 1,
		Succeeded = 2,
		Failed = 3
	};

	struct PlannedReturnResultSlot
	{
		std::atomic<std::uint64_t> sequence{ 0 };
		std::atomic<unsigned char> possibly_started_mask{ 0 };
		std::atomic<unsigned char> state{
			static_cast<unsigned char>(PlannedReturnResultState::Unknown) };
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
	std::uint64_t next_output_generation_ = 0;
	std::uint64_t desired_output_generation_ = 0;
	std::atomic<std::uint64_t> applied_output_generation_{ 0 };
	std::atomic<std::uint64_t> applied_motion_output_generation_{ 0 };
	std::atomic<bool> watchdog_recovery_pending_{ false };

	mutable std::mutex event_mutex_;
	AdsEventState event_state_{};
	std::uint32_t notification_update_mask_ = 0;
	std::uint64_t axis1_return_event_sequence_counter_ = 0;
	std::uint64_t axis6_return_event_sequence_counter_ = 0;

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
	std::array<unsigned long, 17> fast_direct_read_handles_{};
	std::array<unsigned long, 11> fast_fallback_read_handles_{};
	std::array<unsigned long, 16> fast_write_handles_{};
	// 每行依次为 Req、TargetAbs、Velocity、Acc、Dec、Jerk；第0/1行为axis1/axis6。
	std::array<std::array<unsigned long, kPlannedReturnFieldsPerAxis>, 2>
		planned_return_write_handles_{};
	bool fast_handles_valid_ = false;
	std::vector<NotificationRegistration> notification_registrations_;

	// 单生产者（主控制线程）/单消费者（100 Hz ADS线程）的固定容量环形队列。
	std::array<PlannedReturnQueueSlot, kPlannedReturnQueueCapacity>
		planned_return_queue_{};
	std::array<PlannedReturnResultSlot, kPlannedReturnResultCapacity>
		planned_return_results_{};
	std::atomic<std::uint64_t> planned_return_write_index_{ 0 };
	std::atomic<std::uint64_t> planned_return_read_index_{ 0 };
	std::atomic<std::uint64_t> next_planned_return_sequence_{ 0 };
	// Clear提交后，尚未开始且触及同轴的更早Prepare/Commit会在消费者侧失败出队。
	std::array<std::atomic<std::uint64_t>, 2>
		planned_return_clear_barrier_sequence_{};

	std::mutex low_frequency_mutex_;
	std::deque<std::shared_ptr<LowFrequencyRequest>> low_frequency_requests_;
};
