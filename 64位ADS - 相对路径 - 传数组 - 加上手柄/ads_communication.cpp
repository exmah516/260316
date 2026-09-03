#include "ads_communication.h"

#include "plc_io.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <unordered_map>

namespace
{
	constexpr std::uint32_t kCommunicationRateHz = 100;
	constexpr std::uint32_t kHardTimeoutMs = 300;
	constexpr std::uint32_t kDeviceStateCheckPeriodCycles = 10;
	constexpr std::uint32_t kMaxStalledPlcSnapshots = 3;
	constexpr std::size_t kSnapshotQueueCapacity = 2048;
	constexpr std::uint64_t kReconnectIntervalMs = 1000;
	constexpr std::uint32_t kMaxRegistrationIdAttempts = 65536;

	constexpr std::uint32_t kNotifySelfCheckDone = 1;
	constexpr std::uint32_t kNotifyHandleReinitReq = 2;
	constexpr std::uint32_t kNotifyHandleReinitDone = 3;
	constexpr std::uint32_t kNotifyEstopHold = 4;
	constexpr std::uint32_t kNotifyHostTimeout = 5;
	constexpr std::uint32_t kNotifyStartupReady = 6;
	constexpr std::uint32_t kNotifyAxis4Busy = 7;
	constexpr std::uint32_t kNotifyAxis4Done = 8;
	constexpr std::uint32_t kNotifyAxis4Error = 9;
	constexpr std::uint32_t kNotifyAxis4ErrorId = 10;
	constexpr std::uint32_t kNotifyGenState = 11;
	constexpr std::uint32_t kNotifyAxis1ReturnBusy = 12;
	constexpr std::uint32_t kNotifyAxis1ReturnDone = 13;
	constexpr std::uint32_t kNotifyAxis1ReturnError = 14;
	constexpr std::uint32_t kNotifyAxis1ReturnErrorId = 15;
	constexpr std::uint32_t kNotifyAxis6ReturnBusy = 16;
	constexpr std::uint32_t kNotifyAxis6ReturnDone = 17;
	constexpr std::uint32_t kNotifyAxis6ReturnError = 18;
	constexpr std::uint32_t kNotifyAxis6ReturnErrorId = 19;

	enum FastWriteHandleIndex : std::size_t
	{
		kWriteHostSession = 0,
		kWriteHeartbeat,
		kWriteRefer,
		kWriteAxis1FastReturn,
		kWriteAxis6FastRetract,
		kWriteCylinder1,
		kWriteCylinder2,
		kWriteCylinder3,
		kWriteCylinder4,
		kWriteCylinder5Press,
		kWriteAxis4Forward,
		kWriteAxis4Reverse,
		kWriteInjectorPush,
		kWriteInjectorPull,
		kWriteSmoothingBypass,
		kWriteHostRecover,
		kFastWriteHandleCount
	};

	struct NotificationTarget
	{
		AdsCommunicationService* service;
		std::uint32_t event_id;
	};

	std::mutex g_notification_registry_mutex;
	std::unordered_map<std::uint32_t, NotificationTarget> g_notification_registry;
	std::atomic<std::uint32_t> g_next_notification_registration_id{ 1 };

	bool reserve_notification_registration(
		AdsCommunicationService* service,
		std::uint32_t event_id,
		std::uint32_t& registration_id)
	{
		std::lock_guard<std::mutex> lock(g_notification_registry_mutex);
		for (std::uint32_t attempt = 0; attempt < kMaxRegistrationIdAttempts; ++attempt)
		{
			const std::uint32_t candidate =
				g_next_notification_registration_id.fetch_add(1, std::memory_order_relaxed);
			if (candidate == 0 || g_notification_registry.find(candidate) != g_notification_registry.end())
			{
				continue;
			}
			try
			{
				g_notification_registry.emplace(candidate, NotificationTarget{ service, event_id });
			}
			catch (const std::bad_alloc&)
			{
				return false;
			}
			registration_id = candidate;
			return true;
		}
		return false;
	}

	void __stdcall ads_notification_callback(
		AmsAddr*,
		AdsNotificationHeader* notification,
		unsigned long registration_id)
	{
		if (notification == nullptr) return;

		// 固定锁顺序为“注册表 -> 事件状态”。注销会先取得注册表锁，
		// 因而返回后不会再有回调持有已销毁的服务指针。
		std::lock_guard<std::mutex> lock(g_notification_registry_mutex);
		const auto it = g_notification_registry.find(static_cast<std::uint32_t>(registration_id));
		if (it == g_notification_registry.end() || it->second.service == nullptr) return;
		it->second.service->on_notification(
			it->second.event_id,
			notification->data,
			static_cast<std::uint32_t>(notification->cbSampleSize));
	}

	template <typename T>
	bool notification_value(const void* data, std::uint32_t size, T& value)
	{
		if (data == nullptr || size != sizeof(T)) return false;
		std::memcpy(&value, data, sizeof(T));
		return true;
	}

	bool same_discrete_output(const AdsOutputCommand& lhs, const AdsOutputCommand& rhs)
	{
		return std::equal(lhs.cylinder, lhs.cylinder + 4, rhs.cylinder) &&
			lhs.cylinder_valid == rhs.cylinder_valid &&
			lhs.cylinder5_press_req == rhs.cylinder5_press_req &&
			lhs.axis4_forward_req == rhs.axis4_forward_req &&
			lhs.axis4_reverse_req == rhs.axis4_reverse_req &&
			std::equal(lhs.inject_push_req, lhs.inject_push_req + 2, rhs.inject_push_req) &&
			std::equal(lhs.inject_pull_req, lhs.inject_pull_req + 2, rhs.inject_pull_req) &&
			lhs.startup_smoothing_bypass == rhs.startup_smoothing_bypass;
	}

	int planned_return_axis_slot(int axis_index)
	{
		if (axis_index == 0) return 0;
		if (axis_index == 5) return 1;
		return -1;
	}
}

AdsCommunicationService::AdsCommunicationService(CADSComm& ads)
	: ads_(ads)
{
	LARGE_INTEGER frequency{};
	QueryPerformanceFrequency(&frequency);
	qpc_frequency_ = frequency.QuadPart > 0 ? frequency.QuadPart : 1;
	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);
	host_session_id_ = static_cast<std::uint32_t>(
		(now.QuadPart ^ (static_cast<std::uint64_t>(GetCurrentProcessId()) << 16)) & 0xFFFFFFFFu);
	if (host_session_id_ == 0) host_session_id_ = 1;
	stop_event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
}

AdsCommunicationService::~AdsCommunicationService()
{
	stop();
	if (stop_event_ != nullptr)
	{
		CloseHandle(stop_event_);
		stop_event_ = nullptr;
	}
}

bool AdsCommunicationService::start(const double* initial_init_pos, const double* initial_leftlimit)
{
	std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
	if (running_.exchange(true, std::memory_order_acq_rel)) return true;
	if (stop_event_ == nullptr)
	{
		running_.store(false, std::memory_order_release);
		return false;
	}
	ResetEvent(stop_event_);
	plc_restart_active_ = false;
	restart_reconnect_pending_ = false;
	last_full_success_qpc_ = 0;
	latest_valid_qpc_.store(0, std::memory_order_release);
	{
		std::lock_guard<std::mutex> lock(snapshot_mutex_);
		latest_snapshot_ = AdsFastSnapshot{};
		has_snapshot_ = false;
		snapshot_queue_.clear();
	}
	{
		std::lock_guard<std::mutex> lock(output_mutex_);
		desired_output_ = AdsOutputCommand{};
		has_desired_output_ = false;
		has_last_sent_output_ = false;
		desired_output_generation_ = 0;
	}
	if (initial_init_pos != nullptr && initial_leftlimit != nullptr)
	{
		std::lock_guard<std::mutex> lock(coordinate_mutex_);
		std::copy(initial_init_pos, initial_init_pos + 7, init_pos_);
		std::copy(initial_leftlimit, initial_leftlimit + 7, leftlimit_);
		coordinate_cache_valid_ = true;
	}
	else
	{
		std::lock_guard<std::mutex> lock(coordinate_mutex_);
		coordinate_cache_valid_ = false;
	}
	try
	{
		worker_ = std::thread(&AdsCommunicationService::run, this);
	}
	catch (...)
	{
		running_.store(false, std::memory_order_release);
		return false;
	}
	return true;
}

void AdsCommunicationService::stop()
{
	std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
	const bool was_running = running_.exchange(false, std::memory_order_acq_rel);
	fail_queued_low_frequency_requests();
	if (was_running && stop_event_ != nullptr) SetEvent(stop_event_);
	if (was_running) snapshot_cv_.notify_all();
	if (worker_.joinable()) worker_.join();
	// worker退出后再由当前线程排空，避免两个消费者同时推进固定环形队列。
	fail_queued_planned_return_commands();
	unregister_notifications();
}

bool AdsCommunicationService::wait_for_snapshot(
	std::uint64_t after_sequence,
	DWORD timeout_ms,
	AdsFastSnapshot& snapshot)
{
	std::unique_lock<std::mutex> lock(snapshot_mutex_);
	snapshot_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&]()
	{
		return !running_.load(std::memory_order_acquire) ||
			(has_snapshot_ && latest_snapshot_.attempt_sequence > after_sequence);
	});
	if (!has_snapshot_ || latest_snapshot_.attempt_sequence <= after_sequence) return false;
	snapshot = latest_snapshot_;
	return true;
}

bool AdsCommunicationService::latest_snapshot(AdsFastSnapshot& snapshot) const
{
	std::lock_guard<std::mutex> lock(snapshot_mutex_);
	if (!has_snapshot_) return false;
	snapshot = latest_snapshot_;
	return true;
}

void AdsCommunicationService::drain_snapshots(std::vector<AdsFastSnapshot>& snapshots)
{
	std::lock_guard<std::mutex> lock(snapshot_mutex_);
	snapshots.clear();
	snapshots.reserve(snapshot_queue_.size());
	while (!snapshot_queue_.empty())
	{
		snapshots.push_back(snapshot_queue_.front());
		snapshot_queue_.pop_front();
	}
}

std::uint64_t AdsCommunicationService::publish_output(const AdsOutputCommand& command)
{
	std::lock_guard<std::mutex> lock(output_mutex_);
	++next_output_generation_;
	// 0 保留为“尚未发布”，实际运行中几乎不可能回卷；即使回卷也跳过 0。
	if (next_output_generation_ == 0) ++next_output_generation_;
	desired_output_ = command;
	desired_output_generation_ = next_output_generation_;
	has_desired_output_ = true;
	return desired_output_generation_;
}

std::uint64_t AdsCommunicationService::applied_output_generation() const
{
	return applied_output_generation_.load(std::memory_order_acquire);
}

std::uint64_t AdsCommunicationService::applied_motion_output_generation() const
{
	return applied_motion_output_generation_.load(std::memory_order_acquire);
}

std::uint64_t AdsCommunicationService::submit_planned_return_command(
	const AdsPlannedReturnCommand& command)
{
	if (!running_.load(std::memory_order_acquire) ||
		command.leg_count < 1 || command.leg_count > 2)
	{
		return 0;
	}
	switch (command.operation)
	{
	case AdsPlannedReturnOperation::Prepare:
	case AdsPlannedReturnOperation::Commit:
	case AdsPlannedReturnOperation::Clear:
		break;
	default:
		return 0;
	}

	bool axis1_seen = false;
	bool axis6_seen = false;
	for (int i = 0; i < command.leg_count; ++i)
	{
		const AdsPlannedReturnLegCommand& leg = command.legs[i];
		const int axis_slot = planned_return_axis_slot(leg.axis_index);
		if (axis_slot < 0) return 0;
		bool& seen = axis_slot == 0 ? axis1_seen : axis6_seen;
		if (seen) return 0;
		seen = true;
		if (command.operation == AdsPlannedReturnOperation::Prepare &&
			(!std::isfinite(leg.target_abs) || !std::isfinite(leg.velocity) ||
				!std::isfinite(leg.acc) || !std::isfinite(leg.dec) ||
				!std::isfinite(leg.jerk)))
		{
			return 0;
		}
	}

	const std::uint64_t write_index =
		planned_return_write_index_.load(std::memory_order_relaxed);
	const std::uint64_t read_index =
		planned_return_read_index_.load(std::memory_order_acquire);
	if (write_index - read_index >= kPlannedReturnQueueCapacity) return 0;

	std::uint64_t sequence =
		next_planned_return_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
	// 0 永远保留为提交失败。实际运行无法触及64位回卷，仍显式跳过该值。
	if (sequence == 0)
	{
		sequence = next_planned_return_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
	}

	PlannedReturnResultSlot& result =
		planned_return_results_[sequence % kPlannedReturnResultCapacity];
	result.state.store(
		static_cast<unsigned char>(PlannedReturnResultState::Unknown),
		std::memory_order_relaxed);
	result.possibly_started_mask.store(0, std::memory_order_relaxed);
	result.sequence.store(sequence, std::memory_order_release);
	result.state.store(
		static_cast<unsigned char>(PlannedReturnResultState::Pending),
		std::memory_order_release);
	if (command.operation == AdsPlannedReturnOperation::Clear)
	{
		for (int leg_index = 0; leg_index < command.leg_count; ++leg_index)
		{
			const int axis_slot = planned_return_axis_slot(command.legs[leg_index].axis_index);
			auto& barrier = planned_return_clear_barrier_sequence_[
				static_cast<std::size_t>(axis_slot)];
			std::uint64_t previous = barrier.load(std::memory_order_relaxed);
			while (previous < sequence && !barrier.compare_exchange_weak(
				previous,
				sequence,
				std::memory_order_release,
				std::memory_order_relaxed))
			{
			}
		}
	}

	PlannedReturnQueueSlot& slot =
		planned_return_queue_[write_index % kPlannedReturnQueueCapacity];
	slot.command = command;
	slot.sequence = sequence;
	planned_return_write_index_.store(write_index + 1, std::memory_order_release);
	return sequence;
}

bool AdsCommunicationService::planned_return_command_result(
	std::uint64_t sequence,
	bool& completed,
	bool& success,
	std::uint8_t& possibly_started_mask) const
{
	completed = false;
	success = false;
	possibly_started_mask = 0;
	if (sequence == 0) return false;

	const PlannedReturnResultSlot& result =
		planned_return_results_[sequence % kPlannedReturnResultCapacity];
	if (result.sequence.load(std::memory_order_acquire) != sequence) return false;
	const auto state = static_cast<PlannedReturnResultState>(
		result.state.load(std::memory_order_acquire));
	possibly_started_mask = static_cast<std::uint8_t>(
		result.possibly_started_mask.load(std::memory_order_acquire));
	// 防止结果槽恰在查询期间被较新的序号复用。
	if (result.sequence.load(std::memory_order_acquire) != sequence) return false;
	if (state == PlannedReturnResultState::Unknown) return false;
	if (state == PlannedReturnResultState::Pending) return true;
	completed = true;
	success = state == PlannedReturnResultState::Succeeded;
	return true;
}

void AdsCommunicationService::request_coordinate_refresh()
{
	coordinate_refresh_pending_.store(true, std::memory_order_release);
}

bool AdsCommunicationService::refresh_coordinates(DWORD timeout_ms)
{
	double init_pos[7] = {};
	double leftlimit[7] = {};
	const char* symbols[] = { AdsSymbol::init_pos, AdsSymbol::leftlimit };
	const unsigned long lengths[] = { sizeof(init_pos), sizeof(leftlimit) };
	void* outputs[] = { init_pos, leftlimit };
	if (!read_sum(symbols, lengths, outputs, 2, timeout_ms)) return false;
	std::lock_guard<std::mutex> lock(coordinate_mutex_);
	std::copy(init_pos, init_pos + 7, init_pos_);
	std::copy(leftlimit, leftlimit + 7, leftlimit_);
	coordinate_cache_valid_ = true;
	return true;
}

void AdsCommunicationService::request_watchdog_recovery()
{
	watchdog_recovery_pending_.store(true, std::memory_order_release);
}

bool AdsCommunicationService::read(
	const char* symbol,
	unsigned long length,
	void* output,
	DWORD timeout_ms)
{
	if (symbol == nullptr || symbol[0] == '\0' || length == 0 || output == nullptr) return false;
	try
	{
		auto request = std::make_shared<LowFrequencyRequest>();
		request->operation = LowFrequencyOperation::Read;
		request->symbols.emplace_back(symbol);
		request->lengths.push_back(length);
		request->buffers.emplace_back(length);
		if (!submit_low_frequency_request(request, timeout_ms)) return false;
		std::memcpy(output, request->buffers[0].data(), length);
		return true;
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
}

bool AdsCommunicationService::write(
	const char* symbol,
	unsigned long length,
	const void* input,
	DWORD timeout_ms)
{
	if (symbol == nullptr || symbol[0] == '\0' || length == 0 || input == nullptr) return false;
	try
	{
		auto request = std::make_shared<LowFrequencyRequest>();
		request->operation = LowFrequencyOperation::Write;
		request->symbols.emplace_back(symbol);
		request->lengths.push_back(length);
		const auto* bytes = static_cast<const unsigned char*>(input);
		request->buffers.emplace_back(bytes, bytes + length);
		return submit_low_frequency_request(request, timeout_ms);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
}

bool AdsCommunicationService::read_sum(
	const char* const* symbols,
	const unsigned long* lengths,
	void* const* outputs,
	unsigned long count,
	DWORD timeout_ms)
{
	if (symbols == nullptr || lengths == nullptr || outputs == nullptr || count == 0) return false;
	try
	{
		auto request = std::make_shared<LowFrequencyRequest>();
		request->operation = LowFrequencyOperation::ReadSum;
		request->symbols.reserve(count);
		request->lengths.reserve(count);
		request->buffers.reserve(count);
		for (unsigned long i = 0; i < count; ++i)
		{
			if (symbols[i] == nullptr || symbols[i][0] == '\0' || lengths[i] == 0 || outputs[i] == nullptr)
			{
				return false;
			}
			request->symbols.emplace_back(symbols[i]);
			request->lengths.push_back(lengths[i]);
			request->buffers.emplace_back(lengths[i]);
		}
		if (!submit_low_frequency_request(request, timeout_ms)) return false;
		for (unsigned long i = 0; i < count; ++i)
		{
			std::memcpy(outputs[i], request->buffers[i].data(), lengths[i]);
		}
		return true;
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
}

bool AdsCommunicationService::write_sum(
	const char* const* symbols,
	const unsigned long* lengths,
	const void* const* inputs,
	unsigned long count,
	DWORD timeout_ms)
{
	if (symbols == nullptr || lengths == nullptr || inputs == nullptr || count == 0) return false;
	try
	{
		auto request = std::make_shared<LowFrequencyRequest>();
		request->operation = LowFrequencyOperation::WriteSum;
		request->symbols.reserve(count);
		request->lengths.reserve(count);
		request->buffers.reserve(count);
		for (unsigned long i = 0; i < count; ++i)
		{
			if (symbols[i] == nullptr || symbols[i][0] == '\0' || lengths[i] == 0 || inputs[i] == nullptr)
			{
				return false;
			}
			request->symbols.emplace_back(symbols[i]);
			request->lengths.push_back(lengths[i]);
			const auto* bytes = static_cast<const unsigned char*>(inputs[i]);
			request->buffers.emplace_back(bytes, bytes + lengths[i]);
		}
		return submit_low_frequency_request(request, timeout_ms);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
}

bool AdsCommunicationService::submit_low_frequency_request(
	const std::shared_ptr<LowFrequencyRequest>& request,
	DWORD timeout_ms)
{
	if (request == nullptr) return false;
	{
		std::lock_guard<std::mutex> stats_lock(stats_mutex_);
		if (stats_.state != AdsConnectionState::Running) return false;
	}
	try
	{
		std::lock_guard<std::mutex> queue_lock(low_frequency_mutex_);
		if (!running_.load(std::memory_order_acquire)) return false;
		low_frequency_requests_.push_back(request);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}

	std::unique_lock<std::mutex> request_lock(request->mutex);
	const auto started_or_completed = [&]()
	{
		return request->started || request->completed;
	};
	bool signalled = false;
	if (timeout_ms == INFINITE)
	{
		request->cv.wait(request_lock, started_or_completed);
		signalled = true;
	}
	else
	{
		signalled = request->cv.wait_for(
			request_lock,
			std::chrono::milliseconds(timeout_ms),
			started_or_completed);
	}

	if (!signalled && !request->started && !request->completed)
	{
		request_lock.unlock();
		bool cancelled = false;
		{
			std::lock_guard<std::mutex> queue_lock(low_frequency_mutex_);
			std::lock_guard<std::mutex> state_lock(request->mutex);
			if (!request->started && !request->completed)
			{
				const auto it = std::find(
					low_frequency_requests_.begin(), low_frequency_requests_.end(), request);
				if (it != low_frequency_requests_.end())
				{
					low_frequency_requests_.erase(it);
					request->cancelled = true;
					request->completed = true;
					request->success = false;
					cancelled = true;
				}
			}
		}
		if (cancelled)
		{
			request->cv.notify_all();
			return false;
		}
		request_lock.lock();
	}

	// 超时前已经开始的 ADS 请求必须等待完成，避免调用方缓冲区提前失效。
	request->cv.wait(request_lock, [&]() { return request->completed; });
	return request->success;
}

AdsEventState AdsCommunicationService::event_state() const
{
	std::lock_guard<std::mutex> lock(event_mutex_);
	return event_state_;
}

AdsCommunicationStats AdsCommunicationService::stats() const
{
	AdsCommunicationStats result;
	{
		std::lock_guard<std::mutex> lock(stats_mutex_);
		result = stats_;
	}
	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);
	const std::int64_t latest_valid_qpc = latest_valid_qpc_.load(std::memory_order_acquire);
	if (latest_valid_qpc > 0 && now.QuadPart >= latest_valid_qpc)
	{
		result.latest_snapshot_age_us = static_cast<std::uint64_t>(
			(now.QuadPart - latest_valid_qpc) * 1000000LL / qpc_frequency_);
	}
	else
	{
		result.latest_snapshot_age_us = (std::numeric_limits<std::uint64_t>::max)();
	}
	return result;
}

bool AdsCommunicationService::coordinate_cache(double* init_pos, double* leftlimit) const
{
	if (init_pos == nullptr || leftlimit == nullptr) return false;
	std::lock_guard<std::mutex> lock(coordinate_mutex_);
	if (!coordinate_cache_valid_) return false;
	std::copy(init_pos_, init_pos_ + 7, init_pos);
	std::copy(leftlimit_, leftlimit_ + 7, leftlimit);
	return true;
}

void AdsCommunicationService::on_notification(
	std::uint32_t event_id,
	const void* data,
	std::uint32_t size)
{
	std::lock_guard<std::mutex> lock(event_mutex_);
	bool updated = false;
	const bool previous_axis1_return_busy = event_state_.axis1_return_busy;
	const bool previous_axis1_return_done = event_state_.axis1_return_done;
	const bool previous_axis1_return_error = event_state_.axis1_return_error;
	const bool previous_axis6_return_busy = event_state_.axis6_return_busy;
	const bool previous_axis6_return_done = event_state_.axis6_return_done;
	const bool previous_axis6_return_error = event_state_.axis6_return_error;
	switch (event_id)
	{
	case kNotifySelfCheckDone:
		updated = notification_value(data, size, event_state_.self_check_done);
		break;
	case kNotifyHandleReinitReq:
		updated = notification_value(data, size, event_state_.handle_reinit_req);
		break;
	case kNotifyHandleReinitDone:
		updated = notification_value(data, size, event_state_.handle_reinit_done);
		break;
	case kNotifyEstopHold:
		updated = notification_value(data, size, event_state_.estop_hold_req);
		break;
	case kNotifyHostTimeout:
		updated = notification_value(data, size, event_state_.host_comm_timeout);
		break;
	case kNotifyStartupReady:
		updated = notification_value(data, size, event_state_.startup_loading_ready);
		break;
	case kNotifyAxis4Busy:
		updated = notification_value(data, size, event_state_.axis4_manual_busy);
		break;
	case kNotifyAxis4Done:
		updated = notification_value(data, size, event_state_.axis4_manual_done);
		break;
	case kNotifyAxis4Error:
		updated = notification_value(data, size, event_state_.axis4_manual_error);
		break;
	case kNotifyAxis4ErrorId:
		updated = notification_value(data, size, event_state_.axis4_manual_error_id);
		break;
	case kNotifyGenState:
		updated = notification_value(data, size, event_state_.gen_state);
		break;
	case kNotifyAxis1ReturnBusy:
		updated = notification_value(data, size, event_state_.axis1_return_busy);
		break;
	case kNotifyAxis1ReturnDone:
		updated = notification_value(data, size, event_state_.axis1_return_done);
		break;
	case kNotifyAxis1ReturnError:
		updated = notification_value(data, size, event_state_.axis1_return_error);
		break;
	case kNotifyAxis1ReturnErrorId:
		updated = notification_value(data, size, event_state_.axis1_return_error_id);
		break;
	case kNotifyAxis6ReturnBusy:
		updated = notification_value(data, size, event_state_.axis6_return_busy);
		break;
	case kNotifyAxis6ReturnDone:
		updated = notification_value(data, size, event_state_.axis6_return_done);
		break;
	case kNotifyAxis6ReturnError:
		updated = notification_value(data, size, event_state_.axis6_return_error);
		break;
	case kNotifyAxis6ReturnErrorId:
		updated = notification_value(data, size, event_state_.axis6_return_error_id);
		break;
	default:
		break;
	}
	if (updated && event_id < 32)
	{
		notification_update_mask_ |= (1u << event_id);
	}
	if (updated && event_id >= kNotifyAxis1ReturnBusy && event_id <= kNotifyAxis1ReturnErrorId)
	{
		++axis1_return_event_sequence_counter_;
		if (axis1_return_event_sequence_counter_ == 0) ++axis1_return_event_sequence_counter_;
		if (event_id == kNotifyAxis1ReturnBusy &&
			!previous_axis1_return_busy && event_state_.axis1_return_busy)
		{
			event_state_.axis1_return_busy_true_sequence = axis1_return_event_sequence_counter_;
		}
		else if (event_id == kNotifyAxis1ReturnDone &&
			previous_axis1_return_done != event_state_.axis1_return_done)
		{
			if (event_state_.axis1_return_done)
			{
				event_state_.axis1_return_done_true_sequence = axis1_return_event_sequence_counter_;
			}
			else
			{
				event_state_.axis1_return_done_false_sequence = axis1_return_event_sequence_counter_;
			}
		}
		else if (event_id == kNotifyAxis1ReturnError &&
			!previous_axis1_return_error && event_state_.axis1_return_error)
		{
			event_state_.axis1_return_error_true_sequence = axis1_return_event_sequence_counter_;
		}
		else if (event_id == kNotifyAxis1ReturnErrorId &&
			event_state_.axis1_return_error_id != 0)
		{
			event_state_.axis1_return_last_nonzero_error_id =
				event_state_.axis1_return_error_id;
			event_state_.axis1_return_last_nonzero_error_id_sequence =
				axis1_return_event_sequence_counter_;
		}
		LARGE_INTEGER now{};
		QueryPerformanceCounter(&now);
		event_state_.axis1_return_event_sequence = axis1_return_event_sequence_counter_;
		event_state_.axis1_return_event_qpc_ticks = now.QuadPart;
	}
	if (updated && event_id >= kNotifyAxis6ReturnBusy && event_id <= kNotifyAxis6ReturnErrorId)
	{
		++axis6_return_event_sequence_counter_;
		if (axis6_return_event_sequence_counter_ == 0) ++axis6_return_event_sequence_counter_;
		if (event_id == kNotifyAxis6ReturnBusy &&
			!previous_axis6_return_busy && event_state_.axis6_return_busy)
		{
			event_state_.axis6_return_busy_true_sequence = axis6_return_event_sequence_counter_;
		}
		else if (event_id == kNotifyAxis6ReturnDone &&
			previous_axis6_return_done != event_state_.axis6_return_done)
		{
			if (event_state_.axis6_return_done)
			{
				event_state_.axis6_return_done_true_sequence = axis6_return_event_sequence_counter_;
			}
			else
			{
				event_state_.axis6_return_done_false_sequence = axis6_return_event_sequence_counter_;
			}
		}
		else if (event_id == kNotifyAxis6ReturnError &&
			!previous_axis6_return_error && event_state_.axis6_return_error)
		{
			event_state_.axis6_return_error_true_sequence = axis6_return_event_sequence_counter_;
		}
		else if (event_id == kNotifyAxis6ReturnErrorId &&
			event_state_.axis6_return_error_id != 0)
		{
			event_state_.axis6_return_last_nonzero_error_id =
				event_state_.axis6_return_error_id;
			event_state_.axis6_return_last_nonzero_error_id_sequence =
				axis6_return_event_sequence_counter_;
		}
		LARGE_INTEGER now{};
		QueryPerformanceCounter(&now);
		event_state_.axis6_return_event_sequence = axis6_return_event_sequence_counter_;
		event_state_.axis6_return_event_qpc_ticks = now.QuadPart;
	}
}

void AdsCommunicationService::run()
{
	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);
	std::int64_t next_deadline = now.QuadPart;
	std::int64_t next_reconnect_attempt_qpc = now.QuadPart;
	const std::int64_t period_ticks = (std::max<std::int64_t>)(1, qpc_frequency_ / kCommunicationRateHz);
	std::size_t reconnect_backoff_index = 0;
	std::uint32_t cycles_since_state_check = 0;
	bool connection_initialized = false;
	bool connected_once = false;
	bool handles_fresh = false;
	bool reconnect_count_pending = false;

	while (running_.load(std::memory_order_acquire))
	{
		wait_until(next_deadline);
		if (!running_.load(std::memory_order_acquire)) break;
		QueryPerformanceCounter(&now);
		std::uint64_t skipped_deadlines = 0;
		if (now.QuadPart > next_deadline)
		{
			skipped_deadlines = static_cast<std::uint64_t>(
				(now.QuadPart - next_deadline) / period_ticks);
		}
		next_deadline += static_cast<std::int64_t>(skipped_deadlines + 1) * period_ticks;
		if (skipped_deadlines > 0)
		{
			std::lock_guard<std::mutex> lock(stats_mutex_);
			stats_.missed_deadlines += skipped_deadlines;
		}

		// 端口可能由主程序预先打开；服务自身仍必须完成通知和缓存初始化。
		if (!ads_.IsCommOpen() || !connection_initialized)
		{
			const bool reconnecting = connected_once;
			set_connection_state(plc_restart_active_
				? AdsConnectionState::PlcRestarted
				: (reconnecting ? AdsConnectionState::Reconnecting : AdsConnectionState::Connecting));
			if (now.QuadPart >= next_reconnect_attempt_qpc)
			{
				if (ensure_connection(reconnecting))
				{
					connection_initialized = true;
					connected_once = true;
					handles_fresh = true;
					cycles_since_state_check = 0;
					reconnect_backoff_index = 0;
					QueryPerformanceCounter(&now);
					next_reconnect_attempt_qpc = now.QuadPart;
					last_full_success_qpc_ = now.QuadPart;
					if (reconnecting) reconnect_count_pending = true;
				}
				else
				{
					QueryPerformanceCounter(&now);
					const std::int64_t delay_ticks = qpc_frequency_ *
						static_cast<std::int64_t>(kReconnectIntervalMs) / 1000;
					next_reconnect_attempt_qpc = now.QuadPart + (std::max<std::int64_t>)(1, delay_ticks);
				}
			}
			if (!connection_initialized)
			{
				fail_queued_low_frequency_requests();
				fail_queued_planned_return_commands();
				AdsFastSnapshot unavailable_snapshot{};
				unavailable_snapshot.attempt_sequence = ++attempt_sequence_;
				unavailable_snapshot.qpc_ticks = now.QuadPart;
				set_connection_state(plc_restart_active_
					? AdsConnectionState::PlcRestarted
					: (reconnecting ? AdsConnectionState::Reconnecting : AdsConnectionState::Connecting));
				update_rate(unavailable_snapshot.qpc_ticks, 0, false);
				publish_snapshot(unavailable_snapshot);
				continue;
			}
		}

		if (coordinate_refresh_pending_.exchange(false, std::memory_order_acq_rel))
		{
			if (!refresh_coordinate_cache_now())
			{
				coordinate_refresh_pending_.store(true, std::memory_order_release);
			}
		}

		bool device_state_ok = true;
		if (++cycles_since_state_check >= kDeviceStateCheckPeriodCycles)
		{
			cycles_since_state_check = 0;
			unsigned short ads_state = ADSSTATE_INVALID;
			unsigned short device_state = 0;
			device_state_ok = ads_.ReadDeviceState(ads_state, device_state);
			if (device_state_ok && ads_state != ADSSTATE_RUN)
			{
				mark_plc_restart(false);
				fail_queued_low_frequency_requests();
				fail_queued_planned_return_commands();
				AdsFastSnapshot stopped_snapshot{};
				stopped_snapshot.attempt_sequence = ++attempt_sequence_;
				QueryPerformanceCounter(&now);
				stopped_snapshot.qpc_ticks = now.QuadPart;
				update_rate(stopped_snapshot.qpc_ticks, 0, false);
				set_connection_state(AdsConnectionState::PlcRestarted);
				publish_snapshot(stopped_snapshot);
				clear_runtime_connection_state();
				ads_.CloseComm();
				connection_initialized = false;
				handles_fresh = false;
				reconnect_backoff_index = 0;
				next_reconnect_attempt_qpc = stopped_snapshot.qpc_ticks;
				continue;
			}
		}
		if (!device_state_ok)
		{
			fail_queued_low_frequency_requests();
			fail_queued_planned_return_commands();
			AdsFastSnapshot state_failure_snapshot{};
			state_failure_snapshot.attempt_sequence = ++attempt_sequence_;
			QueryPerformanceCounter(&now);
			state_failure_snapshot.qpc_ticks = now.QuadPart;
			const bool hard_timeout = last_full_success_qpc_ == 0 ||
				state_failure_snapshot.qpc_ticks - last_full_success_qpc_ >=
				qpc_frequency_ * kHardTimeoutMs / 1000;
			set_connection_state(hard_timeout
				? AdsConnectionState::Reconnecting
				: AdsConnectionState::SoftHold);
			update_rate(state_failure_snapshot.qpc_ticks, 0, false);
			publish_snapshot(state_failure_snapshot);
			if (hard_timeout)
			{
				clear_runtime_connection_state();
				ads_.CloseComm();
				connection_initialized = false;
				handles_fresh = false;
				reconnect_backoff_index = 0;
				next_reconnect_attempt_qpc = state_failure_snapshot.qpc_ticks;
			}
			continue;
		}

		AdsFastSnapshot snapshot{};
		snapshot.attempt_sequence = ++attempt_sequence_;
		const bool read_ok = read_fast_snapshot(snapshot, handles_fresh);
		handles_fresh = false;
		if (read_ok && snapshot.host_comm_timeout)
		{
			// 新进程首次写入会话号后，PLC 可能才锁存通信超时。
			// 每个有效快照都补发恢复请求，直到 PLC 完成停稳与重初始化握手。
			watchdog_recovery_pending_.store(true, std::memory_order_release);
		}
		bool planned_return_processed = false;
		const bool write_ok = read_ok && !restart_reconnect_pending_
			? write_output_cycle(planned_return_processed)
			: false;
		const bool cycle_ok = device_state_ok && read_ok && write_ok;
		update_rate(snapshot.qpc_ticks, snapshot.rtt_us, cycle_ok);

		const std::int64_t failure_age_ticks = snapshot.qpc_ticks - last_full_success_qpc_;
		const bool hard_timeout = last_full_success_qpc_ == 0 ||
			failure_age_ticks >= qpc_frequency_ * kHardTimeoutMs / 1000;
		bool disconnect = false;
		bool process_low_frequency = false;
		if (restart_reconnect_pending_)
		{
			restart_reconnect_pending_ = false;
			set_connection_state(AdsConnectionState::PlcRestarted);
			disconnect = true;
		}
		else if (plc_restart_active_)
		{
			// 至少保留一个完整恢复周期的 PlcRestarted 状态，避免被同拍 Running 覆盖。
			set_connection_state(AdsConnectionState::PlcRestarted);
			if (cycle_ok)
			{
				last_full_success_qpc_ = snapshot.qpc_ticks;
				plc_restart_active_ = false;
			}
			else if (hard_timeout)
			{
				disconnect = true;
			}
		}
		else if (cycle_ok)
		{
			last_full_success_qpc_ = snapshot.qpc_ticks;
			set_connection_state(AdsConnectionState::Running);
			if (reconnect_count_pending)
			{
				std::lock_guard<std::mutex> lock(stats_mutex_);
				++stats_.reconnect_count;
				reconnect_count_pending = false;
			}
			process_low_frequency = true;
		}
		else
		{
			set_connection_state(hard_timeout
				? AdsConnectionState::Reconnecting
				: AdsConnectionState::SoftHold);
			disconnect = hard_timeout;
		}

		// 只有完整读写成功且本拍正式进入 Running 才发布可用于运动控制的测量；
		// 力数据的首尾 PLC 周期跨度只保留作诊断，不参与有效性和运动保持判定。
		if (!process_low_frequency)
		{
			snapshot.position_valid = false;
			snapshot.force_valid = false;
			snapshot.valid = false;
		}
		// 先提交状态再唤醒快照消费者，避免消费者看到“新快照 + 旧 Running 状态”。
		publish_snapshot(snapshot);
		if (process_low_frequency)
		{
			if (!planned_return_processed) process_one_low_frequency_request();
		}
		else
		{
			fail_queued_planned_return_commands();
			fail_queued_low_frequency_requests();
		}
		if (disconnect)
		{
			clear_runtime_connection_state();
			ads_.CloseComm();
			connection_initialized = false;
			handles_fresh = false;
			reconnect_backoff_index = 0;
			next_reconnect_attempt_qpc = snapshot.qpc_ticks;
		}
	}

	clear_runtime_connection_state();
	fail_queued_planned_return_commands();
	fail_queued_low_frequency_requests();
	set_connection_state(AdsConnectionState::Disconnected);
}

bool AdsCommunicationService::ensure_connection(bool reconnecting)
{
	// 服务只被动建立路由，禁止在后台把人工 STOP/故障 STOP 自动切回 RUN。
	// 握手连接阶段使用 1000 ms 宽容超时
	ads_.SetTimeout(1000);
	if (!ads_.IsCommOpen() && !ads_.OpenCommInsideReadOnly() && !ads_.OpenCommReadOnly()) return false;
	unsigned short ads_state = ADSSTATE_INVALID;
	unsigned short device_state = 0;
	if (!ads_.ReadDeviceState(ads_state, device_state))
	{
		clear_runtime_connection_state();
		ads_.CloseComm();
		return false;
	}
	if (ads_state != ADSSTATE_RUN)
	{
		if (reconnecting) mark_plc_restart(false);
		clear_runtime_connection_state();
		ads_.CloseComm();
		return false;
	}
	if (!initialize_connection())
	{
		clear_runtime_connection_state();
		ads_.CloseComm();
		return false;
	}
	// 运行期允许单次 ADS 调用等待 300 ms，短时调度抖动不直接升级为断线。
	if (!ads_.SetTimeout(300))
	{
		clear_runtime_connection_state();
		ads_.CloseComm();
		return false;
	}
	return true;
}

bool AdsCommunicationService::initialize_connection()
{
	char app_name[64] = {};
	if (!ads_.ADSRead(AdsSymbol::app_name, sizeof(app_name), app_name)) return false;
	app_name[sizeof(app_name) - 1] = '\0';
	const std::string current_app_name(app_name);
	const bool app_name_changed =
		!last_plc_app_name_.empty() && current_app_name != last_plc_app_name_;

	if (!resolve_fast_handles()) return false;
	if (!refresh_coordinate_cache_now()) return false;
	{
		std::lock_guard<std::mutex> lock(event_mutex_);
		event_state_ = AdsEventState{};
		event_state_.axis1_return_event_sequence = axis1_return_event_sequence_counter_;
		event_state_.axis6_return_event_sequence = axis6_return_event_sequence_counter_;
		notification_update_mask_ = 0;
	}
	if (!register_notifications()) return false;

	// 注册完成后主动读取一次初值，避免依赖下一次变化。
	const char* symbols[] = {
		AdsSymbol::self_check_done,
		AdsSymbol::handle_reinit_req,
		"G.handle_reinit_done",
		AdsSymbol::estop_hold_req,
		"G.host_comm_timeout",
		AdsSymbol::startup_loading_ready,
		AdsSymbol::axis4_manual_busy,
		"G.axis4_manual_done",
		AdsSymbol::axis4_manual_error,
		AdsSymbol::axis4_manual_error_id,
		AdsSymbol::gen_state,
		AdsSymbol::axis1_return.busy,
		AdsSymbol::axis1_return.done,
		AdsSymbol::axis1_return.error,
		AdsSymbol::axis1_return.error_id,
		AdsSymbol::axis6_return.busy,
		AdsSymbol::axis6_return.done,
		AdsSymbol::axis6_return.error,
		AdsSymbol::axis6_return.error_id
	};
	AdsEventState initial{};
	const unsigned long lengths[] = {
		sizeof(initial.self_check_done), sizeof(initial.handle_reinit_req),
		sizeof(initial.handle_reinit_done), sizeof(initial.estop_hold_req),
		sizeof(initial.host_comm_timeout), sizeof(initial.startup_loading_ready),
		sizeof(initial.axis4_manual_busy), sizeof(initial.axis4_manual_done),
		sizeof(initial.axis4_manual_error), sizeof(initial.axis4_manual_error_id),
		sizeof(initial.gen_state), sizeof(initial.axis1_return_busy),
		sizeof(initial.axis1_return_done), sizeof(initial.axis1_return_error),
		sizeof(initial.axis1_return_error_id), sizeof(initial.axis6_return_busy),
		sizeof(initial.axis6_return_done), sizeof(initial.axis6_return_error),
		sizeof(initial.axis6_return_error_id)
	};
	void* outputs[] = {
		&initial.self_check_done, &initial.handle_reinit_req, &initial.handle_reinit_done,
		&initial.estop_hold_req, &initial.host_comm_timeout, &initial.startup_loading_ready,
		&initial.axis4_manual_busy, &initial.axis4_manual_done, &initial.axis4_manual_error,
		&initial.axis4_manual_error_id, &initial.gen_state,
		&initial.axis1_return_busy, &initial.axis1_return_done,
		&initial.axis1_return_error, &initial.axis1_return_error_id,
		&initial.axis6_return_busy, &initial.axis6_return_done,
		&initial.axis6_return_error, &initial.axis6_return_error_id
	};
	if (!ads_.ADSReadSum(symbols, lengths, outputs, _countof(symbols))) return false;
	bool host_comm_timeout = false;
	{
		std::lock_guard<std::mutex> lock(event_mutex_);
		auto untouched = [&](std::uint32_t event_id)
		{
			return (notification_update_mask_ & (1u << event_id)) == 0;
		};
		if (untouched(kNotifySelfCheckDone)) event_state_.self_check_done = initial.self_check_done;
		if (untouched(kNotifyHandleReinitReq)) event_state_.handle_reinit_req = initial.handle_reinit_req;
		if (untouched(kNotifyHandleReinitDone)) event_state_.handle_reinit_done = initial.handle_reinit_done;
		if (untouched(kNotifyEstopHold)) event_state_.estop_hold_req = initial.estop_hold_req;
		if (untouched(kNotifyHostTimeout)) event_state_.host_comm_timeout = initial.host_comm_timeout;
		if (untouched(kNotifyStartupReady)) event_state_.startup_loading_ready = initial.startup_loading_ready;
		if (untouched(kNotifyAxis4Busy)) event_state_.axis4_manual_busy = initial.axis4_manual_busy;
		if (untouched(kNotifyAxis4Done)) event_state_.axis4_manual_done = initial.axis4_manual_done;
		if (untouched(kNotifyAxis4Error)) event_state_.axis4_manual_error = initial.axis4_manual_error;
		if (untouched(kNotifyAxis4ErrorId)) event_state_.axis4_manual_error_id = initial.axis4_manual_error_id;
		if (untouched(kNotifyGenState)) event_state_.gen_state = initial.gen_state;
		if (untouched(kNotifyAxis1ReturnBusy)) event_state_.axis1_return_busy = initial.axis1_return_busy;
		if (untouched(kNotifyAxis1ReturnDone)) event_state_.axis1_return_done = initial.axis1_return_done;
		if (untouched(kNotifyAxis1ReturnError)) event_state_.axis1_return_error = initial.axis1_return_error;
		if (untouched(kNotifyAxis1ReturnErrorId)) event_state_.axis1_return_error_id = initial.axis1_return_error_id;
		if (untouched(kNotifyAxis6ReturnBusy)) event_state_.axis6_return_busy = initial.axis6_return_busy;
		if (untouched(kNotifyAxis6ReturnDone)) event_state_.axis6_return_done = initial.axis6_return_done;
		if (untouched(kNotifyAxis6ReturnError)) event_state_.axis6_return_error = initial.axis6_return_error;
		if (untouched(kNotifyAxis6ReturnErrorId)) event_state_.axis6_return_error_id = initial.axis6_return_error_id;
		++axis1_return_event_sequence_counter_;
		if (axis1_return_event_sequence_counter_ == 0) ++axis1_return_event_sequence_counter_;
		if (event_state_.axis1_return_busy)
		{
			event_state_.axis1_return_busy_true_sequence = axis1_return_event_sequence_counter_;
		}
		if (!event_state_.axis1_return_done)
		{
			event_state_.axis1_return_done_false_sequence = axis1_return_event_sequence_counter_;
		}
		else
		{
			event_state_.axis1_return_done_true_sequence = axis1_return_event_sequence_counter_;
		}
		if (event_state_.axis1_return_error)
		{
			event_state_.axis1_return_error_true_sequence = axis1_return_event_sequence_counter_;
		}
		if (event_state_.axis1_return_error_id != 0)
		{
			event_state_.axis1_return_last_nonzero_error_id =
				event_state_.axis1_return_error_id;
			event_state_.axis1_return_last_nonzero_error_id_sequence =
				axis1_return_event_sequence_counter_;
		}
		LARGE_INTEGER event_now{};
		QueryPerformanceCounter(&event_now);
		event_state_.axis1_return_event_sequence = axis1_return_event_sequence_counter_;
		event_state_.axis1_return_event_qpc_ticks = event_now.QuadPart;
		++axis6_return_event_sequence_counter_;
		if (axis6_return_event_sequence_counter_ == 0) ++axis6_return_event_sequence_counter_;
		if (event_state_.axis6_return_busy)
		{
			event_state_.axis6_return_busy_true_sequence = axis6_return_event_sequence_counter_;
		}
		if (!event_state_.axis6_return_done)
		{
			event_state_.axis6_return_done_false_sequence = axis6_return_event_sequence_counter_;
		}
		else
		{
			event_state_.axis6_return_done_true_sequence = axis6_return_event_sequence_counter_;
		}
		if (event_state_.axis6_return_error)
		{
			event_state_.axis6_return_error_true_sequence = axis6_return_event_sequence_counter_;
		}
		if (event_state_.axis6_return_error_id != 0)
		{
			event_state_.axis6_return_last_nonzero_error_id =
				event_state_.axis6_return_error_id;
			event_state_.axis6_return_last_nonzero_error_id_sequence =
				axis6_return_event_sequence_counter_;
		}
		event_state_.axis6_return_event_sequence = axis6_return_event_sequence_counter_;
		event_state_.axis6_return_event_qpc_ticks = event_now.QuadPart;
		host_comm_timeout = event_state_.host_comm_timeout;
		notification_update_mask_ = 0;
	}
	if (app_name_changed) mark_plc_restart(false);
	last_plc_app_name_ = current_app_name;
	if (host_comm_timeout) watchdog_recovery_pending_.store(true, std::memory_order_release);
	return true;
}

bool AdsCommunicationService::resolve_fast_handles()
{
	clear_fast_handles();
	auto required_handle = [&](const char* symbol, unsigned long& handle) -> bool
	{
		handle = ads_.ADSGetAddr(symbol);
		return handle != 0;
	};

	unsigned long cycle = 0;
	unsigned long dc_time = 0;
	unsigned long act_pos = 0;
	unsigned long axis1_act_velocity = 0;
	unsigned long ft_1 = 0;
	unsigned long fn_1 = 0;
	unsigned long fn_2 = 0;
	unsigned long ft_2 = 0;
	unsigned long estop = 0;
	unsigned long host_timeout = 0;
	if (!required_handle("TwinCAT_SystemInfoVarList._TaskInfo[1].CycleCount", cycle) ||
		!required_handle("TwinCAT_SystemInfoVarList._TaskInfo[1].DcTaskTime", dc_time) ||
		!required_handle(AdsSymbol::act_pos, act_pos) ||
		!required_handle("G.axis[1].NcToPlc.ActVelo", axis1_act_velocity) ||
		!required_handle(AdsSymbol::ft_1_value, ft_1) ||
		!required_handle(AdsSymbol::fn_1_value, fn_1) ||
		!required_handle(AdsSymbol::fn_2_value, fn_2) ||
		!required_handle(AdsSymbol::ft_2_value, ft_2) ||
		!required_handle(AdsSymbol::estop_hold_req, estop) ||
		!required_handle(AdsSymbol::host_comm_timeout, host_timeout))
	{
		clear_fast_handles();
		return false;
	}

	fast_fallback_read_handles_ = {
		cycle, dc_time, act_pos, axis1_act_velocity,
		ft_1, fn_1, fn_2, ft_2, estop, host_timeout, cycle
	};
	fast_direct_read_handles_[0] = cycle;
	fast_direct_read_handles_[1] = dc_time;
	fast_direct_read_handles_[9] = axis1_act_velocity;
	fast_direct_read_handles_[10] = ft_1;
	fast_direct_read_handles_[11] = fn_1;
	fast_direct_read_handles_[12] = fn_2;
	fast_direct_read_handles_[13] = ft_2;
	fast_direct_read_handles_[14] = estop;
	fast_direct_read_handles_[15] = host_timeout;
	fast_direct_read_handles_[16] = cycle;
	use_direct_nc_position_ = true;
	for (int axis = 0; axis < 7; ++axis)
	{
		char symbol[64] = {};
		sprintf_s(symbol, "G.axis[%d].NcToPlc.ActPos", axis + 1);
		fast_direct_read_handles_[static_cast<std::size_t>(axis) + 2] = ads_.ADSGetAddr(symbol);
		if (fast_direct_read_handles_[static_cast<std::size_t>(axis) + 2] == 0)
		{
			use_direct_nc_position_ = false;
			break;
		}
	}

	const char* write_symbols[] = {
		AdsSymbol::host_session_id,
		AdsSymbol::host_heartbeat_sequence,
		AdsSymbol::refer,
		AdsSymbol::axis1_fast_return,
		AdsSymbol::axis6_fast_retract,
		AdsSymbol::cylinder1_value,
		AdsSymbol::cylinder2_value,
		AdsSymbol::cylinder3_value,
		AdsSymbol::cylinder4_value,
		AdsSymbol::cylinder5_press_req,
		AdsSymbol::axis4_fwd_req,
		AdsSymbol::axis4_rev_req,
		AdsSymbol::inject_push_req,
		AdsSymbol::inject_pull_req,
		AdsSymbol::startup_smoothing_bypass,
		AdsSymbol::host_recover_req
	};
	static_assert(
		_countof(write_symbols) == kFastWriteHandleCount,
		"快速写句柄符号表与索引枚举数量不一致");
	static_assert(
		_countof(write_symbols) == std::tuple_size<decltype(fast_write_handles_)>::value,
		"快速写句柄符号表与句柄数组数量不一致");
	for (std::size_t i = 0; i < fast_write_handles_.size(); ++i)
	{
		if (!required_handle(write_symbols[i], fast_write_handles_[i]))
		{
			clear_fast_handles();
			return false;
		}
	}
	const AxisReturnAdsSymbols* return_symbols[] = {
		&AdsSymbol::axis1_return,
		&AdsSymbol::axis6_return
	};
	for (std::size_t axis_slot = 0; axis_slot < _countof(return_symbols); ++axis_slot)
	{
		const AxisReturnAdsSymbols& symbols = *return_symbols[axis_slot];
		const char* fields[] = {
			symbols.req,
			symbols.target_abs,
			symbols.velocity,
			symbols.acc,
			symbols.dec,
			symbols.jerk
		};
		for (std::size_t field = 0; field < _countof(fields); ++field)
		{
			if (!required_handle(fields[field], planned_return_write_handles_[axis_slot][field]))
			{
				clear_fast_handles();
				return false;
			}
		}
	}
	fast_handles_valid_ = true;
	return true;
}

void AdsCommunicationService::clear_fast_handles()
{
	fast_direct_read_handles_.fill(0);
	fast_fallback_read_handles_.fill(0);
	fast_write_handles_.fill(0);
	for (auto& handles : planned_return_write_handles_) handles.fill(0);
	fast_handles_valid_ = false;
}

bool AdsCommunicationService::refresh_coordinate_cache_now()
{
	double init_pos[7] = {};
	double leftlimit[7] = {};
	const char* symbols[] = { AdsSymbol::init_pos, AdsSymbol::leftlimit };
	const unsigned long lengths[] = { sizeof(init_pos), sizeof(leftlimit) };
	void* outputs[] = { init_pos, leftlimit };
	if (!ads_.ADSReadSum(symbols, lengths, outputs, 2)) return false;
	std::lock_guard<std::mutex> lock(coordinate_mutex_);
	std::copy(init_pos, init_pos + 7, init_pos_);
	std::copy(leftlimit, leftlimit + 7, leftlimit_);
	coordinate_cache_valid_ = true;
	return true;
}

bool AdsCommunicationService::read_fast_snapshot(AdsFastSnapshot& snapshot, bool handles_fresh)
{
	LARGE_INTEGER before{};
	LARGE_INTEGER after{};
	QueryPerformanceCounter(&before);
	double nc_absolute[7] = {};
	double relative_fallback[7] = {};
	std::array<unsigned long, 17> lengths{};
	std::array<void*, 17> outputs{};
	auto read_snapshot = [&](const unsigned long* handles, bool direct_nc_position)
	{
		std::size_t count = 0;
		auto append = [&](unsigned long length, void* output)
		{
			lengths[count] = length;
			outputs[count] = output;
			++count;
		};
		append(sizeof(snapshot.plc_cycle_begin), &snapshot.plc_cycle_begin);
		append(sizeof(snapshot.plc_dc_task_time), &snapshot.plc_dc_task_time);
		if (direct_nc_position)
		{
			for (int axis = 0; axis < 7; ++axis)
			{
				append(sizeof(nc_absolute[axis]), &nc_absolute[axis]);
			}
		}
		else
		{
			append(sizeof(relative_fallback), relative_fallback);
		}
		append(sizeof(snapshot.axis1_act_velocity_mm_s), &snapshot.axis1_act_velocity_mm_s);
		append(sizeof(snapshot.ft_1_value), &snapshot.ft_1_value);
		append(sizeof(snapshot.fn_1_value), &snapshot.fn_1_value);
		append(sizeof(snapshot.fn_2_value), &snapshot.fn_2_value);
		append(sizeof(snapshot.ft_2_value), &snapshot.ft_2_value);
		append(sizeof(snapshot.estop_hold_req), &snapshot.estop_hold_req);
		append(sizeof(snapshot.host_comm_timeout), &snapshot.host_comm_timeout);
		append(sizeof(snapshot.plc_cycle_end), &snapshot.plc_cycle_end);
		return ads_.ADSReadSumByHandle(
			handles, lengths.data(), outputs.data(), static_cast<unsigned long>(count));
	};

	const bool read_ok = fast_handles_valid_ && (use_direct_nc_position_
		? read_snapshot(fast_direct_read_handles_.data(), true)
		: read_snapshot(fast_fallback_read_handles_.data(), false));
	QueryPerformanceCounter(&after);
	snapshot.qpc_ticks = before.QuadPart + (after.QuadPart - before.QuadPart) / 2;
	snapshot.rtt_us = static_cast<std::uint64_t>(
		(after.QuadPart - before.QuadPart) * 1000000LL / qpc_frequency_);
	if (!read_ok)
	{
		snapshot.valid = false;
		return false;
	}
	snapshot.plc_cycle_span = snapshot.plc_cycle_end - snapshot.plc_cycle_begin;

	bool coordinate_valid = false;
	double init_pos[7] = {};
	double leftlimit[7] = {};
	{
		std::lock_guard<std::mutex> lock(coordinate_mutex_);
		coordinate_valid = coordinate_cache_valid_;
		std::copy(init_pos_, init_pos_ + 7, init_pos);
		std::copy(leftlimit_, leftlimit_ + 7, leftlimit);
	}
	bool positions_finite = coordinate_valid;
	for (int axis = 0; axis < 7; ++axis)
	{
		snapshot.act_pos_rel[axis] = use_direct_nc_position_
			? nc_absolute[axis] - init_pos[axis]
			: relative_fallback[axis];
		snapshot.act_pos_from_left[axis] =
			snapshot.act_pos_rel[axis] + init_pos[axis] - leftlimit[axis];
		positions_finite = positions_finite &&
			std::isfinite(snapshot.act_pos_rel[axis]) &&
			std::isfinite(snapshot.act_pos_from_left[axis]);
	}
	// PLC 侧环形锁存允许首尾周期不同，因此 plc_cycle_span 只保留作诊断。
	// 力数据只要本次 Sum Read 成功且四路数值有限即可使用。
	const bool force_values_finite =
		std::isfinite(static_cast<double>(snapshot.ft_1_value)) &&
		std::isfinite(static_cast<double>(snapshot.fn_1_value)) &&
		std::isfinite(static_cast<double>(snapshot.fn_2_value)) &&
		std::isfinite(static_cast<double>(snapshot.ft_2_value));
	snapshot.position_valid = positions_finite &&
		std::isfinite(snapshot.axis1_act_velocity_mm_s);
	snapshot.force_valid = force_values_finite;
	snapshot.valid = snapshot.position_valid && snapshot.force_valid;

	const bool plc_time_regressed =
		last_plc_dc_time_ > 0 && snapshot.plc_dc_task_time < last_plc_dc_time_;
	const bool plc_cycle_regressed =
		last_plc_cycle_ != 0 && snapshot.plc_cycle_end < last_plc_cycle_ &&
		(last_plc_cycle_ - snapshot.plc_cycle_end) < 0x80000000u;
	const bool plc_task_stalled =
		(last_plc_cycle_ != 0 || last_plc_dc_time_ != 0) &&
		snapshot.plc_cycle_end == last_plc_cycle_ &&
		snapshot.plc_dc_task_time == last_plc_dc_time_;
	stalled_plc_snapshot_count_ = plc_task_stalled
		? (std::min)(stalled_plc_snapshot_count_ + 1, kMaxStalledPlcSnapshots)
		: 0;
	last_plc_dc_time_ = snapshot.plc_dc_task_time;
	last_plc_cycle_ = snapshot.plc_cycle_end;
	if (plc_time_regressed || plc_cycle_regressed)
	{
		mark_plc_restart(!handles_fresh);
		snapshot.position_valid = false;
		snapshot.force_valid = false;
		snapshot.valid = false;
		return false;
	}
	if (stalled_plc_snapshot_count_ >= kMaxStalledPlcSnapshots)
	{
		snapshot.position_valid = false;
		snapshot.force_valid = false;
		snapshot.valid = false;
		return false;
	}
	// 返回值表示本拍是否仍可用于运动控制，而不是力数据是否有效。
	return snapshot.position_valid;
}

bool AdsCommunicationService::write_output_cycle(bool& planned_return_processed)
{
	AdsOutputCommand output{};
	AdsOutputCommand last_sent_output{};
	bool has_output = false;
	bool has_last_sent_output = false;
	std::uint64_t output_generation = 0;
	{
		std::lock_guard<std::mutex> lock(output_mutex_);
		output = desired_output_;
		has_output = has_desired_output_;
		output_generation = desired_output_generation_;
		last_sent_output = last_sent_output_;
		has_last_sent_output = has_last_sent_output_;
	}
	const std::uint8_t output_clear_mask = has_output
		? static_cast<std::uint8_t>(output.planned_return_clear_mask & 0x03u)
		: 0;
	planned_return_processed = output_clear_mask != 0;

	std::uint64_t planned_read_index =
		planned_return_read_index_.load(std::memory_order_relaxed);
	PlannedReturnQueueSlot planned_slot{};
	bool has_planned_return = false;
	for (;;)
	{
		const std::uint64_t planned_write_index =
			planned_return_write_index_.load(std::memory_order_acquire);
		if (planned_read_index == planned_write_index) break;
		planned_slot = planned_return_queue_[
			planned_read_index % kPlannedReturnQueueCapacity];

		bool superseded_by_clear = false;
		if (planned_slot.command.operation == AdsPlannedReturnOperation::Prepare ||
			planned_slot.command.operation == AdsPlannedReturnOperation::Commit)
		{
			for (int leg_index = 0;
				leg_index < planned_slot.command.leg_count;
				++leg_index)
			{
				const int axis_slot = planned_return_axis_slot(
					planned_slot.command.legs[leg_index].axis_index);
				const std::uint8_t axis_mask = axis_slot == 0 ? 0x01u : 0x02u;
				if (axis_slot >= 0 &&
					((output_clear_mask & axis_mask) != 0 ||
						planned_return_clear_barrier_sequence_[static_cast<std::size_t>(axis_slot)]
							.load(std::memory_order_acquire) > planned_slot.sequence))
				{
					superseded_by_clear = true;
					break;
				}
			}
		}
		if (!superseded_by_clear)
		{
			has_planned_return = true;
			planned_return_processed = true;
			break;
		}

		// 被队列Clear屏障或当前输出清Req掩码覆盖的命令尚未开始，不占用本拍ADS写事务。
		complete_planned_return_command(planned_slot.sequence, false);
		++planned_read_index;
		planned_return_read_index_.store(planned_read_index, std::memory_order_release);
	}

	++heartbeat_sequence_;
	// 常规输出最多16项，双腿Prepare最多再增加12项。
	std::array<unsigned long, 32> handles{};
	std::array<unsigned long, 32> lengths{};
	std::array<const void*, 32> values{};
	std::array<unsigned long, 32> item_results{};
	std::size_t count = 0;
	auto append = [&](unsigned long handle, unsigned long length, const void* value) -> std::size_t
	{
		const std::size_t index = count++;
		handles[index] = handle;
		lengths[index] = length;
		values[index] = value;
		return index;
	};

	const bool false_value = false;
	const bool true_value = true;
	if (has_output && output.motion_enabled)
	{
		append(fast_write_handles_[kWriteRefer], sizeof(output.refer), output.refer);
		append(fast_write_handles_[kWriteAxis1FastReturn], sizeof(output.axis1_fast_return), &output.axis1_fast_return);
		append(fast_write_handles_[kWriteAxis6FastRetract], sizeof(output.axis6_fast_retract), &output.axis6_fast_retract);
	}
	else if (has_last_sent_output && last_sent_output.motion_enabled)
	{
		append(fast_write_handles_[kWriteAxis1FastReturn], sizeof(false_value), &false_value);
		append(fast_write_handles_[kWriteAxis6FastRetract], sizeof(false_value), &false_value);
	}

	const bool discrete_dirty = has_output &&
		(!has_last_sent_output || !same_discrete_output(output, last_sent_output));
	if (discrete_dirty)
	{
		if (output.cylinder_valid)
		{
			append(fast_write_handles_[kWriteCylinder1], sizeof(output.cylinder[0]), &output.cylinder[0]);
			append(fast_write_handles_[kWriteCylinder2], sizeof(output.cylinder[1]), &output.cylinder[1]);
			append(fast_write_handles_[kWriteCylinder3], sizeof(output.cylinder[2]), &output.cylinder[2]);
			append(fast_write_handles_[kWriteCylinder4], sizeof(output.cylinder[3]), &output.cylinder[3]);
		}
		append(fast_write_handles_[kWriteCylinder5Press], sizeof(output.cylinder5_press_req), &output.cylinder5_press_req);
		append(fast_write_handles_[kWriteAxis4Forward], sizeof(output.axis4_forward_req), &output.axis4_forward_req);
		append(fast_write_handles_[kWriteAxis4Reverse], sizeof(output.axis4_reverse_req), &output.axis4_reverse_req);
		append(fast_write_handles_[kWriteInjectorPush], sizeof(output.inject_push_req), output.inject_push_req);
		append(fast_write_handles_[kWriteInjectorPull], sizeof(output.inject_pull_req), output.inject_pull_req);
		append(fast_write_handles_[kWriteSmoothingBypass], sizeof(output.startup_smoothing_bypass), &output.startup_smoothing_bypass);
	}

	bool planned_fields_valid = true;
	std::uint8_t planned_req_false_mask = 0;
	std::array<int, 2> planned_req_false_item_index{ -1, -1 };
	std::uint8_t planned_commit_axis_mask = 0;
	std::array<int, 2> planned_commit_req_item_index{ -1, -1 };
	const std::size_t planned_fields_begin = count;
	if (has_planned_return)
	{
		for (int leg_index = 0;
			leg_index < planned_slot.command.leg_count;
			++leg_index)
		{
			const AdsPlannedReturnLegCommand& leg = planned_slot.command.legs[leg_index];
			const int axis_slot = planned_return_axis_slot(leg.axis_index);
			if (axis_slot < 0)
			{
				planned_fields_valid = false;
				break;
			}
			const auto& axis_handles =
				planned_return_write_handles_[static_cast<std::size_t>(axis_slot)];
			const std::uint8_t axis_mask = axis_slot == 0 ? 0x01u : 0x02u;
			switch (planned_slot.command.operation)
			{
			case AdsPlannedReturnOperation::Prepare:
				planned_req_false_item_index[static_cast<std::size_t>(axis_slot)] =
					static_cast<int>(append(axis_handles[0], sizeof(false_value), &false_value));
				planned_req_false_mask |= axis_mask;
				append(axis_handles[1], sizeof(leg.target_abs), &leg.target_abs);
				append(axis_handles[2], sizeof(leg.velocity), &leg.velocity);
				append(axis_handles[3], sizeof(leg.acc), &leg.acc);
				append(axis_handles[4], sizeof(leg.dec), &leg.dec);
				append(axis_handles[5], sizeof(leg.jerk), &leg.jerk);
				break;
			case AdsPlannedReturnOperation::Commit:
				planned_commit_req_item_index[static_cast<std::size_t>(axis_slot)] =
					static_cast<int>(append(axis_handles[0], sizeof(true_value), &true_value));
				planned_commit_axis_mask |= axis_mask;
				break;
			case AdsPlannedReturnOperation::Clear:
				planned_req_false_item_index[static_cast<std::size_t>(axis_slot)] =
					static_cast<int>(append(axis_handles[0], sizeof(false_value), &false_value));
				planned_req_false_mask |= axis_mask;
				break;
			default:
				planned_fields_valid = false;
				break;
			}
			if (!planned_fields_valid) break;
		}
		if (!planned_fields_valid)
		{
			// 无效命令只失败出队，不能把半条Prepare混进常规输出事务。
			count = planned_fields_begin;
			planned_req_false_mask = 0;
			planned_req_false_item_index = { -1, -1 };
			planned_commit_axis_mask = 0;
			planned_commit_req_item_index = { -1, -1 };
		}
	}
	const std::size_t planned_fields_end = count;
	std::array<int, 2> output_clear_item_index{ -1, -1 };
	if ((output_clear_mask & 0x01u) != 0 && (planned_req_false_mask & 0x01u) == 0)
	{
		output_clear_item_index[0] = static_cast<int>(append(
			planned_return_write_handles_[0][0],
			sizeof(false_value),
			&false_value));
	}
	else if ((output_clear_mask & 0x01u) != 0)
	{
		output_clear_item_index[0] = planned_req_false_item_index[0];
	}
	if ((output_clear_mask & 0x02u) != 0 && (planned_req_false_mask & 0x02u) == 0)
	{
		output_clear_item_index[1] = static_cast<int>(append(
			planned_return_write_handles_[1][0],
			sizeof(false_value),
			&false_value));
	}
	else if ((output_clear_mask & 0x02u) != 0)
	{
		output_clear_item_index[1] = planned_req_false_item_index[1];
	}

	// 心跳与输出放在同一Sum事务；这里的排列只用于后续逐项结果分类，
	// 不把SUMUP_WRITE子项顺序误认为PLC扫描级原子提交保证。
	const std::size_t heartbeat_fields_begin = count;
	append(fast_write_handles_[kWriteHostSession], sizeof(host_session_id_), &host_session_id_);
	append(fast_write_handles_[kWriteHeartbeat], sizeof(heartbeat_sequence_), &heartbeat_sequence_);
	const bool recover = watchdog_recovery_pending_.exchange(false, std::memory_order_acq_rel);
	if (recover) append(fast_write_handles_[kWriteHostRecover], sizeof(recover), &recover);
	bool transport_succeeded = false;
	const bool write_attempted = fast_handles_valid_;
	const bool all_items_succeeded = write_attempted && ads_.ADSWriteSumByHandle(
		handles.data(),
		lengths.data(),
		values.data(),
		static_cast<unsigned long>(count),
		item_results.data(),
		&transport_succeeded);
	(void)all_items_succeeded;
	auto item_range_succeeded = [&](std::size_t begin, std::size_t end)
	{
		for (std::size_t index = begin; index < end; ++index)
		{
			if (item_results[index] != 0) return false;
		}
		return true;
	};

	// 核心拍包含常规输出、输出快照携带的清Req以及会话心跳。
	// 只有队列专用字段可以在子项失败时与核心通信健康解耦。
	bool core_write_ok = transport_succeeded &&
		item_range_succeeded(0, planned_fields_begin) &&
		item_range_succeeded(heartbeat_fields_begin, count);
	for (std::size_t axis_slot = 0; axis_slot < output_clear_item_index.size(); ++axis_slot)
	{
		const std::uint8_t axis_mask = axis_slot == 0 ? 0x01u : 0x02u;
		if ((output_clear_mask & axis_mask) == 0) continue;
		const int item_index = output_clear_item_index[axis_slot];
		if (item_index < 0 || item_results[static_cast<std::size_t>(item_index)] != 0)
		{
			core_write_ok = false;
		}
	}
	if (has_planned_return)
	{
		const bool command_items_ok = transport_succeeded && planned_fields_valid &&
			item_range_succeeded(planned_fields_begin, planned_fields_end);
		const bool command_success = core_write_ok && command_items_ok &&
			running_.load(std::memory_order_acquire);
		std::uint8_t possibly_started_mask = 0;
		if (planned_slot.command.operation == AdsPlannedReturnOperation::Commit &&
			planned_fields_valid)
		{
			if (write_attempted && !transport_succeeded)
			{
				// 请求已交给ADS库但没有完整响应，无法证明各腿Req未生效；保守禁止重发。
				possibly_started_mask = planned_commit_axis_mask;
			}
			else if (transport_succeeded)
			{
				for (std::size_t axis_slot = 0;
					axis_slot < planned_commit_req_item_index.size();
					++axis_slot)
				{
					const int item_index = planned_commit_req_item_index[axis_slot];
					if (item_index >= 0 &&
						item_results[static_cast<std::size_t>(item_index)] == 0)
					{
						possibly_started_mask |= axis_slot == 0 ? 0x01u : 0x02u;
					}
				}
			}
		}
		complete_planned_return_command(
			planned_slot.sequence,
			command_success,
			possibly_started_mask);
		planned_return_read_index_.store(planned_read_index + 1, std::memory_order_release);
	}
	if (!core_write_ok)
	{
		if (recover) watchdog_recovery_pending_.store(true, std::memory_order_release);
		return false;
	}
	if (has_output)
	{
		std::lock_guard<std::mutex> lock(output_mutex_);
		last_sent_output_ = output;
		has_last_sent_output_ = true;
	}
	if (output_generation != 0)
	{
		// 仅在包含该命令语义的 Sum Write 成功后推进；发布更快时允许跳过中间代次。
		applied_output_generation_.store(output_generation, std::memory_order_release);
		if (has_output && output.motion_enabled)
		{
			// motion_enabled保证本次事务实际包含refer，不能由仅心跳/清旁路的代次冒充。
			applied_motion_output_generation_.store(output_generation, std::memory_order_release);
		}
	}
	return true;
}

void AdsCommunicationService::complete_planned_return_command(
	std::uint64_t sequence,
	bool success,
	std::uint8_t possibly_started_mask)
{
	if (sequence == 0) return;
	PlannedReturnResultSlot& result =
		planned_return_results_[sequence % kPlannedReturnResultCapacity];
	if (result.sequence.load(std::memory_order_acquire) != sequence) return;
	result.possibly_started_mask.store(possibly_started_mask, std::memory_order_release);
	result.state.store(
		static_cast<unsigned char>(success
			? PlannedReturnResultState::Succeeded
			: PlannedReturnResultState::Failed),
		std::memory_order_release);
}

void AdsCommunicationService::fail_queued_planned_return_commands()
{
	std::uint64_t read_index =
		planned_return_read_index_.load(std::memory_order_relaxed);
	const std::uint64_t write_index =
		planned_return_write_index_.load(std::memory_order_acquire);
	while (read_index != write_index)
	{
		const PlannedReturnQueueSlot& slot =
			planned_return_queue_[read_index % kPlannedReturnQueueCapacity];
		complete_planned_return_command(slot.sequence, false);
		++read_index;
		planned_return_read_index_.store(read_index, std::memory_order_release);
	}
}

bool AdsCommunicationService::register_notifications()
{
	unregister_notifications();
	struct Registration
	{
		const char* symbol;
		unsigned long size;
		std::uint32_t id;
	};
	const Registration registrations[] = {
		{ AdsSymbol::self_check_done, sizeof(bool), kNotifySelfCheckDone },
		{ AdsSymbol::handle_reinit_req, sizeof(bool), kNotifyHandleReinitReq },
		{ "G.handle_reinit_done", sizeof(bool), kNotifyHandleReinitDone },
		{ AdsSymbol::estop_hold_req, sizeof(bool), kNotifyEstopHold },
		{ "G.host_comm_timeout", sizeof(bool), kNotifyHostTimeout },
		{ AdsSymbol::startup_loading_ready, sizeof(bool), kNotifyStartupReady },
		{ AdsSymbol::axis4_manual_busy, sizeof(bool), kNotifyAxis4Busy },
		{ "G.axis4_manual_done", sizeof(bool), kNotifyAxis4Done },
		{ AdsSymbol::axis4_manual_error, sizeof(bool), kNotifyAxis4Error },
		{ AdsSymbol::axis4_manual_error_id, sizeof(std::uint32_t), kNotifyAxis4ErrorId },
		{ AdsSymbol::gen_state, sizeof(int), kNotifyGenState },
		{ AdsSymbol::axis1_return.busy, sizeof(bool), kNotifyAxis1ReturnBusy },
		{ AdsSymbol::axis1_return.done, sizeof(bool), kNotifyAxis1ReturnDone },
		{ AdsSymbol::axis1_return.error, sizeof(bool), kNotifyAxis1ReturnError },
		{ AdsSymbol::axis1_return.error_id, sizeof(std::uint32_t), kNotifyAxis1ReturnErrorId },
		{ AdsSymbol::axis6_return.busy, sizeof(bool), kNotifyAxis6ReturnBusy },
		{ AdsSymbol::axis6_return.done, sizeof(bool), kNotifyAxis6ReturnDone },
		{ AdsSymbol::axis6_return.error, sizeof(bool), kNotifyAxis6ReturnError },
		{ AdsSymbol::axis6_return.error_id, sizeof(std::uint32_t), kNotifyAxis6ReturnErrorId }
	};
	try
	{
		notification_registrations_.reserve(_countof(registrations));
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}

	for (const auto& registration : registrations)
	{
		std::uint32_t registration_id = 0;
		if (!reserve_notification_registration(this, registration.id, registration_id))
		{
			unregister_notifications();
			return false;
		}
		notification_registrations_.push_back(NotificationRegistration{ registration_id, 0 });
		unsigned long handle = 0;
		if (!ads_.ADSAddNotification(
			registration.symbol,
			registration.size,
			&ads_notification_callback,
			registration_id,
			&handle,
			ADSTRANS_SERVERONCHA,
			10000,
			0))
		{
			unregister_notifications();
			return false;
		}
		notification_registrations_.back().notification_handle = handle;
	}
	return true;
}

void AdsCommunicationService::unregister_notifications()
{
	// 先从注册表移除并等待正在执行的回调退出，再调用 ADS 注销接口。
	{
		std::lock_guard<std::mutex> lock(g_notification_registry_mutex);
		for (const auto& registration : notification_registrations_)
		{
			const auto it = g_notification_registry.find(registration.registration_id);
			if (it != g_notification_registry.end() && it->second.service == this)
			{
				g_notification_registry.erase(it);
			}
		}
	}
	for (const auto& registration : notification_registrations_)
	{
		if (registration.notification_handle != 0)
		{
			(void)ads_.ADSDeleteNotification(registration.notification_handle);
		}
	}
	notification_registrations_.clear();
}

void AdsCommunicationService::clear_runtime_connection_state()
{
	// 断线/重连不能让旧回退事务在新连接上继续执行。
	fail_queued_planned_return_commands();
	unregister_notifications();
	clear_fast_handles();
	{
		std::lock_guard<std::mutex> lock(output_mutex_);
		desired_output_ = AdsOutputCommand{};
		has_desired_output_ = false;
		has_last_sent_output_ = false;
		desired_output_generation_ = 0;
	}
	{
		std::lock_guard<std::mutex> lock(coordinate_mutex_);
		coordinate_cache_valid_ = false;
	}
	{
		std::lock_guard<std::mutex> lock(event_mutex_);
		event_state_ = AdsEventState{};
		// 断连时安全状态未知，按保持/超时有效处理，避免其他消费者误判为安全解除。
		event_state_.estop_hold_req = true;
		event_state_.host_comm_timeout = true;
		++axis1_return_event_sequence_counter_;
		if (axis1_return_event_sequence_counter_ == 0) ++axis1_return_event_sequence_counter_;
		++axis6_return_event_sequence_counter_;
		if (axis6_return_event_sequence_counter_ == 0) ++axis6_return_event_sequence_counter_;
		LARGE_INTEGER event_now{};
		QueryPerformanceCounter(&event_now);
		event_state_.axis1_return_event_sequence = axis1_return_event_sequence_counter_;
		event_state_.axis1_return_event_qpc_ticks = event_now.QuadPart;
		event_state_.axis6_return_event_sequence = axis6_return_event_sequence_counter_;
		event_state_.axis6_return_event_qpc_ticks = event_now.QuadPart;
		notification_update_mask_ = 0;
	}
}

void AdsCommunicationService::mark_plc_restart(bool reconnect_required)
{
	const bool first_detection = !plc_restart_active_;
	plc_restart_active_ = true;
	restart_reconnect_pending_ = restart_reconnect_pending_ || reconnect_required;
	{
		std::lock_guard<std::mutex> lock(coordinate_mutex_);
		coordinate_cache_valid_ = false;
	}
	coordinate_refresh_pending_.store(true, std::memory_order_release);
	{
		std::lock_guard<std::mutex> lock(stats_mutex_);
		if (first_detection) ++stats_.plc_restart_count;
		stats_.state = AdsConnectionState::PlcRestarted;
	}
}

void AdsCommunicationService::process_one_low_frequency_request()
{
	std::shared_ptr<LowFrequencyRequest> request;
	{
		std::lock_guard<std::mutex> queue_lock(low_frequency_mutex_);
		while (!low_frequency_requests_.empty())
		{
			request = low_frequency_requests_.front();
			low_frequency_requests_.pop_front();
			std::lock_guard<std::mutex> state_lock(request->mutex);
			if (request->cancelled || request->completed)
			{
				request.reset();
				continue;
			}
			request->started = true;
			break;
		}
	}
	if (request == nullptr) return;
	request->cv.notify_all();

	bool success = false;
	if (running_.load(std::memory_order_acquire) && ads_.IsCommOpen())
	{
		try
		{
			switch (request->operation)
			{
			case LowFrequencyOperation::Read:
				success = ads_.ADSRead(
					request->symbols[0].c_str(),
					request->lengths[0],
					request->buffers[0].data());
				break;
			case LowFrequencyOperation::Write:
				success = ads_.ADSWrite(
					request->symbols[0].c_str(),
					request->lengths[0],
					request->buffers[0].data());
				break;
			case LowFrequencyOperation::ReadSum:
			{
				std::vector<const char*> symbols(request->symbols.size());
				std::vector<void*> outputs(request->buffers.size());
				for (std::size_t i = 0; i < request->symbols.size(); ++i)
				{
					symbols[i] = request->symbols[i].c_str();
					outputs[i] = request->buffers[i].data();
				}
				success = ads_.ADSReadSum(
					symbols.data(),
					request->lengths.data(),
					outputs.data(),
					static_cast<unsigned long>(symbols.size()));
				break;
			}
			case LowFrequencyOperation::WriteSum:
			{
				std::vector<const char*> symbols(request->symbols.size());
				std::vector<const void*> inputs(request->buffers.size());
				for (std::size_t i = 0; i < request->symbols.size(); ++i)
				{
					symbols[i] = request->symbols[i].c_str();
					inputs[i] = request->buffers[i].data();
				}
				success = ads_.ADSWriteSum(
					symbols.data(),
					request->lengths.data(),
					inputs.data(),
					static_cast<unsigned long>(symbols.size()));
				break;
			}
			}
		}
		catch (const std::bad_alloc&)
		{
			success = false;
		}
	}
	if (!running_.load(std::memory_order_acquire)) success = false;
	{
		std::lock_guard<std::mutex> state_lock(request->mutex);
		request->success = success;
		request->completed = true;
	}
	request->cv.notify_all();
}

void AdsCommunicationService::fail_queued_low_frequency_requests()
{
	std::deque<std::shared_ptr<LowFrequencyRequest>> pending;
	{
		std::lock_guard<std::mutex> queue_lock(low_frequency_mutex_);
		pending.swap(low_frequency_requests_);
	}
	for (const auto& request : pending)
	{
		{
			std::lock_guard<std::mutex> state_lock(request->mutex);
			if (request->started || request->completed) continue;
			request->success = false;
			request->completed = true;
		}
		request->cv.notify_all();
	}
}

void AdsCommunicationService::publish_snapshot(const AdsFastSnapshot& snapshot)
{
	bool dropped = false;
	{
		std::lock_guard<std::mutex> lock(snapshot_mutex_);
		latest_snapshot_ = snapshot;
		has_snapshot_ = true;
		if (snapshot_queue_.size() >= kSnapshotQueueCapacity)
		{
			snapshot_queue_.pop_front();
			dropped = true;
		}
		snapshot_queue_.push_back(snapshot);
	}
	if (dropped)
	{
		std::lock_guard<std::mutex> lock(stats_mutex_);
		++stats_.snapshot_queue_dropped;
	}
	if (snapshot.position_valid) latest_valid_qpc_.store(snapshot.qpc_ticks, std::memory_order_release);
	snapshot_cv_.notify_all();
}

void AdsCommunicationService::set_connection_state(AdsConnectionState state)
{
	std::lock_guard<std::mutex> lock(stats_mutex_);
	stats_.state = state;
}

void AdsCommunicationService::update_rate(
	std::int64_t now_qpc,
	std::uint64_t rtt_us,
	bool success)
{
	std::lock_guard<std::mutex> lock(stats_mutex_);
	if (rate_window_qpc_ == 0) rate_window_qpc_ = now_qpc;
	if (success)
	{
		++rate_window_successes_;
		stats_.consecutive_failures = 0;
	}
	else
	{
		++stats_.failed_cycles;
		++stats_.consecutive_failures;
		stats_.max_consecutive_failures = (std::max)(
			stats_.max_consecutive_failures, stats_.consecutive_failures);
	}
	if (now_qpc - rate_window_qpc_ >= qpc_frequency_)
	{
		stats_.actual_hz = static_cast<double>(rate_window_successes_) *
			static_cast<double>(qpc_frequency_) /
			static_cast<double>(now_qpc - rate_window_qpc_);
		rate_window_qpc_ = now_qpc;
		rate_window_successes_ = 0;
	}
	stats_.latest_rtt_us = rtt_us;
}

void AdsCommunicationService::wait_until(std::int64_t qpc_deadline)
{
	for (;;)
	{
		if (!running_.load(std::memory_order_acquire)) return;
		LARGE_INTEGER now{};
		QueryPerformanceCounter(&now);
		const std::int64_t remaining = qpc_deadline - now.QuadPart;
		if (remaining <= 0) return;
		const DWORD wait_ms = static_cast<DWORD>(remaining * 1000 / qpc_frequency_);
		if (wait_ms > 1)
		{
			if (WaitForSingleObject(stop_event_, wait_ms - 1) == WAIT_OBJECT_0) return;
		}
		else
		{
			SwitchToThread();
		}
	}
}
