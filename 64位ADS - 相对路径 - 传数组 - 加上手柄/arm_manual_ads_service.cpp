#include "arm_manual_ads_service.h"

#include "ads_communication.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>

namespace
{
	constexpr int kArmAxisCount = 5;
	constexpr int kReadPeriodMs = 50;
	constexpr int kJogLeaseMs = 300;
	constexpr DWORD kAdsTimeoutMs = 250;

	constexpr const char* kManualEnable = "G.arm_manual_enable";
	constexpr const char* kEnableReq = "G.arm_enable_req";
	constexpr const char* kResetReq = "G.arm_reset_req";
	constexpr const char* kJogPosReq = "G.arm_jog_pos_req";
	constexpr const char* kJogNegReq = "G.arm_jog_neg_req";
	constexpr const char* kJogVelocity = "G.arm_jog_velocity";
	constexpr const char* kJogAcc = "G.arm_jog_acc";
	constexpr const char* kJogDec = "G.arm_jog_dec";
	constexpr const char* kJogJerk = "G.arm_jog_jerk";
	constexpr const char* kActPos = "G.arm_act_pos";
	constexpr const char* kActVel = "G.arm_act_vel";
	constexpr const char* kMotionBusy = "G.arm_motion_busy";
	constexpr const char* kMotionDone = "G.arm_motion_done";
	constexpr const char* kMotionError = "G.arm_motion_error";
	constexpr const char* kMotionErrorId = "G.arm_motion_error_id";
	constexpr const char* kCmdDir = "G.arm_cmd_dir";
	constexpr const char* kCmdConflict = "G.arm_cmd_conflict";

	const char* power_symbol(int axis_one_based, const char* field)
	{
		static const char* const done[] = {
			"G.arm_power_output[1].Done", "G.arm_power_output[2].Done", "G.arm_power_output[3].Done",
			"G.arm_power_output[4].Done", "G.arm_power_output[5].Done" };
		static const char* const busy[] = {
			"G.arm_power_output[1].Busy", "G.arm_power_output[2].Busy", "G.arm_power_output[3].Busy",
			"G.arm_power_output[4].Busy", "G.arm_power_output[5].Busy" };
		static const char* const active[] = {
			"G.arm_power_output[1].Active", "G.arm_power_output[2].Active", "G.arm_power_output[3].Active",
			"G.arm_power_output[4].Active", "G.arm_power_output[5].Active" };
		static const char* const error[] = {
			"G.arm_power_output[1].Error", "G.arm_power_output[2].Error", "G.arm_power_output[3].Error",
			"G.arm_power_output[4].Error", "G.arm_power_output[5].Error" };
		static const char* const error_id[] = {
			"G.arm_power_output[1].ErrorID", "G.arm_power_output[2].ErrorID", "G.arm_power_output[3].ErrorID",
			"G.arm_power_output[4].ErrorID", "G.arm_power_output[5].ErrorID" };
		const int index = axis_one_based - 1;
		if (std::strcmp(field, "Done") == 0) return done[index];
		if (std::strcmp(field, "Busy") == 0) return busy[index];
		if (std::strcmp(field, "Active") == 0) return active[index];
		if (std::strcmp(field, "Error") == 0) return error[index];
		return error_id[index];
	}

	const char* reset_symbol(int axis_one_based, const char* field)
	{
		static const char* const done[] = {
			"G.arm_reset_output[1].Done", "G.arm_reset_output[2].Done", "G.arm_reset_output[3].Done",
			"G.arm_reset_output[4].Done", "G.arm_reset_output[5].Done" };
		static const char* const busy[] = {
			"G.arm_reset_output[1].Busy", "G.arm_reset_output[2].Busy", "G.arm_reset_output[3].Busy",
			"G.arm_reset_output[4].Busy", "G.arm_reset_output[5].Busy" };
		static const char* const active[] = {
			"G.arm_reset_output[1].Active", "G.arm_reset_output[2].Active", "G.arm_reset_output[3].Active",
			"G.arm_reset_output[4].Active", "G.arm_reset_output[5].Active" };
		static const char* const error[] = {
			"G.arm_reset_output[1].Error", "G.arm_reset_output[2].Error", "G.arm_reset_output[3].Error",
			"G.arm_reset_output[4].Error", "G.arm_reset_output[5].Error" };
		static const char* const error_id[] = {
			"G.arm_reset_output[1].ErrorID", "G.arm_reset_output[2].ErrorID", "G.arm_reset_output[3].ErrorID",
			"G.arm_reset_output[4].ErrorID", "G.arm_reset_output[5].ErrorID" };
		const int index = axis_one_based - 1;
		if (std::strcmp(field, "Done") == 0) return done[index];
		if (std::strcmp(field, "Busy") == 0) return busy[index];
		if (std::strcmp(field, "Active") == 0) return active[index];
		if (std::strcmp(field, "Error") == 0) return error[index];
		return error_id[index];
	}

	bool valid_axis(int axis_one_based)
	{
		return axis_one_based >= 1 && axis_one_based <= kArmAxisCount;
	}

	bool valid_parameter(double value)
	{
		return std::isfinite(value) && value >= 0.01 && value <= 10000.0;
	}
}

ArmManualAdsService::ArmManualAdsService(AdsCommunicationService& ads_service)
	: ads_service_(ads_service)
{
}

ArmManualAdsService::~ArmManualAdsService()
{
	stop();
}

bool ArmManualAdsService::start()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (running_) return true;
	stop_requested_ = false;
	try
	{
		worker_ = std::thread(&ArmManualAdsService::run, this);
		running_ = true;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

void ArmManualAdsService::stop()
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (!running_ && !worker_.joinable()) return;
		// 退出阶段禁止健康检测把总使能再次自动置位。
		user_disabled_manual_enable_ = true;
		clear_requests_locked(true);
		stop_requested_ = true;
	}
	wake_cv_.notify_all();
	if (worker_.joinable()) worker_.join();
	std::lock_guard<std::mutex> lock(mutex_);
	running_ = false;
}

ArmManualSnapshot ArmManualAdsService::snapshot() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return snapshot_;
}

void ArmManualAdsService::set_manual_enable(bool enabled)
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		user_disabled_manual_enable_ = !enabled;
		if (desired_manual_enable_ != enabled)
		{
			desired_manual_enable_ = enabled;
			manual_dirty_ = true;
		}
		if (!enabled) clear_requests_locked(false);
	}
	wake_cv_.notify_all();
}

void ArmManualAdsService::set_axis_enable(int axis_one_based, bool enabled)
{
	if (!valid_axis(axis_one_based)) return;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		const int index = axis_one_based - 1;
		if (desired_enable_req_[index] != enabled)
		{
			desired_enable_req_[index] = enabled;
			enable_dirty_ = true;
		}
		if (!enabled)
		{
			desired_jog_pos_req_[index] = false;
			desired_jog_neg_req_[index] = false;
			jog_pos_dirty_ = true;
			jog_neg_dirty_ = true;
		}
	}
	wake_cv_.notify_all();
}

void ArmManualAdsService::request_reset(int axis_one_based)
{
	if (!valid_axis(axis_one_based)) return;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		desired_reset_req_[axis_one_based - 1] = true;
		reset_dirty_ = true;
	}
	wake_cv_.notify_all();
}

void ArmManualAdsService::set_jog_direction(int axis_one_based, int direction)
{
	if (!valid_axis(axis_one_based) || direction < -1 || direction > 1) return;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		const int index = axis_one_based - 1;
		const bool pos = direction > 0;
		const bool neg = direction < 0;
		jog_deadline_[index] = direction == 0
			? std::chrono::steady_clock::time_point{}
			: std::chrono::steady_clock::now() + std::chrono::milliseconds(kJogLeaseMs);
		if (desired_jog_pos_req_[index] != pos)
		{
			desired_jog_pos_req_[index] = pos;
			jog_pos_dirty_ = true;
		}
		if (desired_jog_neg_req_[index] != neg)
		{
			desired_jog_neg_req_[index] = neg;
			jog_neg_dirty_ = true;
		}
	}
	wake_cv_.notify_all();
}

bool ArmManualAdsService::set_jog_parameter(int axis_one_based, int parameter_kind, double value)
{
	if (!valid_axis(axis_one_based) || parameter_kind < 0 || parameter_kind > 3 || !valid_parameter(value)) return false;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		const int index = axis_one_based - 1;
		switch (parameter_kind)
		{
		case 0: desired_jog_velocity_[index] = value; break;
		case 1: desired_jog_acc_[index] = value; break;
		case 2: desired_jog_dec_[index] = value; break;
		case 3: desired_jog_jerk_[index] = value; break;
		default: return false;
		}
		params_dirty_ = true;
	}
	wake_cv_.notify_all();
	return true;
}

void ArmManualAdsService::clear_requests_locked(bool clear_manual_enable)
{
	if (clear_manual_enable && desired_manual_enable_)
	{
		desired_manual_enable_ = false;
		manual_dirty_ = true;
	}
	if (desired_enable_req_ != std::array<bool, 5>{})
	{
		desired_enable_req_.fill(false);
		enable_dirty_ = true;
	}
	if (desired_jog_pos_req_ != std::array<bool, 5>{})
	{
		desired_jog_pos_req_.fill(false);
		jog_pos_dirty_ = true;
	}
	if (desired_jog_neg_req_ != std::array<bool, 5>{})
	{
		desired_jog_neg_req_.fill(false);
		jog_neg_dirty_ = true;
	}
	jog_deadline_.fill(std::chrono::steady_clock::time_point{});
	if (desired_reset_req_ != std::array<bool, 5>{})
	{
		desired_reset_req_.fill(false);
		reset_dirty_ = true;
	}
}

void ArmManualAdsService::update_auto_enable_gate()
{
	const AdsCommunicationStats stats = ads_service_.stats();
	const AdsEventState events = ads_service_.event_state();
	const bool healthy = stats.state == AdsConnectionState::Running && !events.host_comm_timeout;
	std::lock_guard<std::mutex> lock(mutex_);
	if (!healthy)
	{
		clear_requests_locked(true);
		return;
	}
	if (!user_disabled_manual_enable_ && !desired_manual_enable_)
	{
		desired_manual_enable_ = true;
		manual_dirty_ = true;
	}
}

void ArmManualAdsService::expire_jog_deadman()
{
	const auto now = std::chrono::steady_clock::now();
	std::lock_guard<std::mutex> lock(mutex_);
	for (int index = 0; index < kArmAxisCount; ++index)
	{
		if (jog_deadline_[index] == std::chrono::steady_clock::time_point{} ||
			now < jog_deadline_[index])
		{
			continue;
		}

		jog_deadline_[index] = std::chrono::steady_clock::time_point{};
		if (desired_jog_pos_req_[index])
		{
			desired_jog_pos_req_[index] = false;
			jog_pos_dirty_ = true;
		}
		if (desired_jog_neg_req_[index])
		{
			desired_jog_neg_req_[index] = false;
			jog_neg_dirty_ = true;
		}
	}
}

void ArmManualAdsService::run()
{
	auto next_read = std::chrono::steady_clock::now();
	for (;;)
	{
		update_auto_enable_gate();
		expire_jog_deadman();
		bool stopping = false;
		bool has_dirty = false;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			stopping = stop_requested_;
			has_dirty = manual_dirty_ || enable_dirty_ || reset_dirty_ || jog_pos_dirty_ || jog_neg_dirty_ || params_dirty_;
		}
		const bool write_ok = !has_dirty || write_dirty_request();
		if (stopping) break;
		if (!write_ok)
		{
			// ADS 不可用时保留脏请求供恢复后下发，但避免失败路径空转占用 CPU。
			std::unique_lock<std::mutex> lock(mutex_);
			wake_cv_.wait_for(lock, std::chrono::milliseconds(kReadPeriodMs), [this]()
			{
				return stop_requested_;
			});
			continue;
		}

		const auto now = std::chrono::steady_clock::now();
		if (now >= next_read)
		{
			(void)read_snapshot();
			next_read = now + std::chrono::milliseconds(kReadPeriodMs);
		}

		std::unique_lock<std::mutex> lock(mutex_);
		wake_cv_.wait_until(lock, next_read, [this]()
		{
			return stop_requested_ || manual_dirty_ || enable_dirty_ || reset_dirty_ || jog_pos_dirty_ || jog_neg_dirty_ || params_dirty_;
		});
	}
}

bool ArmManualAdsService::write_dirty_request()
{
	bool manual_enable = false;
	std::array<bool, 5> enable_req{};
	std::array<bool, 5> reset_req{};
	std::array<bool, 5> jog_pos_req{};
	std::array<bool, 5> jog_neg_req{};
	std::array<double, 5> velocity{};
	std::array<double, 5> acc{};
	std::array<double, 5> dec{};
	std::array<double, 5> jerk{};
	bool manual_dirty = false;
	bool enable_dirty = false;
	bool reset_dirty = false;
	bool jog_pos_dirty = false;
	bool jog_neg_dirty = false;
	bool params_dirty = false;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		manual_enable = desired_manual_enable_;
		enable_req = desired_enable_req_;
		reset_req = desired_reset_req_;
		jog_pos_req = desired_jog_pos_req_;
		jog_neg_req = desired_jog_neg_req_;
		velocity = desired_jog_velocity_;
		acc = desired_jog_acc_;
		dec = desired_jog_dec_;
		jerk = desired_jog_jerk_;
		manual_dirty = manual_dirty_;
		enable_dirty = enable_dirty_;
		reset_dirty = reset_dirty_;
		jog_pos_dirty = jog_pos_dirty_;
		jog_neg_dirty = jog_neg_dirty_;
		params_dirty = params_dirty_;
	}

	std::vector<const char*> symbols;
	std::vector<unsigned long> lengths;
	std::vector<const void*> inputs;
	auto append = [&](const char* symbol, unsigned long length, const void* value)
	{
		symbols.push_back(symbol);
		lengths.push_back(length);
		inputs.push_back(value);
	};
	if (manual_dirty) append(kManualEnable, sizeof(manual_enable), &manual_enable);
	if (enable_dirty) append(kEnableReq, sizeof(enable_req), enable_req.data());
	if (reset_dirty) append(kResetReq, sizeof(reset_req), reset_req.data());
	if (jog_pos_dirty) append(kJogPosReq, sizeof(jog_pos_req), jog_pos_req.data());
	if (jog_neg_dirty) append(kJogNegReq, sizeof(jog_neg_req), jog_neg_req.data());
	if (params_dirty)
	{
		append(kJogVelocity, sizeof(velocity), velocity.data());
		append(kJogAcc, sizeof(acc), acc.data());
		append(kJogDec, sizeof(dec), dec.data());
		append(kJogJerk, sizeof(jerk), jerk.data());
	}
	if (symbols.empty()) return true;
	const bool ok = ads_service_.write_sum(
		symbols.data(), lengths.data(), inputs.data(), static_cast<unsigned long>(symbols.size()), kAdsTimeoutMs);
	if (!ok) return false;

	std::lock_guard<std::mutex> lock(mutex_);
	if (manual_dirty && desired_manual_enable_ == manual_enable) manual_dirty_ = false;
	if (enable_dirty && desired_enable_req_ == enable_req) enable_dirty_ = false;
	if (reset_dirty && desired_reset_req_ == reset_req)
	{
		desired_reset_req_.fill(false);
		reset_dirty_ = false;
	}
	if (jog_pos_dirty && desired_jog_pos_req_ == jog_pos_req) jog_pos_dirty_ = false;
	if (jog_neg_dirty && desired_jog_neg_req_ == jog_neg_req) jog_neg_dirty_ = false;
	if (params_dirty && desired_jog_velocity_ == velocity && desired_jog_acc_ == acc &&
		desired_jog_dec_ == dec && desired_jog_jerk_ == jerk)
	{
		params_dirty_ = false;
	}
	return true;
}

bool ArmManualAdsService::read_snapshot()
{
	ArmManualSnapshot next{};
	std::vector<const char*> symbols;
	std::vector<unsigned long> lengths;
	std::vector<void*> outputs;
	auto append = [&](const char* symbol, unsigned long length, void* output)
	{
		symbols.push_back(symbol);
		lengths.push_back(length);
		outputs.push_back(output);
	};
	append(kManualEnable, sizeof(next.manual_enable), &next.manual_enable);
	append(kEnableReq, sizeof(next.enable_req), next.enable_req.data());
	append(kJogVelocity, sizeof(next.jog_velocity), next.jog_velocity.data());
	append(kJogAcc, sizeof(next.jog_acc), next.jog_acc.data());
	append(kJogDec, sizeof(next.jog_dec), next.jog_dec.data());
	append(kJogJerk, sizeof(next.jog_jerk), next.jog_jerk.data());
	append(kActPos, sizeof(next.act_pos), next.act_pos.data());
	append(kActVel, sizeof(next.act_vel), next.act_vel.data());
	append(kMotionBusy, sizeof(next.motion_busy), next.motion_busy.data());
	append(kMotionDone, sizeof(next.motion_done), next.motion_done.data());
	append(kMotionError, sizeof(next.motion_error), next.motion_error.data());
	append(kMotionErrorId, sizeof(next.motion_error_id), next.motion_error_id.data());
	append(kCmdDir, sizeof(next.cmd_dir), next.cmd_dir.data());
	append(kCmdConflict, sizeof(next.cmd_conflict), next.cmd_conflict.data());
	for (int axis = 1; axis <= kArmAxisCount; ++axis)
	{
		const int index = axis - 1;
		append(power_symbol(axis, "Done"), sizeof(next.power_done[index]), &next.power_done[index]);
		append(power_symbol(axis, "Busy"), sizeof(next.power_busy[index]), &next.power_busy[index]);
		append(power_symbol(axis, "Active"), sizeof(next.power_active[index]), &next.power_active[index]);
		append(power_symbol(axis, "Error"), sizeof(next.power_error[index]), &next.power_error[index]);
		append(power_symbol(axis, "ErrorID"), sizeof(next.power_error_id[index]), &next.power_error_id[index]);
		append(reset_symbol(axis, "Done"), sizeof(next.reset_done[index]), &next.reset_done[index]);
		append(reset_symbol(axis, "Busy"), sizeof(next.reset_busy[index]), &next.reset_busy[index]);
		append(reset_symbol(axis, "Active"), sizeof(next.reset_active[index]), &next.reset_active[index]);
		append(reset_symbol(axis, "Error"), sizeof(next.reset_error[index]), &next.reset_error[index]);
		append(reset_symbol(axis, "ErrorID"), sizeof(next.reset_error_id[index]), &next.reset_error_id[index]);
	}
	next.valid = ads_service_.read_sum(
		symbols.data(), lengths.data(), outputs.data(), static_cast<unsigned long>(symbols.size()), kAdsTimeoutMs);
	std::lock_guard<std::mutex> lock(mutex_);
	if (next.valid) snapshot_ = next;
	else snapshot_.valid = false;
	return next.valid;
}
