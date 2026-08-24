#include "DualClampController.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>

DualClampController::DualClampController()
{
	open_ads();
}

bool DualClampController::open_ads()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (ads_.is_open()) return true;
	if (!ads_.open())
	{
		last_error_ = "ADS连接失败：" + ads_.last_error();
		return false;
	}
	if (stream_ads_.is_open()) stream_ads_.invalidate_zero();
	last_error_.clear();
	return true;
}

void DualClampController::close_ads()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (recorder_.active())
	{
		ads_.request_abort();
		std::string finalize_error;
		const char* status = phase_ == DualClampPhase::Completed ? "Completed" : "Error";
		if (!recorder_.finalize(status, "ADS连接已断开", stream_status_.zero, finalize_error)) last_error_ = finalize_error;
		started_ = false;
	}
	ads_.close();
	if (stream_ads_.is_open()) stream_ads_.invalidate_zero();
	stream_ads_.close();
	stream_status_.zero = {};
}

bool DualClampController::is_ads_open() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return ads_.is_open();
}

bool DualClampController::prepare(const DualClampConfig& config)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (recorder_.active())
	{
		std::string close_error;
		if (!recorder_.finalize("Aborted", "重新准备定位，结束上一条记录", stream_status_.zero, close_error))
		{
			last_error_ = close_error;
			return false;
		}
	}
	if (config.moving_axis != 1 && config.moving_axis != 6)
	{
		last_error_ = "运动端必须是轴1或轴6";
		return false;
	}
	if (config.axis1_distance_from_left_mm < 0.0 || config.axis6_distance_from_left_mm < 0.0)
	{
		last_error_ = "距左限位距离不能为负数";
		return false;
	}
	if (config.return_retract_distance_mm < 0.0 || config.return_velocity_mm_s <= 0.0
		|| config.return_acceleration_mm_s2 <= 0.0 || config.return_deceleration_mm_s2 <= 0.0
		|| config.return_jerk_mm_s3 <= 0.0)
	{
		last_error_ = "回程距离和运动参数必须为非负/正数";
		return false;
	}
	if (!live_.selfcheck_done || !live_.leftlimit_valid)
	{
		last_error_ = "尚未完成左限位自检";
		return false;
	}
	config_ = config;
	if (!stream_ads_.is_open() && !stream_ads_.open())
	{
		last_error_ = "实时记录ADS连接失败：" + stream_ads_.last_error();
		return false;
	}
	if (!stream_ads_.reset_recording())
	{
		last_error_ = "清空实时记录缓冲失败：" + stream_ads_.last_error();
		return false;
	}
	stream_ads_.invalidate_zero();
	stream_status_ = {};
	expected_block_sequence_ = 0;
	expected_sample_index_ = 0;
	zero_file_written_ = false;
	if (!ads_.clear_sample_buffer())
	{
		last_error_ = "清空PLC采样缓冲失败：" + ads_.last_error();
		return false;
	}
	if (!ads_.write_experiment_config(config_, true, false))
	{
		last_error_ = "下发准备定位请求失败：" + ads_.last_error();
		return false;
	}
	last_error_.clear();
	return true;
}

bool DualClampController::start(const DualClampConfig& config)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (config.moving_axis != 1 && config.moving_axis != 6)
	{
		last_error_ = "运动端必须是轴1或轴6";
		return false;
	}
	if (!live_.selfcheck_done || !live_.leftlimit_valid)
	{
		last_error_ = "尚未完成左限位自检";
		return false;
	}
	if (!live_.setup_done || phase_ != DualClampPhase::ReadyForClamp)
	{
		last_error_ = "尚未完成准备定位";
		return false;
	}
	if (!ads_.is_open() && !ads_.open())
	{
		last_error_ = "ADS连接失败：" + ads_.last_error();
		return false;
	}
	config_ = config;
	if (!stream_status_.zero.valid)
	{
		last_error_ = "请先点击力感取零点";
		return false;
	}
	if (!recorder_.active())
	{
		std::string record_error;
		if (!recorder_.begin("legacy", config_.record_suffix, record_error))
		{
			last_error_ = "创建实时记录目录失败：" + record_error;
			return false;
		}
	}
	abort_reason_.clear();
	last_error_.clear();
	started_ = true;
	if (!ads_.write_experiment_config(config_, false, true))
	{
		last_error_ = "下发开始实验请求失败：" + ads_.last_error();
		started_ = false;
		return false;
	}
	return true;
}

void DualClampController::abort(const std::string& reason)
{
	std::lock_guard<std::mutex> lock(mutex_);
	abort_reason_ = reason;
	ads_.request_abort();
	started_ = false;
}

void DualClampController::tick(double dt_s)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (ads_.is_open())
	{
		DualClampLiveFrame live{};
		if (ads_.read_live(live))
		{
			live_ = live;
			live_.host_qpc = 0;
		}
		else if (started_ || selfcheck_requested_)
		{
			last_error_ = "ADS读取实时数据失败：" + ads_.last_error();
			phase_ = DualClampPhase::Error;
			ads_.request_abort();
			if (recorder_.active())
			{
				std::string finalize_error;
				recorder_.finalize("Error", last_error_, stream_status_.zero, finalize_error);
			}
			started_ = false;
			return;
		}
	}
	poll_stream_locked();
	phase_ = static_cast<DualClampPhase>(live_.plc_phase);
	if (live_.selfcheck_done) selfcheck_requested_ = false;
	if (phase_ == DualClampPhase::Completed || phase_ == DualClampPhase::Aborted || phase_ == DualClampPhase::Error)
	{
		started_ = phase_ == DualClampPhase::Completed;
	}
	if (live_.selfcheck_error || live_.status_error_id != 0)
	{
		last_error_ = "PLC错误ID：" + std::to_string(live_.status_error_id);
	}
	live_.recording = stream_status_.recording;
	live_.recording_overflow = stream_status_.overflow;
	live_.recording_sample_count = stream_status_.total_count;
	live_.recording_error_id = stream_status_.error_id;
	live_.zero_busy = stream_status_.zero.busy;
	live_.zero_done = stream_status_.zero.done;
	live_.zero_values = stream_status_.zero.value;
	if (stream_status_.zero.error_id != 0) last_error_ = "力感取零失败，错误ID：" + std::to_string(stream_status_.zero.error_id);
}

void DualClampController::poll_stream_locked()
{
	if (!stream_ads_.is_open() && !stream_ads_.open()) return;
	if (recorder_.failed())
	{
		last_error_ = recorder_.last_error().empty() ? "实时记录写盘失败" : recorder_.last_error();
		ads_.request_abort();
		started_ = false;
		return;
	}
	if (!stream_ads_.read_status(stream_status_))
	{
		if (started_)
		{
			last_error_ = "实时记录状态读取失败：" + stream_ads_.last_error();
			stream_status_.error_id = 0x7403;
			ads_.request_abort();
			started_ = false;
		}
		return;
	}
	if (stream_status_.zero.done && !zero_file_written_ && recorder_.active())
	{
		std::vector<std::array<double, 4>> zero_samples;
		std::string error;
		if (stream_ads_.read_zero_samples(zero_samples))
		{
			bool ok = true;
			for (std::size_t i = 0; i < zero_samples.size(); ++i) ok = recorder_.append_zero(zero_samples[i], static_cast<std::uint64_t>(i) * 1000, error) && ok;
			ok = recorder_.finish_zero(stream_status_.zero, error) && ok;
			if (!ok)
			{
				last_error_ = error;
				ads_.request_abort();
				started_ = false;
				return;
			}
			zero_file_written_ = true;
		}
		else
		{
			last_error_ = "读取取零样本失败：" + stream_ads_.last_error();
		}
	}
	if (stream_status_.overflow)
	{
		last_error_ = "实时记录缓冲区溢出，实验已中止";
		ads_.request_abort();
		started_ = false;
		return;
	}
	if (!recorder_.active() || stream_status_.source_mode != 0) return;
	for (int pass = 0; pass < 2; ++pass)
	{
		int slot = -1;
		for (int candidate = 0; candidate < 2; ++candidate)
		{
			if (stream_status_.block_ready[candidate] && stream_status_.block_sequence[candidate] == expected_block_sequence_)
			{
				slot = candidate;
				break;
			}
		}
		if (slot < 0)
		{
			bool any_ready = stream_status_.block_ready[0] || stream_status_.block_ready[1];
			if (any_ready)
			{
				last_error_ = "实时记录分块序号不连续";
				ads_.request_abort();
				started_ = false;
			}
			break;
		}
		std::vector<ExperimentStreamSample> raw;
		std::uint32_t sequence = 0;
		if (!stream_ads_.read_block(slot, raw, sequence))
		{
			last_error_ = "实时记录分块读取失败：" + stream_ads_.last_error();
			ads_.request_abort();
			started_ = false;
			return;
		}
		if (sequence != expected_block_sequence_)
		{
			last_error_ = "实时记录分块序号不连续";
			ads_.request_abort();
			started_ = false;
			return;
		}
		std::vector<DualClampSample> converted;
		converted.reserve(raw.size());
		for (const auto& r : raw)
		{
			if (r.index != expected_sample_index_)
			{
				last_error_ = "实时记录样本序号不连续";
				ads_.request_abort();
				started_ = false;
				return;
			}
			++expected_sample_index_;
			DualClampSample s{};
			s.sample_index = r.index; s.plc_time_us = r.time_us; s.phase = r.phase; s.event_sequence = r.event_sequence;
			s.axis1_pos_abs_mm = r.axis1_pos; s.axis1_velocity_mm_s = r.axis1_vel; s.axis1_acceleration_mm_s2 = r.axis1_acc;
			s.axis6_pos_abs_mm = r.axis6_pos; s.axis6_velocity_mm_s = r.axis6_vel; s.axis6_acceleration_mm_s2 = r.axis6_acc;
			s.ft_1_raw = r.ft1; s.fn_1_raw = r.fn1; s.ft_2_raw = r.ft2; s.fn_2_raw = r.fn2;
			s.cylinder2_cmd = r.cylinder2; s.cylinder4_cmd = r.cylinder4;
			converted.push_back(s);
		}
		std::string error;
		if (!recorder_.append_dual(converted, 0, stream_status_.zero, error))
		{
			last_error_ = error;
			ads_.request_abort();
			started_ = false;
			return;
		}
		if (!stream_ads_.acknowledge_block(slot, sequence))
		{
			last_error_ = "实时记录分块确认失败：" + stream_ads_.last_error();
			ads_.request_abort();
			started_ = false;
			return;
		}
		++expected_block_sequence_;
		stream_status_.block_ready[slot] = false;
	}
}

bool DualClampController::request_zero()
{
	std::lock_guard<std::mutex> lock(mutex_);
	poll_stream_locked();
	if (!live_.selfcheck_done || !live_.setup_done || started_)
	{
		last_error_ = "取零点要求自检和准备定位完成，且实验未开始";
		return false;
	}
	if (!stream_ads_.is_open() && !stream_ads_.open())
	{
		last_error_ = "实时记录ADS连接失败：" + stream_ads_.last_error();
		return false;
	}
	if (!recorder_.active())
	{
		std::string error;
		if (!recorder_.begin("legacy", config_.record_suffix, error)) { last_error_ = error; return false; }
	}
	zero_file_written_ = false;
	{
		std::string reset_error;
		if (!recorder_.reset_zero_file(reset_error)) { last_error_ = reset_error; return false; }
	}
	{
		std::string event_error;
		if (!recorder_.begin_zero(event_error)) { last_error_ = event_error; return false; }
	}
	return stream_ads_.request_zero();
}

bool DualClampController::set_record_suffix(const std::string& suffix)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (started_ || (recorder_.active() && phase_ != DualClampPhase::Completed && phase_ != DualClampPhase::Aborted && phase_ != DualClampPhase::Error))
	{
		last_error_ = "实验进行中不能修改保存名称";
		return false;
	}
	config_.record_suffix = suffix;
	return true;
}

void DualClampController::invalidate_zero()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (stream_ads_.is_open()) stream_ads_.invalidate_zero();
	stream_status_.zero = {};
}

ForceZeroState DualClampController::zero_state() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return stream_status_.zero;
}

std::string DualClampController::recording_directory() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return recorder_.directory();
}

bool DualClampController::recording_archived() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return recorder_.archived();
}

bool DualClampController::write_metadata(const std::string& directory, std::string& error) const
{
	std::ofstream out(std::filesystem::path(directory) / "experiment.json", std::ios::binary);
	if (!out)
	{
		error = "无法创建experiment.json";
		return false;
	}
	out << "{\n"
		<< "  \"moving_axis\": " << config_.moving_axis << ",\n"
		<< "  \"fixed_axis\": " << (config_.moving_axis == 1 ? 6 : 1) << ",\n"
		<< "  \"axis1_distance_from_left_mm\": " << config_.axis1_distance_from_left_mm << ",\n"
		<< "  \"axis6_distance_from_left_mm\": " << config_.axis6_distance_from_left_mm << ",\n"
		<< "  \"axis2_angle_abs_deg\": " << config_.axis2_angle_abs_deg << ",\n"
		<< "  \"axis7_angle_abs_deg\": " << config_.axis7_angle_abs_deg << ",\n"
		<< "  \"return_retract_distance_mm\": " << config_.return_retract_distance_mm << ",\n"
		<< "  \"return_velocity_mm_s\": " << config_.return_velocity_mm_s << ",\n"
		<< "  \"return_acceleration_mm_s2\": " << config_.return_acceleration_mm_s2 << ",\n"
		<< "  \"return_deceleration_mm_s2\": " << config_.return_deceleration_mm_s2 << ",\n"
		<< "  \"cylinder1_open_word\": " << config_.cylinder1_open_word << ",\n"
		<< "  \"cylinder3_open_word\": " << config_.cylinder3_open_word << ",\n"
		<< "  \"cylinder2_clamp_word\": " << config_.clamp_axis1_word << ",\n"
		<< "  \"cylinder4_clamp_word\": " << config_.clamp_axis6_word << ",\n"
		<< "  \"recovery_mode\": \"" << (config_.recovery_mode == DualClampRecoveryMode::Hold ? "hold" : "move") << "\",\n"
		<< "  \"note\": \"" << config_.note << "\"\n"
		<< "}\n";
	return true;
}

bool DualClampController::write_csv(const std::string& directory, const std::vector<DualClampSample>& samples, std::string& error) const
{
	std::ofstream out(std::filesystem::path(directory) / "samples_1khz.csv", std::ios::binary);
	if (!out)
	{
		error = "无法创建samples_1khz.csv";
		return false;
	}
	out << "sample_index,plc_time_us,phase,event_sequence,axis1_pos_abs_mm,axis1_velocity_mm_s,axis1_acceleration_mm_s2,axis6_pos_abs_mm,axis6_velocity_mm_s,axis6_acceleration_mm_s2,ft_1_raw,fn_1_raw,ft_2_raw,fn_2_raw,cylinder2_cmd,cylinder4_cmd\n";
	out << std::setprecision(12);
	for (const DualClampSample& s : samples)
	{
		out << s.sample_index << ',' << s.plc_time_us << ',' << static_cast<unsigned int>(s.phase) << ',' << s.event_sequence << ','
			<< s.axis1_pos_abs_mm << ',' << s.axis1_velocity_mm_s << ',' << s.axis1_acceleration_mm_s2 << ','
			<< s.axis6_pos_abs_mm << ',' << s.axis6_velocity_mm_s << ',' << s.axis6_acceleration_mm_s2 << ','
			<< s.ft_1_raw << ',' << s.fn_1_raw << ',' << s.ft_2_raw << ',' << s.fn_2_raw << ','
			<< s.cylinder2_cmd << ',' << s.cylinder4_cmd << '\n';
	}
	return true;
}

bool DualClampController::save_samples(const std::string& directory, std::string& error)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (recorder_.active())
	{
		poll_stream_locked();
		DualClampLiveFrame latest{};
		if (ads_.is_open() && ads_.read_live(latest))
		{
			live_ = latest;
			phase_ = static_cast<DualClampPhase>(live_.plc_phase);
		}
		if (phase_ != DualClampPhase::Completed && phase_ != DualClampPhase::Aborted && phase_ != DualClampPhase::Error)
		{
			error = "实验正在进行，数据已经实时保存";
			return false;
		}
		return recorder_.finalize(phase_ == DualClampPhase::Completed ? "Completed" : phase_ == DualClampPhase::Aborted ? "Aborted" : "Error", last_error_, stream_status_.zero, error);
	}
	if (directory.empty() && !recorder_.directory().empty()) return true;
	if (!started_ || phase_ != DualClampPhase::Completed)
	{
		error = "实验尚未完成";
		return false;
	}
	std::filesystem::create_directories(directory);
	std::uint32_t count = 0;
	bool overflow = false;
	if (!ads_.read_sample_count(count, overflow) || overflow)
	{
		error = overflow ? "PLC采样缓冲区溢出" : "读取PLC采样数量失败：" + ads_.last_error();
		return false;
	}
	std::vector<DualClampSample> samples;
	if (!ads_.read_all_samples(count, samples))
	{
		error = "读取PLC采样数据失败：" + ads_.last_error();
		return false;
	}
	if (!write_metadata(directory, error) || !write_csv(directory, samples, error) || !write_events_csv(directory, samples, error)) return false;
	return true;
}

bool DualClampController::write_events_csv(const std::string& directory, const std::vector<DualClampSample>& samples, std::string& error) const
{
	std::ofstream out(std::filesystem::path(directory) / "events.csv", std::ios::binary);
	if (!out)
	{
		error = "无法创建events.csv";
		return false;
	}
	out << "event_sequence,plc_time_us,phase\n";
	if (samples.empty()) return true;
	std::uint32_t previous = samples.front().event_sequence;
	out << previous << ',' << samples.front().plc_time_us << ',' << static_cast<unsigned int>(samples.front().phase) << '\n';
	for (std::size_t i = 1; i < samples.size(); ++i)
	{
		if (samples[i].event_sequence == previous) continue;
		previous = samples[i].event_sequence;
		out << previous << ',' << samples[i].plc_time_us << ',' << static_cast<unsigned int>(samples[i].phase) << '\n';
	}
	return true;
}

DualClampPhase DualClampController::phase() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return phase_;
}

std::string DualClampController::last_error() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return last_error_;
}

DualClampConfig DualClampController::config() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return config_;
}

DualClampLiveFrame DualClampController::live() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return live_;
}

std::uint32_t DualClampController::event_sequence() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return event_sequence_;
}
