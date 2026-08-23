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
	last_error_.clear();
	return true;
}

void DualClampController::close_ads()
{
	std::lock_guard<std::mutex> lock(mutex_);
	ads_.close();
}

bool DualClampController::is_ads_open() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return ads_.is_open();
}

bool DualClampController::self_check()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!ads_.is_open() && !ads_.open())
	{
		last_error_ = "ADS连接失败：" + ads_.last_error();
		return false;
	}
	if (!ads_.request_self_check())
	{
		last_error_ = "下发自检请求失败：" + ads_.last_error();
		return false;
	}
	selfcheck_requested_ = true;
	started_ = false;
	last_error_.clear();
	return true;
}

bool DualClampController::prepare(const DualClampConfig& config)
{
	std::lock_guard<std::mutex> lock(mutex_);
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
			return;
		}
	}
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
		<< "  \"instrument\": \"" << (config_.instrument == DualClampInstrument::Guidewire ? "guidewire" : "catheter") << "\",\n"
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
