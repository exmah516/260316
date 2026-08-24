#include "ProgrammedDeliveryController.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>

ProgrammedDeliveryController::ProgrammedDeliveryController()
{
	open_ads();
}

bool ProgrammedDeliveryController::open_ads()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (ads_.is_open()) return true;
	if (!ads_.open())
	{
		last_error_ = "程序递送ADS连接失败：" + ads_.last_error();
		return false;
	}
	last_error_.clear();
	return true;
}

void ProgrammedDeliveryController::close_ads()
{
	std::lock_guard<std::mutex> lock(mutex_);
	ads_.close();
}

bool ProgrammedDeliveryController::is_ads_open() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return ads_.is_open();
}

bool ProgrammedDeliveryController::select_mode(ProgrammedDeliveryMode mode)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (mode != ProgrammedDeliveryMode::Legacy && mode != ProgrammedDeliveryMode::Catheter && mode != ProgrammedDeliveryMode::Guidewire)
	{
		last_error_ = "程序递送模式编号无效";
		return false;
	}
	if (live_.valid && live_.mode != mode && live_.phase != ProgrammedDeliveryPhase::Idle &&
		live_.phase != ProgrammedDeliveryPhase::Completed && live_.phase != ProgrammedDeliveryPhase::Aborted &&
		live_.phase != ProgrammedDeliveryPhase::Error)
	{
		last_error_ = "实验正在运行或准备中，不能切换模式";
		return false;
	}
	if (!ads_.is_open() && !ads_.open())
	{
		last_error_ = "ADS连接失败：" + ads_.last_error();
		return false;
	}
	if (!ads_.select_mode(mode))
	{
		last_error_ = "切换PLC实验模式失败：" + ads_.last_error();
		return false;
	}
	config_.mode = mode;
	last_error_.clear();
	return true;
}

bool ProgrammedDeliveryController::validate_config(const ProgrammedDeliveryConfig& config, std::string& error) const
{
	const auto finite_positive = [](double value) { return std::isfinite(value) && value > 0.0; };
	if (config.mode != ProgrammedDeliveryMode::Catheter && config.mode != ProgrammedDeliveryMode::Guidewire)
	{
		error = "请选择导管或导丝程序递送测试";
		return false;
	}
	if (config.cycle_count < 1)
	{
		error = "周期数必须至少为1";
		return false;
	}
	if (!std::isfinite(config.final_forward_distance_mm) || config.final_forward_distance_mm < 0.0 || config.final_forward_distance_mm > 20.0)
	{
		error = "最终前向距离必须在0至20 mm之间";
		return false;
	}
	if (!finite_positive(config.forward_velocity_mm_s) || !finite_positive(config.forward_acceleration_mm_s2) ||
		!finite_positive(config.forward_deceleration_mm_s2) || !finite_positive(config.forward_jerk_mm_s3) ||
		!finite_positive(config.return_velocity_mm_s) || !finite_positive(config.return_acceleration_mm_s2) ||
		!finite_positive(config.return_deceleration_mm_s2) || !finite_positive(config.return_jerk_mm_s3))
	{
		error = "前向和回退速度、加速度、减速度、Jerk必须是有限正数";
		return false;
	}
	const double angle = config.mode == ProgrammedDeliveryMode::Catheter ? config.axis2_angle_deg : config.axis7_angle_deg;
	if (!std::isfinite(angle) || angle < -360.0 || angle > 360.0)
	{
		error = "周向角度必须在-360至360度之间";
		return false;
	}
	if (config.mode == ProgrammedDeliveryMode::Guidewire &&
		(!std::isfinite(config.axis5_from_left_mm) || config.axis5_from_left_mm < 0.0 || config.axis5_from_left_mm + 21.0 > 670.0))
	{
		error = "导丝模式轴5距左限位位置必须在0至649 mm之间";
		return false;
	}
	return true;
}

bool ProgrammedDeliveryController::prepare(const ProgrammedDeliveryConfig& config)
{
	std::lock_guard<std::mutex> lock(mutex_);
	std::string validation_error;
	if (!ads_.is_open())
	{
		last_error_ = "ADS尚未连接";
		return false;
	}
	if (!live_.valid || !live_.selfcheck_done)
	{
		last_error_ = "PLC自动自检尚未完成";
		return false;
	}
	if (!std::isfinite(live_.leftlimit_axis1_abs_mm) || !std::isfinite(live_.leftlimit_axis5_abs_mm) ||
		!std::isfinite(live_.leftlimit_axis6_abs_mm))
	{
		last_error_ = "PLC左限位数据无效";
		return false;
	}
	if (!validate_config(config, validation_error))
	{
		last_error_ = validation_error;
		return false;
	}
	if (!ads_.clear_sample_buffer())
	{
		last_error_ = "清空程序递送采样缓冲失败：" + ads_.last_error();
		return false;
	}
	if (!ads_.write_config(config, true))
	{
		last_error_ = "下发程序递送准备参数失败：" + ads_.last_error();
		return false;
	}
	config_ = config;
	started_ = false;
	last_error_.clear();
	return true;
}

bool ProgrammedDeliveryController::start()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!ads_.is_open())
	{
		last_error_ = "ADS尚未连接";
		return false;
	}
	if (!live_.selfcheck_done)
	{
		last_error_ = "PLC自动自检尚未完成";
		return false;
	}
	if (!live_.setup_done || live_.phase != ProgrammedDeliveryPhase::Ready)
	{
		last_error_ = "程序递送准备定位尚未完成";
		return false;
	}
	if (!ads_.request_start())
	{
		last_error_ = "下发程序递送开始请求失败：" + ads_.last_error();
		return false;
	}
	started_ = true;
	last_error_.clear();
	return true;
}

void ProgrammedDeliveryController::abort()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (ads_.is_open() && !ads_.request_abort()) last_error_ = "下发中止请求失败：" + ads_.last_error();
	started_ = false;
}

void ProgrammedDeliveryController::tick()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!ads_.is_open()) return;
	ProgrammedDeliveryLiveFrame frame{};
	if (!ads_.read_live(frame))
	{
		last_error_ = "读取程序递送实时状态失败：" + ads_.last_error();
		return;
	}
	live_ = frame;
	if (live_.status_error_id != 0) last_error_ = "PLC程序递送错误ID：" + std::to_string(live_.status_error_id);
	else if (last_error_.rfind("PLC程序递送错误ID：", 0) == 0) last_error_.clear();
	if (live_.phase == ProgrammedDeliveryPhase::Aborted || live_.phase == ProgrammedDeliveryPhase::Error) started_ = false;
}

bool ProgrammedDeliveryController::write_metadata(const std::string& directory, std::string& error) const
{
	std::ofstream out(std::filesystem::path(directory) / "experiment.json", std::ios::binary);
	if (!out) { error = "无法创建experiment.json"; return false; }
	out << std::setprecision(12)
		<< "{\n"
		<< "  \"mode\": \"" << programmed_delivery_mode_name(config_.mode) << "\",\n"
		<< "  \"axis1_ready_from_left_mm\": 23.0,\n"
		<< "  \"catheter_trigger_from_left_mm\": 3.0,\n"
		<< "  \"axis5_from_left_mm\": " << config_.axis5_from_left_mm << ",\n"
		<< "  \"axis6_ready_from_left_mm\": " << (config_.axis5_from_left_mm + 21.0) << ",\n"
		<< "  \"axis2_angle_deg\": " << config_.axis2_angle_deg << ",\n"
		<< "  \"axis7_angle_deg\": " << config_.axis7_angle_deg << ",\n"
		<< "  \"cycle_count\": " << config_.cycle_count << ",\n"
		<< "  \"final_forward_distance_mm\": " << config_.final_forward_distance_mm << ",\n"
		<< "  \"forward_velocity_mm_s\": " << config_.forward_velocity_mm_s << ",\n"
		<< "  \"forward_acceleration_mm_s2\": " << config_.forward_acceleration_mm_s2 << ",\n"
		<< "  \"forward_deceleration_mm_s2\": " << config_.forward_deceleration_mm_s2 << ",\n"
		<< "  \"forward_jerk_mm_s3\": " << config_.forward_jerk_mm_s3 << ",\n"
		<< "  \"return_velocity_mm_s\": " << config_.return_velocity_mm_s << ",\n"
		<< "  \"return_acceleration_mm_s2\": " << config_.return_acceleration_mm_s2 << ",\n"
		<< "  \"return_deceleration_mm_s2\": " << config_.return_deceleration_mm_s2 << ",\n"
		<< "  \"return_jerk_mm_s3\": " << config_.return_jerk_mm_s3 << "\n"
		<< "}\n";
	return true;
}

bool ProgrammedDeliveryController::write_samples_csv(const std::string& directory,
	const std::vector<ProgrammedDeliverySample>& samples, std::string& error) const
{
	std::ofstream out(std::filesystem::path(directory) / "samples_1khz.csv", std::ios::binary);
	if (!out) { error = "无法创建samples_1khz.csv"; return false; }
	out << std::setprecision(12);
	if (config_.mode == ProgrammedDeliveryMode::Catheter)
	{
		out << "sample_index,plc_time_us,phase,event_sequence,cycle_index,axis1_pos_abs_mm,axis1_velocity_mm_s,axis1_acceleration_mm_s2,axis2_pos_deg,axis2_velocity_deg_s,axis2_acceleration_deg_s2,cylinder1_cmd,cylinder2_cmd,fn_1_raw,ft_1_raw\n";
		for (const auto& s : samples)
			out << s.sample_index << ',' << s.plc_time_us << ',' << static_cast<unsigned>(s.phase) << ',' << s.event_sequence << ',' << s.cycle_index << ','
				<< s.axis1_pos << ',' << s.axis1_vel << ',' << s.axis1_acc << ',' << s.axis2_pos << ',' << s.axis2_vel << ',' << s.axis2_acc << ','
				<< s.cylinder1 << ',' << s.cylinder2 << ',' << s.fn1 << ',' << s.ft1 << '\n';
	}
	else
	{
		out << "sample_index,plc_time_us,phase,event_sequence,cycle_index,axis5_pos_abs_mm,axis5_velocity_mm_s,axis5_acceleration_mm_s2,axis6_pos_abs_mm,axis6_velocity_mm_s,axis6_acceleration_mm_s2,axis7_pos_deg,axis7_velocity_deg_s,axis7_acceleration_deg_s2,cylinder3_cmd,cylinder4_cmd,fn_2_raw,ft_2_raw\n";
		for (const auto& s : samples)
			out << s.sample_index << ',' << s.plc_time_us << ',' << static_cast<unsigned>(s.phase) << ',' << s.event_sequence << ',' << s.cycle_index << ','
				<< s.axis5_pos << ',' << s.axis5_vel << ',' << s.axis5_acc << ',' << s.axis6_pos << ',' << s.axis6_vel << ',' << s.axis6_acc << ','
				<< s.axis7_pos << ',' << s.axis7_vel << ',' << s.axis7_acc << ',' << s.cylinder3 << ',' << s.cylinder4 << ',' << s.fn2 << ',' << s.ft2 << '\n';
	}
	return true;
}

bool ProgrammedDeliveryController::write_events_csv(const std::string& directory,
	const std::vector<ProgrammedDeliverySample>& samples, std::string& error) const
{
	std::ofstream out(std::filesystem::path(directory) / "events.csv", std::ios::binary);
	if (!out) { error = "无法创建events.csv"; return false; }
	out << "event_sequence,plc_time_us,cycle_index,phase,phase_name\n";
	std::uint32_t previous = static_cast<std::uint32_t>(-1);
	for (const auto& s : samples)
	{
		if (s.event_sequence == previous) continue;
		previous = s.event_sequence;
		out << s.event_sequence << ',' << s.plc_time_us << ',' << s.cycle_index << ',' << static_cast<unsigned>(s.phase) << ','
			<< programmed_delivery_phase_name(static_cast<ProgrammedDeliveryPhase>(s.phase)) << '\n';
	}
	return true;
}

bool ProgrammedDeliveryController::save_samples(const std::string& directory, std::string& error)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (live_.phase != ProgrammedDeliveryPhase::Completed)
	{
		error = "程序递送实验尚未完成";
		return false;
	}
	std::uint32_t count = 0;
	bool overflow = false;
	if (!ads_.read_sample_count(count, overflow) || overflow)
	{
		error = overflow ? "PLC程序递送采样缓冲区溢出" : "读取PLC采样数量失败：" + ads_.last_error();
		return false;
	}
	std::vector<ProgrammedDeliverySample> samples;
	if (!ads_.read_all_samples(config_.mode, count, samples))
	{
		error = "读取PLC程序递送采样失败：" + ads_.last_error();
		return false;
	}
	std::filesystem::create_directories(directory);
	return write_metadata(directory, error) && write_samples_csv(directory, samples, error) && write_events_csv(directory, samples, error);
}

ProgrammedDeliveryConfig ProgrammedDeliveryController::config() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return config_;
}

ProgrammedDeliveryLiveFrame ProgrammedDeliveryController::live() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return live_;
}

std::string ProgrammedDeliveryController::last_error() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return last_error_;
}
