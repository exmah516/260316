#include "ProgrammedDeliveryController.h"
#include "ForceCalibration.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

ProgrammedDeliveryController::ProgrammedDeliveryController()
{
	// 不在构造阶段同步阻塞ADS连接；先启动后端和UI，再由连接命令执行重试。
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
	if (stream_ads_.is_open()) stream_ads_.invalidate_zero();
	last_error_.clear();
	return true;
}

void ProgrammedDeliveryController::close_ads()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (recorder_.active())
	{
		ads_.request_abort();
		std::string finalize_error;
		const char* status = live_.phase == ProgrammedDeliveryPhase::Completed ? "Completed" : "Error";
		if (!recorder_.finalize(status, "ADS连接已断开", stream_status_.zero, finalize_error)) last_error_ = finalize_error;
		started_ = false;
	}
	ads_.close();
	if (stream_ads_.is_open()) stream_ads_.invalidate_zero();
	stream_ads_.close();
	stream_status_.zero = {};
}

bool ProgrammedDeliveryController::is_ads_open() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return ads_.is_open();
}

void ProgrammedDeliveryController::set_shared_selfcheck_state(bool done, bool busy)
{
	std::lock_guard<std::mutex> lock(mutex_);
	shared_selfcheck_done_ = done;
	shared_selfcheck_busy_ = busy;
	shared_selfcheck_valid_ = true;
	live_.selfcheck_done = done;
	live_.selfcheck_busy = busy;
}

bool ProgrammedDeliveryController::select_mode(ProgrammedDeliveryMode mode)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (recorder_.active())
	{
		last_error_ = "请先归档当前实验记录，再切换程序递送模式";
		return false;
	}
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
	if (!stream_ads_.is_open() && !stream_ads_.open())
	{
		last_error_ = "实时记录ADS连接失败：" + stream_ads_.last_error();
		return false;
	}
	if (!stream_ads_.invalidate_zero())
	{
		last_error_ = "切换模式后清除力感零点失败：" + stream_ads_.last_error();
		return false;
	}
	stream_status_.zero = {};
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
		error = "往复夹持次数必须至少为1";
		return false;
	}
	if (!std::isfinite(config.final_forward_distance_mm) || config.final_forward_distance_mm < 0.0)
	{
		error = "最终前向距离必须为不小于0的有限数";
		return false;
	}
	if (config.mode == ProgrammedDeliveryMode::Catheter)
	{
		if (!std::isfinite(config.axis1_prepare_from_left_mm) || !std::isfinite(config.axis1_trigger_from_left_mm) ||
			config.axis1_trigger_from_left_mm < 0.0 || config.axis1_prepare_from_left_mm <= config.axis1_trigger_from_left_mm)
		{
			error = "导管模式要求轴1准备位置大于触发位置，且触发位置不小于0";
			return false;
		}
		if (config.final_forward_distance_mm > config.axis1_prepare_from_left_mm - config.axis1_trigger_from_left_mm)
		{
			error = "最终前向距离不得超过轴1准备位置与触发位置之差";
			return false;
		}
	}
	else
	{
		if (!std::isfinite(config.axis5_from_left_mm) || config.axis5_from_left_mm < 0.0 || config.axis5_from_left_mm > 670.0)
		{
			error = "导丝模式轴5初始位置必须在0至670 mm之间";
			return false;
		}
		if (!std::isfinite(config.axis6_prepare_from_left_mm) || !std::isfinite(config.axis6_trigger_from_left_mm) ||
			config.axis6_trigger_from_left_mm < 0.0 || config.axis6_prepare_from_left_mm > 670.0 ||
			config.axis6_prepare_from_left_mm <= config.axis6_trigger_from_left_mm)
		{
			error = "导丝模式要求轴6初始位置大于触发位置，且两者均不得超过670 mm";
			return false;
		}
		if (config.final_forward_distance_mm > config.axis6_prepare_from_left_mm - config.axis6_trigger_from_left_mm)
		{
			error = "导丝模式最终前向距离不得超过轴6初始位置与触发位置之差";
			return false;
		}
	}
	if (config.release_wait_ms > 60000 || config.reclamp_wait_ms > 60000)
	{
		error = "电缸等待时间必须在0至60000 ms之间";
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
	return true;
}

bool ProgrammedDeliveryController::prepare(const ProgrammedDeliveryConfig& config)
{
	std::lock_guard<std::mutex> lock(mutex_);
	// 配置（包括电缸配合开关）只能在空闲、完成、中止或错误后重新准备时修改。
	// 防止绕过WPF直接发送PROGRAM_PREPARE，在运动或夹爪等待阶段改写PLC参数。
	if (started_ || live_.setup_busy ||
		(live_.valid && live_.phase >= ProgrammedDeliveryPhase::Baseline && live_.phase <= ProgrammedDeliveryPhase::FinalForward))
	{
		last_error_ = "实验正在运行或准备中，不能修改程序递送配置";
		return false;
	}
	if (recorder_.active())
	{
		std::string close_error;
		if (!recorder_.finalize("Aborted", "重新准备定位，结束上一条记录", stream_status_.zero, close_error))
		{
			last_error_ = close_error;
			return false;
		}
	}
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
	if (!ads_.write_config(config, true))
	{
		last_error_ = "下发程序递送准备参数失败：" + ads_.last_error();
		return false;
	}
	config_ = config;
	recorder_.set_program_coupling(config_.cylinder1_coupling_enabled, config_.cylinder3_coupling_enabled);
	recorder_.set_program_cylinder_words(config_.cylinder2_open_word, config_.cylinder2_close_word,
		config_.cylinder4_open_word, config_.cylinder4_close_word);
	recorder_.set_program_guidewire_positions(config_.axis5_from_left_mm, config_.axis6_prepare_from_left_mm,
		config_.axis6_trigger_from_left_mm);
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
	if (!stream_status_.zero.valid)
	{
		last_error_ = "请先点击力感取零点";
		return false;
	}
	if (!recorder_.active())
	{
		std::string record_error;
		if (!recorder_.begin(config_.mode == ProgrammedDeliveryMode::Catheter ? "catheter" : "guidewire", config_.record_suffix, record_error))
		{
			last_error_ = "创建实时记录目录失败：" + record_error;
			return false;
		}
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
		if (started_)
		{
			ads_.request_abort();
			if (recorder_.active())
			{
				std::string finalize_error;
				recorder_.finalize("Error", last_error_, stream_status_.zero, finalize_error);
			}
			started_ = false;
		}
		return;
	}
	if (shared_selfcheck_valid_)
	{
		frame.selfcheck_done = shared_selfcheck_done_;
		frame.selfcheck_busy = shared_selfcheck_busy_;
	}
	live_ = frame;
	poll_stream_locked();
	if (live_.status_error_id != 0)
	{
		const char* source = live_.error_source == 1 ? "准备定位" : live_.error_source == 2 ? "前向至触发位置" : live_.error_source == 3 ? "回退" : live_.error_source == 4 ? "最终前向" : "未知动作";
		std::ostringstream detail;
		detail << "PLC运动错误：ID " << live_.status_error_id << "；轴" << static_cast<unsigned>(live_.error_axis) << "；" << source;
		if (live_.error_axis == 1 || live_.error_axis == 5 || live_.error_axis == 6)
			detail << "；目标距左限位 " << live_.error_target_from_left_mm << " mm";
		else
			detail << "；目标值 " << live_.error_target_abs_mm;
		last_error_ = detail.str();
	}
	else if (last_error_.rfind("PLC运动错误：", 0) == 0 ||
		last_error_.rfind("读取程序递送实时状态失败：", 0) == 0)
		last_error_.clear();
	if (live_.phase == ProgrammedDeliveryPhase::Completed || live_.phase == ProgrammedDeliveryPhase::Aborted || live_.phase == ProgrammedDeliveryPhase::Error)
	{
		// 实验记录随实验终态自动归档，不再要求上位机额外点击保存按钮。
		// 只有PLC已经停止采样且最后一个分块已确认后才能关闭文件，避免终态切换与最后1ms采样竞态。
		// 先明确写入停止请求，避免PLC记录使能因终态切换延后一周期而让界面长时间显示“继续记录”。
		if (recorder_.active()) stream_ads_.stop_recording();
		if (recorder_.active() && !stream_status_.recording &&
			!stream_status_.block_ready[0] && !stream_status_.block_ready[1])
		{
			const bool completed = live_.phase == ProgrammedDeliveryPhase::Completed;
			const char* status = completed ? "Completed" :
				live_.phase == ProgrammedDeliveryPhase::Aborted ? "Aborted" : "Error";
			const std::string reason = completed ? std::string() : last_error_;
			std::string finalize_error;
			if (!recorder_.finalize(status, reason, stream_status_.zero, finalize_error) && !finalize_error.empty())
			{
				last_error_ = finalize_error;
			}
		}
		started_ = false;
	}
	// 记录器已经完成归档后，立即向UI反映非活动状态，不沿用PLC上一周期的使能值。
	live_.recording = recorder_.active() && stream_status_.recording;
	live_.recording_overflow = stream_status_.overflow;
	live_.recording_sample_count = stream_status_.total_count;
	live_.recording_error_id = stream_status_.error_id;
	live_.zero_busy = stream_status_.zero.busy;
	live_.zero_done = stream_status_.zero.done;
	live_.zero_values = stream_status_.zero.value;
	if (stream_status_.zero.error_id != 0) last_error_ = "力感取零失败，错误ID：" + std::to_string(stream_status_.zero.error_id);
}

void ProgrammedDeliveryController::poll_stream_locked()
{
	if (!stream_ads_.is_open() && !stream_ads_.open()) return;
	const double saved_zero_axis2 = stream_status_.zero.axis2_angle_deg;
	const double saved_zero_axis7 = stream_status_.zero.axis7_angle_deg;
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
	if (stream_status_.zero.done)
	{
		if (zero_file_written_)
		{
			stream_status_.zero.axis2_angle_deg = saved_zero_axis2;
			stream_status_.zero.axis7_angle_deg = saved_zero_axis7;
		}
		else
		{
			stream_status_.zero.axis2_angle_deg = live_.axis2_pos;
			stream_status_.zero.axis7_angle_deg = live_.axis7_pos;
		}
	}
	if (stream_status_.zero.done && !zero_file_written_ && recorder_.active())
	{
		std::vector<std::array<double, 4>> zero_samples;
		std::string error;
		if (stream_ads_.read_zero_samples(zero_samples))
		{
			bool ok = true;
			for (std::size_t i = 0; i < zero_samples.size(); ++i)
				ok = recorder_.append_zero(zero_samples[i], static_cast<std::uint64_t>(i) * 1000,
					live_.axis2_pos, live_.axis7_pos, error) && ok;
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
	if (!recorder_.active() || (stream_status_.source_mode != 1 && stream_status_.source_mode != 2)) return;
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
		std::vector<ProgrammedDeliverySample> converted;
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
			ProgrammedDeliverySample s{};
			s.sample_index = r.index; s.plc_time_us = r.time_us; s.phase = r.phase; s.event_sequence = r.event_sequence; s.cycle_index = r.cycle_index;
			s.axis1_pos = r.axis1_pos; s.axis1_vel = r.axis1_vel; s.axis1_acc = r.axis1_acc; s.axis2_pos = r.axis2_pos; s.axis2_vel = r.axis2_vel; s.axis2_acc = r.axis2_acc;
			s.axis5_pos = r.axis5_pos; s.axis5_vel = r.axis5_vel; s.axis5_acc = r.axis5_acc; s.axis6_pos = r.axis6_pos; s.axis6_vel = r.axis6_vel; s.axis6_acc = r.axis6_acc;
			s.axis7_pos = r.axis7_pos; s.axis7_vel = r.axis7_vel; s.axis7_acc = r.axis7_acc;
			s.cylinder1 = r.cylinder1; s.cylinder2 = r.cylinder2; s.cylinder3 = r.cylinder3; s.cylinder4 = r.cylinder4;
			s.fn1 = r.fn1; s.ft1 = r.ft1; s.fn2 = r.fn2; s.ft2 = r.ft2;
			converted.push_back(s);
		}
		std::string error;
		if (!recorder_.append_program(converted, 0, config_.mode, stream_status_.zero, error))
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

bool ProgrammedDeliveryController::request_zero()
{
	try
	{
		std::lock_guard<std::mutex> lock(mutex_);
		poll_stream_locked();
		if (!live_.selfcheck_done || !live_.setup_done || started_ ||
			(live_.phase != ProgrammedDeliveryPhase::Ready && live_.phase != ProgrammedDeliveryPhase::Idle))
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
			if (!recorder_.begin(config_.mode == ProgrammedDeliveryMode::Catheter ? "catheter" : "guidewire", config_.record_suffix, error)) { last_error_ = error; return false; }
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
		if (!stream_ads_.request_zero())
		{
			last_error_ = "下发力感取零请求失败：" + stream_ads_.last_error();
			return false;
		}
		return true;
	}
	catch (const std::exception& ex)
	{
		last_error_ = std::string("力感取零过程异常：") + ex.what();
		return false;
	}
}

bool ProgrammedDeliveryController::set_record_suffix(const std::string& suffix)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (started_ || (recorder_.active() && live_.phase != ProgrammedDeliveryPhase::Completed && live_.phase != ProgrammedDeliveryPhase::Aborted && live_.phase != ProgrammedDeliveryPhase::Error))
	{
		last_error_ = "实验进行中不能修改保存名称";
		return false;
	}
	config_.record_suffix = suffix;
	return true;
}

void ProgrammedDeliveryController::invalidate_zero()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (stream_ads_.is_open()) stream_ads_.invalidate_zero();
	stream_status_.zero = {};
}

ForceZeroState ProgrammedDeliveryController::zero_state() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return stream_status_.zero;
}

std::string ProgrammedDeliveryController::recording_directory() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return recorder_.directory();
}

bool ProgrammedDeliveryController::recording_archived() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return recorder_.archived();
}

bool ProgrammedDeliveryController::write_metadata(const std::string& directory, std::string& error) const
{
	std::ofstream out(std::filesystem::path(directory) / "experiment.json", std::ios::binary);
	if (!out) { error = "无法创建experiment.json"; return false; }
	out << std::setprecision(12)
		<< "{\n"
		<< "  \"calibration_version\": \"two_stage_sensor_0_50g_plus_installed_2026-08-29\",\n"
		<< "  \"calibration_chain\": \"sensor_0_50g_then_installed_calibration\",\n"
		<< "  \"sensor_slopes_N_per_count\": {\"fn1\": " << forcecal::kFn1SensorSlopeNPerCount
		<< ", \"ft1\": " << forcecal::kFt1SensorSlopeNPerCount << ", \"fn2\": " << forcecal::kFn2SensorSlopeNPerCount
		<< ", \"ft2\": " << forcecal::kFt2SensorSlopeNPerCount << "},\n"
		<< "  \"installation_axial_gain\": " << forcecal::kInstallationAxialGain << ",\n"
		<< "  \"installation_axial_bias_N\": " << forcecal::kInstallationAxialBiasN << ",\n"
		<< "  \"installation_torque_gain_Nmm_per_N\": " << forcecal::kInstallationTorqueGainNmmPerN << ",\n"
		<< "  \"installation_torque_bias_Nmm\": " << forcecal::kInstallationTorqueBiasNmm << ",\n"
		<< "  \"units\": {\"realtime_fn\": \"N\", \"realtime_ft\": \"N\", \"csv_torque\": \"N·mm\"},\n"
		<< "  \"crosstalk_decoupling_applied\": true,\n"
		<< "  \"gravity_compensation_applied\": false,\n"
		<< "  \"mode\": \"" << programmed_delivery_mode_name(config_.mode) << "\",\n"
		<< "  \"axis1_prepare_from_left_mm\": " << config_.axis1_prepare_from_left_mm << ",\n"
		<< "  \"axis1_trigger_from_left_mm\": " << config_.axis1_trigger_from_left_mm << ",\n"
		<< "  \"axis5_from_left_mm\": " << config_.axis5_from_left_mm << ",\n"
		<< "  \"axis6_prepare_from_left_mm\": " << config_.axis6_prepare_from_left_mm << ",\n"
		<< "  \"axis6_trigger_from_left_mm\": " << config_.axis6_trigger_from_left_mm << ",\n"
		<< "  \"axis2_angle_deg\": " << config_.axis2_angle_deg << ",\n"
		<< "  \"axis7_angle_deg\": " << config_.axis7_angle_deg << ",\n"
		<< "  \"cycle_count\": " << config_.cycle_count << ",\n"
		<< "  \"cylinder1_coupling_enabled\": " << (config_.cylinder1_coupling_enabled ? "true" : "false") << ",\n"
		<< "  \"cylinder3_coupling_enabled\": " << (config_.cylinder3_coupling_enabled ? "true" : "false") << ",\n"
		<< "  \"cylinder2_open_word\": " << config_.cylinder2_open_word << ",\n"
		<< "  \"cylinder2_close_word\": " << config_.cylinder2_close_word << ",\n"
		<< "  \"cylinder4_open_word\": " << config_.cylinder4_open_word << ",\n"
		<< "  \"cylinder4_close_word\": " << config_.cylinder4_close_word << ",\n"
		<< "  \"final_forward_distance_mm\": " << config_.final_forward_distance_mm << ",\n"
		<< "  \"release_wait_ms\": " << config_.release_wait_ms << ",\n"
		<< "  \"reclamp_wait_ms\": " << config_.reclamp_wait_ms << ",\n"
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
		out << "sample_index,plc_time_us,phase,event_sequence,cycle_index,axis1_pos_abs_mm,axis1_velocity_mm_s,axis1_acceleration_mm_s2,axis2_pos_deg,axis2_velocity_deg_s,axis2_acceleration_deg_s2,cylinder1_cmd,cylinder2_cmd,fn_1_raw,ft_1_raw,fn1_sensor_N,ft1_sensor_N,fn1_cal_delta_N,fn1_cal_abs_N,ft1_cal_delta_N,ft1_cal_abs_N,torque1_cal_delta_Nmm,torque1_cal_abs_Nmm,fn1_decoupled_delta_N,torque1_decoupled_delta_Nmm,fn1_decoupled_abs_N,torque1_decoupled_abs_Nmm,axis2_angle_deg\n";
		for (const auto& s : samples)
		{
			const forcecal::Result cal = forcecal::calculate(s.fn1, s.ft1, s.fn2, s.ft2, stream_status_.zero.value, stream_status_.zero.valid);
			out << s.sample_index << ',' << s.plc_time_us << ',' << static_cast<unsigned>(s.phase) << ',' << s.event_sequence << ',' << s.cycle_index << ','
				<< s.axis1_pos << ',' << s.axis1_vel << ',' << s.axis1_acc << ',' << s.axis2_pos << ',' << s.axis2_vel << ',' << s.axis2_acc << ','
				<< s.cylinder1 << ',' << s.cylinder2 << ',' << s.fn1 << ',' << s.ft1;
			if (cal.valid)
				out << ',' << cal.side1.sensor_force_n << ',' << cal.side1.sensor_tangential_n << ',' << cal.side1.force_cal_delta_n << ',' << cal.side1.force_cal_abs_n
					<< ',' << cal.side1.ft_cal_delta_n << ',' << cal.side1.ft_cal_abs_n << ',' << cal.side1.torque_cal_delta_nmm << ',' << cal.side1.torque_cal_abs_nmm
					<< ',' << cal.side1.force_decoupled_delta_n << ',' << cal.side1.torque_decoupled_delta_nmm << ',' << cal.side1.force_decoupled_abs_n << ',' << cal.side1.torque_decoupled_abs_nmm;
			else out << ",,,,,,,,,,,,";
			out << ',' << s.axis2_pos << '\n';
		}
	}
	else
	{
		out << "sample_index,plc_time_us,phase,event_sequence,cycle_index,axis5_pos_abs_mm,axis5_velocity_mm_s,axis5_acceleration_mm_s2,axis6_pos_abs_mm,axis6_velocity_mm_s,axis6_acceleration_mm_s2,axis7_pos_deg,axis7_velocity_deg_s,axis7_acceleration_deg_s2,cylinder3_cmd,cylinder4_cmd,fn_2_raw,ft_2_raw,fn2_sensor_N,ft2_sensor_N,fn2_cal_delta_N,fn2_cal_abs_N,ft2_cal_delta_N,ft2_cal_abs_N,torque2_cal_delta_Nmm,torque2_cal_abs_Nmm,fn2_decoupled_delta_N,torque2_decoupled_delta_Nmm,fn2_decoupled_abs_N,torque2_decoupled_abs_Nmm,axis7_angle_deg\n";
		for (const auto& s : samples)
		{
			const forcecal::Result cal = forcecal::calculate(s.fn1, s.ft1, s.fn2, s.ft2, stream_status_.zero.value, stream_status_.zero.valid);
			out << s.sample_index << ',' << s.plc_time_us << ',' << static_cast<unsigned>(s.phase) << ',' << s.event_sequence << ',' << s.cycle_index << ','
				<< s.axis5_pos << ',' << s.axis5_vel << ',' << s.axis5_acc << ',' << s.axis6_pos << ',' << s.axis6_vel << ',' << s.axis6_acc << ','
				<< s.axis7_pos << ',' << s.axis7_vel << ',' << s.axis7_acc << ',' << s.cylinder3 << ',' << s.cylinder4 << ',' << s.fn2 << ',' << s.ft2;
			if (cal.valid)
				out << ',' << cal.side2.sensor_force_n << ',' << cal.side2.sensor_tangential_n << ',' << cal.side2.force_cal_delta_n << ',' << cal.side2.force_cal_abs_n
					<< ',' << cal.side2.ft_cal_delta_n << ',' << cal.side2.ft_cal_abs_n << ',' << cal.side2.torque_cal_delta_nmm << ',' << cal.side2.torque_cal_abs_nmm
					<< ',' << cal.side2.force_decoupled_delta_n << ',' << cal.side2.torque_decoupled_delta_nmm << ',' << cal.side2.force_decoupled_abs_n << ',' << cal.side2.torque_decoupled_abs_nmm;
			else out << ",,,,,,,,,,,,";
			out << ',' << s.axis7_pos << '\n';
		}
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
	if (recorder_.active())
	{
		poll_stream_locked();
		ProgrammedDeliveryLiveFrame latest{};
		if (ads_.is_open() && ads_.read_live(latest)) live_ = latest;
		if (live_.phase != ProgrammedDeliveryPhase::Completed && live_.phase != ProgrammedDeliveryPhase::Aborted && live_.phase != ProgrammedDeliveryPhase::Error)
		{
			error = "实验正在进行，数据已经实时保存";
			return false;
		}
		return recorder_.finalize(live_.phase == ProgrammedDeliveryPhase::Completed ? "Completed" : live_.phase == ProgrammedDeliveryPhase::Aborted ? "Aborted" : "Error", last_error_, stream_status_.zero, error);
	}
	if (directory.empty() && !recorder_.directory().empty()) return true;
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
