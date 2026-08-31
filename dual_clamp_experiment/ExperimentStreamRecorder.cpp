#include "ExperimentStreamRecorder.h"
#include "ExperimentStreamAds.h"
#include "ForceCalibration.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <system_error>
#include <utility>
#include <Windows.h>

namespace
{
	std::string now_name()
	{
		const auto now = std::chrono::system_clock::now();
		const std::time_t t = std::chrono::system_clock::to_time_t(now);
		std::tm local{};
		localtime_s(&local, &t);
		std::ostringstream out;
		out << std::put_time(&local, "%Y%m%d_%H%M%S");
		return out.str();
	}

	std::string now_iso()
	{
		const auto now = std::chrono::system_clock::now();
		const std::time_t t = std::chrono::system_clock::to_time_t(now);
		std::tm local{};
		localtime_s(&local, &t);
		std::ostringstream out;
		out << std::put_time(&local, "%Y-%m-%dT%H:%M:%S");
		return out.str();
	}

	std::string zeroed(double raw, double zero)
	{
		return std::to_string(raw - zero);
	}

	const char* phase_event_name(bool program_mode, std::uint8_t phase)
	{
		if (program_mode)
		{
			switch (phase)
			{
			case 3: return "BaselineStart";
			case 4: return "ForwardStart";
			case 5: return "ReleaseStart";
			case 6: return "ReturnStart";
			case 7: return "ReclampStart";
			case 9: return "FinalForwardStart";
			default: return "PhaseChange";
			}
		}
		switch (phase)
		{
		case 3: return "BaselineStart";
		case 5: return "ReleaseStart";
		case 6: return "ReturnStart";
		case 7: return "ReclampStart";
		default: return "PhaseChange";
		}
	}

	std::filesystem::path executable_directory()
	{
		wchar_t buffer[MAX_PATH]{};
		const DWORD length = GetModuleFileNameW(nullptr, buffer, static_cast<DWORD>(std::size(buffer)));
		if (length == 0 || length >= std::size(buffer)) return std::filesystem::current_path();
		return std::filesystem::path(buffer, buffer + length).parent_path();
	}
}

ExperimentStreamRecorder::ExperimentStreamRecorder() = default;

std::string ExperimentStreamRecorder::last_error() const
{
	std::lock_guard<std::mutex> lock(writer_mutex_);
	return last_error_.empty() ? writer_error_ : last_error_;
}

ExperimentStreamRecorder::~ExperimentStreamRecorder()
{
	if (active_)
	{
		std::string ignored;
		ForceZeroState zero;
		finalize("Error", "程序退出", zero, ignored);
	}
	else
	{
		stop_writer();
	}
}

bool ExperimentStreamRecorder::failed() const
{
	std::lock_guard<std::mutex> lock(writer_mutex_);
	return !writer_error_.empty();
}

std::string ExperimentStreamRecorder::sanitize_suffix(const std::string& value)
{
	std::string out;
	for (unsigned char c : value)
	{
		if (c < 32 || c == '\\' || c == '/' || c == ':' || c == '*' || c == '?' || c == '"' || c == '<' || c == '>' || c == '|') continue;
		out.push_back(static_cast<char>(c));
	}
	while (!out.empty() && out.front() == ' ') out.erase(out.begin());
	while (!out.empty() && out.back() == ' ') out.pop_back();
	if (out.empty()) out = "experiment";
	if (out.size() > 64)
	{
		out.resize(64);
		// 按UTF-8代码点截断，避免最后留下不完整的多字节序列导致u8path抛出异常。
		while (!out.empty())
		{
			std::size_t start = out.size() - 1;
			while (start > 0 && (static_cast<unsigned char>(out[start]) & 0xC0) == 0x80) --start;
			const unsigned char lead = static_cast<unsigned char>(out[start]);
			const std::size_t expected = lead < 0x80 ? 1 : lead < 0xE0 ? 2 : lead < 0xF0 ? 3 : lead < 0xF8 ? 4 : 1;
			if (out.size() - start < expected) out.resize(start);
			else break;
		}
	}
	return out;
}


void append_side_columns(std::ostringstream& rows, const forcecal::SideResult& side)
	{
		rows << ',' << side.sensor_force_n
			<< ',' << side.sensor_tangential_n
			<< ',' << side.force_cal_delta_n
			<< ',' << side.force_cal_abs_n
			<< ',' << side.ft_cal_delta_n
			<< ',' << side.ft_cal_abs_n
			<< ',' << side.torque_cal_delta_nmm
			<< ',' << side.torque_cal_abs_nmm
			<< ',' << side.force_decoupled_delta_n
			<< ',' << side.torque_decoupled_delta_nmm
			<< ',' << side.force_decoupled_abs_n
			<< ',' << side.torque_decoupled_abs_nmm;
	}

std::string ExperimentStreamRecorder::json_escape(const std::string& value)
{
	std::string out;
	for (char c : value)
	{
		if (c == '\\' || c == '"') out.push_back('\\');
		if (c == '\n') { out += "\\n"; continue; }
		if (c == '\r') { out += "\\r"; continue; }
		out.push_back(c);
	}
	return out;
}

std::string ExperimentStreamRecorder::phase_name(std::uint8_t phase)
{
	return programmed_delivery_phase_name(static_cast<ProgrammedDeliveryPhase>(phase));
}

bool ExperimentStreamRecorder::begin(const std::string& mode, const std::string& suffix, std::string& error)
{
	try
	{
		if (active_)
		{
			error = "已有实验记录正在进行";
			return false;
		}
		// 上一次会话异常结束时，先回收遗留写线程，避免下一次取零重复使用失效线程对象。
		if (writer_thread_.joinable())
		{
			{
				std::lock_guard<std::mutex> lock(writer_mutex_);
				writer_stop_requested_ = true;
			}
			writer_cv_.notify_all();
			writer_thread_.join();
		}
		{
			std::lock_guard<std::mutex> lock(writer_mutex_);
			writer_queue_.clear();
			writer_busy_ = false;
			writer_stop_requested_ = false;
			writer_finished_ = false;
			writer_error_.clear();
		}
		mode_name_ = mode;
		start_time_local_ = now_iso();
		end_time_local_.clear();
		zero_start_time_local_.clear();
		zero_end_time_local_.clear();
		first_sample_time_us_ = 0;
		last_sample_time_us_ = 0;
		has_sample_time_ = false;
		block_count_ = 0;
		program_mode_ = mode == "catheter" || mode == "guidewire";
		standalone_mode_ = mode == "standalone";
		const std::filesystem::path root = executable_directory() / L"records";
		std::error_code fs_error;
		std::filesystem::create_directories(root, fs_error);
		if (fs_error) throw std::system_error(fs_error, "创建实验记录根目录失败");
		const std::string base_name = now_name() + "_" + sanitize_suffix(suffix);
		const std::string base_utf8 = base_name;
		std::filesystem::path path = root / std::filesystem::u8path(base_utf8);
		for (int i = 1; std::filesystem::exists(path, fs_error); ++i)
		{
			if (fs_error) throw std::system_error(fs_error, "检查实验记录目录失败");
			path = root / std::filesystem::u8path(base_utf8 + "_" + (i < 10 ? "0" : "") + std::to_string(i));
		}
		std::filesystem::create_directories(path, fs_error);
		if (fs_error) throw std::system_error(fs_error, "创建实验记录目录失败");
		directory_ = path.u8string();
		samples_.open(path / L"samples_1khz.csv", std::ios::out | std::ios::trunc);
		events_.open(path / L"events.csv", std::ios::out | std::ios::trunc);
		zero_file_.open(path / L"zero_calibration.csv", std::ios::out | std::ios::trunc);
		if (!samples_ || !events_ || !zero_file_)
		{
			error = "无法创建实验记录文件";
			last_error_ = error;
			return false;
		}
		events_ << "event_sequence,plc_time_us,cycle_index,phase,event_name\n";
		if (!write_zero_header(error)) return false;
		if (standalone_mode_ ? !write_standalone_header(standalone_field_mask_, error) :
			(program_mode_ ? !write_program_header(mode == "catheter" ? ProgrammedDeliveryMode::Catheter : ProgrammedDeliveryMode::Guidewire, error) : !write_dual_header(error))) return false;
		active_ = true;
		archived_ = false;
		sample_count_ = 0;
		zero_sample_count_ = 0;
		last_event_sequence_ = static_cast<std::uint32_t>(-1);
		last_phase_ = 0;
		writer_thread_ = std::thread(&ExperimentStreamRecorder::writer_loop, this);
		return write_event("RecordStart", 0, 0, 0, 0, error);
	}
	catch (const std::exception& ex)
	{
		error = std::string("创建实验记录失败：") + ex.what();
		last_error_ = error;
		active_ = false;
		if (writer_thread_.joinable())
		{
			try
			{
				{
					std::lock_guard<std::mutex> lock(writer_mutex_);
					writer_stop_requested_ = true;
				}
				writer_cv_.notify_all();
				writer_thread_.join();
			}
			catch (...) { }
		}
		samples_.close();
		events_.close();
		zero_file_.close();
		return false;
	}
}

bool ExperimentStreamRecorder::begin_standalone(const std::string& suffix, std::uint64_t field_mask, std::string& error)
{
	standalone_field_mask_ = field_mask;
	return begin("standalone", suffix, error);
}

void ExperimentStreamRecorder::set_program_coupling(bool cylinder1_enabled, bool cylinder3_enabled)
{
	program_cylinder1_coupling_enabled_ = cylinder1_enabled;
	program_cylinder3_coupling_enabled_ = cylinder3_enabled;
}

bool ExperimentStreamRecorder::reconfigure_standalone(std::uint64_t field_mask, std::string& error)
{
	if (!active_ || !standalone_mode_)
	{
		error = "独立记录尚未创建";
		return false;
	}
	if (sample_count_ != 0)
	{
		error = "独立记录已经写入样本，不能修改字段选择";
		return false;
	}
	if (field_mask == 0)
	{
		error = "至少选择一个独立记录字段";
		return false;
	}
	if (!wait_for_queue(error)) return false;
	samples_.flush();
	samples_.close();
	samples_.open(std::filesystem::u8path(directory_) / L"samples_1khz.csv", std::ios::out | std::ios::trunc);
	if (!samples_)
	{
		error = "无法重建独立记录CSV文件";
		last_error_ = error;
		return false;
	}
	standalone_field_mask_ = field_mask;
	return write_standalone_header(field_mask, error);
}

bool ExperimentStreamRecorder::enqueue_write(int kind, std::string data, std::string& error)
{
	std::lock_guard<std::mutex> lock(writer_mutex_);
	if (!writer_error_.empty())
	{
		error = writer_error_;
		last_error_ = error;
		return false;
	}
	if (!writer_thread_.joinable())
	{
		error = "实时记录写线程未启动";
		last_error_ = error;
		return false;
	}
	writer_queue_.push_back(PendingWrite{ kind, std::move(data) });
	writer_cv_.notify_one();
	return true;
}

void ExperimentStreamRecorder::writer_loop()
{
	for (;;)
	{
		PendingWrite pending;
		{
			std::unique_lock<std::mutex> lock(writer_mutex_);
			writer_cv_.wait(lock, [this] { return writer_stop_requested_ || !writer_queue_.empty(); });
			if (writer_queue_.empty() && writer_stop_requested_)
			{
				writer_finished_ = true;
				writer_cv_.notify_all();
				return;
			}
			pending = std::move(writer_queue_.front());
			writer_queue_.pop_front();
			writer_busy_ = true;
		}

		std::ofstream* output = pending.kind == 0 ? &samples_ : pending.kind == 1 ? &events_ : &zero_file_;
		(*output) << pending.data;
		if (!(*output))
		{
			std::lock_guard<std::mutex> lock(writer_mutex_);
			writer_busy_ = false;
			if (writer_error_.empty())
			{
				writer_error_ = pending.kind == 0 ? "实时写入samples_1khz.csv失败" : pending.kind == 1 ? "实时写入events.csv失败" : "实时写入zero_calibration.csv失败";
				last_error_ = writer_error_;
			}
			writer_cv_.notify_all();
		}
		if (*output) {
			std::lock_guard<std::mutex> lock(writer_mutex_);
			writer_busy_ = false;
			writer_cv_.notify_all();
		}
	}
}

bool ExperimentStreamRecorder::wait_for_queue(std::string& error)
{
	std::unique_lock<std::mutex> lock(writer_mutex_);
	writer_cv_.wait(lock, [this] { return (writer_queue_.empty() && !writer_busy_) || !writer_error_.empty(); });
	if (!writer_error_.empty())
	{
		error = writer_error_;
		last_error_ = error;
		return false;
	}
	return true;
}

void ExperimentStreamRecorder::stop_writer()
{
	if (!writer_thread_.joinable()) return;
	{
		std::lock_guard<std::mutex> lock(writer_mutex_);
		writer_stop_requested_ = true;
	}
	writer_cv_.notify_all();
	writer_thread_.join();
}

bool ExperimentStreamRecorder::write_dual_header(std::string& error)
{
	samples_ << "sample_index,plc_time_us,phase,event_sequence,axis1_pos_mm,axis1_vel_mm_s,axis1_acc_mm_s2,axis6_pos_mm,axis6_vel_mm_s,axis6_acc_mm_s2,ft1_raw,fn1_raw,ft2_raw,fn2_raw,cylinder2_cmd,cylinder4_cmd,fn1_zeroed,ft1_zeroed,fn2_zeroed,ft2_zeroed,fn1_sensor_N,ft1_sensor_N,fn1_cal_delta_N,fn1_cal_abs_N,ft1_cal_delta_N,ft1_cal_abs_N,torque1_cal_delta_Nmm,torque1_cal_abs_Nmm,fn1_decoupled_delta_N,torque1_decoupled_delta_Nmm,fn1_decoupled_abs_N,torque1_decoupled_abs_Nmm,axis2_angle_deg,fn2_sensor_N,ft2_sensor_N,fn2_cal_delta_N,fn2_cal_abs_N,ft2_cal_delta_N,ft2_cal_abs_N,torque2_cal_delta_Nmm,torque2_cal_abs_Nmm,fn2_decoupled_delta_N,torque2_decoupled_delta_Nmm,fn2_decoupled_abs_N,torque2_decoupled_abs_Nmm,axis7_angle_deg\n";
	if (!samples_) { error = "写入旧双机构模式CSV表头失败"; return false; }
	return true;
}

bool ExperimentStreamRecorder::write_program_header(ProgrammedDeliveryMode mode, std::string& error)
{
	if (mode == ProgrammedDeliveryMode::Catheter)
		samples_ << "sample_index,plc_time_us,phase,event_sequence,cycle_index,axis1_pos_mm,axis1_vel_mm_s,axis1_acc_mm_s2,axis2_pos_deg,axis2_vel_deg_s,axis2_acc_deg_s2,cylinder1_cmd,cylinder2_cmd,fn1_raw,ft1_raw,fn1_zeroed,ft1_zeroed,fn1_sensor_N,ft1_sensor_N,fn1_cal_delta_N,fn1_cal_abs_N,ft1_cal_delta_N,ft1_cal_abs_N,torque1_cal_delta_Nmm,torque1_cal_abs_Nmm,fn1_decoupled_delta_N,torque1_decoupled_delta_Nmm,fn1_decoupled_abs_N,torque1_decoupled_abs_Nmm,axis2_angle_deg\n";
	else
		samples_ << "sample_index,plc_time_us,phase,event_sequence,cycle_index,axis5_pos_mm,axis5_vel_mm_s,axis5_acc_mm_s2,axis6_pos_mm,axis6_vel_mm_s,axis6_acc_mm_s2,axis7_pos_deg,axis7_vel_deg_s,axis7_acc_deg_s2,cylinder3_cmd,cylinder4_cmd,fn2_raw,ft2_raw,fn2_zeroed,ft2_zeroed,fn2_sensor_N,ft2_sensor_N,fn2_cal_delta_N,fn2_cal_abs_N,ft2_cal_delta_N,ft2_cal_abs_N,torque2_cal_delta_Nmm,torque2_cal_abs_Nmm,fn2_decoupled_delta_N,torque2_decoupled_delta_Nmm,fn2_decoupled_abs_N,torque2_decoupled_abs_Nmm,axis7_angle_deg\n";
	if (!samples_) { error = "写入程序模式CSV表头失败"; return false; }
	return true;
}

bool ExperimentStreamRecorder::write_standalone_header(std::uint64_t field_mask, std::string& error)
{
	std::vector<std::string> columns = { "sample_index", "plc_time_us", "phase", "event_sequence" };
	const auto add = [&](StandaloneRecordField field, const char* name)
	{
		if ((field_mask & standalone_field_bit(field)) != 0) columns.emplace_back(name);
	};
	add(StandaloneRecordField::Axis1Pos, "axis1_pos_mm");
	add(StandaloneRecordField::Axis1Velocity, "axis1_vel_mm_s");
	add(StandaloneRecordField::Axis1Acceleration, "axis1_acc_mm_s2");
	add(StandaloneRecordField::Axis2Pos, "axis2_pos_deg");
	add(StandaloneRecordField::Axis2Velocity, "axis2_vel_deg_s");
	add(StandaloneRecordField::Axis2Acceleration, "axis2_acc_deg_s2");
	add(StandaloneRecordField::Axis5Pos, "axis5_pos_mm");
	add(StandaloneRecordField::Axis5Velocity, "axis5_vel_mm_s");
	add(StandaloneRecordField::Axis5Acceleration, "axis5_acc_mm_s2");
	add(StandaloneRecordField::Axis6Pos, "axis6_pos_mm");
	add(StandaloneRecordField::Axis6Velocity, "axis6_vel_mm_s");
	add(StandaloneRecordField::Axis6Acceleration, "axis6_acc_mm_s2");
	add(StandaloneRecordField::Axis7Pos, "axis7_pos_deg");
	add(StandaloneRecordField::Axis7Velocity, "axis7_vel_deg_s");
	add(StandaloneRecordField::Axis7Acceleration, "axis7_acc_deg_s2");
	add(StandaloneRecordField::Cylinder1, "cylinder1_cmd");
	add(StandaloneRecordField::Cylinder2, "cylinder2_cmd");
	add(StandaloneRecordField::Cylinder3, "cylinder3_cmd");
	add(StandaloneRecordField::Cylinder4, "cylinder4_cmd");
	add(StandaloneRecordField::Fn1Raw, "fn1_raw");
	add(StandaloneRecordField::Ft1Raw, "ft1_raw");
	add(StandaloneRecordField::Fn2Raw, "fn2_raw");
	add(StandaloneRecordField::Ft2Raw, "ft2_raw");
	add(StandaloneRecordField::Fn1Zeroed, "fn1_zeroed");
	add(StandaloneRecordField::Ft1Zeroed, "ft1_zeroed");
	add(StandaloneRecordField::Fn2Zeroed, "fn2_zeroed");
	add(StandaloneRecordField::Ft2Zeroed, "ft2_zeroed");
	add(StandaloneRecordField::Fn1Sensor, "fn1_sensor_N");
	add(StandaloneRecordField::Ft1Sensor, "ft1_sensor_N");
	add(StandaloneRecordField::Fn1CalDelta, "fn1_cal_delta_N");
	add(StandaloneRecordField::Fn1CalAbs, "fn1_cal_abs_N");
	add(StandaloneRecordField::Ft1CalDelta, "ft1_cal_delta_N");
	add(StandaloneRecordField::Ft1CalAbs, "ft1_cal_abs_N");
	add(StandaloneRecordField::Torque1CalDelta, "torque1_cal_delta_Nmm");
	add(StandaloneRecordField::Torque1CalAbs, "torque1_cal_abs_Nmm");
	add(StandaloneRecordField::Fn1DecoupledDelta, "fn1_decoupled_delta_N");
	add(StandaloneRecordField::Torque1DecoupledDelta, "torque1_decoupled_delta_Nmm");
	add(StandaloneRecordField::Fn1DecoupledAbs, "fn1_decoupled_abs_N");
	add(StandaloneRecordField::Torque1DecoupledAbs, "torque1_decoupled_abs_Nmm");
	add(StandaloneRecordField::Axis2Angle, "axis2_angle_deg");
	add(StandaloneRecordField::Fn2Sensor, "fn2_sensor_N");
	add(StandaloneRecordField::Ft2Sensor, "ft2_sensor_N");
	add(StandaloneRecordField::Fn2CalDelta, "fn2_cal_delta_N");
	add(StandaloneRecordField::Fn2CalAbs, "fn2_cal_abs_N");
	add(StandaloneRecordField::Ft2CalDelta, "ft2_cal_delta_N");
	add(StandaloneRecordField::Ft2CalAbs, "ft2_cal_abs_N");
	add(StandaloneRecordField::Torque2CalDelta, "torque2_cal_delta_Nmm");
	add(StandaloneRecordField::Torque2CalAbs, "torque2_cal_abs_Nmm");
	add(StandaloneRecordField::Fn2DecoupledDelta, "fn2_decoupled_delta_N");
	add(StandaloneRecordField::Torque2DecoupledDelta, "torque2_decoupled_delta_Nmm");
	add(StandaloneRecordField::Fn2DecoupledAbs, "fn2_decoupled_abs_N");
	add(StandaloneRecordField::Torque2DecoupledAbs, "torque2_decoupled_abs_Nmm");
	add(StandaloneRecordField::Axis7Angle, "axis7_angle_deg");
	for (std::size_t i = 0; i < columns.size(); ++i)
	{
		if (i != 0) samples_ << ',';
		samples_ << columns[i];
	}
	samples_ << '\n';
	if (!samples_)
	{
		error = "写入独立记录CSV表头失败";
		return false;
	}
	return true;
}

bool ExperimentStreamRecorder::write_zero_header(std::string& error)
{
	zero_file_ << "sample_index,plc_time_us,fn1_raw,ft1_raw,fn2_raw,ft2_raw,axis2_angle_deg,axis7_angle_deg\n";
	if (!zero_file_) { error = "写入零点CSV表头失败"; return false; }
	return true;
}

bool ExperimentStreamRecorder::write_event(const std::string& event, std::uint64_t time_us, std::uint32_t cycle,
	std::uint8_t phase, std::uint32_t sequence, std::string& error)
{
	std::ostringstream row;
	row << sequence << ',' << time_us << ',' << cycle << ',' << static_cast<unsigned>(phase) << ',' << event << '\n';
	if (writer_thread_.joinable()) return enqueue_write(1, row.str(), error);
	if (!events_) { error = "事件文件未打开"; return false; }
	events_ << row.str();
	if (!events_) { error = "写入事件文件失败"; return false; }
	return true;
}

bool ExperimentStreamRecorder::append_dual(const std::vector<DualClampSample>& samples, std::size_t begin_index,
	const ForceZeroState& zero, std::string& error)
{
	if (!active_) return true;
	std::ostringstream rows;
	rows << std::setprecision(12);
	for (std::size_t i = begin_index; i < samples.size(); ++i)
	{
		const auto& s = samples[i];
		const forcecal::Result cal = forcecal::calculate(s.fn_1_raw, s.ft_1_raw, s.fn_2_raw, s.ft_2_raw, zero.value, zero.valid);
		if (last_event_sequence_ == static_cast<std::uint32_t>(-1) && s.phase == 3)
		{
			if (!write_event("BaselineStart", s.plc_time_us, 0, s.phase, s.event_sequence, error)) return false;
		}
		if (s.phase != last_phase_ && last_phase_ == 3)
		{
			if (!write_event("BaselineEnd", s.plc_time_us, 0, s.phase, s.event_sequence, error)) return false;
		}
		if (s.event_sequence != last_event_sequence_ || s.phase != last_phase_)
		{
			if (last_event_sequence_ != static_cast<std::uint32_t>(-1) && !write_event(phase_event_name(false, s.phase), s.plc_time_us, 0, s.phase, s.event_sequence, error)) return false;
			last_event_sequence_ = s.event_sequence;
		}
		last_phase_ = s.phase;
		if (!has_sample_time_) { first_sample_time_us_ = s.plc_time_us; has_sample_time_ = true; }
		last_sample_time_us_ = s.plc_time_us;
		rows << s.sample_index << ',' << s.plc_time_us << ',' << static_cast<unsigned>(s.phase) << ',' << s.event_sequence << ','
			<< s.axis1_pos_abs_mm << ',' << s.axis1_velocity_mm_s << ',' << s.axis1_acceleration_mm_s2 << ','
			<< s.axis6_pos_abs_mm << ',' << s.axis6_velocity_mm_s << ',' << s.axis6_acceleration_mm_s2 << ','
			<< s.ft_1_raw << ',' << s.fn_1_raw << ',' << s.ft_2_raw << ',' << s.fn_2_raw << ',' << s.cylinder2_cmd << ',' << s.cylinder4_cmd << ','
			<< zeroed(s.fn_1_raw, zero.value[0]) << ',' << zeroed(s.ft_1_raw, zero.value[1]) << ',' << zeroed(s.fn_2_raw, zero.value[2]) << ',' << zeroed(s.ft_2_raw, zero.value[3]);
		if (cal.valid)
		{
			append_side_columns(rows, cal.side1);
			rows << ',' << s.axis2_angle_abs_deg;
			append_side_columns(rows, cal.side2);
			rows << ',' << s.axis7_angle_abs_deg;
		}
		else
		{
			rows << ",,,,,,,,,,,,,,,,,,,,,,,,,,";
		}
		rows << '\n';
		++sample_count_;
	}
	if (rows.str().empty()) return true;
	++block_count_;
	return enqueue_write(0, rows.str(), error);
}

bool ExperimentStreamRecorder::append_program(const std::vector<ProgrammedDeliverySample>& samples, std::size_t begin_index,
	ProgrammedDeliveryMode mode, const ForceZeroState& zero, std::string& error)
{
	if (!active_) return true;
	std::ostringstream rows;
	rows << std::setprecision(12);
	for (std::size_t i = begin_index; i < samples.size(); ++i)
	{
		const auto& s = samples[i];
		const forcecal::Result cal = forcecal::calculate(s.fn1, s.ft1, s.fn2, s.ft2, zero.value, zero.valid);
		if (last_event_sequence_ == static_cast<std::uint32_t>(-1) && s.phase == 3)
		{
			if (!write_event("BaselineStart", s.plc_time_us, s.cycle_index, s.phase, s.event_sequence, error)) return false;
		}
		if (s.phase != last_phase_ && last_phase_ == 3)
		{
			if (!write_event("BaselineEnd", s.plc_time_us, s.cycle_index, s.phase, s.event_sequence, error)) return false;
		}
		if (s.event_sequence != last_event_sequence_ || s.phase != last_phase_)
		{
			if (last_event_sequence_ != static_cast<std::uint32_t>(-1) && !write_event(phase_event_name(true, s.phase), s.plc_time_us, s.cycle_index, s.phase, s.event_sequence, error)) return false;
			last_event_sequence_ = s.event_sequence;
		}
		last_phase_ = s.phase;
		if (!has_sample_time_) { first_sample_time_us_ = s.plc_time_us; has_sample_time_ = true; }
		last_sample_time_us_ = s.plc_time_us;
		if (mode == ProgrammedDeliveryMode::Catheter)
		{
			rows << s.sample_index << ',' << s.plc_time_us << ',' << static_cast<unsigned>(s.phase) << ',' << s.event_sequence << ',' << s.cycle_index << ','
				<< s.axis1_pos << ',' << s.axis1_vel << ',' << s.axis1_acc << ',' << s.axis2_pos << ',' << s.axis2_vel << ',' << s.axis2_acc << ','
				<< s.cylinder1 << ',' << s.cylinder2 << ','
				<< s.fn2 << ',' << s.ft2 << ',' << zeroed(s.fn2, zero.value[2]) << ',' << zeroed(s.ft2, zero.value[3]); // <-- 改这里
			if (cal.valid) append_side_columns(rows, cal.side1);
			else rows << ",,,,,,,,,,,,";
			rows << ',' << s.axis2_pos;
			rows << '\n';
		}
		else
		{
			rows << s.sample_index << ',' << s.plc_time_us << ',' << static_cast<unsigned>(s.phase) << ',' << s.event_sequence << ',' << s.cycle_index << ','
				<< s.axis5_pos << ',' << s.axis5_vel << ',' << s.axis5_acc << ',' << s.axis6_pos << ',' << s.axis6_vel << ',' << s.axis6_acc << ','
				<< s.axis7_pos << ',' << s.axis7_vel << ',' << s.axis7_acc << ',' << s.cylinder3 << ',' << s.cylinder4 << ',' << s.fn2 << ',' << s.ft2 << ','
				<< zeroed(s.fn2, zero.value[2]) << ',' << zeroed(s.ft2, zero.value[3]);
			if (cal.valid) append_side_columns(rows, cal.side2);
			else rows << ",,,,,,,,,,,,";
			rows << ',' << s.axis7_pos;
			rows << '\n';
		}
		++sample_count_;
	}
	if (rows.str().empty()) return true;
	++block_count_;
	return enqueue_write(0, rows.str(), error);
}

bool ExperimentStreamRecorder::append_standalone(const std::vector<ExperimentStreamSample>& samples,
	const ForceZeroState& zero, std::uint64_t field_mask, std::string& error)
{
	if (!active_) return true;
	std::ostringstream rows;
	rows << std::setprecision(12);
	for (const auto& s : samples)
	{
		bool first = true;
		const auto value = [&](auto&& item)
		{
			if (!first) rows << ',';
			rows << item;
			first = false;
		};
		const auto optional_zero = [&](bool enabled, double raw, double zero_value)
		{
			if (!enabled) return;
			if (zero.valid) value(raw - zero_value);
			else value("");
		};
		const forcecal::Result cal = forcecal::calculate(s.fn1, s.ft1, s.fn2, s.ft2, zero.value, zero.valid);
		const auto optional_cal = [&](bool enabled, double item)
		{
			if (!enabled) return;
			if (cal.valid) value(item);
			else value(std::string{});
		};
		value(s.index);
		value(s.time_us);
		value(static_cast<unsigned>(s.phase));
		value(s.event_sequence);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis1Pos)) value(s.axis1_pos);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis1Velocity)) value(s.axis1_vel);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis1Acceleration)) value(s.axis1_acc);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis2Pos)) value(s.axis2_pos);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis2Velocity)) value(s.axis2_vel);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis2Acceleration)) value(s.axis2_acc);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis5Pos)) value(s.axis5_pos);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis5Velocity)) value(s.axis5_vel);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis5Acceleration)) value(s.axis5_acc);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis6Pos)) value(s.axis6_pos);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis6Velocity)) value(s.axis6_vel);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis6Acceleration)) value(s.axis6_acc);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis7Pos)) value(s.axis7_pos);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis7Velocity)) value(s.axis7_vel);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis7Acceleration)) value(s.axis7_acc);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Cylinder1)) value(s.cylinder1);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Cylinder2)) value(s.cylinder2);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Cylinder3)) value(s.cylinder3);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Cylinder4)) value(s.cylinder4);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Fn1Raw)) value(s.fn1);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Ft1Raw)) value(s.ft1);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Fn2Raw)) value(s.fn2);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Ft2Raw)) value(s.ft2);
		optional_zero((field_mask & standalone_field_bit(StandaloneRecordField::Fn1Zeroed)) != 0, s.fn1, zero.value[0]);
		optional_zero((field_mask & standalone_field_bit(StandaloneRecordField::Ft1Zeroed)) != 0, s.ft1, zero.value[1]);
		optional_zero((field_mask & standalone_field_bit(StandaloneRecordField::Fn2Zeroed)) != 0, s.fn2, zero.value[2]);
		optional_zero((field_mask & standalone_field_bit(StandaloneRecordField::Ft2Zeroed)) != 0, s.ft2, zero.value[3]);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Fn1Sensor)) != 0, cal.side1.sensor_force_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Ft1Sensor)) != 0, cal.side1.sensor_tangential_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Fn1CalDelta)) != 0, cal.side1.force_cal_delta_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Fn1CalAbs)) != 0, cal.side1.force_cal_abs_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Ft1CalDelta)) != 0, cal.side1.ft_cal_delta_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Ft1CalAbs)) != 0, cal.side1.ft_cal_abs_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Torque1CalDelta)) != 0, cal.side1.torque_cal_delta_nmm);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Torque1CalAbs)) != 0, cal.side1.torque_cal_abs_nmm);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Fn1DecoupledDelta)) != 0, cal.side1.force_decoupled_delta_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Torque1DecoupledDelta)) != 0, cal.side1.torque_decoupled_delta_nmm);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Fn1DecoupledAbs)) != 0, cal.side1.force_decoupled_abs_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Torque1DecoupledAbs)) != 0, cal.side1.torque_decoupled_abs_nmm);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis2Angle)) value(s.axis2_pos);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Fn2Sensor)) != 0, cal.side2.sensor_force_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Ft2Sensor)) != 0, cal.side2.sensor_tangential_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Fn2CalDelta)) != 0, cal.side2.force_cal_delta_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Fn2CalAbs)) != 0, cal.side2.force_cal_abs_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Ft2CalDelta)) != 0, cal.side2.ft_cal_delta_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Ft2CalAbs)) != 0, cal.side2.ft_cal_abs_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Torque2CalDelta)) != 0, cal.side2.torque_cal_delta_nmm);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Torque2CalAbs)) != 0, cal.side2.torque_cal_abs_nmm);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Fn2DecoupledDelta)) != 0, cal.side2.force_decoupled_delta_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Torque2DecoupledDelta)) != 0, cal.side2.torque_decoupled_delta_nmm);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Fn2DecoupledAbs)) != 0, cal.side2.force_decoupled_abs_n);
		optional_cal((field_mask & standalone_field_bit(StandaloneRecordField::Torque2DecoupledAbs)) != 0, cal.side2.torque_decoupled_abs_nmm);
		if (field_mask & standalone_field_bit(StandaloneRecordField::Axis7Angle)) value(s.axis7_pos);
		rows << '\n';
		if (!has_sample_time_)
		{
			first_sample_time_us_ = s.time_us;
			has_sample_time_ = true;
		}
		last_sample_time_us_ = s.time_us;
		++sample_count_;
	}
	if (rows.str().empty()) return true;
	++block_count_;
	return enqueue_write(0, rows.str(), error);
}

bool ExperimentStreamRecorder::append_zero(const std::array<double, 4>& raw, std::uint64_t time_us,
	double axis2_angle_deg, double axis7_angle_deg, std::string& error)
{
	if (!zero_file_) { error = "零点文件未打开"; return false; }
	std::ostringstream row;
	row << zero_sample_count_++ << ',' << time_us << ',' << raw[0] << ',' << raw[1] << ',' << raw[2] << ',' << raw[3]
		<< ',' << axis2_angle_deg << ',' << axis7_angle_deg << '\n';
	return enqueue_write(2, row.str(), error);
}

bool ExperimentStreamRecorder::begin_zero(std::string& error)
{
	zero_start_time_local_ = now_iso();
	zero_end_time_local_.clear();
	return write_event("ZeroStart", 0, 0, 0, 0, error);
}

bool ExperimentStreamRecorder::reset_zero_file(std::string& error)
{
	if (!active_ || directory_.empty())
	{
		error = "实验记录尚未创建";
		return false;
	}
	if (!wait_for_queue(error)) return false;
	zero_file_.flush();
	zero_file_.close();
	zero_file_.open(std::filesystem::u8path(directory_) / L"zero_calibration.csv", std::ios::out | std::ios::trunc);
	if (!zero_file_) { error = "无法重置零点文件"; return false; }
	zero_sample_count_ = 0;
	return write_zero_header(error);
}

bool ExperimentStreamRecorder::finish_zero(const ForceZeroState& zero, std::string& error)
{
	zero_end_time_local_ = now_iso();
	return write_event("ZeroComplete", 0, zero.sample_count, 0, 0, error);
}

bool ExperimentStreamRecorder::write_json(const std::string& status, const std::string& reason,
	const ForceZeroState& zero, std::string& error)
{
	std::ofstream out(std::filesystem::u8path(directory_) / L"experiment.json", std::ios::out | std::ios::trunc);
	if (!out) { error = "无法创建experiment.json"; return false; }
	end_time_local_ = now_iso();
	const bool complete = status == "Completed" && reason.empty() && !failed();
	const bool overflow = reason.find("溢出") != std::string::npos;
	const bool sample_gap = reason.find("样本序号") != std::string::npos;
	const bool ads_error = reason.find("ADS") != std::string::npos;
	const std::string directory_name = std::filesystem::u8path(directory_).filename().u8string();
	out << std::setprecision(12) << "{\n"
		<< "  \"mode\": \"" << json_escape(mode_name_) << "\",\n"
		<< "  \"calibration_version\": \"two_stage_sensor_0_50g_plus_installed_2026-08-29\",\n"
		<< "  \"calibration_chain\": \"sensor_0_50g_then_installed_calibration\",\n"
		<< "  \"sensor_slopes_N_per_count\": {\n"
		<< "    \"fn1\": " << forcecal::kFn1SensorSlopeNPerCount << ",\n"
		<< "    \"ft1\": " << forcecal::kFt1SensorSlopeNPerCount << ",\n"
		<< "    \"fn2\": " << forcecal::kFn2SensorSlopeNPerCount << ",\n"
		<< "    \"ft2\": " << forcecal::kFt2SensorSlopeNPerCount << "\n"
		<< "  },\n"
		<< "  \"installation_axial_gain\": " << forcecal::kInstallationAxialGain << ",\n"
		<< "  \"installation_axial_bias_N\": " << forcecal::kInstallationAxialBiasN << ",\n"
		<< "  \"installation_torque_gain_Nmm_per_N\": " << forcecal::kInstallationTorqueGainNmmPerN << ",\n"
		<< "  \"installation_torque_bias_Nmm\": " << forcecal::kInstallationTorqueBiasNmm << ",\n"
		<< "  \"tangential_arm_mm\": " << forcecal::kTangentialArmMm << ",\n"
		<< "  \"decoupling_matrix\": [[" << forcecal::kDecouplingFf << ", " << forcecal::kDecouplingFt << "], ["
		<< forcecal::kDecouplingTf << ", " << forcecal::kDecouplingTt << "]],\n"
		<< "  \"units\": {\"realtime_fn\": \"N\", \"realtime_ft\": \"N\", \"csv_torque\": \"N·mm\"},\n"
		<< "  \"crosstalk_decoupling_applied\": true,\n"
		<< "  \"gravity_compensation_applied\": false,\n";
	if (standalone_mode_)
	{
		out << "  \"selected_field_mask\": \"0x" << std::hex << standalone_field_mask_ << std::dec << "\",\n";
	}
	if (program_mode_)
	{
		out << "  \"cylinder1_coupling_enabled\": " << (program_cylinder1_coupling_enabled_ ? "true" : "false") << ",\n"
			<< "  \"cylinder3_coupling_enabled\": " << (program_cylinder3_coupling_enabled_ ? "true" : "false") << ",\n";
	}
	out << "  \"directory_name\": \"" << json_escape(directory_name) << "\",\n"
		<< "  \"local_start_time\": \"" << json_escape(start_time_local_) << "\",\n"
		<< "  \"local_end_time\": \"" << json_escape(end_time_local_) << "\",\n"
		<< "  \"status\": \"" << json_escape(status) << "\",\n"
		<< "  \"reason\": \"" << json_escape(reason) << "\",\n"
		<< "  \"sample_count\": " << sample_count_ << ",\n"
		<< "  \"duration_s\": " << (has_sample_time_ ? static_cast<double>(last_sample_time_us_ - first_sample_time_us_) / 1000000.0 : 0.0) << ",\n"
		<< "  \"block_count\": " << block_count_ << ",\n"
		<< "  \"data_complete\": " << (complete ? "true" : "false") << ",\n"
		<< "  \"record_overflow\": " << (overflow ? "true" : "false") << ",\n"
		<< "  \"sample_gap\": " << (sample_gap ? "true" : "false") << ",\n"
		<< "  \"ads_error\": " << (ads_error ? "true" : "false") << ",\n"
		<< "  \"zero_done\": " << (zero.done ? "true" : "false") << ",\n"
		<< "  \"zero_error_id\": " << zero.error_id << ",\n"
		<< "  \"zero_sample_count\": " << zero.sample_count << ",\n"
		<< "  \"zero_start_time\": \"" << json_escape(zero_start_time_local_) << "\",\n"
		<< "  \"zero_end_time\": \"" << json_escape(zero_end_time_local_) << "\",\n"
		<< "  \"zero_axis2_angle_deg\": " << zero.axis2_angle_deg << ",\n"
		<< "  \"zero_axis7_angle_deg\": " << zero.axis7_angle_deg << ",\n"
		<< "  \"fn1_zero\": " << zero.value[0] << ",\n"
		<< "  \"ft1_zero\": " << zero.value[1] << ",\n"
		<< "  \"fn2_zero\": " << zero.value[2] << ",\n"
		<< "  \"ft2_zero\": " << zero.value[3] << ",\n"
		<< "  \"fn1_zero_std\": " << zero.standard_deviation[0] << ",\n"
		<< "  \"ft1_zero_std\": " << zero.standard_deviation[1] << ",\n"
		<< "  \"fn2_zero_std\": " << zero.standard_deviation[2] << ",\n"
		<< "  \"ft2_zero_std\": " << zero.standard_deviation[3] << "\n}\n";
	return static_cast<bool>(out);
}

bool ExperimentStreamRecorder::finalize(const std::string& status, const std::string& reason,
	const ForceZeroState& zero, std::string& error)
{
	try
	{
		if (!active_ && directory_.empty()) return true;
		std::string event_error;
		const bool event_ok = write_event("RecordStop", 0, 0, 0, 0, event_error);
		{
			std::lock_guard<std::mutex> lock(writer_mutex_);
			writer_stop_requested_ = true;
		}
		writer_cv_.notify_all();
		if (writer_thread_.joinable()) writer_thread_.join();
		if (!event_ok && error.empty()) error = event_error;
		{
			std::lock_guard<std::mutex> lock(writer_mutex_);
			if (!writer_error_.empty() && error.empty()) error = writer_error_;
		}
		samples_.flush();
		events_.flush();
		zero_file_.flush();
		const bool json_ok = write_json(status, reason, zero, event_error);
		if (!json_ok && error.empty()) error = event_error;
		samples_.close();
		events_.close();
		zero_file_.close();
		active_ = false;
		const bool ok = event_ok && json_ok && error.empty();
		archived_ = ok;
		return ok;
	}
	catch (const std::exception& ex)
	{
		error = std::string("结束实验记录异常：") + ex.what();
		last_error_ = error;
		active_ = false;
		archived_ = false;
		try
		{
			if (writer_thread_.joinable())
			{
				{
					std::lock_guard<std::mutex> lock(writer_mutex_);
					writer_stop_requested_ = true;
				}
				writer_cv_.notify_all();
				writer_thread_.join();
			}
		}
		catch (...) { }
		samples_.close();
		events_.close();
		zero_file_.close();
		return false;
	}
}
