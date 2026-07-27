// 文件职责说明：主从位移实验 20 Hz CSV 的异步落盘实现。
// 使用 SPSC 环形队列，writer 线程落后时丢弃最新行而不阻塞运动控制循环。
#include "delivery_tracking_logger.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <thread>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace
{
	void write_axis_columns(std::FILE* fp, const DeliveryTrackingAxisSnapshot& axis)
	{
		std::fprintf(fp,
			",%d,%d,%d,%llu,"
			"%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,"
			"%.8f,%.8f,%.8f,%d,"
			"%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%d",
			axis.segment_active ? 1 : 0,
			axis.grip_assumed ? 1 : 0,
			axis.invalid_reason,
			static_cast<unsigned long long>(axis.segment_id),
			axis.handle_raw,
			axis.handle_filtered,
			axis.raw_mapping_increment_axis_mm,
			axis.nominal_forward_increment_mm,
			axis.compensated_requested_forward_increment_mm,
			axis.effective_forward_increment_mm,
			axis.handover_forward_increment_mm,
			axis.session_handover_forward_mm,
			axis.compensation_gain,
			axis.p_term,
			axis.i_term,
			axis.integral_limited ? 1 : 0,
			axis.refer_rel_mm,
			axis.actual_rel_mm,
			axis.actual_forward_delta_mm,
			axis.segment_master_forward_mm,
			axis.segment_actual_forward_mm,
			axis.session_master_forward_mm,
			axis.session_actual_forward_mm,
			axis.tracking_error_mm,
			axis.tracking_error_limited ? 1 : 0);
	}
}

DeliveryTrackingLogger::DeliveryTrackingLogger()
	: ring_(kCapacity)
{
}

DeliveryTrackingLogger::~DeliveryTrackingLogger()
{
	stop_session();
}

bool DeliveryTrackingLogger::start_session(const std::string& output_dir)
{
	if (running_.load())
	{
		return true;
	}
	if (!open_file(output_dir))
	{
		return false;
	}
	stop_requested_.store(false);
	head_.store(0, std::memory_order_relaxed);
	tail_.store(0, std::memory_order_relaxed);
	dropped_.store(0, std::memory_order_relaxed);
	session_id_.fetch_add(1, std::memory_order_relaxed);
	write_header();
	try
	{
		writer_thread_ = std::thread(&DeliveryTrackingLogger::writer_loop, this);
	}
	catch (...)
	{
		close_file();
		return false;
	}
	running_.store(true);
	return true;
}

void DeliveryTrackingLogger::stop_session()
{
	if (!running_.load() && !writer_thread_.joinable())
	{
		return;
	}
	stop_requested_.store(true);
	if (writer_thread_.joinable())
	{
		writer_thread_.join();
	}
	close_file();
	running_.store(false);
}

void DeliveryTrackingLogger::enqueue(const Row& row)
{
	if (!running_.load(std::memory_order_acquire))
	{
		return;
	}
	const std::size_t head = head_.load(std::memory_order_relaxed);
	const std::size_t next = (head + 1) % kCapacity;
	if (next == tail_.load(std::memory_order_acquire))
	{
		dropped_.fetch_add(1, std::memory_order_relaxed);
		return;
	}
	ring_[head] = row;
	head_.store(next, std::memory_order_release);
}

bool DeliveryTrackingLogger::open_file(const std::string& output_dir)
{
	if (!output_dir.empty() && output_dir != ".")
	{
		CreateDirectoryA(output_dir.c_str(), nullptr);
	}

	std::time_t now = std::time(nullptr);
	std::tm local_tm{};
#if defined(_WIN32)
	localtime_s(&local_tm, &now);
#else
	local_tm = *std::localtime(&now);
#endif
	char stamp[32] = {};
	std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local_tm);

	std::string path = output_dir;
	if (!path.empty() && path.back() != '/' && path.back() != '\\')
	{
		path += '/';
	}
	path += "MasterSlaveTracking_";
	path += stamp;
	path += ".csv";

#if defined(_WIN32)
	if (fopen_s(&fp_, path.c_str(), "wb") != 0)
	{
		fp_ = nullptr;
		return false;
	}
#else
	fp_ = std::fopen(path.c_str(), "wb");
	if (!fp_)
	{
		return false;
	}
#endif
	return true;
}

void DeliveryTrackingLogger::close_file()
{
	if (fp_)
	{
		std::fflush(fp_);
		std::fclose(fp_);
		fp_ = nullptr;
	}
}

void DeliveryTrackingLogger::write_header()
{
	const char* axis1_columns =
		"axis1_segment_active,axis1_grip_assumed,axis1_invalid_reason,axis1_segment_id,"
		"axis1_handle_raw,axis1_handle_filtered,axis1_raw_mapping_increment_axis_mm,"
		"axis1_nominal_forward_increment_mm,axis1_compensated_requested_forward_increment_mm,axis1_effective_forward_increment_mm,"
		"axis1_handover_forward_increment_mm,axis1_session_handover_forward_mm,"
		"axis1_compensation_gain,axis1_p_term,axis1_i_term,axis1_integral_limited,"
		"axis1_refer_rel_mm,axis1_actual_rel_mm,axis1_actual_forward_delta_mm,"
		"axis1_segment_master_forward_mm,axis1_segment_actual_forward_mm,"
		"axis1_session_master_forward_mm,axis1_session_actual_forward_mm,axis1_tracking_error_mm,axis1_tracking_error_limited";
	const char* axis6_columns =
		"axis6_segment_active,axis6_grip_assumed,axis6_invalid_reason,axis6_segment_id,"
		"axis6_handle_raw,axis6_handle_filtered,axis6_raw_mapping_increment_axis_mm,"
		"axis6_nominal_forward_increment_mm,axis6_compensated_requested_forward_increment_mm,axis6_effective_forward_increment_mm,"
		"axis6_handover_forward_increment_mm,axis6_session_handover_forward_mm,"
		"axis6_compensation_gain,axis6_p_term,axis6_i_term,axis6_integral_limited,"
		"axis6_refer_rel_mm,axis6_actual_rel_mm,axis6_actual_forward_delta_mm,"
		"axis6_segment_master_forward_mm,axis6_segment_actual_forward_mm,"
		"axis6_session_master_forward_mm,axis6_session_actual_forward_mm,axis6_tracking_error_mm,axis6_tracking_error_limited";
	if (!fp_)
	{
		return;
	}
	std::fprintf(fp_,
		"tick_ms,session_id,guidewire_mode,axis1_phase,axis6_phase,"
		"cyl1_cmd,cyl2_cmd,cyl3_cmd,cyl4_cmd,"
		"compensation_enabled,force_sample_valid,calibration_zeroed,"
		"freeze_active,plc_hold_active,ads_return_fault,spacing_recovery_active,"
		"force_transition_active,manual_cylinder_override,"
		"fn_1_raw_v,ft_1_raw_v,fn_1_zero_v,ft_1_zero_v,calibrated_force_n,calibrated_torque_nm,logger_dropped,");
	std::fprintf(fp_, "%s,%s\n", axis1_columns, axis6_columns);
	std::fflush(fp_);
}

void DeliveryTrackingLogger::writer_loop()
{
	std::size_t flush_counter = 0;
	while (true)
	{
		const std::size_t tail = tail_.load(std::memory_order_relaxed);
		const std::size_t head = head_.load(std::memory_order_acquire);
		if (tail == head)
		{
			if (stop_requested_.load(std::memory_order_acquire))
			{
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			continue;
		}

		const Row& row = ring_[tail];
		if (fp_)
		{
			std::fprintf(fp_,
				"%llu,%llu,%d,%d,%d,%u,%u,%u,%u,"
				"%d,%d,%d,%d,%d,%d,%d,%d,%d,"
				"%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%llu",
				static_cast<unsigned long long>(row.tick_ms),
				static_cast<unsigned long long>(row.session_id),
				row.guidewire_mode,
				row.axis1_phase,
				row.axis6_phase,
				static_cast<unsigned int>(row.cylinder_cmd[0]),
				static_cast<unsigned int>(row.cylinder_cmd[1]),
				static_cast<unsigned int>(row.cylinder_cmd[2]),
				static_cast<unsigned int>(row.cylinder_cmd[3]),
				row.compensation_enabled ? 1 : 0,
				row.force_sample_valid ? 1 : 0,
				row.calibration_zeroed ? 1 : 0,
				row.freeze_active ? 1 : 0,
				row.plc_hold_active ? 1 : 0,
				row.ads_return_fault ? 1 : 0,
				row.spacing_recovery_active ? 1 : 0,
				row.force_transition_active ? 1 : 0,
				row.manual_cylinder_override ? 1 : 0,
				row.fn_1_raw_v,
				row.ft_1_raw_v,
				row.fn_1_zero_v,
				row.ft_1_zero_v,
				row.calibrated_force_n,
				row.calibrated_torque_nm,
				static_cast<unsigned long long>(row.logger_dropped));
			write_axis_columns(fp_, row.axis1);
			write_axis_columns(fp_, row.axis6);
			std::fputc('\n', fp_);
		}

		tail_.store((tail + 1) % kCapacity, std::memory_order_release);
		if (++flush_counter >= 64)
		{
			if (fp_)
			{
				std::fflush(fp_);
			}
			flush_counter = 0;
		}
	}
	if (fp_)
	{
		std::fflush(fp_);
	}
}
