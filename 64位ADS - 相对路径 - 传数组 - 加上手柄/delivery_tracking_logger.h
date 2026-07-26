#pragma once

#include "delivery_tracking.h"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

// 主从位移实验独立 CSV 写入器。主循环只 enqueue，磁盘写入在 writer 线程完成。
class DeliveryTrackingLogger
{
public:
	DeliveryTrackingLogger();
	~DeliveryTrackingLogger();

	bool start_session(const std::string& output_dir);
	void stop_session();
	bool is_running() const { return running_.load(); }
	std::uint64_t session_id() const { return session_id_.load(); }
	std::uint64_t dropped_count() const { return dropped_.load(); }

	struct Row
	{
		std::uint64_t tick_ms = 0;
		std::uint64_t session_id = 0;
		int guidewire_mode = 0;
		int axis1_phase = 0;
		int axis6_phase = 0;
		unsigned short cylinder_cmd[4] = {};
		bool compensation_enabled = false;
		bool force_sample_valid = false;
		bool calibration_zeroed = false;
		bool freeze_active = false;
		bool plc_hold_active = false;
		bool ads_return_fault = false;
		bool spacing_recovery_active = false;
		bool force_transition_active = false;
		bool manual_cylinder_override = false;
		double fn_1_raw_v = 0.0;
		double ft_1_raw_v = 0.0;
		double fn_1_zero_v = 0.0;
		double ft_1_zero_v = 0.0;
		double calibrated_force_n = 0.0;
		double calibrated_torque_nm = 0.0;
		std::uint64_t logger_dropped = 0;
		DeliveryTrackingAxisSnapshot axis1;
		DeliveryTrackingAxisSnapshot axis6;
	};

	void enqueue(const Row& row);

private:
	void writer_loop();
	bool open_file(const std::string& output_dir);
	void close_file();
	void write_header();

	static constexpr std::size_t kCapacity = 4096;
	std::vector<Row> ring_;
	std::atomic<std::size_t> head_{ 0 };
	std::atomic<std::size_t> tail_{ 0 };
	std::atomic<bool> running_{ false };
	std::atomic<bool> stop_requested_{ false };
	std::atomic<std::uint64_t> dropped_{ 0 };
	std::atomic<std::uint64_t> session_id_{ 0 };

	std::thread writer_thread_;
	std::FILE* fp_ = nullptr;
};
