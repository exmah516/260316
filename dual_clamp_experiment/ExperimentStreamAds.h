#pragma once

#include "ADSComm1.h"
#include "ExperimentStreamRecorder.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

struct ExperimentStreamSample
{
	std::uint32_t index = 0;
	std::uint64_t time_us = 0;
	std::uint8_t phase = 0;
	std::uint32_t event_sequence = 0;
	std::uint16_t cycle_index = 0;
	double axis1_pos = 0.0, axis1_vel = 0.0, axis1_acc = 0.0;
	double axis2_pos = 0.0, axis2_vel = 0.0, axis2_acc = 0.0;
	double axis5_pos = 0.0, axis5_vel = 0.0, axis5_acc = 0.0;
	double axis6_pos = 0.0, axis6_vel = 0.0, axis6_acc = 0.0;
	double axis7_pos = 0.0, axis7_vel = 0.0, axis7_acc = 0.0;
	std::uint16_t cylinder1 = 0, cylinder2 = 0, cylinder3 = 0, cylinder4 = 0;
	short fn1 = 0, ft1 = 0, fn2 = 0, ft2 = 0;
};

struct ExperimentStreamStatus
{
	bool recording = false;
	bool overflow = false;
	std::uint32_t total_count = 0;
	std::uint32_t error_id = 0;
	std::uint8_t source_mode = 0;
	std::array<bool, 2> block_ready{};
	std::array<std::uint32_t, 2> block_sequence{};
	std::array<std::uint16_t, 2> block_count{};
	ForceZeroState zero{};
};

class ExperimentStreamAds
{
public:
	ExperimentStreamAds();
	~ExperimentStreamAds();

	bool open();
	void close();
	bool is_open() const;
	std::string last_error() const;
	bool reset_recording();
	// 请求PLC停止当前实时记录；用于实验状态机已进入终态但等待最后一块数据交付的收尾阶段。
	bool stop_recording();
	bool request_zero();
	bool invalidate_zero();
	bool read_status(ExperimentStreamStatus& status);
	bool read_block(int slot, std::vector<ExperimentStreamSample>& samples, std::uint32_t& sequence);
	bool acknowledge_block(int slot, std::uint32_t sequence);
	bool read_zero_samples(std::vector<std::array<double, 4>>& samples);

private:
	CADSComm comm_;
};
