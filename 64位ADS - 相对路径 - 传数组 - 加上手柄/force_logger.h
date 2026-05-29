// 文件职责说明：
// 1) 高频力数据 CSV 记录器：传感器线程入队，独立 writer 线程落盘。
// 2) 主线程仅做"发布最新轴位置快照"这一原子写，不参与 IO。
// 3) 列：tick_ms, axis1/2/6/7 绝对位置(mm), AIN0/AIN1 原始电压(V)。
#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

class ForceLogger
{
public:
	ForceLogger();
	~ForceLogger();

	// 在指定目录下创建 Force_sensor_YYYYMMDD_HHMMSS.csv，启动 writer 线程。
	bool start(const std::string& output_dir);
	void stop();
	bool is_running() const { return running_.load(); }

	// 主线程每周期调用：发布最新轴位置（绝对坐标 mm），供后续传感器样本读取。
	void publish_axis_snapshot(double a1_abs, double a2_abs, double a6_abs, double a7_abs);

	// 传感器线程回调：每帧入队一次。
	void on_sensor_sample(std::uint64_t tick_ms, const double v[6]);

	std::uint64_t dropped_count() const { return dropped_.load(); }

private:
	struct Row
	{
		std::uint64_t tick_ms;
		double axis_abs[4];
		double v0;
		double v1;
	};

	void writer_loop();
	bool open_file(const std::string& output_dir);
	void close_file();

	static constexpr std::size_t kCapacity = 4096;
	std::vector<Row> ring_;
	std::atomic<std::size_t> head_{ 0 };
	std::atomic<std::size_t> tail_{ 0 };

	std::atomic<double> axis_snapshot_[4]{};

	std::atomic<bool> running_{ false };
	std::atomic<bool> stop_requested_{ false };
	std::atomic<std::uint64_t> dropped_{ 0 };

	std::thread writer_thread_;
	std::FILE* fp_ = nullptr;
};
