// 文件职责说明：
// 1) SPSC 环形缓冲：传感器线程是唯一生产者，writer 线程是唯一消费者。
// 2) 缓冲容量 4096 行；溢出策略：drop newest，仅累计计数避免阻塞生产者。
// 3) writer 空闲时 sleep 2ms；每 64 行 fflush 一次。
#include "force_logger.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <thread>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

ForceLogger::ForceLogger()
	: ring_(kCapacity)
{
	for (int i = 0; i < 4; ++i)
	{
		axis_snapshot_[i].store(0.0, std::memory_order_relaxed);
	}
}

ForceLogger::~ForceLogger()
{
	stop();
}

bool ForceLogger::start(const std::string& output_dir)
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
	try
	{
		writer_thread_ = std::thread(&ForceLogger::writer_loop, this);
	}
	catch (...)
	{
		close_file();
		return false;
	}
	running_.store(true);
	return true;
}

void ForceLogger::stop()
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

void ForceLogger::publish_axis_snapshot(double a1_abs, double a2_abs, double a6_abs, double a7_abs)
{
	axis_snapshot_[0].store(a1_abs, std::memory_order_relaxed);
	axis_snapshot_[1].store(a2_abs, std::memory_order_relaxed);
	axis_snapshot_[2].store(a6_abs, std::memory_order_relaxed);
	axis_snapshot_[3].store(a7_abs, std::memory_order_relaxed);
}

void ForceLogger::on_sensor_sample(std::uint64_t tick_ms, const double v[6])
{
	if (!running_.load(std::memory_order_acquire))
	{
		return;
	}
	const std::size_t head = head_.load(std::memory_order_relaxed);
	const std::size_t next = (head + 1) % kCapacity;
	if (next == tail_.load(std::memory_order_acquire))
	{
		// 缓冲满：writer 跟不上 → 丢最新样本，避免阻塞 DAQ 线程。
		dropped_.fetch_add(1, std::memory_order_relaxed);
		return;
	}
	Row& row = ring_[head];
	row.tick_ms = tick_ms;
	for (int i = 0; i < 4; ++i)
	{
		row.axis_abs[i] = axis_snapshot_[i].load(std::memory_order_relaxed);
	}
	row.v0 = v[0];
	row.v1 = v[1];
	head_.store(next, std::memory_order_release);
}

bool ForceLogger::open_file(const std::string& output_dir)
{
	if (!output_dir.empty() && output_dir != ".")
	{
		CreateDirectoryA(output_dir.c_str(), nullptr);
	}

	std::time_t t = std::time(nullptr);
	std::tm tm_local;
#if defined(_WIN32)
	localtime_s(&tm_local, &t);
#else
	tm_local = *std::localtime(&t);
#endif
	char stamp[32] = { 0 };
	std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_local);

	std::string path = output_dir;
	if (!path.empty() && path.back() != '/' && path.back() != '\\')
	{
		path += '/';
	}
	path += "Force_sensor_";
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
	const char* header = "tick_ms,axis1_pos_abs_mm,axis2_pos_abs_mm,axis6_pos_abs_mm,axis7_pos_abs_mm,ain0_raw_v,ain1_raw_v\n";
	std::fwrite(header, 1, std::strlen(header), fp_);
	std::fflush(fp_);
	return true;
}

void ForceLogger::close_file()
{
	if (fp_)
	{
		std::fflush(fp_);
		std::fclose(fp_);
		fp_ = nullptr;
	}
}

void ForceLogger::writer_loop()
{
	std::size_t flush_counter = 0;
	char buf[256];
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
		const int n = std::snprintf(buf, sizeof(buf),
			"%llu,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f\n",
			static_cast<unsigned long long>(row.tick_ms),
			row.axis_abs[0], row.axis_abs[1], row.axis_abs[2], row.axis_abs[3],
			row.v0, row.v1);
		if (n > 0 && fp_)
		{
			std::fwrite(buf, 1, static_cast<std::size_t>(n), fp_);
		}
		tail_.store((tail + 1) % kCapacity, std::memory_order_release);
		if (++flush_counter >= 64)
		{
			std::fflush(fp_);
			flush_counter = 0;
		}
	}
	if (fp_)
	{
		std::fflush(fp_);
	}
}
