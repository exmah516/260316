// 文件职责说明：
// 1) 提供无 UI 依赖的 TCP 采集客户端（采集卡 -> 上位机）。
// 2) 后台线程循环采集，主线程通过 get_latest_raw 非阻塞取最新帧。
// 3) 内置断线重连与线程安全缓存，避免主控制环阻塞。
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

class TcpForceDaqClient
{
public:
	TcpForceDaqClient() = default;
	~TcpForceDaqClient();

	bool start(const std::string& ip, unsigned short port);
	void stop();

	// 读取最近一帧原始电压（V1~V6）；若当前无有效帧则返回 false。
	bool get_latest_raw(double out_v[6], std::uint64_t& timestamp_ms) const;
	// 日志专用接口：返回第0/1通道，分别映射为 ft_1 / fn_1。
	bool get_latest_ft1_fn1(double& out_ft1, double& out_fn1, std::uint64_t& timestamp_ms) const;
	bool is_running() const { return running_.load(); }

private:
	void worker_loop(std::string ip, unsigned short port);

	mutable std::mutex frame_mutex_;
	bool has_frame_ = false;
	double latest_v_[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	std::uint64_t latest_tick_ms_ = 0;

	std::atomic<bool> running_{ false };
	std::atomic<bool> stop_requested_{ false };
	std::thread worker_;
};
