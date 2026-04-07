// 文件职责说明：
// 1) 实现采集卡 TCP 协议轮询（与 MFC_ITCP_Use 的查询帧一致）。
// 2) 收包解析 6 路 INT16 原始值并换算为电压（/1000.0）。
// 3) 对外仅暴露最近一帧，异常时自动重连，不阻塞主控制循环。
#include "tcp_force_daq.h"

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <algorithm>
#include <array>
#include <cstring>

#pragma comment(lib, "Ws2_32.lib")

namespace
{
	constexpr int kResponseSize = 25;
	constexpr int kChannelCount = 6;
	constexpr DWORD kIoTimeoutMs = 300;
	constexpr DWORD kReconnectDelayMs = 200;
	constexpr unsigned char kQueryCmd[12] = {
		0x00, 0x03, 0x00, 0x00, 0x00, 0x06, 0x01, 0x04, 0x00, 0x40, 0x00, 0x08
	};

	bool send_all(SOCKET sock, const char* data, int total)
	{
		int sent = 0;
		while (sent < total)
		{
			const int n = send(sock, data + sent, total - sent, 0);
			if (n == SOCKET_ERROR || n == 0)
			{
				return false;
			}
			sent += n;
		}
		return true;
	}

	bool recv_exact(SOCKET sock, char* data, int total)
	{
		int received = 0;
		while (received < total)
		{
			const int n = recv(sock, data + received, total - received, 0);
			if (n == SOCKET_ERROR || n == 0)
			{
				return false;
			}
			received += n;
		}
		return true;
	}

	bool parse_response_to_voltage(const char* recv_buf, double out_v[6])
	{
		for (int i = 0; i < kChannelCount; ++i)
		{
			const unsigned char hi = static_cast<unsigned char>(recv_buf[i * 2 + 9]);
			const unsigned char lo = static_cast<unsigned char>(recv_buf[i * 2 + 10]);
			const short raw = static_cast<short>((static_cast<unsigned short>(hi) << 8) | lo);
			out_v[i] = static_cast<double>(raw) / 1000.0;
		}
		return true;
	}
}

TcpForceDaqClient::~TcpForceDaqClient()
{
	stop();
}

bool TcpForceDaqClient::start(const std::string& ip, unsigned short port)
{
	if (running_.load())
	{
		return true;
	}

	stop_requested_.store(false);
	try
	{
		worker_ = std::thread(&TcpForceDaqClient::worker_loop, this, ip, port);
	}
	catch (...)
	{
		running_.store(false);
		return false;
	}
	running_.store(true);
	return true;
}

void TcpForceDaqClient::stop()
{
	stop_requested_.store(true);
	if (worker_.joinable())
	{
		worker_.join();
	}
	running_.store(false);
}

bool TcpForceDaqClient::get_latest_raw(double out_v[6], std::uint64_t& timestamp_ms) const
{
	std::lock_guard<std::mutex> lock(frame_mutex_);
	if (!has_frame_)
	{
		return false;
	}
	for (int i = 0; i < kChannelCount; ++i)
	{
		out_v[i] = latest_v_[i];
	}
	timestamp_ms = latest_tick_ms_;
	return true;
}

bool TcpForceDaqClient::get_latest_ft1_fn1(double& out_ft1, double& out_fn1, std::uint64_t& timestamp_ms) const
{
	double raw_v[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	if (!get_latest_raw(raw_v, timestamp_ms))
	{
		return false;
	}
	out_ft1 = raw_v[0];
	out_fn1 = raw_v[1];
	return true;
}

void TcpForceDaqClient::worker_loop(std::string ip, unsigned short port)
{
	WSADATA wsa_data;
	if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0)
	{
		running_.store(false);
		return;
	}

	while (!stop_requested_.load())
	{
		SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (sock == INVALID_SOCKET)
		{
			Sleep(kReconnectDelayMs);
			continue;
		}

		// 防止网络抖动时 Send/Recv 永久阻塞。
		setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&kIoTimeoutMs), sizeof(kIoTimeoutMs));
		setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&kIoTimeoutMs), sizeof(kIoTimeoutMs));

		sockaddr_in server_addr;
		std::memset(&server_addr, 0, sizeof(server_addr));
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(port);
		if (inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) != 1)
		{
			closesocket(sock);
			Sleep(kReconnectDelayMs);
			continue;
		}

		if (connect(sock, reinterpret_cast<const sockaddr*>(&server_addr), sizeof(server_addr)) == SOCKET_ERROR)
		{
			closesocket(sock);
			Sleep(kReconnectDelayMs);
			continue;
		}

		while (!stop_requested_.load())
		{
			if (!send_all(sock, reinterpret_cast<const char*>(kQueryCmd), static_cast<int>(sizeof(kQueryCmd))))
			{
				break;
			}

			char recv_buf[kResponseSize] = { 0 };
			if (!recv_exact(sock, recv_buf, kResponseSize))
			{
				break;
			}

			double parsed_v[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			if (!parse_response_to_voltage(recv_buf, parsed_v))
			{
				break;
			}

			{
				std::lock_guard<std::mutex> lock(frame_mutex_);
				for (int i = 0; i < kChannelCount; ++i)
				{
					latest_v_[i] = parsed_v[i];
				}
				latest_tick_ms_ = static_cast<std::uint64_t>(GetTickCount64());
				has_frame_ = true;
			}
		}

		closesocket(sock);
		if (!stop_requested_.load())
		{
			Sleep(kReconnectDelayMs);
		}
	}

	WSACleanup();
	running_.store(false);
}
