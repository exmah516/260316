// 文件职责说明：
// 远程手柄数据桥。主端手柄采样经远程网关写入这里，从端 Handle 类（远程模式）从这里读取；
// 从端算法写出的力/力矩指令也存放在这里，由网关通过触觉帧回传主端。
// 只依赖标准库，线程安全（网关 UDP 线程写入，主循环线程读取）。
#pragma once

#include <windows.h>

#include <cstdint>
#include <mutex>

namespace remote_handle_bridge
{
	// 物理序列号 -> 槽位。与协议中的手柄 A / B 一致。
	constexpr DWORD kSerialA = 582;
	constexpr DWORD kSerialB = 587;

	inline int slot_of(DWORD serial)
	{
		if (serial == kSerialA) return 0;
		if (serial == kSerialB) return 1;
		return -1;
	}

	struct Sample
	{
		bool valid = false;
		ULONGLONG rx_ms = 0;
		std::uint8_t buttons = 0;
		std::int32_t encoders[2] = {};
		float joints[2] = {};
		float vels[2] = {};
	};

	struct Output
	{
		bool enable = false;
		int axis = 0;
		double force = 0.0;
		double torque = 0.0;
	};

	namespace detail
	{
		struct State
		{
			std::mutex m;
			Sample samples[2];
			Output outputs[2];
		};
		inline State& state()
		{
			static State s;
			return s;
		}
	}

	// ---- 采样：网关写入，Handle 读取 ----
	inline void put_sample(int slot, Sample s)
	{
		if (slot < 0 || slot > 1) return;
		s.rx_ms = GetTickCount64();
		auto& st = detail::state();
		std::lock_guard<std::mutex> lock(st.m);
		st.samples[slot] = s;
	}

	// 取最近一帧；超过 max_age_ms 或标记无效则返回 false。
	inline bool get_sample(int slot, Sample& out, ULONGLONG max_age_ms)
	{
		if (slot < 0 || slot > 1) return false;
		auto& st = detail::state();
		std::lock_guard<std::mutex> lock(st.m);
		const Sample& s = st.samples[slot];
		if (!s.valid) return false;
		if (GetTickCount64() - s.rx_ms > max_age_ms) return false;
		out = s;
		return true;
	}

	inline void clear_samples()
	{
		auto& st = detail::state();
		std::lock_guard<std::mutex> lock(st.m);
		st.samples[0] = Sample();
		st.samples[1] = Sample();
	}

	// ---- 力输出：Handle 写入，网关读取并回传 ----
	inline void set_output(int slot, const Output& o)
	{
		if (slot < 0 || slot > 1) return;
		auto& st = detail::state();
		std::lock_guard<std::mutex> lock(st.m);
		st.outputs[slot] = o;
	}

	inline Output get_output(int slot)
	{
		if (slot < 0 || slot > 1) return Output();
		auto& st = detail::state();
		std::lock_guard<std::mutex> lock(st.m);
		return st.outputs[slot];
	}

	inline void clear_outputs()
	{
		auto& st = detail::state();
		std::lock_guard<std::mutex> lock(st.m);
		st.outputs[0] = Output();
		st.outputs[1] = Output();
	}
}
