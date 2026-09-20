#pragma once

#include <windows.h>
#include <iostream>
#include <filesystem>
#include <vector>

namespace handle582
{

class Handle582Feedback
{
public:
	explicit Handle582Feedback(DWORD serial = 582)
		: serial_(serial)
	{
	}

	~Handle582Feedback()
	{
		close();
	}

	// 初始化手柄连接
	bool init()
	{
		if (connected_) return true;

		if (!load_dll())
		{
			std::cout << "[手柄582] 未找到或无法加载 FLCatheter.dll，手柄力反馈未激活。" << std::endl;
			return false;
		}

		device_id_ = fn_openDevice_(serial_);
		if (device_id_ < 0)
		{
			std::cout << "[手柄582] 打开设备失败（SN: " << serial_ << "），请确认USB连接或驱动正常。" << std::endl;
			return false;
		}

		int sn_actual = fn_getSerialNumber_ ? fn_getSerialNumber_(device_id_) : (int)serial_;
		std::cout << "[手柄582] 成功连接设备！实际 SN: " << sn_actual << "（请求: " << serial_ << "），启动力反馈伺服循环。" << std::endl;

		if (fn_startServoLoop_)
		{
			fn_startServoLoop_(sync_callback, nullptr);
			servo_loop_started_ = true;
		}

		if (fn_enableForces_)
		{
			fn_enableForces_(true, device_id_);
		}

		connected_ = true;
		last_sent_force_ = 0.0;
		return true;
	}

	// 极简力反馈更新：仅在力传感器归零后才输出实际力，未归零时输出 0.0 N
	void update(bool zero_valid, double force_n)
	{
		if (!connected_ || device_id_ < 0 || !fn_sendForce_) return;

		double target_force = 0.0;
		if (zero_valid)
		{
			// 归零有效后，将真实重构力输出至手柄
			target_force = force_n;
		}

		// 轴1为SDK轴向力对应通道 (fForce[1])
		double fForce[3] = { 0.0, target_force, 0.0 };
		fn_sendForce_(fForce, 0.0, device_id_);
		last_sent_force_ = target_force;
	}

	void reset()
	{
		if (!connected_ || device_id_ < 0 || !fn_sendForce_) return;
		double fForce[3] = { 0.0, 0.0, 0.0 };
		fn_sendForce_(fForce, 0.0, device_id_);
		last_sent_force_ = 0.0;
	}

	void close()
	{
		if (!connected_ && dll_handle_ == nullptr) return;

		if (device_id_ >= 0)
		{
			reset();
			if (fn_enableForces_) fn_enableForces_(false, device_id_);
			device_id_ = -1;
		}

		if (servo_loop_started_ && fn_stopServoLoop_)
		{
			fn_stopServoLoop_();
			servo_loop_started_ = false;
		}

		if (fn_closeDevice_)
		{
			fn_closeDevice_();
		}

		if (dll_handle_)
		{
			FreeLibrary(dll_handle_);
			dll_handle_ = nullptr;
		}

		connected_ = false;
	}

	bool is_connected() const { return connected_; }
	double last_sent_force() const { return last_sent_force_; }
	DWORD serial() const { return serial_; }

private:
	static int __stdcall sync_callback(void* /*param*/)
	{
		return 0;
	}

	bool load_dll()
	{
		if (dll_handle_) return true;

		// 候选路径：当前工作目录、同级模块目录、以及相对引用工程目录
		const std::vector<std::wstring> candidate_paths = {
			L"FLCatheter.dll",
			L".\\FLCatheter.dll",
			L"..\\64位ADS - 相对路径 - 传数组 - 加上手柄\\FLCatheter.dll",
			L"..\\..\\64位ADS - 相对路径 - 传数组 - 加上手柄\\FLCatheter.dll"
		};

		for (const auto& path : candidate_paths)
		{
			dll_handle_ = LoadLibraryW(path.c_str());
			if (dll_handle_) break;
		}

		if (!dll_handle_)
		{
			// 尝试从当前 exe 所在目录加载
			wchar_t module_path[MAX_PATH] = { 0 };
			if (GetModuleFileNameW(nullptr, module_path, MAX_PATH) > 0)
			{
				std::filesystem::path p(module_path);
				std::filesystem::path dll_path = p.parent_path() / L"FLCatheter.dll";
				dll_handle_ = LoadLibraryW(dll_path.c_str());
			}
		}

		if (!dll_handle_) return false;

		fn_openDevice_ = reinterpret_cast<FnOpenDevice>(GetProcAddress(dll_handle_, "openDevice"));
		fn_closeDevice_ = reinterpret_cast<FnCloseDevice>(GetProcAddress(dll_handle_, "closeDevice"));
		fn_startServoLoop_ = reinterpret_cast<FnStartServoLoop>(GetProcAddress(dll_handle_, "startServoLoop"));
		fn_stopServoLoop_ = reinterpret_cast<FnStopServoLoop>(GetProcAddress(dll_handle_, "stopServoLoop"));
		fn_enableForces_ = reinterpret_cast<FnEnableForces>(GetProcAddress(dll_handle_, "enableForces"));
		fn_sendForce_ = reinterpret_cast<FnSendForce>(GetProcAddress(dll_handle_, "sendForce"));
		fn_getSerialNumber_ = reinterpret_cast<FnGetSerialNumber>(GetProcAddress(dll_handle_, "getSerialNumber"));
		fn_deviceStatus_ = reinterpret_cast<FnDeviceStatus>(GetProcAddress(dll_handle_, "deviceStatus"));

		return fn_openDevice_ && fn_sendForce_;
	}

	using FnOpenDevice = int(*)(DWORD sn);
	using FnCloseDevice = void(*)();
	using FnStartServoLoop = int(*)(int(__stdcall *func)(void *), void *lpParam);
	using FnStopServoLoop = int(*)();
	using FnEnableForces = void(*)(bool en, int id);
	using FnSendForce = void(*)(double force[], double torque, int id);
	using FnGetSerialNumber = int(*)(int id);
	using FnDeviceStatus = int(*)(int id);

	DWORD serial_ = 582;
	HMODULE dll_handle_ = nullptr;
	int device_id_ = -1;
	bool connected_ = false;
	bool servo_loop_started_ = false;
	double last_sent_force_ = 0.0;

	FnOpenDevice fn_openDevice_ = nullptr;
	FnCloseDevice fn_closeDevice_ = nullptr;
	FnStartServoLoop fn_startServoLoop_ = nullptr;
	FnStopServoLoop fn_stopServoLoop_ = nullptr;
	FnEnableForces fn_enableForces_ = nullptr;
	FnSendForce fn_sendForce_ = nullptr;
	FnGetSerialNumber fn_getSerialNumber_ = nullptr;
	FnDeviceStatus fn_deviceStatus_ = nullptr;
};

} // namespace handle582
