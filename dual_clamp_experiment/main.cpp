#include "DualClampPipe.h"

#include <windows.h>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <string>

#include <tlhelp32.h>

namespace
{
	bool is_ui_process_running()
	{
		HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
		if (snapshot == INVALID_HANDLE_VALUE) return false;
		PROCESSENTRY32W entry{};
		entry.dwSize = sizeof(entry);
		bool found = false;
		if (Process32FirstW(snapshot, &entry))
		{
			do
			{
				if (_wcsicmp(entry.szExeFile, L"DualClampExperimentUI.exe") == 0)
				{
					found = true;
					break;
				}
			} while (Process32NextW(snapshot, &entry));
		}
		CloseHandle(snapshot);
		return found;
	}

	bool launch_ui()
	{
		if (is_ui_process_running())
		{
			return true; // 已经由 VS 调试器或用户启动，不再重复启动
		}

		wchar_t module_path[MAX_PATH]{};
		const DWORD length = GetModuleFileNameW(nullptr, module_path, static_cast<DWORD>(std::size(module_path)));
		if (length == 0 || length >= std::size(module_path)) return false;

		const std::filesystem::path backend_path(module_path, module_path + length);
		const std::filesystem::path project_path = backend_path.parent_path().parent_path().parent_path();
		const std::vector<std::filesystem::path> candidate_paths = {
			project_path / L"AdsControlUI" / L"bin" / L"Debug" / L"net472" / L"DualClampExperimentUI.exe",
			project_path / L"AdsControlUI" / L"bin" / L"x64" / L"Debug" / L"net472" / L"DualClampExperimentUI.exe",
			project_path / L"AdsControlUI" / L"bin" / L"Release" / L"net472" / L"DualClampExperimentUI.exe",
			project_path / L"AdsControlUI" / L"bin" / L"x64" / L"Release" / L"net472" / L"DualClampExperimentUI.exe",
			backend_path.parent_path() / L"DualClampExperimentUI.exe",
			backend_path.parent_path() / L"net472" / L"DualClampExperimentUI.exe"
		};
		std::filesystem::path ui_path;
		for (const auto& p : candidate_paths)
		{
			if (std::filesystem::exists(p))
			{
				ui_path = p;
				break;
			}
		}
		if (ui_path.empty()) return false;

		std::wstring command_line = L"\"" + ui_path.wstring() + L"\"";
		STARTUPINFOW startup{};
		startup.cb = sizeof(startup);
		PROCESS_INFORMATION process{};
		const bool started = CreateProcessW(
			nullptr, command_line.data(), nullptr, nullptr, FALSE, 0, nullptr,
			ui_path.parent_path().c_str(), &startup, &process) != FALSE;
		if (started)
		{
			CloseHandle(process.hThread);
			CloseHandle(process.hProcess);
		}
		return started;
	}
}

int main(int argc, char* argv[])
{
	// 控制台统一使用 UTF-8，避免中文启动提示按本地代码页显示为乱码。
	SetConsoleOutputCP(CP_UTF8);
	SetConsoleCP(CP_UTF8);
	std::ios::sync_with_stdio(false);

	DualClampController controller;
	ProgrammedDeliveryController program_controller;
	if (controller.is_ads_open())
	{
		std::cout << "ADS 状态：已成功建立路由连接（端口 851）。" << std::endl;
	}
	else
	{
		std::cout << "ADS 状态：尚未就绪（" << controller.last_error() << "），等待PLC就绪或UI连接。" << std::endl;
	}

	const bool no_ui = argc > 1 && std::string(argv[1]) == "--no-ui";
	if (!no_ui && launch_ui())
	{
		std::cout << "WPF界面已自动启动。" << std::endl;
	}
	else if (!no_ui)
	{
		std::cout << "提示：未自动启动WPF界面，可手动运行 DualClampExperimentUI.exe。" << std::endl;
	}
	DualClampPipeServer server;
	std::cout << "双机构夹持扰动实验服务端已就绪，正在监听命名管道 \\\\.\\pipe\\DualClampExperiment ..." << std::endl;
	return server.run(controller, program_controller);
}
