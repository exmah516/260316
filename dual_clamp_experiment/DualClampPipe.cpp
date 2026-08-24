#include "DualClampPipe.h"

#include <windows.h>

#include <chrono>
#include <sstream>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <iomanip>

namespace
{
	constexpr wchar_t kPipeName[] = L"\\\\.\\pipe\\DualClampExperiment";

	std::string read_line(HANDLE pipe)
	{
		std::string line;
		char ch = 0;
		DWORD read = 0;
		while (ReadFile(pipe, &ch, 1, &read, nullptr) && read == 1)
		{
			if (ch == '\n') break;
			if (ch != '\r') line.push_back(ch);
			if (line.size() > 4096) break;
		}
		return line;
	}

	std::string sanitize_for_pipe(std::string text)
	{
		for (char& c : text)
		{
			if (c == '\r' || c == '\n' || c == '|') c = ' ';
		}
		while (!text.empty() && text.back() == ' ') text.pop_back();
		return text;
	}

	void write_line(HANDLE pipe, const std::string& line)
	{
		const std::string payload = line + "\n";
		DWORD written = 0;
		WriteFile(pipe, payload.data(), static_cast<DWORD>(payload.size()), &written, nullptr);
	}

	bool apply_config_fields(const std::string& command, DualClampConfig& config, std::string& error)
	{
		const std::size_t separator = command.find('|');
		if (separator == std::string::npos) return true;
		std::istringstream fields(command.substr(separator + 1));
		std::string field;
		while (std::getline(fields, field, '|'))
		{
			const std::size_t equal = field.find('=');
			if (equal == std::string::npos || equal == 0) continue;
			const std::string key = field.substr(0, equal);
			const std::string text = field.substr(equal + 1);
			double value = 0.0;
			try
			{
				value = std::stod(text);
			}
			catch (const std::exception&)
			{
				error = "参数不是有效数字：" + key;
				return false;
			}

			if (key == "moving_axis") config.moving_axis = static_cast<int>(value);
			else if (key == "axis1_distance") config.axis1_distance_from_left_mm = value;
			else if (key == "axis6_distance") config.axis6_distance_from_left_mm = value;
			else if (key == "axis2_angle") config.axis2_angle_abs_deg = value;
			else if (key == "axis7_angle") config.axis7_angle_abs_deg = value;
			else if (key == "return_retract") config.return_retract_distance_mm = value;
			else if (key == "return_velocity") config.return_velocity_mm_s = value;
			else if (key == "return_acc") config.return_acceleration_mm_s2 = value;
			else if (key == "return_dec") config.return_deceleration_mm_s2 = value;
			else if (key == "return_jerk") config.return_jerk_mm_s3 = value;
			else if (key == "recovery_mode") config.recovery_mode = value > 0.5 ? DualClampRecoveryMode::Move : DualClampRecoveryMode::Hold;
		}
		return true;
	}

	bool apply_program_fields(const std::string& command, ProgrammedDeliveryConfig& config, std::string& error)
	{
		const std::size_t separator = command.find('|');
		if (separator == std::string::npos) return true;
		std::istringstream fields(command.substr(separator + 1));
		std::string field;
		while (std::getline(fields, field, '|'))
		{
			const std::size_t equal = field.find('=');
			if (equal == std::string::npos || equal == 0) continue;
			const std::string key = field.substr(0, equal);
			const std::string text = field.substr(equal + 1);
			if (key == "mode")
			{
				if (text == "legacy") config.mode = ProgrammedDeliveryMode::Legacy;
				else if (text == "catheter") config.mode = ProgrammedDeliveryMode::Catheter;
				else if (text == "guidewire") config.mode = ProgrammedDeliveryMode::Guidewire;
				else { error = "mode必须是legacy、catheter或guidewire"; return false; }
				continue;
			}
			double value = 0.0;
			try { value = std::stod(text); }
			catch (const std::exception&) { error = "参数不是有效数字：" + key; return false; }
			if (key == "axis5_from_left") config.axis5_from_left_mm = value;
			else if (key == "axis2_angle") config.axis2_angle_deg = value;
			else if (key == "axis7_angle") config.axis7_angle_deg = value;
			else if (key == "cycle_count")
			{
				if (!std::isfinite(value) || value < 1.0 || value > 65535.0 || std::floor(value) != value)
				{
					error = "cycle_count必须是1至65535之间的整数";
					return false;
				}
				config.cycle_count = static_cast<std::uint16_t>(value);
			}
			else if (key == "final_forward_distance") config.final_forward_distance_mm = value;
			else if (key == "forward_velocity") config.forward_velocity_mm_s = value;
			else if (key == "forward_acceleration") config.forward_acceleration_mm_s2 = value;
			else if (key == "forward_deceleration") config.forward_deceleration_mm_s2 = value;
			else if (key == "forward_jerk") config.forward_jerk_mm_s3 = value;
			else if (key == "return_velocity") config.return_velocity_mm_s = value;
			else if (key == "return_acceleration") config.return_acceleration_mm_s2 = value;
			else if (key == "return_deceleration") config.return_deceleration_mm_s2 = value;
			else if (key == "return_jerk") config.return_jerk_mm_s3 = value;
		}
		return true;
	}
}

DualClampPipeServer::DualClampPipeServer() = default;

std::string DualClampPipeServer::handle_command(DualClampController& controller, const std::string& command)
{
	if (command == "GET")
	{
		const DualClampLiveFrame live = controller.live();
		const bool ads_open = controller.is_ads_open();
		std::ostringstream out;
		out << "STATE|" << static_cast<int>(controller.phase()) << '|'
			<< live.axis1_pos_abs_mm << '|' << live.axis6_pos_abs_mm << '|'
			<< live.axis1_velocity_mm_s << '|' << live.axis6_velocity_mm_s << '|'
			<< live.axis1_acceleration_mm_s2 << '|' << live.axis6_acceleration_mm_s2 << '|'
			<< live.axis2_angle_abs_deg << '|' << live.axis7_angle_abs_deg << '|'
			<< live.fn_1_raw << '|' << live.fn_2_raw << '|' << live.ft_1_raw << '|' << live.ft_2_raw << '|'
			<< (ads_open ? 1 : 0) << '|'
			<< (live.selfcheck_done ? 1 : 0) << '|'
			<< (live.selfcheck_busy ? 1 : 0) << '|'
			<< (live.leftlimit_valid ? 1 : 0) << '|'
			<< live.leftlimit_axis1_abs_mm << '|'
			<< live.leftlimit_axis6_abs_mm << '|'
			<< (live.setup_busy ? 1 : 0) << '|'
			<< (live.setup_done ? 1 : 0) << '|'
			<< live.setup_target_axis1_abs_mm << '|'
			<< live.setup_target_axis6_abs_mm << '|'
			<< live.return_target_abs_mm << '|'
			<< live.status_error_id << '|'
			<< sanitize_for_pipe(controller.last_error());
		return out.str();
	}
	if (command == "CONNECT_ADS" || command == "CONNECT")
	{
		if (controller.open_ads())
		{
			return "OK|CONNECT_ADS";
		}
		return "ERROR|" + sanitize_for_pipe(controller.last_error());
	}
	if (command == "DISCONNECT_ADS")
	{
		controller.close_ads();
		return "OK|DISCONNECT_ADS";
	}
	if (command == "ABORT")
	{
		controller.abort("UI abort");
		return "OK|ABORT";
	}
	if (command.rfind("PREPARE", 0) == 0)
	{
		DualClampConfig config = controller.config();
		std::string parse_error;
		if (!apply_config_fields(command, config, parse_error)) return "ERROR|" + sanitize_for_pipe(parse_error);
		return controller.prepare(config) ? "OK|PREPARE" : "ERROR|" + sanitize_for_pipe(controller.last_error());
	}
	if (command.rfind("START", 0) == 0)
	{
		// 开始实验使用最近一次“准备定位”成功下发的参数。
		DualClampConfig config = controller.config();
		std::string parse_error;
		if (!apply_config_fields(command, config, parse_error)) return "ERROR|" + sanitize_for_pipe(parse_error);
		return controller.start(config) ? "OK|START" : "ERROR|" + sanitize_for_pipe(controller.last_error());
	}
	if (command.rfind("SAVE|", 0) == 0)
	{
		std::string error;
		return controller.save_samples(command.substr(5), error) ? "OK|SAVE" : "ERROR|" + sanitize_for_pipe(error);
	}
	return "ERROR|unknown command";
}

std::string DualClampPipeServer::handle_program_command(ProgrammedDeliveryController& controller, const std::string& command)
{
	if (command == "GET_PROGRAM")
	{
		const ProgrammedDeliveryLiveFrame live = controller.live();
		const bool ads_open = controller.is_ads_open();
		std::ostringstream out;
		out << "PROGRAM_STATE|" << static_cast<unsigned>(live.mode) << '|'
			<< static_cast<unsigned>(live.phase) << '|' << live.cycle_index << '|' << live.cycle_total << '|'
			<< (live.setup_busy ? 1 : 0) << '|' << (live.setup_done ? 1 : 0) << '|'
			<< (live.selfcheck_done ? 1 : 0) << '|'
			<< live.axis1_pos << '|' << live.axis1_vel << '|' << live.axis1_acc << '|'
			<< live.axis2_pos << '|' << live.axis2_vel << '|' << live.axis2_acc << '|'
			<< live.axis5_pos << '|' << live.axis5_vel << '|' << live.axis5_acc << '|'
			<< live.axis6_pos << '|' << live.axis6_vel << '|' << live.axis6_acc << '|'
			<< live.axis7_pos << '|' << live.axis7_vel << '|' << live.axis7_acc << '|'
			<< live.fn1 << '|' << live.ft1 << '|' << live.fn2 << '|' << live.ft2 << '|'
			<< live.cylinder1 << '|' << live.cylinder2 << '|' << live.cylinder3 << '|' << live.cylinder4 << '|'
			<< live.target_axis1_abs_mm << '|' << live.target_axis5_abs_mm << '|' << live.target_axis6_abs_mm << '|'
			<< live.target_axis2_deg << '|' << live.target_axis7_deg << '|'
			<< live.trigger_target_abs_mm << '|' << live.return_target_abs_mm << '|' << live.final_target_abs_mm << '|'
			<< live.status_error_id << '|' << (ads_open ? 1 : 0) << '|' << sanitize_for_pipe(controller.last_error());
		return out.str();
	}
	if (command.rfind("PROGRAM_MODE", 0) == 0)
	{
		ProgrammedDeliveryConfig config = controller.config();
		std::string error;
		if (!apply_program_fields(command, config, error)) return "ERROR|" + sanitize_for_pipe(error);
		return controller.select_mode(config.mode) ? "OK|PROGRAM_MODE" : "ERROR|" + sanitize_for_pipe(controller.last_error());
	}
	if (command.rfind("PROGRAM_PREPARE", 0) == 0)
	{
		ProgrammedDeliveryConfig config = controller.config();
		std::string error;
		if (!apply_program_fields(command, config, error)) return "ERROR|" + sanitize_for_pipe(error);
		return controller.prepare(config) ? "OK|PROGRAM_PREPARE" : "ERROR|" + sanitize_for_pipe(controller.last_error());
	}
	if (command == "PROGRAM_START")
		return controller.start() ? "OK|PROGRAM_START" : "ERROR|" + sanitize_for_pipe(controller.last_error());
	if (command == "PROGRAM_ABORT")
	{
		controller.abort();
		return "OK|PROGRAM_ABORT";
	}
	if (command.rfind("PROGRAM_SAVE|", 0) == 0)
	{
		std::string error;
		return controller.save_samples(command.substr(13), error) ? "OK|PROGRAM_SAVE" : "ERROR|" + sanitize_for_pipe(error);
	}
	return "ERROR|unknown program command";
}

int DualClampPipeServer::run(DualClampController& controller)
{
	std::atomic<bool> running{ true };
	std::thread tick_thread([&]()
	{
		while (running.load())
		{
			controller.tick(0.01);
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	});
	for (;;)
	{
		HANDLE pipe = CreateNamedPipeW(
			kPipeName,
			PIPE_ACCESS_DUPLEX,
			PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT,
			PIPE_UNLIMITED_INSTANCES,
			8192,
			8192,
			0,
			nullptr);
		if (pipe == INVALID_HANDLE_VALUE)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		const BOOL connected = ConnectNamedPipe(pipe, nullptr) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);
		if (connected)
		{
			for (;;)
			{
				const std::string command = read_line(pipe);
				if (command.empty()) break;
				if (command == "QUIT")
				{
					write_line(pipe, "OK|QUIT");
					FlushFileBuffers(pipe);
					DisconnectNamedPipe(pipe);
					CloseHandle(pipe);
					running.store(false);
					tick_thread.join();
					return 0;
				}
				write_line(pipe, handle_command(controller, command));
			}
		}
		FlushFileBuffers(pipe);
		DisconnectNamedPipe(pipe);
		CloseHandle(pipe);
	}
}

int DualClampPipeServer::run(DualClampController& controller, ProgrammedDeliveryController& program_controller)
{
	std::atomic<bool> running{ true };
	std::thread tick_thread([&]()
	{
		while (running.load())
		{
			controller.tick(0.01);
			program_controller.tick();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	});
	for (;;)
	{
		HANDLE pipe = CreateNamedPipeW(
			kPipeName, PIPE_ACCESS_DUPLEX, PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT,
			PIPE_UNLIMITED_INSTANCES, 8192, 8192, 0, nullptr);
		if (pipe == INVALID_HANDLE_VALUE) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }
		const BOOL connected = ConnectNamedPipe(pipe, nullptr) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);
		if (connected)
		{
			for (;;)
			{
				const std::string command = read_line(pipe);
				if (command.empty()) break;
				if (command == "QUIT")
				{
					write_line(pipe, "OK|QUIT"); FlushFileBuffers(pipe); DisconnectNamedPipe(pipe); CloseHandle(pipe);
					running.store(false); tick_thread.join(); return 0;
				}
				if (command == "CONNECT_ADS" || command == "CONNECT")
				{
					const bool legacy_connected = controller.open_ads();
					const bool program_connected = program_controller.open_ads();
					if (legacy_connected && program_connected) write_line(pipe, "OK|CONNECT_ADS");
					else write_line(pipe, "ERROR|ADS连接失败：" + sanitize_for_pipe(
						legacy_connected ? program_controller.last_error() : controller.last_error()));
					continue;
				}
				if (command == "DISCONNECT_ADS")
				{
					controller.close_ads();
					program_controller.close_ads();
					write_line(pipe, "OK|DISCONNECT_ADS");
					continue;
				}
				const bool is_program = command.rfind("PROGRAM_", 0) == 0 || command == "GET_PROGRAM";
				write_line(pipe, is_program ? handle_program_command(program_controller, command) : handle_command(controller, command));
			}
		}
		FlushFileBuffers(pipe); DisconnectNamedPipe(pipe); CloseHandle(pipe);
	}
}
