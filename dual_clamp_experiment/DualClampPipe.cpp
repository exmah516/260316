#include "DualClampPipe.h"
#include "ForceCalibration.h"

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
		if (key == "record_name")
		{
			config.record_suffix = text;
			continue;
		}
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
		// 新字段缺省按参与配合处理，兼容未携带配合参数的旧版 PROGRAM_PREPARE 命令。
		config.cylinder1_coupling_enabled = true;
		config.cylinder3_coupling_enabled = true;
		config.release_lead_ms = 50;
		config.reclamp_lead_ms = 50;
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
		if (key == "record_name")
		{
			config.record_suffix = text;
			continue;
		}
		if (key == "cylinder1_coupling" || key == "cylinder3_coupling")
		{
			if (text != "0" && text != "1")
			{
				error = key + "必须是0或1";
				return false;
			}
			const bool enabled = text == "1";
			if (key == "cylinder1_coupling") config.cylinder1_coupling_enabled = enabled;
			else config.cylinder3_coupling_enabled = enabled;
			continue;
		}
		if (key == "cylinder2_open" || key == "cylinder2_close" || key == "cylinder4_open" || key == "cylinder4_close")
		{
			std::uint64_t word = 0;
			try
			{
				std::size_t used = 0;
				word = std::stoull(text, &used, 0);
				if (used != text.size() || word > 65535) throw std::invalid_argument("range");
			}
			catch (const std::exception&)
			{
				error = key + "必须是0至65535之间的整数";
				return false;
			}
			if (key == "cylinder2_open") config.cylinder2_open_word = static_cast<std::uint16_t>(word);
			else if (key == "cylinder2_close") config.cylinder2_close_word = static_cast<std::uint16_t>(word);
			else if (key == "cylinder4_open") config.cylinder4_open_word = static_cast<std::uint16_t>(word);
			else config.cylinder4_close_word = static_cast<std::uint16_t>(word);
			continue;
		}
			double value = 0.0;
			try { value = std::stod(text); }
			catch (const std::exception&) { error = "参数不是有效数字：" + key; return false; }
			if (key == "axis1_prepare_from_left") config.axis1_prepare_from_left_mm = value;
			else if (key == "axis1_trigger_from_left") config.axis1_trigger_from_left_mm = value;
			else if (key == "axis5_from_left")
			{
				config.axis5_from_left_mm = value;
				// 兼容旧版导丝命令：未传轴6字段时，继续使用原来的轴5相对窗口。
				config.axis6_prepare_from_left_mm = value + 21.0;
				config.axis6_trigger_from_left_mm = value + 1.0;
			}
			else if (key == "axis6_prepare_from_left") config.axis6_prepare_from_left_mm = value;
			else if (key == "axis6_trigger_from_left") config.axis6_trigger_from_left_mm = value;
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
			else if (key == "release_wait_ms" || key == "reclamp_wait_ms" || key == "release_lead_ms" || key == "reclamp_lead_ms")
			{
				if (!std::isfinite(value) || value < 0.0 || value > 60000.0 || std::floor(value) != value)
				{
					error = key + "必须是0至60000之间的整数";
					return false;
				}
				if (key == "release_wait_ms") config.release_wait_ms = static_cast<std::uint32_t>(value);
				else if (key == "reclamp_wait_ms") config.reclamp_wait_ms = static_cast<std::uint32_t>(value);
				else if (key == "release_lead_ms") config.release_lead_ms = static_cast<std::uint32_t>(value);
				else config.reclamp_lead_ms = static_cast<std::uint32_t>(value);
			}
		}
		return true;
	}

	std::string command_field(const std::string& command, const std::string& key)
	{
		std::istringstream fields(command);
		std::string field;
		const std::string prefix = key + "=";
		while (std::getline(fields, field, '|'))
		{
			if (field.rfind(prefix, 0) == 0) return field.substr(prefix.size());
		}
		return {};
	}

	bool parse_unsigned_field(const std::string& command, const std::string& key, std::uint64_t& value, std::string& error)
	{
		const std::string text = command_field(command, key);
		if (text.empty())
		{
			error = "缺少参数：" + key;
			return false;
		}
		try
		{
			std::size_t used = 0;
			value = std::stoull(text, &used, 0);
			if (used != text.size()) throw std::invalid_argument("trailing");
		}
		catch (const std::exception&)
		{
			error = "参数不是有效整数：" + key;
			return false;
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
		const ForceZeroState zero = controller.zero_state();
		const forcecal::Result force = forcecal::calculate(live.fn_1_raw, live.ft_1_raw, live.fn_2_raw, live.ft_2_raw, zero.value, zero.valid);
		const double fn1_realtime = force.valid ? force.side1.force_cal_delta_n : 0.0;
		const double fn2_realtime = force.valid ? force.side2.force_cal_delta_n : 0.0;
		const double ft1_realtime = force.valid ? force.side1.ft_cal_delta_n : 0.0;
		const double ft2_realtime = force.valid ? force.side2.ft_cal_delta_n : 0.0;
		std::ostringstream out;
		out << "STATE|" << static_cast<int>(controller.phase()) << '|'
			<< live.axis1_pos_abs_mm << '|' << live.axis6_pos_abs_mm << '|'
			<< live.axis1_velocity_mm_s << '|' << live.axis6_velocity_mm_s << '|'
			<< live.axis1_acceleration_mm_s2 << '|' << live.axis6_acceleration_mm_s2 << '|'
			<< live.axis2_angle_abs_deg << '|' << live.axis7_angle_abs_deg << '|'
			<< fn1_realtime << '|' << fn2_realtime << '|' << ft1_realtime << '|' << ft2_realtime << '|'
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
			<< sanitize_for_pipe(controller.last_error()) << '|'
			<< (controller.zero_state().busy ? 1 : 0) << '|'
			<< (controller.zero_state().done ? 1 : 0) << '|'
			<< controller.zero_state().value[0] << '|'
			<< controller.zero_state().value[1] << '|'
			<< controller.zero_state().value[2] << '|'
			<< controller.zero_state().value[3] << '|'
			<< (live.recording ? 1 : 0) << '|' << live.recording_sample_count << '|' << live.recording_error_id << '|'
			<< sanitize_for_pipe(controller.recording_directory()) << '|'
			<< (controller.recording_archived() ? 1 : 0) << '|'
			<< (force.valid ? 1 : 0) << '|' << force.side1.force_cal_delta_n << '|' << force.side1.ft_cal_delta_n << '|'
			<< force.side2.force_cal_delta_n << '|' << force.side2.ft_cal_delta_n;
		return out.str();
	}
	if (command == "ZERO_STATUS") return handle_command(controller, "GET");
	if (command.rfind("RECORD_NAME|", 0) == 0)
	{
		const std::string prefix = "RECORD_NAME|name=";
		const std::string suffix = command.rfind(prefix, 0) == 0 ? command.substr(prefix.size()) : command.substr(std::string("RECORD_NAME|").size());
		return controller.set_record_suffix(suffix) ? "OK|RECORD_NAME" : "ERROR|" + sanitize_for_pipe(controller.last_error());
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
	if (command == "SAVE")
	{
		std::string error;
		return controller.save_samples("", error) ? "OK|SAVE" : "ERROR|" + sanitize_for_pipe(error);
	}
	if (command == "ZERO_FORCE")
	{
		return controller.request_zero() ? "OK|ZERO_FORCE" : "ERROR|" + sanitize_for_pipe(controller.last_error());
	}
	return "ERROR|unknown command";
}

std::string DualClampPipeServer::handle_program_command(ProgrammedDeliveryController& controller, const std::string& command)
{
	if (command == "GET_PROGRAM")
	{
		const ProgrammedDeliveryLiveFrame live = controller.live();
		const bool ads_open = controller.is_ads_open();
		const ForceZeroState zero = controller.zero_state();
		const forcecal::Result force = forcecal::calculate(live.fn1, live.ft1, live.fn2, live.ft2, zero.value, zero.valid);
		const double fn1_realtime = force.valid ? force.side1.force_cal_delta_n : 0.0;
		const double fn2_realtime = force.valid ? force.side2.force_cal_delta_n : 0.0;
		const double ft1_realtime = force.valid ? force.side1.ft_cal_delta_n : 0.0;
		const double ft2_realtime = force.valid ? force.side2.ft_cal_delta_n : 0.0;
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
			<< fn1_realtime << '|' << ft1_realtime << '|' << fn2_realtime << '|' << ft2_realtime << '|'
			<< live.cylinder1 << '|' << live.cylinder2 << '|' << live.cylinder3 << '|' << live.cylinder4 << '|'
			<< live.target_axis1_abs_mm << '|' << live.target_axis5_abs_mm << '|' << live.target_axis6_abs_mm << '|'
			<< live.target_axis2_deg << '|' << live.target_axis7_deg << '|'
			<< live.trigger_target_abs_mm << '|' << live.return_target_abs_mm << '|' << live.final_target_abs_mm << '|'
			<< live.status_error_id << '|' << (ads_open ? 1 : 0) << '|'
			<< sanitize_for_pipe(controller.last_error()) << '|'
			<< (controller.zero_state().busy ? 1 : 0) << '|'
			<< (controller.zero_state().done ? 1 : 0) << '|'
			<< controller.zero_state().value[0] << '|'
			<< controller.zero_state().value[1] << '|'
			<< controller.zero_state().value[2] << '|'
			<< controller.zero_state().value[3] << '|'
			<< (live.recording ? 1 : 0) << '|' << live.recording_sample_count << '|' << live.recording_error_id << '|'
			<< sanitize_for_pipe(controller.recording_directory()) << '|'
			<< (controller.recording_archived() ? 1 : 0) << '|'
			<< static_cast<unsigned>(live.wait_action) << '|'
			<< static_cast<unsigned>(live.error_source) << '|'
			<< static_cast<unsigned>(live.error_axis) << '|'
			<< static_cast<unsigned>(live.error_phase) << '|'
			<< live.error_target_abs_mm << '|' << live.error_target_from_left_mm << '|'
			<< (force.valid ? 1 : 0) << '|' << force.side1.force_cal_delta_n << '|' << force.side1.ft_cal_delta_n << '|'
			<< force.side2.force_cal_delta_n << '|' << force.side2.ft_cal_delta_n << '|'
			<< (live.selfcheck_busy ? 1 : 0);
		return out.str();
	}
	if (command == "PROGRAM_ZERO_STATUS") return handle_program_command(controller, "GET_PROGRAM");
	if (command.rfind("PROGRAM_RECORD_NAME|", 0) == 0)
	{
		const std::string prefix = "PROGRAM_RECORD_NAME|name=";
		const std::string suffix = command.rfind(prefix, 0) == 0 ? command.substr(prefix.size()) : command.substr(std::string("PROGRAM_RECORD_NAME|").size());
		return controller.set_record_suffix(suffix) ? "OK|PROGRAM_RECORD_NAME" : "ERROR|" + sanitize_for_pipe(controller.last_error());
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
	if (command == "PROGRAM_ZERO_FORCE")
		return controller.request_zero() ? "OK|PROGRAM_ZERO_FORCE" : "ERROR|" + sanitize_for_pipe(controller.last_error());
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
	if (command == "PROGRAM_SAVE")
	{
		std::string error;
		return controller.save_samples("", error) ? "OK|PROGRAM_SAVE" : "ERROR|" + sanitize_for_pipe(error);
	}
	return "ERROR|unknown program command";
}

std::string DualClampPipeServer::handle_standalone_command(StandaloneRecordController& controller, const std::string& command)
{
	if (command == "GET_STANDALONE_RECORD" || command == "STANDALONE_ZERO_STATUS")
	{
		const StandaloneRecordLiveFrame live = controller.live();
		const ForceZeroState zero = controller.zero_state();
		std::ostringstream out;
		out << "STANDALONE_STATE|"
			<< (controller.is_ads_open() ? 1 : 0) << '|'
			<< (live.selfcheck_done ? 1 : 0) << '|'
			<< live.legacy_phase << '|'
			<< live.cylinder[0] << '|' << live.cylinder[1] << '|' << live.cylinder[2] << '|' << live.cylinder[3] << '|'
			<< (live.manual_control_enabled ? 1 : 0) << '|'
			<< (controller.recording_active() ? 1 : 0) << '|'
			<< controller.recording_sample_count() << '|'
			<< (live.manual_error_id != 0 ? live.manual_error_id : 0) << '|'
			<< sanitize_for_pipe(controller.recording_directory()) << '|'
			<< (controller.recording_archived() ? 1 : 0) << '|'
			<< (zero.busy ? 1 : 0) << '|'
			<< (zero.done ? 1 : 0) << '|'
			<< zero.value[0] << '|' << zero.value[1] << '|' << zero.value[2] << '|' << zero.value[3] << '|'
			<< controller.field_mask() << '|'
			<< sanitize_for_pipe(controller.last_error()) << '|'
			<< (controller.recording_stopping() ? 1 : 0);
		return out.str();
	}
	if (command.rfind("MANUAL_CYLINDER_CONFIG", 0) == 0)
	{
		std::uint64_t cylinder = 0, enabled = 1, open_value = 0, close_value = 0;
		std::string error;
		if (!parse_unsigned_field(command, "cylinder", cylinder, error) ||
			!parse_unsigned_field(command, "enabled", enabled, error) ||
			!parse_unsigned_field(command, "open", open_value, error) ||
			!parse_unsigned_field(command, "close", close_value, error))
			return "ERROR|" + sanitize_for_pipe(error);
		if (cylinder < 1 || cylinder > 4 || open_value > 65535 || close_value > 65535 || enabled > 1)
			return "ERROR|电缸配置参数超出范围";
		return controller.set_cylinder_config(static_cast<int>(cylinder), enabled != 0,
			static_cast<std::uint16_t>(open_value), static_cast<std::uint16_t>(close_value))
			? "OK|MANUAL_CYLINDER_CONFIG" : "ERROR|" + sanitize_for_pipe(controller.last_error());
	}
	if (command.rfind("MANUAL_CYLINDER_OPEN", 0) == 0 || command.rfind("MANUAL_CYLINDER_CLOSE", 0) == 0)
	{
		std::uint64_t cylinder = 0;
		std::string error;
		if (!parse_unsigned_field(command, "cylinder", cylinder, error) || cylinder < 1 || cylinder > 4)
			return "ERROR|" + sanitize_for_pipe(error.empty() ? "电缸编号必须为1至4" : error);
		const bool ok = command.rfind("MANUAL_CYLINDER_OPEN", 0) == 0
			? controller.cylinder_open(static_cast<int>(cylinder))
			: controller.cylinder_close(static_cast<int>(cylinder));
		return ok ? "OK|" + std::string(command.rfind("MANUAL_CYLINDER_OPEN", 0) == 0 ? "MANUAL_CYLINDER_OPEN" : "MANUAL_CYLINDER_CLOSE")
			: "ERROR|" + sanitize_for_pipe(controller.last_error());
	}
	if (command.rfind("STANDALONE_RECORD_START", 0) == 0)
	{
		std::uint64_t fields = 0;
		std::string error;
		if (!parse_unsigned_field(command, "fields", fields, error)) return "ERROR|" + sanitize_for_pipe(error);
		const std::string suffix = command_field(command, "record_name");
		return controller.start_record(suffix, fields) ? "OK|STANDALONE_RECORD_START" : "ERROR|" + sanitize_for_pipe(controller.last_error());
	}
	if (command == "STANDALONE_RECORD_STOP")
		return controller.stop_record() ? "OK|STANDALONE_RECORD_STOP" : "ERROR|" + sanitize_for_pipe(controller.last_error());
	if (command == "STANDALONE_RECORD_ABORT")
		return controller.abort_record("UI abort") ? "OK|STANDALONE_RECORD_ABORT" : "ERROR|" + sanitize_for_pipe(controller.last_error());
	if (command == "STANDALONE_ZERO_FORCE")
		return controller.request_zero() ? "OK|STANDALONE_ZERO_FORCE" : "ERROR|" + sanitize_for_pipe(controller.last_error());
	if (command == "STANDALONE_RECORD_SAVE")
	{
		std::string error;
		return controller.save(error) ? "OK|STANDALONE_RECORD_SAVE" : "ERROR|" + sanitize_for_pipe(error);
	}
	if (command == "STANDALONE_RELEASE_MANUAL")
	{
		controller.release_manual_control();
		return "OK|STANDALONE_RELEASE_MANUAL";
	}
	return "ERROR|unknown standalone command";
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
			const DualClampLiveFrame legacy_live = controller.live();
			program_controller.set_shared_selfcheck_state(legacy_live.selfcheck_done, legacy_live.selfcheck_busy);
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
				if (command.rfind("PROGRAM_MODE", 0) == 0)
				{
					// 实验模式切换会使两个控制器各自保存的力感零点同时失效。
					controller.invalidate_zero();
					program_controller.invalidate_zero();
				}
				try
				{
					write_line(pipe, is_program ? handle_program_command(program_controller, command) : handle_command(controller, command));
				}
				catch (const std::exception& ex)
				{
					write_line(pipe, "ERROR|后端处理命令异常：" + sanitize_for_pipe(ex.what()));
				}
			}
		}
		FlushFileBuffers(pipe); DisconnectNamedPipe(pipe); CloseHandle(pipe);
	}
}

int DualClampPipeServer::run(DualClampController& controller, ProgrammedDeliveryController& program_controller,
	StandaloneRecordController& standalone_controller)
{
	std::atomic<bool> running{ true };
	std::thread tick_thread([&]()
	{
		while (running.load())
		{
			controller.tick(0.01);
			const DualClampLiveFrame legacy_live = controller.live();
			program_controller.set_shared_selfcheck_state(legacy_live.selfcheck_done, legacy_live.selfcheck_busy);
			// 旧双机构模式不需要轮询程序递送整帧；否则在线PLC缺少新增程序符号时，
			// 后端会在旧模式下持续产生隐藏的ADS错误。
			if (program_controller.config().mode != ProgrammedDeliveryMode::Legacy)
				program_controller.tick();
			standalone_controller.tick();
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
					standalone_controller.abort_record("程序退出");
					standalone_controller.release_manual_control();
					write_line(pipe, "OK|QUIT"); FlushFileBuffers(pipe); DisconnectNamedPipe(pipe); CloseHandle(pipe);
					running.store(false); tick_thread.join(); return 0;
				}
				if (command == "CONNECT_ADS" || command == "CONNECT")
				{
					const bool legacy_connected = controller.open_ads();
					const bool program_connected = program_controller.open_ads();
					const bool standalone_connected = standalone_controller.open_ads();
					if (legacy_connected && program_connected && standalone_connected) write_line(pipe, "OK|CONNECT_ADS");
					else
					{
						const std::string error = !legacy_connected ? controller.last_error() : !program_connected ? program_controller.last_error() : standalone_controller.last_error();
						write_line(pipe, "ERROR|ADS连接失败：" + sanitize_for_pipe(error));
					}
					continue;
				}
				if (command == "DISCONNECT_ADS")
				{
					controller.close_ads();
					program_controller.close_ads();
					standalone_controller.close_ads();
					write_line(pipe, "OK|DISCONNECT_ADS");
					continue;
				}
				if (command.rfind("PROGRAM_MODE", 0) == 0)
				{
					if (standalone_controller.recording_active() || standalone_controller.recording_stopping())
					{
						write_line(pipe, "ERROR|独立记录进行中，请先停止并归档记录后再切换模式");
						continue;
					}
					controller.invalidate_zero();
					program_controller.invalidate_zero();
					standalone_controller.invalidate_zero();
					standalone_controller.release_manual_control();
				}
				try
				{
					if (command.rfind("MANUAL_CYLINDER_", 0) == 0 || command.rfind("STANDALONE_", 0) == 0 || command == "GET_STANDALONE_RECORD")
						write_line(pipe, handle_standalone_command(standalone_controller, command));
					else if (command.rfind("PROGRAM_", 0) == 0 || command == "GET_PROGRAM")
						write_line(pipe, handle_program_command(program_controller, command));
					else
						write_line(pipe, handle_command(controller, command));
				}
				catch (const std::exception& ex)
				{
					write_line(pipe, "ERROR|后端处理命令异常：" + sanitize_for_pipe(ex.what()));
				}
			}
		}
		FlushFileBuffers(pipe); DisconnectNamedPipe(pipe); CloseHandle(pipe);
	}
}
