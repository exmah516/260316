#include "DualClampPipe.h"
#include "ForceCalibration.h"
#include "CylinderCommand.h"

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

	bool requires_motion_session(const std::string& command)
	{
		return command.rfind("PROGRAM_PREPARE", 0) == 0 || command == "PROGRAM_START" ||
			command == "PROGRAM_ZERO_FORCE" || command.rfind("PREPARE", 0) == 0 ||
			command.rfind("START", 0) == 0 || command == "ZERO_FORCE";
	}

	bool motion_session_ready(const DualClampController& controller)
	{
		const auto state = controller.self_check();
		return state.valid && state.done && state.status == 4 && state.gen_state == 6 && !state.host_timeout;
	}

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
		config.forward_pause_enabled = false;
		config.forward_pause_distance_mm = 10.0;
		config.forward_pause_duration_ms = 3000;
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
				else if (text == "external_validation") config.mode = ProgrammedDeliveryMode::ExternalValidation;
				else { error = "mode必须是legacy、catheter、guidewire或external_validation"; return false; }
				continue;
		}
		if (key == "record_name")
		{
			config.record_suffix = text;
			continue;
		}
		if (key == "model_sign")
		{
			if (text != "1" && text != "-1") { error = "model_sign必须为1或-1"; return false; }
			config.dynamics.axial_sign = text == "1" ? 1.0 : -1.0;
			continue;
		}
		if (key == "model_validation" || key == "model_conditions_confirmed")
		{
			if (text != "0" && text != "1") { error = key + "必须为0或1"; return false; }
			if (key == "model_validation") config.dynamics.validation_mode = text == "1";
			else config.dynamics.conditions_confirmed = text == "1";
			continue;
		}
		if (key == "model_reconstruct")
		{
			if (text != "0" && text != "1") { error = "model_reconstruct必须为0或1"; return false; }
			// 兼容旧版UI：旧客户端可能仍发送model_reconstruct=1。
			// 该功能未经验证，始终强制关闭，但不能因此阻断电机准备定位。
			config.dynamics.reconstruct_external = false;
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
		if (key == "forward_pause_enabled")
		{
			if (text != "0" && text != "1") { error = key + "必须是0或1"; return false; }
			config.forward_pause_enabled = text == "1";
			continue;
		}
		if (key == "forward_pause_distance_mm" || key == "forward_pause_duration_ms")
		{
			double value = 0.0;
			try {
				std::size_t used = 0;
				value = std::stod(text, &used);
				if (used != text.size() || !std::isfinite(value) || value <= 0.0 ||
					(key == "forward_pause_duration_ms" && (value > 60000.0 || std::floor(value) != value)))
					throw std::invalid_argument("range");
			} catch (const std::exception&) { error = "前进暂停参数无效：" + key; return false; }
			if (key == "forward_pause_distance_mm") config.forward_pause_distance_mm = value;
			else config.forward_pause_duration_ms = static_cast<std::uint32_t>(value);
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
			const auto safe_word = cylindercommand::normalize(static_cast<std::uint16_t>(word));
			if (key == "cylinder2_open") config.cylinder2_open_word = safe_word;
			else if (key == "cylinder2_close") config.cylinder2_close_word = safe_word;
			else if (key == "cylinder4_open") config.cylinder4_open_word = safe_word;
			else config.cylinder4_close_word = safe_word;
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

bool validate_program_config(const ProgrammedDeliveryConfig& config, std::string& error)
{
	if (!clampdynamics::valid_config(config.dynamics))
	{
		error = config.dynamics.validation_mode && !config.dynamics.conditions_confirmed
			? "验证模式须人工确认无器械、夹爪张开、仅轴向运动"
			: "运动补偿模型参数无效，请检查模型配置";
		return false;
	}
	const auto finite_positive = [](double value) { return std::isfinite(value) && value > 0.0; };
	if (!is_catheter_motion(config.mode) && config.mode != ProgrammedDeliveryMode::Guidewire)
	{
		error = "请选择导管、导丝或外源验证模式";
		return false;
	}
	if (config.cycle_count < 1)
	{
		error = "往复夹持次数必须至少为1";
		return false;
	}
	if (!std::isfinite(config.final_forward_distance_mm) || config.final_forward_distance_mm < 0.0)
	{
		error = "最终前向距离必须为不小于0的有限数";
		return false;
	}
	if (is_catheter_motion(config.mode))
	{
		if (!std::isfinite(config.axis1_prepare_from_left_mm) || !std::isfinite(config.axis1_trigger_from_left_mm) ||
			config.axis1_trigger_from_left_mm < 0.0 || config.axis1_prepare_from_left_mm <= config.axis1_trigger_from_left_mm)
		{
			error = "导管模式要求轴1准备位置大于触发位置，且触发位置不小于0";
			return false;
		}
		if (config.final_forward_distance_mm > config.axis1_prepare_from_left_mm - config.axis1_trigger_from_left_mm)
		{
			error = "最终前向距离不得超过轴1准备位置与触发位置之差";
			return false;
		}
	}
	else
	{
		if (!std::isfinite(config.axis5_from_left_mm) || config.axis5_from_left_mm < 0.0 || config.axis5_from_left_mm > 670.0)
		{
			error = "导丝模式轴5初始位置必须在0至670 mm之间";
			return false;
		}
		if (!std::isfinite(config.axis6_prepare_from_left_mm) || !std::isfinite(config.axis6_trigger_from_left_mm) ||
			config.axis6_trigger_from_left_mm < 0.0 || config.axis6_prepare_from_left_mm > 670.0 ||
			config.axis6_prepare_from_left_mm <= config.axis6_trigger_from_left_mm)
		{
			error = "导丝模式要求轴6初始位置大于触发位置，且两者均不得超过670 mm";
			return false;
		}
		if (config.final_forward_distance_mm > config.axis6_prepare_from_left_mm - config.axis6_trigger_from_left_mm)
		{
			error = "导丝模式最终前向距离不得超过轴6初始位置与触发位置之差";
			return false;
		}
	}
	if (config.mode == ProgrammedDeliveryMode::ExternalValidation &&
		(!std::isfinite(config.axis6_prepare_from_left_mm) ||
		config.axis6_prepare_from_left_mm < 0.0 || config.axis6_prepare_from_left_mm > 670.0))
	{
		error = "轴6初始位置必须在0至670 mm之间";
		return false;
	}
	if (config.mode == ProgrammedDeliveryMode::ExternalValidation &&
		(config.axis1_prepare_from_left_mm > 96.0 ||
		 config.axis6_prepare_from_left_mm < externalvalidation::total_forward(config)))
	{
		error = "外源模式要求轴1准备位置不超过96 mm，且轴6剩余行程覆盖全部周期";
		return false;
	}
	if (config.release_wait_ms > 60000 || config.reclamp_wait_ms > 60000 ||
		config.release_lead_ms > 60000 || config.reclamp_lead_ms > 60000)
	{
		error = "电缸等待和错开时间必须在0至60000 ms之间";
		return false;
	}
	if (!finite_positive(config.forward_velocity_mm_s) || !finite_positive(config.forward_acceleration_mm_s2) ||
		!finite_positive(config.forward_deceleration_mm_s2) || !finite_positive(config.forward_jerk_mm_s3) ||
		!finite_positive(config.return_velocity_mm_s) || !finite_positive(config.return_acceleration_mm_s2) ||
		!finite_positive(config.return_deceleration_mm_s2) || !finite_positive(config.return_jerk_mm_s3))
	{
		error = "前向和回退速度、加速度、减速度、Jerk必须是有限正数";
		return false;
	}
	if (config.forward_pause_enabled &&
		(config.mode != ProgrammedDeliveryMode::Catheter ||
		 !finite_positive(config.forward_pause_distance_mm) ||
		 config.forward_pause_distance_mm > config.axis1_prepare_from_left_mm - config.axis1_trigger_from_left_mm ||
		 config.forward_pause_duration_ms < 1 || config.forward_pause_duration_ms > 60000))
	{
		error = "前进暂停仅适用于导管：距离须大于0且不超过周期行程，停留时间须为1至60000 ms";
		return false;
	}
	const double angle = is_catheter_motion(config.mode) ? config.axis2_angle_deg : config.axis7_angle_deg;
	if (!std::isfinite(angle) || angle < -360.0 || angle > 360.0)
	{
		error = "周向角度必须在-360至360度之间";
		return false;
	}
	return true;
}

