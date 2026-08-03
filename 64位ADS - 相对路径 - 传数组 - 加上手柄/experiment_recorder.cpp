#include "experiment_recorder.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cwctype>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace
{
	constexpr std::uint32_t kForceRecordRateHz = 100;
	constexpr std::uint32_t kMotionRecordRateHz = 50;

	constexpr const char* kForceHeader =
		"sample_index,elapsed_us,sample_valid,calibrated_valid,"
		"fn_1_raw_v,ft_1_raw_v,fn_1_zero_v,ft_1_zero_v,clean_force_n,clean_handle_torque_nm\n";
	constexpr const char* kMotionHeader =
		"sample_index,elapsed_us,position_valid,"
		"axis1_from_left_mm,axis2_from_left_mm,axis3_from_left_mm,axis4_from_left_mm,"
		"axis5_from_left_mm,axis6_from_left_mm,axis7_from_left_mm,"
		"axis1_handle_valid,axis1_handle_linear_raw,axis1_handle_linear_filtered,"
		"axis1_handle_rotation_raw,axis1_handle_rotation_filtered,"
		"axis6_handle_valid,axis6_handle_linear_raw,axis6_handle_linear_filtered,"
		"axis6_handle_rotation_raw,axis6_handle_rotation_filtered\n";
	constexpr const char* kTransitionHeader =
		"elapsed_us,valid,trial_id,velocity_level,repeat_in_level,phase_code,"
		"phase_elapsed_ms,v_ratio,axis1_from_left_mm,clean_force_n,clean_handle_torque_nm\n";

	bool directory_exists(const std::wstring& path)
	{
		const DWORD attributes = GetFileAttributesW(path.c_str());
		return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
	}

	bool ensure_directory(const std::wstring& path)
	{
		if (directory_exists(path))
		{
			return true;
		}
		return CreateDirectoryW(path.c_str(), nullptr) != FALSE || GetLastError() == ERROR_ALREADY_EXISTS;
	}

	std::wstring join_path(const std::wstring& left, const wchar_t* right)
	{
		std::wstring path = left;
		if (!path.empty() && path.back() != L'\\' && path.back() != L'/')
		{
			path.push_back(L'\\');
		}
		path += right;
		return path;
	}

	std::uint64_t current_utc_filetime_100ns()
	{
		FILETIME file_time{};
		GetSystemTimeAsFileTime(&file_time);
		ULARGE_INTEGER value{};
		value.LowPart = file_time.dwLowDateTime;
		value.HighPart = file_time.dwHighDateTime;
		return value.QuadPart;
	}

	std::string format_utc_filetime(std::uint64_t value_100ns)
	{
		ULARGE_INTEGER value{};
		value.QuadPart = value_100ns;
		FILETIME file_time{};
		file_time.dwLowDateTime = value.LowPart;
		file_time.dwHighDateTime = value.HighPart;
		SYSTEMTIME utc{};
		if (!FileTimeToSystemTime(&file_time, &utc))
		{
			return std::string();
		}
		char text[40] = {};
		std::snprintf(
			text,
			sizeof(text),
			"%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
			static_cast<unsigned int>(utc.wYear),
			static_cast<unsigned int>(utc.wMonth),
			static_cast<unsigned int>(utc.wDay),
			static_cast<unsigned int>(utc.wHour),
			static_cast<unsigned int>(utc.wMinute),
			static_cast<unsigned int>(utc.wSecond),
			static_cast<unsigned int>(utc.wMilliseconds));
		return text;
	}

	std::string json_escape(const std::string& value)
	{
		std::ostringstream escaped;
		for (const unsigned char ch : value)
		{
			switch (ch)
			{
			case '\\': escaped << "\\\\"; break;
			case '"': escaped << "\\\""; break;
			case '\b': escaped << "\\b"; break;
			case '\f': escaped << "\\f"; break;
			case '\n': escaped << "\\n"; break;
			case '\r': escaped << "\\r"; break;
			case '\t': escaped << "\\t"; break;
			default:
				if (ch < 0x20)
				{
					escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
						<< static_cast<int>(ch) << std::dec;
				}
				else
				{
					escaped << static_cast<char>(ch);
				}
				break;
			}
		}
		return escaped.str();
	}

	bool write_double(std::FILE* fp, double value)
	{
		if (!std::isfinite(value))
		{
			return std::fputs("NaN", fp) >= 0;
		}
		return std::fprintf(fp, "%.10g", value) >= 0;
	}

	const char* camera_state_text(Action4CameraState state)
	{
		switch (state)
		{
		case Action4CameraState::Closed: return "Closed";
		case Action4CameraState::Opening: return "Opening";
		case Action4CameraState::Previewing: return "Previewing";
		case Action4CameraState::Recording: return "Recording";
		case Action4CameraState::Missing: return "Missing";
		case Action4CameraState::Disconnected: return "Disconnected";
		case Action4CameraState::Error: return "Error";
		default: return "Unknown";
		}
	}

	const char* camera_format_text(Action4InputFormat format)
	{
		switch (format)
		{
		case Action4InputFormat::H264: return "H264";
		case Action4InputFormat::Mjpeg: return "MJPEG";
		case Action4InputFormat::Rgb32: return "RGB32";
		default: return "Unknown";
		}
	}

	const char* ads_connection_state_text(AdsConnectionState state)
	{
		switch (state)
		{
		case AdsConnectionState::Disconnected: return "Disconnected";
		case AdsConnectionState::Connecting: return "Connecting";
		case AdsConnectionState::Running: return "Running";
		case AdsConnectionState::SoftHold: return "SoftHold";
		case AdsConnectionState::Reconnecting: return "Reconnecting";
		case AdsConnectionState::PlcRestarted: return "PlcRestarted";
		case AdsConnectionState::Error: return "Error";
		default: return "Unknown";
		}
	}

	void append_json_double(std::ostringstream& json, double value)
	{
		if (std::isfinite(value))
		{
			json << value;
		}
		else
		{
			json << "null";
		}
	}
}

SessionClock::SessionClock()
{
	LARGE_INTEGER frequency{};
	if (QueryPerformanceFrequency(&frequency) != FALSE && frequency.QuadPart > 0)
	{
		frequency_ = frequency.QuadPart;
	}
	reset();
}

void SessionClock::reset()
{
	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);
	anchor_qpc_ = now.QuadPart;
	utc_start_filetime_100ns_ = current_utc_filetime_100ns();
}

std::int64_t SessionClock::now_qpc() const
{
	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);
	return now.QuadPart;
}

std::uint64_t SessionClock::elapsed_us(std::int64_t qpc_ticks) const
{
	if (qpc_ticks <= anchor_qpc_ || frequency_ <= 0)
	{
		return 0;
	}
	const std::int64_t delta = qpc_ticks - anchor_qpc_;
	return static_cast<std::uint64_t>((delta * 1000000LL) / frequency_);
}

ExperimentRecorder::ExperimentRecorder() = default;

ExperimentRecorder::~ExperimentRecorder()
{
	stop_and_wait("program_exit");
	camera_.shutdown();
}

void ExperimentRecorder::normalize_force_row(ForceCsvRow& row)
{
	const double nan = std::numeric_limits<double>::quiet_NaN();
	if (!row.sample_valid)
	{
		row.calibrated_valid = false;
		row.fn_1_raw_v = nan;
		row.ft_1_raw_v = nan;
		row.fn_1_zero_v = nan;
		row.ft_1_zero_v = nan;
		row.clean_force_n = nan;
		row.clean_handle_torque_nm = nan;
		return;
	}
	if (!row.calibrated_valid)
	{
		row.fn_1_zero_v = nan;
		row.ft_1_zero_v = nan;
		row.clean_force_n = nan;
		row.clean_handle_torque_nm = nan;
	}
}

void ExperimentRecorder::normalize_motion_row(MotionCsvRow& row)
{
	const double nan = std::numeric_limits<double>::quiet_NaN();
	if (!row.position_valid)
	{
		for (double& value : row.axis_from_left_mm)
		{
			value = nan;
		}
	}
	if (!row.axis1_handle_valid)
	{
		row.axis1_handle_linear_raw = nan;
		row.axis1_handle_linear_filtered = nan;
		row.axis1_handle_rotation_raw = nan;
		row.axis1_handle_rotation_filtered = nan;
	}
	if (!row.axis6_handle_valid)
	{
		row.axis6_handle_linear_raw = nan;
		row.axis6_handle_linear_filtered = nan;
		row.axis6_handle_rotation_raw = nan;
		row.axis6_handle_rotation_filtered = nan;
	}
}

void ExperimentRecorder::normalize_transition_row(ForceTransitionCsvRow& row)
{
	if (row.valid)
	{
		return;
	}
	const double nan = std::numeric_limits<double>::quiet_NaN();
	row.axis1_from_left_mm = nan;
	row.clean_force_n = nan;
	row.clean_handle_torque_nm = nan;
}

bool ExperimentRecorder::write_force_row(std::FILE* fp, const ForceCsvRow& row)
{
	ForceCsvRow normalized = row;
	normalize_force_row(normalized);
	if (std::fprintf(fp, "%llu,%llu,%d,%d,",
		static_cast<unsigned long long>(normalized.sample_index),
		static_cast<unsigned long long>(normalized.elapsed_us),
		normalized.sample_valid ? 1 : 0,
		normalized.calibrated_valid ? 1 : 0) < 0)
	{
		return false;
	}
	const double values[] = {
		normalized.fn_1_raw_v, normalized.ft_1_raw_v, normalized.fn_1_zero_v, normalized.ft_1_zero_v,
		normalized.clean_force_n, normalized.clean_handle_torque_nm
	};
	for (std::size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
	{
		if (!write_double(fp, values[i]) || std::fputc(i + 1 == sizeof(values) / sizeof(values[0]) ? '\n' : ',', fp) == EOF)
		{
			return false;
		}
	}
	return true;
}

bool ExperimentRecorder::write_motion_row(std::FILE* fp, const MotionCsvRow& row)
{
	MotionCsvRow normalized = row;
	normalize_motion_row(normalized);
	if (std::fprintf(fp, "%llu,%llu,%d,",
		static_cast<unsigned long long>(normalized.sample_index),
		static_cast<unsigned long long>(normalized.elapsed_us),
		normalized.position_valid ? 1 : 0) < 0)
	{
		return false;
	}
	for (int axis = 0; axis < 7; ++axis)
	{
		if (!write_double(fp, normalized.axis_from_left_mm[axis]) || std::fputc(',', fp) == EOF)
		{
			return false;
		}
	}
	if (std::fprintf(fp, "%d,", normalized.axis1_handle_valid ? 1 : 0) < 0)
	{
		return false;
	}
	const double axis1_values[] = {
		normalized.axis1_handle_linear_raw, normalized.axis1_handle_linear_filtered,
		normalized.axis1_handle_rotation_raw, normalized.axis1_handle_rotation_filtered
	};
	for (double value : axis1_values)
	{
		if (!write_double(fp, value) || std::fputc(',', fp) == EOF)
		{
			return false;
		}
	}
	if (std::fprintf(fp, "%d,", normalized.axis6_handle_valid ? 1 : 0) < 0)
	{
		return false;
	}
	const double axis6_values[] = {
		normalized.axis6_handle_linear_raw, normalized.axis6_handle_linear_filtered,
		normalized.axis6_handle_rotation_raw, normalized.axis6_handle_rotation_filtered
	};
	for (std::size_t i = 0; i < sizeof(axis6_values) / sizeof(axis6_values[0]); ++i)
	{
		if (!write_double(fp, axis6_values[i]) ||
			std::fputc(i + 1 == sizeof(axis6_values) / sizeof(axis6_values[0]) ? '\n' : ',', fp) == EOF)
		{
			return false;
		}
	}
	return true;
}

bool ExperimentRecorder::write_transition_row(std::FILE* fp, const ForceTransitionCsvRow& row)
{
	ForceTransitionCsvRow normalized = row;
	normalize_transition_row(normalized);
	if (std::fprintf(fp, "%llu,%d,%d,%d,%d,%d,%llu,",
		static_cast<unsigned long long>(normalized.elapsed_us),
		normalized.valid ? 1 : 0,
		normalized.trial_id,
		normalized.velocity_level,
		normalized.repeat_in_level,
		normalized.phase_code,
		static_cast<unsigned long long>(normalized.phase_elapsed_ms)) < 0)
	{
		return false;
	}
	const double values[] = {
		normalized.v_ratio, normalized.axis1_from_left_mm,
		normalized.clean_force_n, normalized.clean_handle_torque_nm
	};
	for (std::size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
	{
		if (!write_double(fp, values[i]) || std::fputc(i + 1 == sizeof(values) / sizeof(values[0]) ? '\n' : ',', fp) == EOF)
		{
			return false;
		}
	}
	return true;
}

std::wstring ExperimentRecorder::utf8_to_wide(const std::string& text)
{
	if (text.empty())
	{
		return std::wstring();
	}
	const int length = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()), nullptr, 0);
	if (length <= 0)
	{
		return std::wstring();
	}
	std::wstring result(static_cast<std::size_t>(length), L'\0');
	MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()), &result[0], length);
	return result;
}

std::string ExperimentRecorder::wide_to_utf8(const std::wstring& text)
{
	if (text.empty())
	{
		return std::string();
	}
	const int length = WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
	if (length <= 0)
	{
		return std::string();
	}
	std::string result(static_cast<std::size_t>(length), '\0');
	WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), &result[0], length, nullptr, nullptr);
	return result;
}

std::wstring ExperimentRecorder::sanitize_experiment_name(const std::wstring& raw_name)
{
	std::size_t begin = 0;
	while (begin < raw_name.size() && std::iswspace(raw_name[begin]))
	{
		++begin;
	}
	std::size_t end = raw_name.size();
	while (end > begin && std::iswspace(raw_name[end - 1]))
	{
		--end;
	}

	std::wstring cleaned;
	cleaned.reserve(std::min<std::size_t>(64, end - begin));
	for (std::size_t i = begin; i < end && cleaned.size() < 64; ++i)
	{
		const wchar_t ch = raw_name[i];
		const bool invalid = ch < 32 || ch == L'<' || ch == L'>' || ch == L':' || ch == L'"' ||
			ch == L'/' || ch == L'\\' || ch == L'|' || ch == L'?' || ch == L'*';
		cleaned.push_back(invalid ? L'_' : ch);
	}
	if (!cleaned.empty() && cleaned.back() >= 0xD800 && cleaned.back() <= 0xDBFF)
	{
		cleaned.pop_back();
	}
	while (!cleaned.empty() && (cleaned.back() == L'.' || std::iswspace(cleaned.back())))
	{
		cleaned.pop_back();
	}
	if (cleaned.empty())
	{
		cleaned = L"session";
	}

	std::wstring upper = cleaned;
	std::transform(upper.begin(), upper.end(), upper.begin(), [](wchar_t ch) { return static_cast<wchar_t>(std::towupper(ch)); });
	const std::size_t extension = upper.find(L'.');
	const std::wstring device_base = upper.substr(0, extension);
	static const wchar_t* reserved[] = {
		L"CON", L"PRN", L"AUX", L"NUL", L"COM1", L"COM2", L"COM3", L"COM4", L"COM5",
		L"COM6", L"COM7", L"COM8", L"COM9", L"LPT1", L"LPT2", L"LPT3", L"LPT4", L"LPT5",
		L"LPT6", L"LPT7", L"LPT8", L"LPT9"
	};
	for (const wchar_t* name : reserved)
	{
		if (device_base == name)
		{
			cleaned.insert(cleaned.begin(), L'_');
			break;
		}
	}
	return cleaned;
}

bool ExperimentRecorder::create_session_paths(const std::wstring& cleaned_name)
{
	const DWORD required = GetCurrentDirectoryW(0, nullptr);
	if (required == 0)
	{
		return false;
	}
	std::vector<wchar_t> current_directory(required + 1, L'\0');
	if (GetCurrentDirectoryW(static_cast<DWORD>(current_directory.size()), current_directory.data()) == 0)
	{
		return false;
	}

	const std::wstring records_directory = join_path(current_directory.data(), L"records");
	if (!ensure_directory(records_directory))
	{
		return false;
	}
	SYSTEMTIME local{};
	GetLocalTime(&local);
	wchar_t date[16] = {};
	wchar_t time[16] = {};
	swprintf_s(date, L"%04u%02u%02u", local.wYear, local.wMonth, local.wDay);
	swprintf_s(time, L"%02u%02u%02u", local.wHour, local.wMinute, local.wSecond);
	const std::wstring date_directory = join_path(records_directory, date);
	if (!ensure_directory(date_directory))
	{
		return false;
	}

	const std::wstring base_name = std::wstring(time) + L"_" + cleaned_name;
	for (unsigned int suffix = 0; suffix <= 9999; ++suffix)
	{
		std::wstring candidate_name = base_name;
		if (suffix > 0)
		{
			wchar_t suffix_text[16] = {};
			swprintf_s(suffix_text, L"_%03u", suffix);
			candidate_name += suffix_text;
		}
		const std::wstring candidate = join_path(date_directory, candidate_name.c_str());
		if (CreateDirectoryW(candidate.c_str(), nullptr) != FALSE)
		{
			session_directory_ = candidate;
			force_path_ = join_path(candidate, L"force.csv");
			motion_path_ = join_path(candidate, L"motion.csv");
			video_path_ = join_path(candidate, L"video.mp4");
			video_frames_path_ = join_path(candidate, L"video_frames.csv");
			metadata_path_ = join_path(candidate, L"session.json");
			return true;
		}
		if (GetLastError() != ERROR_ALREADY_EXISTS)
		{
			return false;
		}
	}
	return false;
}

bool ExperimentRecorder::start(const std::string& experiment_name_utf8, const ExperimentStartInfo& info)
{
	ExperimentRecordingState state = state_.load(std::memory_order_acquire);
	if (state == ExperimentRecordingState::Starting || state == ExperimentRecordingState::Recording ||
		state == ExperimentRecordingState::Stopping)
	{
		return false;
	}
	if (stop_thread_.joinable())
	{
		stop_thread_.join();
	}

	state_.store(ExperimentRecordingState::Starting, std::memory_order_release);
	error_.store(ExperimentRecordingError::None, std::memory_order_release);
	raw_name_utf8_ = experiment_name_utf8;
	const std::wstring raw_wide = utf8_to_wide(experiment_name_utf8);
	if (!experiment_name_utf8.empty() && raw_wide.empty())
	{
		error_.store(ExperimentRecordingError::InvalidName, std::memory_order_release);
		state_.store(ExperimentRecordingState::Error, std::memory_order_release);
		return false;
	}
	cleaned_name_ = sanitize_experiment_name(raw_wide);
	start_info_ = info;

	if (!create_session_paths(cleaned_name_))
	{
		error_.store(ExperimentRecordingError::CreateDirectoryFailed, std::memory_order_release);
		state_.store(ExperimentRecordingState::Error, std::memory_order_release);
		return false;
	}

	clock_.reset();
	force_sample_index_ = 0;
	motion_sample_index_ = 0;
	force_first_ads_sequence_ = 0;
	force_last_ads_sequence_ = 0;
	motion_first_ads_sequence_ = 0;
	motion_last_ads_sequence_ = 0;
	transition_first_force_sequence_ = 0;
	transition_last_force_sequence_ = 0;
	force_ads_accepted_.store(0, std::memory_order_relaxed);
	force_ads_invalid_.store(0, std::memory_order_relaxed);
	force_ads_pre_session_rejected_.store(0, std::memory_order_relaxed);
	force_ads_duplicate_rejected_.store(0, std::memory_order_relaxed);
	force_ads_sequence_skipped_.store(0, std::memory_order_relaxed);
	motion_ads_accepted_.store(0, std::memory_order_relaxed);
	motion_ads_invalid_.store(0, std::memory_order_relaxed);
	motion_ads_pre_session_rejected_.store(0, std::memory_order_relaxed);
	motion_ads_duplicate_rejected_.store(0, std::memory_order_relaxed);
	motion_ads_sequence_skipped_.store(0, std::memory_order_relaxed);
	transition_ads_accepted_.store(0, std::memory_order_relaxed);
	transition_ads_invalid_.store(0, std::memory_order_relaxed);
	transition_ads_pre_session_rejected_.store(0, std::memory_order_relaxed);
	transition_ads_duplicate_rejected_.store(0, std::memory_order_relaxed);
	transition_ads_sequence_skipped_.store(0, std::memory_order_relaxed);
	transition_file_index_ = 0;
	transition_dropped_completed_ = 0;
	transition_current_pending_ = false;
	transition_writer_used_.store(false, std::memory_order_release);
	{
		std::lock_guard<std::mutex> lock(ads_stats_mutex_);
		ads_stats_baseline_ = ads_stats_latest_;
		ads_stats_baseline_available_ = ads_stats_latest_available_;
		ads_stats_session_end_ = AdsCommunicationStats{};
		ads_stats_session_end_available_ = false;
		ads_previous_failed_cycles_ = ads_stats_baseline_.failed_cycles;
		ads_previous_consecutive_failures_ = ads_stats_baseline_.consecutive_failures;
		ads_pre_session_failure_streak_ = ads_stats_baseline_available_
			? ads_stats_baseline_.consecutive_failures
			: 0;
		ads_session_max_consecutive_failures_ = 0;
		ads_failure_observation_available_ = ads_stats_baseline_available_;
	}

	if (!force_writer_.start(force_path_, kForceHeader, &ExperimentRecorder::write_force_row))
	{
		error_.store(ExperimentRecordingError::OpenForceCsvFailed, std::memory_order_release);
		rollback_failed_start();
		state_.store(ExperimentRecordingState::Error, std::memory_order_release);
		return false;
	}
	if (!motion_writer_.start(motion_path_, kMotionHeader, &ExperimentRecorder::write_motion_row))
	{
		error_.store(ExperimentRecordingError::OpenMotionCsvFailed, std::memory_order_release);
		rollback_failed_start();
		state_.store(ExperimentRecordingState::Error, std::memory_order_release);
		return false;
	}

	write_session_json(false, std::string());
	state_.store(ExperimentRecordingState::Recording, std::memory_order_release);
	camera_.start_recording(video_path_, video_frames_path_, clock_.anchor_qpc(), clock_.frequency());
	return true;
}

void ExperimentRecorder::rollback_failed_start()
{
	motion_writer_.stop();
	force_writer_.stop();
	if (!motion_path_.empty()) DeleteFileW(motion_path_.c_str());
	if (!force_path_.empty()) DeleteFileW(force_path_.c_str());
	if (!metadata_path_.empty()) DeleteFileW(metadata_path_.c_str());
	if (!session_directory_.empty()) RemoveDirectoryW(session_directory_.c_str());
}

void ExperimentRecorder::stop_async(const char* reason)
{
	ExperimentRecordingState expected = ExperimentRecordingState::Recording;
	{
		// 状态切换与 ADS 统计冻结共用一把锁，避免停止边界多算一次后台更新。
		std::lock_guard<std::mutex> lock(ads_stats_mutex_);
		if (!state_.compare_exchange_strong(
			expected, ExperimentRecordingState::Stopping, std::memory_order_acq_rel))
		{
			return;
		}
		ads_stats_session_end_ = ads_stats_latest_;
		ads_stats_session_end_available_ = ads_stats_latest_available_;
	}
	if (stop_thread_.joinable())
	{
		stop_thread_.join();
	}
	try
	{
		stop_thread_ = std::thread(&ExperimentRecorder::stop_worker, this, reason != nullptr ? reason : "user_stop");
	}
	catch (...)
	{
		error_.store(ExperimentRecordingError::StopThreadFailed, std::memory_order_release);
		stop_worker(reason != nullptr ? reason : "stop_thread_failed");
	}
}

void ExperimentRecorder::stop_and_wait(const char* reason)
{
	stop_async(reason);
	if (stop_thread_.joinable())
	{
		stop_thread_.join();
	}
}

void ExperimentRecorder::stop_worker(std::string reason)
{
	camera_.stop_recording_and_wait();
	transition_writer_.stop();
	force_writer_.stop();
	motion_writer_.stop();
	write_session_json(true, reason);
	state_.store(ExperimentRecordingState::Idle, std::memory_order_release);
}

void ExperimentRecorder::poll_health()
{
	if (!is_recording())
	{
		return;
	}
	if (force_writer_.has_error())
	{
		error_.store(ExperimentRecordingError::ForceWriterFailed, std::memory_order_release);
		stop_async("force_writer_error");
	}
	else if (motion_writer_.has_error())
	{
		error_.store(ExperimentRecordingError::MotionWriterFailed, std::memory_order_release);
		stop_async("motion_writer_error");
	}
	else if (transition_writer_used_.load(std::memory_order_acquire) && transition_writer_.has_error())
	{
		error_.store(ExperimentRecordingError::TransitionWriterFailed, std::memory_order_release);
		stop_async("force_transition_writer_error");
	}
	else if (camera_.timing_writer_failed())
	{
		error_.store(ExperimentRecordingError::VideoFrameWriterFailed, std::memory_order_release);
		stop_async("video_frame_writer_error");
	}
}

bool ExperimentRecorder::accept_ads_snapshot(
	std::uint64_t sequence,
	std::int64_t qpc_ticks,
	std::uint64_t expected_sequence_step,
	std::uint64_t& first_sequence,
	std::uint64_t& last_sequence,
	std::atomic<std::uint64_t>& accepted,
	std::atomic<std::uint64_t>& invalid,
	bool row_valid,
	std::atomic<std::uint64_t>& pre_session_rejected,
	std::atomic<std::uint64_t>& duplicate_rejected,
	std::atomic<std::uint64_t>& sequence_skipped)
{
	// sequence=0 保留给非 ADS 数据源；这类行仍沿用调用方提供的时间戳。
	if (sequence == 0)
	{
		return true;
	}
	if (qpc_ticks <= 0 || qpc_ticks < clock_.anchor_qpc())
	{
		pre_session_rejected.fetch_add(1, std::memory_order_relaxed);
		return false;
	}
	if (last_sequence != 0 && sequence <= last_sequence)
	{
		duplicate_rejected.fetch_add(1, std::memory_order_relaxed);
		return false;
	}
	const std::uint64_t sequence_step = sequence - last_sequence;
	if (last_sequence != 0 && expected_sequence_step > 0 && sequence_step > expected_sequence_step)
	{
		// 力表每个 ADS 序号一行，运动表每两个序号一行；这里统计缺失的目标行数。
		const std::uint64_t skipped_rows = (sequence_step - 1) / expected_sequence_step;
		sequence_skipped.fetch_add(skipped_rows, std::memory_order_relaxed);
	}
	if (first_sequence == 0)
	{
		first_sequence = sequence;
	}
	last_sequence = sequence;
	accepted.fetch_add(1, std::memory_order_relaxed);
	if (!row_valid)
	{
		invalid.fetch_add(1, std::memory_order_relaxed);
	}
	return true;
}

bool ExperimentRecorder::enqueue_force(const ForceCsvRow& row)
{
	if (!is_recording()) return false;
	if (!accept_ads_snapshot(
		row.ads_snapshot_sequence,
		row.source_qpc_ticks,
		1,
		force_first_ads_sequence_,
		force_last_ads_sequence_,
		force_ads_accepted_,
		force_ads_invalid_,
		row.sample_valid,
		force_ads_pre_session_rejected_,
		force_ads_duplicate_rejected_,
		force_ads_sequence_skipped_))
	{
		return false;
	}
	ForceCsvRow indexed = row;
	if (indexed.source_qpc_ticks >= clock_.anchor_qpc())
	{
		indexed.elapsed_us = clock_.elapsed_us(indexed.source_qpc_ticks);
	}
	normalize_force_row(indexed);
	indexed.sample_index = force_sample_index_++;
	return force_writer_.try_enqueue(indexed);
}

bool ExperimentRecorder::enqueue_motion(const MotionCsvRow& row)
{
	if (!is_recording()) return false;
	if (!accept_ads_snapshot(
		row.ads_snapshot_sequence,
		row.source_qpc_ticks,
		2,
		motion_first_ads_sequence_,
		motion_last_ads_sequence_,
		motion_ads_accepted_,
		motion_ads_invalid_,
		row.position_valid,
		motion_ads_pre_session_rejected_,
		motion_ads_duplicate_rejected_,
		motion_ads_sequence_skipped_))
	{
		return false;
	}
	MotionCsvRow indexed = row;
	if (indexed.source_qpc_ticks >= clock_.anchor_qpc())
	{
		indexed.elapsed_us = clock_.elapsed_us(indexed.source_qpc_ticks);
	}
	normalize_motion_row(indexed);
	indexed.sample_index = motion_sample_index_++;
	return motion_writer_.try_enqueue(indexed);
}

bool ExperimentRecorder::start_force_transition_log()
{
	if (!is_recording() || transition_writer_.is_running() || !transition_writer_.stop_completed())
	{
		return false;
	}
	// 回收上一次异步停止后的线程；该等待只发生在用户显式启动下一轮实验时。
	if (transition_current_pending_)
	{
		transition_dropped_completed_ += transition_writer_.dropped_count();
		transition_current_pending_ = false;
	}
	transition_writer_.stop();
	++transition_file_index_;
	wchar_t file_name[64] = {};
	swprintf_s(file_name, L"force_transition_%03u.csv", transition_file_index_);
	transition_writer_used_.store(true, std::memory_order_release);
	const bool started = transition_writer_.start(
		join_path(session_directory_, file_name),
		kTransitionHeader,
		&ExperimentRecorder::write_transition_row);
	transition_current_pending_ = started;
	return started;
}

void ExperimentRecorder::stop_force_transition_log()
{
	transition_writer_.request_stop();
}

bool ExperimentRecorder::enqueue_force_transition(const ForceTransitionCsvRow& row)
{
	if (!is_recording() || !transition_writer_.is_running())
	{
		return false;
	}
	if (!accept_ads_snapshot(
		row.force_snapshot_sequence,
		row.source_qpc_ticks,
		1,
		transition_first_force_sequence_,
		transition_last_force_sequence_,
		transition_ads_accepted_,
		transition_ads_invalid_,
		row.valid,
		transition_ads_pre_session_rejected_,
		transition_ads_duplicate_rejected_,
		transition_ads_sequence_skipped_))
	{
		return false;
	}
	ForceTransitionCsvRow normalized = row;
	if (normalized.source_qpc_ticks >= clock_.anchor_qpc())
	{
		normalized.elapsed_us = clock_.elapsed_us(normalized.source_qpc_ticks);
	}
	normalize_transition_row(normalized);
	return transition_writer_.try_enqueue(normalized);
}

void ExperimentRecorder::update_ads_communication_stats(const AdsCommunicationStats& stats)
{
	std::lock_guard<std::mutex> lock(ads_stats_mutex_);
	ads_stats_latest_ = stats;
	ads_stats_latest_available_ = true;
	if (!is_recording())
	{
		return;
	}
	if (!ads_stats_baseline_available_)
	{
		ads_stats_baseline_ = stats;
		ads_stats_baseline_available_ = true;
		ads_previous_failed_cycles_ = stats.failed_cycles;
		ads_previous_consecutive_failures_ = stats.consecutive_failures;
		ads_pre_session_failure_streak_ = stats.consecutive_failures;
		ads_failure_observation_available_ = true;
		return;
	}

	const std::uint64_t failed_delta = ads_failure_observation_available_
		? counter_delta(stats.failed_cycles, ads_previous_failed_cycles_)
		: 0;
	const std::uint64_t consecutive_delta =
		stats.consecutive_failures >= ads_previous_consecutive_failures_
			? stats.consecutive_failures - ads_previous_consecutive_failures_
			: 0;
	const bool same_failure_streak = ads_failure_observation_available_ &&
		ads_previous_consecutive_failures_ > 0 &&
		stats.consecutive_failures >= ads_previous_consecutive_failures_ &&
		failed_delta == consecutive_delta;
	if (stats.consecutive_failures == 0)
	{
		ads_pre_session_failure_streak_ = 0;
	}
	else if (!same_failure_streak)
	{
		// 上一次观测后已经成功过，当前是会话内新的连续失败段。
		ads_pre_session_failure_streak_ = 0;
	}
	const std::uint64_t session_streak =
		stats.consecutive_failures >= ads_pre_session_failure_streak_
			? stats.consecutive_failures - ads_pre_session_failure_streak_
			: stats.consecutive_failures;
	ads_session_max_consecutive_failures_ = (std::max)(
		ads_session_max_consecutive_failures_, session_streak);
	ads_previous_failed_cycles_ = stats.failed_cycles;
	ads_previous_consecutive_failures_ = stats.consecutive_failures;
	ads_failure_observation_available_ = true;
}

std::uint64_t ExperimentRecorder::counter_delta(std::uint64_t current, std::uint64_t baseline)
{
	return current >= baseline ? current - baseline : current;
}

void ExperimentRecorder::set_camera_preview(bool enabled)
{
	camera_.set_preview_enabled(enabled);
}

Action4CameraSnapshot ExperimentRecorder::camera_snapshot() const
{
	return camera_.snapshot();
}

ExperimentRecorderSnapshot ExperimentRecorder::snapshot() const
{
	ExperimentRecorderSnapshot result;
	result.state = state_.load(std::memory_order_acquire);
	result.error = error_.load(std::memory_order_acquire);
	if (result.state == ExperimentRecordingState::Recording || result.state == ExperimentRecordingState::Stopping)
	{
		result.elapsed_us = clock_.elapsed_us(clock_.now_qpc());
	}
	result.force_dropped = force_writer_.dropped_count();
	result.motion_dropped = motion_writer_.dropped_count();
	{
		std::lock_guard<std::mutex> lock(ads_stats_mutex_);
		const bool use_session_end = result.state != ExperimentRecordingState::Recording &&
			ads_stats_session_end_available_;
		const AdsCommunicationStats& observed = use_session_end
			? ads_stats_session_end_
			: ads_stats_latest_;
		const bool observed_available = use_session_end
			? ads_stats_session_end_available_
			: ads_stats_latest_available_;
		if (observed_available && ads_stats_baseline_available_)
		{
			// wire 字段保持兼容：力表直接对应 100 Hz ADS 时隙。
			result.force_missed = counter_delta(
				observed.missed_deadlines, ads_stats_baseline_.missed_deadlines);
		}
		// 运动表是从偶数 ADS 快照抽取的 50 Hz 表，不存在独立调度器可供统计。
		result.motion_missed = 0;
	}
	result.ads_pre_session_rejected =
		force_ads_pre_session_rejected_.load(std::memory_order_acquire) +
		motion_ads_pre_session_rejected_.load(std::memory_order_acquire) +
		transition_ads_pre_session_rejected_.load(std::memory_order_acquire);
	result.ads_duplicate_rejected =
		force_ads_duplicate_rejected_.load(std::memory_order_acquire) +
		motion_ads_duplicate_rejected_.load(std::memory_order_acquire) +
		transition_ads_duplicate_rejected_.load(std::memory_order_acquire);
	result.camera = camera_.snapshot();
	return result;
}

void ExperimentRecorder::write_session_json(bool final, const std::string& stop_reason)
{
	if (metadata_path_.empty())
	{
		return;
	}
	const Action4CameraSnapshot camera = camera_.snapshot();
	AdsCommunicationStats ads_baseline{};
	AdsCommunicationStats ads_latest{};
	bool ads_metrics_available = false;
	bool ads_baseline_available = false;
	std::uint64_t ads_session_max_consecutive_failures = 0;
	{
		std::lock_guard<std::mutex> lock(ads_stats_mutex_);
		ads_baseline = ads_stats_baseline_;
		if (final)
		{
			ads_latest = ads_stats_session_end_;
			ads_metrics_available = ads_stats_session_end_available_;
		}
		else
		{
			ads_latest = ads_stats_latest_;
			ads_metrics_available = ads_stats_latest_available_;
		}
		ads_baseline_available = ads_stats_baseline_available_;
		ads_session_max_consecutive_failures = ads_session_max_consecutive_failures_;
	}
	const bool ads_session_counters_available = ads_metrics_available && ads_baseline_available;
	const std::uint64_t ads_missed_deadlines_during_session = ads_session_counters_available
		? counter_delta(ads_latest.missed_deadlines, ads_baseline.missed_deadlines)
		: 0;
	std::ostringstream json;
	json << "{\n";
	json << "  \"schema_version\": 3,\n";
	json << "  \"experiment_name_raw\": \"" << json_escape(raw_name_utf8_) << "\",\n";
	json << "  \"experiment_name_cleaned\": \"" << json_escape(wide_to_utf8(cleaned_name_)) << "\",\n";
	json << "  \"utc_start\": \"" << format_utc_filetime(clock_.utc_start_filetime_100ns()) << "\",\n";
	json << "  \"utc_end\": ";
	if (final) json << "\"" << format_utc_filetime(current_utc_filetime_100ns()) << "\"";
	else json << "null";
	json << ",\n";
	json << "  \"qpc_frequency\": " << clock_.frequency() << ",\n";
	json << "  \"qpc_anchor\": " << clock_.anchor_qpc() << ",\n";
	json << "  \"force_rate_hz\": " << kForceRecordRateHz << ",\n";
	json << "  \"motion_rate_hz\": " << kMotionRecordRateHz << ",\n";
	json << "  \"axis1_handle_serial\": " << start_info_.axis1_handle_serial << ",\n";
	json << "  \"axis6_handle_serial\": " << start_info_.axis6_handle_serial << ",\n";
	json << "  \"single_handle_mode\": " << (start_info_.single_handle_mode ? "true" : "false") << ",\n";
	json << "  \"force_writer_dropped\": " << force_writer_.dropped_count() << ",\n";
	json << "  \"motion_writer_dropped\": " << motion_writer_.dropped_count() << ",\n";
	json << "  \"force_schedule_missed\": ";
	if (ads_session_counters_available) json << ads_missed_deadlines_during_session;
	else json << "null";
	json << ",\n";
	json << "  \"motion_schedule_missed\": null,\n";
	json << "  \"schedule_missed_basis\": {\n";
	json << "    \"force\": \"ads_100hz_missed_deadlines_during_session\",\n";
	json << "    \"motion\": \"not_applicable_even_ads_snapshot_selection\"\n";
	json << "  },\n";
	const std::uint64_t transition_dropped = transition_dropped_completed_ +
		(transition_current_pending_ ? transition_writer_.dropped_count() : 0);
	json << "  \"force_transition_writer_dropped\": " << transition_dropped << ",\n";
	json << "  \"force_transition_writer_error\": "
		<< ((transition_writer_used_.load() && transition_writer_.has_error()) ? "true" : "false") << ",\n";
	json << "  \"ads\": {\n";
	json << "    \"metrics_available\": " << (ads_metrics_available ? "true" : "false") << ",\n";
	json << "    \"baseline_available\": " << (ads_baseline_available ? "true" : "false") << ",\n";
	json << "    \"connection_state_at_session_end\": ";
	if (final && ads_metrics_available)
		json << "\"" << ads_connection_state_text(ads_latest.state) << "\"";
	else
		json << "null";
	json << ",\n";
	json << "    \"actual_hz_at_session_end\": ";
	if (final && ads_metrics_available) append_json_double(json, ads_latest.actual_hz);
	else json << "null";
	json << ",\n";
	json << "    \"snapshot_age_us_at_session_end\": ";
	if (final && ads_metrics_available) json << ads_latest.latest_snapshot_age_us;
	else json << "null";
	json << ",\n";
	json << "    \"rtt_us_at_session_end\": ";
	if (final && ads_metrics_available) json << ads_latest.latest_rtt_us;
	else json << "null";
	json << ",\n";
	json << "    \"failed_cycles_during_session\": ";
	if (ads_session_counters_available)
		json << counter_delta(ads_latest.failed_cycles, ads_baseline.failed_cycles);
	else
		json << "null";
	json << ",\n";
	json << "    \"consecutive_failures_at_session_end\": ";
	if (final && ads_metrics_available) json << ads_latest.consecutive_failures;
	else json << "null";
	json << ",\n";
	json << "    \"max_consecutive_failures_during_session\": ";
	if (ads_session_counters_available)
		json << ads_session_max_consecutive_failures;
	else
		json << "null";
	json << ",\n";
	json << "    \"reconnects_during_session\": ";
	if (ads_session_counters_available)
		json << counter_delta(ads_latest.reconnect_count, ads_baseline.reconnect_count);
	else
		json << "null";
	json << ",\n";
	json << "    \"plc_restarts_during_session\": ";
	if (ads_session_counters_available)
		json << counter_delta(ads_latest.plc_restart_count, ads_baseline.plc_restart_count);
	else
		json << "null";
	json << ",\n";
	json << "    \"missed_deadlines_total_at_session_end\": ";
	if (final && ads_metrics_available)
		json << ads_latest.missed_deadlines;
	else
		json << "null";
	json << ",\n";
	json << "    \"missed_deadlines_during_session\": ";
	if (ads_session_counters_available)
		json << ads_missed_deadlines_during_session;
	else
		json << "null";
	json << ",\n";
	json << "    \"snapshot_queue_dropped_during_session\": ";
	if (ads_session_counters_available)
	{
		json << counter_delta(ads_latest.snapshot_queue_dropped, ads_baseline.snapshot_queue_dropped);
	}
	else
	{
		json << "null";
	}
	json << ",\n";
	json << "    \"recorder\": {\n";
	json << "      \"force_first_sequence\": " << force_first_ads_sequence_ << ",\n";
	json << "      \"force_last_sequence\": " << force_last_ads_sequence_ << ",\n";
	json << "      \"force_accepted\": " << force_ads_accepted_.load() << ",\n";
	json << "      \"force_invalid\": " << force_ads_invalid_.load() << ",\n";
	json << "      \"force_pre_session_rejected\": " << force_ads_pre_session_rejected_.load() << ",\n";
	json << "      \"force_duplicate_rejected\": " << force_ads_duplicate_rejected_.load() << ",\n";
	json << "      \"force_sequences_skipped\": " << force_ads_sequence_skipped_.load() << ",\n";
	json << "      \"motion_first_sequence\": " << motion_first_ads_sequence_ << ",\n";
	json << "      \"motion_last_sequence\": " << motion_last_ads_sequence_ << ",\n";
	json << "      \"motion_accepted\": " << motion_ads_accepted_.load() << ",\n";
	json << "      \"motion_invalid\": " << motion_ads_invalid_.load() << ",\n";
	json << "      \"motion_pre_session_rejected\": " << motion_ads_pre_session_rejected_.load() << ",\n";
	json << "      \"motion_duplicate_rejected\": " << motion_ads_duplicate_rejected_.load() << ",\n";
	json << "      \"motion_sequences_skipped\": " << motion_ads_sequence_skipped_.load() << ",\n";
	json << "      \"transition_first_force_sequence\": " << transition_first_force_sequence_ << ",\n";
	json << "      \"transition_last_force_sequence\": " << transition_last_force_sequence_ << ",\n";
	json << "      \"transition_accepted\": " << transition_ads_accepted_.load() << ",\n";
	json << "      \"transition_invalid\": " << transition_ads_invalid_.load() << ",\n";
	json << "      \"transition_pre_session_rejected\": " << transition_ads_pre_session_rejected_.load() << ",\n";
	json << "      \"transition_duplicate_rejected\": " << transition_ads_duplicate_rejected_.load() << ",\n";
	json << "      \"transition_sequences_skipped\": " << transition_ads_sequence_skipped_.load() << "\n";
	json << "    }\n";
	json << "  },\n";
	json << "  \"stop_reason\": ";
	if (final) json << "\"" << json_escape(stop_reason) << "\"";
	else json << "null";
	json << ",\n";
	json << "  \"camera\": {\n";
	json << "    \"device_name\": \"OsmoAction4\",\n";
	json << "    \"state\": \"" << camera_state_text(camera.state) << "\",\n";
	json << "    \"input_format\": \"" << camera_format_text(camera.input_format) << "\",\n";
	json << "    \"output_codec\": \"H264\",\n";
	json << "    \"audio\": false,\n";
	json << "    \"width\": " << camera.width << ",\n";
	json << "    \"height\": " << camera.height << ",\n";
	json << "    \"fps_numerator\": " << camera.fps_numerator << ",\n";
	json << "    \"fps_denominator\": " << camera.fps_denominator << ",\n";
	json << "    \"frame_count\": " << camera.frame_count << ",\n";
	json << "    \"dropped_frames\": " << camera.dropped_frames << ",\n";
	json << "    \"timing_rows_dropped\": " << camera.timing_rows_dropped << ",\n";
	json << "    \"timing_writer_error\": " << (camera.timing_writer_error ? "true" : "false") << ",\n";
	json << "    \"error_code\": " << camera.error_code << "\n";
	json << "  }\n";
	json << "}\n";

	std::FILE* fp = nullptr;
	if (_wfopen_s(&fp, metadata_path_.c_str(), L"wb") != 0 || fp == nullptr)
	{
		return;
	}
	const std::string data = json.str();
	std::fwrite(data.data(), 1, data.size(), fp);
	std::fflush(fp);
	std::fclose(fp);
}
