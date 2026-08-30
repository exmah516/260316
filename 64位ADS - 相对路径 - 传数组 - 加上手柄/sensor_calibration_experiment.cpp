#include "sensor_calibration_experiment.h"

#include "force_calibration.h"
#include "force_feedback.h"
#include "plc_io.h"

#include <ADSComm1.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <shellapi.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <windows.h>

#pragma comment(lib, "Shell32.lib")

namespace
{
	constexpr int kSensorCount = 4;
	constexpr int kChannelCount = 4;
	constexpr int kPointCount = 6;
	constexpr int kSampleCount = 100;
	constexpr int kMinimumValidSamples = 80;
	constexpr int kSamplePeriodMs = 50;
	constexpr double kMinimumAttemptedRateHz = 18.0;
	constexpr double kOutlierThresholdCounts = 300.0;
	constexpr double kGravityMps2 = 9.80665;
	constexpr double kMinimumR2 = 0.98;
	constexpr double kMinimumInputSpanCounts = 1.0;
	constexpr double kMinimumSpanToNoiseRatio = 5.0;

	const std::array<int, kPointCount> kWeightsG = { 0, 2, 5, 10, 20, 50 };
	const std::array<const char*, kChannelCount> kChannelSymbols = {
		"G.ft_1_value",
		"G.fn_1_value",
		"G.fn_2_value",
		"G.ft_2_value"
	};
	const std::array<const char*, kChannelCount> kChannelShortNames = {
		"ft_1_value",
		"fn_1_value",
		"fn_2_value",
		"ft_2_value"
	};
	const std::array<const wchar_t*, kSensorCount> kOriginalFileNames = {
		L"传感器1标定数据.csv",
		L"传感器2标定数据.csv",
		L"传感器3标定数据.csv",
		L"传感器4标定数据.csv"
	};

	struct Options
	{
		std::wstring calibration_directory;
		bool validate_only = false;
		bool self_test = false;
		bool show_help = false;
	};

	struct OriginalCalibration
	{
		std::wstring path;
		double slope = 0.0;
		double intercept = 0.0;
		std::array<double, kPointCount> reference_means{};
		std::array<double, kPointCount> reference_stddevs{};
	};

	struct ChannelStats
	{
		int total_count = 0;
		int valid_count = 0;
		double median = 0.0;
		double mean = 0.0;
		double stddev = 0.0;
	};

	struct PointMeasurement
	{
		int weight_g = 0;
		int ads_failures = 0;
		int schedule_overruns = 0;
		double elapsed_seconds = 0.0;
		double attempted_rate_hz = 0.0;
		std::array<ChannelStats, kChannelCount> channels{};
	};

	struct FitResult
	{
		bool valid = false;
		double slope = 0.0;
		double intercept = 0.0;
		double r_squared = 0.0;
		double rmse = 0.0;
		double max_abs_residual = 0.0;
		double input_span = 0.0;
	};

	struct SensorResult
	{
		std::array<PointMeasurement, kPointCount> points{};
		std::array<bool, kPointCount> captured{};
		FitResult equivalent_fit;
		FitResult direct_fit;
		int dominant_channel = -1;
		bool mapping_warning = false;
		bool mapping_confirmed = false;
		bool complete = false;
		bool recommended = false;
		bool input_span_adequate = false;
		double max_point_stddev_counts = 0.0;
		double span_to_noise_ratio = 0.0;
	};

	std::string build_report(
		const std::wstring& directory,
		const std::array<OriginalCalibration, kSensorCount>& originals,
		const std::array<SensorResult, kSensorCount>& results,
		bool all_complete,
		const std::string& status_note,
		const std::string& connection_info);

	enum class PromptChoice
	{
		Accept,
		Retry,
		Abort
	};

	std::wstring join_path(const std::wstring& left, const std::wstring& right)
	{
		if (left.empty()) return right;
		std::wstring result = left;
		if (result.back() != L'\\' && result.back() != L'/') result.push_back(L'\\');
		result += right;
		return result;
	}

	bool path_is_directory(const std::wstring& path)
	{
		const DWORD attributes = GetFileAttributesW(path.c_str());
		return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
	}

	bool path_is_file(const std::wstring& path)
	{
		const DWORD attributes = GetFileAttributesW(path.c_str());
		return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) == 0;
	}

	std::wstring absolute_path(const std::wstring& path)
	{
		const DWORD required = GetFullPathNameW(path.c_str(), 0, nullptr, nullptr);
		if (required == 0) return path;
		std::vector<wchar_t> buffer(required + 1, L'\0');
		if (GetFullPathNameW(path.c_str(), static_cast<DWORD>(buffer.size()), buffer.data(), nullptr) == 0)
		{
			return path;
		}
		return std::wstring(buffer.data());
	}

	std::wstring parent_path(std::wstring path)
	{
		while (path.size() > 3 && (path.back() == L'\\' || path.back() == L'/')) path.pop_back();
		const std::wstring::size_type separator = path.find_last_of(L"\\/");
		if (separator == std::wstring::npos) return std::wstring();
		if (separator == 2 && path.size() >= 3 && path[1] == L':') return path.substr(0, 3);
		return path.substr(0, separator);
	}

	std::wstring executable_directory()
	{
		std::vector<wchar_t> buffer(32768, L'\0');
		const DWORD count = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
		if (count == 0 || count >= buffer.size()) return std::wstring();
		return parent_path(std::wstring(buffer.data(), count));
	}

	std::wstring current_directory()
	{
		const DWORD required = GetCurrentDirectoryW(0, nullptr);
		if (required == 0) return std::wstring();
		std::vector<wchar_t> buffer(required + 1, L'\0');
		if (GetCurrentDirectoryW(static_cast<DWORD>(buffer.size()), buffer.data()) == 0)
		{
			return std::wstring();
		}
		return std::wstring(buffer.data());
	}

	std::string wide_to_utf8(const std::wstring& value)
	{
		if (value.empty()) return std::string();
		const int required = WideCharToMultiByte(
			CP_UTF8,
			WC_ERR_INVALID_CHARS,
			value.data(),
			static_cast<int>(value.size()),
			nullptr,
			0,
			nullptr,
			nullptr);
		if (required <= 0) return std::string();
		std::string result(static_cast<std::size_t>(required), '\0');
		if (WideCharToMultiByte(
			CP_UTF8,
			WC_ERR_INVALID_CHARS,
			value.data(),
			static_cast<int>(value.size()),
			&result[0],
			required,
			nullptr,
			nullptr) <= 0)
		{
			return std::string();
		}
		return result;
	}

	bool directory_has_original_files(const std::wstring& directory)
	{
		if (!path_is_directory(directory)) return false;
		for (const wchar_t* file_name : kOriginalFileNames)
		{
			if (!path_is_file(join_path(directory, file_name))) return false;
		}
		return true;
	}

	bool search_from_ancestor(const std::wstring& start, std::wstring& found)
	{
		std::wstring base = absolute_path(start);
		for (int depth = 0; depth < 10 && !base.empty(); ++depth)
		{
			if (directory_has_original_files(base))
			{
				found = base;
				return true;
			}
			const std::wstring candidate = join_path(base, L"力感测试程序加数据\\传感器标定");
			if (directory_has_original_files(candidate))
			{
				found = candidate;
				return true;
			}
			const std::wstring parent = parent_path(base);
			if (parent.empty() || parent == base) break;
			base = parent;
		}
		return false;
	}

	bool locate_calibration_directory(const Options& options, std::wstring& found, std::string& error)
	{
		if (!options.calibration_directory.empty())
		{
			const std::wstring requested = absolute_path(options.calibration_directory);
			if (!directory_has_original_files(requested))
			{
				error = "--calibration-dir 指定目录不存在，或缺少四份原标定 CSV：" + wide_to_utf8(requested);
				return false;
			}
			found = requested;
			return true;
		}

		const std::wstring cwd = current_directory();
		if (!cwd.empty() && search_from_ancestor(cwd, found)) return true;
		const std::wstring exe_dir = executable_directory();
		if (!exe_dir.empty() && search_from_ancestor(exe_dir, found)) return true;

		error = "无法从当前目录或 ADS.exe 上级目录定位 力感测试程序加数据\\传感器标定。"
			"请使用 --calibration-dir 显式指定。";
		return false;
	}

	std::vector<std::string> split_csv_line(const std::string& line)
	{
		std::vector<std::string> fields;
		std::string current;
		bool quoted = false;
		for (std::size_t i = 0; i < line.size(); ++i)
		{
			const char ch = line[i];
			if (ch == '"')
			{
				if (quoted && i + 1 < line.size() && line[i + 1] == '"')
				{
					current.push_back('"');
					++i;
				}
				else
				{
					quoted = !quoted;
				}
			}
			else if (ch == ',' && !quoted)
			{
				fields.push_back(current);
				current.clear();
			}
			else
			{
				current.push_back(ch);
			}
		}
		fields.push_back(current);
		return fields;
	}

	int find_csv_column(const std::vector<std::string>& fields, const char* column_name)
	{
		for (std::size_t index = 0; index < fields.size(); ++index)
		{
			if (fields[index] == column_name) return static_cast<int>(index);
		}
		return -1;
	}

	bool parse_double(const std::string& text, double& value)
	{
		const char* begin = text.c_str();
		char* end = nullptr;
		errno = 0;
		const double parsed = std::strtod(begin, &end);
		if (begin == end || errno == ERANGE || !std::isfinite(parsed)) return false;
		while (*end == ' ' || *end == '\t') ++end;
		if (*end != '\0') return false;
		value = parsed;
		return true;
	}

	bool parse_integer(const std::string& text, int& value)
	{
		const char* begin = text.c_str();
		char* end = nullptr;
		errno = 0;
		const long parsed = std::strtol(begin, &end, 10);
		if (begin == end || errno == ERANGE) return false;
		while (*end == ' ' || *end == '\t') ++end;
		if (*end != '\0' ||
			parsed < (std::numeric_limits<int>::min)() ||
			parsed > (std::numeric_limits<int>::max)())
		{
			return false;
		}
		value = static_cast<int>(parsed);
		return true;
	}

	int weight_index(int weight_g)
	{
		for (int i = 0; i < kPointCount; ++i)
		{
			if (kWeightsG[i] == weight_g) return i;
		}
		return -1;
	}

	bool read_binary_file(const std::wstring& path, std::string& contents, std::string& error)
	{
		FILE* file = nullptr;
		if (_wfopen_s(&file, path.c_str(), L"rb") != 0 || file == nullptr)
		{
			error = "无法打开原标定文件：" + wide_to_utf8(path);
			return false;
		}

		if (std::fseek(file, 0, SEEK_END) != 0)
		{
			std::fclose(file);
			error = "无法读取原标定文件长度：" + wide_to_utf8(path);
			return false;
		}
		const long length = std::ftell(file);
		if (length <= 0 || length > 1024 * 1024)
		{
			std::fclose(file);
			error = "原标定文件大小异常：" + wide_to_utf8(path);
			return false;
		}
		std::rewind(file);
		contents.assign(static_cast<std::size_t>(length), '\0');
		const std::size_t read_count = std::fread(&contents[0], 1, contents.size(), file);
		std::fclose(file);
		if (read_count != contents.size())
		{
			error = "原标定文件读取不完整：" + wide_to_utf8(path);
			return false;
		}
		return true;
	}

	bool parse_original_calibration(
		const std::wstring& path,
		OriginalCalibration& calibration,
		std::string& error)
	{
		std::string contents;
		if (!read_binary_file(path, contents, error)) return false;
		if (contents.size() < 3 ||
			static_cast<unsigned char>(contents[0]) != 0xEF ||
			static_cast<unsigned char>(contents[1]) != 0xBB ||
			static_cast<unsigned char>(contents[2]) != 0xBF)
		{
			error = "原标定 CSV 必须是 UTF-8 BOM 编码：" + wide_to_utf8(path);
			return false;
		}
		contents.erase(0, 3);

		bool slope_found = false;
		bool intercept_found = false;
		bool table_header_found = false;
		int weight_column = -1;
		int mean_column = -1;
		int stddev_column = -1;
		std::array<bool, kPointCount> point_found{};
		std::istringstream input(contents);
		std::string line;
		while (std::getline(input, line))
		{
			if (!line.empty() && line.back() == '\r') line.pop_back();
			if (line.empty()) continue;
			const std::vector<std::string> fields = split_csv_line(line);
			if (fields.size() >= 2 && fields[0] == u8"斜率 (k)")
			{
				if (slope_found || !parse_double(fields[1], calibration.slope))
				{
					error = "原标定 CSV 的斜率重复或无效：" + wide_to_utf8(path);
					return false;
				}
				slope_found = true;
				continue;
			}
			if (fields.size() >= 2 && fields[0] == u8"截距 (b)")
			{
				if (intercept_found || !parse_double(fields[1], calibration.intercept))
				{
					error = "原标定 CSV 的截距重复或无效：" + wide_to_utf8(path);
					return false;
				}
				intercept_found = true;
				continue;
			}

			const int candidate_weight_column = find_csv_column(fields, u8"重量(g)");
			if (candidate_weight_column >= 0)
			{
				if (table_header_found)
				{
					error = "原标定 CSV 包含重复数据表头：" + wide_to_utf8(path);
					return false;
				}
				weight_column = candidate_weight_column;
				mean_column = find_csv_column(fields, u8"传感器均值");
				stddev_column = find_csv_column(fields, u8"标准差");
				if (mean_column < 0 || stddev_column < 0)
				{
					error = "原标定 CSV 表头缺少传感器均值或标准差列：" + wide_to_utf8(path);
					return false;
				}
				table_header_found = true;
				continue;
			}
			if (!table_header_found) continue;
			const int largest_column = (std::max)(weight_column, (std::max)(mean_column, stddev_column));
			if (largest_column < 0 || static_cast<std::size_t>(largest_column) >= fields.size()) continue;
			int weight_g = 0;
			if (!parse_integer(fields[weight_column], weight_g)) continue;
			const int index = weight_index(weight_g);
			if (index < 0) continue;
			if (point_found[index])
			{
				error = "原标定 CSV 包含重复的 " + std::to_string(weight_g) + "g 数据点：" +
					wide_to_utf8(path);
				return false;
			}
			double mean = 0.0;
			double stddev = 0.0;
			if (!parse_double(fields[mean_column], mean) ||
				!parse_double(fields[stddev_column], stddev) || stddev < 0.0)
			{
				error = "原标定 CSV 的均值或标准差无效（标准差不得为负）：" + wide_to_utf8(path);
				return false;
			}
			calibration.reference_means[index] = mean;
			calibration.reference_stddevs[index] = stddev;
			point_found[index] = true;
		}

		if (!slope_found || !intercept_found || std::fabs(calibration.slope) <= 1e-15)
		{
			error = "原标定 CSV 缺少有效斜率或截距，或斜率为零：" + wide_to_utf8(path);
			return false;
		}
		if (!table_header_found)
		{
			error = "原标定 CSV 缺少重量/传感器均值/标准差数据表头：" + wide_to_utf8(path);
			return false;
		}
		for (int i = 0; i < kPointCount; ++i)
		{
			if (!point_found[i])
			{
				error = "原标定 CSV 缺少 " + std::to_string(kWeightsG[i]) + "g 数据点：" + wide_to_utf8(path);
				return false;
			}
		}
		const auto reference_minmax = std::minmax_element(
			calibration.reference_means.begin(),
			calibration.reference_means.end());
		if (*reference_minmax.second - *reference_minmax.first <= 1e-9)
		{
			error = "原标定 CSV 的六个参考均值跨度退化：" + wide_to_utf8(path);
			return false;
		}
		calibration.path = path;
		return true;
	}

	bool load_original_calibrations(
		const std::wstring& directory,
		std::array<OriginalCalibration, kSensorCount>& calibrations,
		std::string& error)
	{
		for (int sensor = 0; sensor < kSensorCount; ++sensor)
		{
			const std::wstring path = join_path(directory, kOriginalFileNames[sensor]);
			if (!parse_original_calibration(path, calibrations[sensor], error)) return false;
		}
		return true;
	}

	ChannelStats calculate_channel_stats(const std::vector<short>& samples)
	{
		ChannelStats stats;
		stats.total_count = static_cast<int>(samples.size());
		if (samples.empty()) return stats;

		std::vector<short> sorted = samples;
		std::sort(sorted.begin(), sorted.end());
		const std::size_t middle = sorted.size() / 2;
		stats.median = sorted.size() % 2 == 0
			? (static_cast<double>(sorted[middle - 1]) + static_cast<double>(sorted[middle])) / 2.0
			: static_cast<double>(sorted[middle]);

		double sum = 0.0;
		for (const short sample : samples)
		{
			if (std::fabs(static_cast<double>(sample) - stats.median) <= kOutlierThresholdCounts)
			{
				sum += static_cast<double>(sample);
				++stats.valid_count;
			}
		}
		if (stats.valid_count == 0) return stats;
		stats.mean = sum / static_cast<double>(stats.valid_count);

		double squared_sum = 0.0;
		for (const short sample : samples)
		{
			if (std::fabs(static_cast<double>(sample) - stats.median) <= kOutlierThresholdCounts)
			{
				const double difference = static_cast<double>(sample) - stats.mean;
				squared_sum += difference * difference;
			}
		}
		stats.stddev = std::sqrt(squared_sum / static_cast<double>(stats.valid_count));
		return stats;
	}

	bool fit_line(const std::vector<double>& x, const std::vector<double>& y, FitResult& fit)
	{
		fit = FitResult{};
		if (x.size() != y.size() || x.size() < 2) return false;
		const auto minmax_x = std::minmax_element(x.begin(), x.end());
		fit.input_span = *minmax_x.second - *minmax_x.first;
		if (!std::isfinite(fit.input_span) || std::fabs(fit.input_span) <= 1e-9) return false;

		double mean_x = 0.0;
		double mean_y = 0.0;
		for (std::size_t i = 0; i < x.size(); ++i)
		{
			if (!std::isfinite(x[i]) || !std::isfinite(y[i])) return false;
			mean_x += x[i];
			mean_y += y[i];
		}
		mean_x /= static_cast<double>(x.size());
		mean_y /= static_cast<double>(y.size());

		double sxx = 0.0;
		double sxy = 0.0;
		double syy = 0.0;
		for (std::size_t i = 0; i < x.size(); ++i)
		{
			const double dx = x[i] - mean_x;
			const double dy = y[i] - mean_y;
			sxx += dx * dx;
			sxy += dx * dy;
			syy += dy * dy;
		}
		if (sxx <= 1e-12 || syy <= 1e-20) return false;

		fit.slope = sxy / sxx;
		fit.intercept = mean_y - fit.slope * mean_x;
		double residual_sum_squares = 0.0;
		for (std::size_t i = 0; i < x.size(); ++i)
		{
			const double residual = fit.slope * x[i] + fit.intercept - y[i];
			residual_sum_squares += residual * residual;
			fit.max_abs_residual = (std::max)(fit.max_abs_residual, std::fabs(residual));
		}
		fit.r_squared = 1.0 - residual_sum_squares / syy;
		fit.rmse = std::sqrt(residual_sum_squares / static_cast<double>(x.size()));
		fit.valid = std::isfinite(fit.slope) && std::isfinite(fit.intercept) &&
			std::isfinite(fit.r_squared) && std::isfinite(fit.rmse);
		return fit.valid;
	}

	void print_usage()
	{
		std::cout
			<< "用法：\n"
			<< "  ADS.exe --sensor-calibration [--calibration-dir <目录>]\n"
			<< "  ADS.exe --sensor-calibration [--calibration-dir <目录>] --validate-only\n"
			<< "  ADS.exe --sensor-calibration --self-test\n";
	}

	bool parse_options(Options& options, std::string& error)
	{
		int argument_count = 0;
		LPWSTR* arguments = CommandLineToArgvW(GetCommandLineW(), &argument_count);
		if (arguments == nullptr)
		{
			error = "无法解析 Unicode 命令行参数。";
			return false;
		}

		bool success = true;
		for (int i = 2; i < argument_count; ++i)
		{
			const std::wstring argument = arguments[i];
			if (argument == L"--calibration-dir")
			{
				if (i + 1 >= argument_count)
				{
					error = "--calibration-dir 后必须提供目录。";
					success = false;
					break;
				}
				options.calibration_directory = arguments[++i];
			}
			else if (argument == L"--validate-only")
			{
				options.validate_only = true;
			}
			else if (argument == L"--self-test")
			{
				options.self_test = true;
			}
			else if (argument == L"--help" || argument == L"-h")
			{
				options.show_help = true;
			}
			else
			{
				error = "未知参数：" + wide_to_utf8(argument);
				success = false;
				break;
			}
		}
		LocalFree(arguments);
		return success;
	}

	bool nearly_equal(double left, double right, double tolerance)
	{
		return std::fabs(left - right) <= tolerance;
	}

	bool input_span_is_adequate(
		double input_span,
		double max_point_stddev,
		double& span_to_noise_ratio)
	{
		if (!std::isfinite(input_span) || !std::isfinite(max_point_stddev) ||
			input_span < 0.0 || max_point_stddev < 0.0)
		{
			span_to_noise_ratio = 0.0;
			return false;
		}
		span_to_noise_ratio = max_point_stddev > 1e-12
			? input_span / max_point_stddev
			: (std::numeric_limits<double>::infinity)();
		return input_span >= kMinimumInputSpanCounts &&
			span_to_noise_ratio >= kMinimumSpanToNoiseRatio;
	}

	bool run_self_test()
	{
		bool success = true;
		auto check = [&](bool condition, const char* name)
		{
			std::cout << (condition ? "[PASS] " : "[FAIL] ") << name << std::endl;
			if (!condition) success = false;
		};

		const std::vector<double> x = { 100.0, 120.0, 170.0, 250.0, 400.0, 900.0 };
		std::vector<double> positive_y;
		std::vector<double> negative_y;
		for (const double value : x)
		{
			positive_y.push_back(2.5 * value - 17.0);
			negative_y.push_back(-1.75 * value + 420.0);
		}

		FitResult positive_fit;
		check(fit_line(x, positive_y, positive_fit), "正斜率拟合可执行");
		check(nearly_equal(positive_fit.slope, 2.5, 1e-12), "正斜率系数");
		check(nearly_equal(positive_fit.intercept, -17.0, 1e-9), "正斜率截距");
		check(nearly_equal(positive_fit.r_squared, 1.0, 1e-12), "正斜率 R2");

		FitResult negative_fit;
		check(fit_line(x, negative_y, negative_fit), "负斜率拟合可执行");
		check(nearly_equal(negative_fit.slope, -1.75, 1e-12), "负斜率系数");
		check(nearly_equal(negative_fit.intercept, 420.0, 1e-9), "负斜率截距");

		FitResult singular_fit;
		check(!fit_line(
			std::vector<double>{ 10.0, 10.0, 10.0 },
			std::vector<double>{ 0.0, 1.0, 2.0 },
			singular_fit), "退化输入被拒绝");

		const ChannelStats stats = calculate_channel_stats(std::vector<short>{ 9, 10, 11, 1000 });
		check(stats.valid_count == 3, "异常值剔除数量");
		check(nearly_equal(stats.mean, 10.0, 1e-12), "异常值剔除均值");
		check(calculate_channel_stats(std::vector<short>(79, 10)).valid_count <
			kMinimumValidSamples, "有效样本不足可识别");
		check(find_csv_column(
			std::vector<std::string>{ u8"标准差", u8"重量(g)", u8"传感器均值" },
			u8"传感器均值") == 2, "CSV表头按名称定位");

		double span_to_noise = 0.0;
		check(input_span_is_adequate(100.0, 10.0, span_to_noise) &&
			nearly_equal(span_to_noise, 10.0, 1e-12), "计数跨度质量通过");
		check(!input_span_is_adequate(0.5, 0.01, span_to_noise), "小于1 count的跨度被拒绝");
		check(!input_span_is_adequate(100.0, 25.1, span_to_noise), "低信噪比跨度被拒绝");

		FitResult poor_fit;
		check(fit_line(
			std::vector<double>{ 0.0, 1.0, 2.0, 3.0, 4.0, 5.0 },
			std::vector<double>{ 0.0, 1.0, 2.0, 10.0, 4.0, 5.0 },
			poor_fit) && poor_fit.r_squared < kMinimumR2, "低R2结果可识别");

		const double old_k = 0.0012;
		const double old_b = -0.31;
		const double composed_k = old_k * positive_fit.slope;
		const double composed_b = old_k * positive_fit.intercept + old_b;
		const double sample = 333.0;
		check(nearly_equal(
			composed_k * sample + composed_b,
			old_k * (positive_fit.slope * sample + positive_fit.intercept) + old_b,
			1e-12), "兼容原标定曲线展开");

		ForceCalibrationConfig direct_cfg;
		direct_cfg.gravity_comp_enabled = false;
		direct_cfg.deadband_f_n = 0.0;
		direct_cfg.deadband_t_nm = 0.0;
		direct_cfg.f_max_n = 100.0;
		direct_cfg.t_max_nm = 100.0;

		ForceCalibrationState unset_zero;
		const CleanForce unset_clean = calculate_clean_force(1.0, 1.0, direct_cfg, unset_zero);
		check(nearly_equal(unset_clean.force_n, 0.0, 1e-15) &&
			nearly_equal(unset_clean.handle_torque_nm, 0.0, 1e-15),
			"运行时未调零不输出物理力");

		ForceCalibrationState direct_zero;
		direct_zero.f_zero = -0.82962; // 传感器2本次0 g均值。
		direct_zero.ft_zero = -0.02503; // 传感器1本次0 g均值。
		direct_zero.fn_2_zero = -1.39613; // 传感器3本次0 g均值。
		direct_zero.ft_2_zero = 0.20103; // 传感器4本次0 g均值。
		direct_zero.zeroed = true;
		const CleanForce zero_clean = calculate_clean_force(
			direct_zero.f_zero, direct_zero.ft_zero, direct_cfg, direct_zero);
		check(nearly_equal(zero_clean.force_n, 0.0, 1e-15) &&
			nearly_equal(zero_clean.handle_torque_nm, 0.0, 1e-15),
			"导管F_direct动态零点严格归零");
		const CalibratedForce guidewire_zero_feedback = calibrate_guidewire_force(
			direct_zero.fn_2_zero, direct_zero.ft_2_zero, 0.0, direct_cfg, direct_zero);
		check(nearly_equal(guidewire_zero_feedback.f_feedback_n, 0.0, 1e-15) &&
			nearly_equal(guidewire_zero_feedback.t_feedback_nm, 0.0, 1e-15),
			"导丝F_direct动态零点严格归零");

		const double fn_1_50g_v = -0.03059;
		const double ft_1_50g_v = 0.67303;
		const double expected_axial_n = 0.490953762385746;
		const double expected_tangential_n = 0.491213129859387;
		const CleanForce loaded_clean = calculate_clean_force(
			fn_1_50g_v, ft_1_50g_v, direct_cfg, direct_zero);
		check(nearly_equal(loaded_clean.force_n, expected_axial_n, 1e-12),
			"传感器2轴向力F_direct映射");
		check(nearly_equal(
			loaded_clean.handle_torque_nm,
			expected_tangential_n * direct_cfg.handle_radius_mm * 0.001,
			1e-12), "传感器1切向力到手柄扭矩映射");

		const CalibratedForce loaded_feedback = calibrate_force(
			fn_1_50g_v, ft_1_50g_v, 0.0, direct_cfg, direct_zero);
		check(nearly_equal(loaded_feedback.f_feedback_n, loaded_clean.force_n, 1e-12) &&
			nearly_equal(loaded_feedback.t_feedback_nm, loaded_clean.handle_torque_nm, 1e-12),
			"纯净力与反馈力共用F_direct");

		const double fn_2_50g_v = -0.38615;
		const double ft_2_50g_v = 1.53886;
		const double expected_guidewire_axial_n = 1.14576622884054;
		const double expected_guidewire_tangential_n = 0.17843125546337;
		const CalibratedForce guidewire_loaded_feedback = calibrate_guidewire_force(
			fn_2_50g_v, ft_2_50g_v, 0.0, direct_cfg, direct_zero);
		check(nearly_equal(
			guidewire_loaded_feedback.f_feedback_n, expected_guidewire_axial_n, 1e-12),
			"传感器3轴向力到导丝手柄F_direct映射");
		check(nearly_equal(
			guidewire_loaded_feedback.t_feedback_nm,
			expected_guidewire_tangential_n * direct_cfg.handle_radius_mm * 0.001,
			1e-12), "传感器4切向力到导丝手柄扭矩映射");

		ControlConfig routing_cfg;
		Handle unopened_catheter_handle(582);
		Handle unopened_guidewire_handle(587);
		ForceSampleFrame routing_sample;
		routing_sample.fn_1_value_v = fn_1_50g_v;
		routing_sample.ft_1_value_v = ft_1_50g_v;
		routing_sample.fn_2_value_v = fn_2_50g_v;
		routing_sample.ft_2_value_v = ft_2_50g_v;
		routing_sample.valid = true;
		ForceFeedbackState routing_ff;
		routing_ff.enabled = true;
		process_force_feedback(
			routing_ff, routing_sample,
			unopened_catheter_handle, unopened_guidewire_handle,
			GuidewireMode::None,
			true, false, false, false, false, false, false, 0, 0,
			routing_cfg, direct_cfg, direct_zero);
		check(nearly_equal(routing_ff.force_582_f, expected_axial_n, 1e-12) &&
			nearly_equal(routing_ff.force_587_f, 0.0, 1e-15),
			"导管模式仅路由传感器2/1到582");

		routing_ff.reset();
		process_force_feedback(
			routing_ff, routing_sample,
			unopened_catheter_handle, unopened_guidewire_handle,
			GuidewireMode::Independent,
			true, false, false, false, false, false, false, 0, 0,
			routing_cfg, direct_cfg, direct_zero);
		check(nearly_equal(routing_ff.force_582_f, 0.0, 1e-15) &&
			nearly_equal(routing_ff.force_587_f, expected_guidewire_axial_n, 1e-12),
			"独立导丝模式仅路由传感器3/4到587");

		routing_ff.reset();
		process_force_feedback(
			routing_ff, routing_sample,
			unopened_catheter_handle, unopened_guidewire_handle,
			GuidewireMode::Cooperative,
			true, false, false, false, false, false, false, 0, 0,
			routing_cfg, direct_cfg, direct_zero);
		check(nearly_equal(routing_ff.force_582_f, expected_axial_n, 1e-12) &&
			nearly_equal(routing_ff.force_587_f, expected_guidewire_axial_n, 1e-12),
			"协同模式同时路由582与587");

		routing_ff.reset();
		process_force_feedback(
			routing_ff, routing_sample,
			unopened_catheter_handle, unopened_catheter_handle,
			GuidewireMode::Independent,
			true, false, false, false, false, false, false, 0, 0,
			routing_cfg, direct_cfg, direct_zero);
		check(nearly_equal(routing_ff.force_582_f, 0.0, 1e-15) &&
			nearly_equal(routing_ff.force_587_f, expected_guidewire_axial_n, 1e-12),
			"单手柄别名保持当前导丝语义输出");

		routing_ff.reset();
		process_force_feedback(
			routing_ff, routing_sample,
			unopened_catheter_handle, unopened_guidewire_handle,
			GuidewireMode::Cooperative,
			true, false, false, true, false, false, false, 0, 0,
			routing_cfg, direct_cfg, direct_zero);
		const double frozen_582_f = routing_ff.force_582_f;
		const double frozen_587_f = routing_ff.force_587_f;
		ForceSampleFrame changed_routing_sample = routing_sample;
		changed_routing_sample.fn_1_value_v += 0.5;
		changed_routing_sample.fn_2_value_v += 0.5;
		process_force_feedback(
			routing_ff, changed_routing_sample,
			unopened_catheter_handle, unopened_guidewire_handle,
			GuidewireMode::Cooperative,
			true, false, false, true, false, false, false, 0, 0,
			routing_cfg, direct_cfg, direct_zero);
		check(nearly_equal(routing_ff.force_582_f, frozen_582_f, 1e-12) &&
			nearly_equal(routing_ff.force_587_f, frozen_587_f, 1e-12),
			"协同快退同时锁存582与587输出");
		process_force_feedback(
			routing_ff, changed_routing_sample,
			unopened_catheter_handle, unopened_guidewire_handle,
			GuidewireMode::Cooperative,
			true, false, false, false, false, false, false, 0, 0,
			routing_cfg, direct_cfg, direct_zero);
		check(!nearly_equal(routing_ff.force_582_f, frozen_582_f, 1e-12) &&
			!nearly_equal(routing_ff.force_587_f, frozen_587_f, 1e-12),
			"快退结束后582与587恢复实时映射");

		ForceCalibrationConfig locked_gravity_cfg = direct_cfg;
		locked_gravity_cfg.gravity_comp_enabled = true;
		locked_gravity_cfg.gravity_comp_validated = false;
		const CalibratedForce locked_gravity_feedback = calibrate_force(
			fn_1_50g_v, ft_1_50g_v, 180.0, locked_gravity_cfg, direct_zero);
		check(nearly_equal(locked_gravity_feedback.f_feedback_n, loaded_feedback.f_feedback_n, 1e-12) &&
			nearly_equal(locked_gravity_feedback.t_feedback_nm, loaded_feedback.t_feedback_nm, 1e-12),
			"未复标重力模型被有效门槛锁定");

		const double fn_absolute_loaded =
			force_direct_calibration::fn_1_slope_n_per_count * (fn_1_50g_v * 1000.0) +
			force_direct_calibration::fn_1_intercept_n;
		const double fn_absolute_zero =
			force_direct_calibration::fn_1_slope_n_per_count * (direct_zero.f_zero * 1000.0) +
			force_direct_calibration::fn_1_intercept_n;
		const double ft_absolute_loaded =
			force_direct_calibration::ft_1_slope_n_per_count * (ft_1_50g_v * 1000.0) +
			force_direct_calibration::ft_1_intercept_n;
		const double ft_absolute_zero =
			force_direct_calibration::ft_1_slope_n_per_count * (direct_zero.ft_zero * 1000.0) +
			force_direct_calibration::ft_1_intercept_n;
		check(nearly_equal(fn_absolute_loaded, 0.489688315853313, 1e-12) &&
			nearly_equal(fn_absolute_zero, -0.0012654465324331, 1e-12) &&
			nearly_equal(ft_absolute_loaded, 0.490683826766822, 1e-12) &&
			nearly_equal(ft_absolute_zero, -0.000529303092565662, 1e-12),
			"报告F_direct绝对值锚点");
		check(nearly_equal(fn_absolute_loaded - fn_absolute_zero, loaded_clean.force_n, 1e-12) &&
			nearly_equal(ft_absolute_loaded - ft_absolute_zero, expected_tangential_n, 1e-12),
			"导管F_direct截距在动态调零中抵消");
		const double fn_2_absolute_loaded =
			force_direct_calibration::fn_2_slope_n_per_count * (fn_2_50g_v * 1000.0) +
			force_direct_calibration::fn_2_intercept_n;
		const double fn_2_absolute_zero =
			force_direct_calibration::fn_2_slope_n_per_count * (direct_zero.fn_2_zero * 1000.0) +
			force_direct_calibration::fn_2_intercept_n;
		const double ft_2_absolute_loaded =
			force_direct_calibration::ft_2_slope_n_per_count * (ft_2_50g_v * 1000.0) +
			force_direct_calibration::ft_2_intercept_n;
		const double ft_2_absolute_zero =
			force_direct_calibration::ft_2_slope_n_per_count * (direct_zero.ft_2_zero * 1000.0) +
			force_direct_calibration::ft_2_intercept_n;
		check(nearly_equal(
			fn_2_absolute_loaded - fn_2_absolute_zero, expected_guidewire_axial_n, 1e-12) &&
			nearly_equal(
				ft_2_absolute_loaded - ft_2_absolute_zero, expected_guidewire_tangential_n, 1e-12),
			"导丝F_direct截距在动态调零中抵消");

		ForceCalibrationState rezero = direct_zero;
		rezero.f_zero = -0.70000;
		rezero.ft_zero = 0.12500;
		const CleanForce rezero_clean = calculate_clean_force(
			rezero.f_zero, rezero.ft_zero, direct_cfg, rezero);
		const CleanForce rezero_delta = calculate_clean_force(
			rezero.f_zero - 0.1, rezero.ft_zero + 0.1, direct_cfg, rezero);
		check(nearly_equal(rezero_clean.force_n, 0.0, 1e-15) &&
			nearly_equal(rezero_clean.handle_torque_nm, 0.0, 1e-15) &&
			nearly_equal(rezero_delta.force_n, -0.1 * direct_cfg.axial_direct_n_per_v, 1e-12) &&
			nearly_equal(
				rezero_delta.handle_torque_nm,
				0.1 * direct_cfg.tangential_direct_n_per_v * direct_cfg.handle_radius_mm * 0.001,
				1e-12), "重复调零只改变动态截距");

		const double int16_full_span_v = 65.535;
		check(nearly_equal(
			force_direct_calibration::zeroed_force_n(
				32.767, -32.768, direct_cfg.axial_direct_n_per_v),
			force_direct_calibration::fn_1_slope_n_per_count * 65535.0,
			1e-10) &&
			nearly_equal(32.767 - (-32.768), int16_full_span_v, 1e-12),
			"INT16极值先转double再做差");

		ForceCalibrationConfig limited_cfg;
		check(nearly_equal(limited_cfg.f_max_n, 5.0, 1e-12) &&
			nearly_equal(limited_cfg.t_max_nm, 0.020, 1e-12),
			"用户指定运行限幅为正负5N和正负20Nmm");
		ForceCalibrationState origin_zero;
		origin_zero.zeroed = true;
		const CalibratedForce limited_feedback = calibrate_force(
			10.0, 20.0, 0.0, limited_cfg, origin_zero);
		const CalibratedForce negative_limited_feedback = calibrate_force(
			-10.0, -20.0, 0.0, limited_cfg, origin_zero);
		check(nearly_equal(limited_feedback.f_feedback_n, limited_cfg.f_max_n, 1e-12) &&
			nearly_equal(limited_feedback.t_feedback_nm, limited_cfg.t_max_nm, 1e-12) &&
			nearly_equal(negative_limited_feedback.f_feedback_n, -limited_cfg.f_max_n, 1e-12) &&
			nearly_equal(negative_limited_feedback.t_feedback_nm, -limited_cfg.t_max_nm, 1e-12),
			"导管F_direct反馈按运行上限正负限幅");
		const CalibratedForce guidewire_limited_feedback = calibrate_guidewire_force(
			20.0, 20.0, 0.0, limited_cfg, origin_zero);
		const CalibratedForce guidewire_negative_limited_feedback = calibrate_guidewire_force(
			-20.0, -20.0, 0.0, limited_cfg, origin_zero);
		check(nearly_equal(
			guidewire_limited_feedback.f_feedback_n, limited_cfg.f_max_n, 1e-12) &&
			nearly_equal(
				guidewire_limited_feedback.t_feedback_nm, limited_cfg.t_max_nm, 1e-12) &&
			nearly_equal(
				guidewire_negative_limited_feedback.f_feedback_n, -limited_cfg.f_max_n, 1e-12) &&
			nearly_equal(
				guidewire_negative_limited_feedback.t_feedback_nm, -limited_cfg.t_max_nm, 1e-12),
			"导丝F_direct反馈按运行上限正负限幅");

		std::array<OriginalCalibration, kSensorCount> report_originals;
		std::array<SensorResult, kSensorCount> report_results;
		report_results[0].complete = true;
		report_results[0].recommended = true;
		report_results[0].dominant_channel = 0;
		const std::string incomplete_report = build_report(
			L".",
			report_originals,
			report_results,
			false,
			"自检中止",
			"未连接");
		check(incomplete_report.find("old_equivalent =") == std::string::npos &&
			incomplete_report.find("质量结论=可用") == std::string::npos &&
			incomplete_report.find("抑制全部拟合公式") != std::string::npos,
			"未完成报告抑制部署结论");

		std::cout << (success ? "标定数学自检通过。" : "标定数学自检失败。") << std::endl;
		return success;
	}

	void print_original_summary(
		const std::wstring& directory,
		const std::array<OriginalCalibration, kSensorCount>& calibrations)
	{
		std::cout << "原标定目录：" << wide_to_utf8(directory) << std::endl;
		for (int sensor = 0; sensor < kSensorCount; ++sensor)
		{
			std::cout << "传感器" << sensor + 1 << " -> " << kChannelSymbols[sensor]
				<< "，原曲线 F(N)=" << std::setprecision(12) << calibrations[sensor].slope
				<< "*raw" << (calibrations[sensor].intercept >= 0.0 ? "+" : "")
				<< calibrations[sensor].intercept << "，参考均值：";
			for (int point = 0; point < kPointCount; ++point)
			{
				if (point != 0) std::cout << ", ";
				std::cout << kWeightsG[point] << "g=" << calibrations[sensor].reference_means[point];
			}
			std::cout << std::endl;
		}
	}

	PromptChoice prompt_choice(const char* prompt, bool allow_accept)
	{
		while (true)
		{
			std::cout << prompt;
			std::string line;
			if (!std::getline(std::cin, line)) return PromptChoice::Abort;
			if (line.empty() && allow_accept) return PromptChoice::Accept;
			if (line == "r" || line == "R") return PromptChoice::Retry;
			if (line == "q" || line == "Q") return PromptChoice::Abort;
			std::cout << "请输入 Enter、R 或 Q。" << std::endl;
		}
	}

	bool confirm_ads_target(const std::string& connection_info)
	{
		while (true)
		{
			std::cout << "实际ADS连接：" << connection_info
				<< "。确认这是目标PLC后按 Enter，Q 中止：";
			std::string line;
			if (!std::getline(std::cin, line)) return false;
			if (line.empty()) return true;
			if (line == "q" || line == "Q") return false;
			std::cout << "请输入 Enter 或 Q。" << std::endl;
		}
	}

	bool wait_for_weight(int sensor_index, int weight_g)
	{
		std::ostringstream prompt;
		if (weight_g == 0)
		{
			prompt << "请移除传感器" << sensor_index + 1 << "上的全部砝码，保持其他传感器空载。"
				<< "按 Enter 开始采集，Q 中止：";
		}
		else
		{
			prompt << "请在传感器" << sensor_index + 1 << "上放置 " << weight_g
				<< "g 砝码，保持其他传感器空载。按 Enter 开始采集，Q 中止：";
		}
		while (true)
		{
			std::cout << prompt.str();
			std::string line;
			if (!std::getline(std::cin, line)) return false;
			if (line.empty()) return true;
			if (line == "q" || line == "Q") return false;
			std::cout << "请输入 Enter 或 Q。" << std::endl;
		}
	}

	bool read_all_channels(CADSComm& ads, std::array<short, kChannelCount>& values)
	{
		const char* symbols[kChannelCount] = {
			AdsSymbol::ft_1_value,
			AdsSymbol::fn_1_value,
			AdsSymbol::fn_2_value,
			AdsSymbol::ft_2_value
		};
		const unsigned long lengths[kChannelCount] = {
			static_cast<unsigned long>(sizeof(values[0])),
			static_cast<unsigned long>(sizeof(values[1])),
			static_cast<unsigned long>(sizeof(values[2])),
			static_cast<unsigned long>(sizeof(values[3]))
		};
		void* outputs[kChannelCount] = {
			&values[0], &values[1], &values[2], &values[3]
		};
		return ads.ADSReadSum(symbols, lengths, outputs, kChannelCount);
	}

	PointMeasurement collect_point(CADSComm& ads, int weight_g)
	{
		PointMeasurement measurement;
		measurement.weight_g = weight_g;
		std::array<std::vector<short>, kChannelCount> samples;
		for (auto& channel : samples) channel.reserve(kSampleCount);

		std::cout << "正在采集 " << weight_g << "g：5 秒 / 20 Hz ..." << std::endl;
		const auto start_time = std::chrono::steady_clock::now();
		auto scheduled_sample_time = start_time;
		for (int attempt = 0; attempt < kSampleCount; ++attempt)
		{
			std::array<short, kChannelCount> values{};
			if (read_all_channels(ads, values))
			{
				for (int channel = 0; channel < kChannelCount; ++channel)
				{
					samples[channel].push_back(values[channel]);
				}
			}
			else
			{
				++measurement.ads_failures;
			}
			if ((attempt + 1) % 20 == 0)
			{
				std::cout << "  进度 " << attempt + 1 << "/" << kSampleCount
					<< "，ADS失败 " << measurement.ads_failures << std::endl;
			}
			auto next_deadline = scheduled_sample_time + std::chrono::milliseconds(kSamplePeriodMs);
			const auto after_read = std::chrono::steady_clock::now();
			if (after_read > next_deadline)
			{
				// 已错过的周期不追赶，避免连续突发读取破坏采样独立性。
				next_deadline = after_read;
				++measurement.schedule_overruns;
			}
			std::this_thread::sleep_until(next_deadline);
			scheduled_sample_time = next_deadline;
		}
		measurement.elapsed_seconds = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - start_time).count();
		if (measurement.elapsed_seconds > 0.0)
		{
			measurement.attempted_rate_hz =
				static_cast<double>(kSampleCount) / measurement.elapsed_seconds;
		}

		for (int channel = 0; channel < kChannelCount; ++channel)
		{
			measurement.channels[channel] = calculate_channel_stats(samples[channel]);
		}
		return measurement;
	}

	void print_measurement(const PointMeasurement& measurement, int target_channel)
	{
		std::cout << "采集结果（中位数 ±300 counts 后）：" << std::endl;
		for (int channel = 0; channel < kChannelCount; ++channel)
		{
			const ChannelStats& stats = measurement.channels[channel];
			std::cout << (channel == target_channel ? "  * " : "    ")
				<< kChannelSymbols[channel]
				<< " mean=" << std::fixed << std::setprecision(3) << stats.mean
				<< ", std=" << stats.stddev
				<< ", valid=" << stats.valid_count << "/" << stats.total_count
				<< std::endl;
		}
		std::cout.unsetf(std::ios::floatfield);
		if (measurement.ads_failures > 0)
		{
			std::cout << "  ADS 读取失败次数：" << measurement.ads_failures << std::endl;
		}
		std::cout << "  实际采集耗时=" << std::fixed << std::setprecision(3)
			<< measurement.elapsed_seconds << " s，尝试频率="
			<< measurement.attempted_rate_hz << " Hz，调度超期="
			<< measurement.schedule_overruns << " 次" << std::endl;
		std::cout.unsetf(std::ios::floatfield);
	}

	PromptChoice capture_point(
		CADSComm& ads,
		int sensor_index,
		int point_index,
		PointMeasurement& accepted)
	{
		while (true)
		{
			const int weight_g = kWeightsG[point_index];
			if (!wait_for_weight(sensor_index, weight_g)) return PromptChoice::Abort;
			PointMeasurement measurement = collect_point(ads, weight_g);
			print_measurement(measurement, sensor_index);

			if (measurement.channels[sensor_index].valid_count < kMinimumValidSamples)
			{
				std::cout << "目标通道有效样本少于 " << kMinimumValidSamples
					<< "，本点不能接受。" << std::endl;
				const PromptChoice choice = prompt_choice("按 R 重测，Q 中止：", false);
				if (choice == PromptChoice::Abort) return choice;
				continue;
			}
			if (measurement.attempted_rate_hz < kMinimumAttemptedRateHz)
			{
				std::cout << "实际尝试频率低于 " << kMinimumAttemptedRateHz
					<< " Hz，本点不能接受；请检查ADS超时或系统负载。" << std::endl;
				const PromptChoice choice = prompt_choice("按 R 重测，Q 中止：", false);
				if (choice == PromptChoice::Abort) return choice;
				continue;
			}

			const PromptChoice choice = prompt_choice("按 Enter 接受，R 重测，Q 中止：", true);
			if (choice == PromptChoice::Accept)
			{
				accepted = measurement;
				return choice;
			}
			if (choice == PromptChoice::Abort) return choice;
		}
	}

	int find_dominant_channel(const SensorResult& result)
	{
		if (!result.captured.front() || !result.captured.back()) return -1;
		int dominant = 0;
		double largest_response = -1.0;
		for (int channel = 0; channel < kChannelCount; ++channel)
		{
			const double response = std::fabs(
				result.points.back().channels[channel].mean -
				result.points.front().channels[channel].mean);
			if (response > largest_response)
			{
				largest_response = response;
				dominant = channel;
			}
		}
		return dominant;
	}

	bool calculate_sensor_fits(
		int sensor_index,
		const OriginalCalibration& original,
		SensorResult& result)
	{
		std::vector<double> current_raw;
		std::vector<double> original_equivalent;
		std::vector<double> physical_force;
		for (int point = 0; point < kPointCount; ++point)
		{
			if (!result.captured[point]) return false;
			current_raw.push_back(result.points[point].channels[sensor_index].mean);
			original_equivalent.push_back(original.reference_means[point]);
			physical_force.push_back(static_cast<double>(kWeightsG[point]) / 1000.0 * kGravityMps2);
			result.max_point_stddev_counts = (std::max)(
				result.max_point_stddev_counts,
				result.points[point].channels[sensor_index].stddev);
		}
		const bool fits_valid = fit_line(current_raw, original_equivalent, result.equivalent_fit) &&
			fit_line(current_raw, physical_force, result.direct_fit);
		if (!fits_valid) return false;
		result.input_span_adequate = input_span_is_adequate(
			result.equivalent_fit.input_span,
			result.max_point_stddev_counts,
			result.span_to_noise_ratio);
		return true;
	}

	std::wstring timestamp_text()
	{
		SYSTEMTIME time{};
		GetLocalTime(&time);
		wchar_t buffer[32] = {};
		swprintf_s(
			buffer,
			L"%04u%02u%02u_%02u%02u%02u",
			time.wYear,
			time.wMonth,
			time.wDay,
			time.wHour,
			time.wMinute,
			time.wSecond);
		return buffer;
	}

	std::wstring make_output_path(const std::wstring& directory, bool complete)
	{
		const std::wstring timestamp = timestamp_text();
		const std::wstring suffix = complete ? L"" : L"_未完成";
		for (int index = 0; index < 1000; ++index)
		{
			std::wostringstream name;
			name << L"传感器重新标定_" << timestamp << suffix;
			if (index > 0) name << L"_" << std::setw(3) << std::setfill(L'0') << index;
			name << L".txt";
			const std::wstring path = join_path(directory, name.str());
			if (!path_is_file(path)) return path;
		}
		return join_path(directory, L"传感器重新标定_输出冲突.txt");
	}

	void append_fit_summary(
		std::ostringstream& output,
		const char* title,
		const char* result_name,
		const FitResult& fit)
	{
		output << title << "：" << result_name << " = " << std::setprecision(12)
			<< fit.slope << " * current_raw "
			<< (fit.intercept >= 0.0 ? "+ " : "- ") << std::fabs(fit.intercept) << "\n";
		output << "  R2=" << fit.r_squared
			<< ", RMSE=" << fit.rmse
			<< ", 最大绝对残差=" << fit.max_abs_residual
			<< ", 当前计数跨度=" << fit.input_span << "\n";
	}

	std::string build_report(
		const std::wstring& directory,
		const std::array<OriginalCalibration, kSensorCount>& originals,
		const std::array<SensorResult, kSensorCount>& results,
		bool all_complete,
		const std::string& status_note,
		const std::string& connection_info)
	{
		bool all_recommended = all_complete;
		for (const SensorResult& result : results)
		{
			if (!result.complete || !result.recommended) all_recommended = false;
		}

		std::ostringstream output;
		output.imbue(std::locale::classic());
		output << "四路传感器重新标定结果\n";
		output << "状态=" << (all_complete ? "完成" : "未完成，禁止作为正式标定结果使用") << "\n";
		if (!all_complete)
		{
			output << "终止或失败原因="
				<< (status_note.empty() ? "实验未完成，原因未记录" : status_note) << "\n";
		}
		output << "总体质量结论=";
		if (!all_complete)
		{
			output << "禁止部署，需完成全部四路采集与拟合\n";
		}
		else if (!all_recommended)
		{
			output << "至少一路未通过全部质量门槛，禁止部署未通过通道，需复测\n";
		}
		else
		{
			output << "四路均通过全部质量门槛，可按各通道处理建议实施\n";
		}
		output << "生成时间=" << wide_to_utf8(timestamp_text()) << "\n";
		output << "原标定目录=" << wide_to_utf8(directory) << "\n";
		output << "ADS连接=" << (connection_info.empty() ? "未记录" : connection_info) << "\n";
		output << "采样配置=目标20 Hz、5 s、100次；按通道中位数±300 counts剔除异常值；"
			"目标通道至少80个有效样本；实际尝试频率低于" << kMinimumAttemptedRateHz
			<< " Hz强制重测\n";
		output << "跨度门槛=当前计数跨度至少" << kMinimumInputSpanCounts
			<< " count，且跨度/六点最大标准差至少" << kMinimumSpanToNoiseRatio << "\n";
		output << "标准重力加速度=" << std::setprecision(8) << kGravityMps2 << " m/s^2\n";
		output << "固定映射=传感器1->G.ft_1_value；传感器2->G.fn_1_value；传感器3->G.fn_2_value；传感器4->G.ft_2_value\n";
		output << "说明=本工具只读取PLC原始INT计数，不修改PLC变量或上位机运行标定参数。\n\n";

		for (int sensor = 0; sensor < kSensorCount; ++sensor)
		{
			const OriginalCalibration& original = originals[sensor];
			const SensorResult& result = results[sensor];
			output << "============================================================\n";
			output << "传感器" << sensor + 1 << " / " << kChannelSymbols[sensor] << "\n";
			output << "源文件=" << wide_to_utf8(original.path) << "\n";
			output << "原标定曲线=F(N) = " << std::setprecision(12) << original.slope
				<< " * old_raw " << (original.intercept >= 0.0 ? "+ " : "- ")
				<< std::fabs(original.intercept) << "\n";
			output << "采集状态=" << (result.complete ? "完整" : "未完成") << "\n";
			output << "重量(g) | ft_1 mean/std/median/valid/total | fn_1 mean/std/median/valid/total"
				" | fn_2 mean/std/median/valid/total | ft_2 mean/std/median/valid/total"
				" | ADS失败 | 实际秒/尝试Hz/调度超期 | 原参考均值/标准差\n";
			for (int point = 0; point < kPointCount; ++point)
			{
				if (!result.captured[point]) continue;
				output << result.points[point].weight_g;
				for (int channel = 0; channel < kChannelCount; ++channel)
				{
					const ChannelStats& stats = result.points[point].channels[channel];
					output << " | " << std::fixed << std::setprecision(3)
						<< stats.mean << "/" << stats.stddev << "/" << stats.median << "/"
						<< stats.valid_count << "/" << stats.total_count;
				}
				output << " | " << result.points[point].ads_failures
					<< " | " << std::fixed << std::setprecision(3)
					<< result.points[point].elapsed_seconds << "/"
					<< result.points[point].attempted_rate_hz << "/"
					<< result.points[point].schedule_overruns
					<< " | " << std::fixed << std::setprecision(2)
					<< original.reference_means[point] << "/"
					<< original.reference_stddevs[point] << "\n";
			}
			output.unsetf(std::ios::floatfield);

			if (!result.complete) continue;
			if (!all_complete)
			{
				output << "诊断说明=本通道虽已完成采集，但四路实验整体未完成；"
					"本文件抑制全部拟合公式、可用结论和部署建议。\n";
				continue;
			}
			output << "通道核对=固定映射" << kChannelSymbols[sensor]
				<< "；50g-0g主响应通道" << kChannelSymbols[result.dominant_channel]
				<< (result.mapping_warning ? "；不一致，已人工确认" : "；一致") << "\n";
			append_fit_summary(output, "等效原计数拟合", "old_equivalent", result.equivalent_fit);
			output << "输入跨度质量=" << (result.input_span_adequate ? "通过" : "退化")
				<< "；跨度=" << result.equivalent_fit.input_span
				<< " counts，六点最大标准差=" << result.max_point_stddev_counts
				<< " counts，跨度/噪声=" << result.span_to_noise_ratio << "\n";
			double minimum_current_raw = result.points[0].channels[sensor].mean;
			double maximum_current_raw = minimum_current_raw;
			for (int point = 1; point < kPointCount; ++point)
			{
				const double current_raw = result.points[point].channels[sensor].mean;
				minimum_current_raw = (std::min)(minimum_current_raw, current_raw);
				maximum_current_raw = (std::max)(maximum_current_raw, current_raw);
			}
			output << "验证范围=仅限0-50g（0-"
				<< static_cast<double>(kWeightsG.back()) / 1000.0 * kGravityMps2
				<< " N），本次current_raw范围=[" << minimum_current_raw << ", "
				<< maximum_current_raw << "]；范围外属于未验证外推，不得直接采用。\n";

			const double compatible_slope = original.slope * result.equivalent_fit.slope;
			const double compatible_intercept =
				original.slope * result.equivalent_fit.intercept + original.intercept;
			output << "兼容原标定力曲线 F_compatible(N)：F = " << std::setprecision(12)
				<< compatible_slope << " * current_raw "
				<< (compatible_intercept >= 0.0 ? "+ " : "- ")
				<< std::fabs(compatible_intercept) << "\n";
			append_fit_summary(output, "独立物理标定拟合(N)", "F_direct", result.direct_fit);
			output << "逐点误差：\n";
			output << "重量(g) | current_raw | old_target | old_pred | old_error"
				" | force_target(N) | compatible_pred(N) | compatible_error(N)"
				" | direct_pred(N) | direct_error(N)\n";
			for (int point = 0; point < kPointCount; ++point)
			{
				const double current = result.points[point].channels[sensor].mean;
				const double old_target = original.reference_means[point];
				const double old_prediction = result.equivalent_fit.slope * current + result.equivalent_fit.intercept;
				const double force_target = static_cast<double>(kWeightsG[point]) / 1000.0 * kGravityMps2;
				const double compatible_prediction = original.slope * old_prediction + original.intercept;
				const double force_prediction = result.direct_fit.slope * current + result.direct_fit.intercept;
				output << kWeightsG[point]
					<< " | " << std::setprecision(9) << current
					<< " | " << old_target
					<< " | " << old_prediction
					<< " | " << old_prediction - old_target
					<< " | " << force_target
					<< " | " << compatible_prediction
					<< " | " << compatible_prediction - force_target
					<< " | " << force_prediction
					<< " | " << force_prediction - force_target << "\n";
			}
			output << "质量结论=" << (result.recommended ? "可用" : "不建议使用，需复测") << "\n";
			if (result.recommended)
			{
				output << "处理建议=为保持原系统行为，首选先计算old_equivalent，再交给原标定链。"
					"PLC中使用LREAL；如必须转回INT，先四舍五入并限幅到[-32768,32767]。\n";
			}
			else
			{
				output << "处理建议=禁止在PLC或上位机部署上述曲线；保持现有参数，完成复测并通过全部质量门槛。"
					"复测合格后，等效计数补偿仍是保持原系统行为的首选方案。\n";
			}
		}
		return output.str();
	}

	bool write_utf8_bom_file(const std::wstring& path, const std::string& contents, std::string& error)
	{
		FILE* file = nullptr;
		if (_wfopen_s(&file, path.c_str(), L"wb") != 0 || file == nullptr)
		{
			error = "无法创建结果文件：" + wide_to_utf8(path);
			return false;
		}
		const unsigned char bom[3] = { 0xEF, 0xBB, 0xBF };
		const bool bom_ok = std::fwrite(bom, 1, sizeof(bom), file) == sizeof(bom);
		const bool body_ok = contents.empty() ||
			std::fwrite(contents.data(), 1, contents.size(), file) == contents.size();
		const bool close_ok = std::fclose(file) == 0;
		if (!bom_ok || !body_ok || !close_ok)
		{
			error = "结果文件写入失败：" + wide_to_utf8(path);
			return false;
		}
		return true;
	}

	bool save_report(
		const std::wstring& directory,
		const std::array<OriginalCalibration, kSensorCount>& originals,
		const std::array<SensorResult, kSensorCount>& results,
		bool complete,
		const std::string& status_note,
		const std::string& connection_info,
		std::wstring& output_path,
		std::string& error)
	{
		output_path = make_output_path(directory, complete);
		return write_utf8_bom_file(
			output_path,
			build_report(directory, originals, results, complete, status_note, connection_info),
			error);
	}

	bool connect_read_only(CADSComm& ads, std::string& connection_info)
	{
		connection_info.clear();
		auto verify_run_state = [&ads](std::string& failure_reason) -> bool
		{
			unsigned short ads_state = 0;
			unsigned short device_state = 0;
			if (!ads.ReadDeviceState(ads_state, device_state))
			{
				failure_reason = ads.GetLastErrorCopy();
				ads.CloseComm();
				return false;
			}
			if (ads_state == ADSSTATE_RUN) return true;
			failure_reason = "ADS状态=" + std::to_string(ads_state) + "，要求ADSSTATE_RUN";
			std::cout << "PLC 当前 ADS 状态=" << ads_state
				<< "，只读标定模式要求 PLC 已处于 RUN，程序不会替你切换状态。" << std::endl;
			ads.CloseComm();
			return false;
		};

		std::string local_error;
		if (ads.OpenCommInsideReadOnly())
		{
			if (verify_run_state(local_error))
			{
				connection_info = "本地AMS路由，端口851";
				std::cout << "ADS 只读连接成功：本地 AMS 路由，端口 851。" << std::endl;
				return true;
			}
		}
		else
		{
			local_error = ads.GetLastErrorCopy();
		}

		std::string remote_error;
		if (ads.OpenCommReadOnly())
		{
			if (verify_run_state(remote_error))
			{
				connection_info = "远端AMS NetId 169.254.119.135.1.1，端口851";
				std::cout << "ADS 只读连接成功：远端 AMS NetId 169.254.119.135.1.1，端口 851。" << std::endl;
				return true;
			}
		}
		else
		{
			remote_error = ads.GetLastErrorCopy();
		}
		std::cout << "ADS 只读连接失败。\n  本地：" << local_error
			<< "  远端：" << remote_error;
		connection_info = "连接失败；本地=" + local_error + "；远端=" + remote_error;
		return false;
	}
}

namespace sensor_calibration_experiment
{
	bool is_command(int argc, char* argv[])
	{
		return argc > 1 && argv != nullptr && argv[1] != nullptr &&
			std::strcmp(argv[1], "--sensor-calibration") == 0;
	}

	int run(int argc, char* argv[])
	{
		(void)argc;
		(void)argv;
		Options options;
		std::string error;
		if (!parse_options(options, error))
		{
			std::cout << error << std::endl;
			print_usage();
			return 2;
		}
		if (options.show_help)
		{
			print_usage();
			return 0;
		}
		if (options.self_test)
		{
			return run_self_test() ? 0 : 2;
		}

		std::wstring calibration_directory;
		if (!locate_calibration_directory(options, calibration_directory, error))
		{
			std::cout << error << std::endl;
			return 2;
		}

		std::array<OriginalCalibration, kSensorCount> originals;
		if (!load_original_calibrations(calibration_directory, originals, error))
		{
			std::cout << error << std::endl;
			return 2;
		}
		print_original_summary(calibration_directory, originals);
		if (options.validate_only)
		{
			std::cout << "四份原标定 CSV 校验通过。" << std::endl;
			return 0;
		}
		std::array<SensorResult, kSensorCount> results;
		std::string connection_info = "未连接";

		std::cout
			<< "\n=== 四路传感器重新标定（ADS只读独立模式）===\n"
			<< "本模式不会初始化手柄、不会进入运动主循环，也不会写入PLC变量。\n"
			<< "请先关闭正常ADS控制程序，确认机器人静止，并保持未测传感器空载。\n";
		if (prompt_choice("确认现场安全后按 Enter 继续，Q 中止：", true) == PromptChoice::Abort)
		{
			std::wstring output_path;
			const std::string abort_reason = "用户在现场安全确认前中止，或控制台输入已关闭";
			if (!save_report(
				calibration_directory,
				originals,
				results,
				false,
				abort_reason,
				connection_info,
				output_path,
				error))
			{
				std::cout << error << std::endl;
				return 2;
			}
			std::cout << "实验未开始，诊断报告已保存：" << wide_to_utf8(output_path) << std::endl;
			return 3;
		}

		CADSComm ads;
		if (!connect_read_only(ads, connection_info))
		{
			std::wstring output_path;
			const std::string failure_reason = "ADS只读连接失败，未开始采集";
			if (save_report(
				calibration_directory,
				originals,
				results,
				false,
				failure_reason,
				connection_info,
				output_path,
				error))
			{
				std::cout << "连接失败诊断报告已保存：" << wide_to_utf8(output_path) << std::endl;
			}
			else
			{
				std::cout << error << std::endl;
			}
			return 2;
		}
		if (!confirm_ads_target(connection_info))
		{
			ads.CloseComm();
			std::wstring output_path;
			const std::string abort_reason = "用户未确认当前ADS连接目标，或控制台输入已关闭";
			if (!save_report(
				calibration_directory,
				originals,
				results,
				false,
				abort_reason,
				connection_info,
				output_path,
				error))
			{
				std::cout << error << std::endl;
				return 2;
			}
			std::cout << "实验已中止，连接目标诊断报告已保存："
				<< wide_to_utf8(output_path) << std::endl;
			return 3;
		}
		bool aborted = false;
		std::string abort_reason;

		for (int sensor = 0; sensor < kSensorCount && !aborted; ++sensor)
		{
			bool repeat_sensor = true;
			while (repeat_sensor && !aborted)
			{
				repeat_sensor = false;
				results[sensor] = SensorResult{};
				std::cout << "\n--- 传感器" << sensor + 1 << " -> "
					<< kChannelSymbols[sensor] << " ---" << std::endl;
				for (int point = 0; point < kPointCount; ++point)
				{
					PointMeasurement measurement;
					if (capture_point(ads, sensor, point, measurement) == PromptChoice::Abort)
					{
						aborted = true;
						abort_reason = "用户在传感器" + std::to_string(sensor + 1) + "的" +
							std::to_string(kWeightsG[point]) + "g采集流程中中止，或控制台输入已关闭";
						break;
					}
					results[sensor].points[point] = measurement;
					results[sensor].captured[point] = true;
				}
				if (aborted) break;

				SensorResult& result = results[sensor];
				result.dominant_channel = find_dominant_channel(result);
				result.mapping_warning = result.dominant_channel != sensor;
				if (result.mapping_warning)
				{
					std::cout << "警告：固定映射通道为 " << kChannelSymbols[sensor]
						<< "，但 50g-0g 主响应通道为 "
						<< kChannelSymbols[result.dominant_channel] << "。" << std::endl;
					const PromptChoice choice = prompt_choice(
						"确认物理映射无误则按 Enter，R 重测整只传感器，Q 中止：",
						true);
					if (choice == PromptChoice::Retry)
					{
						repeat_sensor = true;
						continue;
					}
					if (choice == PromptChoice::Abort)
					{
						aborted = true;
						abort_reason = "用户在传感器" + std::to_string(sensor + 1) +
							"主响应通道不符确认阶段中止";
						break;
					}
				}
				result.mapping_confirmed = true;

				if (!calculate_sensor_fits(sensor, originals[sensor], result))
				{
					std::cout << "当前计数跨度退化或拟合失败，本传感器不能形成标定曲线。" << std::endl;
					const PromptChoice choice = prompt_choice("按 R 重测整只传感器，Q 中止：", false);
					if (choice == PromptChoice::Retry)
					{
						repeat_sensor = true;
						continue;
					}
					aborted = true;
					abort_reason = "传感器" + std::to_string(sensor + 1) +
						"当前计数跨度退化或拟合失败，用户未选择重测";
					break;
				}

				result.recommended =
					result.input_span_adequate &&
					result.equivalent_fit.r_squared >= kMinimumR2 &&
					result.direct_fit.r_squared >= kMinimumR2;
				std::cout << "等效原计数：old_equivalent = " << std::setprecision(12)
					<< result.equivalent_fit.slope << " * current_raw "
					<< (result.equivalent_fit.intercept >= 0.0 ? "+ " : "- ")
					<< std::fabs(result.equivalent_fit.intercept)
					<< "，R2=" << result.equivalent_fit.r_squared << std::endl;
				std::cout << "独立物理力：F_direct(N) = " << result.direct_fit.slope
					<< " * current_raw "
					<< (result.direct_fit.intercept >= 0.0 ? "+ " : "- ")
					<< std::fabs(result.direct_fit.intercept)
					<< "，R2=" << result.direct_fit.r_squared << std::endl;

				if (!result.recommended)
				{
					std::cout << "警告：结果未通过全部质量门槛，将标记为不建议使用。" << std::endl;
					if (!result.input_span_adequate)
					{
						std::cout << "  当前计数跨度=" << result.equivalent_fit.input_span
							<< " counts，六点最大标准差=" << result.max_point_stddev_counts
							<< " counts，跨度/噪声=" << result.span_to_noise_ratio
							<< "；要求跨度至少" << kMinimumInputSpanCounts
							<< "且跨度/噪声至少" << kMinimumSpanToNoiseRatio << "。" << std::endl;
					}
					if (result.equivalent_fit.r_squared < kMinimumR2 ||
						result.direct_fit.r_squared < kMinimumR2)
					{
						std::cout << "  至少一种拟合 R2 < " << kMinimumR2 << "。" << std::endl;
					}
					const PromptChoice choice = prompt_choice(
						"按 Enter 保留诊断结果并继续，R 重测整只传感器，Q 中止：",
						true);
					if (choice == PromptChoice::Retry)
					{
						repeat_sensor = true;
						continue;
					}
					if (choice == PromptChoice::Abort)
					{
						aborted = true;
						abort_reason = "用户在传感器" + std::to_string(sensor + 1) +
							"拟合质量未通过确认阶段中止";
						break;
					}
				}
				result.complete = true;
			}
		}

		ads.CloseComm();
		const bool all_complete = !aborted && std::all_of(
			results.begin(),
			results.end(),
			[](const SensorResult& result) { return result.complete; });
		if (!all_complete && abort_reason.empty())
		{
			abort_reason = "实验未完成，至少一路传感器没有形成完整结果";
		}
		std::wstring output_path;
		if (!save_report(
			calibration_directory,
			originals,
			results,
			all_complete,
			abort_reason,
			connection_info,
			output_path,
			error))
		{
			std::cout << error << std::endl;
			return 2;
		}
		std::cout << (!all_complete ? "实验已中止，诊断数据已保存：" : "标定完成，结果已保存：")
			<< wide_to_utf8(output_path) << std::endl;
		return all_complete ? 0 : 3;
	}
}
