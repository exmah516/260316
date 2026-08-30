#include "StandaloneRecordController.h"

#include <algorithm>
#include <chrono>
#include <thread>

namespace
{
    bool is_legacy_motion_phase(std::uint16_t phase)
    {
        return (phase >= 3 && phase <= 9) || phase == 14;
    }

    bool is_program_motion_phase(std::uint8_t phase)
    {
        return phase >= 1 && phase <= 9;
    }

    constexpr std::uint64_t default_force_fields()
    {
        return standalone_field_bit(StandaloneRecordField::Fn1Raw) |
            standalone_field_bit(StandaloneRecordField::Ft1Raw) |
            standalone_field_bit(StandaloneRecordField::Fn2Raw) |
            standalone_field_bit(StandaloneRecordField::Ft2Raw);
    }
}

StandaloneRecordController::StandaloneRecordController()
{
    // 不在构造阶段同步阻塞ADS连接；先启动后端和UI，再由连接命令执行重试。
}

bool StandaloneRecordController::open_ads()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (ads_.is_open()) return true;
    if (!ads_.open())
    {
        set_error_locked("独立功能ADS连接失败：" + ads_.last_error());
        return false;
    }
    if (!stream_ads_.is_open() && !stream_ads_.open())
    {
        set_error_locked("独立记录ADS连接失败：" + stream_ads_.last_error());
        return false;
    }
    stream_ads_.invalidate_zero();
    zero_ = {};
    last_error_.clear();
    return true;
}

void StandaloneRecordController::close_ads()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (recording_ || stop_requested_)
    {
        if (ads_.is_open()) ads_.set_record_enable(false, ++event_sequence_);
        stop_requested_ = true;
        stop_status_ = "Error";
        stop_reason_ = "ADS连接已断开";
        poll_blocks_locked();
        if (recorder_.active())
        {
            std::string ignored;
            recorder_.finalize("Error", stop_reason_, zero_, ignored);
        }
    }
    if (ads_.is_open()) ads_.set_manual_cylinder(false, cylinder_current_);
    if (stream_ads_.is_open()) stream_ads_.invalidate_zero();
    ads_.close();
    stream_ads_.close();
    recording_ = false;
    stop_requested_ = false;
    stop_poll_cycles_ = 0;
    zero_ = {};
}

bool StandaloneRecordController::is_ads_open() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return ads_.is_open() && stream_ads_.is_open();
}

void StandaloneRecordController::set_error_locked(const std::string& error)
{
    last_error_ = error;
}

bool StandaloneRecordController::valid_idle_locked() const
{
    return !is_legacy_motion_phase(live_.legacy_phase) && !is_program_motion_phase(live_.program_phase);
}

bool StandaloneRecordController::set_cylinder_config(int cylinder, bool enabled,
    std::uint16_t open_value, std::uint16_t close_value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (cylinder < 1 || cylinder > 4)
    {
        set_error_locked("电缸编号必须为1至4");
        return false;
    }
    if (!valid_idle_locked())
    {
        set_error_locked("实验运动或夹爪切换期间不能修改电缸配置");
        return false;
    }
    const std::size_t index = static_cast<std::size_t>(cylinder - 1);
    cylinder_enabled_[index] = enabled;
    cylinder_open_[index] = open_value;
    cylinder_close_[index] = close_value;
    return true;
}

bool StandaloneRecordController::cylinder_open(int cylinder)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (cylinder < 1 || cylinder > 4)
    {
        set_error_locked("电缸编号必须为1至4");
        return false;
    }
    if (!live_.selfcheck_done)
    {
        set_error_locked("PLC自检未完成，暂不能控制电缸");
        return false;
    }
    if (!valid_idle_locked())
    {
        set_error_locked("实验运动或夹爪切换期间不能控制电缸");
        return false;
    }
    const std::size_t index = static_cast<std::size_t>(cylinder - 1);
    if (!cylinder_enabled_[index])
    {
        set_error_locked("该电缸未启用");
        return false;
    }
    cylinder_current_[index] = cylinder_open_[index];
    if (!ads_.set_manual_cylinder(true, cylinder_current_))
    {
        set_error_locked("下发电缸打开命令失败：" + ads_.last_error());
        return false;
    }
    return true;
}

bool StandaloneRecordController::cylinder_close(int cylinder)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (cylinder < 1 || cylinder > 4)
    {
        set_error_locked("电缸编号必须为1至4");
        return false;
    }
    if (!live_.selfcheck_done)
    {
        set_error_locked("PLC自检未完成，暂不能控制电缸");
        return false;
    }
    if (!valid_idle_locked())
    {
        set_error_locked("实验运动或夹爪切换期间不能控制电缸");
        return false;
    }
    const std::size_t index = static_cast<std::size_t>(cylinder - 1);
    if (!cylinder_enabled_[index])
    {
        set_error_locked("该电缸未启用");
        return false;
    }
    cylinder_current_[index] = cylinder_close_[index];
    if (!ads_.set_manual_cylinder(true, cylinder_current_))
    {
        set_error_locked("下发电缸关闭命令失败：" + ads_.last_error());
        return false;
    }
    return true;
}

bool StandaloneRecordController::request_zero()
{
    try
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!ads_.is_open() || !stream_ads_.is_open())
        {
            set_error_locked("ADS未连接");
            return false;
        }
        if (recording_ || stop_requested_)
        {
            set_error_locked("独立记录进行中不能取零");
            return false;
        }
        if (!valid_idle_locked())
        {
            set_error_locked("实验运动或夹爪切换期间不能取零");
            return false;
        }
        if (!recorder_.active())
        {
            std::string error;
            if (!recorder_.begin_standalone(record_suffix_, default_force_fields(), error))
            {
                set_error_locked(error);
                return false;
            }
        }
        zero_file_written_ = false;
        std::string error;
        if (!recorder_.reset_zero_file(error) || !recorder_.begin_zero(error) || !stream_ads_.request_zero())
        {
            set_error_locked(error.empty() ? "下发独立取零请求失败：" + stream_ads_.last_error() : error);
            return false;
        }
        return true;
    }
    catch (const std::exception& ex)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        set_error_locked(std::string("独立力感取零过程异常：") + ex.what());
        return false;
    }
}

bool StandaloneRecordController::start_record(const std::string& suffix, std::uint64_t field_mask)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ads_.is_open() || !stream_ads_.is_open())
    {
        set_error_locked("ADS未连接");
        return false;
    }
    if (field_mask == 0)
    {
        set_error_locked("至少选择一个独立记录字段");
        return false;
    }
    if (recording_ || stop_requested_)
    {
        set_error_locked("独立记录已经在进行");
        return false;
    }
    if (!valid_idle_locked())
    {
        set_error_locked("旧双机构实验正在运动，不能启动独立记录");
        return false;
    }
    record_suffix_ = suffix.empty() ? "standalone_record" : suffix;
    field_mask_ = field_mask;
    if (!recorder_.active())
    {
        std::string error;
        if (!recorder_.begin_standalone(record_suffix_, field_mask_, error))
        {
            set_error_locked(error);
            return false;
        }
    }
    else
    {
        std::string error;
        if (!recorder_.reconfigure_standalone(field_mask_, error))
        {
            set_error_locked(error);
            return false;
        }
    }
    if (!stream_ads_.reset_recording())
    {
        set_error_locked("清空独立记录缓冲失败：" + stream_ads_.last_error());
        return false;
    }
    expected_block_sequence_ = 0;
    expected_sample_index_ = 0;
    stop_poll_cycles_ = 0;
    ++event_sequence_;
    if (!ads_.set_record_enable(true, event_sequence_))
    {
        set_error_locked("启动独立记录失败：" + ads_.last_error());
        return false;
    }
    recording_ = true;
    stop_requested_ = false;
    stop_status_ = "Completed";
    stop_reason_.clear();
    last_error_.clear();
    return true;
}

bool StandaloneRecordController::stop_record()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!recording_ && !stop_requested_)
    {
        set_error_locked("独立记录尚未开始");
        return false;
    }
    if (ads_.is_open() && !ads_.set_record_enable(false, ++event_sequence_))
    {
        set_error_locked("停止独立记录失败：" + ads_.last_error());
        return false;
    }
    recording_ = false;
    stop_requested_ = true;
    stop_poll_cycles_ = 0;
    stop_status_ = "Completed";
    stop_reason_.clear();
    return true;
}

bool StandaloneRecordController::abort_record(const std::string& reason)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!recording_ && !stop_requested_) return true;
    if (ads_.is_open()) ads_.set_record_enable(false, ++event_sequence_);
    recording_ = false;
    stop_requested_ = true;
    stop_poll_cycles_ = 0;
    stop_status_ = "Aborted";
    stop_reason_ = reason;
    return true;
}

bool StandaloneRecordController::finalize_locked(const char* status, const std::string& reason)
{
    if (!recorder_.active())
    {
        stop_requested_ = false;
        return true;
    }
    std::string error;
    const bool ok = recorder_.finalize(status, reason, zero_, error);
    if (!ok) set_error_locked(error.empty() ? recorder_.last_error() : error);
    stop_requested_ = false;
    return ok;
}

bool StandaloneRecordController::poll_blocks_locked()
{
    ExperimentStreamStatus status{};
    if (!stream_ads_.read_status(status))
    {
        if (recording_ || stop_requested_)
        {
            set_error_locked("读取独立记录状态失败：" + stream_ads_.last_error());
            stop_status_ = "Error";
            stop_reason_ = last_error_;
            if (ads_.is_open()) ads_.set_record_enable(false, ++event_sequence_);
            recording_ = false;
            stop_requested_ = true;
        }
        return false;
    }
    const double saved_zero_axis2 = zero_.axis2_angle_deg;
    const double saved_zero_axis7 = zero_.axis7_angle_deg;
    zero_ = status.zero;
    if (zero_.done)
    {
        if (zero_file_written_)
        {
            zero_.axis2_angle_deg = saved_zero_axis2;
            zero_.axis7_angle_deg = saved_zero_axis7;
        }
        else
        {
            zero_.axis2_angle_deg = live_.axis2_angle_deg;
            zero_.axis7_angle_deg = live_.axis7_angle_deg;
        }
    }
    if (status.zero.done && !zero_file_written_ && recorder_.active())
    {
        std::vector<std::array<double, 4>> zero_samples;
        std::string error;
        if (!stream_ads_.read_zero_samples(zero_samples))
        {
            set_error_locked("读取独立取零样本失败：" + stream_ads_.last_error());
            return false;
        }
        bool ok = true;
        for (std::size_t i = 0; i < zero_samples.size(); ++i)
            ok = recorder_.append_zero(zero_samples[i], static_cast<std::uint64_t>(i) * 1000,
                live_.axis2_angle_deg, live_.axis7_angle_deg, error) && ok;
        ok = recorder_.finish_zero(status.zero, error) && ok;
        if (!ok)
        {
            set_error_locked(error);
            return false;
        }
        zero_file_written_ = true;
    }
    if (status.overflow)
    {
        set_error_locked("独立记录缓冲区溢出");
        stop_status_ = "Error";
        stop_reason_ = last_error_;
        if (ads_.is_open()) ads_.set_record_enable(false, ++event_sequence_);
        recording_ = false;
        stop_requested_ = true;
        return false;
    }
    if (!recorder_.active() || status.source_mode != 3) return true;
    for (int pass = 0; pass < 2; ++pass)
    {
        int slot = -1;
        for (int candidate = 0; candidate < 2; ++candidate)
        {
            if (status.block_ready[candidate] && status.block_sequence[candidate] == expected_block_sequence_)
            {
                slot = candidate;
                break;
            }
        }
        if (slot < 0) break;
        std::vector<ExperimentStreamSample> samples;
        std::uint32_t sequence = 0;
        if (!stream_ads_.read_block(slot, samples, sequence) || sequence != expected_block_sequence_)
        {
            set_error_locked("独立记录分块读取失败或序号不连续");
            stop_status_ = "Error";
            stop_reason_ = last_error_;
            if (ads_.is_open()) ads_.set_record_enable(false, ++event_sequence_);
            recording_ = false;
            stop_requested_ = true;
            return false;
        }
        for (const auto& sample : samples)
        {
            if (sample.index != expected_sample_index_)
            {
                set_error_locked("独立记录样本序号不连续");
                stop_status_ = "Error";
                stop_reason_ = last_error_;
                if (ads_.is_open()) ads_.set_record_enable(false, ++event_sequence_);
                recording_ = false;
                stop_requested_ = true;
                return false;
            }
            ++expected_sample_index_;
        }
        std::string error;
        if (!recorder_.append_standalone(samples, zero_, field_mask_, error) || !stream_ads_.acknowledge_block(slot, sequence))
        {
            set_error_locked(error.empty() ? "确认独立记录分块失败：" + stream_ads_.last_error() : error);
            stop_status_ = "Error";
            stop_reason_ = last_error_;
            if (ads_.is_open()) ads_.set_record_enable(false, ++event_sequence_);
            recording_ = false;
            stop_requested_ = true;
            return false;
        }
        ++expected_block_sequence_;
        status.block_ready[slot] = false;
    }
    if (stop_requested_)
    {
        ++stop_poll_cycles_;
        // 给PLC至少两个任务轮次完成“停止采样并提交最后不满块”的状态转换。
        if (stop_poll_cycles_ >= 2 && !status.recording && !status.block_ready[0] && !status.block_ready[1])
            finalize_locked(stop_status_.c_str(), stop_reason_);
    }
    return true;
}

bool StandaloneRecordController::read_and_poll_locked()
{
    if (!ads_.is_open() || !stream_ads_.is_open()) return false;
    if (!ads_.read_live(live_))
    {
        set_error_locked("读取独立功能状态失败：" + ads_.last_error());
        return false;
    }
    return poll_blocks_locked();
}

void StandaloneRecordController::tick()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ads_.is_open() || !stream_ads_.is_open()) return;
    read_and_poll_locked();
    if (recorder_.failed())
    {
        set_error_locked(recorder_.last_error());
        if (recording_ && ads_.is_open()) ads_.set_record_enable(false, ++event_sequence_);
        recording_ = false;
        stop_requested_ = true;
        stop_status_ = "Error";
        stop_reason_ = last_error_;
    }
}

void StandaloneRecordController::release_manual_control()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (ads_.is_open()) ads_.set_manual_cylinder(false, cylinder_current_);
}

void StandaloneRecordController::invalidate_zero()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_ads_.is_open()) stream_ads_.invalidate_zero();
    zero_ = {};
    zero_file_written_ = false;
}

bool StandaloneRecordController::save(std::string& error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (recording_ || stop_requested_)
    {
        error = "独立记录正在停止或进行中";
        return false;
    }
    if (!recorder_.active() && !recorder_.archived())
    {
        error = "独立记录尚未开始";
        return false;
    }
    if (recorder_.archived()) return true;
    return recorder_.finalize("Completed", last_error_, zero_, error);
}

StandaloneRecordLiveFrame StandaloneRecordController::live() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return live_;
}

ForceZeroState StandaloneRecordController::zero_state() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return zero_;
}

std::string StandaloneRecordController::last_error() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
}

std::string StandaloneRecordController::recording_directory() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return recorder_.directory();
}

bool StandaloneRecordController::recording_archived() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return recorder_.archived();
}

bool StandaloneRecordController::recording_active() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return recording_;
}

bool StandaloneRecordController::recording_stopping() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return stop_requested_;
}

std::uint32_t StandaloneRecordController::recording_sample_count() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<std::uint32_t>(recorder_.sample_count());
}

std::uint64_t StandaloneRecordController::field_mask() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return field_mask_;
}
