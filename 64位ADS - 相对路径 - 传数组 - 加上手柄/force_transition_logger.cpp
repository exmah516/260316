// 文件职责说明：力过渡预实验专用 CSV 写入器实现（仿 ForceLogger）。
// SPSC 4096 + writer 线程；列结构详见 force_transition_logger.h。

#include "force_transition_logger.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <thread>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

ForceTransitionLogger::ForceTransitionLogger()
    : ring_(kCapacity)
{
}

ForceTransitionLogger::~ForceTransitionLogger()
{
    stop_session();
}

bool ForceTransitionLogger::start_session(const std::string& output_dir)
{
    if (running_.load())
        return true;
    if (!open_file(output_dir))
        return false;
    stop_requested_.store(false);
    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
    dropped_.store(0, std::memory_order_relaxed);
    write_header();
    try
    {
        writer_thread_ = std::thread(&ForceTransitionLogger::writer_loop, this);
    }
    catch (...)
    {
        close_file();
        return false;
    }
    running_.store(true);
    return true;
}

void ForceTransitionLogger::stop_session()
{
    if (!running_.load() && !writer_thread_.joinable())
        return;
    stop_requested_.store(true);
    if (writer_thread_.joinable())
        writer_thread_.join();
    close_file();
    running_.store(false);
}

void ForceTransitionLogger::mark_trial_started(int /*trial_id*/, int /*velocity_level*/, int /*repeat_in_level*/, double /*v_ratio*/)
{
    // 占位：主循环已将元数据合并入每行，无需在 logger 内额外维护。
}

void ForceTransitionLogger::mark_trial_finished(int /*trial_id*/)
{
    // 占位：现阶段元数据全部由 enqueue 调用方携带。
}

void ForceTransitionLogger::enqueue(const Row& row)
{
    if (!running_.load(std::memory_order_acquire))
        return;
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) % kCapacity;
    if (next == tail_.load(std::memory_order_acquire))
    {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    ring_[head] = row;
    head_.store(next, std::memory_order_release);
}

bool ForceTransitionLogger::open_file(const std::string& output_dir)
{
    if (!output_dir.empty() && output_dir != ".")
    {
        CreateDirectoryA(output_dir.c_str(), nullptr);
    }

    std::time_t t = std::time(nullptr);
    std::tm tm_local;
#if defined(_WIN32)
    localtime_s(&tm_local, &t);
#else
    tm_local = *std::localtime(&t);
#endif
    char stamp[32] = { 0 };
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_local);

    std::string path = output_dir;
    if (!path.empty() && path.back() != '/' && path.back() != '\\')
        path += '/';
    path += "ForceTransition_";
    path += stamp;
    path += ".csv";

#if defined(_WIN32)
    if (fopen_s(&fp_, path.c_str(), "wb") != 0)
    {
        fp_ = nullptr;
        return false;
    }
#else
    fp_ = std::fopen(path.c_str(), "wb");
    if (!fp_)
        return false;
#endif
    return true;
}

void ForceTransitionLogger::close_file()
{
    if (fp_)
    {
        std::fflush(fp_);
        std::fclose(fp_);
        fp_ = nullptr;
    }
}

void ForceTransitionLogger::write_header()
{
    const char* header =
        "trial_id,velocity_level,repeat_in_level,phase_code,"
        "tick_ms,dt_ms_from_phase_start,"
        "axis1_act_pos_mm,axis1_refer_mm,axis1_v_limit_used,"
        "fn_1_raw_v,ft_1_raw_v,fn_1_zero_v,ft_1_zero_v,"
        "f_feedback_n,t_feedback_nm,"
        "ff_enabled,cal_zeroed,freeze_active,fast_active,"
        "guidewire_mode,axis1_reverse,"
        "cyl1_cmd,cyl2_cmd\n";
    if (fp_)
    {
        std::fwrite(header, 1, std::strlen(header), fp_);
        std::fflush(fp_);
    }
}

void ForceTransitionLogger::writer_loop()
{
    std::size_t flush_counter = 0;
    char buf[512];
    while (true)
    {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t head = head_.load(std::memory_order_acquire);
        if (tail == head)
        {
            if (stop_requested_.load(std::memory_order_acquire))
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        const Row& row = ring_[tail];
        const int n = std::snprintf(buf, sizeof(buf),
            "%d,%d,%d,%d,"
            "%llu,%llu,"
            "%.4f,%.4f,%.4f,"
            "%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,"
            "%d,%d,%d,%d,"
            "%d,%d,"
            "%u,%u\n",
            row.trial_id, row.velocity_level, row.repeat_in_level, row.phase_code,
            static_cast<unsigned long long>(row.tick_ms),
            static_cast<unsigned long long>(row.dt_ms_from_phase_start),
            row.axis1_act_pos_mm, row.axis1_refer_mm, row.axis1_v_limit_used,
            row.fn_1_raw_v, row.ft_1_raw_v, row.fn_1_zero_v, row.ft_1_zero_v,
            row.f_feedback_n, row.t_feedback_nm,
            row.ff_enabled ? 1 : 0, row.cal_zeroed ? 1 : 0,
            row.freeze_active ? 1 : 0, row.fast_active ? 1 : 0,
            row.guidewire_mode, row.axis1_reverse ? 1 : 0,
            static_cast<unsigned int>(row.cyl1_cmd),
            static_cast<unsigned int>(row.cyl2_cmd));
        if (n > 0 && fp_)
        {
            std::fwrite(buf, 1, static_cast<std::size_t>(n), fp_);
        }
        tail_.store((tail + 1) % kCapacity, std::memory_order_release);
        if (++flush_counter >= 64)
        {
            std::fflush(fp_);
            flush_counter = 0;
        }
    }
    if (fp_)
        std::fflush(fp_);
}
