// 文件职责说明：
// 1) 力过渡决定性预实验（论文 §6.1）专用 CSV 写入器。
//    每次 start_session() 创建新文件，stop_session() 收尾。
// 2) SPSC 环形缓冲，主线程入队、writer 线程落盘；不阻塞主循环。
// 3) 仅在 ForceTransitionExperiment 处于 active 期间，由主循环调 enqueue() 才入队。
// 4) 列结构覆盖论文 §6.1 步骤 4 三量同步要求（力、从端位移、时间），并保留
//    试次/档位/相位元数据用于离线切片。
#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

class ForceTransitionLogger
{
public:
    ForceTransitionLogger();
    ~ForceTransitionLogger();

    // 一次完整扫描（多档位×多重复）对应一个 CSV 文件。文件名 ForceTransition_YYYYMMDD_HHMMSS.csv。
    bool start_session(const std::string& output_dir);
    void stop_session();
    bool is_running() const { return running_.load(); }

    // ForceTransitionExperiment 在状态机里调用，标记新试次开始/结束（仅写入元数据快照供 enqueue 读）。
    void mark_trial_started(int trial_id, int velocity_level, int repeat_in_level, double v_ratio);
    void mark_trial_finished(int trial_id);

    struct Row
    {
        int trial_id;
        int velocity_level;
        int repeat_in_level;
        int phase_code;

        std::uint64_t tick_ms;
        std::uint64_t dt_ms_from_phase_start;

        double axis1_act_pos_mm;
        double axis1_refer_mm;
        double axis1_v_limit_used;

        double fn_1_raw_v;
        double ft_1_raw_v;
        double fn_1_zero_v;
        double ft_1_zero_v;

        double f_feedback_n;
        double t_feedback_nm;

        bool ff_enabled;
        bool cal_zeroed;
        bool freeze_active;
        bool fast_active;
        int guidewire_mode;
        bool axis1_reverse;

        unsigned short cyl1_cmd;
        unsigned short cyl2_cmd;
    };

    // 主循环每拍调用一次（仅当实验 active）。
    void enqueue(const Row& row);

    std::uint64_t dropped_count() const { return dropped_.load(); }

private:
    void writer_loop();
    bool open_file(const std::string& output_dir);
    void close_file();
    void write_header();

    static constexpr std::size_t kCapacity = 4096;
    std::vector<Row> ring_;
    std::atomic<std::size_t> head_{ 0 };
    std::atomic<std::size_t> tail_{ 0 };

    std::atomic<bool> running_{ false };
    std::atomic<bool> stop_requested_{ false };
    std::atomic<std::uint64_t> dropped_{ 0 };

    std::thread writer_thread_;
    std::FILE* fp_ = nullptr;
};
