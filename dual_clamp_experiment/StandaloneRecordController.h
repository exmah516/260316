#pragma once

#include "ExperimentStreamAds.h"
#include "ExperimentStreamRecorder.h"
#include "StandaloneRecordAds.h"

#include <array>
#include <cstdint>
#include <mutex>
#include <string>

class StandaloneRecordController
{
public:
    StandaloneRecordController();

    bool open_ads();
    void close_ads();
    bool is_ads_open() const;
    void tick();

    bool set_cylinder_config(int cylinder, bool enabled, std::uint16_t open_value, std::uint16_t close_value);
    bool cylinder_open(int cylinder);
    bool cylinder_close(int cylinder);
    bool request_zero();
    bool start_record(const std::string& suffix, std::uint64_t field_mask);
    bool stop_record();
    bool abort_record(const std::string& reason);
    bool save(std::string& error);
    void release_manual_control();
    void invalidate_zero();

    StandaloneRecordLiveFrame live() const;
    ForceZeroState zero_state() const;
    std::string last_error() const;
    std::string recording_directory() const;
    bool recording_archived() const;
    bool recording_active() const;
    bool recording_stopping() const;
    std::uint32_t recording_sample_count() const;
    std::uint64_t field_mask() const;

private:
    bool read_and_poll_locked();
    bool poll_blocks_locked();
    bool finalize_locked(const char* status, const std::string& reason);
    bool valid_idle_locked() const;
    void set_error_locked(const std::string& error);

    mutable std::mutex mutex_;
    StandaloneRecordAds ads_;
    ExperimentStreamAds stream_ads_;
    ExperimentStreamRecorder recorder_;
    StandaloneRecordLiveFrame live_{};
    ForceZeroState zero_{};
    std::array<bool, 4> cylinder_enabled_{ { true, true, true, true } };
    std::array<std::uint16_t, 4> cylinder_open_{ { 1000, 0, 400, 0 } };
    std::array<std::uint16_t, 4> cylinder_close_{ { 0, 600, 50, 500 } };
    std::array<std::uint16_t, 4> cylinder_current_{ { 1000, 0, 400, 0 } };
    std::string record_suffix_ = "standalone_record";
    std::uint64_t field_mask_ = 0;
    std::uint32_t event_sequence_ = 0;
    std::uint32_t expected_block_sequence_ = 0;
    std::uint32_t expected_sample_index_ = 0;
    bool recording_ = false;
    bool stop_requested_ = false;
    unsigned stop_poll_cycles_ = 0;
    std::string stop_status_ = "Completed";
    std::string stop_reason_;
    std::string last_error_;
    bool zero_file_written_ = false;
};
