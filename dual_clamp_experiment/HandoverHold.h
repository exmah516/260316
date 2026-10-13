#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <ostream>

namespace handoverhold {
inline constexpr const char* kVersion = "handover_hold_v1";
inline constexpr unsigned kStableSamples = 50, kSpatialSamples = 20;
inline constexpr double kStableRangeN = 0.1, kSearchStartMm = 5.0, kForcedStartMm = 2.0;
inline constexpr double kSampleDistanceMm = 1.0, kResumeDistanceMm = 2.0;
enum class Source : unsigned { None, StableWindow, ForcedAt2mm };
enum class Status : unsigned {
    Inactive, Live, Searching, Sampling, ReferenceReady, Holding, Resumed,
    NoReference, InvalidInput, SampleGap, Ended, SpatialUndersampled
};
inline const char* source_name(Source s) {
    switch (s) {
    case Source::StableWindow: return "stable_window";
    case Source::ForcedAt2mm: return "forced_at_2mm";
    default: return "none";
    }
}
inline const char* status_name(Status s) {
    switch (s) {
    case Status::Live: return "live";
    case Status::Searching: return "searching";
    case Status::Sampling: return "sampling";
    case Status::ReferenceReady: return "reference_ready";
    case Status::Holding: return "holding";
    case Status::Resumed: return "resumed";
    case Status::NoReference: return "no_reference";
    case Status::InvalidInput: return "invalid_input";
    case Status::SampleGap: return "sample_gap";
    case Status::Ended: return "ended";
    case Status::SpatialUndersampled: return "spatial_undersampled";
    default: return "inactive";
    }
}
struct Input {
    std::uint64_t time_us = 0;
    std::uint32_t index = 0;
    unsigned phase = 0, cycle = 0;
    double position_mm = 0, trigger_mm = 3, fn_N = 0, torque_Nmm = 0;
    bool valid = false, catheter = false;
};
struct Result {
    double fn_N = 0, torque_Nmm = 0;
    bool valid = false, reference_valid = false, holding = false;
    Source source = Source::None;
    Status status = Status::Inactive;
    unsigned sample_count = 0;
    std::uint32_t reference_start_index = 0;
};

class Tracker {
public:
    void reset() { *this = Tracker{}; }
    Result update(const Input& in) {
        if (!in.catheter) { reset(); return {}; }
        if (in.phase == 0 || in.phase >= 10) { reset(); Result r; r.status = Status::Ended; return r; }
        if (!in.valid || !std::isfinite(in.position_mm) || !std::isfinite(in.trigger_mm) ||
            !std::isfinite(in.fn_N) || !std::isfinite(in.torque_Nmm)) {
            reset(); Result r; r.status = Status::InvalidInput; return r;
        }
        const bool gap = seen_ && (in.index != index_ + 1 || in.time_us <= time_us_ || in.time_us - time_us_ != 1000);
        if (gap) { reset(); remember(in); Result r; r.status = Status::SampleGap; return r; }
        const bool entered = !seen_ || phase_ != in.phase || cycle_ != in.cycle;
        if (entered && (in.phase == 4 || in.phase == 9)) {
            forward_start_mm_ = in.position_mm;
            recovering_ = holding_;
            candidate_ = {};
            stable_count_ = stable_cursor_ = 0;
        }
        if (entered && in.phase == 5) {
            held_ = (seen_ && phase_ == 4) ? candidate_ : Candidate{};
            holding_ = held_.ready;
            recovering_ = false;
        }
        bool resumed = false;
        if (holding_ && recovering_ && (in.phase == 4 || in.phase == 9) &&
            forward_start_mm_ - in.position_mm >= kResumeDistanceMm - 1e-9) {
            holding_ = recovering_ = false;
            resumed = true;
        }
        if (in.phase < 4) { holding_ = recovering_ = false; candidate_ = {}; }
        if (in.phase == 4) acquire(in);

        Result r;
        r.fn_N = in.fn_N; r.torque_Nmm = in.torque_Nmm; r.valid = true;
        r.status = Status::Live;
        const Candidate& ref = holding_ ? held_ : ((in.phase >= 5 && in.phase <= 8) ? held_ : candidate_);
        r.reference_valid = ref.ready; r.source = ref.source;
        r.sample_count = ref.count; r.reference_start_index = ref.start_index;
        if (holding_) {
            r.fn_N = held_.fn_sum / kSpatialSamples;
            r.torque_Nmm = held_.torque_sum / kSpatialSamples;
            r.holding = true; r.status = Status::Holding;
        } else if (in.phase >= 5 && in.phase <= 8) {
            r.valid = false; r.status = Status::NoReference;
        } else if (resumed) r.status = Status::Resumed;
        else if (in.phase == 4) r.status = candidate_.undersampled ? Status::SpatialUndersampled :
            candidate_.ready ? Status::ReferenceReady : candidate_.source != Source::None ? Status::Sampling : Status::Searching;
        remember(in);
        return r;
    }
private:
    struct Candidate {
        Source source = Source::None;
        bool ready = false, undersampled = false;
        unsigned count = 0;
        double start_mm = 0, progress_mm = 0, fn_sum = 0, torque_sum = 0;
        std::uint32_t start_index = 0;
    } candidate_, held_;
    std::array<double, kStableSamples> stable_{};
    unsigned stable_count_ = 0, stable_cursor_ = 0, phase_ = 0, cycle_ = 0;
    bool seen_ = false, holding_ = false, recovering_ = false;
    double forward_start_mm_ = 0;
    std::uint64_t time_us_ = 0;
    std::uint32_t index_ = 0;
    void remember(const Input& in) {
        seen_ = true; time_us_ = in.time_us; index_ = in.index; phase_ = in.phase; cycle_ = in.cycle;
    }
    void acquire(const Input& in) {
        if (candidate_.ready || candidate_.undersampled) return;
        if (candidate_.source == Source::None) {
            const double remaining = in.position_mm - in.trigger_mm;
            bool stable = false;
            if (remaining >= kForcedStartMm && remaining <= kSearchStartMm) {
                stable_[stable_cursor_++ % kStableSamples] = in.fn_N;
                stable_count_ = std::min(stable_count_ + 1, kStableSamples);
                if (stable_count_ == kStableSamples) {
                    const auto range = std::minmax_element(stable_.begin(), stable_.end());
                    stable = *range.second - *range.first <= kStableRangeN + 1e-12;
                }
            } else stable_count_ = stable_cursor_ = 0;
            if (stable || remaining <= kForcedStartMm) {
                candidate_.source = stable ? Source::StableWindow : Source::ForcedAt2mm;
                candidate_.start_mm = in.position_mm; candidate_.start_index = in.index;
            }
            return;
        }
        candidate_.progress_mm = std::max(candidate_.progress_mm, candidate_.start_mm - in.position_mm);
        const double step = kSampleDistanceMm / kSpatialSamples;
        if (candidate_.progress_mm + 1e-9 < (candidate_.count + 1) * step) return;
        // 一个实测点不能冒充多个空间采样点；跨过两格时本轮参考无效。
        if (candidate_.count + 1 < kSpatialSamples &&
            candidate_.progress_mm + 1e-9 >= (candidate_.count + 2) * step) {
            candidate_.undersampled = true; return;
        }
        candidate_.fn_sum += in.fn_N; candidate_.torque_sum += in.torque_Nmm;
        candidate_.ready = ++candidate_.count == kSpatialSamples;
    }
};

inline void write_csv_header(std::ostream& out) {
    out << ",handover_fn_N,handover_torque_Nmm,handover_valid,handover_reference_valid,"
        "handover_holding,handover_source,handover_status,handover_sample_count,handover_reference_start_index";
}
inline void write_csv(std::ostream& out, const Result& r) {
    out << ',';
    if (r.valid) out << r.fn_N;
    out << ',';
    if (r.valid) out << r.torque_Nmm;
    out << ',' << r.valid << ',' << r.reference_valid << ',' << r.holding
        << ',' << source_name(r.source) << ',' << status_name(r.status)
        << ',' << r.sample_count << ',' << r.reference_start_index;
}
inline void write_metadata(std::ostream& out) {
    out << "{\"version\":\"" << kVersion << "\",\"scope\":\"catheter_axis1_display_only\","
        "\"display_default_enabled\":false,\"recording_always_computed\":true,"
        "\"stable_samples\":" << kStableSamples << ",\"stable_range_N\":" << kStableRangeN
        << ",\"search_start_mm\":" << kSearchStartMm << ",\"forced_start_mm\":" << kForcedStartMm
        << ",\"sample_distance_mm\":" << kSampleDistanceMm << ",\"spatial_samples\":" << kSpatialSamples
        << ",\"resume_distance_mm\":" << kResumeDistanceMm
        << ",\"sample_interval_us\":1000,\"hold_start\":\"phase4_to_phase5\",\"resume_phases\":[4,9]}";
}
}
