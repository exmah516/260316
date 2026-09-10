#pragma once
#include "ClampModelSelection.h"
#include <deque>
#include <iomanip>
#include <sstream>
#include <string>

namespace clampmodel {
struct CurvePoint {
    std::uint64_t sequence = 0;
    double time = 0, fn = 0, ft = 0, corrected_fn = 0, corrected_ft = 0;
    bool valid = false;
    int phase = 0;
    double illustration_fn = 0, illustration_ft = 0;
    bool illustration_valid_fn = false, illustration_valid_ft = false;
};
// Called under ProgrammedDeliveryController's mutex.
class CurveBuffer {
public:
    void reset() { points_.clear(); ++generation_; }
    void push(CurvePoint p) {
        p.sequence = ++sequence_;
        points_.push_back(p);
        while (!points_.empty() && (points_.front().time < p.time - 10.0 || points_.size() > 10001))
            points_.pop_front();
    }
    std::string response(std::uint64_t after, std::uint64_t generation, int mode,
                         bool available, bool calibration_ok, bool record_ok,
                         double compute_us, double span_ms, const char* version = kVersion) const {
        const bool changed = generation != generation_;
        const bool gap = !changed && !points_.empty() && after + 1 < points_.front().sequence;
        std::ostringstream out;
        out << std::setprecision(12) << "PROGRAM_CURVES|" << generation_ << '|' << mode << '|'
            << available << '|' << calibration_ok << '|' << record_ok << '|' << version << '|'
            << compute_us << '|' << span_ms << '|' << gap << '|';
        int count = 0;
        for (const auto& p : points_) {
            if (!changed && p.sequence <= after) continue;
            if (count++) out << ';';
            out << p.sequence << ',' << p.time << ',' << p.fn << ',' << p.ft << ','
                << p.corrected_fn << ',' << p.corrected_ft << ',' << p.valid << ',' << p.phase
                << ',' << p.illustration_fn << ',' << p.illustration_ft
                << ',' << p.illustration_valid_fn << ',' << p.illustration_valid_ft;
            if (count >= 1024) break;
        }
        return out.str();
    }
private:
    std::deque<CurvePoint> points_;
    std::uint64_t sequence_ = 0, generation_ = 1;
};

inline std::string parameter_snapshot(const Parameters& params, bool guidewire) {
    std::ostringstream out;
    out << std::setprecision(17) << "{\"version\":\"" << kVersion
        << "\",\"experimental\":true,\"available\":" << (params.available ? "true" : "false")
        << ",\"application_mode\":\"" << (guidewire ? "guidewire" : "catheter")
        << "\",\"parameter_source\":\"" << parameter_source(guidewire)
        << "\",\"cross_mode_preview\":" << (!guidewire && kCatheterBorrowGuidewire ? "true" : "false")
        << ",\"force_definition\":\"installed_delta_N; same as GET_PROGRAM; no decoupling\""
        << ",\"feature_names\":[\"bias\",\"velocity\",\"abs_velocity\",\"acceleration\",\"jerk\","
           "\"moving_cmd\",\"fixed_cmd\",\"sin_angle\",\"cos_angle\",\"release\",\"return\","
           "\"reclamp\",\"elapsed\",\"elapsed_squared\",\"velocity_acceleration\"]"
        << ",\"tau_s\":0.003,\"max_dt_s\":0.003,\"warmup_s\":0.010,\"calibration\":[";
    for (int i = 0; i < 7; ++i) { if (i) out << ','; out << kCalibration[i]; }
    out << "],\"center\":[";
    for (int i = 0; i < kFeatureCount; ++i) { if (i) out << ','; out << params.center[i]; }
    out << "],\"scale\":[";
    for (int i = 0; i < kFeatureCount; ++i) { if (i) out << ','; out << params.scale[i]; }
    out << "],\"beta\":[";
    for (int i = 0; i < kFeatureCount; ++i) {
        if (i) out << ',';
        out << '[' << params.beta[i][0] << ',' << params.beta[i][1] << ']';
    }
    out << "]}";
    return out.str();
}
} // namespace clampmodel
