#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace forcepulse {
inline constexpr const char* kVersion = "periodic-pulse-causal-v1";
enum class Status : unsigned {
    Pass = 0, Learning = 1, Replaced = 2, Recovered = 3,
    Timeout = 4, Invalid = 5, Reset = 6, UnsupportedShape = 7
};
struct Result {
    double fn = 0, ft = 0; // Sensor counts, before zero and calibration.
    bool valid = false, replaced = false, locked = false;
    std::uint64_t source_index = 0, age_us = 0;
    Status status = Status::Pass;
};

// Fixed storage; no waits, look-ahead, sample revision or heap allocation.
// Thresholds describe the shared count-level pulse signature observed on side 1.
// Each side must independently acquire three recovered, periodic candidates.
class Guard {
    struct Point {
        std::uint64_t time = 0, index = 0;
        double fn = 0, ft = 0;
        bool usable = false;
    };
public:
    void reset() { *this = Guard{}; }
    Result update(std::uint64_t time, std::uint64_t index, double fn, double ft, bool valid) {
        if (!valid || !std::isfinite(fn) || !std::isfinite(ft)) {
            reset();
            return {fn, ft, false, false, false, index, 0, Status::Invalid};
        }
        if (seen_ && time == last_time_ && index == last_index_) return last_;
        bool discontinuity = seen_ && (index != last_index_+1 || time <= last_time_ ||
                                       time-last_time_ > 3000);
        if (discontinuity) reset();
        seen_ = true;
        last_time_ = time; last_index_ = index;
        Result result{fn,ft,true,false,locked_,index,0,
                      discontinuity ? Status::Reset : Status::Pass};
        if (locked_ && time > confirmed_[2] && time-confirmed_[2] > 5100000) {
            locked_ = false; confirmed_count_ = 0;
        }
        if (active_) {
            const double excursion = ft-reference_;
            if (time-onset_ > 100000) {
                active_ = replacing_ = locked_ = false;
                confirmed_count_ = 0;
                result.status = Status::Timeout;
            } else if (std::abs(excursion) <= 8.0) {
                // The first recovered sample passes immediately.
                active_ = replacing_ = false;
                recovery_count_ = 1;
                result.status = Status::Recovered;
            } else if (excursion < -90.0 || excursion > 200.0) {
                active_ = replacing_ = locked_ = false;
                confirmed_count_ = 0;
                result.status = Status::UnsupportedShape;
            } else {
                result.status = replacing_ ? Status::Replaced : Status::Learning;
                if (replacing_) replace(result, time);
            }
        } else if (recovery_count_) {
            if (std::abs(ft-reference_) <= 8.0 && time-onset_ <= 103000) {
                if (++recovery_count_ == 3) {
                    confirm(onset_);
                    recovery_count_ = 0;
                }
            } else {
                recovery_count_ = 0;
            }
        } else if (count_ >= 80) {
            const auto& previous = history_[(head_+127)%128];
            const auto& before = history_[(head_+126)%128];
            const double rise = ft-std::min(previous.ft,before.ft);
            double fn_edge = std::max(std::abs(fn-previous.fn),std::abs(previous.fn-before.fn));
            const auto& earlier = history_[(head_+125)%128];
            fn_edge = std::max(fn_edge,std::abs(before.fn-earlier.fn));
            if (rise >= 35.0 && fn_edge >= 23.0) {
                std::array<double,128> values{};
                unsigned n = 0;
                for (unsigned k=0;k<count_;++k) {
                    const auto& p = history_[(head_+127-k)%128];
                    if (p.usable && time >= p.time+10000 && time <= p.time+100000)
                        values[n++] = p.ft;
                }
                if (n >= 40) {
                    std::sort(values.begin(),values.begin()+n);
                    const double reference = .5*(values[(n-1)/2]+values[n/2]);
                    // A transverse baseline changing strongly is not pulse evidence.
                    const bool stable = values[9*n/10]-values[n/10] <= 10.0;
                    Point source{};
                    bool source_found = false;
                    for (unsigned k=0;k<3;++k) {
                        const auto& p = history_[(head_+127-k)%128];
                        if (p.usable && time-p.time <= 3000 &&
                            std::abs(p.ft-reference) <= 8.0) {
                            source = p; source_found = true; break;
                        }
                    }
                    if (stable && source_found && ft-reference >= 43.0 &&
                        ft-reference <= 180.0) {
                        reference_ = reference;
                        onset_ = time;
                        hold_ = source;
                        active_ = true;
                        replacing_ = predicted(time);
                        result.status = replacing_ ? Status::Replaced : Status::Learning;
                        if (replacing_) replace(result,time);
                    }
                }
            }
        }
        result.locked = locked_;
        history_[head_] = {time,index,fn,ft,!active_};
        head_ = (head_+1)%128;
        count_ = std::min(128u,count_+1);
        last_ = result;
        return result;
    }
private:
    bool predicted(std::uint64_t time) const {
        if (!locked_ || time <= confirmed_[2]) return false;
        const double elapsed = double(time-confirmed_[2]);
        const double cycles = std::round(elapsed/period_us_);
        return cycles >= 1 && cycles <= 2 && std::abs(elapsed-cycles*period_us_) <= 130000;
    }
    void confirm(std::uint64_t onset) {
        if (confirmed_count_ && onset-confirmed_[confirmed_count_-1] < 2200000) return;
        if (confirmed_count_ && onset-confirmed_[confirmed_count_-1] > 2550000) {
            // A missed event cannot train a new period from a doubled interval.
            confirmed_count_ = 0; locked_ = false;
        }
        if (confirmed_count_ == 3) {
            confirmed_[0] = confirmed_[1]; confirmed_[1] = confirmed_[2];
            confirmed_count_ = 2;
        }
        confirmed_[confirmed_count_++] = onset;
        if (confirmed_count_ == 3) {
            const double a = double(confirmed_[1]-confirmed_[0]);
            const double b = double(confirmed_[2]-confirmed_[1]);
            locked_ = std::abs(a-b) <= 130000;
            period_us_ = (a+b)*.5;
        }
    }
    void replace(Result& r, std::uint64_t time) const {
        r.fn = hold_.fn; r.ft = hold_.ft;
        r.replaced = true;
        r.source_index = hold_.index;
        r.age_us = time-hold_.time;
    }
    std::array<Point,128> history_{};
    unsigned head_ = 0, count_ = 0, recovery_count_ = 0, confirmed_count_ = 0;
    std::array<std::uint64_t,3> confirmed_{};
    std::uint64_t last_time_ = 0, last_index_ = 0, onset_ = 0;
    bool seen_ = false, active_ = false, replacing_ = false, locked_ = false;
    double reference_ = 0, period_us_ = 2370000;
    Point hold_{};
    Result last_{};
};
} // namespace forcepulse
