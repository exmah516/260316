#pragma once
#include "ProgrammedDeliveryTypes.h"
#include "ForceCalibration.h"
#include <deque>
#include <iomanip>
#include <ostream>
#include <sstream>

namespace externalvalidation {
inline const char* sync_name(unsigned state)
{
    switch (state) {
    case 1: return "GearInStart";
    case 2: return "InGear";
    case 3: return "GearOutStart";
    case 4: return "GearOutDone";
    default: return "IndependentHold";
    }
}

inline double total_forward(const ProgrammedDeliveryConfig& c)
{
    return c.cycle_count * (c.axis1_prepare_from_left_mm - c.axis1_trigger_from_left_mm)
        + c.final_forward_distance_mm;
}

struct Comparison {
    double fn1 = 0, torque1 = 0, fn6 = 0, torque6 = 0;
    double corrected_fn1 = 0, corrected_torque1 = 0;
    bool valid = false, model_valid = false;
};

inline Comparison compare(const forcecal::Result& cal, const ProgrammedDeliverySample& s)
{
    Comparison r;
    r.valid = cal.valid;
    if (!r.valid) return r;
    r.fn1 = cal.side1.force_decoupled_delta_n;
    r.torque1 = cal.side1.torque_decoupled_delta_nmm;
    r.fn6 = cal.side2.force_decoupled_delta_n;
    r.torque6 = cal.side2.torque_decoupled_delta_nmm;
    r.model_valid = s.model_valid;
    // 惯性试算先在原安装标定层扣除，再应用与两侧实测相同的解耦矩阵。
    const double fn = cal.side1.force_cal_delta_n - (s.model_valid ? s.model_fn : 0.0);
    const double torque = cal.side1.torque_cal_delta_nmm
        - (s.model_valid ? s.model_ft * forcecal::kTangentialArmMm : 0.0);
    r.corrected_fn1 = forcecal::kDecouplingFf * fn + forcecal::kDecouplingFt * torque;
    r.corrected_torque1 = forcecal::kDecouplingTf * fn + forcecal::kDecouplingTt * torque;
    return r;
}

inline void write_header(std::ostream& out)
{
    out << "sample_index,plc_time_us,phase,event_sequence,cycle_index,sync_state,"
        "axis1_pos_mm,axis1_vel_mm_s,axis1_acc_mm_s2,axis2_pos_deg,axis2_vel_deg_s,axis2_acc_deg_s2,"
        "axis6_pos_mm,axis6_vel_mm_s,axis6_acc_mm_s2,axis7_pos_deg,axis7_vel_deg_s,axis7_acc_deg_s2,"
        "cylinder1_cmd,cylinder2_cmd,cylinder3_cmd,cylinder4_cmd,"
        "fn1_raw,ft1_raw,fn2_raw,ft2_raw,fn1_zeroed,ft1_zeroed,fn2_zeroed,ft2_zeroed";
    for (int side : {1, 2}) {
        const auto n = std::to_string(side);
        out << ",fn" << n << "_sensor_N,ft" << n << "_sensor_N,fn" << n << "_cal_delta_N,fn" << n
            << "_cal_abs_N,ft" << n << "_cal_delta_N,ft" << n << "_cal_abs_N,torque" << n
            << "_cal_delta_Nmm,torque" << n << "_cal_abs_Nmm,fn" << n << "_decoupled_delta_N,torque" << n
            << "_decoupled_delta_Nmm,fn" << n << "_decoupled_abs_N,torque" << n << "_decoupled_abs_Nmm";
    }
    out << ",fn1_model_decoupled_N,torque1_model_decoupled_Nmm,force_valid,model_valid\n";
}

inline void write_side(std::ostream& out, const forcecal::SideResult& s)
{
    out << ',' << s.sensor_force_n << ',' << s.sensor_tangential_n
        << ',' << s.force_cal_delta_n << ',' << s.force_cal_abs_n
        << ',' << s.ft_cal_delta_n << ',' << s.ft_cal_abs_n
        << ',' << s.torque_cal_delta_nmm << ',' << s.torque_cal_abs_nmm
        << ',' << s.force_decoupled_delta_n << ',' << s.torque_decoupled_delta_nmm
        << ',' << s.force_decoupled_abs_n << ',' << s.torque_decoupled_abs_nmm;
}

inline void write_sample(std::ostream& out, const ProgrammedDeliverySample& s,
    const std::array<double, 4>& zero, bool zero_valid)
{
    const auto cal = forcecal::calculate(s.fn1, s.ft1, s.fn2, s.ft2, zero, zero_valid);
    const auto c = compare(cal, s);
    out << s.sample_index << ',' << s.plc_time_us << ',' << unsigned(s.phase) << ',' << s.event_sequence
        << ',' << s.cycle_index << ',' << unsigned(s.sync_state)
        << ',' << s.axis1_pos << ',' << s.axis1_vel << ',' << s.axis1_acc
        << ',' << s.axis2_pos << ',' << s.axis2_vel << ',' << s.axis2_acc
        << ',' << s.axis6_pos << ',' << s.axis6_vel << ',' << s.axis6_acc
        << ',' << s.axis7_pos << ',' << s.axis7_vel << ',' << s.axis7_acc
        << ',' << s.cylinder1 << ',' << s.cylinder2 << ',' << s.cylinder3 << ',' << s.cylinder4
        << ',' << s.fn1 << ',' << s.ft1 << ',' << s.fn2 << ',' << s.ft2;
    if (zero_valid)
        out << ',' << s.fn1 - zero[0] << ',' << s.ft1 - zero[1]
            << ',' << s.fn2 - zero[2] << ',' << s.ft2 - zero[3];
    else out << ",,,,";
    if (cal.valid) { write_side(out, cal.side1); write_side(out, cal.side2); }
    else for (int i = 0; i < 24; ++i) out << ',';
    if (c.model_valid) out << ',' << c.corrected_fn1 << ',' << c.corrected_torque1;
    else out << ",,";
    out << ',' << c.valid << ',' << c.model_valid << '\n';
}

inline void write_metadata(std::ostream& out, const ProgrammedDeliveryConfig& c)
{
    out << "  \"external_reference\": {\"axis\":6,\"assumed_accurate\":true,\"accuracy_verified\":false,"
        "\"definition\":\"zeroed_installed_decoupled_delta\",\"force_unit\":\"N\",\"torque_unit\":\"Nmm\","
        "\"model_compensated\":false,\"sign_changed\":false,\"filtered\":false},\n"
        << "  \"external_motion\": {\"master\":1,\"slave\":6,\"gear_ratio\":1,\"axis5_moved\":false,"
        "\"axis7_rotated\":false,\"cylinder3_word\":400,\"cylinder4_clamped_throughout\":true},\n"
        << "  \"axis1_prepare_from_left_mm\": " << c.axis1_prepare_from_left_mm << ",\n"
        << "  \"axis1_trigger_from_left_mm\": " << c.axis1_trigger_from_left_mm << ",\n"
        << "  \"axis6_prepare_from_left_mm\": " << c.axis6_prepare_from_left_mm << ",\n"
        << "  \"axis6_total_forward_mm\": " << total_forward(c) << ",\n"
        << "  \"axis6_expected_end_from_left_mm\": " << c.axis6_prepare_from_left_mm - total_forward(c) << ",\n"
        << "  \"cycle_count\": " << c.cycle_count << ",\n"
        << "  \"final_forward_distance_mm\": " << c.final_forward_distance_mm << ",\n"
        << "  \"release_lead_ms\": " << c.release_lead_ms << ",\n"
        << "  \"release_wait_ms\": " << c.release_wait_ms << ",\n"
        << "  \"reclamp_lead_ms\": " << c.reclamp_lead_ms << ",\n"
        << "  \"reclamp_wait_ms\": " << c.reclamp_wait_ms << ",\n"
        << "  \"forward_velocity_mm_s\": " << c.forward_velocity_mm_s << ",\n"
        << "  \"forward_acceleration_mm_s2\": " << c.forward_acceleration_mm_s2 << ",\n"
        << "  \"forward_deceleration_mm_s2\": " << c.forward_deceleration_mm_s2 << ",\n"
        << "  \"forward_jerk_mm_s3\": " << c.forward_jerk_mm_s3 << ",\n"
        << "  \"return_velocity_mm_s\": " << c.return_velocity_mm_s << ",\n"
        << "  \"return_acceleration_mm_s2\": " << c.return_acceleration_mm_s2 << ",\n"
        << "  \"return_deceleration_mm_s2\": " << c.return_deceleration_mm_s2 << ",\n"
        << "  \"return_jerk_mm_s3\": " << c.return_jerk_mm_s3 << ",\n";
}

struct CurvePoint {
    std::uint64_t sequence = 0;
    double time = 0;
    Comparison value;
    unsigned phase = 0, cycle = 0, sync_state = 0;
};

class CurveBuffer {
public:
    void reset() { points_.clear(); ++generation_; }
    void push(CurvePoint p) {
        p.sequence = ++sequence_;
        points_.push_back(p);
        while (!points_.empty() && (points_.front().time < p.time - 10.0 || points_.size() > 10001))
            points_.pop_front();
    }
    std::string response(std::uint64_t after, std::uint64_t generation, bool calibrated, bool record_ok) const {
        const bool changed = generation != generation_;
        const bool gap = !changed && !points_.empty() && after + 1 < points_.front().sequence;
        std::ostringstream out;
        out << std::setprecision(12) << "PROGRAM_EXTERNAL_CURVES|" << generation_ << "|4|"
            << calibrated << '|' << record_ok << '|' << gap << '|';
        int count = 0;
        for (const auto& p : points_) {
            if (!changed && p.sequence <= after) continue;
            if (count++) out << ';';
            const auto& v = p.value;
            out << p.sequence << ',' << p.time << ',' << v.fn1 << ',' << v.torque1
                << ',' << v.fn6 << ',' << v.torque6 << ',' << v.corrected_fn1 << ',' << v.corrected_torque1
                << ',' << v.valid << ',' << v.model_valid << ',' << p.phase << ',' << p.cycle << ',' << p.sync_state;
            if (count >= 1024) break;
        }
        return out.str();
    }
private:
    std::deque<CurvePoint> points_;
    std::uint64_t sequence_ = 0, generation_ = 1;
};
}
