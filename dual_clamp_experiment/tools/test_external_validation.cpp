#include "../ExternalValidation.h"
#include "../ExternalHandleMotion.h"
#include "../ExperimentStreamRecorder.h"
#include "../ExperimentStreamAds.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
void require_close(double a, double b) { check(std::abs(a - b) < 1e-9, "Numeric mismatch"); }
std::vector<std::string> fields(const std::string& line)
{
    std::vector<std::string> values;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) values.push_back(field);
    if (!line.empty() && line.back() == ',') values.emplace_back();
    return values;
}

void test_handle_mapping()
{
    const std::array<double, 2> anchor{1., 2.};
    const std::array<double, 7> actual{123., 17., 83., 84., 85., 1151., 17.};
    const std::array<double, 7> initial{30., 9., 0., 0., 0., 30., 13.};
    std::array<double, 7> refer{};
    check(externalhandle::map(anchor, anchor, actual, initial, 123., 103., refer), "Initial handle mapping");
    for (std::size_t axis = 0; axis < refer.size(); ++axis)
        require_close(refer[axis] + initial[axis], actual[axis]);
    check(externalhandle::map({1.01, 2.5}, anchor, actual, initial, 123., 103., refer), "Combined motion");
    require_close(refer[0] + initial[0], 115.5);
    require_close(refer[5] + initial[5], 1143.5);
    require_close(refer[1] + initial[1], 17. - 90. / 3.14159265358979323846);
    require_close(refer[6] + initial[6], refer[1] + initial[1]);
    require_close(refer[4] + initial[4], 85.);
    check(externalhandle::map({0.5, 2.}, anchor, actual, initial, 123., 103., refer), "Reverse input");
    require_close(refer[0] + initial[0], 123.);
    require_close(refer[5] + initial[5], 1151.);
    check(externalhandle::map({2., 2.}, anchor, actual, initial, 123., 103., refer), "Forward limit");
    require_close(refer[0] + initial[0], 103.);
    require_close(refer[4] + initial[4], 85.);
    require_close(refer[5] + initial[5], 1131.);
    const std::array<double, 7> next_actual{123., -21., 83., 84., 85., 1131., -21.};
    const std::array<double, 2> next_anchor{2., -1.};
    check(externalhandle::map(next_anchor, next_anchor, next_actual, initial, 123., 103., refer), "Rebase");
    for (std::size_t axis = 0; axis < refer.size(); ++axis)
        require_close(refer[axis] + initial[axis], next_actual[axis]);
    check(!externalhandle::map(anchor, anchor, actual, initial, 103., 123., refer), "Invalid bounds accepted");
    check(!externalhandle::map({std::numeric_limits<double>::quiet_NaN(), 2.}, anchor,
        actual, initial, 123., 103., refer), "Non-finite input accepted");
}

void test_handle_boundary_absorption()
{
    std::array<double, 2> anchor{1.0, 2.0};
    std::array<double, 7> axis_anchor{123.0, 17.0, 83.0, 84.0, 85.0, 1151.0, 17.0};
    const std::array<double, 7> initial{30.0, 9.0, 0.0, 0.0, 0.0, 30.0, 13.0};
    std::array<double, 7> refer{};
    check(externalhandle::absorb_axis1_boundary_overtravel(
        {2.0, 2.0}, anchor, axis_anchor, 103.0, 1131.0, 123.0, 103.0),
        "Boundary overtravel must be absorbed");
    check(anchor[0] == 2.0 && axis_anchor[0] == 103.0 && axis_anchor[5] == 1131.0,
        "Boundary baseline must follow actual position");
    check(externalhandle::map({2.0, 2.0}, anchor, axis_anchor, initial, 123.0, 103.0, refer),
        "Boundary remap");
    require_close(refer[0] + initial[0], 103.0);
    require_close(refer[5] + initial[5], 1131.0);
    check(externalhandle::map({1.99, 2.0}, anchor, axis_anchor, initial, 123.0, 103.0, refer),
        "Reverse after boundary");
    check(refer[0] + initial[0] > 103.0 && refer[0] + initial[0] < 111.0,
        "Reverse must resume continuously from boundary");

    anchor = {1.0, 2.0};
    axis_anchor = {103.0, 17.0, 83.0, 84.0, 85.0, 1131.0, 17.0};
    check(externalhandle::absorb_axis1_boundary_overtravel(
        {0.0, 2.0}, anchor, axis_anchor, 123.0, 1151.0, 123.0, 103.0),
        "Upper boundary overtravel must be absorbed");
    check(externalhandle::map({0.0, 2.0}, anchor, axis_anchor, initial, 123.0, 103.0, refer),
        "Upper boundary remap");
    require_close(refer[0] + initial[0], 123.0);
    check(externalhandle::map({0.01, 2.0}, anchor, axis_anchor, initial, 123.0, 103.0, refer),
        "Reverse after upper boundary");
    check(refer[0] + initial[0] < 123.0 && refer[0] + initial[0] > 115.0,
        "Upper reverse must resume continuously");
    check(!externalhandle::absorb_axis1_boundary_overtravel(
        {0.0, 2.0}, anchor, axis_anchor, std::numeric_limits<double>::quiet_NaN(), 1151.0,
        123.0, 103.0), "Invalid actual feedback must not rebase");
}
}

int main(int argc, char** argv)
{
    try {
        test_handle_mapping();
        test_handle_boundary_absorption();
        check(argc == 3, "Expected PLC trace and output directory");
        const std::filesystem::path output = std::filesystem::u8path(argv[2]);
        std::filesystem::create_directories(output);
        std::ifstream trace(std::filesystem::u8path(argv[1]));
        check(bool(trace), "Cannot read PLC trace");
        std::string line;
        std::getline(trace, line);
        const auto header = fields(line);
        std::map<std::string, std::size_t> index;
        for (std::size_t i = 0; i < header.size(); ++i) index[header[i]] = i;
        ProgrammedDeliveryConfig config;
        config.mode = ProgrammedDeliveryMode::ExternalValidation;
        config.cycle_count = 3;
        config.cylinder1_coupling_enabled = true;
        config.cylinder3_coupling_enabled = false;
        require_close(externalvalidation::total_forward(config), 60.);
        check(std::string(programmed_delivery_mode_name(config.mode)) == "external_validation", "Mode name");
        check(is_catheter_motion(config.mode) && !is_catheter_motion(ProgrammedDeliveryMode::Guidewire), "Axis family");
        ForceZeroState zero;
        zero.valid = zero.done = true; zero.sample_count = 1000;
        zero.value = {10., -8., 30., -4.};
        clampdynamics::Predictor predictor;
        std::vector<ProgrammedDeliverySample> samples;
        while (std::getline(trace, line)) {
            const auto row = fields(line);
            const auto number = [&](const std::string& name) { return std::stod(row.at(index.at(name))); };
            ProgrammedDeliverySample s;
            s.sample_index = static_cast<std::uint32_t>(number("sample_index"));
            s.plc_time_us = static_cast<std::uint64_t>(number("plc_time_us"));
            s.phase = static_cast<std::uint8_t>(number("phase"));
            s.sync_state = static_cast<std::uint8_t>(number("sync_state"));
            s.cycle_index = static_cast<std::uint16_t>(number("cycle_index"));
            s.event_sequence = static_cast<std::uint32_t>(number("event_sequence"));
            s.axis1_pos = number("axis1_pos"); s.axis1_vel = number("axis1_vel"); s.axis1_acc = number("axis1_acc");
            s.axis2_pos = number("axis2_pos"); s.axis2_vel = number("axis2_vel"); s.axis2_acc = number("axis2_acc");
            s.axis5_pos = number("axis5_pos"); s.axis5_vel = number("axis5_vel"); s.axis5_acc = number("axis5_acc");
            s.axis6_pos = number("axis6_pos"); s.axis6_vel = number("axis6_vel"); s.axis6_acc = number("axis6_acc");
            s.axis7_pos = number("axis7_pos"); s.axis7_vel = number("axis7_vel"); s.axis7_acc = number("axis7_acc");
            s.cylinder1 = static_cast<std::uint16_t>(number("cylinder1"));
            s.cylinder2 = static_cast<std::uint16_t>(number("cylinder2"));
            s.cylinder3 = static_cast<std::uint16_t>(number("cylinder3"));
            s.cylinder4 = static_cast<std::uint16_t>(number("cylinder4"));
            const double t = s.plc_time_us * 1e-6;
            s.fn1 = static_cast<short>(200 + 30 * std::sin(t * 12) + (s.phase == 6 ? 180 : 0));
            s.ft1 = static_cast<short>(-60 + 12 * std::cos(t * 9));
            s.fn2 = static_cast<short>(500 + 25 * std::sin(t * 10) + (s.phase == 6 ? 28 : 0));
            s.ft2 = static_cast<short>(150 + 35 * std::cos(t * 8));
            const auto cal = forcecal::calculate(s.fn1, s.ft1, s.fn2, s.ft2, zero.value, true);
            const clampdynamics::Input input{t, s.axis1_vel, double(s.cylinder2), double(s.cylinder1),
                s.phase, s.cycle_index, s.axis1_acc, true, s.sample_index};
            s.dynamics = predictor.update(input, config.dynamics);
            s.model_valid = s.dynamics.valid; s.model_fn = s.dynamics.fn_N; s.model_ft = s.dynamics.ft_N;
            s.model_gate = s.dynamics.gate; s.model_acceleration = s.axis1_acc; s.model_inertia = s.model_fn;
            const auto comparison = externalvalidation::compare(cal, s);
            require_close(comparison.fn6, cal.side2.force_decoupled_delta_n);
            require_close(comparison.torque6, cal.side2.torque_decoupled_delta_nmm);
            require_close(comparison.corrected_fn1, comparison.fn1 - forcecal::kDecouplingFf * s.model_fn);
            require_close(comparison.corrected_torque1, comparison.torque1 - forcecal::kDecouplingTf * s.model_fn);
            samples.push_back(s);
        }
        check(samples.size() > 1024 && samples.size() % 512 != 0, "Need partial final block fixture");
        ExperimentStreamRecorder recorder;
        ProgrammedDeliveryLiveFrame reference;
        reference.valid = reference.leftlimit_valid = true;
        recorder.set_program_context(config, reference);
        recorder.set_dynamics_config(config.dynamics);
        recorder.set_program_coupling(true, false);
        recorder.set_program_cylinder_words(5, 600, 5, 500);
        std::string error;
        check(recorder.begin("external_validation", "OFFLINE_EXTERNAL_VALIDATION_SYNTHETIC", error), error.c_str());
        std::ostringstream manual;
        manual << std::setprecision(12);
        externalvalidation::write_header(manual);
        for (const auto& s : samples) externalvalidation::write_sample(manual, s, zero.value, true);
        for (std::size_t begin = 0; begin < samples.size(); begin += 512) {
            const auto finish = std::min(begin + 512, samples.size());
            const std::vector<ProgrammedDeliverySample> block(samples.begin() + begin, samples.begin() + finish);
            check(recorder.append_program(block, 0, config.mode, zero, error), error.c_str());
        }
        check(recorder.finalize("Completed", "", zero, error), error.c_str());
        const auto directory = std::filesystem::u8path(recorder.directory());
        std::ifstream recorded(directory / "samples_1khz.csv");
        std::ostringstream actual;
        while (std::getline(recorded, line)) actual << line << '\n';
        check(actual.str() == manual.str(), "Streaming and manual CSV differ");
        check(!std::filesystem::exists(directory / "model2_illustration.csv"), "External illustration must not be generated");
        std::ofstream(output / "manual_samples_1khz.csv") << manual.str();

        // 不同通道输入、未取零和模型无效不得被伪装成有效参考。
        std::ostringstream empty;
        externalvalidation::write_sample(empty, samples.front(), zero.value, false);
        auto invalid = fields(empty.str().substr(0, empty.str().size() - 1));
        std::istringstream header_stream(manual.str());
        std::getline(header_stream, line);
        const auto sample_header = fields(line);
        check(invalid.size() == sample_header.size(), "Invalid row/header width");
        const auto column = [&](const std::string& name) {
            const auto found = std::find(sample_header.begin(), sample_header.end(), name);
            check(found != sample_header.end(), "Missing physical column");
            return static_cast<std::size_t>(found - sample_header.begin());
        };
        check(invalid[column("fn1_cal_delta_N")].empty() && invalid[column("fn2_cal_delta_N")].empty()
            && invalid[column("force_valid")] == "0" && invalid[column("model_valid")] == "0", "Invalid physical columns");
        auto s = samples.front();
        s.model_valid = false;
        auto cal = forcecal::calculate(s.fn1, s.ft1, s.fn2, s.ft2, zero.value, true);
        auto c = externalvalidation::compare(cal, s);
        check(!c.model_valid && c.valid, "Separate model and force validity");
        require_close(c.corrected_fn1, c.fn1);

        externalvalidation::CurveBuffer curves;
        for (const auto& point : samples) {
            auto value = forcecal::calculate(point.fn1, point.ft1, point.fn2, point.ft2, zero.value, true);
            curves.push({0, point.plc_time_us * 1e-6, externalvalidation::compare(value, point),
                point.phase, point.cycle_index, point.sync_state});
        }
        const auto first = curves.response(0, 0, true, true);
        check(first.rfind("PROGRAM_EXTERNAL_CURVES|1|4|1|1|0|", 0) == 0, "Curve envelope");
        check(std::count(first.begin(), first.end(), ';') == 1023, "Curve pagination");
        const auto next = curves.response(1024, 1, true, true);
        check(next.find("|1025,") != std::string::npos, "Curve cursor");
        curves.reset();
        check(curves.response(0, 0, false, true) == "PROGRAM_EXTERNAL_CURVES|2|4|0|1|0|", "Curve reset/zero");
        for (unsigned i = 0; i < 11000; ++i) curves.push({0, i*.001, {}, 6, 1, 0});
        check(curves.response(1, 2, true, true).find("|1|1|1|") != std::string::npos, "Curve gap detection");

        std::ofstream report(output / "record_tests.json");
        report << "{\"mode\":\"external_validation\",\"sample_count\":" << samples.size()
            << ",\"record_directory\":\"" << directory.generic_u8string()
            << "\",\"stream_manual_parity\":true,\"reference_unchanged\":true,\"hardware_connected\":false}\n";
        ExperimentStreamRecorder standalone;
        const auto mask = static_cast<std::uint64_t>(StandaloneRecordField::Fn1Raw)
            | static_cast<std::uint64_t>(StandaloneRecordField::Fn2Raw);
        check(standalone.begin_standalone("OFFLINE_EXTERNAL_REGRESSION_STANDALONE", mask, error), error.c_str());
        ExperimentStreamSample raw;
        raw.fn1 = 113; raw.fn2 = 207;
        check(standalone.append_standalone({raw}, zero, mask, error), error.c_str());
        check(standalone.finalize("Completed", "", zero, error), error.c_str());
        std::ifstream independent(std::filesystem::u8path(standalone.directory()) / "samples_1khz.csv");
        std::getline(independent, line);
        check(line.find("sync_state") == std::string::npos, "Standalone schema changed");
        std::getline(independent, line);
        check(line.find(",113,207") != std::string::npos, "Standalone channel mapping");
        ExperimentStreamRecorder legacy;
        check(legacy.begin("legacy", "OFFLINE_EXTERNAL_REGRESSION_LEGACY", error), error.c_str());
        DualClampSample dual;
        dual.fn_1_raw = 113; dual.fn_2_raw = 207;
        check(legacy.append_dual({dual}, 0, zero, error), error.c_str());
        check(legacy.finalize("Completed", "", zero, error), error.c_str());
        std::ifstream dual_file(std::filesystem::u8path(legacy.directory()) / "samples_1khz.csv");
        std::getline(dual_file, line);
        check(line.find("sync_state") == std::string::npos, "Legacy schema changed");
        std::cout << "PASS: handle mapping, external calibration, recording, manual parity, invalid data, curve cursor\n"
            << directory.u8string() << '\n';
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
