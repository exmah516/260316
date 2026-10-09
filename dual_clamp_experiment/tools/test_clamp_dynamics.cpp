#include "../ClampDynamics.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> fields;
    std::istringstream input(line);
    std::string field;
    while (std::getline(input, field, ',')) fields.push_back(field);
    return fields;
}

void replay(const char* input_path, const char* output_path, const clampdynamics::Config& cfg) {
    std::ifstream input(input_path);
    check(bool(input), "cannot open normalized replay input");
    std::string line;
    std::getline(input, line);
    const auto headers = split(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < headers.size(); ++i) columns.emplace(headers[i], i);
    for (const char* name : {"sample_index", "time_s", "velocity_mm_s", "moving_cmd",
        "fixed_cmd", "phase", "cycle", "acceleration_mm_s2", "force_valid",
        "fn_original_N", "ft_original_N"})
        check(columns.count(name) == 1, "missing replay column");

    std::ofstream output(output_path);
    check(bool(output), "cannot create replay output");
    output << "sample_index,time_s,phase,cycle,fn_original_N,fn_prediction_N,fn_corrected_N,"
        "model_valid,motion_N,low_frequency_N,transient_N,wavelet_strength,event_type,event_relative_s\n";

    clampdynamics::Predictor predictor;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        const auto fields = split(line);
        check(fields.size() == headers.size(), "replay row width");
        auto value = [&](const char* name) {
            const auto& text = fields.at(columns.at(name));
            if (text.empty()) return std::numeric_limits<double>::quiet_NaN();
            return std::stod(text);
        };
        clampdynamics::Input in{value("time_s"), value("velocity_mm_s"), value("moving_cmd"),
            value("fixed_cmd"), int(value("phase")), int(value("cycle")),
            value("acceleration_mm_s2"), value("force_valid") == 1,
            static_cast<std::uint64_t>(value("sample_index")), value("fn_original_N"),
            value("ft_original_N")};
        in.force_valid = in.force_valid && std::isfinite(in.force_cal_delta_N) &&
            std::isfinite(in.ft_cal_delta_N);
        const auto r = predictor.update(in, cfg);
        output << in.sample_index << ',' << in.time_s << ',' << in.phase << ',' << in.cycle
            << ',' << in.force_cal_delta_N << ',' << r.fn_N << ','
            << in.force_cal_delta_N - r.fn_N << ',' << r.valid << ',' << r.motion_N
            << ',' << r.low_frequency_N << ',' << r.transient_N << ','
            << r.wavelet_strength << ',' << r.event_type << ',' << r.event_relative_s << '\n';
    }
}

void unit_tests() {
    clampdynamics::Config catheter;
    catheter.installation_gain = 1.80;
    clampdynamics::Input in{0.0, 0.0, 600.0, 5.0, 6, 1, 0.0, true,
        std::numeric_limits<std::uint64_t>::max(), 0.5, 0.0};
    clampdynamics::Predictor p;

    auto r = p.update(in, catheter);
    check(r.valid, "first sample must be valid");
    check(r.fn_N == 0.0, "no event must produce no event compensation");

    // Phase 4 and Phase 9 are always force-compensation free.
    in.time_s += 0.001;
    in.phase = 4;
    r = p.update(in, catheter);
    check(r.valid && r.fn_N == 0.0, "phase 4 compensation must be zero");
    in.time_s += 0.001;
    in.phase = 9;
    r = p.update(in, catheter);
    check(r.valid && r.fn_N == 0.0, "phase 9 compensation must be zero");

    // A command edge creates an event; the template is delayed, then becomes nonzero.
    p.reset();
    in = {};
    in.force_valid = true;
    in.acceleration_mm_s2 = 0.0;
    in.moving_cmd = 600.0;
    in.fixed_cmd = 5.0;
    in.phase = 5;
    in.cycle = 1;
    r = p.update(in, catheter);
    check(r.valid, "release baseline");
    in.time_s = 0.001;
    in.moving_cmd = 5.0;
    r = p.update(in, catheter);
    check(r.event_type == static_cast<int>(clampdynamics::EventType::Release),
        "600 to 5 must be release");
    for (int i = 0; i < 220; ++i) {
        in.time_s += 0.001;
        r = p.update(in, catheter);
    }
    check(r.event_type == static_cast<int>(clampdynamics::EventType::Release),
        "release event must remain observable");
    check(std::isfinite(r.transient_N), "release transient must be finite");

    // No command edge means no new event.
    p.reset();
    in = {};
    in.force_valid = true;
    in.acceleration_mm_s2 = 0.0;
    in.moving_cmd = 5.0;
    in.fixed_cmd = 5.0;
    in.phase = 6;
    r = p.update(in, catheter);
    in.time_s = 0.001;
    r = p.update(in, catheter);
    check(r.event_type == 0 && r.transient_N == 0.0, "constant command must not trigger");

    // A constant-clamp experiment stores identical open/close words for the next prepare.
    // Both preparations and the samples between them must accept that configuration.
    for (auto constant : {clampdynamics::kCatheter, clampdynamics::kGuidewire}) {
        for (const auto word : {5, 500}) {
            constant.moving_open_word = constant.moving_close_word = word;
            for (int run = 0; run < 2; ++run) {
                check(clampdynamics::valid_config(constant),
                    "repeated constant-clamp preparation must accept identical open/close words");
                p.reset();
                in = {};
                in.acceleration_mm_s2 = 0.0;
                in.moving_cmd = word;
                in.fixed_cmd = 5.0;
                in.phase = 6;
                for (int sample = 0; sample < 700; ++sample) {
                    in.time_s = sample * 0.001;
                    r = p.update(in, constant);
                    check(r.valid, "constant-clamp model must remain valid");
                    check(r.event_type == 0 && r.transient_N == 0.0,
                        "identical open/close words must not generate fictitious clamp events");
                }
            }
        }
    }

    auto unconfirmed = catheter;
    unconfirmed.validation_mode = true;
    check(!clampdynamics::valid_config(unconfirmed), "validation still requires confirmation");
    unconfirmed.conditions_confirmed = true;
    check(clampdynamics::valid_config(unconfirmed), "confirmed validation must be accepted");

    // Catheter and guidewire use distinct close words.
    auto guidewire = clampdynamics::kGuidewire;
    p.reset();
    in = {};
    in.force_valid = true;
    in.acceleration_mm_s2 = 0.0;
    in.moving_cmd = 500.0;
    in.fixed_cmd = 5.0;
    in.phase = 7;
    r = p.update(in, guidewire);
    in.time_s = 0.001;
    in.moving_cmd = 5.0;
    r = p.update(in, guidewire);
    check(r.event_type == static_cast<int>(clampdynamics::EventType::Release),
        "500 to 5 must be guidewire release");

    // Unsupported inverse reconstruction remains rejected; guidewire is now available.
    catheter.reconstruct_external = true;
    check(!clampdynamics::valid_config(catheter), "inverse reconstruction must remain disabled");
    check(clampdynamics::valid_config(guidewire), "guidewire config must be valid");

    std::cout << "{\"edge_tests\":\"passed\",\"hardware_connected\":false}\n";
}

int main(int argc, char** argv) {
    try {
        if (argc == 1) {
            unit_tests();
            return 0;
        }
        check(argc == 7 && std::string(argv[1]) == "--replay",
            "usage: --replay input output gain sign validation");
        clampdynamics::Config cfg;
        cfg.installation_gain = std::stod(argv[4]);
        cfg.axial_sign = std::stod(argv[5]);
        cfg.validation_mode = std::string(argv[6]) == "1";
        cfg.conditions_confirmed = cfg.validation_mode;
        check(clampdynamics::valid_config(cfg), "invalid replay configuration");
        replay(argv[2], argv[3], cfg);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
