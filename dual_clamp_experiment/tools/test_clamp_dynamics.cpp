#include "../ClampDynamics.h"
#include "../ClampIllustration.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void near(double a, double b, const char* message) { check(std::abs(a - b) < 1e-12, message); }

// 回放输入由Python csv解析器规范化；此处只接受无引号的数值及状态字段。
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> fields;
    std::istringstream input(line);
    std::string field;
    while (std::getline(input, field, ',')) {
        if (!field.empty() && field.back() == '\r') field.pop_back();
        fields.push_back(field);
    }
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
    for (const char* name : {"sample_index", "time_s", "velocity_mm_s", "moving_cmd", "fixed_cmd",
         "phase", "cycle", "acceleration_mm_s2", "force_valid", "fn_original_N", "ft_original_N"})
        check(columns.count(name) == 1, "missing replay column");
    std::ofstream output(output_path);
    check(bool(output), "cannot create replay output");
    output << std::setprecision(17)
        << "sample_index,time_s,phase,cycle,fn_original_N,ft_original_N,fn_prediction_N,ft_prediction_N,"
           "fn_corrected_N,ft_corrected_N,model_valid,model_gate,feedback_acceleration_mm_s2,"
           "used_acceleration_m_s2,sensor_prediction_N,display_prediction_N,model_status,reset_reason,"
           "model2_fn_N,model2_ft_N,model2_fn_valid,model2_ft_valid\n";
    clampdynamics::Predictor predictor;
    clampdynamics::OperationGate gate;
    clampillustration::Generator illustration;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        const auto fields = split(line);
        check(fields.size() == headers.size(), "replay row width");
        auto value = [&](const char* name) {
            const auto& text = fields.at(columns.at(name));
            if (text.empty()) return std::numeric_limits<double>::quiet_NaN();
            std::size_t used = 0;
            const double number = std::stod(text, &used);
            check(used == text.size(), "invalid numeric replay cell");
            return number;
        };
        clampdynamics::Input in{value("time_s"), value("velocity_mm_s"),
            value("moving_cmd"), value("fixed_cmd"), int(value("phase")), int(value("cycle")),
            value("acceleration_mm_s2"), value("force_valid") == 1,
            static_cast<std::uint64_t>(value("sample_index"))};
        const double fn = value("fn_original_N"), ft = value("ft_original_N");
        in.force_valid = in.force_valid && std::isfinite(fn) && std::isfinite(ft);
        const auto r = predictor.update(in, cfg);
        clampillustration::Result m2;
        if (in.force_valid) m2 = illustration.update(in.time_s, fn, ft, gate.update(in));
        else { gate.reset(); illustration.reset(); }
        output << in.sample_index << ',' << in.time_s << ',' << in.phase << ',' << in.cycle
            << ',' << fn << ',' << ft << ',' << r.fn_N << ',' << r.ft_N
            << ',' << fn - r.fn_N << ',' << ft << ',' << r.valid << ',' << r.gate
            << ',' << in.acceleration_mm_s2 << ',' << r.acceleration_m_s2
            << ',' << r.sensor_prediction_N << ',' << r.display_prediction_N
            << ',' << r.status << ',' << r.reset_reason
            << ',' << m2.fn << ',' << m2.ft << ',' << m2.valid_fn << ',' << m2.valid_ft << '\n';
    }
    output.flush();
    check(bool(output), "replay write failed");
}
void unit_tests() {
    clampdynamics::Config cfg;
    cfg.installation_gain = 2.85;
    clampdynamics::Predictor p;
    clampdynamics::Input in{0, 500, 0, 400, 6, 1, 1000};
    auto r = p.update(in, cfg);
    check(r.valid && r.gate, "direct feedback must not need warmup");
    near(r.sensor_prediction_N, .025, "sensor SI units");
    near(r.display_prediction_N, .07125, "installed gain");
    near(r.fn_N, .07125, "applied display increment, no intercept");
    near(r.ft_N, 0, "ft unchanged");
    cfg.axial_sign = -1;
    in.time_s += .001;
    r = p.update(in, cfg);
    near(r.fn_N, -.07125, "sign");
    check(std::string(r.reset_reason) == "configuration_changed", "sign reset");
    in.time_s += .001; in.acceleration_mm_s2 = 0; in.velocity_mm_s = 999;
    near(p.update(in, cfg).fn_N, 0, "no velocity drag or fallback");
    in.time_s += .001; in.acceleration_mm_s2 = std::numeric_limits<double>::quiet_NaN();
    check(!p.update(in, cfg).valid, "NaN acceleration");
    in.time_s += .001; in.acceleration_mm_s2 = 1000; in.force_valid = false;
    check(!p.update(in, cfg).valid, "zero invalid");
    in.time_s += .001; in.force_valid = true;
    check(p.update(in, cfg).valid, "recovery");
    check(!p.update(in, cfg).valid, "duplicate timestamp");
    check(!p.update(in, cfg).valid, "repeated duplicate after reset");
    in.time_s += .001; p.update(in, cfg); in.time_s -= .002;
    check(!p.update(in, cfg).valid, "backwards timestamp");
    in.time_s = 1; p.update(in, cfg); in.time_s = 1.1;
    check(!p.update(in, cfg).valid, "time gap");
    in.time_s = 2; in.sample_index = 0; p.update(in, cfg);
    in.time_s += .001; in.sample_index = 2;
    check(!p.update(in, cfg).valid, "sample index gap");
    p.reset("zero_requested"); in.time_s += .001;
    check(std::string(p.update(in, cfg).reset_reason) == "zero_requested", "explicit reset reason");
    in.sample_index = std::numeric_limits<std::uint64_t>::max();
    cfg.validation_mode = true; cfg.conditions_confirmed = false;
    check(!p.update(in, cfg).valid, "validation confirmation");
    cfg.conditions_confirmed = true;
    in.time_s += .001; in.phase = 9;
    check(p.update(in, cfg).gate, "validation includes final forward");
    cfg.mass_kg = 1;
    check(!p.update(in, cfg).valid, "reject whole assembly mass");
    cfg = {}; cfg.installation_gain = 2.85;
    clampdynamics::Predictor full, chunks, prefix, shifted;
    clampdynamics::OperationGate op;
    std::vector<clampdynamics::Result> expected;
    std::vector<clampdynamics::Input> inputs;
    std::vector<double> timings;
    for (int i = 0; i < 1200; ++i) {
        int phase = i < 100 ? 3 : i < 300 ? 4 : i < 400 ? 5 :
            i < 700 ? 6 : i < 800 ? 7 : i < 850 ? 8 : i < 1150 ? 9 : 10;
        clampdynamics::Input x{i*.001, double(i % 30), double(i < 250 ? 600 : 0),
            400, phase, 1, i < 350 ? 1000.0 : i < 650 ? 0.0 : -1000.0, true,
            static_cast<std::uint64_t>(i)};
        inputs.push_back(x);
        const auto begin = std::chrono::steady_clock::now();
        const auto a = full.update(x, cfg);
        timings.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-begin).count());
        expected.push_back(a);
        auto y = x;
        y.moving_cmd = i < 250 ? 123 : 456; y.fixed_cmd = 900;
        near(shifted.update(y, cfg).fn_N, a.fn_N, "command magnitude independence");
        check(a.gate == op.update(x), "normal operation gate parity");
        if (i < 250 || i >= 800) check(!a.gate, "normal phase gate");
        if (i >= 250 && i < 800) check(a.gate, "latched gate");
    }
    for (std::size_t start = 0; start < inputs.size(); start += 512)
        for (std::size_t i = start; i < std::min(start+512, inputs.size()); ++i)
            near(chunks.update(inputs[i], cfg).fn_N, expected[i].fn_N, "block parity");
    for (std::size_t i = 0; i < 500; ++i)
        near(prefix.update(inputs[i], cfg).fn_N, expected[i].fn_N, "prefix causality");
    cfg.validation_mode = cfg.conditions_confirmed = true;
    p.reset();
    for (const auto& x : inputs) {
        const auto a = p.update(x, cfg);
        check(a.valid && a.gate, "validation full record");
        near(a.fn_N, 2.85 * .025 * x.acceleration_mm_s2 * .001, "full-record amplitude");
    }
    std::sort(timings.begin(), timings.end());
    std::cout << std::setprecision(12) << "{\"edge_tests\":\"passed\",\"samples\":1200,\"p95_us\":"
        << timings[timings.size()*95/100] << ",\"max_us\":" << timings.back()
        << ",\"hardware_connected\":false}\n";
}
int main(int argc, char** argv) {
    try {
        if (argc == 1) unit_tests();
        else {
            check(argc == 7 && std::string(argv[1]) == "--replay", "usage: --replay input output gain sign validation");
            clampdynamics::Config cfg;
            cfg.installation_gain = std::stod(argv[4]); cfg.axial_sign = std::stod(argv[5]);
            cfg.validation_mode = std::string(argv[6]) == "1";
            // 离线全程开门仅为计算覆盖测试，不是对历史实验条件的事实确认。
            cfg.conditions_confirmed = cfg.validation_mode;
            check(clampdynamics::valid_config(cfg), "invalid replay configuration");
            replay(argv[2], argv[3], cfg);
        }
    } catch (const std::exception& ex) { std::cerr << ex.what() << '\n'; return 1; }
}
