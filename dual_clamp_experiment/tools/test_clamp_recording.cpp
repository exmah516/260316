#include "../ExperimentStreamRecorder.h"
#include "../ClampCurveBuffer.h"
#include "../ClampDynamics.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

void check(bool result, const std::string& message) {
    if (!result) throw std::runtime_error(message);
}
int main() {
    try {
        for (auto mode : {ProgrammedDeliveryMode::Catheter, ProgrammedDeliveryMode::Guidewire})
        for (bool validation : {false, true})
        for (double sign : {1.0, -1.0}) {
            ExperimentStreamRecorder recorder;
            std::string error;
            const auto name = mode == ProgrammedDeliveryMode::Catheter ? "catheter" : "guidewire";
            auto params = clampdynamics::config(mode == ProgrammedDeliveryMode::Guidewire);
            params.validation_mode = params.conditions_confirmed = validation;
            params.axial_sign = sign;
            recorder.set_dynamics_config(params);
            check(recorder.begin(name, std::string("OFFLINE_SYNTHETIC_25g_") + name
                + (validation ? "_validation" : "_normal") + (sign > 0 ? "_plus" : "_minus"), error), error);
            ForceZeroState zero;
            zero.valid = zero.done = true;
            std::vector<ProgrammedDeliverySample> samples;
            clampdynamics::Predictor predictor;
            clampdynamics::OperationGate operation;
            clampillustration::Generator illustration;
            for (int i = 0; i < 600; ++i) {
                ProgrammedDeliverySample s;
                s.sample_index = i; s.plc_time_us = i * 1000; s.cycle_index = 1;
                s.phase = i < 100 ? 4 : i < 300 ? 5 : i < 400 ? 6 : i < 500 ? 7 : 9;
                s.axis1_vel = s.axis6_vel = i;
                s.axis1_acc = s.axis6_acc = i < 200 ? 1000.0 : i < 350 ? 0.0 : -1000.0;
                s.fn1 = s.fn2 = 20; s.ft1 = s.ft2 = 10;
                const clampdynamics::Input input{i*.001, double(i), 0, 0, s.phase, 1, s.axis1_acc, true,
                    static_cast<std::uint64_t>(i)};
                const auto r = predictor.update(input, params);
                s.dynamics = r;
                s.model_fn = r.fn_N; s.model_ft = r.ft_N; s.model_valid = r.valid;
                s.model_gate = r.gate; s.model_acceleration = r.acceleration_mm_s2;
                s.model_inertia = r.fn_N; s.model_viscous = 0;
                const auto calibrated = forcecal::calculate(s.fn1, s.ft1, s.fn2, s.ft2, zero.value, zero.valid);
                const auto& side = mode == ProgrammedDeliveryMode::Guidewire ? calibrated.side2 : calibrated.side1;
                s.illustration = illustration.update(i*.001, side.force_cal_delta_n, side.ft_cal_delta_n, operation.update(input));
                samples.push_back(s);
            }
            check(recorder.append_program(samples, 0, mode, zero, error), error);
            check(recorder.finalize("Completed", "OFFLINE SYNTHETIC VERIFICATION ONLY", zero, error), error);
            const auto dir = std::filesystem::u8path(recorder.directory());
            std::ifstream original(dir / "samples_1khz.csv"), derived(dir / "causal_force.csv");
            std::string line;
            int a = 0, b = 0;
            while (std::getline(original, line)) ++a;
            while (std::getline(derived, line)) ++b;
            check(a == 601 && b == 601, "sidecar/original row count mismatch");
            check(std::filesystem::file_size(dir / "causal_model.json") > 100, "missing snapshot");
            std::ifstream sidecar(dir / "model2_illustration.csv");
            int c=0;
            while(std::getline(sidecar,line)) { ++c; check(c==1 || line.find("target_illustration")!=std::string::npos, "missing purpose"); }
            check(c==601, "illustration row count");
            check(std::filesystem::file_size(dir / "model2_illustration.json") > 100, "missing illustration metadata");
            std::cout << dir.u8string() << '\n';
        }
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
