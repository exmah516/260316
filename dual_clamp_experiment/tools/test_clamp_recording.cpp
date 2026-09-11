#include "../ExperimentStreamRecorder.h"
#include "../ClampCurveBuffer.h"
#include "../ClampDynamics.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <sstream>

void check(bool result, const std::string& message) {
    if (!result) throw std::runtime_error(message);
}
std::vector<std::string> fields(const std::string& line) {
    std::vector<std::string> out;
    std::istringstream stream(line);
    std::string field;
    while(std::getline(stream,field,',')) out.push_back(field);
    if(!line.empty() && line.back()==',') out.emplace_back();
    return out;
}
void pulse_recording_test() {
    for(auto mode:{ProgrammedDeliveryMode::Catheter,ProgrammedDeliveryMode::Guidewire}) {
        ExperimentStreamRecorder recorder;
        ProgrammedDeliveryConfig config;
        config.mode=mode;
        config.return_acceleration_mm_s2=600;
        ProgrammedDeliveryLiveFrame reference;
        reference.valid=reference.leftlimit_valid=true;
        reference.leftlimit_axis1_abs_mm=-90;
        reference.leftlimit_axis6_abs_mm=20;
        recorder.set_program_context(config,reference);
        std::string error;
        check(recorder.begin(mode==ProgrammedDeliveryMode::Catheter?"catheter":"guidewire",
            "OFFLINE_PULSE_POSITION_TEST",error),error);
        ForceZeroState zero;
        zero.done=zero.valid=true;
        zero.value={-1000,-800,-900,-700};
        forcepulse::Guard guard;
        std::vector<ProgrammedDeliverySample> block;
        std::vector<double> append_us;
        const auto begin=std::chrono::steady_clock::now();
        for(int i=0;i<12000;++i) {
            ProgrammedDeliverySample s;
            s.sample_index=i;s.plc_time_us=i*1000ULL;s.phase=6;s.cycle_index=1;
            s.axis1_pos=-75+i*.0001;s.axis2_pos=12;s.axis6_pos=45;s.axis7_pos=34;
            s.position_reference_valid=i!=11999;
            s.axis1_from_left_mm=s.axis1_pos+90;s.axis6_from_left_mm=25;
            s.fn1=-1000;s.ft1=-800;s.fn2=-900;s.ft2=-700;
            const bool pulse=i>=1000&&(i-1000)%2370<60;
            if(pulse) {s.fn1+=130;s.ft1+=110;s.fn2+=130;s.ft2+=110;}
            const bool wire=mode==ProgrammedDeliveryMode::Guidewire;
            s.pulse=guard.update(s.plc_time_us,s.sample_index,wire?s.fn2:s.fn1,wire?s.ft2:s.ft1,true);
            const auto clean=forcecal::calculate(wire?s.fn1:short(s.pulse.fn),
                wire?s.ft1:short(s.pulse.ft),wire?short(s.pulse.fn):s.fn2,
                wire?short(s.pulse.ft):s.ft2,zero.value,true);
            const auto& side=wire?clean.side2:clean.side1;
            s.pulse_fn_N=side.force_cal_delta_n;s.pulse_ft_N=side.ft_cal_delta_n;
            block.push_back(s);
            if(block.size()==512 || i==11999) {
                const auto start=std::chrono::steady_clock::now();
                check(recorder.append_program(block,0,mode,zero,error),error);
                append_us.push_back(std::chrono::duration<double,std::micro>(
                    std::chrono::steady_clock::now()-start).count());
                block.clear();
            }
        }
        check(recorder.finalize("Completed","OFFLINE SYNTHETIC ONLY",zero,error),error);
        const double wall_ms=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-begin).count();
        const auto dir=std::filesystem::u8path(recorder.directory());
        std::ifstream file(dir/"samples_1khz.csv");
        std::string line;
        std::getline(file,line);
        auto header=fields(line);
        const auto col=[&](const char* name){
            auto it=std::find(header.begin(),header.end(),name);
            check(it!=header.end(),std::string("missing column ")+name);
            return std::size_t(it-header.begin());
        };
        unsigned replaced=0,index=0;
        while(std::getline(file,line)) {
            auto f=fields(line);
            check(f.size()==header.size(),"column count");
            check(std::stoul(f[col("sample_index")])==index,"index");
            check(std::stoll(f[col("plc_time_us")])==index*1000LL,"timestamp");
            check(std::stod(f[col("record_axis2_nc_deg")])==12,"axis2 coordinate");
            check(std::stod(f[col("record_axis7_nc_deg")])==34,"axis7 coordinate");
            if(index<11999) check(std::abs(std::stod(f[col("axis1_from_left_mm")])-(15+index*.0001))<1e-8,"left reference");
            else check(f[col("axis1_from_left_mm")].empty()&&f[col("axis6_from_left_mm")].empty(),"invalid reference blank");
            const int original=mode==ProgrammedDeliveryMode::Catheter?-1000:-900;
            const bool pulse=index>=1000&&(index-1000)%2370<60;
            check(std::stoi(f[col(mode==ProgrammedDeliveryMode::Catheter?"fn1_raw":"fn2_raw")])==
                original+(pulse?130:0),"raw channel unchanged");
            if(f[col("pulse_replaced")]=="1") {
                ++replaced;
                check(std::stod(f[col("fn_despiked_N")])==0,"clean force");
                check(std::stoul(f[col("pulse_source_index")])<index,"historical source");
            }
            ++index;
        }
        check(index==12000&&replaced==120,"record replacement count");
        std::ifstream meta(dir/"experiment.json");
        std::string text((std::istreambuf_iterator<char>(meta)),{});
        check(text.find("\"return_acceleration_mm_s2\": 600")!=std::string::npos,"settings saved");
        std::cout<<"PULSE_RECORD "<<dir.u8string()<<" points=12000 total_ms="<<wall_ms
                 <<" append_max_us="<<*std::max_element(append_us.begin(),append_us.end())
                 <<" samples_bytes="<<std::filesystem::file_size(dir/"samples_1khz.csv")<<'\n';
    }
}
int main(int argc,char**argv) {
    try {
        if(argc>1&&std::string(argv[1])=="--pulse-only") {pulse_recording_test();return 0;}
        pulse_recording_test();
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
