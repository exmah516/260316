#include "../ExperimentStreamRecorder.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

void check(bool ok, const std::string& message) { if (!ok) throw std::runtime_error(message); }
std::vector<std::string> fields(const std::string& line) {
    std::vector<std::string> result;
    std::istringstream stream(line);
    std::string value;
    while (std::getline(stream, value, ',')) result.push_back(value);
    return result;
}

int main(int argc, char** argv) {
    try {
        check(argc == 2, "Expected production PLC trace");
        std::ifstream input(argv[1]);
        check(input.good(), "Trace not found");
        ProgrammedDeliveryConfig config;
        config.forward_pause_enabled = true;
        config.forward_pause_duration_ms = 35000;
        config.final_forward_distance_mm = 15.;
        ProgrammedDeliveryLiveFrame reference;
        reference.valid = reference.leftlimit_valid = true;
        ExperimentStreamRecorder recorder;
        recorder.set_program_context(config, reference);
        std::string error, line;
        check(recorder.begin("catheter", "OFFLINE_FORWARD_PAUSE_35S", error), error);
        ForceZeroState zero;
        zero.valid = zero.done = true;
        std::vector<ProgrammedDeliverySample> block;
        std::getline(input, line);
        std::size_t count = 0;
        while (std::getline(input, line)) {
            const auto f = fields(line);
            check(f.size() == 12, "Unexpected PLC trace schema");
            ProgrammedDeliverySample s;
            s.sample_index = std::stoul(f[0]); s.plc_time_us = std::stoull(f[1]);
            s.phase = static_cast<std::uint8_t>(std::stoul(f[2])); s.event_sequence = std::stoul(f[3]);
            s.cycle_index = static_cast<std::uint16_t>(std::stoul(f[4]));
            s.axis1_pos = std::stod(f[5]); s.axis1_vel = std::stod(f[6]); s.axis1_acc = std::stod(f[7]);
            s.cylinder1 = static_cast<std::uint16_t>(std::stoul(f[8])); s.cylinder2 = static_cast<std::uint16_t>(std::stoul(f[9]));
            s.fn1 = static_cast<short>(std::stoi(f[10])); s.ft1 = static_cast<short>(std::stoi(f[11]));
            check(s.sample_index == count && s.plc_time_us == count * 1000, "Gap in PLC trace");
            ++count;
            block.push_back(s);
            if (block.size() == 512) {
                check(recorder.append_program(block, 0, config.mode, zero, error), error);
                block.clear();
            }
        }
        if (!block.empty()) check(recorder.append_program(block, 0, config.mode, zero, error), error);
        check(recorder.finalize("Completed", "OFFLINE PLC FORWARD PAUSE TEST", zero, error), error);
        const auto dir = std::filesystem::u8path(recorder.directory());
        std::ifstream saved(dir / "samples_1khz.csv"), metadata(dir / "experiment.json");
        std::getline(saved, line);
        const auto header = fields(line);
        auto column = [&](const char* name) {
            auto it = std::find(header.begin(), header.end(), name);
            check(it != header.end(), std::string("Missing CSV column: ") + name);
            return static_cast<std::size_t>(it - header.begin());
        };
        std::size_t rows = 0, stationary = 0;
        while (std::getline(saved, line)) {
            const auto f = fields(line);
            check(std::stoul(f[0]) == rows && std::stoull(f[1]) == rows * 1000, "Archive lost or reordered samples");
            check(std::stoi(f[column("fn1_raw")]) == 113, "Raw force changed");
            if (std::stod(f[column("axis1_pos_mm")]) == 13. && std::stod(f[column("axis1_vel_mm_s")]) == 0.) ++stationary;
            ++rows;
        }
        check(rows == count && count > 70000 && stationary >= 70000, "Long pause archive incomplete");
        std::string json((std::istreambuf_iterator<char>(metadata)), {});
        for (const auto* field : {"\"forward_pause_enabled\": true", "\"forward_pause_distance_mm\": 10",
                                 "\"forward_pause_duration_ms\": 35000"})
            check(json.find(field) != std::string::npos, "Pause metadata missing");
        std::cout << "PASS complete long-pause archive: " << rows << " samples\n" << dir.u8string() << '\n';
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
