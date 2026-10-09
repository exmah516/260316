// 构建脚本从生产源码提取无设备依赖的解析与校验函数，不修改函数体。
#include "pause_host_functions.inc"
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <cstring>

namespace {
std::map<std::string, unsigned long> schema;
std::string failed_symbol;
unsigned setup_writes = 0;
bool config_written = false;
void check(bool ok, const std::string& message) { if (!ok) throw std::runtime_error(message); }
}

CADSComm::CADSComm() = default;
CADSComm::~CADSComm() = default;
bool CADSComm::IsCommOpen() const { return true; }
bool CADSComm::CloseComm() { return true; }
bool CADSComm::SetTimeout(unsigned long) { return true; }
bool CADSComm::OpenCommInsideReadOnly() { return true; }
bool CADSComm::OpenCommReadOnly() { return true; }
bool CADSComm::ReadDeviceState(unsigned short&, unsigned short&) { return true; }
std::string CADSComm::GetLastErrorCopy() const { return "offline pause test"; }
bool CADSComm::ADSRead(const char*, unsigned long, void*) { throw std::runtime_error("Unexpected read"); }
bool CADSComm::ADSReadSum(const char* const*, const unsigned long*, void* const*, unsigned long)
{ throw std::runtime_error("Unexpected read"); }
bool CADSComm::ADSWrite(const char* name, unsigned long size, void* value)
{
    check(std::string(name) == "G.program_test_setup_req" && size == 1, "Unexpected request");
    check(config_written && *static_cast<bool*>(value), "Setup sent before configuration succeeded");
    ++setup_writes;
    return true;
}
bool CADSComm::ADSWriteSum(const char* const* names, const unsigned long* sizes, const void* const* values, unsigned long count)
{
    config_written = false;
    bool ok = true;
    unsigned pause_fields = 0;
    for (unsigned long i = 0; i < count; ++i) {
        std::string name = names[i];
        check(name != "G.program_test_setup_req", "Setup must not share the parameter batch");
        check(schema.count(name) && schema.at(name) == sizes[i], "PLC type/size mismatch: " + name);
        if (name == failed_symbol) ok = false;
        if (name == "G.program_test_forward_pause_enabled") {
            check(*static_cast<const bool*>(values[i]), "Pause enable lost"); ++pause_fields;
        } else if (name == "G.program_test_forward_pause_distance_mm") {
            check(*static_cast<const double*>(values[i]) == 10., "Pause distance lost"); ++pause_fields;
        } else if (name == "G.program_test_forward_pause_duration_ms") {
            check(*static_cast<const std::uint32_t*>(values[i]) == 35000, "Pause duration lost"); ++pause_fields;
        }
    }
    check(pause_fields == 3, "Missing pause fields");
    config_written = ok;
    return ok;
}

int main(int argc, char** argv)
{
    try {
        check(argc == 2, "Expected PLC scalar schema");
        std::ifstream input(argv[1]);
        std::string name, error;
        unsigned long size;
        while (input >> name >> size) schema[name] = size;
        ProgrammedDeliveryConfig config;
        const std::string prefix = "PROGRAM_PREPARE|mode=catheter|forward_pause_enabled=1|";
        check(apply_program_fields(prefix + "forward_pause_distance_mm=10|forward_pause_duration_ms=35000", config, error), error);
        check(config.forward_pause_enabled && config.forward_pause_distance_mm == 10. && config.forward_pause_duration_ms == 35000,
            "Pipe configuration mismatch");
        check(validate_program_config(config, error), error);
        for (double distance : {0., -1., 21., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
            auto invalid = config;
            invalid.forward_pause_distance_mm = distance;
            check(!validate_program_config(invalid, error), "Backend accepted invalid distance");
        }
        for (unsigned duration : {0u, 60001u}) {
            auto invalid = config;
            invalid.forward_pause_duration_ms = duration;
            check(!validate_program_config(invalid, error), "Backend accepted invalid duration");
        }
        for (auto mode : {ProgrammedDeliveryMode::Guidewire, ProgrammedDeliveryMode::ExternalValidation}) {
            auto invalid = config;
            invalid.mode = mode;
            check(!validate_program_config(invalid, error), "Backend accepted pause outside catheter mode");
        }
        ProgrammedDeliveryAds ads;
        for (const auto* field : {"G.program_test_forward_pause_enabled", "G.program_test_forward_pause_distance_mm",
                                  "G.program_test_forward_pause_duration_ms"}) {
            failed_symbol = field;
            check(!ads.write_config(config, true) && setup_writes == 0, "Failed config triggered movement");
        }
        failed_symbol.clear();
        check(ads.write_config(config, true) && setup_writes == 1, "Valid config did not trigger setup");
        for (const auto* field : {"forward_pause_enabled=2", "forward_pause_distance_mm=nan", "forward_pause_distance_mm=inf",
                                  "forward_pause_distance_mm=-1", "forward_pause_distance_mm=0", "forward_pause_distance_mm=2x",
                                  "forward_pause_duration_ms=0", "forward_pause_duration_ms=60001", "forward_pause_duration_ms=1.5"})
            check(!apply_program_fields(prefix + field, config, error), "Invalid pause field accepted");
        check(apply_program_fields("PROGRAM_PREPARE|mode=catheter", config, error) && !config.forward_pause_enabled
            && config.forward_pause_distance_mm == 10. && config.forward_pause_duration_ms == 3000, "Legacy command retained pause");
        std::cout << "PASS pause pipe parsing, defaults, ADS types and failure-before-setup\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
