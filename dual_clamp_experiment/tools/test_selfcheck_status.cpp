#include "../ProgrammedDeliveryAds.h"
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

namespace {
bool plc_done = true;
std::int32_t plc_status = 1;
std::uint8_t plc_mode = 4;
bool fail_read = false;
std::unordered_map<std::string, unsigned long> symbol_sizes;

void check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

template<typename Value>
bool copy_value(unsigned long length, void* output, Value value)
{
    check(length == sizeof(value), "ADS symbol type/size mismatch");
    std::memcpy(output, &value, sizeof(value));
    return true;
}
}

CADSComm::CADSComm() = default;
CADSComm::~CADSComm() = default;
bool CADSComm::IsCommOpen() const { return true; }
bool CADSComm::CloseComm() { return true; }
bool CADSComm::SetTimeout(unsigned long) { return true; }
bool CADSComm::OpenCommInsideReadOnly() { return true; }
bool CADSComm::OpenCommReadOnly() { return true; }
bool CADSComm::ReadDeviceState(unsigned short&, unsigned short&) { return true; }
std::string CADSComm::GetLastErrorCopy() const { return "offline test"; }
bool CADSComm::ADSWrite(const char*, unsigned long, void*) { throw std::runtime_error("Unexpected write"); }
bool CADSComm::ADSWriteSum(const char* const*, const unsigned long*, const void* const*, unsigned long)
{ throw std::runtime_error("Unexpected write"); }

bool CADSComm::ADSRead(const char* symbol, unsigned long length, void* output)
{
    if (fail_read) return false;
    const auto found = symbol_sizes.find(symbol);
    check(found != symbol_sizes.end(), "ADS symbol missing from target PLC");
    check(found->second == length, "ADS length differs from target PLC declaration");
    if (std::strcmp(symbol, "G.self_check_done") == 0) return copy_value(length, output, plc_done);
    if (std::strcmp(symbol, "G.selfcheck_status") == 0) return copy_value(length, output, plc_status);
    if (std::strcmp(symbol, "G.program_test_mode") == 0) return copy_value(length, output, plc_mode);
    std::memset(output, 0, length);
    return true;
}

bool CADSComm::ADSReadSum(const char* const* symbols, const unsigned long* lengths,
    void* const* outputs, unsigned long count)
{
    for (unsigned long index = 0; index < count; ++index)
        if (!ADSRead(symbols[index], lengths[index], outputs[index])) return false;
    return true;
}

int main(int argc, char** argv)
{
    try {
        check(argc == 2, "Expected target PLC symbol table");
        std::ifstream schema(argv[1]);
        check(schema.good(), "Cannot open target PLC symbol table");
        std::string symbol;
        unsigned long size = 0;
        while (schema >> symbol >> size) symbol_sizes.emplace(symbol, size);
        ProgrammedDeliveryAds ads;
        for (const auto mode : {1, 2, 4}) {
            plc_mode = static_cast<std::uint8_t>(mode);
            ProgrammedDeliveryLiveFrame frame;
            plc_done = true;
            plc_status = 1;
            check(ads.read_live(frame), "Live read failed");
            check(frame.selfcheck_done, "PLC completed but program UI receives unfinished");
            check(frame.leftlimit_valid && !frame.selfcheck_busy, "Completed state is inconsistent");
            for (const auto status : {0, 1, 2, 3}) {
                plc_done = false;
                plc_status = status;
                check(ads.read_live(frame), "Live read failed");
                check(!frame.selfcheck_done && !frame.leftlimit_valid, "Stale completed state");
                check(frame.selfcheck_busy == (status == 2), "Only status 2 means running");
            }
            fail_read = true;
            check(!ads.read_live(frame), "Failed ADS read accepted");
            check(!frame.valid, "Failed ADS read retained stale valid status");
            fail_read = false;
        }
        std::cout << "PASS selfcheck completed/running/waiting/rejected and read failure in modes 1/2/4\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
