#include "../DualClampAds.h"
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace {
std::mutex plc_mutex;
std::condition_variable changed;
std::uint32_t version = 20261006, session = 0, heartbeat = 0;
std::int32_t status = 1;
std::int16_t gen_state = 7;
bool done = false, pending = false, timeout = true, recover = false;
bool consume = true, reject = false, missing_status = false;
bool block_legacy = false, legacy_entered = false;
unsigned heartbeat_writes = 0;
unsigned long heartbeat_delay_ms = 0;

void check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

template<class Value>
bool copy_value(unsigned long length, void* output, Value value)
{
    check(length == sizeof(value), "ADS type/size mismatch");
    std::memcpy(output, &value, sizeof(value));
    return true;
}
}

CADSComm::CADSComm() : m_PAmsAddr(nullptr), m_adsPort(0), m_bOpen(false), m_timeoutMs(100) {}
CADSComm::~CADSComm() = default;
bool CADSComm::IsCommOpen() const { std::lock_guard<std::recursive_mutex> lock(m_mutex); return m_bOpen; }
bool CADSComm::CloseComm() { std::lock_guard<std::recursive_mutex> lock(m_mutex); m_bOpen = false; return true; }
bool CADSComm::SetTimeout(unsigned long value) { m_timeoutMs = value; return true; }
bool CADSComm::OpenCommInsideReadOnly() { m_bOpen = true; return true; }
bool CADSComm::OpenCommReadOnly() { return OpenCommInsideReadOnly(); }
bool CADSComm::ReadDeviceState(unsigned short& state, unsigned short& device) { state = 5; device = 0; return true; }
std::string CADSComm::GetLastErrorCopy() const { return "mock missing symbol"; }

bool CADSComm::ADSRead(const char* symbol, unsigned long length, void* output)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    std::unique_lock<std::mutex> plc_lock(plc_mutex);
    if (std::strcmp(symbol, "G.program_interface_version") == 0) return copy_value(length, output, version);
    if (std::strcmp(symbol, "G.self_check_done") == 0) return copy_value(length, output, done);
    if (std::strcmp(symbol, "G.selfcheck_status") == 0) return !missing_status && copy_value(length, output, status);
    if (std::strcmp(symbol, "G.gen_state") == 0) return copy_value(length, output, gen_state);
    if (std::strcmp(symbol, "G.selfcheck_start_req") == 0) return copy_value(length, output, pending);
    if (std::strcmp(symbol, "G.host_comm_timeout") == 0) return copy_value(length, output, timeout);
    if (std::strcmp(symbol, "G.host_recover_req") == 0) return copy_value(length, output, recover);
    if (std::strcmp(symbol, "G.host_session_id") == 0) return copy_value(length, output, session);
    if (std::strcmp(symbol, "G.host_heartbeat_sequence") == 0) return copy_value(length, output, heartbeat);
    if (std::strncmp(symbol, "G.dual_clamp_", 13) == 0) {
        legacy_entered = true;
        changed.notify_all();
        changed.wait(plc_lock, [] { return !block_legacy; });
        return false;
    }
    std::memset(output, 0, length);
    return true;
}

bool CADSComm::ADSReadSum(const char* const* symbols, const unsigned long* lengths, void* const* outputs, unsigned long count)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    for (unsigned index = 0; index < count; ++index)
        if (!ADSRead(symbols[index], lengths[index], outputs[index])) return false;
    return true;
}

bool CADSComm::ADSWrite(const char* symbol, unsigned long length, void* input)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    std::lock_guard<std::mutex> plc_lock(plc_mutex);
    if (std::strcmp(symbol, "G.host_session_id") == 0) return copy_value(length, &session, *static_cast<std::uint32_t*>(input));
    if (std::strcmp(symbol, "G.host_heartbeat_sequence") == 0) {
        check(length == 4, "Heartbeat must be UDINT");
        if (heartbeat_delay_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                heartbeat_delay_ms < m_timeoutMs ? heartbeat_delay_ms : m_timeoutMs));
            if (heartbeat_delay_ms > m_timeoutMs) return false;
        }
        heartbeat = *static_cast<std::uint32_t*>(input);
        ++heartbeat_writes;
        changed.notify_all();
        return true;
    }
    if (std::strcmp(symbol, "G.host_recover_req") == 0) return copy_value(length, &recover, *static_cast<bool*>(input));
    if (std::strcmp(symbol, "G.selfcheck_start_req") == 0) {
        check(length == 1, "Start request must be BOOL");
        pending = *static_cast<bool*>(input);
        if (pending && consume) { pending = false; status = reject ? 3 : 2; }
        return true;
    }
    throw std::runtime_error("Unexpected ADS write");
}

bool CADSComm::ADSWriteSum(const char* const*, const unsigned long*, const void* const*, unsigned long)
{
    throw std::runtime_error("Unexpected experiment write");
}

int main()
{
    try {
        DualClampAds ads;
        version = 0;
        check(!ads.open() && !ads.is_open(), "Wrong interface version left connection open");
        check(heartbeat_writes == 0, "Version rejection wrote a heartbeat");
        version = 20261006;
        missing_status = true;
        check(!ads.open() && !ads.is_open(), "Missing selfcheck symbol left connection open");
        missing_status = false;
        check(ads.open(), "Host connection depends on legacy symbols");
        check(ads.self_check().valid && ads.self_check().host_timeout, "Waiting snapshot unavailable");
        check(ads.request_self_check(), "Waiting startup incorrectly blocked by host timeout");
        check(!ads.request_self_check(), "Duplicate start accepted during selfcheck");
        {
            std::lock_guard<std::mutex> lock(plc_mutex);
            status = 1; reject = true;
        }
        check(!ads.request_self_check(), "PLC rejection reported as success");
        {
            std::lock_guard<std::mutex> lock(plc_mutex);
            status = 1; reject = false; consume = false;
        }
        check(!ads.request_self_check(), "Unconsumed request reported as success");
        {
            std::lock_guard<std::mutex> lock(plc_mutex);
            check(!pending, "Unconsumed request was not withdrawn");
            block_legacy = true;
        }
        DualClampLiveFrame frame;
        frame.valid = true;
        bool legacy_result = true;
        std::thread blocked_reader([&] { legacy_result = ads.read_live(frame); });
        bool entered = false, heartbeat_progress = false;
        {
            std::unique_lock<std::mutex> lock(plc_mutex);
            entered = changed.wait_for(lock, std::chrono::seconds(1), [] { return legacy_entered; });
            const unsigned before = heartbeat_writes;
            heartbeat_progress = changed.wait_for(lock, std::chrono::milliseconds(250), [&] { return heartbeat_writes >= before + 3; });
            block_legacy = false;
            changed.notify_all();
        }
        blocked_reader.join();
        check(entered && heartbeat_progress, "Blocked legacy read starved independent heartbeat");
        check(!legacy_result && !frame.valid, "Failed legacy frame remained valid");
        ads.close();
        check(!ads.self_check().valid && !ads.is_open(), "Closed session retained valid state");
        status = 1; gen_state = 6; consume = true;
        check(ads.open(), "Connection failed outside selfcheck");
        check(!ads.request_self_check(), "Start accepted outside gen_state=7");
        ads.close();
        status = 2; gen_state = 9;
        check(!ads.open(), "PLC error state treated as running selfcheck");
        status = 4; done = false; gen_state = 6;
        check(!ads.open(), "Inconsistent completed state accepted");
        done = true;
        check(ads.open(), "Completed PLC could not establish host session");
        {
            std::unique_lock<std::mutex> lock(plc_mutex);
            heartbeat_delay_ms = 30;
            const unsigned before = heartbeat_writes;
            check(changed.wait_for(lock, std::chrono::seconds(1), [&] { return heartbeat_writes >= before + 3; }),
                "30ms ADS response after selfcheck incorrectly terminated heartbeat");
        }
        check(ads.is_open() && ads.self_check().valid, "Delayed response invalidated completed selfcheck");
        {
            std::lock_guard<std::mutex> lock(plc_mutex);
            heartbeat_delay_ms = 150;
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (ads.is_open() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        check(!ads.is_open() && !ads.self_check().valid, "Genuine ADS timeout did not invalidate session");
        unsigned stopped_writes;
        {
            std::lock_guard<std::mutex> lock(plc_mutex);
            stopped_writes = heartbeat_writes;
            heartbeat_delay_ms = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        {
            std::lock_guard<std::mutex> lock(plc_mutex);
            check(heartbeat_writes == stopped_writes, "Failed session resumed automatically");
        }
        ads.close();
        std::cout << "PASS offline host/selfcheck contract, consumption, rejection, cancellation and independent heartbeat\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
