#include "arm_manual_ads_service.h"
#include "ads_communication.h"
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
std::mutex fake_mutex;
std::array<double, 5> actual{{100, 90, 180, 180, 0}};
ArmCurve staged_curve;
std::uint32_t acknowledged = 0, plc_state = 0, plc_error = 0;
std::vector<std::string> writes;
std::string fail_symbol;
bool fail_read = false;
bool stage_written = false;
void check(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}
template<typename F> void until(F condition)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!condition()) {
        check(std::chrono::steady_clock::now() < deadline, "Mock ADS wait timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
size_t count(const char* name)
{
    std::lock_guard<std::mutex> lock(fake_mutex);
    return static_cast<size_t>(std::count(writes.begin(), writes.end(), name));
}
} // namespace

// 仅替换测试二进制的传输实现，不链接ADS DLL，不访问网络或真实PLC。
CADSComm::CADSComm() {}
CADSComm::~CADSComm() {}
AdsCommunicationService::AdsCommunicationService(CADSComm& ads) : ads_(ads) {}
AdsCommunicationService::~AdsCommunicationService() {}
AdsCommunicationStats AdsCommunicationService::stats() const
{
    AdsCommunicationStats result;
    result.state = AdsConnectionState::Running;
    return result;
}
AdsEventState AdsCommunicationService::event_state() const { return {}; }
bool AdsCommunicationService::write_sum(const char* const* names, const unsigned long* sizes,
    const void* const* values, unsigned long size, DWORD)
{
    std::lock_guard<std::mutex> lock(fake_mutex);
    for (unsigned long i = 0; i < size; ++i) {
        const std::string name(names[i]);
        writes.push_back(name);
        if (name == fail_symbol) return false;
        if (name == "G.arm_curve_data") {
            check(sizes[i] == sizeof(staged_curve.values), "Wrong curve packet size");
            std::memcpy(staged_curve.values.data(), values[i], sizes[i]);
            stage_written = true;
        }
        if (name == "G.arm_curve_request") {
            check(stage_written, "Commit before curve staging");
            std::memcpy(&acknowledged, values[i], sizeof(acknowledged));
            plc_state = 1;
            plc_error = 0;
            stage_written = false;
        }
        if (name == "G.arm_curve_cancel" && *static_cast<const bool*>(values[i]))
            plc_state = 3;
    }
    return true;
}
bool AdsCommunicationService::read_sum(const char* const* names, const unsigned long* sizes,
    void* const* values, unsigned long size, DWORD)
{
    std::lock_guard<std::mutex> lock(fake_mutex);
    if (fail_read) return false;
    for (unsigned long i = 0; i < size; ++i) {
        const std::string name(names[i]);
        std::memset(values[i], 0, sizes[i]);
        if (name == "G.arm_act_pos") std::memcpy(values[i], actual.data(), sizes[i]);
        else if (name == "G.arm_curve_ack") std::memcpy(values[i], &acknowledged, sizes[i]);
        else if (name == "G.arm_curve_state") std::memcpy(values[i], &plc_state, sizes[i]);
        else if (name == "G.arm_curve_error") std::memcpy(values[i], &plc_error, sizes[i]);
        else if (name == "G.arm_manual_enable" || name == "G.arm_enable_req"
            || (name.find("G.arm_power_output[") == 0 && name.find(".Done") != std::string::npos))
            std::memset(values[i], 1, sizes[i]);
        else if (name == "G.arm_jog_velocity" || name == "G.arm_jog_acc"
            || name == "G.arm_jog_dec" || name == "G.arm_jog_jerk") {
            auto* output = static_cast<double*>(values[i]);
            for (int axis = 0; axis < 5; ++axis) output[axis] = name == "G.arm_jog_velocity" ? 5 : 50;
        }
    }
    return true;
}

int main()
{
    try {
        const std::array<double, 5> velocity{{5,5,5,5,5}}, acceleration{{50,50,50,50,50}};
        ArmCartesianSettings settings{0, 20, 200, 30};
        auto plan = plan_arm_curve(actual, velocity, acceleration, settings, 0, 0);
        check(plan.already_home && plan.error == 0, "Home recognition in PLC units");
        auto q = actual; q[4] = 2;
        plan = plan_arm_curve(q, velocity, acceleration, settings, 0, 0);
        check(plan.error == 0 && plan.curve.values[9] == 0, "Home target uses degrees");
        check(plan.curve.values[5] == 100, "Home changed lift");
        plan = plan_arm_curve(actual, velocity, acceleration, settings, 3, 1);
        check(plan.error == 0 && std::abs(plan.curve.values[5] - 100.1) < 1e-9,
            "Millimeter jog conversion");
        plan = plan_arm_curve(actual, velocity, acceleration, settings, 4, 1);
        check(plan.error == 0 && std::abs(plan.curve.values[9] + 0.1) < 1e-9,
            "Pitch sign conversion");
        check(plan_arm_curve(actual, velocity, acceleration, settings, 1, 1).error == 1104,
            "Folded-zero Cartesian jog not rejected");
        check(plan_arm_curve(actual, velocity, acceleration, {}, 3, 1).error == 1001,
            "Missing calibration not rejected");

        CADSComm transport;
        AdsCommunicationService ads(transport);
        ArmManualAdsService service(ads);
        for (int i = 1; i <= 5; ++i) service.set_axis_enable(i, true);
        service.set_cartesian_parameter(0, 0);
        service.set_cartesian_parameter(1, 20);
        service.set_cartesian_parameter(2, 200);
        service.set_cartesian_parameter(3, 30);
        check(service.start(), "Service start");
        until([&] { return service.snapshot().valid; });
        service.request_program_zero();
        until([&] { return service.snapshot().cartesian_status == 3; });
        check(count("G.arm_curve_request") == 0, "Already-home caused movement");

        service.set_cartesian_jog(3, 1);
        until([&] { return count("G.arm_curve_request") == 1; });
        service.stop_cartesian();
        until([&] { return service.snapshot().cartesian_status == 4; });
        check(count("G.arm_curve_request") == 1, "Stop resubmitted curve");

        { std::lock_guard<std::mutex> lock(fake_mutex); fail_symbol = "G.arm_curve_data"; }
        service.set_cartesian_jog(3, 1);
        until([&] { return service.snapshot().cartesian_error == 1003; });
        const auto failed_stages = count("G.arm_curve_data");
        service.set_cartesian_jog(3, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        check(count("G.arm_curve_data") == failed_stages
            && count("G.arm_curve_request") == 1, "Failed stage retried or committed");
        service.stop_cartesian();
        until([&] { return service.snapshot().cartesian_status == 4; });
        { std::lock_guard<std::mutex> lock(fake_mutex); fail_symbol = "G.arm_curve_request"; }
        service.set_cartesian_jog(3, 1);
        until([&] { return service.snapshot().cartesian_error == 1003; });
        check(count("G.arm_curve_request") == 2, "Commit attempt missing");
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        check(count("G.arm_curve_request") == 2, "Unknown commit retried");

        service.stop_cartesian();
        until([&] { return service.snapshot().cartesian_status == 4; });
        { std::lock_guard<std::mutex> lock(fake_mutex); fail_symbol.clear(); }
        service.set_cartesian_jog(3, 1);
        until([&] { return count("G.arm_curve_request") == 3; });
        until([&] { return service.snapshot().cartesian_error == 1005; });
        check(count("G.arm_curve_request") == 3, "Lease expiry continued movement");
        service.stop_cartesian();
        until([&] { return service.snapshot().cartesian_status == 4; });
        service.set_cartesian_jog(3, 1);
        until([&] { return count("G.arm_curve_request") == 4; });
        { std::lock_guard<std::mutex> lock(fake_mutex); fail_read = true; }
        until([&] { return service.snapshot().cartesian_error == 1004; });
        check(count("G.arm_curve_request") == 4, "Feedback loss continued movement");
        { std::lock_guard<std::mutex> lock(fake_mutex); fail_read = false; }
        service.stop_cartesian();
        until([&] { return service.snapshot().valid && service.snapshot().cartesian_status == 4; });
        service.set_cartesian_jog(3, 1);
        until([&] { return count("G.arm_curve_request") == 5; });
        { std::lock_guard<std::mutex> lock(fake_mutex); plc_state = 4; plc_error = 2008; }
        until([&] { return service.snapshot().cartesian_error == 2008; });
        check(count("G.arm_curve_request") == 5, "PLC error retried motion");
        service.stop_cartesian();
        until([&] { return service.snapshot().cartesian_status == 4; });
        service.set_cartesian_jog(3, 1);
        until([&] { return count("G.arm_curve_request") == 6; });
        const auto cancels_before_shutdown = count("G.arm_curve_cancel");
        service.stop();
        check(count("G.arm_curve_cancel") == cancels_before_shutdown + 1,
            "Service shutdown did not cancel active curve");
        check(count("G.arm_curve_request") == 6, "Shutdown committed another curve");
        std::cout << "PASS: PLC units, home window, staged commit, cancellation, no retries, lease/feedback loss, PLC error, shutdown; no hardware.\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
