#include "StandaloneRecordAds.h"

#include <chrono>
#include <thread>

namespace
{
    constexpr const char* kSelfcheckDone = "G.self_check_done";
    constexpr const char* kLegacyPhase = "G.dual_clamp_phase";
    constexpr const char* kProgramPhase = "G.program_test_phase";
    constexpr const char* kAxis2ActPos = "G.axis[2].NcToPlc.ActPos";
    constexpr const char* kAxis7ActPos = "G.axis[7].NcToPlc.ActPos";
    constexpr const char* kCylinder1 = "G.cylinder1_value";
    constexpr const char* kCylinder2 = "G.cylinder2_value";
    constexpr const char* kCylinder3 = "G.cylinder3_value";
    constexpr const char* kCylinder4 = "G.cylinder4_value";
    constexpr const char* kManualEnable = "G.manual_cylinder_enable";
    constexpr const char* kManualCmd = "G.manual_cylinder_cmd";
    constexpr const char* kRecordEnable = "G.standalone_record_enable";
    constexpr const char* kRecordEvent = "G.standalone_record_event_seq";
}

StandaloneRecordAds::StandaloneRecordAds() = default;

StandaloneRecordAds::~StandaloneRecordAds()
{
    close();
}

bool StandaloneRecordAds::open()
{
    if (comm_.IsCommOpen()) return true;
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        comm_.SetTimeout(1000);
        if (comm_.OpenCommInsideReadOnly() || comm_.OpenCommReadOnly())
        {
            unsigned short ads_state = 0;
            unsigned short device_state = 0;
            if (comm_.ReadDeviceState(ads_state, device_state))
            {
                comm_.SetTimeout(100);
                return true;
            }
        }
        comm_.CloseComm();
        if (attempt < 2) std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}

void StandaloneRecordAds::close()
{
    if (comm_.IsCommOpen()) comm_.CloseComm();
}

bool StandaloneRecordAds::is_open() const
{
    return comm_.IsCommOpen();
}

std::string StandaloneRecordAds::last_error() const
{
    return comm_.GetLastErrorCopy();
}

bool StandaloneRecordAds::read_live(StandaloneRecordLiveFrame& frame)
{
    bool selfcheck_done = false;
    unsigned short legacy_phase = 0;
    unsigned char program_phase = 0;
    double axis2_angle = 0.0;
    double axis7_angle = 0.0;
    std::array<unsigned short, 4> cylinders{};
    bool manual_enabled = false;
    std::uint32_t manual_error = 0;
    const char* symbols[] = {
        kSelfcheckDone, kLegacyPhase, kProgramPhase, kAxis2ActPos, kAxis7ActPos, kCylinder1, kCylinder2, kCylinder3, kCylinder4,
        kManualEnable, "G.manual_cylinder_error_id"
    };
    const unsigned long lengths[] = {
        sizeof(selfcheck_done), sizeof(legacy_phase), sizeof(program_phase), sizeof(axis2_angle), sizeof(axis7_angle), sizeof(cylinders[0]), sizeof(cylinders[1]),
        sizeof(cylinders[2]), sizeof(cylinders[3]), sizeof(manual_enabled), sizeof(manual_error)
    };
    void* outputs[] = {
        &selfcheck_done, &legacy_phase, &program_phase, &axis2_angle, &axis7_angle, &cylinders[0], &cylinders[1], &cylinders[2], &cylinders[3],
        &manual_enabled, &manual_error
    };
    if (!comm_.ADSReadSum(symbols, lengths, outputs, static_cast<unsigned long>(std::size(symbols)))) return false;
    frame.valid = true;
    frame.selfcheck_done = selfcheck_done;
    frame.legacy_phase = legacy_phase;
    frame.program_phase = program_phase;
    frame.axis2_angle_deg = axis2_angle;
    frame.axis7_angle_deg = axis7_angle;
    frame.cylinder = cylinders;
    frame.manual_control_enabled = manual_enabled;
    frame.manual_error_id = manual_error;
    return true;
}

bool StandaloneRecordAds::set_record_enable(bool enable, std::uint32_t event_sequence)
{
    const bool value = enable;
    const char* symbols[] = { kRecordEnable, kRecordEvent };
    const unsigned long lengths[] = { sizeof(value), sizeof(event_sequence) };
    const void* inputs[] = { &value, &event_sequence };
    return comm_.ADSWriteSum(symbols, lengths, inputs, 2);
}

bool StandaloneRecordAds::set_manual_cylinder(bool enable, const std::array<std::uint16_t, 4>& values)
{
    const bool enabled = enable;
    // 整体写入数组，避免不同 TwinCAT 版本对数组元素符号导出名称不一致。
    const char* symbols[] = { "G.manual_cylinder_cmd", kManualEnable };
    const unsigned long lengths[] = { static_cast<unsigned long>(sizeof(values)), sizeof(enabled) };
    const void* inputs[] = { values.data(), &enabled };
    return comm_.ADSWriteSum(symbols, lengths, inputs, static_cast<unsigned long>(std::size(symbols)));
}
