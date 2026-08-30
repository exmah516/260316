#pragma once

#include <array>
#include <cstdint>

enum class StandaloneRecordField : std::uint64_t
{
    Axis1Pos = 1ull << 0,
    Axis1Velocity = 1ull << 1,
    Axis1Acceleration = 1ull << 2,
    Axis2Pos = 1ull << 3,
    Axis2Velocity = 1ull << 4,
    Axis2Acceleration = 1ull << 5,
    Axis5Pos = 1ull << 6,
    Axis5Velocity = 1ull << 7,
    Axis5Acceleration = 1ull << 8,
    Axis6Pos = 1ull << 9,
    Axis6Velocity = 1ull << 10,
    Axis6Acceleration = 1ull << 11,
    Axis7Pos = 1ull << 12,
    Axis7Velocity = 1ull << 13,
    Axis7Acceleration = 1ull << 14,
    Cylinder1 = 1ull << 15,
    Cylinder2 = 1ull << 16,
    Cylinder3 = 1ull << 17,
    Cylinder4 = 1ull << 18,
    Fn1Raw = 1ull << 19,
    Ft1Raw = 1ull << 20,
    Fn2Raw = 1ull << 21,
    Ft2Raw = 1ull << 22,
    Fn1Zeroed = 1ull << 23,
    Ft1Zeroed = 1ull << 24,
    Fn2Zeroed = 1ull << 25,
    Ft2Zeroed = 1ull << 26,
    Fn1Sensor = 1ull << 27,
    Ft1Sensor = 1ull << 28,
    Fn1CalDelta = 1ull << 29,
    Fn1CalAbs = 1ull << 30,
    Ft1CalDelta = 1ull << 31,
    Ft1CalAbs = 1ull << 32,
    Torque1CalDelta = 1ull << 33,
    Torque1CalAbs = 1ull << 34,
    Fn1DecoupledDelta = 1ull << 35,
    Torque1DecoupledDelta = 1ull << 36,
    Fn1DecoupledAbs = 1ull << 37,
    Torque1DecoupledAbs = 1ull << 38,
    Axis2Angle = 1ull << 39,
    Fn2Sensor = 1ull << 40,
    Ft2Sensor = 1ull << 41,
    Fn2CalDelta = 1ull << 42,
    Fn2CalAbs = 1ull << 43,
    Ft2CalDelta = 1ull << 44,
    Ft2CalAbs = 1ull << 45,
    Torque2CalDelta = 1ull << 46,
    Torque2CalAbs = 1ull << 47,
    Fn2DecoupledDelta = 1ull << 48,
    Torque2DecoupledDelta = 1ull << 49,
    Fn2DecoupledAbs = 1ull << 50,
    Torque2DecoupledAbs = 1ull << 51,
    Axis7Angle = 1ull << 52
};

constexpr std::uint64_t standalone_field_bit(StandaloneRecordField field)
{
    return static_cast<std::uint64_t>(field);
}

struct StandaloneRecordLiveFrame
{
    bool valid = false;
    bool selfcheck_done = false;
	std::uint16_t legacy_phase = 0;
	std::uint8_t program_phase = 0;
	double axis2_angle_deg = 0.0;
	double axis7_angle_deg = 0.0;
	std::array<std::uint16_t, 4> cylinder{};
    bool manual_control_enabled = false;
    std::uint32_t manual_error_id = 0;
};
