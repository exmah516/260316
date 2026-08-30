#pragma once

#include <array>

namespace forcecal
{
// 两级标定和串扰解耦的统一常量，所有实时显示与CSV记录均使用这里的定义。
inline constexpr double kFn1SensorSlopeNPerCount = 0.000614437208097;
inline constexpr double kFt1SensorSlopeNPerCount = 0.000703683250522;
inline constexpr double kFn2SensorSlopeNPerCount = 0.001134444473;
inline constexpr double kFt2SensorSlopeNPerCount = 0.000133373639;
inline constexpr double kInstallationAxialGain = 1.913504;
inline constexpr double kInstallationAxialBiasN = 0.183108;
inline constexpr double kInstallationTorqueGainNmmPerN = 18.440851;
inline constexpr double kInstallationTorqueBiasNmm = 0.020906;
inline constexpr double kTangentialArmMm = 37.0;
inline constexpr double kDecouplingFf = 0.996063;
inline constexpr double kDecouplingFt = 0.037854;
inline constexpr double kDecouplingTf = -0.103597;
inline constexpr double kDecouplingTt = 0.996063;

struct SideResult
{
	double sensor_force_n = 0.0;
	double sensor_tangential_n = 0.0;
	double force_cal_delta_n = 0.0;
	double force_cal_abs_n = 0.0;
	double ft_cal_delta_n = 0.0;
	double ft_cal_abs_n = 0.0;
	double torque_cal_delta_nmm = 0.0;
	double torque_cal_abs_nmm = 0.0;
	double force_decoupled_delta_n = 0.0;
	double torque_decoupled_delta_nmm = 0.0;
	double force_decoupled_abs_n = 0.0;
	double torque_decoupled_abs_nmm = 0.0;
};

struct Result
{
	bool valid = false;
	SideResult side1;
	SideResult side2;
};

Result calculate(short fn1, short ft1, short fn2, short ft2,
	const std::array<double, 4>& zero, bool zero_valid);
}
