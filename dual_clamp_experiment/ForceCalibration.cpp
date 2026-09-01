#include "ForceCalibration.h"

namespace forcecal
{
namespace
{
	SideResult calculate_side(short fn, short ft, double fn_zero, double ft_zero, double fn_slope, double ft_slope)
	{
		SideResult result;
		result.sensor_force_n = fn_slope * (static_cast<double>(fn) - fn_zero);
		result.sensor_tangential_n = ft_slope * (static_cast<double>(ft) - ft_zero);
		result.force_cal_delta_n = kInstallationAxialGain * result.sensor_force_n;
		result.force_cal_abs_n = result.force_cal_delta_n + kInstallationAxialBiasN;
		result.torque_cal_delta_nmm = kInstallationTorqueGainNmmPerN * result.sensor_tangential_n;
		result.torque_cal_abs_nmm = result.torque_cal_delta_nmm + kInstallationTorqueBiasNmm;
		result.ft_cal_delta_n = result.torque_cal_delta_nmm / kTangentialArmMm;
		result.ft_cal_abs_n = result.torque_cal_abs_nmm / kTangentialArmMm;
		result.force_decoupled_delta_n = kDecouplingFf * result.force_cal_delta_n + kDecouplingFt * result.torque_cal_delta_nmm;
		result.torque_decoupled_delta_nmm = kDecouplingTf * result.force_cal_delta_n + kDecouplingTt * result.torque_cal_delta_nmm;
		result.force_decoupled_abs_n = kDecouplingFf * result.force_cal_abs_n + kDecouplingFt * result.torque_cal_abs_nmm;
		result.torque_decoupled_abs_nmm = kDecouplingTf * result.force_cal_abs_n + kDecouplingTt * result.torque_cal_abs_nmm;
		return result;
	}
}

Result calculate(short fn1, short ft1, short fn2, short ft2,
	const std::array<double, 4>& zero, bool zero_valid)
{
	Result result;
	if (!zero_valid) return result;
	result.valid = true;
	result.side1 = calculate_side(fn1, ft1, zero[0], zero[1], kFn1SensorSlopeNPerCount, kFt1SensorSlopeNPerCount);
	result.side2 = calculate_side(fn2, ft2, zero[2], zero[3], kFn2SensorSlopeNPerCount, kFt2SensorSlopeNPerCount);
	return result;
}
}
