#pragma once
#include <cmath>

namespace force_direct_calibration
{
	// 2026-08-03 实机砝码标定。正常链路以 raw/1000.0 表示电压；系数仅由 ADS 数据验证。
	constexpr double counts_per_volt = 1000.0;
	constexpr double ft_1_slope_n_per_count = 0.000703683250522;
	constexpr double ft_1_intercept_n = 0.017083888668;
	constexpr double fn_1_slope_n_per_count = 0.000614437208097;
	constexpr double fn_1_intercept_n = 0.508483950049;
	constexpr double fn_2_slope_n_per_count = 0.000484209690578;
	constexpr double fn_2_intercept_n = 0.678319980964;
	constexpr double ft_2_slope_n_per_count = 0.00036652860894;
	constexpr double ft_2_intercept_n = -0.0742765932021;
	constexpr double ft_1_slope_n_per_volt = ft_1_slope_n_per_count * counts_per_volt;
	constexpr double fn_1_slope_n_per_volt = fn_1_slope_n_per_count * counts_per_volt;
	constexpr double fn_2_slope_n_per_volt = fn_2_slope_n_per_count * counts_per_volt;
	constexpr double ft_2_slope_n_per_volt = ft_2_slope_n_per_count * counts_per_volt;
	constexpr double max_validated_force_n = 0.4903325;
	constexpr double max_validated_handle_torque_nm = max_validated_force_n * 3.0 * 0.001;
	constexpr double operational_force_limit_n = 5.0;
	constexpr double operational_handle_torque_limit_nm = 20.0 * 0.001;

	// 动态调零等价于 F_direct(raw)-F_direct(raw_zero)，因此固定截距会自然抵消。
	inline double zeroed_force_n(double sample_v, double zero_v, double slope_n_per_volt)
	{
		return slope_n_per_volt * (sample_v - zero_v);
	}
}

struct ForceCalibrationConfig
{
	// 传感器2(fn_1)为导管轴向力，传感器1(ft_1)为导管切向力；四路仅验证0～50 g正向载荷。
	double axial_direct_n_per_v = force_direct_calibration::fn_1_slope_n_per_volt;
	double tangential_direct_n_per_v = force_direct_calibration::ft_1_slope_n_per_volt;
	// 传感器3(fn_2)和传感器4(ft_2)分别驱动导丝手柄的轴向力与力矩。
	double guidewire_axial_direct_n_per_v = force_direct_calibration::fn_2_slope_n_per_volt;
	double guidewire_tangential_direct_n_per_v = force_direct_calibration::ft_2_slope_n_per_volt;

	double decouple_ff =  0.996063;
	double decouple_ft =  0.0; // 轴向力反馈只由轴向力通道决定，避免扭矩通道串入轴向力。
	double decouple_tf = -0.103597;
	double decouple_tt =  0.996063;

	double handle_radius_mm = 3.0;
	double k_feedback = 1.0;

	// 参数来自返工前实验；validated=false时即使误设enabled也不会进入计算。
	bool gravity_comp_enabled = false;
	bool gravity_comp_validated = false;
	double grav_ft_a = 0.4983;
	double grav_ft_phi_deg = -53.54;
	double grav_ft_offset = -0.1119;
	double grav_fa_a = 0.3195;
	double grav_fa_phi_deg = 48.42;
	double grav_fa_offset = -0.1861;
	double rot_counts_per_deg = 2150.0;

	// 用户指定运行上限；该上限超过本次0～50 g单向砝码实验的验证范围。
	double f_max_n = force_direct_calibration::operational_force_limit_n;
	double t_max_nm = force_direct_calibration::operational_handle_torque_limit_nm;
	double deadband_f_n = 0.02;
	double deadband_t_nm = 0.0001;
};

struct ForceCalibrationState
{
	double f_zero = 0.0; // fn_1 装机空载零点，单位 V。
	double ft_zero = 0.0; // ft_1 装机空载零点，单位 V。
	double fn_2_zero = 0.0; // fn_2 装机空载零点，单位 V。
	double ft_2_zero = 0.0; // ft_2 装机空载零点，单位 V。
	double theta0_deg = 0.0;
	bool zeroed = false;
};

struct CalibratedForce
{
	double f_feedback_n = 0.0;
	double t_feedback_nm = 0.0;
};

struct CleanForce
{
	double force_n = 0.0;
	double handle_torque_nm = 0.0;
};

// 纯净力只应用装机零点、F_direct斜率和手柄半径，不进入反馈处理链。
inline CleanForce calculate_clean_force(
	double fn_raw_v,
	double ft_raw_v,
	const ForceCalibrationConfig& cfg,
	const ForceCalibrationState& state)
{
	CleanForce out;
	if (!state.zeroed)
	{
		return out;
	}
	out.force_n = force_direct_calibration::zeroed_force_n(
		fn_raw_v, state.f_zero, cfg.axial_direct_n_per_v);
	const double tangential_force_n = force_direct_calibration::zeroed_force_n(
		ft_raw_v, state.ft_zero, cfg.tangential_direct_n_per_v);
	out.handle_torque_nm = tangential_force_n * cfg.handle_radius_mm * 0.001;
	return out;
}

// 导丝侧纯净力与导管侧保持相同语义：只扣除装机零点并应用本侧 F_direct 斜率。
inline CleanForce calculate_clean_guidewire_force(
	double fn_2_raw_v,
	double ft_2_raw_v,
	const ForceCalibrationConfig& cfg,
	const ForceCalibrationState& state)
{
	CleanForce out;
	if (!state.zeroed)
	{
		return out;
	}
	out.force_n = force_direct_calibration::zeroed_force_n(
		fn_2_raw_v, state.fn_2_zero, cfg.guidewire_axial_direct_n_per_v);
	const double tangential_force_n = force_direct_calibration::zeroed_force_n(
		ft_2_raw_v, state.ft_2_zero, cfg.guidewire_tangential_direct_n_per_v);
	out.handle_torque_nm = tangential_force_n * cfg.handle_radius_mm * 0.001;
	return out;
}

inline CalibratedForce calibrate_direct_pair(
	double f_sensor_v,
	double ft_sensor_v,
	double f_zero_v,
	double ft_zero_v,
	double axial_slope_n_per_v,
	double tangential_slope_n_per_v,
	double axis2_deg,
	const ForceCalibrationConfig& cfg,
	const ForceCalibrationState& state,
	bool apply_gravity_compensation)
{
	CalibratedForce out;
	if (!state.zeroed) return out;

	double df_n = force_direct_calibration::zeroed_force_n(
		f_sensor_v, f_zero_v, axial_slope_n_per_v);
	double dft_n = force_direct_calibration::zeroed_force_n(
		ft_sensor_v, ft_zero_v, tangential_slope_n_per_v);

	if (apply_gravity_compensation && cfg.gravity_comp_enabled && cfg.gravity_comp_validated)
	{
		double theta_deg = std::fmod(axis2_deg, 360.0);
		if (theta_deg < 0.0) theta_deg += 360.0;
		double theta0 = std::fmod(state.theta0_deg, 360.0);
		if (theta0 < 0.0) theta0 += 360.0;

		constexpr double pi = 3.14159265358979323846;
		constexpr double w = 2.0 * pi / 360.0;
		double ft_phi_rad = cfg.grav_ft_phi_deg * pi / 180.0;
		double fa_phi_rad = cfg.grav_fa_phi_deg * pi / 180.0;

		auto ft_grav = [&](double deg) {
			return cfg.grav_ft_a * std::sin(w * deg + ft_phi_rad) + cfg.grav_ft_offset;
		};
		auto fa_grav = [&](double deg) {
			return cfg.grav_fa_a * std::sin(w * deg + fa_phi_rad) + cfg.grav_fa_offset;
		};

		dft_n -= (ft_grav(theta_deg) - ft_grav(theta0));
		df_n  -= (fa_grav(theta_deg) - fa_grav(theta0));
	}

	// 旧解耦矩阵来自返工前的系统级标定，当前保持旁路；重新启用前必须用新物理量复标。
	double F_dec = df_n;
	double Ft_dec = dft_n;

	out.f_feedback_n = F_dec;
	out.t_feedback_nm = cfg.k_feedback * Ft_dec * cfg.handle_radius_mm * 0.001;

	if (std::abs(out.f_feedback_n) < cfg.deadband_f_n) out.f_feedback_n = 0.0;
	if (std::abs(out.t_feedback_nm) < cfg.deadband_t_nm) out.t_feedback_nm = 0.0;

	if (out.f_feedback_n > cfg.f_max_n) out.f_feedback_n = cfg.f_max_n;
	if (out.f_feedback_n < -cfg.f_max_n) out.f_feedback_n = -cfg.f_max_n;
	if (out.t_feedback_nm > cfg.t_max_nm) out.t_feedback_nm = cfg.t_max_nm;
	if (out.t_feedback_nm < -cfg.t_max_nm) out.t_feedback_nm = -cfg.t_max_nm;

	return out;
}

inline CalibratedForce calibrate_force(
	double f_sensor_v,
	double ft_sensor_v,
	double axis2_deg,
	const ForceCalibrationConfig& cfg,
	const ForceCalibrationState& state)
{
	return calibrate_direct_pair(
		f_sensor_v,
		ft_sensor_v,
		state.f_zero,
		state.ft_zero,
		cfg.axial_direct_n_per_v,
		cfg.tangential_direct_n_per_v,
		axis2_deg,
		cfg,
		state,
		true);
}

inline CalibratedForce calibrate_guidewire_force(
	double fn_2_sensor_v,
	double ft_2_sensor_v,
	double axis2_deg,
	const ForceCalibrationConfig& cfg,
	const ForceCalibrationState& state)
{
	// 返工后的导丝传感器尚无独立旋转重力模型，当前只应用F_direct和动态零点。
	return calibrate_direct_pair(
		fn_2_sensor_v,
		ft_2_sensor_v,
		state.fn_2_zero,
		state.ft_2_zero,
		cfg.guidewire_axial_direct_n_per_v,
		cfg.guidewire_tangential_direct_n_per_v,
		axis2_deg,
		cfg,
		state,
		false);
}
