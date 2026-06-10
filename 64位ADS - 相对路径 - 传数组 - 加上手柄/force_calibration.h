#pragma once
#include <cmath>

struct ForceCalibrationConfig
{
	double axial_k = 1.913504;
	double torque_k = 18.440851;

	double decouple_ff =  0.996063;
	double decouple_ft =  0.0; // 轴向力反馈只由轴向力通道决定，避免扭矩通道串入轴向力。
	double decouple_tf = -0.103597;
	double decouple_tt =  0.996063;

	double handle_radius_mm = 3.0;
	double slave_lever_mm = 37.0;
	double k_feedback = 1.0;

	bool gravity_comp_enabled = false;
	double grav_ft_a = 0.4983;
	double grav_ft_phi_deg = -53.54;
	double grav_ft_offset = -0.1119;
	double grav_fa_a = 0.3195;
	double grav_fa_phi_deg = 48.42;
	double grav_fa_offset = -0.1861;
	double rot_counts_per_deg = 2150.0;

	double f_max_n = 5.0;
	double t_max_nm = 0.030;
	double deadband_f_n = 0.02;
	double deadband_t_nm = 0.0001;
};

struct ForceCalibrationState
{
	double f_zero = 0.0;
	double ft_zero = 0.0;
	double theta0_deg = 0.0;
	bool zeroed = false;
};

struct CalibratedForce
{
	double f_feedback_n = 0.0;
	double t_feedback_nm = 0.0;
};

inline CalibratedForce calibrate_force(
	double f_sensor,
	double ft_sensor,
	double rot_counts,
	double rot_zero_counts,
	const ForceCalibrationConfig& cfg,
	const ForceCalibrationState& state)
{
	CalibratedForce out;
	if (!state.zeroed) return out;

	double df = f_sensor - state.f_zero;
	double dft = ft_sensor - state.ft_zero;

	if (cfg.gravity_comp_enabled)
	{
		double theta_deg = std::fmod((rot_counts - rot_zero_counts) / cfg.rot_counts_per_deg, 360.0);
		if (theta_deg < 0.0) theta_deg += 360.0;
		double theta0 = state.theta0_deg;

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

		dft -= (ft_grav(theta_deg) - ft_grav(theta0));
		df  -= (fa_grav(theta_deg) - fa_grav(theta0));
	}

	double F_cal = cfg.axial_k * df;
	double T_cal = cfg.torque_k * dft;

	double F_dec = cfg.decouple_ff * F_cal + cfg.decouple_ft * T_cal;
	double T_dec = cfg.decouple_tf * F_cal + cfg.decouple_tt * T_cal;

	out.f_feedback_n = F_dec;
	out.t_feedback_nm = cfg.k_feedback * T_dec
		* (cfg.handle_radius_mm / cfg.slave_lever_mm)
		* 0.001;

	if (std::abs(out.f_feedback_n) < cfg.deadband_f_n) out.f_feedback_n = 0.0;
	if (std::abs(out.t_feedback_nm) < cfg.deadband_t_nm) out.t_feedback_nm = 0.0;

	if (out.f_feedback_n > cfg.f_max_n) out.f_feedback_n = cfg.f_max_n;
	if (out.f_feedback_n < -cfg.f_max_n) out.f_feedback_n = -cfg.f_max_n;
	if (out.t_feedback_nm > cfg.t_max_nm) out.t_feedback_nm = cfg.t_max_nm;
	if (out.t_feedback_nm < -cfg.t_max_nm) out.t_feedback_nm = -cfg.t_max_nm;

	return out;
}
