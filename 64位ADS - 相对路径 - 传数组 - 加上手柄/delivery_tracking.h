#pragma once

#include <cstdint>

// 主从位移实验只覆盖普通导管递送的 axis1 和独立导丝递送的 axis6。
// 该控制器不访问 ADS，也不写文件，便于在主控制循环中以常数时间调用。
enum class DeliveryTrackingAxis : int
{
	Axis1 = 0,
	Axis6 = 1
};

// 状态码保持稳定，供主循环门控与 WPF 状态显示使用。
enum class TrackingInvalidReason : int
{
	None = 0,
	NotForwardDelivery = 2,
	ControlInactive = 3,
	StartupActive = 4,
	Paused = 5,
	PlcHold = 6,
	AdsReturnFault = 7,
	SpacingRecovery = 8,
	ForceTransitionExperiment = 9,
	ManualCylinderOverride = 10,
	CrawlReturnActive = 11,
	GripSettling = 12
};

// 管道 SetTrackingCompensationParam 的 param1 编码。
enum class TrackingParameterField : int
{
	Axis1Kp = 0,
	Axis1Ki = 1,
	Axis1MaxGain = 2,
	Axis1MaxError = 3,
	Axis6Kp = 4,
	Axis6Ki = 5,
	Axis6MaxGain = 6,
	Axis6MaxError = 7
};

struct DeliveryTrackingParameters
{
	// Kp 单位为 1/mm，Ki 单位为 1/(mm*s)。默认值与 WPF 输入框一致。
	double kp = 0.5;
	double ki = 2.0;
	double max_gain = 2.0;
	// 换手前向欠账的硬上界。超过上界的原始手柄位移仍记日志，但不会再进入补偿队列。
	double max_error_mm = 20.0;
};

// 每拍状态快照供控制器内部累计与 UI 显示使用，不访问磁盘。
struct DeliveryTrackingAxisSnapshot
{
	bool segment_active = false;
	bool grip_assumed = false;
	int invalid_reason = static_cast<int>(TrackingInvalidReason::NotForwardDelivery);
	std::uint64_t segment_id = 0;

	double handle_raw = 0.0;
	double handle_filtered = 0.0;
	// 保留原轴坐标符号；递送时通常为负值（朝左限位）。
	double raw_mapping_increment_axis_mm = 0.0;
	// 以下三个增量统一为“正向递送”为正的归一化单位 mm。
	double nominal_forward_increment_mm = 0.0;
	double compensated_requested_forward_increment_mm = 0.0;
	double effective_forward_increment_mm = 0.0;
	// 仅在 SwitchWait/FastMove/RestoreWait 期间记录。回拉手柄时保持为 0。
	double handover_forward_increment_mm = 0.0;
	double session_handover_forward_mm = 0.0;

	double compensation_gain = 1.0;
	double p_term = 0.0;
	double i_term = 0.0;
	bool integral_limited = false;

	double refer_rel_mm = 0.0;
	double actual_rel_mm = 0.0;
	double actual_forward_delta_mm = 0.0;
	double segment_master_forward_mm = 0.0;
	double segment_actual_forward_mm = 0.0;
	double session_master_forward_mm = 0.0;
	double session_actual_forward_mm = 0.0;
	// 当前尚未由实际补偿运动消除的换手前向欠账，始终位于 [0, max_error_mm]。
	double tracking_error_mm = 0.0;
	bool tracking_error_limited = false;
};

class DeliveryTrackingController
{
public:
	static constexpr std::uint32_t kGripSettleMs = 150;

	DeliveryTrackingController();

	bool set_parameter(TrackingParameterField field, double value);
	const DeliveryTrackingParameters& parameters(DeliveryTrackingAxis axis) const;

	bool can_enable_compensation(DeliveryTrackingAxis active_axis) const;
	bool set_compensation_enabled(bool enabled, DeliveryTrackingAxis active_axis);
	bool compensation_enabled() const { return compensation_enabled_; }
	void disable_compensation();

	// 每拍在执行递送命令前调用。base_eligible 已包含模式、Follow、暂停、回退等门控。
	void update_gate(
		DeliveryTrackingAxis axis,
		bool base_eligible,
		bool grip_command_active,
		std::uint32_t now_ms,
		double actual_abs_mm,
		TrackingInvalidReason invalid_reason);

	// 未能读取 PLC 状态或主控制分支未执行时立即结束当前段。
	// 普通换手仅冻结 PI 而保留前向欠账；暂停、模式退出等安全状态会清除欠账。
	void invalidate_all(TrackingInvalidReason reason);

	// 写入本拍原始手柄数据，清空上一拍的命令增量显示。
	void begin_cycle(
		DeliveryTrackingAxis axis,
		double handle_raw,
		double handle_filtered,
		double raw_mapping_increment_axis_mm,
		double actual_rel_mm);

	// 只在正向递送的换手阶段调用：记录医生持续前推、但当前机构不跟随的映射位移。
	// 回拉手柄的归一化增量应由调用方传 0，因此不会形成负欠账。
	void record_handover_forward_increment(
		DeliveryTrackingAxis axis,
		double forward_increment_mm);

	// 输入和输出均使用“正向递送为正”的归一化 mm。
	// 换手欠账跨阶段保留，并在后续正向 Follow 输入中按受限映射增益逐步补偿；
	// 松手或回拉时返回 0，不产生自主追赶。
	double request_compensated_forward_increment(
		DeliveryTrackingAxis axis,
		double nominal_forward_increment_mm,
		std::uint32_t now_ms);

	// 约束后的实际命令回写。被窗口或停止条件夹紧时冻结积分，防止 wind-up；
	// 实际补偿增量会作为待确认量，只有轴实际前进后才从换手欠账中扣除。
	void commit_command(
		DeliveryTrackingAxis axis,
		double nominal_forward_increment_mm,
		double compensated_requested_forward_increment_mm,
		double effective_forward_increment_mm,
		bool output_clamped);

	void finish_cycle(DeliveryTrackingAxis axis, double refer_rel_mm, double actual_rel_mm);
	DeliveryTrackingAxisSnapshot snapshot(DeliveryTrackingAxis axis) const;

private:
	struct AxisRuntime
	{
		bool active = false;
		std::uint32_t grip_command_since_ms = 0;
		std::uint32_t last_pi_tick_ms = 0;
		double last_actual_abs_mm = 0.0;
		double integral_mm_s = 0.0;
		double integral_before_last_update_mm_s = 0.0;
		double pending_handover_error_mm = 0.0;
		double nominal_forward_outstanding_mm = 0.0;
		double compensation_forward_outstanding_mm = 0.0;
		bool pending_handover_error_limited = false;
		DeliveryTrackingAxisSnapshot snapshot;
	};

	static constexpr double kMaxUiKp = 1.0;
	static constexpr double kMaxUiKi = 100.0;
	static constexpr double kMaxUiGain = 5.0;
	static constexpr double kMaxUiErrorMm = 20.0;
	static constexpr double kMaxIntegralMmS = 20.0;
	static constexpr double kMaxCompensationStepMm = 0.25;

	AxisRuntime& runtime(DeliveryTrackingAxis axis);
	const AxisRuntime& runtime(DeliveryTrackingAxis axis) const;
	void reset_pi(AxisRuntime& state);
	void reset_handover_error(AxisRuntime& state);
	void clear_outstanding_motion(AxisRuntime& state);
	void end_segment(AxisRuntime& state, TrackingInvalidReason reason, bool keep_grip_timer);
	void begin_segment(AxisRuntime& state, std::uint32_t now_ms, double actual_abs_mm);
	void update_actual_progress(AxisRuntime& state, double actual_abs_mm);
	void refresh_error(AxisRuntime& state);
	static bool should_preserve_handover_error(TrackingInvalidReason reason);
	static bool is_finite(double value);

	AxisRuntime axis_[2];
	DeliveryTrackingParameters parameters_[2];
	bool compensation_enabled_ = false;
	std::uint64_t next_segment_id_ = 0;
};
