// ForceSignalProcessor.h
// 力信号在线处理模块：尖峰检测 + 零位跟踪 + 模式/置信度输出
// 用法：在 main.cpp 中 #include "ForceSignalProcessor.h"
//       在主循环中调用 processor.update(...) 即可
// 不依赖任何外部库，仅使用标准 C++。

#pragma once
#include <cmath>
#include <algorithm>
#include <cstring>

// ============================================================
// 单通道力信号处理器
// ============================================================
class SingleChannelForceFilter
{
public:
	// --- 可调参数（构造后也可直接改） ----------------------

	// 尖峰检测：滑窗长度
	int spike_window = 32;
	// 尖峰检测：MAD 倍数阈值（越大越宽松）
	double spike_k = 3.5;

	// 零位跟踪：静止窗口最小连续采样数
	int quiet_min_samples = 20;
	// 零位跟踪：EMA 更新系数（0~1，越大跟踪越快）
	double zero_alpha = 0.3;
	// 零位跟踪：复位刚结束后的快速更新系数
	double zero_alpha_fast = 0.8;

	// --- 输出 ---------------------------------------------

	// 补偿后的力值（raw - 零位估计，且尖峰已替换）
	double compensated = 0.0;
	// 当前零位估计
	double zero_estimate = 0.0;
	// 当前帧是否被判定为尖峰
	bool is_spike = false;
	// 置信度 [0, 1]
	double confidence = 1.0;

	// --- 接口 ---------------------------------------------

	// 初始化：传入首个 raw 值作为初始零位
	void init(double first_raw)
	{
		zero_estimate = first_raw;
		initialized_ = true;
		ring_count_ = 0;
		quiet_count_ = 0;
		post_reset_frames_ = 0;
	}

	// 每个主循环调用一次
	// raw          : 原始传感器 short 值（转成 double 传入）
	// is_quiet     : 当前帧是否处于静止状态（push_pull_code == 0 且 rot_sign_code == 0）
	// is_fast_move : 当前帧是否处于快速回退/复位状态
	void update(double raw, bool is_quiet, bool is_fast_move)
	{
		if (!initialized_)
		{
			init(raw);
		}

		// ---- 1) 环形缓冲区维护 ----
		ring_buf_[ring_head_] = raw;
		ring_head_ = (ring_head_ + 1) % RING_MAX;
		if (ring_count_ < RING_MAX) ring_count_++;

		// ---- 2) 尖峰检测 ----
		is_spike = false;
		double cleaned = raw;

		if (ring_count_ >= spike_window)
		{
			double med = 0.0, mad = 0.0;
			compute_median_mad(med, mad);

			const double threshold = spike_k * 1.4826 * mad;
			if (threshold > 1e-6 && std::fabs(raw - med) > threshold)
			{
				is_spike = true;
				cleaned = med; // 用中值替代尖峰
			}
		}

		// ---- 3) 零位跟踪 ----
		if (is_fast_move)
		{
			// 复位期间不更新零位，但标记进入过复位
			post_reset_frames_ = post_reset_settle_;
			quiet_count_ = 0;
		}
		else if (is_quiet && !is_spike)
		{
			quiet_count_++;
			quiet_sum_ += cleaned;

			if (quiet_count_ >= quiet_min_samples)
			{
				double local_zero = quiet_sum_ / quiet_count_;
				// 复位刚结束：用快速系数
				double alpha = (post_reset_frames_ > 0) ? zero_alpha_fast : zero_alpha;
				zero_estimate = alpha * local_zero + (1.0 - alpha) * zero_estimate;

				// 每次更新后重置累计，这样后续静止段会持续微调
				quiet_count_ = 0;
				quiet_sum_ = 0.0;
			}
		}
		else
		{
			quiet_count_ = 0;
			quiet_sum_ = 0.0;
		}

		if (post_reset_frames_ > 0) post_reset_frames_--;

		// ---- 4) 输出 ----
		compensated = cleaned - zero_estimate;

		// 置信度
		if (is_fast_move)
			confidence = 0.2;
		else if (is_spike)
			confidence = 0.3;
		else if (post_reset_frames_ > 0)
			confidence = 0.6;
		else
			confidence = 1.0;
	}

private:
	static const int RING_MAX = 128; // 环形缓冲区最大长度，需 >= spike_window
	double ring_buf_[RING_MAX] = {};
	int ring_head_ = 0;
	int ring_count_ = 0;

	bool initialized_ = false;
	int quiet_count_ = 0;
	double quiet_sum_ = 0.0;
	int post_reset_frames_ = 0;
	int post_reset_settle_ = 50; // 复位后多少帧视为"恢复过渡期"

	// 从环形缓冲区中计算中值和 MAD
	void compute_median_mad(double& out_median, double& out_mad) const
	{
		// 取最近 spike_window 个值
		int n = (spike_window < ring_count_) ? spike_window : ring_count_;
		double tmp[RING_MAX];
		for (int i = 0; i < n; i++)
		{
			int idx = (ring_head_ - 1 - i + RING_MAX) % RING_MAX;
			tmp[i] = ring_buf_[idx];
		}
		std::sort(tmp, tmp + n);
		out_median = (n % 2 == 1) ? tmp[n / 2] : (tmp[n / 2 - 1] + tmp[n / 2]) * 0.5;

		for (int i = 0; i < n; i++)
			tmp[i] = std::fabs(tmp[i] - out_median);
		std::sort(tmp, tmp + n);
		out_mad = (n % 2 == 1) ? tmp[n / 2] : (tmp[n / 2 - 1] + tmp[n / 2]) * 0.5;
	}
};


// ============================================================
// 四通道力信号处理器（包装层，对应你的 4 路传感器）
// ============================================================
struct ForceProcessorOutput
{
	double fn_1 = 0.0;   // 补偿后轴向力 1
	double ft_1 = 0.0;   // 补偿后扭矩 1
	double fn_2 = 0.0;   // 补偿后轴向力 2
	double ft_2 = 0.0;   // 补偿后扭矩 2

	double conf_fn_1 = 1.0;
	double conf_ft_1 = 1.0;
	double conf_fn_2 = 1.0;
	double conf_ft_2 = 1.0;

	bool spike_fn_1 = false;
	bool spike_ft_1 = false;
	bool spike_fn_2 = false;
	bool spike_ft_2 = false;
};

class ForceSignalProcessor
{
public:
	// 四个独立通道的滤波器
	SingleChannelForceFilter filter_fn_1;
	SingleChannelForceFilter filter_ft_1;
	SingleChannelForceFilter filter_fn_2;
	SingleChannelForceFilter filter_ft_2;

	// 处理结果
	ForceProcessorOutput output;

	// 每个主循环调用一次
	// 参数直接对应 ForceSampleFrame 的字段和状态编码
	void update(
		short fn_1_raw, short ft_1_raw,
		short fn_2_raw, short ft_2_raw,
		int push_pull_code, int rot_sign_code,
		bool is_fast_move)
	{
		bool is_quiet = (push_pull_code == 0 && rot_sign_code == 0);

		filter_fn_1.update(static_cast<double>(fn_1_raw), is_quiet, is_fast_move);
		filter_ft_1.update(static_cast<double>(ft_1_raw), is_quiet, is_fast_move);
		filter_fn_2.update(static_cast<double>(fn_2_raw), is_quiet, is_fast_move);
		filter_ft_2.update(static_cast<double>(ft_2_raw), is_quiet, is_fast_move);

		output.fn_1 = filter_fn_1.compensated;
		output.ft_1 = filter_ft_1.compensated;
		output.fn_2 = filter_fn_2.compensated;
		output.ft_2 = filter_ft_2.compensated;

		output.conf_fn_1 = filter_fn_1.confidence;
		output.conf_ft_1 = filter_ft_1.confidence;
		output.conf_fn_2 = filter_fn_2.confidence;
		output.conf_ft_2 = filter_ft_2.confidence;

		output.spike_fn_1 = filter_fn_1.is_spike;
		output.spike_ft_1 = filter_ft_1.is_spike;
		output.spike_fn_2 = filter_fn_2.is_spike;
		output.spike_ft_2 = filter_ft_2.is_spike;
	}
};
