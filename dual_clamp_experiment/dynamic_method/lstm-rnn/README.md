# LSTM-RNN: 实时振动估计与超前预测补偿方法

## 1. 对应论文
* **题目**: *Real-Time Vibration Estimation and Compensation With Long Short-Term Memory Recurrent Neural Network*
* **期刊**: IEEE/ASME Transactions on Mechatronics (T-MECH), 2025, Vol. 30, No. 2, pp. 829-839.
* **DOI**: `10.1109/TMECH.2024.3496533`
* **论文文件**: `He 等 - 2025 - Real-Time Vibration Estimation and Compensation With Long Short-Term Memory Recurrent Neural Network.pdf`

## 2. 文件夹内容
1. `He 等 - 2025 - ... .pdf`: 原始论文 PDF。
2. `padasip_official/`: 官方自适应滤波开源库（包含 RLS、LMS、NLMS 等自适应滤波实现）。
3. `lstm_vibration_compensation.py`: 针对介入机器人往复夹持力波动的完整 PyTorch 超前预测对冲实现脚本。

## 3. 核心机制与架构
```
[传感器原始力信号] 
       │
       ▼
[RLS 自适应滤波]  --> 消除基底零漂与高频带外白噪声
       │
       ▼
[LSTM 时序滑窗]  --> 输入 [y_{t-L+1}, ..., y_t]
       │
       ▼
[超前预测外推]   --> 预测未来第 t + Δt 步的瞬态冲击扰动 (Δt = 滤波时延 + 机构响应时延)
       │
       ▼
[前馈对冲消除]   --> F_pure = F_raw - d_pred (实现零相位滞后的瞬态对冲)
```

## 4. 如何适配到双夹爪实验 (`dual_clamp_experiment`)
在双夹爪递送系统（`dual_clamp_experiment`）中：
1. **数据源接入**：
   * 从 `ExperimentStreamRecorder` 或 `ClampCurveBuffer.h` 读取拉压力传感器的原始读数 $F_n$（轴向力）和 $F_t$（切向力）。
   * 结合夹爪开合状态触发信号（状态机进入“夹紧”阶段的时间点 $t_{close}$）。
2. **时延对冲部署**：
   * 将经过训练的 LSTM 导出为 ONNX 模型，在 C++ 工程（`DualClampController.cpp` 或 `ClampDisturbance.h`）中通过 ONNX Runtime 或 LibTorch 调用；
   * 在夹爪闭合事件触发时，提前计算未来几毫秒的闭合碰撞反弹力，并从实时读数中反向扣除，确保上位机力觉反馈（`Handle582Feedback.h`）平滑无冲击。
