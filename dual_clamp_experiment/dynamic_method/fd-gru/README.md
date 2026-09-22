# FD-GRU: 机理引导摩擦-动力学耦合双 GRU 网络

## 1. 对应论文
* **题目**: *Mechanism-informed friction-dynamics coupling GRU neural network for real-time cutting force prediction*
* **期刊**: Mechanical Systems and Signal Processing (MSSP), 2024, Vol. 221, Article 111749.
* **DOI**: `10.1016/j.ymssp.2024.111749`
* **论文文件**: `Cheng 等 - 2024 - Mechanism-informed friction-dynamics coupling GRU neural network for real-time cutting force predict.pdf`

## 2. 文件夹内容
1. `Cheng 等 - 2024 - ... .pdf`: 原始论文 PDF。
2. `APHYNITY_official/`: 物理机理与深度学习结合领域的开源官方基准库 (ICLR/NeurIPS)。
3. `fd_gru_decoupling.py`: 针对介入机器人双夹爪滑轨摩擦与传动纹波解耦的完整 PyTorch 双子网实现。

## 3. 核心机制与架构
```
                ┌──────────────────┐
速度序列 v(t) ──►│  Friction-GRU    ├──► 估计非线性摩擦力 F_f(t)
                └──────────────────┘          │
                                              ▼
总驱动力 F(t)  ───────┐               ┌──────────────────┐
加速度 a(t)    ───────┼──────────────►│  Dynamics-GRU    ├──► 纯净外部有效接触力 F_ext(t)
                      │               └──────────────────┘
                      └─ 物理拓扑硬连接: F_ext = F_drive - m*a - F_f
```

## 4. 如何适配到双夹爪实验 (`dual_clamp_experiment`)
在双夹爪实验中，滑块在直线导轨上往复运动，传感器测得的轴向推力包含了：
$$F_{\text{sensor}} = m_{\text{eq}} \ddot{x} + F_{\text{rail\_friction}}(\dot{x}) + F_{\text{clamp\_shock}} + F_{\text{catheter\_interaction}}$$
* **Friction-GRU 适配**：输入滑台实时编码器速度 $\dot{x}$，专门拟合滑轨库仑摩擦、静摩擦及微观黏滑特性；
* **Dynamics-GRU 适配**：输入传感器测得的轴向力 $F_n$、加速度 $\ddot{x}$ 及 Friction-GRU 估计出的摩擦力，反求出剔除导轨阻力与冲击后的**导管真实前端受力**。
* 对应 C++ 代码模块：可与 `ClampDynamics.h`（名义参数）和 `DualClampPipe.cpp`（数据管道）无缝配合。
