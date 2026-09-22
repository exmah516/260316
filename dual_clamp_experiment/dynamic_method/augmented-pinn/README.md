# Augmented-PINN: 增强动力学与物理损失约束网络

## 1. 对应论文
* **题目**: *A Physics-Informed Neural Network Approach to Augmented Dynamics Visual Servoing of Multirotors*
* **期刊**: IEEE Transactions on Cybernetics (T-CYB), 2024, Vol. 54, No. 11, pp. 6319–6332.
* **DOI**: `10.1109/TCYB.2024.3392476`
* **论文文件**: `Kamath 等 - 2024 - A Physics-Informed Neural Network Approach to Augmented Dynamics Visual Servoing of Multirotors.pdf`

## 2. 文件夹内容
1. `Kamath 等 - 2024 - ... .pdf`: 原始论文 PDF。
2. `deepxde_official/`: 国际最通用的物理信息神经网络开源框架 `DeepXDE`（支持各类自定义物理方程正则化损失与符号求导）。
3. `augmented_pinn_model.py`: 针对未建模动态与小样本物理约束学习的完整 PyTorch 实现脚本。

## 3. 核心机制与架构
```
运动状态 x(t), 控制输入 u(t)
          │
          ├─────────────────────────┐
          ▼                         ▼
[已知标称刚体动力学 f_nom]    [神经网络扰动估计器 f_net]
          │                         │
          └───────────┬─────────────┘
                      ▼
            [增强动力学总响应 f_aug]
                      │
                      ▼
     Loss = MSE(实测数据) + λ * Loss(物理边界惩罚)
```

## 4. 如何适配到双夹爪实验 (`dual_clamp_experiment`)
* **小样本标定优势**：在介入手术机器人中，不可能反复采集成千上万次真实导丝插入血管的破坏性实验数据。
* **物理约束嵌入**：
  * 在双夹爪推进导管时，将导轨刚体质量 $m_{\text{nom}}$ 与名义推力作为标称项；
  * 网络专门学习微小导向鞘管非线性形变阻力；
  * 在损失函数中加入物理硬约束：当速度为零且未夹持时，未建模外力必须趋向于 0；
  * **在样本量仅有几十组的情形下，就能快速收敛且杜绝外推时物理发散的危险**。
