# Koopman-EVOLVER: 在线自适应扰动学习与前瞻预测框架

## 1. 对应论文
* **题目**: *EVOLVER: Online Learning and Prediction of Disturbances for Robot Control*
* **期刊**: IEEE Transactions on Robotics (T-RO), 2024, Vol. 40, pp. 382–402.
* **DOI**: `10.1109/TRO.2023.3332029`
* **论文文件**: `Jia 等 - 2024 - EVOLVER Online Learning and Prediction of Disturbances for Robot Control.pdf`

## 2. 文件夹内容
1. `Jia 等 - 2024 - ... .pdf`: 原始论文 PDF。
2. `EVOLVER_official/`: 论文第一作者 Jindou Jia 官方公开的 GitHub 源码仓库。
3. `pykoopman_official/`: 华盛顿大学 Brunton 团队开发的权威开源库 `pykoopman`（目前业界最强大的 Koopman 算子与 EDMD 动态学习库）。
4. `koopman_evolver_online.py`: 针对时变扰动在线毫秒级自适应闭式解预测的代码实现。

## 3. 核心机制与架构
```
传感器测量状态 / 名义动力学残差 e_t
                  │
                  ▼
         [提升函数 Phi(e_t)]  ---> 升维至高维特征流形空间
                  │
                  ▼
         [在线 EDMD 矩阵更新] ---> K = Y * X^T * (X * X^T + λI)^(-1) (无BP反向传播，纯矩阵解析逆)
                  │
                  ▼
         [前瞻预测 e_{t+1}]   ---> C * K * Phi(e_t) (超前一步/多步扰动波形预测)
                  │
                  ▼
         [前馈抵消通道]       ---> tau_ctrl = tau_nominal - e_{pred} (主动对冲突发扰动)
```

## 4. 如何适配到双夹爪实验 (`dual_clamp_experiment`)
* **最大优势**：**完全不需要 GPU，也不需要耗时的离线训练与反向传播**。
* 在双夹爪切换（夹爪 A 释放、滑台回程、夹爪 B 夹紧推进）阶段：
  * 将 `ClampCurveBuffer.h` 中记录的前几次换手周期的扰动时序作为数据滑动窗口；
  * 利用 EDMD 闭式解在 C++ 中在线拟合 Koopman 转移矩阵（只需几行 Eigen 库矩阵乘法）；
  * 当状态机（`ProgrammedDeliveryController.cpp`）再次触发换手时，直接前瞻推算出本轮夹爪闭合碰撞力的波形并在力觉通道（`ForcePulseGuard.h`）中实时减去，具有极佳的计算实时性（单步 $< 0.1 \text{ ms}$）。
