# DeLaN-PINN: 深度拉格朗日与 Stribeck 摩擦耦合物理信息网络

## 1. 对应论文
* **题目**: *A PINN-Based Friction-Inclusive Dynamics Modeling Method for Industrial Robots*
* **期刊**: IEEE Transactions on Industrial Electronics (T-IE), 2025, Vol. 72, No. 5, pp. 5136-5144.
* **DOI**: `10.1109/TIE.2024.3412586`
* **论文文件**: `Hu 等 - 2025 - A PINN-Based Friction-Inclusive Dynamics Modeling Method for Industrial Robots.pdf`

## 2. 文件夹内容
1. `Hu 等 - 2025 - ... .pdf`: 原始论文 PDF。
2. `DeLaN_official/`: 德国达姆施塔特工业大学（Michael Lutter, Jan Peters）开源的官方 Deep Lagrangian Networks (DeLaN) 完整工程代码库。
3. `delan_stribeck_pinn.py`: 结合 Stribeck 摩擦项与双环交替参数冻结训练策略的完整实现脚本。

## 3. 核心机制与架构
* **能量守恒与正定性保证**：通过网络参数化下三角矩阵 $L(q)$，强制构造质量惯量矩阵 $M(q) = L(q)L(q)^T > 0$，绝不出现物理失真；
* **显式摩擦方程解耦**：将非线性摩擦抽象为经典 Stribeck 方程，参数作为网络可学习的物理标量，不仅精度高，而且参数可直接读取并用于控制设计；
* **双环冻结训练**：
  * Loop 1: 惯性网与摩擦参数联合优化；
  * Loop 2: 冻结惯性网，单训摩擦参数；再冻结摩擦参数，单训惯性网。

## 4. 如何适配到双夹爪实验 (`dual_clamp_experiment`)
* 在双夹爪直线滑台递送模型中（对应工程中的 `ClampDynamics.h` 和 `ClampDisturbanceParameters.h`）：
  * 传统的最小二乘法在低速换向区由于摩擦与惯性耦合往往失效；
  * 本方法可以利用机器人的空载往复运动数据（位移 $q$、速度 $\dot{q}$、加速度 $\ddot{q}$ 和电机推力 $\tau$），在**不需要加装任何附加摩擦力传感器**的前提下，将滑块质量 $m$ 与滑轨 Stribeck 摩擦参数（$F_c, F_s, v_s, F_v$）高精度辨识出来，直接填充到 `ClampDisturbanceParameters.h` 中。
