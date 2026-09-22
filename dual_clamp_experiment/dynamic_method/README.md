# 动力学结合神经网络方法库 (Dynamic-Neural Methods Library)

本目录归档了近 3 年内发表于顶刊（IEEE T-RO、MSSP、IEEE T-IE、IEEE/ASME T-MECH、IEEE T-CYB）的 5 种**“动力学模型 + 机器学习/神经网络”**消除力波动与扰动的代表性前沿方法。

针对本系统（双夹爪往复夹持介入机器人 `dual_clamp_experiment`）中的**换手过程冲击、重夹持初期瞬态力波动、滑轨非线性摩擦与远端接触力去嵌**，所有方法均已按核心名称分类整理，并附带原论文 PDF、官方/权威开源代码库及针对本实验系统的可执行 Python 落地实现。

---

## 目录索引与方法矩阵

| 文件夹名称 (核心方法) | 原论文出处 (期刊/年份/IF) | 包含官方开源库 | 本地可执行实现脚本 | 适用本实验系统的核心场景 |
| :--- | :--- | :--- | :--- | :--- |
| [**`lstm-rnn/`**](./lstm-rnn/) | **IEEE/ASME T-MECH (2025)**<br>IF: 6.1, 中科院1区Top | `padasip_official/`<br>(自适应滤波库) | `lstm_vibration_compensation.py` | **零相位滞后对冲**：重夹持闭合瞬间的瞬态冲击波峰前瞻预测与反向抵消 |
| [**`fd-gru/`**](./fd-gru/) | **MSSP (2024)**<br>IF: 8.4, 中科院1区Top | `APHYNITY_official/`<br>(物理增强深度学习库) | `fd_gru_decoupling.py` | **机械摩擦与外力解耦**：从电机总推力中彻底剥离滑轨非线性 Stribeck 摩擦 |
| [**`delan-pinn/`**](./delan-pinn/) | **IEEE T-IE (2025)**<br>IF: 7.5, 中科院1区Top | `DeLaN_official/`<br>(深度拉格朗日官方库) | `delan_stribeck_pinn.py` | **无传感器无偏参数辨识**：无各关节独立力矩传感器下，正定惯量与摩擦参数辨识 |
| [**`koopman-evolver/`**](./koopman-evolver/) | **IEEE T-RO (2024)**<br>IF: 9.4, 中科院1区Top | `EVOLVER_official/`<br>`pykoopman_official/` | `koopman_evolver_online.py` | **在线极速自适应演化**：无须反向传播，纯矩阵解析逆在线毫秒级跟踪时变突发扰动 |
| [**`augmented-pinn/`**](./augmented-pinn/) | **IEEE T-CYB (2024)**<br>IF: 11.8, 中科院1区Top | `deepxde_official/`<br>(通用 PINN 框架) | `augmented_pinn_model.py` | **小样本物理正则化**：在样本量极少时，基于物理守恒软约束防止外推力失真发散 |

---

## 各子目录标准结构说明
每一个子目录均统一包含以下 4 项资产：
1. **原版学术论文 PDF**：完整的顶刊出版物，供理论推演与论文撰写引用；
2. **官方开源仓库 / 权威母本仓库**：通过 Git 克隆完整的开源代码库（如 `DeLaN_official`, `pykoopman_official`, `deepxde_official` 等）；
3. **针对性工程脚本 (`.py`)**：根据双夹爪往复运动特征重构的轻量、即插即用 PyTorch/NumPy 模型脚本；
4. **方法使用说明 (`README.md`)**：详细阐述算法原理、数学公式、接口参数以及与本工程 C++ 驱动（`DualClampController.cpp`, `ClampDynamics.h`, `ExperimentStreamRecorder.cpp` 等）的对接方案。

---

## 与双夹爪实验系统的对接推荐流程
1. **第一步（静态/滑轨摩擦校准）**：使用 `delan-pinn` 或 `fd-gru` 脚本，在空载往复运动下精确辨识导轨的 Stribeck 摩擦系数与滑块质量，填充进 `ClampDisturbanceParameters.h`。
2. **第二步（瞬态夹紧对冲消除）**：使用 `lstm-rnn` 的超前固定步长外推机制，在夹爪闭合触发时输出对冲波形，与实时力传感器读数相减，消除 `Handle582Feedback.h` 中的触觉震荡。
3. **第三步（长时间耐磨在线更新）**：引入 `koopman-evolver` 的 EDMD 在线闭式解，在机器人长周期运行时实时修正机械磨损导致的参数漂移。
