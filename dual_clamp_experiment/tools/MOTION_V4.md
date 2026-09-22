# 运动补偿软件修复：2026-09-20

版本：`motion-only-axis1-v4-20260920`。

## 当前计算合同

轴1/fn1 使用第一阶段原始完整精度 OLS 系数，输入是递送正方向的 SI 速度、NC加速度及速度符号：

```text
a = -axis1_acc_mm_s2 * 0.001
v = -axis1_vel_mm_s * 0.001
q = sign(v) when abs(v) > 0.0002, otherwise 0
d = 0.06826911006047173*a + 0.07362809525467874*v - 0.0012146051548365536*q
F_corr = F_raw - d
```

不减模型截距，不做非负截断，不乘传递效率倒数，不执行阶段清零。Phase 4/9 是递送段，其余阶段的值是机构测力，不是末端力。当前模型不含夹持瞬态项，也未把探索性的52/91ms平移或不稳定极点部署为实时模型。

轴6/side2模型不可用，但允许原始实验采集；派生CSV的预测/补偿值留空，`model_valid=0`。轴1模型不能借此标成轴6模型。界面显示原始信号，隐藏无效补偿段，状态为待辨识。

UI和后端默认符号均为-1；模式切换保留各模式选择。禁止旧客户端通过 `model_reconstruct=1` 启用未经验证的逆重构。旧的alpha、预载和消隐字段仅保留协议兼容，非默认值会被拒绝。

脉冲清理与原始输入模型为独立曲线：脉冲开关不再改变补偿值；模型2仍是默认关闭的目标示意。无效模型不能以原始力伪装为有效补偿结果。

## 手柄与设备

移除了程序递送控制器对手柄的自动初始化和块末力发送。没有实现新的手柄控制器。恢复力反馈前须单独完成逐周期通路、有效性、超时、限幅/变化率和硬件验证，不应把512点记录块作为实时控制通路。

本次没有修改PLC、启动ADS、驱动机械轴或进行硬件验证。软件状态字段不再声称物理模型、时钟同步和符号已完成硬件验证。

## 构建与验收

隔离后端：`x64/SoftwareFix20260920/DualClampExperiment.exe`。
配套UI：`AdsControlUI/bin/SoftwareFix20260920/DualClampExperimentUI.exe`。
该UI目录配有同版后端，防止优先启动旧Debug后端；历史标准Debug未覆盖。这里给出路径不表示已启动或部署到设备。

```powershell
.\tools\build_inertia_tests.ps1
python .\tools\verify_motion_v4.py
.\tools\verify_inertia_ui.ps1 -UiExecutable "<工程绝对路径>\AdsControlUI\bin\SoftwareFix20260920\DualClampExperimentUI.exe"
.\tools\build_external_tests.ps1
```

旧 `verify_dynamics.py` 已转发至当前验证入口。核心测试覆盖负力、Phase 9、外力增量保留、轴6禁用、无效输入、模型重置、因果前缀、分块一致性。CSV测试覆盖轴6空值、原始数值保留、模型元数据。真实6文件回放只读取2026-09-18源记录，所有文件哈希不变。外源模式使用离线运动块替身执行状态机，不等价于TwinCAT编译或硬件通过。

测试输出：`x64/SoftwareFix20260920/verification/verification.json`。

补充实验执行表位于论文工作区：
`D:\Work_files\Vessel intervention Robot\paper\translation_identification\EXPERIMENT_SUPPLEMENT_20260920.md`。
