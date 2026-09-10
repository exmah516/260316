# 25 g 一维惯性试算

## 实现与公式

当前版本 `inertia-feedback-v2-m025`，导管轴1和导丝轴6各自假设有效质量0.025 kg。
这不是整体1 kg或单侧0.17 kg，也不是从曲线拟合得到的质量。

```text
d_sensor_N = sign * 0.025 * feedback_acceleration_mm_s2 * 0.001
d_display_N = installation_axial_gain * d_sensor_N
fn_processed_N = fn_installed_delta_N - gate * d_display_N
ft_processed_N = ft_original_N
```

预测先建立在传感器层，再通过安装标定斜率转换成显示层的扰动增量。
不加入静态截距，不将解耦力与未解耦力相减。当前代码安装增益为2.85；
历史记录可能是1.913504，回放以该次 `experiment.json` 为准并核验传感器/显示字段。
1000 mm/s²对应传感器预测0.025 N，增益2.85时显示增量0.07125 N。

`ClampDynamics.h` 是实时与离线共用的计算核心。
`ProgrammedDeliveryController.cpp` 逐采样接入；`ExperimentStreamRecorder.cpp` 保存派生结果。
`MainWindow.Causal.cs` 和XAML显示原始力、25 g试算及独立模型2。

## 使用与边界

准备定位前为当前侧选择±1符号，默认+1且始终明确标记未验证；两侧选择独立保存。
正常模式沿用旧操作门控：前向阶段命令边沿触发，5/6/7阶段开启，8/9和终态关闭。
命令WORD仅表征操作事件，不代表夹爪位移、速度或力。

“无器械验证”只让模型在整个采样记录期间开启，包括前向、回程、匀速和最终前向。
必须人工确认无器械、夹爪保持张开、仅轴向运动；点击开始时再次提醒确认。
它不会打开夹爪，也不会改变现有程序的电缸开闭值、轨迹或状态机。
若原程序的参数仍会使夹爪闭合，不能将该次实验视为张开夹爪验证。
不新增独立记录模式或PLC控制入口，不影响电机控制和力反馈。

直接使用当前采样的 `NcToPlc.ActAcc`，仅作mm/s²到m/s²换算。
不使用速度差分兜底、自动降级、居中平滑、递推滤波、未来数据、时间平移或尖峰剔除。
有限的零加速度也按零计算，但不是对反馈有效性的证明；非数、缺失、未取零和时间异常时不补偿并记录原因。
正常有限输入无需预热。相同时间戳、倒退、超过3 ms的间隔或样本序号缺口触发状态重置。
模式切换、重新准备/开始、取零和连接异常均重置；异常后保留时间水位以拒绝反复重复点。
数值有效与物理已验证是不同状态，后者本版始终为false。

模型2读取原始显示力，独立维护旧操作门控。其输出不进入惯性预测、参数选择、误差或残差统计。

## 记录与协议

原始 `samples_1khz.csv`、事件、零点记录的格式和数值生成链路不变。
每次新实验另存 `causal_force.csv`、`causal_model.json`；原有模型2独立文件保留。

- 原派生CSV前17列兼容；`fn_prediction_N` 是门控后的显示层实际应用增量。
- `inertia_N` 是同一实际应用增量的旧列别名，`viscous_N`恒为0，不再存在阻尼配置。
- 新列包含模型版本、质量、符号、人工条件确认、验证模式、反馈与SI加速度、两层未门控预测、安装增益、有效状态和重置原因。
- 模型JSON记录本次配置；传感器斜率、静态截距和零点的完整标定快照仍由 `experiment.json` 和零点文件提供。
- `PROGRAM_PREPARE` 追加上位机字段 `model_sign=1|-1`、`model_validation=0|1`、`model_conditions_confirmed=0|1`。
- `PROGRAM_CURVES` 保留旧曲线行结构，在版本标记 `dynamics_25g` 后追加符号、验证模式、计算状态、重置原因、安装增益。
- 这些字段均不进入PLC ADS写配置，也不改变PLC结构体布局。

PLC采样时间为 `sample_index * 1000 us`，只是名义时间，不是传感器硬件采样时间。
同一PLC周期复制运动反馈和EtherCAT力输入，不证明内部滤波、刷新延迟或物理时间同步。
PLC轴1映射NC Axis 6，PLC轴6映射NC Axis 11；不要将NC Axis 1的角度单位误用于本轴。

## 构建与离线验证

从 `dual_clamp_experiment` 目录执行：

```powershell
& 'D:\Work_software\VS2022\MSBuild\Current\Bin\MSBuild.exe' .\DualClampExperiment.vcxproj /p:Configuration=Debug /p:Platform=x64 /p:OutDir=x64\Debug_inertia25g\ /p:IntDir=obj\Debug_inertia25g\ /m
dotnet build .\AdsControlUI\AdsControlUI.csproj -c Debug -p:Platform=x64 -p:OutputPath=bin\Inertia25g\ -p:IntermediateOutputPath=obj\Inertia25g\
& .\tools\build_inertia_tests.ps1
python .\tools\verify_dynamics.py
python .\tools\replay_clamp_dynamics.py
& .\tools\verify_inertia_ui.ps1
```

构建产物隔离于 `x64/Debug_inertia25g` 和 `AdsControlUI/bin/Inertia25g`，不覆盖原可执行文件。
以上为隔离验证构建。正常部署必须按README中的标准Debug命令同时构建后端和WPF，
避免现有启动器优先找到旧程序；不要将隔离UI与旧后端混用。
本次交付已同步构建标准Debug两端，但未正常启动任何一端。
测试程序只链接计算和文件记录代码，不链接ADS实现，不启动生产入口。
回放也接受显式记录目录，输出到带唯一时间戳的新目录；逐文件SHA-256核验源记录未变。
缺少安装增益、力层级不一致或缺少必需列时直接报错，不猜测参数。
无效加速度样本保留并标记，不自动替换。

界面只能使用 `--curve-replay <ui_fixture.csv> --dynamics-replay --curve-snapshot <png>` 离线验证入口。
可附加 `--validation-replay`、`--negative-sign-replay`、`--model2-replay` 和尺寸参数。
该入口跳过后端启动、管道连接和轮询，关闭时不发送QUIT。
正常启动UI仍沿用既有联网行为，本次验证不使用它。

报告包含原始/处理力、传感器层预测、反馈加速度和速度的同时间轴图。
分段统计阈值仅用于报告，不改变模型、不剔除样本；full回放只是全程门控的计算测试，
不证明历史记录满足无器械或夹爪张开条件。不根据平滑度或峰峰值选择符号或拟合质量。

## 尚未验证

25 g组件与基座同步运动、其惯性经过轴向传感器、拉压与轴坐标符号、反馈加速度的内部处理、
实际传感器时间同步仍需验证。反馈非零不代表这些假设成立。
后续实验由操作者实施，在无器械、夹爪全程张开的条件下做轴向往复。
先比较加减速符号、幅值和时间对应，再比较多个速度、相同方向的重复匀速段残差。
没有可重复的速度相关残差证据前，不增加阻力项。
