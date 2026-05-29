# 机器人力/扭矩传感标定、串扰解耦与主端反馈参数

生成日期：2026-05-28  
适用对象：全维力感知血管介入机器人从端力觉传感与主端力反馈换算  
数据来源：`D:\Work_files\4-27-test\AxialForceConsole\data` 中当前用于 Fig. 9 / Fig. 10 的标定与串扰实验数据

本文档整理实际使用机器人时可能用到的核心数值，包括：

- 轴向力传感器线性标定参数
- 扭矩/切向力通道线性标定参数
- 轴向力-扭矩二维串扰矩阵及其逆矩阵
- 在线使用推荐计算流程
- 主端手柄反馈扭矩换算
- 当前误差、串扰和量程统计

---

## 1. 通道定义与单位约定

### 1.1 从端传感器通道

实际计算中建议先对当前装夹状态进行零点采集，然后使用零点扣除后的原始读数：

```text
f_raw  = 轴向力通道零点扣除后读数，单位 N
ft_raw = 扭矩通道对应的切向拉压传感器零点扣除后读数，单位 N
```

其中：

```text
f_raw  对应 CSV 中的 value0 - zero0，或 F_robot_N
ft_raw 对应 CSV 中的 value1 - zero1，或 F_torque_raw_N
```

扭矩通道的物理来源是切向拉压传感器乘以从端力臂：

```text
L_slave = 37 mm
T_robot = ft_raw × 37
```

由于：

```text
1 N·mm = 1 mN·m
```

所以 `T_robot` 的单位可写为 `N·mm`，也可等价写为 `mN·m`。为了和原始标定脚本一致，本文档中从端扭矩默认写为：

```text
T：mN·m 或 N·mm，二者数值相同
```

### 1.2 主端手柄参数

主端手柄直径：

```text
D_handle = 6 mm
```

主端手柄半径：

```text
r_handle = 3 mm
```

主端反馈扭矩如果使用国际单位制，应换算为：

```text
1 N·mm = 0.001 N·m
1 mN·m = 0.001 N·m
```

---

## 2. 单通道线性标定参数

### 2.1 轴向力标定

实验拟合关系为：

```text
F_ref = 1.913504 × F_robot + 0.183108
```

因此轴向力通道的标定公式为：

```text
F_cal = 1.913504 × f_raw + 0.183108
```

其中：

```text
F_cal：标定后的轴向力，单位 N
f_raw：轴向力通道原始零点扣除后读数，单位 N
```

标定参数：

```text
AXIAL_FIT_K = 1.913504
AXIAL_FIT_B = 0.183108 N
```

如果在机器人实际使用中，每次装夹后都重新采集当前无载零点，则推荐使用增量形式：

```text
ΔF_cal = 1.913504 × Δf_raw
```

这里的截距 `0.183108 N` 会在当前装夹零点扣除时抵消。也就是说，在线力反馈更推荐使用“当前零点下的变化量”，而不是直接把实验室标定截距作为实时反馈偏置输出。

### 2.2 扭矩通道标定

原始扭矩通道先由切向力乘以从端力臂得到：

```text
T_robot = ft_raw × 37
```

实验拟合关系为：

```text
T_robot = 2.006415 × T_ref - 0.041946
```

因此扭矩标定公式为：

```text
T_cal = (T_robot + 0.041946) / 2.006415
```

等价地，直接由切向力 `ft_raw` 计算：

```text
T_cal = 18.440851 × ft_raw + 0.020906
```

其中：

```text
T_cal：标定后的从端扭矩，单位 mN·m，也等价于 N·mm
ft_raw：扭矩通道切向拉压传感器零点扣除后读数，单位 N
```

标定参数：

```text
TORQUE_FIT_K = 2.006415
TORQUE_FIT_B = -0.041946 mN·m
LEVER_ARM_SLAVE = 37 mm

T_CAL_K_FROM_FORCE = 37 / 2.006415 = 18.440851 mN·m/N
T_CAL_B_FROM_FORCE = 0.041946 / 2.006415 = 0.020906 mN·m
```

同样，如果实际机器人每次装夹后都重新采集当前无载零点，推荐在线反馈使用增量形式：

```text
ΔT_cal = 18.440851 × Δft_raw
```

截距 `0.020906 mN·m` 在当前零点扣除后抵消。

---

## 3. 串扰线性关系

在完成单通道标定后，轴向力与扭矩之间仍存在小量线性串扰。用标定后的物理量表示，当前数据得到的有符号串扰系数为：

```text
扭矩 → 轴向力：
a = -0.038004 N/(mN·m)

轴向力 → 扭矩：
b =  0.104006 mN·m/N
```

其物理含义为：

```text
每 1 mN·m 的从端扭矩，会在轴向力通道中引入约 -0.038004 N 的等效变化。
每 1 N 的轴向力，会在扭矩通道中引入约 +0.104006 mN·m 的等效变化。
```

注意：

- 论文图中的串扰百分比使用绝对值统计；
- 实际在线补偿必须保留正负号；
- 该串扰矩阵来自当前装夹与当前实验数据，若传感器、力臂、夹爪或结构改变，需要重新标定。

---

## 4. 二维线性矩阵解耦

把单通道标定后的读数写成：

```text
y = [F_cal, T_cal]^T
```

把解耦后的估计值写成：

```text
x = [F_dec, T_dec]^T
```

串扰模型为：

```text
y = M x
```

其中：

```text
M =
[ 1.000000   -0.038004 ]
[ 0.104006    1.000000 ]
```

展开为：

```text
F_cal = F_dec - 0.038004 × T_dec
T_cal = 0.104006 × F_dec + T_dec
```

这里：

```text
F_cal, F_dec：N
T_cal, T_dec：mN·m 或 N·mm
```

矩阵逆为：

```text
M^-1 =
[  0.996063    0.037854 ]
[ -0.103597    0.996063 ]
```

因此在线解耦公式为：

```text
F_dec =  0.996063 × F_cal + 0.037854 × T_cal
T_dec = -0.103597 × F_cal + 0.996063 × T_cal
```

其中：

```text
F_dec：解耦后的轴向力，单位 N
T_dec：解耦后的从端扭矩，单位 mN·m，也等价于 N·mm
```

由于当前 `a × b = -0.003953`，串扰耦合较弱，精确矩阵逆和一阶近似非常接近。但程序中建议直接使用上述矩阵逆，避免符号和近似误差。

---

## 5. 实际机器人在线使用推荐流程

推荐在线流程如下。

如果当前应用中旋转机构姿态变化会导致传感器自重或线缆重力分量进入 `f_raw` / `ft_raw`，应先执行第 5.1-5.2 节的重力补偿，再进入单通道标定与串扰解耦。

### 5.1 当前装夹零点采集

每次装夹导丝/器械、改变夹持力或重新安装传感器后，先在无外载状态下采集零点：

```text
f_zero  = 当前装夹状态下轴向力通道零点
ft_zero = 当前装夹状态下扭矩/切向力通道零点
```

实时读取：

```text
f_sensor
ft_sensor
```

零点扣除：

```text
Δf_raw  = f_sensor  - f_zero
Δft_raw = ft_sensor - ft_zero
```

### 5.2 可选：按旋转角度进行重力补偿

重力干扰来自传感器、夹持结构或线缆等部件在旋转过程中的重力分量投影。当前已有 5 组 360° full-turn 实验对该项进行了正弦拟合。拟合对象为：

```text
F_tangent_N：扭矩通道切向力，单位 N
F_axis_N：轴向力通道，单位 N
```

因此，重力补偿应放在单通道线性标定之前，即先在原始力通道上扣除角度相关的重力项，再做：

```text
F_cal = 1.913504 × f_raw
T_cal = 18.440851 × ft_raw
```

#### 5.2.1 旋转角度换算

full-turn 重力实验中，旋转电机一圈约为：

```text
rotation_counts_per_revolution = 774000 counts/rev
```

因此：

```text
counts_per_deg = 774000 / 360 = 2150 counts/deg
deg_per_count = 360 / 774000 = 0.000465116 deg/count
```

在线使用时，建议在当前一圈运动或当前零点姿态下记录：

```text
rot_zero_counts
```

然后计算当前姿态角：

```text
theta_deg = (rot_pos_counts - rot_zero_counts) / 2150
```

如果需要限制到 0-360°：

```text
theta_deg = fmod(theta_deg, 360.0)
if (theta_deg < 0) theta_deg += 360.0
```

注意：`AxialForceConsole\analysis_outputs\20260511_092814_rotation_counts_angle_summary.txt` 中曾有一份早期角度标定 `555.56 counts/deg`，但当前 full-turn 扭矩/重力实验及程序配置 `rotation_counts_per_revolution=774000` 对应的是约 `2150 counts/deg`。用于这套重力补偿模型时，应采用当前 full-turn 数据对应的 `2150 counts/deg`。

#### 5.2.2 正向 full-turn 重力模型

当前脚本 `torque_calibration\analyze_gravity.py` 使用 5 组 full-turn 数据，并优先采用 forward 段平均参数。正向模型为：

```text
w = 2π / 360

F_tang_gravity(theta) = 0.4983 × sin(w × theta_deg - 53.54°) - 0.1119
F_axis_gravity(theta) = 0.3195 × sin(w × theta_deg + 48.42°) - 0.1861
```

其中：

```text
F_tang_gravity：切向/扭矩通道重力分量，单位 N
F_axis_gravity：轴向通道重力分量，单位 N
theta_deg：旋转角度，单位 deg
```

5 组 forward 数据的拟合质量：

```text
F_tang_gravity:
  幅值 A = 0.498 ± 0.005 N
  相位 phi = -53.54°
  偏置 offset = -0.112 ± 0.059 N
  平均 R^2 = 0.910
  平均 RMSE = 0.108 N

F_axis_gravity:
  幅值 A = 0.320 ± 0.007 N
  相位 phi = 48.42°
  偏置 offset = -0.186 ± 0.056 N
  平均 R^2 = 0.934
  平均 RMSE = 0.058 N
```

切向通道重力项换算成从端扭矩约为：

```text
原始扭矩幅值 = 0.4983 × 37 = 18.44 N·mm
标定后扭矩幅值 = 0.4983 × 37 / 2.006415 = 9.19 N·mm
```

这说明重力项对扭矩通道并不小；如果机器人在实际使用中姿态角会变化，建议启用重力补偿或至少进行姿态固定下的当前零点扣除。

#### 5.2.3 零点角度下的增量式重力补偿

实际在线反馈不建议直接扣完整正弦模型的绝对值，因为当前装夹零点已经包含某个角度下的重力偏置。更稳妥的方式是扣除“当前角度相对零点角度的重力变化量”：

```text
theta0_deg = 零点采集时的旋转角度
theta_deg  = 当前旋转角度

ΔF_tang_gravity = F_tang_gravity(theta_deg) - F_tang_gravity(theta0_deg)
ΔF_axis_gravity = F_axis_gravity(theta_deg) - F_axis_gravity(theta0_deg)

ft_raw_gcorr = Δft_raw - ΔF_tang_gravity
f_raw_gcorr  = Δf_raw  - ΔF_axis_gravity
```

然后用 `ft_raw_gcorr` 和 `f_raw_gcorr` 进入单通道标定：

```text
F_cal = 1.913504 × f_raw_gcorr
T_cal = 18.440851 × ft_raw_gcorr
```

如果实际使用时旋转姿态固定，或者每次姿态变化后都会重新采集当前零点，则可暂时不启用正弦重力补偿。

#### 5.2.4 反向 full-turn 拟合结果，仅供参考

反向段也进行了拟合，但参数与 forward 段不同，说明回差、线缆拖拽或运动方向相关因素会影响重力/姿态项。反向平均结果为：

```text
F_tang_gravity reverse:
  A = 0.436 ± 0.031 N
  phi = -85.77°
  offset = 0.136 ± 0.067 N
  平均 R^2 = 0.983

F_axis_gravity reverse:
  A = 0.493 ± 0.014 N
  phi = 151.21°
  offset = -0.180 ± 0.068 N
  平均 R^2 = 0.988
```

如果未来实际机器人存在频繁正反向旋转，建议重新设计一组专门的重力/线缆补偿实验，分别建立 forward/reverse 模型，或把角度、方向、线缆状态作为补偿模型输入。

### 5.3 单通道增量标定

```text
F_cal = 1.913504 × f_raw_gcorr
T_cal = 18.440851 × ft_raw_gcorr
```

如果不启用重力补偿，则令：

```text
f_raw_gcorr  = Δf_raw
ft_raw_gcorr = Δft_raw
```

此时 `F_cal` 和 `T_cal` 是当前装夹零点下的力/扭矩变化量。

### 5.4 串扰矩阵解耦

```text
F_dec =  0.996063 × F_cal + 0.037854 × T_cal
T_dec = -0.103597 × F_cal + 0.996063 × T_cal
```

### 5.5 C/C++ 伪代码

```cpp
// Runtime zeroed raw sensor readings
double df_raw  = f_sensor  - f_zero;   // N
double dft_raw = ft_sensor - ft_zero;  // N, tangential force channel

// Optional gravity compensation
double theta_deg = std::fmod((rot_pos_counts - rot_zero_counts) / 2150.0, 360.0);
if (theta_deg < 0.0) theta_deg += 360.0;

double theta0_deg = 0.0; // angle at zeroing, or store the actual value
double w = 2.0 * M_PI / 360.0;

auto ft_gravity = [&](double deg) {
    return 0.4983 * std::sin(w * deg + (-53.54 * M_PI / 180.0)) - 0.1119;
};
auto fa_gravity = [&](double deg) {
    return 0.3195 * std::sin(w * deg + (48.42 * M_PI / 180.0)) - 0.1861;
};

double dft_g = ft_gravity(theta_deg) - ft_gravity(theta0_deg);
double dfa_g = fa_gravity(theta_deg) - fa_gravity(theta0_deg);

double dft_raw_gcorr = dft_raw - dft_g;
double df_raw_gcorr  = df_raw  - dfa_g;

// Single-channel calibration, increment form
double F_cal = 1.913504 * df_raw_gcorr;       // N
double T_cal = 18.440851 * dft_raw_gcorr;     // mN·m == N·mm

// Crosstalk decoupling
double F_dec =  0.996063 * F_cal + 0.037854 * T_cal;   // N
double T_dec = -0.103597 * F_cal + 0.996063 * T_cal;   // mN·m == N·mm
```

如果程序必须复现实验室绝对标定值，也可以先使用带截距公式：

```cpp
double F_abs = 1.913504 * df_raw + 0.183108;
double T_abs = 18.440851 * dft_raw + 0.020906;
```

但用于实时力反馈时，仍建议在当前装夹状态下减去无载基线，使无外载时反馈输出为零。

---

## 6. 主端手柄反馈扭矩换算

### 6.1 从端解耦扭矩

经过矩阵解耦后得到：

```text
T_dec：从端解耦扭矩，单位 N·mm，也等价于 mN·m
```

由于：

```text
1 N·mm = 0.001 N·m
```

如果主端控制器需要标准 SI 单位 `N·m`，则：

```text
T_dec_Nm = T_dec_Nmm × 0.001
```

### 6.2 按主端手柄半径进行反馈扭矩映射

从端扭矩通道的力臂：

```text
L_slave = 37 mm
```

主端手柄半径：

```text
r_handle = 3 mm
```

如果按照“同等切向力映射到主端手柄半径”的方式计算主端反馈扭矩，则：

```text
T_feedback_Nmm = T_dec_Nmm × r_handle / L_slave
```

代入数值：

```text
T_feedback_Nmm = T_dec_Nmm × 3 / 37
```

也就是：

```text
T_feedback_Nmm = 0.081081 × T_dec_Nmm
```

换算为 `N·m`：

```text
T_feedback_Nm = T_feedback_Nmm × 0.001
```

合并后：

```text
T_feedback_Nm = T_dec_Nmm × 3 / 37 × 0.001
```

即：

```text
T_feedback_Nm = 0.000081081 × T_dec_Nmm
```

### 6.3 这个 `×3/37` 的物理含义

由于从端扭矩来自：

```text
T_dec ≈ F_tangent_equiv × 37 mm
```

等效切向力为：

```text
F_tangent_equiv = T_dec / 37
```

如果把这个等效切向力作用到主端半径 `3 mm` 的手柄上，则主端反馈扭矩为：

```text
T_feedback = F_tangent_equiv × 3
           = T_dec × 3 / 37
```

因此，`×3/37` 对应的是：

```text
保持等效切向力一致，并根据主端手柄半径重新计算反馈扭矩。
```

它不是“等扭矩再现”。如果目标是让主端手柄感受到与从端相同大小的扭矩，则不应乘 `3/37`，而应直接使用：

```text
T_feedback_Nmm = T_dec_Nmm
T_feedback_Nm  = T_dec_Nmm × 0.001
```

如果目标是让主端手柄表面产生该反馈扭矩所需的切向力，则：

```text
F_handle = T_feedback_Nmm / 3
```

对于当前采用 `×3/37` 的半径映射：

```text
F_handle = (T_dec_Nmm × 3 / 37) / 3
         = T_dec_Nmm / 37
```

也就是说，主端手柄表面的切向反馈力等于从端 37 mm 力臂下的等效切向力。

### 6.4 加入反馈增益

实际系统中通常还会加入主从反馈增益 `K_feedback`：

```text
T_feedback_Nmm = K_feedback × T_dec_Nmm × 3 / 37
T_feedback_Nm  = K_feedback × T_dec_Nmm × 3 / 37 × 0.001
```

若没有额外缩放：

```text
K_feedback = 1
```

建议程序中保留该增益，便于安全限幅和主观手感调节。

### 6.5 示例

若从端解耦扭矩为：

```text
T_dec = 6.4 N·mm
```

按主端手柄半径映射：

```text
T_feedback = 6.4 × 3 / 37 = 0.519 N·mm
```

换算为 `N·m`：

```text
T_feedback = 0.519 × 0.001 = 0.000519 N·m
```

如果主端控制器需要的是手柄表面切向力：

```text
F_handle = 0.519 / 3 = 0.173 N
```

等价于：

```text
F_handle = 6.4 / 37 = 0.173 N
```

---

## 7. 量程与限幅建议

### 7.1 原始传感器量程

```text
轴向力原始传感器量程：±5 N
扭矩通道切向拉压传感器量程：±5 N
```

从端扭矩原始量程：

```text
T_robot_range = ±5 N × 37 mm = ±185 N·mm
```

也就是：

```text
±185 mN·m
```

经扭矩线性标定后，`ft_raw = ±5 N` 约对应：

```text
T_cal ≈ ±92.2 N·mm
```

### 7.2 轴向力校正后范围

按最终轴向线性标定：

```text
f_raw = -5 N → F_cal = -9.384 N
f_raw = +5 N → F_cal = +9.751 N
```

因此可近似认为校正后轴向通道单侧量程为：

```text
约 ±9.68 N
```

### 7.3 主端反馈限幅

若按 `×3/37` 主端半径映射，且 `K_feedback = 1`，则：

```text
T_feedback_Nmm = T_dec_Nmm × 0.081081
```

如果从端扭矩达到标定后约 `92.2 N·mm`，主端反馈扭矩约为：

```text
T_feedback ≈ 92.2 × 3 / 37 = 7.48 N·mm
```

换算为：

```text
T_feedback ≈ 0.00748 N·m
```

对应手柄表面切向力：

```text
F_handle = 7.48 / 3 = 2.49 N
```

实际系统中建议根据主端执行器能力、操作者舒适性和安全性设置：

```text
T_feedback_max
F_handle_max
K_feedback
```

---

## 8. 当前保守误差统计

以下结果采用更保守的原始稳定采样点/单组加载点回测口径，而不是图中均值曲线口径。

### 8.1 轴向力标定后误差

统计对象：

```text
93 个原始 SNAPSHOT 点
```

结果：

```text
平均绝对误差 = 0.142 ± 0.075 N
RMSE = 0.160 N
最大绝对误差 = 0.290 N
```

### 8.2 扭矩标定后误差

统计对象：

```text
27 个每组质量点均值
```

结果：

```text
平均绝对误差 = 0.143 ± 0.155 N·mm
RMSE = 0.208 N·mm
最大绝对误差 = 0.659 N·mm
```

等价写法：

```text
平均绝对误差 = 0.143 ± 0.155 mN·m
RMSE = 0.208 mN·m
最大绝对误差 = 0.659 mN·m
```

---

## 9. 当前串扰统计

串扰百分比按单侧量程计算：

```text
轴向力 → 扭矩通道：除以 ±5 N 中的 5 N
扭矩 → 轴向力通道：除以校正后轴向单侧量程 9.68 N
```

### 9.1 轴向力 → 扭矩通道串扰

计算方式：

```text
percent = |F_torque_raw_N| / 5 × 100%
```

保守原始 SNAPSHOT 口径：

```text
平均串扰 = 0.216 ± 0.135% F.S.
RMS = 0.254% F.S.
最大串扰 = 0.482% F.S.
```

### 9.2 扭矩 → 轴向力通道串扰

计算方式：

```text
每组实验先减去 0g/no-load 基线：
ΔF_cal = F_cal(current mass) - F_cal(0g)

percent = |ΔF_cal| / 9.68 × 100%
```

使用最终轴向标定公式重新计算后的保守口径：

```text
平均串扰 = 1.219 ± 0.806% F.S.
RMS = 1.453% F.S.
最大串扰 = 2.958% F.S.
```

该结果与旧图脚本中使用 `1.919386` 轴向校正系数得到的 `1.223 ± 0.809% F.S.` 非常接近。实际机器人在线使用建议统一采用本文档中的最终标定系数：

```text
F_cal = 1.913504 × f_raw + 0.183108
```

---

## 10. 推荐保存为程序常量的参数

```cpp
// Geometry
constexpr double SLAVE_LEVER_MM = 37.0;
constexpr double HANDLE_DIAMETER_MM = 6.0;
constexpr double HANDLE_RADIUS_MM = 3.0;
constexpr double HANDLE_RADIUS_RATIO = HANDLE_RADIUS_MM / SLAVE_LEVER_MM; // 0.081081081
constexpr double ROTATION_COUNTS_PER_REV = 774000.0;
constexpr double ROTATION_COUNTS_PER_DEG = ROTATION_COUNTS_PER_REV / 360.0; // 2150

// Axial force calibration
constexpr double AXIAL_K = 1.913504;
constexpr double AXIAL_B_N = 0.183108;

// Torque calibration
constexpr double TORQUE_K = 2.006415;
constexpr double TORQUE_B_MN_M = -0.041946;
constexpr double TORQUE_FROM_TANGENTIAL_FORCE = SLAVE_LEVER_MM / TORQUE_K; // 18.440851

// Crosstalk matrix inverse
constexpr double DECOUPLE_FF =  0.996063;
constexpr double DECOUPLE_FT =  0.037854;
constexpr double DECOUPLE_TF = -0.103597;
constexpr double DECOUPLE_TT =  0.996063;

// Gravity compensation, forward full-turn model
constexpr double GRAV_FT_A_N = 0.4983;
constexpr double GRAV_FT_PHI_DEG = -53.54;
constexpr double GRAV_FT_OFFSET_N = -0.1119;
constexpr double GRAV_FA_A_N = 0.3195;
constexpr double GRAV_FA_PHI_DEG = 48.42;
constexpr double GRAV_FA_OFFSET_N = -0.1861;

// Unit conversion
constexpr double NMM_TO_NM = 1e-3;
```

在线计算示例：

```cpp
double df_raw = f_sensor - f_zero;       // N
double dft_raw = ft_sensor - ft_zero;    // N

double theta_deg = std::fmod((rot_pos_counts - rot_zero_counts) / ROTATION_COUNTS_PER_DEG, 360.0);
if (theta_deg < 0.0) theta_deg += 360.0;

double theta0_deg = 0.0; // use actual zeroing angle if not zero
double w = 2.0 * M_PI / 360.0;
double ft_g = GRAV_FT_A_N * std::sin(w * theta_deg + GRAV_FT_PHI_DEG * M_PI / 180.0) + GRAV_FT_OFFSET_N;
double ft_g0 = GRAV_FT_A_N * std::sin(w * theta0_deg + GRAV_FT_PHI_DEG * M_PI / 180.0) + GRAV_FT_OFFSET_N;
double fa_g = GRAV_FA_A_N * std::sin(w * theta_deg + GRAV_FA_PHI_DEG * M_PI / 180.0) + GRAV_FA_OFFSET_N;
double fa_g0 = GRAV_FA_A_N * std::sin(w * theta0_deg + GRAV_FA_PHI_DEG * M_PI / 180.0) + GRAV_FA_OFFSET_N;

double dft_raw_gcorr = dft_raw - (ft_g - ft_g0);
double df_raw_gcorr = df_raw - (fa_g - fa_g0);

double F_cal = AXIAL_K * df_raw_gcorr;                         // N
double T_cal = TORQUE_FROM_TANGENTIAL_FORCE * dft_raw_gcorr;   // N·mm == mN·m

double F_dec = DECOUPLE_FF * F_cal + DECOUPLE_FT * T_cal; // N
double T_dec = DECOUPLE_TF * F_cal + DECOUPLE_TT * T_cal; // N·mm

double T_feedback_Nmm = feedback_gain * T_dec * HANDLE_RADIUS_RATIO;
double T_feedback_Nm  = T_feedback_Nmm * NMM_TO_NM;
```

---

## 11. 使用注意事项

1. 每次装夹、改变夹持力或更换器械后，应重新采集当前无载零点。
2. 在线反馈建议使用增量形式，即截距在当前零点扣除中抵消。
3. 串扰矩阵适用于当前实验装夹和结构状态；机械结构或传感器布置改变后需要重标定。
4. 扭矩单位在程序内部建议统一为 `N·mm`，输出给需要 SI 单位的主端控制器时再转换为 `N·m`。
5. `×3/37` 是按主端手柄半径进行的反馈扭矩映射，不是等扭矩再现。
6. 主端反馈方向需要通过实际手柄运动方向验证；如果操作者感觉方向相反，应调整反馈输出符号，而不是改动标定参数本身。
7. 建议保留 `feedback_gain` 和限幅参数，用于调节主观手感并保证安全。
