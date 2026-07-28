# PLC 状态机与流程说明 / 项目内维护文档

> 本文档梳理 PLC 侧 `Untitled2` 项目内**所有 POU 的详细逻辑**：状态跳转、执行流程、参数含义与调参边界。
>
> 修改 POU 内部逻辑、状态跳转条件、或 `state.TcDUT` 枚举，请同步在最末"变更日志"追加条目。

更新时间：2026-07-28
适用工程：`250902\Untitled2`
对应代码版本：2026-07-28 主分支（基于 `MAIN.TcPOU` / `ArmManual.TcPOU` / `init.TcPOU` / `SelfCheck.TcPOU` / `handle.TcPOU` / `err.TcPOU` / `reset.TcPOU` / `clear_err.TcPOU` 当前状态校对）

***

## 1. 顶层调度（`MAIN.TcPOU`）

### 1.1 结构

```
PROGRAM MAIN
VAR
    init_       : init;
    handle1_    : handle;
    clear_err_  : clear_err;
    err_        : err;
    reset_      : reset;
    self_check_ : SelfCheck;
    arm_manual_ : ArmManual;
    i           : INT;
END_VAR
```

MAIN 是 PLC 唯一的顶层 `PROGRAM`，每个任务周期（1 ms）执行以下顺序：

```
1) FOR i:=1 TO 7 DO G.axis[i](); END_FOR    -- 刷新 7 个轴对象
2) IF FirstCycle THEN                        -- 首周期复位全局
      G.gen_state := _init;
      G.self_check_done := FALSE;
      G.handle_reinit_req := FALSE;
      G.handle_reinit_done := FALSE;
      G.estop_hold_req := TRUE;
      G.selfcheck_reset_req := TRUE;
      G.arm_manual_enable := FALSE;
      清 arm_enable/reset/jog 请求
      清 SetPointGenEnable/Disable/axis_Reset 的 Execute
   END_IF
3) arm_manual_();      -- 定位臂 5 轴独立点动，不参与 gen_state
4) POWERED();          -- Action：对原机器人 7 轴调用 MC_Power (FBD)
5) SETPOSITIONGEN();   -- Action：对原机器人 7 轴调用 MC_ExtSetPointGenEnable/Disable (FBD)
6) RESETED();          -- Action：对原机器人 7 轴调用 MC_Reset (FBD)
7) CASE G.gen_state OF
      _init:       init_();
      _self_check: self_check_();
      _handle:     handle1_();
      _clear_err:  clear_err_();
      _err:        err_();
      _reset:      reset_();
   END_CASE
```

### 1.2 Action 说明

三个 Action 是 FBD 网络，功能等价于以下 ST 伪码：

- **POWERED**：对每个 i，调用 `G.power_[i](Axis:=G.axis[i], Enable:=G.power_[i].Enable, Enable_Positive:=TRUE, Enable_Negative:=TRUE, ...)`，输出到 `G.power_output[i]`。
- **SETPOSITIONGEN**：对每个 i，调用 `G.SetPointGenEnable[i]` 与 `G.SetPointGenDisable[i]` 两个 FB，输出到 `G.SetPointGenEnable_output[i]` / `G.SetPointGenDisable_output[i]`。**执行触发由具体 POU 通过 `.Execute` 决定**，Action 本身只是把 FB 实例串到轴对象上。
- **RESETED**：对每个 i，调用 `G.axis_Reset[i](Axis:=G.axis[i], Execute:=G.axis_Reset[i].Execute)`，输出到 `G.reset_output[i]`。

**关键理解**：这三个 Action 每周期都会被调用（不受状态影响），但**是否触发**由各 POU 通过设置 `.Execute` 决定。这样 POU 只需要写标志位、不需要自己实例化 FB。

### 1.3 定位臂独立调度（`ArmManual.TcPOU`）

`ArmManual` 是新增定位臂 5 轴的独立功能块，放在 `MAIN` 首周期初始化之后、原 7 轴 `POWERED/SETPOSITIONGEN/RESETED` Action 之前调用。它只操作 `G.arm_axis[1..5]` 和 `G.arm_*` 变量，不读写 `G.refer[1..7]`、`G.Act_pos[1..7]`、`G.return_cmd[1..7]`，也不改变 `G.gen_state`。

每周期执行逻辑：

1. 调用 `G.arm_axis[i]()` 并刷新 `G.arm_act_pos[i] / G.arm_act_vel[i]`。
2. 按 `G.arm_enable_req[i]` 调用 `MC_Power`，输出到 `G.arm_power_output[i]`。
3. 对 `G.arm_reset_req[i]` 采用消费式命令：请求置位后触发 `MC_Reset`，完成或报错后 PLC 自动清 `G.arm_reset_req[i]`。
4. 只有 `G.arm_manual_enable=TRUE`、对应轴已上电、无 reset busy、无命令冲突时才允许 `MC_MoveVelocity`。
5. `G.arm_jog_pos_req[i]` 与 `G.arm_jog_neg_req[i]` 同时为 TRUE 时，置 `G.arm_cmd_conflict[i]=TRUE`，若正在运动则先 `MC_Stop`。
6. 点动方向变化时先停止，再在后续周期启动反向运动；松开按钮时撤销 MoveVelocity 并执行 Stop。

定位臂首版不做自动回零、自检、软限位和正运动学；机械限位/驱动限位由 NC/驱动配置兜底。定位臂错误通过 `G.arm_motion_error[] / G.arm_motion_error_id[]` 单独报告，不切换原介入机器人主状态机。

### 1.4 状态跳转矩阵

| 当前态 | 触发条件 | 目标态 | 触发方 |
|---|---|---|---|
| — | PLC 首周期 | `_init` | MAIN |
| `_init` | 7 轴 `power_output.Done` 全部为真 | `_self_check` | `init` |
| `_init` | 上电超时 5s 或任一轴错误 | `_err` | `init` |
| `_self_check` | 全 7 轴 `stu = 10` | `_handle` | `SelfCheck`（末尾） |
| `_handle` | 掉使能且检测到错误 | `_err` | `handle` |
| `_handle` | 单纯掉使能（无错误） | 保持 `_handle`，置 `handle_reinit_req` | `handle` |
| `_err` | 无错误 & 全轴 powered | `_clear_err` | `err` |
| `_clear_err` | 有错误 | `_err` | `clear_err` |
| `_clear_err` | 无错误 & 未自检完成 | `_init` | `clear_err` |
| `_clear_err` | 无错误 & 已自检 & 已上电 & 稳定 200ms | `_handle`（含 `handle_reinit_req`） | `clear_err` |
| `_reset` | 全部 FB Done/Error | `_init` 或 `_err`（视错误状态） | `reset` |

`_reset` 在当前主链路中**不会被自动跳入**，`err` POU 直接切 `_clear_err`。`_reset` 作为可选路径保留，供外部手动触发。

## 2. `init.TcPOU` — 上电初始化

### 2.1 职责

将 7 个轴上电（`MC_Power.Enable := TRUE`），等待所有轴 `power_output.Done`；成功后记录当前绝对位置作为 `init_pos[]`（临时零点），跳转 `_self_check`。

### 2.2 状态机

```
step = 0: 进入初始化
  - 清 SetPointGenEnable/Disable/axis_Reset 的 Execute
  - G.power_[i].Enable := TRUE
  - G.self_check_done := FALSE
  - G.startup_loading_ready := FALSE
  - G.selfcheck_reset_req := TRUE
  - 启动看门狗 (5s)
  - step := 1

step = 1: 等待上电完成
  - 保持 power_.Enable := TRUE
  - 若任一轴 Status.Error 或 power_output.Error → _err
  - 若看门狗超时 → _err
  - 若全部 power_output.Done → 
        FOR i: G.init_pos[i] := G.axis[i].NcToPlc.ActPos
        step := 0
        G.gen_state := _self_check
```

### 2.3 参数与调参边界

| 参数 | 值 | 修改建议 |
|---|---|---|
| `init_watchdog` PT | `T#5S` | 若伺服上电较慢（如带热身），可增大 |

### 2.4 注意事项

- **`init_pos` 在此处只是"临时零点"**：后续 `SelfCheck` 完成后会用左限位工作位覆盖 `init_pos`。
- 一旦跳到 `_err`，`init.step` 已复位到 0，下次经 `_clear_err` 回来重新走一遍。

## 3. `SelfCheck.TcPOU` — 自检寻参

### 3.1 职责

以左限位为参考建立各平移轴的绝对坐标基准，然后回到工作位并交给 `_handle`：

1. 对目标平移轴（1/3/5/6）：低速逼近左限位 → 记录 `G.leftlimit[i]` → 停止 → 清错 → `MC_SetPosition(ActPos → SetPos)` → 回退到工作位。
2. 对非目标轴（2/4/7）：直接 `MC_MoveAbsolute` 到绝对位置 0。
3. 全部完成后置 `G.self_check_done := TRUE`、`G.startup_loading_ready := TRUE`、`G.handle_reinit_req := TRUE`，用 ActPos 刷新 `Act_pos / refer / ref_slow / act_h / act_hf`，跳 `_handle`。

### 3.2 每轴子状态（`stu[i]`）

```
0  WaitDisable       禁用 SetPointGen
1  StartScan          错峰启动 + 记 scan_start_pos + 启动 MC_MoveVelocity
2  ScanMonitor        监控逼近 → 到限位/堵转 → 记 G.leftlimit → MC_Stop
3  WaitStop           等待停稳 → 判断错误 → 分支到 4/55/6
4  ResetTrigger       触发 MC_Reset
5  WaitReset          等待复位 → 分支到 6/55
55 RepowerPulse       重上电脉冲（MC_Power.Enable 短暂 FALSE）
56 RepowerWait        等待重上电就绪
6  SyncWait           同步等待（全部目标轴到达 stu>=6 后放行）
7  SetPosStart        触发 MC_SetPosition (SetPos := ActPos)
71 SetPosWait         等待 SetPosition 完成
72 BackoffDelay       错峰回位调度（轴6 先，其他按 stagger_delay）
8  BackoffMove        等待 MC_MoveAbsolute 完成
10 Done               以回退终点作为 init_pos[i]
```

### 3.3 目标轴与错峰参数

| 参数 | 值 | 含义 |
|---|---|---|
| `target_axes` | `[T, F, T, F, T, T, F]` | 参与寻参的轴（1/3/5/6） |
| `vel_scan` | `30.0` | 逼近速度，单位随轴 |
| `vel_back` | `50.0` | 回位速度 |
| `init_target_from_left` | `[96, 0, 280, 0, 430, 580, 0]` | 回位目标（距左限位） |
| `scan_stagger_delay` | `[0, 0, 300, 0, 600, 900, 0] ms` | 逼近启动错峰（stu=1） |
| `stagger_delay` | `[3, 0, 2, 0, 1, 0, 0] s` | 回位启动错峰（stu=72） |
| `dist_scan_large` / `target_pos_large` | `1000.0` | 保留，未使用 |

**错峰逻辑**（stu=72）：
- 轴6 **立即启动回退**并置 `stagger_started := TRUE` + `stagger_timer(IN:=FALSE)` 复位；
- 其他目标轴（1/3/5）在 `stagger_timer.ET >= stagger_delay[i]` 后放行；
- 因此按时间顺序：**轴6（0s）→ 轴5（1s）→ 轴3（2s）→ 轴1（3s）**。

其中 axis1/3/5/6 的 `[96,280,430,580] mm` 也是上位机定义的**标准器械装卸等待姿态**。SelfCheck 全部完成时 PLC 才置 `G.startup_loading_ready=TRUE`；该位不能代替实际位置校验。

### 3.4 到限位判据（stu=2）

```
Busy = TRUE AND ABS(ActVelo) < 0.5 AND (
    ABS(SetVelo) > 5.0
    OR (ABS(SetVelo) < 0.5 AND ABS(ActPos - scan_start_pos) > 1.0)
) 持续 100ms
```

同时 `fb_move_vel.Error` 也会立刻触发退出。这是为了同时兼容"电流限位"（SetVelo 还在跑但实际不动）和"位置误差跳错"（SetVelo 归零但已移动了一小段）两种情况。

### 3.5 电缸预设

自检期间：
- stu=1 且 i=1（错峰阶段第一次进入）：写一次 `cylinder1=1000 / cylinder2=0 / cylinder3=1000 / cylinder4=0`（导管夹紧、导丝夹紧）。
- 每周期顶部固定：`cylinder5_value := 2000`、`cylinder5_cmd := 2000`、`cylinder5_press_req := FALSE`（Y 阀打开）。
- 全部完成后（`all_done`）再写一次 `cylinder1=1000 / cylinder2=0 / cylinder3=1000 / cylinder4=0`，确保切 handle 时电缸组合固定。

### 3.6 重初始化

如果 `G.selfcheck_reset_req = TRUE`（由 `init` 拉起），本 POU 首行会：
- 清 `G.startup_loading_ready`，撤销上一轮标准装卸启动资格；
- 清空所有 `stu[]`、动作变量、计时器、FB Execute；
- 清 `return_cmd[]`、`axis4_manual_*`；
- 清 `G.selfcheck_reset_req`。

### 3.7 完成动作

```
IF all_done THEN
    cylinder1..4 写固定组合;
    G.self_check_done := TRUE;
    G.startup_loading_ready := TRUE;
    G.handle_reinit_req := TRUE;
    FOR i:=1 TO 7 DO
        G.Act_pos[i] := ActPos - G.init_pos[i];
        G.act_pos_from_left[i] := ActPos - G.leftlimit[i];
        G.refer[i] := G.Act_pos[i];
        G.ref_slow[i] := G.Act_pos[i];
        G.act_h[i] := G.Act_pos[i];
        G.act_hf[i] := G.Act_pos[i];
        G.refer_from_left[i] := G.act_pos_from_left[i];
    END_FOR
    G.gen_state := _handle;
END_IF
```

**注意**：这里没有覆盖 `G.init_pos[i]`——`stu=10` 内已经写过 `G.init_pos[i] := ActPos`。切 `_handle` 时，`G.Act_pos = 0` 就是"当前工作位"。`G.startup_loading_ready` 是一次性资格位：上位机启动准备/直接控制入口会写 FALSE；若 PLC 未重启而机构已由上一轮上位机移离装卸位，下次启动将改走中断恢复路径。

## 4. `handle.TcPOU` — 正常跟随控制

`handle` 是 PLC 主链路的核心。分为**四大功能块**：

1. **重初始化握手**（响应 `G.handle_reinit_req`）
2. **掉使能/错误保持**（`hold_active` 分支）
3. **参考位置平滑与设定点输出**（速度限幅 + 滑动平均 + 二阶滤波 + `MC_ExtSetPointGenFeed`）
4. **计划回退 FB**（`return_state[i]`）+ **axis4 手动点动**（`axis4_manual_state`）

### 4.1 重初始化握手

```
IF G.handle_reinit_req THEN
    init_done := FALSE;
    G.handle_reinit_done := FALSE;
    G.handle_reinit_req := FALSE;
    G.estop_hold_req := TRUE;
    重置 ext_setpoint_enabled / return_state / return_cmd /
         refer_interval/tick/progress / traj_pos/vel/acc /
         axis4_manual_state / axis4_fwd/rev_req / axis4_manual_busy/done/error;
END_IF

IF NOT init_done THEN
    FOR i: G.SetPointGenEnable[i].Execute := TRUE
    IF 全部 SetPointGenEnable.Done THEN
        FOR j:
            G.Act_pos[j] := ActPos - init_pos[j];
            G.act_pos_from_left[j] := ActPos - leftlimit[j];
            G.ref_slow[j] := G.act_h[j] := G.act_hf[j] := G.refer[j] := G.Act_pos[j];
            G.refer_from_left[j] := G.act_pos_from_left[j];
            用 Act_pos 填充 act_hh[j][1..30];
            act_h_prev[j] := refer_prev[j] := refer_interp[j] := Act_pos;
            traj_pos[j] := Act_pos; traj_vel := traj_acc := 0;
            IF slimit_enable[j]: right_slimit[j] := leftlimit[j] + rslimit_distance[j];
        END_FOR
        init_done := TRUE;
        G.handle_reinit_done := TRUE;
    END_IF
    RETURN;
END_IF
```

**关键**：`init_done` 只保证过一次 `SetPointGenEnable.Done`；后续通过 `all_powered / has_axis_error` 判定是否需要再次进入。

### 4.2 掉使能/错误保持

```
all_powered := 全 power_output.Done;
has_axis_error := 任一 axis.Status.Error / power_output.Error / reset_output.Error;

IF NOT all_powered THEN
    G.estop_hold_req := TRUE;
    hold_active := TRUE;
    重置 Act_pos / refer / ref_slow / act_h / act_hf / act_hh / act_h_prev /
         traj_pos/vel/acc 到当前实际位置;
    IF has_axis_error: G.gen_state := _err;
    RETURN;
END_IF

IF hold_active THEN     -- 从 hold 恢复
    hold_active := FALSE;
    G.handle_reinit_req := TRUE;
    G.estop_hold_req := TRUE;
    IF has_axis_error: G.gen_state := _err;
    RETURN;
END_IF

IF has_axis_error THEN
    G.estop_hold_req := TRUE;
    G.gen_state := _err;
    RETURN;
END_IF

G.estop_hold_req := FALSE;
```

**语义**：任何时候检测到掉使能，立刻把所有 refer/平滑/二阶滤波状态**同步到当前实际位置**（防止使能恢复瞬间跳变），并通过 `hold_active` 边沿在下一周期请求重同步。

### 4.3 电缸5 每周期映射

```
IF G.cylinder5_press_req THEN
    G.cylinder5_value := 0;      -- 按下 → 夹紧
ELSE
    G.cylinder5_value := 2000;   -- 释放 → 打开
END_IF
```

这是 Y 阀操作的语义包装，见 `PLC项目说明.md §3.2`。

### 4.4 参考平滑三段

对每个轴 `j = 1..7` 每周期依次：

**步骤 1：判断是否处于回退接管旁路**
```
return_handoff_bypass := ((j=1) AND (G.axis1_fast_return OR G.return_cmd[1].Busy))
    OR ((j=6) AND (G.axis6_fast_retract OR G.return_cmd[6].Busy));

IF return_handoff_bypass:
    avg_window := 1; ramp_counter[j] := 0;
ELSIF G.startup_smoothing_bypass:
    ramp_counter[j] := MIN(ramp_counter+1, avg_window_legacy=20);
    avg_window := MAX(1, ramp_counter[j]);
ELSE:
    ramp_counter[j] := MIN(ramp_counter+1, avg_window_follow=20);
    avg_window := MAX(1, ramp_counter[j]);
```

`ramp_counter` 让 `avg_window` 在启动/恢复瞬间**从 1 缓升到 20**，避免直接用满窗口引入的滞后。

**步骤 2：右软限位**
```
IF slimit_enable[j]:
    IF (ActPos >= right_slimit[j]) AND (G.refer[j] > G.ref_slow[j]):
        G.refer[j] := G.ref_slow[j];   -- 禁止继续增大
```

**步骤 3：速度限幅**
```
v_step := G.v_limit[j];
IF return_handoff_bypass:
    G.ref_slow[j] := G.refer[j];             -- 快退直通
ELSIF (G.refer - G.ref_slow) > v_step:
    G.ref_slow += v_step;
ELSIF (G.refer - G.ref_slow) < -v_step:
    G.ref_slow -= v_step;
ELSE:
    G.ref_slow := G.refer;
```

**步骤 4：滑动平均**
```
FOR k:=2 TO 20 DO act_hh[j][22-k] := act_hh[j][21-k];  -- 队列右移
IF return_handoff_bypass:
    FOR i:=1 TO 20: act_hh[j][i] := G.ref_slow[j];     -- 全部填成当前值
    G.act_h[j] := G.ref_slow[j];
ELSE:
    act_hh[j][1] := G.ref_slow[j];
    G.act_h[j] := SUM(act_hh[j][1..avg_window]) / avg_window;
```

**步骤 5：二阶滤波（仅对 `traj_enable[j] = TRUE` 的平移轴）**
```
IF traj_enable[j] AND NOT return_handoff_bypass AND NOT G.startup_smoothing_bypass:
    acc_raw := traj_wn² × (G.refer[j] - traj_pos[j]) - 2 × traj_wn × traj_vel[j];
    钳位 acc_raw 到 ±traj_amax[j];
    traj_acc[j] := acc_raw;
    traj_vel[j] += 0.001 × traj_acc;    -- dt=1ms
    钳位 traj_vel 到 ±traj_vmax[j];
    traj_pos[j] += 0.001 × traj_vel;
    IF 收敛（|refer-pos|<0.001 且 |vel|<0.01）:
        traj_pos := G.refer; traj_vel := 0; traj_acc := 0;
ELSE:
    traj_pos[j] := G.act_h[j]; traj_vel := 0; traj_acc := 0;
```

**参数**：
| 参数 | 值 | 含义 |
|---|---|---|
| `traj_wn` | `60.0 rad/s` | 二阶带宽（同轴共用） |
| `traj_vmax` | `[80, 0, 80, 0, 80, 80, 0]` | 速度上限（mm/s） |
| `traj_amax` | `[2000, 0, 2000, 0, 2000, 2000, 0]` | 加速度上限（mm/s²） |
| `traj_enable` | `[T, F, T, F, T, T, F]` | 仅平移轴启用 |

### 4.5 设定点输出

```
FOR i:=1 TO 7 DO
    IF ext_setpoint_enabled[i]:
        IF traj_enable[i] AND NOT G.startup_smoothing_bypass:
            pos_abs := traj_pos[i] + G.init_pos[i];
            MC_ExtSetPointGenFeed(
                Position := pos_abs,
                Velocity := traj_vel[i],
                Acceleration := traj_acc[i],
                Direction := 1,
                Axis := G.axis[i]);
        ELSE:
            pos_abs := G.act_h[i] + G.init_pos[i];
            legacy_vel := (G.act_h[i] - act_h_prev[i]) / 0.001;
            MC_ExtSetPointGenFeed(
                Position := pos_abs,
                Velocity := legacy_vel,
                Acceleration := 0,
                Direction := 1,
                Axis := G.axis[i]);
        END_IF
        G.act_hf[i] := G.act_h[i];
    ELSE:
        G.act_hf[i] := G.axis[i].NcToPlc.ActPos - G.init_pos[i];
    END_IF
    act_h_prev[i] := G.act_h[i];
END_FOR
```

**双路径**：
- 二阶滤波路径（平移轴常用）：送 `traj_pos/vel/acc`；
- 传统路径（旋转轴 2/4/7、快退旁路、启动准备旁路）：送 `act_h`，速度由 `act_h` 差分算出。

`ext_setpoint_enabled[i]` 是 handle POU 内部的映射标志，只有它为 TRUE 且 `SetPointGenEnable.Done` 已达成才输出。计划回退 stu=10 会临时 `ext_setpoint_enabled[i] := FALSE` 禁用外部给定。

### 4.6 计划回退 FB（每轴 return_state[i]）

上位机通过 `G.axisN_return_cmd`（`ST_AxisPlannedReturnCmd`）触发；PLC 侧仅对 `return_enable[i] = [T, F, T, F, T, T, F]` 的轴响应：

```
return_state = 0:  空闲
    IF G.return_cmd[i].Req 且未 Done/Error:
        return_state := 10;

return_state = 10:  禁用外部给定
    G.return_cmd[i].Busy := TRUE;
    spg_disable_exec[i] := TRUE;
    IF SetPointGenDisable.Done OR NOT Enabled:
        ext_setpoint_enabled[i] := FALSE;
        return_state := 20;

return_state = 20:  执行 MC_MoveAbsolute
    return_exec[i] := TRUE;
    读入 target_abs / velocity / acc / dec / jerk;
    G.refer_from_left[i] := target_abs - leftlimit[i];   -- 让 UI 看到目标
    IF Done OR (NOT Busy 且 |ActPos - target_abs| ≤ 0.05):
        return_state := 30;
    IF Error OR CommandAborted:
        return_finish_error := TRUE;
        return_error_id_pending := ErrorID;
        return_state := 30;

return_state = 30:  等待停稳 → 重建平滑基准 → 重新使能 SPG
    IF |ActVelo| ≤ 0.5:
        用 ActPos 重建 Act_pos / refer / ref_slow / act_h / act_hf / act_hh / act_h_prev / traj_pos/vel/acc;
        spg_enable_exec[i] := TRUE;
        IF SetPointGenEnable.Done:
            ext_setpoint_enabled[i] := TRUE;
            G.return_cmd[i].Busy := FALSE;
            IF return_finish_error:
                G.return_cmd[i].Error := TRUE;
                G.return_cmd[i].ErrorId := return_error_id_pending;
            ELSE:
                G.return_cmd[i].Done := TRUE;
            END_IF
            return_state := 0;
        END_IF
    END_IF
```

**语义**：这是一个"临时接管 → 运动 → 交还"的**双缓冲切换机制**。stu=10 → 关外部给定；stu=20 → 独占轴运行 `MC_MoveAbsolute`；stu=30 → 停稳后重建 refer 基准，再重开外部给定并置 Done。上位机看到 Done 后清 Req，此时 PLC 已经在跟随上位机的 refer 流。

### 4.7 axis4 手动点动（`axis4_manual_state`）

上位机拉 `G.axis4_fwd_req` / `axis4_rev_req` 中之一为 TRUE，PLC 用 `MC_MoveVelocity` 手动点动。反向请求会先 `MC_Stop` 再切换方向。

| 状态 | 动作 |
|---|---|
| 0 | 空闲，等待请求 |
| 10 | 禁用 SPG |
| 20 | `MC_MoveVelocity` 运行；若请求方向变了 → 30；若无请求 → 30 |
| 30 | `MC_Stop`；停稳后若有 pending 反向 → 回 20；否则 → 39 |
| 39 | 一周期缓冲（让 MC_Stop 的 Execute=FALSE 生效） |
| 40 | 重建平滑基准 + 重新使能 SPG → Done → 回 0 |

参数：`axis4_manual_velocity = 30`、`acc/dec = 100`、`jerk = 500`。

### 4.8 每周期末尾统一调 FB

```
FOR i:=1 TO 7 DO
    G.SetPointGenEnable[i](Axis:=G.axis[i], Execute:=spg_enable_exec[i]);
    G.SetPointGenDisable[i](Axis:=G.axis[i], Execute:=spg_disable_exec[i]);
    fb_return_move_abs[i](Axis:=G.axis[i], Position:=..., ..., Execute:=return_exec[i]);
END_FOR
fb_axis4_move_velocity(Axis:=G.axis[4], Execute:=axis4_move_exec, ..., Direction:=axis4_manual_direction);
fb_axis4_stop(Axis:=G.axis[4], Execute:=axis4_stop_exec, ...);
```

所有 Execute 都是**本周期状态机决定的中间变量**，在此统一下发。这是 TwinCAT ST 的标准写法：FB 的 Execute 边沿由中间变量控制，避免在 CASE 分支内直接调用 FB 导致的重复触发。

## 5. `err.TcPOU` — 错误保持

### 5.1 职责

在 `_err` 态下，`estop_hold_req := TRUE` 长期保持，尝试恢复错误：
- 若唯一错误码是 `16#00004650`（Power 未就绪）→ 通过 `MC_Power.Enable` 脉冲切换重上电；
- 其他错误 → `MC_Reset`；
- 无错且全轴 powered → 跳 `_clear_err`。

### 5.2 状态机

```
step = 0: 进入
    禁用 SetPointGenEnable，触发 SetPointGenDisable=TRUE，清 axis_Reset.Execute
    step := 1;

step = 1: 分诊
    IF !has_err && all_powered:  → _clear_err
    IF !has_err (但有轴未 powered):
        FOR i: axis_Reset.Execute := FALSE; power_.Enable := TRUE
        启 retry_timer 2s
        step := 11
    IF has_power_not_ready && !has_other_error:  step := 10 (脉冲重上电)
    否则触发 MC_Reset，step := 2

step = 2: 等 MC_Reset 脉冲 200ms → step := 3

step = 3: 检查恢复
    IF !has_err && all_powered:  → _clear_err
    IF retry_timer.Q (2s):  step := 1 (重新分诊)

step = 10: RepowerPulse
    对报 4650 的轴 power_.Enable := FALSE；其他 TRUE
    200ms 后全部 → TRUE
    启 retry_timer 3s
    step := 11

step = 11: 等重上电就绪
    IF !has_err && all_powered:  → _clear_err
    IF has_err (新增错误):  step := 1
    IF retry_timer.Q (3s):  step := 1
```

**错误码 `16#00004650`**：TwinCAT NC 定义为"Power not ready"，通常在自检期间因短暂过载或跟随误差触发。用切电重来比 Reset 更快恢复。

## 6. `clear_err.TcPOU` — 错误清除过渡

### 6.1 职责

在 `_clear_err` 态下：
- 保持 `estop_hold_req := TRUE`；
- 若有错 → 回 `_err`；
- 若未自检完成 → 回 `_init`（冷启后先自检）；
- 若未全轴 powered → 保持 `power_.Enable := TRUE` 等待；
- 稳定 200ms 后 → 置 `handle_reinit_req := TRUE`，跳 `_handle`。

### 6.2 状态机

```
step = 0: 进入
    清 SPG Enable/Disable/axis_Reset.Execute
    step := 1

step = 1: 判断分支
    IF has_err:  → _err
    IF !self_check_done:  → _init
    IF !all_powered:  power_.Enable := TRUE; RETURN
    settle_timer(IN:=TRUE, PT:=T#200MS)
    IF settle_timer.Q:
        G.handle_reinit_req := TRUE;
        G.gen_state := _handle;
```

`clear_err` 的作用是**从错误恢复回正常态时保证一个稳定间隙**（200ms），期间不写 SPG、也不改 refer，让整个系统状态先安定下来。

### 6.3 遗留声明

POU 头部保留了一组 PID 变量（`Kp/Ki/Kd/err/err_f/value_f/err_sum/v1/v2/act_hh`）——**当前实现里未使用**。这是历史遗留，不要误以为是活的 PID 环。

## 7. `reset.TcPOU` — 复位过渡（当前主链路不主动进入）

### 7.1 职责

对全部 7 轴触发 `MC_Reset`，等待 Done/Error/错误已清除，超时（3s）也强制退出。完成后：
- 若仍有错 → `_err`；
- 否则 → `_init`。

### 7.2 状态机

```
step = 0: 预处理
    axis_Reset.Execute := FALSE
    SetPointGenEnable.Execute := FALSE
    SetPointGenDisable.Execute := TRUE   -- 复位期间禁用外部给定
    step := 1

step = 1: 触发复位
    axis_Reset.Execute := TRUE
    SetPointGenDisable.Execute := FALSE   -- 只需上升沿
    reset_timer 3s
    step := 2

step = 2: 等待
    IF 全部 (reset_output.Done OR reset_output.Error OR NOT axis.Status.Error):
        axis_Reset.Execute := FALSE
        IF 任一轴仍 Error / power_output.Error / reset_output.Error → _err
        否则 → _init
    ELSIF reset_timer.Q:  同上（强制退出）
```

### 7.3 使用场景

当前主链路不使用此 POU（`err` 直接切 `_clear_err`）。保留供未来手动触发或外部诊断使用。

## 8. `move_edge.TcPOU`（保留，未在 MAIN 中调用）

历史遗留 POU：对每个轴监控 `G.refer[i]` 的变化，当变化量 > `epsilon(0.01)` 时触发一次 `MC_MoveAbsolute(Position:=refer+init_pos, Velocity:=50)`。**当前 MAIN 中未实例化**，仅作为参考实现保留（例如在旧版无外部给定模式时使用）。

## 9. 参数调参速查

### 9.1 影响到位精度

| 参数 | 位置 | 影响 |
|---|---|---|
| `crawl_arrive_tol_mm`（上位机） | `ControlConfig` | 爬行到位判定容差 |
| stu=8 完成条件 `Done` 或 `|ActPos - target| < 0.01`（旋转轴） | `SelfCheck.TcPOU` | 自检到位判定 |
| `return_state=20` 完成条件 `|ActPos - target| ≤ 0.05` | `handle.TcPOU` | 计划回退到位判定 |

### 9.2 影响响应速度

| 参数 | 位置 | 影响 |
|---|---|---|
| `G.v_limit[]` | `G.TcGVL` 初值 `[7.5,7.5,1.5,1.5,1.5,7.5,1.5]`，可被上位机改 | refer 每拍最大增量 |
| `traj_wn` | `handle.TcPOU` 初值 `60.0` | 二阶滤波带宽 |
| `traj_vmax` / `traj_amax` | `handle.TcPOU` | 二阶滤波速度/加速度上限 |
| `avg_window_follow` | `handle.TcPOU` 初值 `20` | 滑动平均窗口 |

### 9.3 影响自检节奏

| 参数 | 位置 | 影响 |
|---|---|---|
| `vel_scan` / `vel_back` | `SelfCheck.TcPOU` 初值 `30.0` / `50.0` | 自检速度 |
| `scan_stagger_delay` | `SelfCheck.TcPOU` 初值 `[0,0,300,0,600,900,0] ms` | 逼近错峰 |
| `stagger_delay` | `SelfCheck.TcPOU` 初值 `[3,0,2,0,1,0,0] s` | 回位错峰 |
| `init_watchdog` PT | `init.TcPOU` `T#5S` | 上电超时 |

## 10. 已知问题 / 待确认点

### 10.1 `_reset` 在主链路中永远不会被进入

主 CASE 有 `_reset:` 分支，但 `err` POU 只切 `_clear_err`，`clear_err` 只切 `_init` / `_err` / `_handle`，其他 POU 也不写 `_reset`。这是**有意的**——错误恢复走 `_err → _clear_err → _init` 更简单。若未来需要"强制复位"入口，可从外部手动置 `G.gen_state := _reset`。

### 10.2 `clear_err` POU 内保留了一组 PID 变量

`Kp/Ki/Kd/err/err_f/value_f/err_sum/v1/v2/act_hh` 是历史遗留声明，**当前 clear_err 实现里未使用**。若未来清理代码，可删除这一节声明。

### 10.3 handle POU 内 `refer_interp` / `refer_step_from/to` / `refer_interval/tick/progress` 变量声明存在但未使用

只在重初始化时被写入（复位到 Act_pos 和 50），主控制路径中未参与设定点计算。属于历史遗留，可清理。

### 10.4 SelfCheck `dist_scan_large` / `target_pos_large` 未使用

初值 `1000.0` 的两个变量在当前实现中没有引用点。

### 10.5 自检失败后没有"给出诊断码"的机制

如果某个轴 stu 永远卡在某一状态（如反复 Repower 无果），除了看 `G.selfcheck_stu[i]` 和轴自身 `Status.ErrorId`，PLC 侧不会主动通过握手位告诉上位机"自检失败"。目前只能通过上位机看 `G.selfcheck_stu[]` 判断。

### 10.6 `handle.G.axis_Reset[i].Execute := TRUE`（section 末尾）会短时影响状态机

在 handle 主体末尾有：
```
FOR i:=1 TO 7 DO
    IF G.power_output[i].Error THEN
        G.axis_Reset[i].Execute := TRUE;
        G.flag := i;
        G.flagpos := G.axis[i].NcToPlc.ActPos;
    END_IF
END_FOR
```

这段在检测到 `power_output.Error` 时**同一周期内立即拉 `axis_Reset.Execute`**。理论上此时状态机会走到 `_err` 分支（`has_axis_error`），但由于 RESETED Action 也是在同一 MAIN 周期执行的，reset FB 会拿到这个 Execute。**这不是标准的 handle 职责范围**，属于兜底逻辑；如果频繁触发这里，说明 handle 期间出现了 Power 错误，应优先排查负载/电流。

## 11. 修改状态机与流程代码时的维护要求

任何修改 `init.TcPOU` / `SelfCheck.TcPOU` / `handle.TcPOU` / `ArmManual.TcPOU` / `err.TcPOU` / `clear_err.TcPOU` / `reset.TcPOU` / `MAIN.TcPOU` / `state.TcDUT` 的工作，**必须**：

1. 在本文档"§12 变更日志"追加条目，注明：日期 / 变更人 / 涉及文件与 POU / 行为变化要点 / 是否影响与上位机的契约。
2. 若改了状态跳转条件或加了新状态，更新 §1.4 状态跳转矩阵。
3. 若改了 SelfCheck 参数（速度、目标、错峰、判据），更新 §3 + `PLC项目说明.md §7` 参数速查。
4. 若改了 handle 平滑参数（`v_limit / traj_wn / traj_vmax / traj_amax / avg_window_*`），更新 §4.4 / §9.2。
5. 若改了计划回退 FB 状态机（`return_state`），更新 §4.6 + `../64位ADS - 相对路径 - 传数组 - 加上手柄/ADS通讯说明.md §5.2`。
6. 若改了 axis4 手动点动逻辑，更新 §4.7 + 上位机 `plc_io::write_axis4_manual_requests` 契约。
7. 若改了错误恢复策略（`err` / `clear_err`），更新 §5 / §6 + 说明是否影响上位机的重同步路径（`sync_all` / `sync_axis1` / `sync_axis6`）。

## 12. 变更日志

### 2026-07-28 — SelfCheck 输出标准装卸启动资格
- 作者：AI（Codex）。
- 涉及文件：`G.TcGVL`、`init.TcPOU`、`SelfCheck.TcPOU`。
- 行为变化：`init` 与 SelfCheck 重置时清 `G.startup_loading_ready`；SelfCheck 全轴完成并回到 `[96,280,430,580] mm` 标准装卸等待姿态后置 TRUE。上位机开始控制前消费该位，并结合实际位置选择标准启动或中断恢复启动。
- 契约影响：新增一个 BOOL ADS 握手位；不改变 PLC 顶层状态枚举、7 轴运动数组、计划回退或定位臂链路。

### 2026-07-06 — 新增定位臂独立点动流程
- 作者：AI（Codex，应用户计划实施）。
- 涉及文件：`MAIN.TcPOU`、`ArmManual.TcPOU`、`G.TcGVL`。
- 行为变化：`MAIN` 首周期清空定位臂 enable/reset/jog 请求，并在原 7 轴 Actions 与主状态机之前调用 `arm_manual_()`；`ArmManual` 每周期负责定位臂 5 轴上电、复位、点动、停止、方向冲突处理和状态上报。
- 与原状态机关系：定位臂不进入 `_init/_self_check/_handle/_err` 链路，定位臂错误不切换 `G.gen_state`。
- 上位机契约：原 7 轴 ADS 符号不变；新增 `G.arm_*` 符号供未来 UI 按需读写。

### 2026-07-06 — 文档初版（无代码变更）
- 作者：AI（Claude，应用户要求梳理）。
- 内容：基于现网代码逐 POU 编写第一版说明；建立状态跳转矩阵、SelfCheck 子状态详解、handle 四大功能块拆解、错误恢复链。
- 与历史 `Project_Documentation.txt`（2026-03-23）的关键差异：
  - handle 已新增二阶滤波（`traj_wn / traj_vmax / traj_amax / traj_enable`）；
  - handle 已实现完整的计划回退 FB（`return_state[i]` 三阶段状态机）；
  - handle 已实现 axis4 手动点动状态机；
  - SelfCheck 错峰改为两级（`scan_stagger_delay` 逼近错峰 + `stagger_delay` 回位错峰）；
  - SelfCheck 增加了 `RepowerPulse`/`RepowerWait`（stu=55/56）子状态用于自愈 Power 未就绪；
  - `init_target_from_left` 从旧版 `[96,0,280,0,530,685,0]` 更新为 `[96,0,280,0,430,580,0]`；
  - `err` POU 增加了对错误码 `16#00004650` 的针对性处理。
- 已列出 §10 六处已知点（`_reset` 未进入、`clear_err` PID 遗留、handle `refer_interp` 遗留、SelfCheck 未用变量、无诊断码握手位、handle 末尾兜底 Reset）。
- 未触碰任何源文件。

### （此处持续追加）
