# PLC 全局变量与 ADS 接口说明 / 项目内维护文档

> 本文档是 PLC 侧 `G.TcGVL` 与 `ST_AxisPlannedReturnCmd.TcDUT` 的完整参考，以及 PLC 对 ADS 通讯契约的"服务端视角"说明（对应上位机侧见 `../64位ADS - 相对路径 - 传数组 - 加上手柄/ADS通讯说明.md`）。
>
> 新增/删除/修改 GVL 变量，或改变读写方向/语义，请同步更新本文档、`PLC项目说明.md §5`，以及上位机 `ADS通讯说明.md §3`。

更新时间：2026-08-03
适用工程：`250902\Untitled2`（`GVLs\G.TcGVL`）
对应代码版本：2026-08-03 工作区（100 Hz ADS 通信与主机看门狗）

***

## 1. 命名空间与属性

`G.TcGVL` 声明了 `{attribute 'qualified_only'}`，即所有引用必须加 `G.` 前缀（`G.refer[1]` 而非 `refer[1]`）。这防止了与局部变量同名时的歧义。

## 2. 全局变量完整表

### 2.1 轴控功能块实例

| 变量 | 类型 | 说明 |
|---|---|---|
| `G.axis[1..7]` | `ARRAY[1..7] OF axis_REF` | NC 轴对象。每周期 MAIN 首行调用 `G.axis[i]()` 刷新状态 |
| `G.power_[1..7]` | `ARRAY[1..7] OF MC_Power` | 使能 FB。由 MAIN::POWERED Action 每周期调用 |
| `G.power_output[1..7]` | `ARRAY[1..7] OF ST_McOutputs` | MC_Power 输出（Done/Busy/Active/Error/ErrorID） |
| `G.axis_Reset[1..7]` | `ARRAY[1..7] OF MC_Reset` | 清错 FB。由 MAIN::RESETED Action 每周期调用 |
| `G.reset_output[1..7]` | `ARRAY[1..7] OF ST_McOutputs` | MC_Reset 输出 |
| `G.SetPointGenEnable[1..7]` | `ARRAY[1..7] OF MC_ExtSetPointGenEnable` | 激活外部给定模式 |
| `G.SetPointGenDisable[1..7]` | `ARRAY[1..7] OF MC_ExtSetPointGenDisable` | 禁用外部给定模式 |
| `G.SetPointGenEnable_output[1..7]` | `ARRAY[1..7] OF ST_McOutputs` | Enable FB 输出 |
| `G.SetPointGenDisable_output[1..7]` | `ARRAY[1..7] OF ST_McOutputs` | Disable FB 输出 |
| `G.move_jog[1..7]` | `ARRAY[1..7] OF MC_Jog` | 历史遗留，当前主链路未使用 |

### 2.1.1 定位臂 5 轴独立点动变量

这些变量是 2026-07-06 新增的独立 ADS 入口，只服务新增定位臂 5 轴。它们不改变原 `G.axis[1..7]`、`G.refer[1..7]`、`G.Act_pos[1..7]` 等介入机器人 7 轴契约。

| 变量 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `G.arm_axis[1..5]` | `ARRAY[1..5] OF AXIS_REF` | PLC 内部 / TwinCAT 链接 | 定位臂 NC Axis 1..5 的 PLC 轴引用 |
| `G.arm_manual_enable` | `BOOL` | 上位机写 | 定位臂手动点动总使能；FALSE 时任意 jog 请求都不会运动 |
| `G.arm_enable_req[1..5]` | `ARRAY[1..5] OF BOOL` | 上位机写 | 对应轴 MC_Power 使能请求 |
| `G.arm_reset_req[1..5]` | `ARRAY[1..5] OF BOOL` | 上位机写，PLC 消费后清零 | 对应轴 MC_Reset 请求；Done/Error 后 PLC 自动写回 FALSE |
| `G.arm_jog_pos_req[1..5]` | `ARRAY[1..5] OF BOOL` | 上位机写 | 正向点动按钮请求，建议 UI 按下 TRUE、松开 FALSE |
| `G.arm_jog_neg_req[1..5]` | `ARRAY[1..5] OF BOOL` | 上位机写 | 反向点动按钮请求，建议 UI 按下 TRUE、松开 FALSE |
| `G.arm_jog_velocity[1..5]` | `ARRAY[1..5] OF LREAL` | 上位机可写 | 点动速度，默认 `5.0`，单位跟随 NC Axis |
| `G.arm_jog_acc[1..5]` / `G.arm_jog_dec[1..5]` | `ARRAY[1..5] OF LREAL` | 上位机可写 | 点动加速度/减速度，默认 `50.0 / 50.0` |
| `G.arm_jog_jerk[1..5]` | `ARRAY[1..5] OF LREAL` | 上位机可写 | 点动 jerk，默认 `500.0` |
| `G.arm_power_output[1..5]` | `ARRAY[1..5] OF ST_McOutputs` | PLC 写，上位机可读 | 定位臂 MC_Power 输出快照 |
| `G.arm_reset_output[1..5]` | `ARRAY[1..5] OF ST_McOutputs` | PLC 写，上位机可读 | 定位臂 MC_Reset 输出快照 |
| `G.arm_act_pos[1..5]` | `ARRAY[1..5] OF LREAL` | PLC 写，上位机读 | 定位臂实际位置，来自 `G.arm_axis[i].NcToPlc.ActPos` |
| `G.arm_act_vel[1..5]` | `ARRAY[1..5] OF LREAL` | PLC 写，上位机读 | 定位臂实际速度，来自 `G.arm_axis[i].NcToPlc.ActVelo` |
| `G.arm_motion_busy[1..5]` | `ARRAY[1..5] OF BOOL` | PLC 写，上位机读 | 对应轴正在点动或停止处理中 |
| `G.arm_motion_done[1..5]` | `ARRAY[1..5] OF BOOL` | PLC 写，上位机读 | 对应轴已停止且无错误 |
| `G.arm_motion_error[1..5]` | `ARRAY[1..5] OF BOOL` | PLC 写，上位机读 | 对应轴点动/上电/复位或轴状态错误 |
| `G.arm_motion_error_id[1..5]` | `ARRAY[1..5] OF UDINT` | PLC 写，上位机读 | 定位臂错误码，优先来自运动/上电/复位 FB |
| `G.arm_cmd_dir[1..5]` | `ARRAY[1..5] OF SINT` | PLC 写，上位机读 | 当前命令方向：`-1` 反向、`0` 停止、`1` 正向 |
| `G.arm_cmd_conflict[1..5]` | `ARRAY[1..5] OF BOOL` | PLC 写，上位机读 | 正反向同时请求时置 TRUE；若正在运动则先停止 |

### 2.2 顶层控制变量

| 变量 | 类型 | 初值 | 说明 |
|---|---|---|---|
| `G.gen_state` | `state` | `_init` | 主状态机当前状态 |
| `G.enable` / `G.disable` | `BOOL` | — | 历史遗留，未在主链路中引用 |
| `G.flag` | `INT` | 0 | 调试用：最后一次检测到 power_output.Error 的轴号 |
| `G.flagpos` | `LREAL` | 0 | 对应轴的 ActPos（调试快照） |

### 2.3 位置坐标变量

| 变量 | 类型 | 读写方向 | 说明 |
|---|---|---|---|
| `G.init_pos[1..7]` | `ARRAY[1..7] OF LREAL` | PLC 内部写，上位机只读 | 相对坐标零点（绝对值）。`init` 首次记录；SelfCheck 完成时以左限位工作位覆盖 |
| `G.refer[1..7]` | `ARRAY[1..7] OF LREAL` | **上位机写**，PLC 读 | 上位机目标位置（相对 init_pos）。handle POU 用此驱动平滑链路 |
| `G.Act_pos[1..7]` | `ARRAY[1..7] OF LREAL` | PLC 写，上位机兼容读 | 各轴实际位置（相对 init_pos）。正常 100 Hz 快照直接读取 `NcToPlc.ActPos` 后在上位机减 `init_pos`；本数组保留给 PLC 内部和兼容诊断 |
| `G.act_h[1..7]` | `ARRAY[1..7] OF LREAL` | PLC 内部 | 速度限幅 + 滑动平均后的平滑参考 |
| `G.act_hf[1..7]` | `ARRAY[1..7] OF LREAL` | PLC 内部 | 最终输出给 MC_ExtSetPointGenFeed 的前一周期平滑位置；ext_setpoint 未使能时 = ActPos - init_pos |
| `G.ref_slow[1..7]` | `ARRAY[1..7] OF LREAL` | PLC 内部 | 速度限幅后的中间值（滑动平均的输入） |
| `G.leftlimit[1..7]` | `ARRAY[1..7] OF LREAL` | PLC 写（SelfCheck），上位机只读 | 自检记录的左限位绝对位置。上位机用于 from_left 坐标换算与窗口计算 |
| `G.act_pos_from_left[1..7]` | `ARRAY[1..7] OF LREAL` | PLC 写，兼容/诊断 | 各轴距左限位距离；正常上位机不读取，直接用绝对实际位置减 `leftlimit` 计算 |
| `G.refer_from_left[1..7]` | `ARRAY[1..7] OF LREAL` | PLC 写（由 handle 维护），兼容/诊断 | refer 对应的距左限位距离；正常上位机不读取 |

### 2.4 自检相关

| 变量 | 类型 | 说明 |
|---|---|---|
| `G.self_check_done` | `BOOL` | SelfCheck 完成标志。上位机用 OnChange Notification 监听并在注册时读取初值；`init` 每次进入时清零 |
| `G.startup_loading_ready` | `BOOL` | SelfCheck 完成且机构位于标准器械装卸等待姿态时置 TRUE；`init` 或 SelfCheck 重置时清零，上位机开始启动准备/直接控制前消费为 FALSE |
| `G.selfcheck_reset_req` | `BOOL` | `init` 拉起，请求 SelfCheck 重置内部状态机 |
| `G.selfcheck_stu[1..7]` | `ARRAY[1..7] OF INT` | SelfCheck 各轴当前子状态（上位机可读用于诊断） |
| `G.selfcheck_all_reach_limit` | `BOOL` | 所有目标轴已到限位（stu ≥ 6）的聚合标志 |

### 2.5 握手位

| 变量 | 类型 | 写方 | 读方 | 说明 |
|---|---|---|---|---|
| `G.handle_reinit_req` | `BOOL` | PLC（SelfCheck/handle/clear_err） | 上位机 Notification，处理后置 FALSE | PLC 请求上位机重同步 |
| `G.handle_reinit_done` | `BOOL` | PLC（handle 重初始化完成时） | PLC 内部（诊断） | handle 已完成 SetPointGen 使能 |
| `G.startup_loading_ready` | `BOOL` | PLC（SelfCheck；init/reset 清零） | 上位机（启动入口读，开始控制前写 FALSE） | 标准装卸启动的一次性资格位；实际位置仍由上位机二次校验 |
| `G.estop_hold_req` | `BOOL` | PLC（handle/err/clear_err/init） | 上位机 100 Hz 快照 + Notification | 急停/保持激活；上位机应停写 refer |
| `G.startup_smoothing_bypass` | `BOOL` | **上位机写** | PLC（handle） | 启动准备阶段临时旁路二阶滤波，用传统滑动平均路径 |

### 2.5.1 上位机通信看门狗

| 变量 | 类型 | 写方 | 读方 | 说明 |
|---|---|---|---|---|
| `G.host_session_id` | `UDINT` | 上位机 100 Hz | PLC | 本次上位机进程会话号；新进程接管时变化 |
| `G.host_heartbeat_sequence` | `UDINT` | 上位机 100 Hz | PLC | 每个通信周期递增；连续 100 ms 不变化触发超时锁存 |
| `G.host_recover_req` | `BOOL` | 上位机请求，PLC 清除 | 双向握手 | 心跳恢复且机构停稳后请求解除超时并重初始化 |
| `G.host_comm_timeout` | `BOOL` | PLC | 上位机 100 Hz 快照 + Notification | 主机通信超时锁存；为 TRUE 时冻结外部参考并执行安全清理 |

### 2.6 快退控制

| 变量 | 类型 | 写方 | 说明 |
|---|---|---|---|
| `G.axis1_fast_return` | `BOOL` | **上位机写**（每拍） | 轴1 快退进行中；handle POU 走 return_handoff_bypass，avg_window=1，跳过二阶滤波 |
| `G.axis6_fast_retract` | `BOOL` | **上位机写**（每拍） | 轴6 快退进行中；同上 |
| `G.return_cmd[1..7]` | `ARRAY[1..7] OF ST_AxisPlannedReturnCmd` | 上位机写 Req/Target/Vel/Acc/Dec/Jerk；PLC 写 Busy/Done/Error/ErrorId | 计划回退命令结构，详见 §4 |

### 2.7 速度与平滑参数（可被上位机修改）

| 变量 | 类型 | 初值 | 说明 |
|---|---|---|---|
| `G.v_limit[1..7]` | `ARRAY[1..7] OF LREAL` | `[7.5, 7.5, 1.5, 1.5, 1.5, 7.5, 1.5]` | 每拍 refer 增量上限（mm/s 或 deg/step）。上位机在启动准备阶段会缩放 |
| `G.ref_slow[1..7]` | `ARRAY[1..7] OF LREAL` | — | 见 §2.3（只写，不暴露给上位机读） |

PID 参数（`G.Kp / G.Ki / G.Kd / G.pid_i / G.pid_e_prev / G.pid_i_limit`）：**当前 handle 实现未使用**，属于历史遗留变量，不建议通过 ADS 修改。

### 2.8 axis4 手动点动

| 变量 | 类型 | 写方 | 读方 | 说明 |
|---|---|---|---|---|
| `G.axis4_fwd_req` | `BOOL` | **上位机写** | PLC | 正向点动请求 |
| `G.axis4_rev_req` | `BOOL` | **上位机写** | PLC | 反向点动请求 |
| `G.axis4_manual_busy` | `BOOL` | PLC | **上位机 Notification** | 点动进行中 |
| `G.axis4_manual_done` | `BOOL` | PLC | 上位机读 | 点动已停止（正常） |
| `G.axis4_manual_error` | `BOOL` | PLC | 上位机读 | 点动报错 |
| `G.axis4_manual_error_id` | `UDINT` | PLC | 上位机读 | 错误码 |

### 2.9 电缸控制量

电缸1..4 由上位机直接写 WORD 原始值，PLC 侧仅做总线 I/O 映射（`AT %Q*`）：

| 变量 | 类型 | 写方 | 安装位置 | 作用 |
|---|---|---|---|---|
| `G.cylinder1_value` | `WORD AT %Q*` | **上位机写** | 物理夹持底座 | 夹/松导管 |
| `G.cylinder2_value` | `WORD AT %Q*` | **上位机写** | 轴1 机构 | 与电缸1 交替夹持导管 |
| `G.cylinder3_value` | `WORD AT %Q*` | **上位机写** | 轴5 机构 | 夹/松导丝 |
| `G.cylinder4_value` | `WORD AT %Q*` | **上位机写** | 轴6 机构 | 与电缸3 交替夹持导丝 |
| `G.cylinder5_value` | `WORD AT %Q*` | **PLC 内部写**（handle 每周期映射） | 轴3 机构 | Y 阀控制（0=夹紧，2000=打开） |
| `G.cylinder5_cmd` | `WORD` | 历史遗留，未被 handle 主链路使用 | — | — |
| `G.cylinder5_press_req` | `BOOL` | **上位机写** | — | TRUE=按下→ cylinder5=0；FALSE=松开→ cylinder5=2000 |

电缸值含义（参考上位机 `ADS通讯说明.md §3.3`）：
- `cylinder1`：`0` 开 / `400` 预夹 / `1000` 夹
- `cylinder2`：`0` 开 / `150` 预开 / `400` 预夹 / `600` 夹
- `cylinder3`：`50` 夹 / `200` 预夹 / `250` 开 / `400` 跟随释放 / `500` 启动准备开
- `cylinder4`：`0` 开 / `100` 跟随释放 / `300` 预 / `500` 夹
- `cylinder5`：`0` 夹 / `2000` 开

### 2.10 力传感器 IO

| 变量 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `G.ft_1_value` | `INT AT %I*` | 输入 | 扭矩通道（ADS 可读） |
| `G.fn_1_value` | `INT AT %I*` | 输入 | 轴向力通道 |
| `G.ft_2_value` | `INT AT %I*` | 输入 | 第二路扭矩 |
| `G.fn_2_value` | `INT AT %I*` | 输入 | 第二路轴向力 |

PLC 侧无滤波或标定逻辑，仅做总线映射。正常上位机在 100 Hz 快速 Sum Read 中同步读取四路；TCP_DAQ 仅为人工选择的回退源，见 `../64位ADS - 相对路径 - 传数组 - 加上手柄/力反馈说明.md §2.1`。

## 3. `ST_AxisPlannedReturnCmd.TcDUT`

```
TYPE ST_AxisPlannedReturnCmd :
STRUCT
    Req        : BOOL;      -- 上位机拉高触发，Done/Error 后上位机清零
    Busy       : BOOL;      -- PLC 置位（return_state 10→30 期间）
    Done       : BOOL;      -- PLC 置位（return_state=30 完成且无错）
    Error      : BOOL;      -- PLC 置位（MoveAbsolute 失败）
    ErrorId    : UDINT;     -- PLC 填入的 ErrorID
    TargetAbs  : LREAL;     -- 上位机写：目标绝对位置（mm）
    Velocity   : LREAL;     -- 上位机写：速度
    Acc        : LREAL;     -- 上位机写：加速度
    Dec        : LREAL;     -- 上位机写：减速度
    Jerk       : LREAL;     -- 上位机写：加加速度
END_STRUCT
```

`G.return_cmd[1..7]` 是此类型的数组。上位机实际上只用 1/3/5/6 四路（与 `return_enable[i]` 掩码 `[T,F,T,F,T,T,F]` 一致），其他路即使写 Req=TRUE PLC 也不会处理。

**使用协议（来自 `handle.TcPOU` 的 return_state 实现）**：

```
1. 上位机写入 TargetAbs / Velocity / Acc / Dec / Jerk;
2. 上位机写 Req := TRUE;
3. PLC 内 return_state 从 0 → 10 → 20 → 30，期间 Busy := TRUE;
4. 完成时 PLC 置 Done := TRUE 或 Error := TRUE（Busy 同时置 FALSE）;
5. 上位机检测到 Done / Error 后写 Req := FALSE（此时 PLC 在 return_state=0 清 Done/Error）;
```

**注意**：若上位机在 Busy=TRUE 时重新写 Req=TRUE 并改 TargetAbs，PLC 检测不到，运动目标不会更新。不应在 Busy 期间重触发同一路。

## 4. ADS 通讯契约（PLC 服务端视角）

### 4.1 读写方向汇总

下表按方向分组，"PLC 符号"与上位机 `plc_io.cpp::AdsSymbol` 常量对应：

**上位机 → PLC（写入）**

| PLC 符号 | 类型 | GVL 变量 | 频率约定 |
|---|---|---|---|
| `G.host_session_id` / `G.host_heartbeat_sequence` | `UDINT` / `UDINT` | 同名 | 100 Hz 统一 Sum Write，每周期必写 |
| `G.host_recover_req` | `BOOL` | 同名 | 看门狗恢复握手时按需写 |
| `G.refer` | `double[7]` | `G.refer[1..7]` | 运动输出有效时随 100 Hz 统一 Sum Write |
| `G.v_limit` | `double[7]` | `G.v_limit[1..7]` | 启动准备/恢复时 |
| `G.cylinder1_value` .. `G.cylinder4_value` | `WORD` | 同名 | 变化时并入统一 Sum Write，失败保持脏状态重试 |
| `G.cylinder5_press_req` | `BOOL` | 同名 | 变化时写 |
| `G.axis1_fast_return` | `BOOL` | 同名 | 有效期内随 100 Hz 输出写 |
| `G.axis6_fast_retract` | `BOOL` | 同名 | 有效期内随 100 Hz 输出写 |
| `G.startup_smoothing_bypass` | `BOOL` | 同名 | 变化时写 |
| `G.startup_loading_ready` | `BOOL` | 同名 | 启动准备或直接控制入口消费时写 FALSE |
| `G.axis4_fwd_req` / `G.axis4_rev_req` | `BOOL` | 同名 | 变化时写，失败重试 |
| `G.axisN_return_cmd.Req` + `TargetAbs/Vel/Acc/Dec/Jerk` | `BOOL` + `LREAL×5` | `G.return_cmd[N]` | 触发时写，完成后清 Req |
| `G.handle_reinit_req` | `BOOL` | 同名 | 读后清零（写 FALSE） |
| `G.arm_manual_enable` | `BOOL` | 同名 | 定位臂手动点动总使能，按需写 |
| `G.arm_enable_req` | `BOOL[5]` | `G.arm_enable_req[1..5]` | 定位臂上电请求，按需写 |
| `G.arm_reset_req` | `BOOL[5]` | `G.arm_reset_req[1..5]` | 定位臂复位请求，触发后由 PLC 消费清零 |
| `G.arm_jog_pos_req` / `G.arm_jog_neg_req` | `BOOL[5]` | 同名 | 定位臂正/反向点动按钮请求，按下 TRUE、松开 FALSE |
| `G.arm_jog_velocity/acc/dec/jerk` | `double[5]` | 同名 | 定位臂点动参数，默认值可按需覆盖 |

**PLC → 上位机（读取）**

| PLC 符号 | 类型 | GVL 变量 | 频率约定 |
|---|---|---|---|
| `G.axis[1..7].NcToPlc.ActPos` | `LREAL×7` | NC 轴过程数据 | 100 Hz 快速 Sum Read；上位机据此计算相对位置和 from-left |
| `G.Act_pos` | `double[7]` | `G.Act_pos[1..7]` | 兼容/诊断，不进入正常 100 Hz 路径 |
| `G.init_pos` | `double[7]` | `G.init_pos[1..7]` | 首次连接、重连、自检完成上升沿和显式重同步时缓存 |
| `G.leftlimit` | `double[7]` | `G.leftlimit[1..7]` | 与 `init_pos` 同步刷新缓存 |
| `G.act_pos_from_left` / `G.refer_from_left` | `double[7]` / `double[7]` | 同名 | 正常上位机不读取，保留 PLC 内部和兼容诊断 |
| `G.self_check_done` | `BOOL` | 同名 | Notification + 注册时初值读取 |
| `G.startup_loading_ready` | `BOOL` | 同名 | 上位机进程启动时探测符号；每次启动准备入口读取 |
| `G.handle_reinit_req` | `BOOL` | 同名 | Notification + 注册时初值读取 |
| `G.estop_hold_req` / `G.host_comm_timeout` | `BOOL` / `BOOL` | 同名 | 100 Hz 快速 Sum Read + Notification |
| `G.axisN_return_cmd.Busy/Done/Error/ErrorId` | `BOOL×3 + UDINT` | `G.return_cmd[N]` | FastMove 状态机按需提交一次专项 Sum Read |
| `G.axis4_manual_busy/done/error/error_id` | `BOOL×3 + UDINT` | 同名 | Notification + 注册时初值读取 |
| `G.ft_1/fn_1/fn_2/ft_2_value` | `INT` | 同名 | 100 Hz 快速 Sum Read，同一包同步获取四路 |
| `G.gen_state` | `WORD` | 同名 | Notification + 注册时初值读取 |
| `G.arm_power_output` / `G.arm_reset_output` | `ST_McOutputs[5]` | 同名 | 定位臂诊断，按需读 |
| `G.arm_act_pos` / `G.arm_act_vel` | `double[5]` | 同名 | 定位臂实际位置/速度，按需读 |
| `G.arm_motion_busy/done/error/error_id` | `BOOL[5] / BOOL[5] / BOOL[5] / UDINT[5]` | 同名 | 定位臂点动状态，按需读 |
| `G.arm_cmd_dir` / `G.arm_cmd_conflict` | `SINT[5] / BOOL[5]` | 同名 | 定位臂命令方向与正反向冲突状态，按需读 |

新增定位臂符号采用按需通信策略：旧上位机继续只读写原 7 轴符号时，PLC 只是多暴露符号表，不会主动产生额外 ADS 流量；未来 UI 只有在显示或操作定位臂时再读写 `G.arm_*`。

### 4.2 PLC 侧保证

1. **`G.Act_pos[i]` 每周期由 handle / SelfCheck 更新**，若当前态不在 `_handle` / `_self_check`，上位机读到的值来自上一次写入（不是实时 ActPos）。
2. **`G.leftlimit[i]` 只在 SelfCheck 完成后有意义**；自检前为 0（PLC 初始值），上位机不应在 `self_check_done = FALSE` 时使用 `from_left` 换算结果。
3. **`G.estop_hold_req` 在以下时刻强制为 TRUE**：`init`（上电期间）、`_err`、`_clear_err`（200ms 前）、`handle` 的 `NOT init_done` 阶段、`handle` 的 `hold_active` 阶段。上位机应当把此标志作为"禁止手柄推动"的门控。
4. **`G.return_cmd[i].Req` 上位机写 TRUE 后，必须等到 Done/Error 其中之一置位，再写 FALSE**。不允许在 Busy 期间重写 TRUE（见 §3）。
5. **`G.cylinder5_value` 上位机不应直接写**；通过 `G.cylinder5_press_req` 写布尔量，由 PLC 每周期映射。直接写 `cylinder5_value` 会在下一周期被 PLC 覆盖。
6. **定位臂变量不参与原 `G.gen_state` 状态机**；`G.arm_manual_enable=FALSE` 或对应轴未上电时，`G.arm_jog_*` 请求不会产生运动。正反向同时为 TRUE 时 `G.arm_cmd_conflict[i]=TRUE`，PLC 停止该轴并拒绝启动新运动。
7. **`G.startup_loading_ready` 不是位置反馈**：PLC 仅在本轮 SelfCheck 全部完成后置 TRUE。上位机还会核对 axis1/3/5/6 距左限位是否分别处于 `[96,280,430,580] ±2 mm`；任一条件不满足时改走中断恢复启动。旧 PLC 没有该符号时仅按实际位置兼容判定。
8. **主机看门狗超时不是普通状态提示**：`host_heartbeat_sequence` 连续 100 ms 不变化时，PLC 锁存 `host_comm_timeout`，冻结外部参考，清快退、axis4 和平滑旁路请求，并对运行中的计划回退/axis4 运动执行受控停止；气缸保持最后状态。只有新鲜心跳、显式 `host_recover_req` 和停稳条件同时满足后才受理恢复。

### 4.3 ADS 连接参数

| 参数 | 值 | 来源 |
|---|---|---|
| TwinCAT 端口 | `851` | TwinCAT 3 PLC1 默认端口 |
| 本机路由 | `OpenCommInsideReadOnly()` 优先 | 上位机 `main.cpp` |
| 远端 NetId（回退） | `169.254.119.135.1.1` | 上位机 `main.cpp hardcoded_ads_netid` |
| PLC 应用名（诊断） | `TwinCAT_SystemInfoVarList._AppInfo.AppName` | 上位机启动时验证 |

PLC 侧无 ADS 配置文件，由 TwinCAT XAR 路由表管理。上位机连接和重连只接受已经处于 `ADSSTATE_RUN` 的 PLC，不自动切换 PLC 状态，也不执行 Activate Configuration。

## 5. 历史遗留变量速查

以下变量存在于 `G.TcGVL` 但当前主链路未使用，仅保留为向后兼容或调试入口：

| 变量 | 说明 |
|---|---|
| `G.move_jog[1..7]` | `MC_Jog` FB 实例，旧版点动逻辑遗留，MAIN 中无 JOG 调用 |
| `G.Kp / G.Ki / G.Kd` | PLC 侧 PID 参数数组，`handle` 当前实现未使用 |
| `G.pid_i / G.pid_e_prev / G.pid_i_limit` | PID 积分/误差历史/限幅，同上 |
| `G.enable / G.disable` | 历史遗留 BOOL，MAIN 和各 POU 不引用 |
| `G.cylinder5_cmd` | 历史遗留，早期设计为"上位机写此变量 → handle 周期下发"，现已简化为 `cylinder5_press_req` 机制 |

## 6. 修改 GVL 与 ADS 契约时的维护要求

任何新增/删除/改语义/改类型的 GVL 变量工作，**必须**：

1. 更新本文档 §2 对应变量条目。
2. 更新 `PLC项目说明.md §5` 的分工契约表。
3. 若变量通过 ADS 暴露给上位机：同步更新上位机 `ADS通讯说明.md §3` 符号表 + `plc_io.cpp::AdsSymbol` 常量定义。
4. 若改了 `ST_AxisPlannedReturnCmd` 结构（添加字段）：同步更新 §3 + 上位机 `control_types.h::AxisReturnAdsSymbols` + `plc_io.cpp` 中该结构的符号名常量组。
5. 若删除了变量：全工程 grep（`Untitled2/**/*.TcPOU` + `plc_io.cpp` + `main.cpp`）确认无引用后再删。
6. 在本文档"§7 变更日志"追加条目，注明：日期 / 变更人 / 变量名 / 改动类型 / 是否需要上位机同步修改。

## 7. 变更日志

### 2026-08-03 — 100 Hz ADS 契约与主机看门狗
- 作者：AI（Codex）。
- 变量：新增 `G.host_session_id`、`G.host_heartbeat_sequence`、`G.host_recover_req`、`G.host_comm_timeout`。
- 通信变化：7 轴 NC 实际位置与四路力进入 100 Hz 快速 Sum Read；`init_pos/leftlimit` 改为事件刷新缓存，低频状态改用 Notification，`act_pos_from_left/refer_from_left` 不再由正常上位机读取。
- 安全行为：心跳连续 100 ms 不变化时冻结外部参考并受控停止相关运动；连接/重连不自动把 PLC 切到 RUN，不激活 TwinCAT 配置。
- PDO 影响：不修改或猜测四路力 EtherCAT PDO 映射。

### 2026-07-28 — 新增器械装卸启动资格握手位
- 作者：AI（Codex）。
- 变量：新增 `G.startup_loading_ready : BOOL`；SelfCheck 重置和 `init` 入口清零，自检完成并回到标准装卸等待姿态时置 TRUE，上位机开始启动准备或直接控制前写 FALSE 消费。
- 行为：该标志与 axis1/3/5/6 的 `[96,280,430,580] ±2 mm` 实际位置共同决定标准启动；不满足时上位机执行中断恢复启动。旧 PLC 无符号时保留位置兼容路径。
- 接口影响：上位机 `plc_io` 同步新增读写符号；不修改 `G.refer[1..7]`、`G.Act_pos[1..7]`、计划回退、定位臂或命名管道结构。

### 2026-07-06 — 新增定位臂 5 轴 ADS 符号
- 作者：AI（Codex，应用户计划实施）。
- 变量：新增 `G.arm_axis[1..5]`、`G.arm_manual_enable`、`G.arm_enable_req/reset_req/jog_pos_req/jog_neg_req`、`G.arm_jog_velocity/acc/dec/jerk`、`G.arm_power_output/reset_output`、`G.arm_act_pos/vel`、`G.arm_motion_*`、`G.arm_cmd_dir/conflict`。
- 改动类型：新增变量，不扩展原 `G.refer[1..7]`、`G.Act_pos[1..7]`、`G.init_pos[1..7]`、`G.return_cmd[1..7]` 等 7 轴接口。
- 上位机同步：旧上位机无需修改；未来 UI 可按需添加定位臂按钮和状态读取。新增符号不读写时不会增加 ADS 通信负载。

### 2026-07-06 — 文档初版（无代码变更）
- 作者：AI（Claude，应用户要求梳理）。
- 内容：基于现网 `G.TcGVL` 和 `ST_AxisPlannedReturnCmd.TcDUT`、结合 `handle.TcPOU` / `SelfCheck.TcPOU` 使用方式，编写第一版全局变量说明与 ADS 契约。
- 明确了以下先前文档缺失的内容：
  - `G.cylinder5_value` 不应由上位机直接写，需走 `cylinder5_press_req` 二值化通道；
  - `G.return_cmd[]` 仅 1/3/5/6 四路有效（对应 `return_enable` 掩码）；
  - `G.leftlimit[]` 在自检完成前不可用；
  - 历史遗留变量（PID、move_jog、cylinder5_cmd 等）列表。
- 未触碰任何源文件。

### （此处持续追加）
