# ADS 通讯说明 / 项目内维护文档

> 本文档梳理上位机与倍福 PLC 之间的 ADS 通讯实现：连接、符号表、读写封装、断线/重同步路径，以及与主循环的契约。
> 修改 `plc_io.cpp/.h`、`ADSComm.cpp`、符号表、或新增 PLC 变量时，请同步在最末"变更日志"追加条目。

更新时间：2026-07-26
适用工程：`64位ADS - 相对路径 - 传数组 - 加上手柄\ADS.sln`
对应代码版本：2026-06-26 主分支（基于 `plc_io.cpp` / `ADSComm1.h` / `main.cpp` 当前状态校对）

***

## 1. 角色与拓扑

上位机作为 ADS Client，倍福 PLC（TwinCAT 3，端口 851）作为 ADS Server。所有控制下发、状态回读和轴1力/扭矩首选采样都走 ADS。`G.fn_1_value` / `G.ft_1_value` 保持 PLC 原始 `INT` 契约，C++ 统一按 `raw / 1000.0 = V` 换算；TCP 采集卡代码仅保留为人工切换的回退路径。

```
[上位机 ADS.exe]
   |
   |-- OpenComm_inside()  本地 AMS 路由（优先）
   |-- OpenComm()         远端 AMS NetId（回退，默认 169.254.119.135.1.1）
   v
[TwinCAT PLC, Port 851, GVL=G]
   G.refer[7]      <-- 上位机写入：每拍目标位置（相对 init_pos）
   G.Act_pos[7]    --> 上位机读取：实际位置（相对 init_pos）
   G.init_pos[7]   --> 上位机读取：相对零点偏置
   G.leftlimit[7]  --> 上位机读取：左限位绝对位置
   ... (完整符号表见 §3)
```

## 2. 主要源文件

| 文件 | 角色 |
|---|---|
| `ADS/Include/ADSComm1.h` + `ADSComm.cpp` | `CADSComm` 类：封装 ADS API。提供 `OpenComm/OpenComm_inside/CloseComm`、`ADSRead/ADSWrite`（按符号名）、`ADSReadSum`（多符号批量读）、按索引的 `ADSRead/ADSWrite` 重载、`ADSGetAddr`、`IsCommOpen`、`GetLastError`。内部用 `unordered_map<string, ulong>` 缓存符号句柄。 |
| `ADS/Include/TcAdsAPI.h` + `TcAdsDef.h` | 倍福官方 ADS API 头文件，由 SDK 提供。 |
| `plc_io.h/.cpp` | 业务层 I/O：定义 `AdsSymbol::*` 符号名常量、`AxisReturnAdsSymbols` 结构、`namespace plc_io` 工具函数。所有业务模块都通过 `plc_io::` 调用 ADS，不直接接触 `CADSComm`。 |
| `control_types.h` | 定义 `AppContext`，其中 `CADSComm* ads` 作为所有读写的统一入口；另定义 `AxisReturnAdsSymbols` / `AxisReturnStatus`。 |
| `main.cpp` 主循环前段（ADS 连接建立段） | `OpenComm_inside` 优先 + `OpenComm` 回退 + `app_name` 诊断 + 一系列 lambda 包装读写函数（`read_plc_state`/`read_force_sample`/`write_refer`/`request_axis_return`/...）。约在 `int main()` 内、`while(true)` 主循环之前（2026-06-26 行号约 L209-L330）。 |

## 3. 符号表（`plc_io.cpp::AdsSymbol`）

所有符号名常量集中定义在 `plc_io.cpp` 顶部。新增 PLC 变量时**必须**在此添加常量并更新本表。

### 3.1 数组类（7维对应 axis1..axis7）

| 常量 | PLC 符号 | 类型/长度 | 读写 | 含义 |
|---|---|---|---|---|
| `refer` | `G.refer` | double[7] | 写 | 上位机本周期目标位置（相对 init_pos） |
| `act_pos` | `G.Act_pos` | double[7] | 读 | PLC 实际位置（相对 init_pos） |
| `init_pos` | `G.init_pos` | double[7] | 读 | 相对零点偏置（refer/Act_pos 的坐标系基线） |
| `leftlimit` | `G.leftlimit` | double[7] | 读 | 左限位绝对位置（用于 from_left 坐标换算） |
| `act_pos_from_left` | `G.act_pos_from_left` | double[7] | 读（降频 1/5） | 距左限位距离（监测/门控用） |
| `refer_from_left` | `G.refer_from_left` | double[7] | 读（降频 1/5） | refer 距左限位距离 |
| `v_limit` | `G.v_limit` | double[7] | 读+写 | 各轴速度上限（启动准备阶段缩放/恢复用） |

### 3.2 力采样（ADS 首选源）

| 常量 | PLC 符号 | 类型 | 读写 | 含义 |
|---|---|---|---|---|
| `ft_1_value` | `G.ft_1_value` | short | 读 | 扭矩原始输入；有符号 `INT`，`raw / 1000.0 = V` |
| `fn_1_value` | `G.fn_1_value` | short | 读 | 轴向力原始输入；有符号 `INT`，`raw / 1000.0 = V` |
| `fn_2_value` | `G.fn_2_value` | short | 读 | 第二组轴向力（仅 ADS 链路使用） |
| `ft_2_value` | `G.ft_2_value` | short | 读 | 第二组扭矩（同上） |

> 力反馈链路真正消费的是 `force_sample.fn_1_value_v` / `ft_1_value_v`。默认 ADS 分支在 `plc_io::read_force_sample` 内完成 `INT -> V` 换算；TCP_DAQ 模式才会覆盖这两路。ADS 读取同时返回 axis1/axis2 实际位置（`act_pos` 快照），见 [力反馈说明.md §2.2](力反馈说明.md#22-主循环采样maincpp-主循环步骤-15力采样节拍段)。

### 3.3 气缸/电磁阀

| 常量 | PLC 符号 | 类型 | 读写 | 含义 |
|---|---|---|---|---|
| `cylinder1_value` | `G.cylinder1_value` | unsigned short | 写 | 导管侧夹爪对 cyl1（0/400 开 / 00 夹 / 320 预夹） |
| `cylinder2_value` | `G.cylinder2_value` | unsigned short | 写 | 导管侧夹爪对 cyl2（0 开 / 600 夹 / 150 预开 / 400 预夹） |
| `cylinder3_value` | `G.cylinder3_value` | unsigned short | 写 | 导丝侧夹爪对 cyl3（250 开 / 50 夹 / 200 预夹 / 400 跟随释放） |
| `cylinder4_value` | `G.cylinder4_value` | unsigned short | 写 | 导丝侧夹爪对 cyl4（0 开 / 500 夹 / 300 预开/预夹 / 100 跟随释放） |
| `cylinder5_cmd` | `G.cylinder5_cmd` | unsigned short | 读（诊断） | 电缸5命令读回（诊断用） |
| `cylinder5_press_req` | `G.cylinder5_press_req` | bool | 写+读（诊断） | 电缸5按键请求（PLC 映射为 0/2000） |
| `cylinder5_value` | `G.cylinder5_value` | unsigned short | 读（诊断） | 电缸5实际输出（诊断用） |

### 3.4 计划回退命令（数组元素，每轴一组）

PLC 统一使用 `G.return_cmd[1..7]` 数组；上位机将启用回退的第 1、3、5、6 轴各封装为一组 `AxisReturnAdsSymbols`。不要使用旧的 `G.axisN_return_cmd.*` 命名。

| 字段 | PLC 符号模板 | 类型 | 读写 | 含义 |
|---|---|---|---|---|
| `req` | `G.return_cmd[N].Req` | bool | 写 | 触发位（true=下发一次回退） |
| `busy` | `G.return_cmd[N].Busy` | bool | 读 | 正在执行 |
| `done` | `G.return_cmd[N].Done` | bool | 读 | 完成 |
| `error` | `G.return_cmd[N].Error` | bool | 读 | 报错 |
| `error_id` | `G.return_cmd[N].ErrorId` | unsigned long | 读 | 错误码 |
| `target_abs` | `G.return_cmd[N].TargetAbs` | double | 写 | 目标绝对位置（mm） |
| `velocity` | `G.return_cmd[N].Velocity` | double | 写 | 速度（mm/s） |
| `acc` | `G.return_cmd[N].Acc` | double | 写 | 加速度（mm/s²） |
| `dec` | `G.return_cmd[N].Dec` | double | 写 | 减速度（mm/s²） |
| `jerk` | `G.return_cmd[N].Jerk` | double | 写 | 加加速度（mm/s³） |

### 3.5 axis4 手动点动

| 常量 | PLC 符号 | 类型 | 读写 | 含义 |
|---|---|---|---|---|
| `axis4_fwd_req` | `G.axis4_fwd_req` | bool | 写 | axis4 正向点动请求 |
| `axis4_rev_req` | `G.axis4_rev_req` | bool | 写 | axis4 反向点动请求 |
| `axis4_manual_busy` | `G.axis4_manual_busy` | bool | 读（降频 1/20） | axis4 手动控制正忙 |
| `axis4_manual_error` | `G.axis4_manual_error` | bool | 读（降频 1/20） | axis4 手动控制报错 |
| `axis4_manual_error_id` | `G.axis4_manual_error_id` | unsigned long | 读（降频 1/20） | 错误码 |

### 3.6 顶层状态/握手位

| 常量 | PLC 符号 | 类型 | 读写 | 含义 |
|---|---|---|---|---|
| `self_check_done` | `G.self_check_done` | bool | 读 | PLC 自检完成 |
| `handle_reinit_req` | `G.handle_reinit_req` | bool | 读+写清除 | PLC 请求上位机重新同步 |
| `estop_hold_req` | `G.estop_hold_req` | bool | 读（降频 1/10） | PLC 急停/保持激活 |
| `axis1_fast_return` | `G.axis1_fast_return` | bool | 写 | 轴1快退标志（PLC 用于平滑旁路） |
| `axis6_fast_retract` | `G.axis6_fast_retract` | bool | 写 | 轴6快退标志（同上） |
| `startup_smoothing_bypass` | `G.startup_smoothing_bypass` | bool | 写 | 启动准备期平滑旁路 |
| `gen_state` | `G.gen_state` | unsigned short | 读（诊断） | PLC 通用状态字 |
| `app_name` | `TwinCAT_SystemInfoVarList._AppInfo.AppName` | char[64] | 读（一次性） | 当前 PLC 应用名（连接诊断） |

### 3.7 定位臂 5 轴手动点动（Qt 上位机）

定位臂是新增的独立 5 轴通道，只通过 `G.arm_*` 符号控制，不扩展原 `G.refer[1..7]`、`G.Act_pos[1..7]`、`G.init_pos[1..7]` 或 `G.return_cmd[1..7]`。当前上位机只做单轴手动点动，不做正运动学、逆运动学或整体位姿联动。

| PLC 符号 | 类型/长度 | 读写 | 上位机行为 |
|---|---|---|---|
| `G.arm_manual_enable` | bool | 写 | Qt 界面默认写 TRUE；关闭时同时清 jog 请求 |
| `G.arm_enable_req` | bool[5] | 写 | 每轴“上电”按钮保持 TRUE/FALSE |
| `G.arm_reset_req[i]` | bool | 写 | 每轴“复位”按钮写 TRUE；PLC 消费后自动清零，上位机不主动写 FALSE |
| `G.arm_jog_pos_req` / `G.arm_jog_neg_req` | bool[5] | 写 | Jog+ / Jog- 按下写 TRUE、松开写 FALSE |
| `G.arm_jog_velocity` / `G.arm_jog_acc` / `G.arm_jog_dec` / `G.arm_jog_jerk` | double[5] | 写 | UI 可编辑，默认 `5.0 / 50.0 / 50.0 / 500.0` |
| `G.arm_power_output[i].Done/Error/ErrorID` | bool/bool/unsigned long | 读 | 显示每轴上电状态和错误码 |
| `G.arm_reset_output[i].Done/Busy/Error/ErrorID` | bool/bool/bool/unsigned long | 读 | 显示复位状态和错误码 |
| `G.arm_act_pos` / `G.arm_act_vel` | double[5] | 读 | 显示定位臂实际位置/速度 |
| `G.arm_motion_busy/done/error` | bool[5] | 读 | 显示点动运行状态 |
| `G.arm_motion_error_id` | unsigned long[5] | 读 | 显示运动错误码 |
| `G.arm_cmd_dir` / `G.arm_cmd_conflict` | signed char[5] / bool[5] | 读 | 显示命令方向与正反向冲突 |

## 4. 连接建立（在 `main.cpp` 主循环开始之前）

```cpp
if (ads.OpenComm_inside()) {
    // 优先：本地 AMS 路由，端口 851
} else if (ads.OpenComm()) {
    // 回退：远端 AMS NetId = 169.254.119.135.1.1
} else {
    // 两者都失败 → 退出程序
}
// 诊断：读 app_name 确认连的是正确的 PLC 实例
```

- `OpenComm_inside` 走本机 AMS Router（要求本机装了 TwinCAT XAR 或 ADS Router 路由配置）。
- `OpenComm` 走远端 NetId（`hardcoded_ads_netid = "169.254.119.135.1.1"`，链路本地地址，对应 Windows 防火墙规则的 169.254.x.x 段）。如果要换 PLC，**改这个常量**。
- 失败时 `ads.GetLastError()` 返回的字符串直接打印。
- 连上后立刻读 `app_name` 验证目标 PLC（避免连错实例）。

## 5. 读写封装语义（`plc_io::`）

所有调用都通过 `AppContext::ads` 指针。函数返回 `true/false` 表示本次 ADS 操作是否成功；失败时主循环通常只是跳过本拍，不重试。

### 5.1 周期读写

| 函数 | 调用频率 | 行为 |
|---|---|---|
| `read_plc_state` | 每控制拍 | `ADSReadSum` 一次性读 `act_pos / init_pos / leftlimit` 三个 double[7]。 |
| `read_force_sample` | 力反馈/旧高频记录按原节拍；仅主从实验记录时 50ms | `ADSReadSum` 读 `ft_1 / fn_1 / fn_2 / ft_2 / act_pos`，并把 axis1 两路原始 `INT` 统一换算为伏特。 |
| `write_refer` | 每控制拍 | `ADSWrite` 写 `G.refer`（double[7]）。 |
| `read_v_limit` / `write_v_limit` | 启动准备 | 读 / 写 `G.v_limit`。 |

### 5.2 计划回退时序

```cpp
// 触发一次回退
request_axis_return(symbols, target_abs, vel, acc, dec, jerk)
  // 顺序写 Req=false -> TargetAbs -> Velocity -> Acc -> Dec -> Jerk -> Req=true
  // 任一步失败都会保持 Req=false，上层保持实际位置并停止运动控制

// 每拍轮询
read_axis_return_status(symbols, status)
  // 读 Busy/Done/Error/ErrorId

// 完成或出错后
clear_axis_return_request(symbols)
  // 写 Req=false
```

- **不要并发触发同一路 FB**：在 `Req=true` 之后到 `clear_axis_return_request` 之间，`plc_move_requested` 标志位会阻止重复触发。
- `clear_axis1_group_return_requests` 一次性清 1/3/5/6 四路的 Req（用于全量重同步前）。

### 5.3 重同步路径

主循环在以下情况会触发重同步（`sync_all` / `sync_axis1` / `sync_axis6`）：

- 启动：自检完成或直接控制进入时 `sync_all(30)`。
- PLC 主动请求：读到 `G.handle_reinit_req == true` -> 清除该位 + `sync_all(30)`。
- PLC 急停保持解除：`sync_all(20)`。
- 轴回退报错后：单轴 `sync_axis1(3)` 或 `sync_axis6(3, ...)`。
- 轴回退正常完成 RestoreWait 阶段后：单轴 sync。
- 导丝模式进入前：先清 `G.return_cmd[6].Req` 并确认 `Busy=false`，随后执行 `sync_axis6` 或独立模式同步；退出时执行 `sync_axis1`。
- 主循环兜底：`startup.completed && !control_active && !freeze && !estop_hold` 时每拍 `sync_all(20)` 尝试恢复。

`sync_*` 内部都会做：清回退请求 → 读 PLC 当前实际位置 → `load_pos_from_actual` 把 refer 重置为 Act_pos → 平均 N 个手柄样本重建基准 → 写 refer。任何一步 ADS 失败都直接返回 false，让上层重试。

## 6. 频率/节拍约定

> 触发位置以"主循环第 N 步"为准（步骤编号见 [项目架构总览.md §4](项目架构总览.md#4-主循环骨架maincpp)）。括号内行号是 2026-06-26 校对时的真实位置，仅作辅助定位；任何代码插入都会让行号漂移，**不要把行号当作稳定锚点**。

| 操作 | 频率 | 触发位置（主循环步号 + 函数名） | 2026-06-26 行号 |
|---|---|---|---|
| `read_plc_state` + `write_refer` | 每主循环拍 | 步骤 11 内（运动激活分支起点 `read_plc_state()` -> 末尾 `write_refer()`） | 约 L1276、L2171 |
| `act_pos_from_left` / `refer_from_left` 读取（ADSReadSum 两符号） | 每 5 拍 | 步骤 2（快退标志清零之后） | 约 L736-L750 |
| `estop_hold_req` 读取 | 每 10 拍 | 步骤 4 | 约 L916 |
| `self_check_done` / `handle_reinit_req` 读取 | 每 50 拍 | 步骤 8（自检/重同步轮询） | 约 L1179、L1213 |
| `axis4_manual_*` 读取 | 每 20 拍 | 步骤 13（轴 4 手动控制轮询） | 约 L2180 |
| `axis1_fast_return` / `axis6_fast_retract` / `startup_smoothing_bypass` 写 | 每拍 | 步骤 14（气缸/快退标志写入 PLC） | 约 L2268-L2270 |
| 气缸 1/2/3/4 写 + `cylinder5_press_req` 写 | 每拍（控制激活或启动序列激活时） | 步骤 14 | 约 L2205-L2211 |
| 定位臂 `G.arm_*` 命令写 | 按需 | Qt `ControlEngine::writeArmCommands`，命令变化、Jog 按下/松开、复位触发时写 | 2026-07-06 |
| 定位臂 `G.arm_*` 状态读 | Qt 每 5 个控制拍（约 50ms） | Qt `ControlEngine::readArmState`，与旧 7 轴控制读写解耦 | 2026-07-06 |

降频读是为了减轻 ADS 通信负担；这些变量在 PLC 侧不会突变，降频读到的值代表"当前事实"，不会引发控制问题。

## 7. 失败/异常处理约定

- `ADSRead/ADSWrite/ADSReadSum` 任何一次失败只返回 false，不抛异常、不退出程序。
- `write_refer` 失败时主循环跳过本拍写入，下一拍重试。**没有断线检测**：如果 ADS 连接彻底断开，主循环会持续返回 false，控制环冻结但不会自愈。
- `request_axis_return` 中间任意一步写失败，会 `clear_axis_return_request` 后返回 false，下一拍重试。
- `read_axis_return_status` 部分失败时 `ok` 标志被 AND 累计，但函数仍返回 true 并使用上一次的 `status` 值。这意味着偶发读失败会让 Busy/Done 多保持一拍旧值，**不会**误触发完成处理。

## 8. 已知问题 / 待确认点

### 8.1 没有 ADS 断线检测与自动重连

`CADSComm` 内部没有断线探测。如果 TCP 中断（拔网线）或 PLC 重启，ADS API 调用会持续返回错误码，但 `OpenComm` 不会被重新调用。
**TODO**：建议在 `read_plc_state` 连续失败 N 次后调用 `ads.CloseComm()` + `ads.OpenComm_inside()` / `OpenComm()` 重新建链。当前的"冻结 + 等待人工介入"策略对手术机器人是安全的，但限制了远程恢复能力。

### 8.2 ADS 力输入映射需要现场确认

当前默认 `force_sample_source == ADS`。`zero_force_sensor` 会同步读取一帧 ADS 样本后采零，不依赖 TCP worker 或力反馈开关。现场必须确认 `G.fn_1_value`（轴向力）和 `G.ft_1_value`（扭矩）均为有符号 `INT`，且同一放大器量纲满足 `1000 counts = 1 V`；符号或量程不一致会影响零点、标定与力反馈方向。

### 8.3 `app_name` 诊断只在启动时打印一次

如果中途 PLC 切换实例（极少见），上位机不会察觉。可接受，但调试时需知。

### 8.4 `refer_from_left` 与 `act_pos_from_left` 读取降频到每 5 拍

意味着主循环里用 `plc_act_pos_from_left` 做的判定（如 axis3 投送停止位 axis3_delivery_stop_active）也是 5 拍延迟。当前 axis3 推进速度受限，5 拍延迟不会引发安全问题；但若未来提高推进速度，需要复核。

## 9. 修改 ADS 相关代码时的维护要求

任何修改 `plc_io.cpp/.h`、`ADSComm.cpp`、新增/删除 PLC 符号、改变读写频率或失败处理策略的工作，**必须**：

1. 在本文档"§10 变更日志"追加条目，注明：日期 / 变更人 / 涉及文件与函数 / 行为变化要点 / 是否影响与 PLC 的契约（需要 PLC 侧配合改吗）。
2. 若新增 PLC 符号，更新 §3 符号表 + `plc_io.cpp::AdsSymbol` 常量定义。
3. 若新增 axisN_return FB 实例，更新 §3.4 + 在 `plc_io.cpp` 末尾添加对应 `AxisReturnAdsSymbols` 常量。
4. 若改了 `sync_*` 内部的步骤顺序，更新 §5.3 + 同步告知 [运动流程说明.md §3](运动流程说明.md#3-窗口与跟随基准)（窗口/同步流程的契约）。
5. 若改了连接拓扑（如新增冗余 NetId、TLS），更新 §1 拓扑图 + §4 连接建立。

## 10. 变更日志

### 2026-06-26 — 文档初版（无代码变更）
- 作者：AI（Claude，应用户要求梳理）。
- 内容：基于现网代码（`ADSComm1.h`、`plc_io.cpp/.h`、`control_types.h` 中的 `AppContext`、`main.cpp` 主循环前段的 ADS 连接与 lambda 包装、主循环步骤 4/8/13 的自检/重同步/axis4 诊断段）编写第一版说明。
- 已记录 §8 四处已知点（无断线重连、ADS 力采样备份未启用、app_name 仅启动时验证、from_left 读取降频）。
- 未触碰任何源文件。

### 2026-06-26 — 文档体系评议后修订（无代码变更）
- 作者：AI（Claude，应用户要求按多 Agent 评议结论修订）。
- 修复硬伤 2：§6 频率表的行号引用全部改为"主循环步号 + 函数名（2026-06-26 行号约 LXXXX）"形式；§2 主要源文件表、§4 章节标题、§10 变更日志初版条目里的硬行号引用同步替换为"主循环前段""主循环步号 N"锚点描述。步号定义见 [项目架构总览.md §4](项目架构总览.md#4-主循环骨架maincpp-主循环-whiletrue)。
- 改进 A：补充跨文档锚点链接（§1 力反馈主路径引用、§3.2 力反馈 §2.2 跳转、§9.4 运动流程 §3 同步告知）。
- 改进 D：头部新增"对应代码版本"字段。
- 未触碰任何源文件。

### 2026-07-06 — Qt 上位机新增定位臂手动 ADS 通道
- 作者：AI（Codex，应用户要求实施）。
- 涉及文件：`../CatheterRobotUI/src/SharedState.h`、`ControlEngine.h/.cpp`、`MainWindow.h/.cpp`。
- 行为变化：新增 `G.arm_*` ADS 符号绑定，Qt UI 提供定位臂 5 轴单轴上电、复位、Jog+、Jog-、点动参数编辑和状态显示。
- 通讯策略：命令按变化写入，状态约 50ms 降频读取；原 7 轴 `G.refer/Act_pos/init_pos/return_cmd` 高频链路不扩展、不改语义。

### 2026-07-26 — 主从位移实验与 ADS 力采样首选迁移
- 作者：AI（Codex）。
- 涉及文件：`plc_io.cpp`、`control_types.h`、`main.cpp`、`delivery_tracking.*`、`delivery_tracking_logger.*`、可视化管道与 WPF。
- 行为变化：默认力源改为 ADS；`G.fn_1_value/G.ft_1_value` 在 C++ 中按 `raw/1000.0` 转换为伏特，ADS 模式下 UI 零点采集同步读取一帧 ADS 数据。TCP 保留为显式代码回退，不自动切换。
- 采样节拍：主从位移 CSV 会话运行而力反馈关闭时，ADS 力采样至少按 20 Hz 更新；不改变 `G.refer[1..7]`、计划回退或定位臂 ADS 契约。

### （此处持续追加）
