# ADS 通讯说明 / 项目内维护文档

> 本文档梳理上位机与倍福 PLC 之间的 ADS 通讯实现：连接、符号表、读写封装、断线/重同步路径，以及与主循环的契约。
> 修改 `plc_io.cpp/.h`、`ADSComm.cpp`、符号表、或新增 PLC 变量时，请同步在最末"变更日志"追加条目。

更新时间：2026-06-26
适用工程：`64位ADS - 相对路径 - 传数组 - 加上手柄\ADS.sln`

***

## 1. 角色与拓扑

上位机作为 ADS Client，倍福 PLC（TwinCAT 3，端口 851）作为 ADS Server。所有控制下发、状态回读、力采样备份都走 ADS。力反馈链路里的 ft_1/fn_1 主路径走 TCP（见 `力反馈说明.md`），ADS 提供备份采样源 + axis1/axis2 实际位置 + axis3/5/6 镜像跟随目标。

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
| `main.cpp` L209-L307 | ADS 连接建立、PLC 应用名诊断、lambda 包装读写函数（`read_plc_state`/`read_force_sample`/`write_refer`/...）。 |

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

### 3.2 力采样（ADS 备份源）

| 常量 | PLC 符号 | 类型 | 读写 | 含义 |
|---|---|---|---|---|
| `ft_1_value` | `G.ft_1_value` | short | 读 | 扭矩通道（ADS 备份源；TCP_DAQ 模式下不被使用） |
| `fn_1_value` | `G.fn_1_value` | short | 读 | 轴向力通道（同上） |
| `fn_2_value` | `G.fn_2_value` | short | 读 | 第二组轴向力（仅 ADS 链路使用） |
| `ft_2_value` | `G.ft_2_value` | short | 读 | 第二组扭矩（同上） |

> 力反馈链路真正消费的是 `force_sample.fn_1_value_v` / `ft_1_value_v`，TCP_DAQ 模式下这两路被 TCP 覆盖。ADS 读取仍用于拿到 axis1/axis2 实际位置（`read_force_sample` 内顺手返回 `act_pos` 快照），见 `力反馈说明.md §2.2`。

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

### 3.4 计划回退命令（FB 实例，每轴一组）

每轴一组共 10 个符号，封装在 `AxisReturnAdsSymbols` 结构里。当前定义了 4 组：`axis1_return` / `axis3_return` / `axis5_return` / `axis6_return`。

| 字段 | PLC 符号模板 | 类型 | 读写 | 含义 |
|---|---|---|---|---|
| `req` | `G.axisN_return_cmd.Req` | bool | 写 | 触发位（true=下发一次回退） |
| `busy` | `G.axisN_return_cmd.Busy` | bool | 读 | 正在执行 |
| `done` | `G.axisN_return_cmd.Done` | bool | 读 | 完成 |
| `error` | `G.axisN_return_cmd.Error` | bool | 读 | 报错 |
| `error_id` | `G.axisN_return_cmd.ErrorId` | unsigned long | 读 | 错误码 |
| `target_abs` | `G.axisN_return_cmd.TargetAbs` | double | 写 | 目标绝对位置（mm） |
| `velocity` | `G.axisN_return_cmd.Velocity` | double | 写 | 速度（mm/s） |
| `acc` | `G.axisN_return_cmd.Acc` | double | 写 | 加速度（mm/s²） |
| `dec` | `G.axisN_return_cmd.Dec` | double | 写 | 减速度（mm/s²） |
| `jerk` | `G.axisN_return_cmd.Jerk` | double | 写 | 加加速度（mm/s³） |

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

## 4. 连接建立（`main.cpp` L209-L239）

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
| `read_force_sample` | 按 `force_log.period_ms` | `ADSReadSum` 读 `ft_1 / fn_1 / fn_2 / ft_2 / act_pos`。 |
| `write_refer` | 每控制拍 | `ADSWrite` 写 `G.refer`（double[7]）。 |
| `read_v_limit` / `write_v_limit` | 启动准备 | 读 / 写 `G.v_limit`。 |

### 5.2 计划回退时序

```cpp
// 触发一次回退
request_axis_return(symbols, target_abs, vel, acc, dec, jerk)
  // 顺序写 TargetAbs -> Velocity -> Acc -> Dec -> Jerk -> Req=true

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
- 轴回退报错后：单轴 `sync_axis1(3, false, 0)` 或 `sync_axis6(3, ...)`。
- 轴回退正常完成 RestoreWait 阶段后：单轴 sync。
- 导丝模式进入/退出：`sync_axis6` 或 `sync_axis1`。
- 主循环兜底：`startup.completed && !control_active && !freeze && !estop_hold` 时每拍 `sync_all(20)` 尝试恢复。

`sync_*` 内部都会做：清回退请求 → 读 PLC 当前实际位置 → `load_pos_from_actual` 把 refer 重置为 Act_pos → 平均 N 个手柄样本重建基准 → 写 refer。任何一步 ADS 失败都直接返回 false，让上层重试。

## 6. 频率/节拍约定

| 操作 | 频率 | 触发位置 |
|---|---|---|
| `read_plc_state` + `write_refer` | 每主循环拍 | `main.cpp` L1232 运动激活分支 |
| `act_pos_from_left` / `refer_from_left` 读取 | 每 5 拍 | `main.cpp` L705 |
| `estop_hold_req` 读取 | 每 10 拍 | `main.cpp` L870 |
| `self_check_done` / `handle_reinit_req` 读取 | 每 50 拍 | `main.cpp` L1133 |
| `axis4_manual_*` 读取 | 每 20 拍 | `main.cpp` L2133 |
| `axis1_fast_return` / `axis6_fast_retract` / `startup_smoothing_bypass` 写 | 每拍 | `main.cpp` L2224 |
| 气缸写入 | 每拍（控制激活或启动序列激活时） | `main.cpp` L2155 |

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

### 8.2 力采样 ADS 备份源未真正启用

`force_sample_source == ADS` 分支虽然存在，但当前默认 `TCP_DAQ`，且 `zero_force_sensor` 在 TCP_DAQ 模式下**不会回退到 ADS**。如果 TCP 卡掉，ADS 这一路力采样也不会自动接管，力反馈会直接置零。
**TODO**：如果希望 ADS 作为 TCP 失联时的备份采样源，需要：1) `zero_force_sensor` 在 TCP 无帧时回退到 ADS；2) 验证 PLC 那边的 ft_1/fn_1 是否同步自同一个放大器（否则单位/零点会不一致）。

### 8.3 `app_name` 诊断只在启动时打印一次

如果中途 PLC 切换实例（极少见），上位机不会察觉。可接受，但调试时需知。

### 8.4 `refer_from_left` 与 `act_pos_from_left` 读取降频到每 5 拍

意味着主循环里用 `plc_act_pos_from_left` 做的判定（如 axis3 投送停止位 axis3_delivery_stop_active）也是 5 拍延迟。当前 axis3 推进速度受限，5 拍延迟不会引发安全问题；但若未来提高推进速度，需要复核。

## 9. 修改 ADS 相关代码时的维护要求

任何修改 `plc_io.cpp/.h`、`ADSComm.cpp`、新增/删除 PLC 符号、改变读写频率或失败处理策略的工作，**必须**：

1. 在本文档"§10 变更日志"追加条目，注明：日期 / 变更人 / 涉及文件与函数 / 行为变化要点 / 是否影响与 PLC 的契约（需要 PLC 侧配合改吗）。
2. 若新增 PLC 符号，更新 §3 符号表 + `plc_io.cpp::AdsSymbol` 常量定义。
3. 若新增 axisN_return FB 实例，更新 §3.4 + 在 `plc_io.cpp` 末尾添加对应 `AxisReturnAdsSymbols` 常量。
4. 若改了 `sync_*` 内部的步骤顺序，更新 §5.3 + 同步告知"运动流程说明.md"。
5. 若改了连接拓扑（如新增冗余 NetId、TLS），更新 §1 拓扑图 + §4 连接建立。

## 10. 变更日志

### 2026-06-26 — 文档初版（无代码变更）
- 作者：AI（Claude，应用户要求梳理）。
- 内容：基于现网代码（`ADSComm1.h`、`plc_io.cpp/.h`、`control_types.h` 中的 `AppContext`、`main.cpp` L209-L307 ADS 段、L1133-L1194 自检/重同步段、L2133-L2150 axis4 诊断段）编写第一版说明。
- 已记录 §8 四处已知点（无断线重连、ADS 力采样备份未启用、app_name 仅启动时验证、from_left 读取降频）。
- 未触碰任何源文件。

### （此处持续追加）
