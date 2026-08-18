# ADS 通讯说明 / 项目内维护文档

> 本文档梳理上位机与倍福 PLC 之间的 ADS 通讯实现：连接、符号表、读写封装、断线/重同步路径，以及与主循环的契约。
> 修改 `ads_communication.cpp/.h`、`plc_io.cpp/.h`、`ADSComm.cpp`、`sensor_calibration_experiment.cpp/.h`、符号表、或新增 PLC 变量时，请同步在最末"变更日志"追加条目。

更新时间：2026-08-11
适用工程：`64位ADS - 相对路径 - 传数组 - 加上手柄\ADS.sln`
对应代码版本：2026-08-11 当前工作区（100 Hz 主链路、固定容量异步回退命令队列、双轴 Notification 与取消闭环）

***

## 1. 角色与拓扑

上位机作为 ADS Client，倍福 PLC（TwinCAT 3，端口 851）作为 ADS Server。正常模式由独立 `AdsCommunicationService` 以 QPC 绝对截止时间运行 100 Hz 通信周期：每周期一次 Sum Read 形成位置/力/通信状态统一快照，再以一次 Sum Write 下发心跳和本周期输出。`G.fn_1_value` / `G.ft_1_value` 保持 PLC 原始 `INT` 契约，C++ 统一按 `raw / 1000.0 = V` 换算，再由 `force_calibration.h` 使用 2026-08-03 的 `F_direct` 斜率换算为 N；TCP 采集卡代码仅保留为人工切换的回退路径。

```
[上位机 ADS.exe]
   |
   |-- OpenCommInsideReadOnly()  本地 AMS 路由（优先）
   |-- OpenCommReadOnly()        远端 AMS NetId（回退，默认 169.254.119.135.1.1）
   v
[TwinCAT PLC, Port 851, GVL=G]
   G.refer[7]      <-- 上位机写入：每拍目标位置（相对 init_pos）
   G.Act_pos[7]    --> 上位机读取：实际位置（相对 init_pos）
   G.init_pos[7]   --> 上位机读取：相对零点偏置
   G.leftlimit[7]  --> 上位机读取：左限位绝对位置
   G.host_*        <-> 100 Hz 心跳、超时锁存与显式恢复握手
   ... (完整符号表见 §3)
```

100 Hz 快照序号 `AdsFastSnapshot::attempt_sequence` 和主机侧快照队列都只存在于 C++ 进程内。PLC 不维护环形缓冲区，也没有 `G.write_sequence` 一类记录序号；这两类概念不要混为一谈。

## 2. 主要源文件

| 文件 | 角色 |
|---|---|
| `ADS/Include/ADSComm1.h` + `ADSComm.cpp` | `CADSComm` 类：封装 ADS API。提供正常/只读连接、单项读写、按句柄 Sum Read/Sum Write、Notification 注册/注销、超时设置和设备状态读取；内部缓存符号句柄，并用互斥量保护同一 ADS 端口。 |
| `ads_communication.h/.cpp` | 正常模式 ADS 通信服务：100 Hz QPC 调度、统一快照、Sum Write 输出、OnChange Notifications、首拍软保持、100 ms 断线判定、退避重连、PLC 重启识别、主机看门狗恢复和诊断统计。 |
| `sensor_calibration_experiment.h/.cpp` | 四路传感器重新标定工具：解析 `--sensor-calibration` 命令、校验原标定 CSV、建立只读 ADS 连接、同步采集四路 `INT16` 并生成 TXT。该模块不调用 `plc_io::` 控制写入接口，也不修改 PLC 或正常上位机标定参数。 |
| `ADS/Include/TcAdsAPI.h` + `TcAdsDef.h` | 倍福官方 ADS API 头文件，由 SDK 提供。 |
| `plc_io.h/.cpp` | 业务层 I/O：定义 `AdsSymbol::*`、计划回退封装和低频/专项读写。正常主循环绑定 `ads_service` 后，快照读取只复制内存，运行期专项请求提交给通信线程串行执行。 |
| `control_types.h` | 定义 `AppContext`：`CADSComm* ads` 仅供服务启动前初始化/无服务兼容路径使用，`AdsCommunicationService* ads_service` 承担正常运行期快照、输出和专项请求；另定义 `AxisReturnAdsSymbols` / `AxisReturnStatus`。 |
| `main.cpp` | 建立初始 ADS 路由后启动 `AdsCommunicationService`；主循环等待快照、消费 Notifications 状态、发布 `AdsOutputCommand`，并分别处理同进程重连和 PLC 重启。 |

## 3. 符号表（`plc_io.cpp::AdsSymbol`）

所有符号名常量集中定义在 `plc_io.cpp` 顶部。新增 PLC 变量时**必须**在此添加常量并更新本表。

### 3.1 数组类（7维对应 axis1..axis7）

| 常量 | PLC 符号 | 类型/长度 | 读写 | 含义 |
|---|---|---|---|---|
| `refer` | `G.refer` | double[7] | 写 | 上位机本周期目标位置（相对 init_pos） |
| `act_pos` | `G.Act_pos` | double[7] | 读 | PLC 实际位置（相对 init_pos） |
| `init_pos` | `G.init_pos` | double[7] | 读 | 相对零点偏置（refer/Act_pos 的坐标系基线） |
| `leftlimit` | `G.leftlimit` | double[7] | 读 | 左限位绝对位置（用于 from_left 坐标换算） |
| `act_pos_from_left` | `G.act_pos_from_left` | double[7] | 兼容/诊断读 | 正常 100 Hz 路径在上位机用实际绝对位置、`init_pos`、`leftlimit` 计算，不依赖该数组的降频轮询 |
| `refer_from_left` | `G.refer_from_left` | double[7] | 兼容/诊断读 | PLC 内部参考位置监测；正常上位机快照不把它作为位置源 |
| `v_limit` | `G.v_limit` | double[7] | 读+写 | 各轴速度上限（启动准备阶段缩放/恢复用） |

### 3.2 力采样（ADS 首选源）

| 常量 | PLC 符号 | 类型 | 读写 | 含义 |
|---|---|---|---|---|
| `ft_1_value` | `G.ft_1_value` | short | 读 | 扭矩原始输入；有符号 `INT`，`raw / 1000.0 = V` |
| `fn_1_value` | `G.fn_1_value` | short | 读 | 轴向力原始输入；有符号 `INT`，`raw / 1000.0 = V` |
| `fn_2_value` | `G.fn_2_value` | short | 读 | 第二组轴向力（仅 ADS 链路使用） |
| `ft_2_value` | `G.ft_2_value` | short | 读 | 第二组扭矩（同上） |

> 力反馈链路真正消费的是 `force_sample.fn_1_value_v` / `ft_1_value_v`。默认 ADS 分支在 `plc_io::read_force_sample` 内完成 `INT -> V` 换算；`calibrate_force` 再分别按 `0.614437208097 N/V`、`0.703683250522 N/V` 做动态零点后的 `F_direct` 映射。TCP_DAQ 模式才会覆盖这两路。ADS 读取同时返回 axis1/axis2 实际位置（`act_pos` 快照），见 [力反馈说明.md §2.2](力反馈说明.md#22-主循环采样maincpp-主循环步骤-15力采样节拍段)。

重新标定工具直接读取 PLC 原始 `short` 计数，不执行 `raw / 1000.0` 换算。工具内的固定映射为：传感器1 -> `G.ft_1_value`、传感器2 -> `G.fn_1_value`、传感器3 -> `G.fn_2_value`、传感器4 -> `G.ft_2_value`。每个采样时刻通过同一次 `ADSReadSum` 获取四路快照，避免四次独立读取造成通道时间错位。

### 3.2.1 100 Hz 快照附带的时序符号

| PLC/TwinCAT 符号 | 类型 | 用法 |
|---|---|---|
| `TwinCAT_SystemInfoVarList._TaskInfo[1].CycleCount` | UDINT | 同一次 Sum Read 的首尾各读一次，差值过大时判定本帧跨越过多 PLC 周期 |
| `TwinCAT_SystemInfoVarList._TaskInfo[1].DcTaskTime` | LINT | 检测 PLC 任务时间回退；与 CycleCount 回退、应用名变化共同用于识别 PLC 重启/应用重载 |
| `G.axis[N].NcToPlc.ActPos` | LREAL，N=1..7 | 首选实际绝对位置源；若这些 ADS 符号句柄不可解析，则整组回退到 `G.Act_pos[7]` |

上述位置源都是 TwinCAT 已发布的 ADS 符号。上位机不猜测 EtherCAT PDO 地址、不扫描未知 I/O 映射，也不修改 NC/PDO 配置。

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

PLC 统一使用 `G.return_cmd[1..7]` 数组；上位机保留第 1、3、5、6 轴的 `AxisReturnAdsSymbols` 以兼容全量清理与重同步，但当前六种运行模式的计划动作执行腿只有 axis1 和 axis6。不要使用旧的 `G.axisN_return_cmd.*` 命名。

| 字段 | PLC 符号模板 | 类型 | 读写 | 含义 |
|---|---|---|---|---|
| `req` | `G.return_cmd[N].Req` | bool | 写 | 触发位（true=下发一次回退） |
| `busy` | `G.return_cmd[N].Busy` | bool | axis1/axis6 OnChange Notification + 连接/重连初值批读；axis3/5 当前不轮询 | 正在执行 |
| `done` | `G.return_cmd[N].Done` | bool | axis1/axis6 OnChange Notification + 连接/重连初值批读；axis3/5 当前不轮询 | 完成 |
| `error` | `G.return_cmd[N].Error` | bool | axis1/axis6 OnChange Notification + 连接/重连初值批读；axis3/5 当前不轮询 | 报错 |
| `error_id` | `G.return_cmd[N].ErrorId` | unsigned long | axis1/axis6 OnChange Notification + 连接/重连初值批读；axis3/5 当前不轮询 | 错误码 |
| `target_abs` | `G.return_cmd[N].TargetAbs` | double | 写 | 目标绝对位置（mm） |
| `velocity` | `G.return_cmd[N].Velocity` | double | 写 | 速度（mm/s） |
| `acc` | `G.return_cmd[N].Acc` | double | 写 | 加速度（mm/s²） |
| `dec` | `G.return_cmd[N].Dec` | double | 写 | 减速度（mm/s²） |
| `jerk` | `G.return_cmd[N].Jerk` | double | 写 | 加加速度（mm/s³） |

axis1 和 axis6 的运行期计划回退结果都由通信服务的 OnChange Notification 更新。两轴各自保存当前值、单调事件序号、最近一次 `Busy=TRUE`、`Done=FALSE`、`Done=TRUE`、`Error=TRUE` 边沿，以及最近一次非零 `ErrorId`。`PlannedReturnCoordinator` 只接受请求基线之后的本轴事件：`Busy=TRUE` 或 `Done=FALSE` 用于 ACK，最终 `Done=TRUE` 必须晚于 ACK 且当前 `Busy=FALSE`。旧完成位、另一轴事件和单独的错误码更新都不能推进状态机。axis1/axis6 均不再为了等待计划回退结果执行周期状态 Sum Read。

### 3.5 axis4 手动点动

| 常量 | PLC 符号 | 类型 | 读写 | 含义 |
|---|---|---|---|---|
| `axis4_fwd_req` | `G.axis4_fwd_req` | bool | 写 | axis4 正向点动请求 |
| `axis4_rev_req` | `G.axis4_rev_req` | bool | 写 | axis4 反向点动请求 |
| `axis4_manual_busy` | `G.axis4_manual_busy` | bool | Notification + 初值批量读 | axis4 手动控制正忙 |
| `axis4_manual_done` | `G.axis4_manual_done` | bool | Notification + 初值批量读 | axis4 手动控制完成 |
| `axis4_manual_error` | `G.axis4_manual_error` | bool | Notification + 初值批量读 | axis4 手动控制报错 |
| `axis4_manual_error_id` | `G.axis4_manual_error_id` | unsigned long | Notification + 初值批量读 | 错误码 |

### 3.6 顶层状态/握手位

| 常量 | PLC 符号 | 类型 | 读写 | 含义 |
|---|---|---|---|---|
| `self_check_done` | `G.self_check_done` | bool | Notification + 初值批量读 | PLC 自检完成 |
| `startup_loading_ready` | `G.startup_loading_ready` | bool | Notification + 写清除 | 标准装卸启动一次性资格位；启动入口读取，开始控制前写 FALSE 消费 |
| `handle_reinit_req` | `G.handle_reinit_req` | bool | Notification + 写清除 | PLC 请求上位机重新同步 |
| `handle_reinit_done` | `G.handle_reinit_done` | bool | Notification + 初值批量读 | PLC 重初始化完成确认 |
| `estop_hold_req` | `G.estop_hold_req` | bool | 100 Hz 快照 + Notification | PLC 急停/保持激活 |
| `axis1_fast_return` | `G.axis1_fast_return` | bool | 写 | 轴1快退标志（PLC 用于平滑旁路） |
| `axis6_fast_retract` | `G.axis6_fast_retract` | bool | 写 | 轴6快退标志（同上） |
| `startup_smoothing_bypass` | `G.startup_smoothing_bypass` | bool | 写 | 启动准备期平滑旁路 |
| `gen_state` | `G.gen_state` | int | Notification + 初值批量读 | PLC 通用状态字 |
| `host_session_id` | `G.host_session_id` | UDINT | 100 Hz 写 | 本次上位机进程会话号；进程内保持不变，新进程接管时变化 |
| `host_heartbeat_sequence` | `G.host_heartbeat_sequence` | UDINT | 100 Hz 写 | 每个通信周期递增；PLC 连续 100 ms 未看到变化即锁存超时 |
| `host_recover_req` | `G.host_recover_req` | bool | 恢复时写 TRUE | 心跳恢复且 PLC 已停稳后，请求 PLC 清超时并进入既有重初始化流程 |
| `host_comm_timeout` | `G.host_comm_timeout` | bool | 100 Hz 快照 + Notification | PLC 看门狗超时锁存；为 TRUE 时上位机也按保持处理 |
| `app_name` | `TwinCAT_SystemInfoVarList._AppInfo.AppName` | char[64] | 初连及每次重连读取 | 当前 PLC 应用名；变化时按 PLC 重启/应用重载处理 |

PLC 看门狗超时时会置 `estop_hold_req`，清除快退、axis4 和计划回退请求，把外部参考冻结到实际位置，并对超时发生时仍在运行的计划回退/axis4 点动执行受控停止；气缸输出不在该分支改写。恢复必须满足“心跳已恢复 + 独立运动已停稳 + `host_recover_req=TRUE`”，PLC 才清 `host_comm_timeout`、消费恢复请求并置 `handle_reinit_req`，最终以 `handle_reinit_done` 确认重初始化完成。

### 3.7 定位臂 5 轴手动点动（Qt 上位机）

定位臂是新增的独立 5 轴通道，只通过 `G.arm_*` 符号控制，不扩展原 `G.refer[1..7]`、`G.Act_pos[1..7]`、`G.init_pos[1..7]` 或 `G.return_cmd[1..7]`。当前上位机只做单轴手动点动，不做正运动学、逆运动学或整体位姿联动。

| PLC 符号 | 类型/长度 | 读写 | 上位机行为 |
|---|---|---|---|
| `G.arm_manual_enable` | bool | 写 | 定位臂总使能；PLC 默认 FALSE，必须先写 TRUE，关闭时 PLC 关闭所有定位臂 MC_Power 并清 jog/单轴上电请求 |
| `G.arm_enable_req` | bool[5] | 写 | 每轴“上电”按钮保持 TRUE/FALSE；实际 MC_Power 使能还受 `G.arm_manual_enable` 门控 |
| `G.arm_reset_req[i]` | bool | 写 | 每轴“复位”按钮写 TRUE；PLC 先停轴、再执行 MC_Reset，消费后自动清零，上位机不主动写 FALSE |
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
if (ads.OpenCommInsideReadOnly()) {
    // 优先：本地 AMS 路由，端口 851
} else if (ads.OpenCommReadOnly()) {
    // 回退：远端 AMS NetId = 169.254.119.135.1.1
} else {
    // 两者都失败 → 退出程序
}
// 启动 AdsCommunicationService；设备必须已经处于 ADSSTATE_RUN
```

- `OpenCommInsideReadOnly` 走本机 AMS Router（要求本机装了 TwinCAT XAR 或 ADS Router 路由配置）。
- `OpenCommReadOnly` 走远端 NetId（`hardcoded_ads_netid = "169.254.119.135.1.1"`，链路本地地址，对应 Windows 防火墙规则的 169.254.x.x 段）。如果要换 PLC，**改这个常量**。
- 失败时 `ads.GetLastError()` 返回的字符串直接打印。
- 连上后读 `app_name` 验证目标 PLC；通信服务在每次重连初始化时再次读取，并用应用名变化辅助识别 PLC 重启/应用重载。
- 正常通信服务只接受已经处于 `ADSSTATE_RUN` 的设备，不替现场切换 PLC 状态，也不执行 TwinCAT **Activate Configuration**。
- 本轮没有修改 EtherCAT、NC 或 PDO 映射，因此部署不要求 Activate Configuration；PLC 程序/GVL 变更按现场既有的编译、登录和下载流程执行即可。

### 4.1 传感器重新标定工具的只读连接

`main.cpp` 在控制台 UTF-8 初始化后立即识别 `ADS.exe --sensor-calibration`，并在手柄对象、WPF 管道、运动控制对象和正常主循环初始化之前分流到 `sensor_calibration_experiment::run`。因此标定模式不会进入正常控制链路。

完整实机标定的连接顺序如下：

1. 先校验四份原标定 CSV 并等待现场安全确认；`--validate-only` 和 `--self-test` 均不会连接 PLC。
2. 调用 `OpenCommInsideReadOnly()` 尝试本地 AMS 路由；失败时调用 `OpenCommReadOnly()` 回退到远端 AMS NetId `169.254.119.135.1.1`，目标端口均为 851。
3. 每次连接成功后调用 `ReadDeviceState()`，仅当 ADS 状态严格等于 `ADSSTATE_RUN` 才开始采样。若不是 RUN，工具关闭当前连接并报告状态，随后继续连接回退或结束；它不会替用户切换 PLC 状态。
4. 建链成功后显示实际连接方式，并要求操作者确认这是目标 PLC；未确认时立即关闭连接并保存 `_未完成` 诊断报告。

`OpenCommInsideReadOnly()` 和 `OpenCommReadOnly()` 只开端口、设置超时并调用 `AdsSyncReadStateReqEx` 确认设备状态可读，不调用状态切换 API。正常 100 Hz 通信服务同样要求设备已经处于 `ADSSTATE_RUN`，连接或重连不会替用户切换 PLC 状态。标定模块本身不调用 `ADSWrite`、`ADSWriteSum` 或任何 `plc_io::write_*`；它只解析符号句柄、读取设备状态和读取四路原始计数。`ADSReadSumByHandle` 在底层使用 `AdsSyncReadWriteReqEx2(..., ADSIGRP_SUMUP_READ, ...)` 发送批量读描述符，该请求不会写入 PLC 符号值。

运行完整标定前仍必须关闭正常 ADS 控制程序并保持机器人静止。"只读"只表示本工具不写 PLC，不代表可以与另一个正在控制机器人的进程并行运行。

## 5. 读写封装语义（正常模式 `plc_io::` / 标定工具直连）

正常控制模式同时持有 `AppContext::ads` 和 `AppContext::ads_service`。前者只用于服务启动前的初始化和无服务兼容路径；服务启动后，高频周期与运行期专项低频 ADS 都由 `AdsCommunicationService` 工作线程串行执行。标定工具是独立模式，持有自己的局部 `CADSComm`，只调用状态读取和批量读取接口。

### 5.1 周期读写

| 路径 | 调用频率 | 行为 |
|---|---|---|
| `AdsCommunicationService::read_fast_snapshot` | 目标 100 Hz | 一次 Sum Read 获取任务 CycleCount 首尾值、DcTaskTime、7 轴位置、四路力原始值、`estop_hold_req` 和 `host_comm_timeout`；QPC 调用前后中点作为整帧时刻。 |
| `AdsCommunicationService::write_output_cycle` | 目标 100 Hz | 每周期只调用一次 Sum Write：始终写 `host_session_id` / `host_heartbeat_sequence`；运动有效时同包写 `refer` 和快退位，离散输出只在变化时追加，恢复握手、至多一条计划回退命令和正常完成清 Req 掩码按需并入。 |
| `read_plc_state` | 主循环按需复制 | 绑定通信服务后只复制最新有效位置快照和坐标缓存，不再发起新的 ADS 读取；无服务的兼容路径才直接 Sum Read。 |
| `read_force_sample` | 力反馈/专项门控按需复制 | ADS 模式绑定通信服务后只复制最新有效力快照并完成 `INT -> V`；TCP_DAQ 模式另取 TCP worker 最新帧。 |
| OnChange Notifications | 变量变化时 | `self_check_done`、axis1/axis6 计划回退 Busy/Done/Error/ErrorId、重初始化、急停/看门狗、启动资格、axis4 状态和 `gen_state` 由回调更新；注册完成后先批量填充初值。回调线程只复制数据，不发起 ADS 请求。 |
| `sensor_calibration_experiment::read_all_channels` | 仅 `--sensor-calibration` 实机模式；每点按单调时钟 50 ms 截止时间调度，目标 20 Hz、5 秒，共 100 次尝试 | 一次 `ADSReadSum` 同步读取 `ft_1 / fn_1 / fn_2 / ft_2` 四个 `short` 原始计数；错过截止时间后重建节拍而不突发追赶，不写 PLC 状态或变量，并在报告中记录实际耗时、尝试频率和调度超期次数；低于 18 Hz 强制重测。 |
| 固定容量计划回退队列 | `Prepare/Commit/Clear`，每个 100 Hz 周期至多选择一条 | 主循环只复制命令并取得序号；32 槽 SPSC 命令环和 128 槽结果环均预分配。命令字段并入本周期唯一一次输出 Sum Write，选中命令时优先于通用低频请求。 |
| 通用运行期专项请求队列 | 启动准备和其他低频状态操作 | 主循环提交请求并等待有界结果；仅在本周期没有计划回退命令时由通信线程执行一项，不与快速周期并发访问端口。 |
| `read_v_limit` / `write_v_limit` | 启动准备 | 服务启动后经专项请求队列读 / 写 `G.v_limit`。 |
| `read_startup_loading_ready` / `write_startup_loading_ready` | 进程启动探测、启动准备/直接控制入口 | 初始化前可直读；服务启动后经专项请求队列消费 PLC 标志。 |

`G.startup_loading_ready` 与实际位置组成双重判定：标志为 TRUE，且 axis1/3/5/6 距左限位分别处于 `[96,280,430,580] ±2 mm`，才执行标准装卸启动。否则执行中断恢复启动。旧 PLC 没有该符号时，上位机打印一次兼容提示并仅按实际位置判断；这不会改变旧 7 轴 ADS 数组布局。

### 5.2 计划回退时序

```cpp
PlannedReturnCoordinator
  ClampSettle
       // 发布完整夹爪组合并记录 output generation
       // 等 applied_output_generation 确认 Sum Write 成功后才开始 50 ms
  -> SubmitRequest
       // prepare：一次 Sum Write 同时写全部腿 Req=false + 完整参数
       // 状态确认空闲后，先确认快退旁路/冻结参考 output generation
       // 再记录每条腿 Notification 基线并提交 commit
       // commit与已确认的保护输出在下一次 Sum Write 中同时携带
  -> AwaitAccepted
       // 本次请求之后的 Busy=TRUE 或 Done=FALSE 才是 ACK
  -> AwaitDone
       // ACK 之后的新 Done=TRUE，且当前 Busy=FALSE
  -> AwaitFreshSnapshot
  -> nonblocking rebase from the same fresh 100 Hz snapshot
  -> PublishHandoff / AwaitHandoffApplied
       // AdsOutputCommand持续携带各腿Req=false掩码
       // 同一Sum Write确认清Req + 实际位置保持 + 快退关闭 + 夹爪恢复
  -> OptionalLead only for normal catheter delivery

cancel / fault
  -> async Clear(exact active legs, when PLC request may be active)
  -> CancelWait
       // 等Clear结果、Busy/Error清零、必要的新Notification和Clear后的新鲜快照
  -> nonblocking rebase from actual position
  -> publish actual-position hold + fast-return FALSE
  -> wait applied_motion_output_generation >= cancel_hold_generation
```

- 协调器任一时刻只拥有一组计划动作；普通导管可同时包含 axis1 与 axis6 联动腿，协同模式由单一 owner 串行执行，同拍冲突时 axis1 优先。
- 夹爪稳定计时从对应输出 generation 已被通信线程成功应用后开始，而不是从主线程首次构造夹爪命令时开始。这样 50ms 表示 PLC 已收到命令后的稳定时间，不包含输出排队或临时 ADS 失败时间。
- 主线程通过 `submit_planned_return_command()` 把 `Prepare/Commit/Clear` 复制到固定 32 槽 SPSC 环形队列，提交过程不调用 ADS、不等待通信线程，也不进行每拍堆分配；0 表示参数非法、服务未运行或队列已满。每条命令使用单调序号，其 Pending/Success/Failure 与失败 Commit 的逐轴 `possibly_started_mask` 保存在固定 128 槽结果环，结果槽被更晚序号覆盖时查询会明确失败。
- 通信线程在快速 Sum Read 成功后，从固定队列至多选择一条计划回退命令，把其字段与本拍 refer、快退、离散输出和心跳合并进唯一一次 Sum Write；随后才发布本拍快照。有回退命令或正常完成清 Req 掩码时，本拍不再处理通用低频请求。`Prepare` 与 `Commit` 仍分成两阶段：第一阶段在 `Req=FALSE` 时写完整参数；第二阶段只有在包含快退旁路/冻结参考的 `commit_guard_generation` 已应用后才允许置 `Req=TRUE`，因此 Commit 不会与旧输出配对。
- `CADSComm::ADSWriteSumByHandle` 可返回 transport 状态和每个子项的 ADS 结果。常规输出、正常完成清 Req 掩码、主机会话和心跳属于核心项：任一失败都令整拍无效且不推进 `applied_output_generation`。固定队列计划字段若单独失败，则核心 100 Hz 拍仍可有效，但该命令序号返回 Failure。对于 Commit，通信线程同时返回逐腿 Req 子项成功掩码；若事务没有完整响应，则保守把本次 Commit 的全部腿标记为“可能已启动”。主线程锁存该状态，即使 Busy/Done Notification 尚未到达也禁止整组自动重发。
- 每次提交 `Clear` 都会立即提高它所包含轴的 `planned_return_clear_barrier_sequence`。消费者遇到更早且触及同轴的尚未开始 `Prepare/Commit` 时，直接将旧命令标记 Failure 并出队，不发 ADS 请求；Clear 本身仍保留 FIFO 顺序。该屏障不能撤销已经执行的 ADS 写，已启动动作仍由 PLC Req 下降沿受控停止。
- ACK 等待默认上限 `250 ms`；从第一条腿 ACK 起，全部执行腿完成等待默认上限 `5000 ms`。Prepare 写失败、成功 Commit 后仍未收到 ACK、或 PLC 在 ACK 前报告错误时，只有确认所有腿均未启动才允许进入 `RetryClear`。失败 Commit 中任一 Req 子项成功、Commit 传输结果未知、结果槽不可判定或 Commit 结果等待超时时，都按“可能已启动”进入故障/取消闭环，绝不自动重发。
- 安全重试最多一次：先清本次全部 Req，等待下一份新鲜有效快照并确认 axis1/axis6 对应状态均为 `Busy=FALSE、Error=FALSE`，重新校验目标窗口和 axis6 软限位，然后进行第二次、也是最后一次下发。任一腿已 ACK/启动后禁止重试；清理失败、`500 ms` 内状态未复位或第二次失败都进入故障保持。
- 故障和安全重试通过固定回退队列异步清理本任务的精确执行腿；正常完成则由交接 `AdsOutputCommand` 的逐轴清 Req 掩码完成，使 Req=FALSE 与实际位置保持、快退关闭和夹爪恢复处于同一次 Sum Write。Notification 回调均不发起 ADS 写入。广域 `clear_axis1_group_return_requests()` 只保留给连接/重同步、PLC 故障或全局清理 1/3/5/6 四路 Req 的兼容路径。
- 清除活动动作的 `Req` 不是“立即停轴”命令：PLC 在 `return_state=10/20/30/31` 收到 `Req=FALSE` 时转入状态 40，维持 `Busy=TRUE、Done=FALSE` 并使用 `MC_Stop` 受控停止。只有真实 `MC_Stop.Done` 才进入状态 30/31，按实际位置重建 `refer`、平滑链并恢复外部设定值；无错误的主动取消最终也不置 `Done=TRUE`。
- 状态 40 若收到 `MC_Stop.Error`，PLC 在没有更早运动错误时锁存首次 Stop `ErrorID`，立即发布 `G.return_cmd[N].Error/ErrorId`，并让 Execute 至少拉低一拍后继续尝试受控停止；已有原运动错误时保留原错误。持续 Stop Error 因而表现为 `Busy=TRUE、Done=FALSE、Error=TRUE`，不会无限静默重试或本地伪装完成，axis1/axis6 的 Error Notification 会把该状态送到上位机。
- 上位机取消路径保留 `PlannedReturnCoordinator::CancelWait` 和原任务腿。Clear 成功只表示 Req 写入已完成；还要等待每条腿 `Busy=FALSE、Error=FALSE`，对取消时已经 started/Busy 的腿等待基线之后的新 Notification，并取得 Clear 后的新鲜有效快照完成非阻塞 rebase。若正常交接的 `handoff_generation` 已经应用，则该代次本身已经证明 Req 清除成功，取消链直接复用这一事实，不再等待一个不会发生的重复 Notification。随后即使常规运动输出已因暂停/回退故障关闭，也会单独发布一次“当前实际位置保持 + 快退标志 FALSE”，直到 `applied_motion_output_generation >= cancel_hold_generation` 才 reset 协调器；该专用高水位只在 Sum Write 确实包含 `motion_enabled=true` 的 `refer` 时推进，急停或无效拍产生的纯心跳/清旁路高代次不能冒充保持命令。取消等待超过 5 秒会锁止控制并清力输出，但仍继续清 Req、等待 PLC 停稳和保持输出确认。

### 5.3 重同步路径

主循环在以下情况会触发重同步（`sync_all` / `sync_axis1` / `sync_axis6`）：

- 启动：自检完成或直接控制进入时 `sync_all(30)`。
- PLC 主动请求：读到 `G.handle_reinit_req == true` -> 清除该位 + `sync_all(30)`。
- PLC 急停保持解除：`sync_all(20)`。
- 六种模式的正常计划回退完成：在 Done 之后等待同一份新的有效 100 Hz 快照，按 Axis1、Axis6 或 Cooperative 范围使用该快照和主循环已有滤波手柄值执行非阻塞内存基准重建；不重新轮询手柄，不调用 `Sleep()`，也不再调用 `sync_axis1(3)` / `sync_axis6(3)` / `sync_cooperative_guidewire(3)`。
- 只有普通导管递送在统一输出 generation 确认后可进入默认 `1 mm` axis1 自动先行；导管撤出、独立导丝双方向和协同双方向均直接恢复 Follow。自动先行完成后再按最新快照执行一次 axis1 非阻塞重建。
- 取消路径不调用阻塞式 `sync_*`：`CancelWait` 在 Clear 和 PLC 停稳之后用同一份新鲜快照重建，并等待实际位置保持输出 generation。普通导管自动先行也有独立 5 秒上限；超时进入相同故障保持/取消闭环，不把旧目标继续留给 PLC 平滑链追赶。
- 导丝模式进入前：先清 `G.return_cmd[6].Req` 并确认 `Busy=false`，随后执行 `sync_axis6` 或独立模式同步；退出时执行 `sync_axis1`。
- 主循环兜底：`startup.completed && !control_active && !freeze && !estop_hold` 时每拍 `sync_all(20)` 尝试恢复。

阻塞式 `sync_*` 继续用于启动、显式模式切换、PLC 保持解除和故障恢复；正常计划回退完成路径改用 `rebase_axis1_after_return`、`rebase_axis6_after_return`、`rebase_cooperative_after_return`。这些函数只消费当前内存快照与滤波手柄值，统一 refer、保持轴、窗口和上一拍差分基准；随后由 `publish_output()` generation 确认交接输出已经进入 100 Hz Sum Write。

## 6. 频率/节拍约定

| 操作 | 频率/时限 | 调度与用途 |
|---|---|---|
| 统一快速 Sum Read | 100 Hz，约 10 ms 周期 | `AdsCommunicationService` 独立线程用 QPC 绝对截止时间调度；快照的 QPC 时刻取整次调用前后中点 |
| 统一输出 Sum Write | 100 Hz | 与快照周期同一工作线程串行执行；包含主机心跳，运动/离散输出按有效性和变化追加 |
| `force.csv` | 100 Hz | 直接消费主机侧 ADS 快照队列，每个尝试序号最多一行；无效帧保留时刻并写 `NaN` |
| `motion.csv` | 50 Hz | 取 100 Hz 快照的偶数 `attempt_sequence`；位置与力来自同一快速快照体系，不做额外 ADS 补读 |
| 设备状态检查 | 每 10 个通信周期，约 100 ms | 确认仍为 `ADSSTATE_RUN`；非 RUN 按 PLC 重启/停机处理 |
| 首次失败软保持 | 第 1 个失败周期立即 | 停止发布运动有效命令、清力反馈并丢弃故障期间手柄增量，但暂不关闭连接 |
| 硬超时与重连 | 距最后完整成功 100 ms | 关闭端口并按 `250/500/1000/2000 ms` 上限退避重连 |
| OnChange Notifications | 状态变化触发 | 更新 axis1/axis6 计划回退结果及其他低频状态，替代对应的主循环轮询；回调只更新内存状态 |
| 固定容量计划回退队列 | 每个 100 Hz 周期至多一条 | 先选取命令并合入本拍唯一 Sum Write，成功后再发布快照；32 条待执行命令、128 条结果历史，Clear 屏障或完成交接清 Req 掩码可淘汰尚未开始的同轴旧 Prepare/Commit |
| 定位臂 `G.arm_*` | C++ `ArmManualAdsService` 独立 20 Hz | 命令仅在变化时 Sum Write，状态批量 Sum Read；与 7 轴 100 Hz 服务解耦。ADS 不健康、主机看门狗超时或程序退出时撤销总使能、单轴上电与点动请求。 |

### 6.1 WPF ADS 诊断

`VisState` 末尾发布 `ads_state`、实际频率、最新有效快照数据龄、最近 RTT、累计失败周期、重连次数、PLC 重启次数和 `host_comm_timeout`。WPF 主界面顶部显示：

- 状态：ADS 未启动 / 正在连接 / 正常 / 单拍软保持 / 重连中 / PLC 已重启 / 通信错误；
- 实时量：实际 Hz、数据龄、RTT；
- 累计量：失败、重连、PLC 重启。

只有命名管道已连接、`ads_state == Running` 且 `host_comm_timeout == false` 时，UI 才把 ADS 标为健康。当前 `VisState` 为 pack(1) 的 **877 字节**，C++ 与 WPF 必须成对更新。

## 7. 失败/异常处理约定

- `ADSRead/ADSWrite/ADSReadSum/ADSWriteSum` 失败只返回 false，不向控制主循环抛异常；通信服务统一累计失败周期和连续失败数。
- 第一次快照或输出失败即进入 `SoftHold`：保持最后参考、停止把运动命令标记为有效、清力反馈，并在恢复前丢弃手柄增量，避免断线期间位移一次性补入。
- 若 100 ms 内恢复完整读写，连接可直接回到 `Running`；主循环从实际位置重建 refer 和手柄基准。
- 若持续 100 ms 未完整成功，服务关闭 ADS 端口并自动重连。**同一运行实例的链路重连**会刷新坐标缓存、执行 PLC 看门狗恢复握手，取消中断中的计划回退、屈曲恢复、力过渡和 PI 瞬态，但保留稳定模式与方向。
- 小于 100 ms 的短时 ADS 软保持恢复同样只恢复稳定模式：重新建立实际位置和手柄基准后，明确清除回退握手、交接 generation 与自动先行阶段，不续跑断拍前的瞬态。
- **PLC 重启/应用重载**不是普通链路重连：设备非 RUN、DcTaskTime/CycleCount 回退或应用名变化都会进入 `PlcRestarted`。上位机清除力零点、关闭力反馈和 PI、退出控制并锁定启动，必须重新完成 PLC 自检、力感调零和人工启动。
- PLC 看门狗超时后，即使 ADS 已重新连上，也必须等心跳恢复、PLC 受控停止完成并完成 `host_recover_req -> handle_reinit_req -> handle_reinit_done` 握手，不能直接续跑旧运动。
- `request_axis_return` 中间任意一步写失败，会 `clear_axis_return_request` 后返回 false，下一拍重试。
- 计划回退状态的 Notification 注册或初值读取失败时不拼接不完整的 Busy/Done/Error；通信服务按连接故障路径处理，主循环保持实际位置。暂停、急停、自检重建或 ADS 故障取消活动换手时进入 `CancelWait`，精确清本任务执行腿并保留任务上下文；清请求失败按 250 ms 间隔重试。只有 Clear 结果、PLC Busy/Error、取消后新鲜快照和已写入 `refer` 的 motion output generation 全部确认，才允许主机状态回到 Follow。
- 无效的 100 Hz 快照仍进入主机侧记录队列，CSV 以有效位和 `NaN` 表达空洞；不会让后续新鲜数据伪装成故障时刻的数据。
- 标定工具将一次四路 `ADSReadSum` 失败计为一次 ADS 失败，该时刻四路数据均不进入统计，不用不完整快照拼接样本。每点完成 100 次采样尝试后，目标通道经"中位数 ±300 counts"过滤的有效样本少于 80 个时必须重测。
- 标定工具连接失败、设备状态不是 `ADSSTATE_RUN` 或状态读取失败时不会开始实验，并尽量保存带 `_未完成` 后缀的连接诊断 TXT；用户在任一交互阶段中止时也保存 `_未完成` 文件。只要四路整体未完成，报告就抑制全部拟合公式、`可用` 结论和部署建议。

## 8. 设计边界 / 待确认点

### 8.1 不在 PLC 内建立采样环形缓冲

当前记录链路使用 C++ 主机侧 `snapshot_queue_` 和 `AdsFastSnapshot::attempt_sequence`，不新增 PLC 环形缓冲区、记录数组或 `write_sequence` 符号。它能显式统计主机队列丢帧、无效帧和序号空洞，但不宣称提供 PLC 内逐扫描的无损历史。若未来要求 PLC 扫描级审计，应单独设计并评审 PLC 缓冲契约，不能把主机序号改名后当作 PLC 序号。

### 8.2 不猜测 PDO 映射，也不要求 Activate Configuration

程序只解析明确的 ADS 符号句柄：优先 `G.axis[N].NcToPlc.ActPos`，不可用时回退 `G.Act_pos[7]`。本轮不读取裸 IndexGroup/Offset 去猜 EtherCAT PDO，不修改 I/O、NC、EtherCAT 或 PDO 映射，因此不要求执行 TwinCAT **Activate Configuration**。

### 8.3 ADS 力输入映射需要现场确认

当前默认 `force_sample_source == ADS`。`zero_force_sensor` 会同步读取一帧 ADS 样本后采零，不依赖力反馈开关；成功后立即清除旧的快退冻结反馈和手柄输出。现场必须确认 `G.fn_1_value`（轴向力）和 `G.ft_1_value`（切向力）均为有符号 `INT`，且满足 `1000 counts = 1 V`。装机零点通过 `p × (raw-raw_zero)` 消除，固定截距不再加入增量输出；符号或量程不一致仍会影响标定与反馈方向。TCP_DAQ 尚未证明与本次 ADS 标定同源，因此当前 TCP 模式明确拒绝采零和反馈输出。

### 8.4 坐标缓存刷新边界

`init_pos` / `leftlimit` 在连接初始化时批量读取并缓存，普通 100 Hz 快照只更新实际位置。计划修改零点或限位时必须显式调用 `request_coordinate_refresh()`，否则上位机继续使用旧缓存；重连路径会自动请求刷新。

## 9. 修改 ADS 相关代码时的维护要求

任何修改 `ads_communication.cpp/.h`、`plc_io.cpp/.h`、`ADSComm.cpp`、新增/删除 PLC 符号、改变读写频率或失败处理策略的工作，**必须**：

1. 在本文档"§10 变更日志"追加条目，注明：日期 / 变更人 / 涉及文件与函数 / 行为变化要点 / 是否影响与 PLC 的契约（需要 PLC 侧配合改吗）。
2. 若新增 PLC 符号，更新 §3 符号表 + `plc_io.cpp::AdsSymbol` 常量定义。
3. 若新增 axisN_return FB 实例，更新 §3.4 + 在 `plc_io.cpp` 末尾添加对应 `AxisReturnAdsSymbols` 常量。
4. 若改了 `sync_*` 内部的步骤顺序，更新 §5.3 + 同步告知 [运动流程说明.md §3](运动流程说明.md#3-窗口与跟随基准)（窗口/同步流程的契约）。
5. 若改了连接拓扑（如新增冗余 NetId、TLS），更新 §1 拓扑图 + §4 连接建立。

## 10. 变更日志

### 2026-08-03 — 实机 F_direct 接入正常力反馈
- 作者：AI（Codex）。
- 行为变化：ADS/TCP 的 `INT -> V` 通讯契约不变；传感器2 `fn_1` 和传感器1 `ft_1` 在反馈层分别使用 `0.614437208097 N/V`、`0.703683250522 N/V`，装机调零后按电压差映射，截距抵消。
- 零点与安全：原键盘/UI 调零入口保留，成功后立即清旧输出；公式只在本次 0～50 g 与报告 raw 范围内验证，反馈默认限幅同步收至50g量级。TCP 回退在完成同源验证和独立验收前拒绝采零。

### 2026-08-03 — 100 Hz ADS 快照、自动重连与 PLC 主机看门狗
- 作者：AI（Codex）。
- 涉及文件：`ads_communication.*`、`ADSComm.cpp`、`ADS/Include/ADSComm1.h`、`plc_io.*`、`main.cpp`、`experiment_recorder.*`、`vis_server.*`、`AdsControlUI`，以及 PLC `G.TcGVL` / `handle.TcPOU`。
- 通信变化：正常模式改为独立 100 Hz QPC 通信线程；快速输入走 Sum Read、输出和心跳走 Sum Write，低频状态走 OnChange Notifications。第一次失败立即软保持，距最后完整成功 100 ms 后关闭端口并退避重连。
- 恢复边界：同一 PLC 运行实例重连会重建位置/手柄基准并清瞬态；PLC 重启或应用重载会清力零点、关闭反馈/PI、退出控制，要求重新自检、调零和人工启动。
- PLC 契约：新增 `G.host_session_id`、`G.host_heartbeat_sequence`、`G.host_recover_req`、`G.host_comm_timeout`，超时后 PLC 冻结外部参考并受控停止独立运动，再通过重初始化握手恢复。
- 记录与 UI：`force.csv` 对齐 100 Hz 快照，`motion.csv` 为稳定 50 Hz；`session.json` 写 ADS/序号诊断，WPF 显示状态、实际频率、数据龄、RTT、失败、重连和 PLC 重启；该次 `VisState` 扩展为 499 字节，后续版本见 2026-08-04 的 877 字节协议。
- 明确边界：不新增 PLC 环形缓冲或 `write_sequence`，不猜测 PDO 映射，不要求 Activate Configuration，连接/重连不替现场切换 PLC 到 RUN。

### 2026-08-03 — 新增四路传感器只读重新标定模式
- 作者：AI（Codex）。
- 涉及文件：`sensor_calibration_experiment.cpp/.h`、`main.cpp`、`ADSComm.cpp`、`ADS/Include/ADSComm1.h`、`ADS.vcxproj`。
- 行为变化：新增 `ADS.exe --sensor-calibration` 独立工具模式，在手柄、WPF 和运动主循环初始化前分流；按传感器1/2/3/4分别对应 `G.ft_1_value/G.fn_1_value/G.fn_2_value/G.ft_2_value`，每个采样时刻用一次 `ADSReadSum` 同步读取四路 `INT16` 原始计数。每点按 20 Hz、5 秒进行 100 次采样尝试，并对 ADS 失败和有效样本不足进行显式处理。
- 连接策略：标定工具通过专用 `OpenCommInsideReadOnly()` / `OpenCommReadOnly()` 实现本地 AMS 路由优先、远端 NetId 回退，连接后必须通过 `ReadDeviceState()` 确认 `ADSSTATE_RUN`；所有现行连接接口都只读取设备状态，不替操作者切换 PLC 状态。标定工具不调用 ADS 变量写入接口，也不自动修改 PLC 或上位机标定参数。
- 契约影响：没有新增、删除或重命名 PLC ADS 符号；正常控制主循环及既有 WPF/PLC 通讯契约不变。

### 2026-07-28 — 新增标准装卸启动资格 ADS 握手
- 作者：AI（Codex）。
- 涉及文件：`plc_io.cpp/.h`、`startup_sequence.cpp/.h`、`main.cpp`，以及 PLC `G.TcGVL/init.TcPOU/SelfCheck.TcPOU`。
- 行为变化：绑定 `G.startup_loading_ready`；上位机启动时探测符号，每次启动准备读取，开始启动准备或直接控制前写 FALSE 消费。标志与 `[96,280,430,580] ±2 mm` 实际位置共同决定标准/中断恢复路径；旧 PLC 无符号时只按位置兼容判定。
- 契约影响：新增单个 BOOL 符号；原 7 轴高频读写、定位臂通道和 328 字节 WPF/C++ 命名管道协议不变。

### 2026-06-26 — 文档初版（无代码变更）
- 作者：AI（Claude，应用户要求梳理）。
- 内容：基于现网代码（`ADSComm1.h`、`plc_io.cpp/.h`、`control_types.h` 中的 `AppContext`、`main.cpp` 主循环前段的 ADS 连接与 lambda 包装、主循环步骤 4/8/13 的自检/重同步/axis4 诊断段）编写第一版说明。
- 初版曾记录四处当时的通讯限制；现行失败处理、应用名复核和位置快照语义均以 2026-08-03 条目及正文为准。
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

### 2026-08-04 — WPF 定位臂控制、axis4 点动与注射器轴预留
- 作者：AI（Codex）。
- 涉及文件：`arm_manual_ads_service.*`、`main.cpp`、`vis_server.*`、`AdsControlUI`，以及 PLC `G.TcGVL` / `MAIN.TcPOU` / `ArmManual.TcPOU` / `250902.tsproj`。
- 行为变化：维护中的 WPF 界面新增定位臂五轴单轴上电、复位、按住 Jog-/Jog+、参数编辑和状态显示；总使能在 ADS 健康后自动请求，任一轴仍需单独上电。axis4 新增按住前进/后退按钮，C++ 继续保留既有反接线交换，因此 UI 物理语义不变。
- 通讯与安全：定位臂读写移入独立 20 Hz ADS 工作线程，命令仅在变化时下发；WPF 按住点动时每 100 ms 续约，C++ 在 300 ms 未续约后自动停止。主机看门狗超时、ADS 不健康或程序退出时会清总使能、单轴上电和点动。原 7 轴 100 Hz `G.refer/return_cmd` 包没有新增字段。
- 注射器：`G.inject_axis[1..2]` 仅链接 NC Axis 13/14（Drive 16/17），没有 `MC_Power`、运动命令或上位机 UI。
- 本地协议：新增命令 29..34 和定位臂/axis4 状态，`VisState` 扩展为 pack(1) **877 字节**；C++ 与 WPF 必须同时更新并重启。Qt 定位臂界面保留为历史实现，不再作为维护入口。

### 2026-07-26 — 主从位移实验与 ADS 力采样首选迁移
- 作者：AI（Codex）。
- 涉及文件：`plc_io.cpp`、`control_types.h`、`main.cpp`、`delivery_tracking.*`、当时的独立记录模块（现已删除）、可视化管道与 WPF。
- 行为变化：默认力源改为 ADS；`G.fn_1_value/G.ft_1_value` 在 C++ 中按 `raw/1000.0` 转换为伏特，ADS 模式下 UI 零点采集同步读取一帧 ADS 数据。TCP 保留为显式代码回退，不自动切换。
- 采样节拍：当时的按需 ADS 力采样和进程启动自动 CSV 均已被后续统一记录及 100 Hz 快照服务替代。不改变力变量的 `INT -> V` 量纲约定。

### 2026-07-27 — 协同撤出可视化协议扩展（无 PLC ADS 符号变更）
- 作者：AI（Codex）。
- 涉及文件：`main.cpp`、`vis_server.h`、`AdsControlUI/VisProtocol.cs`、`AdsControlUI/VisPipeClient.cs`、WPF 模式区。
- 行为变化：新增 UI 命令 `SetCooperativeRetraction=23`；`VisState` 末尾追加 `int cooperative_direction`，编码为 `0=None`、`1=Delivery`、`2=Retraction`，用于显示实际激活的协同方向。
- 协议约束：C++ 与 C# 都按 Pack=1 的 327 字节布局编译，新旧上位机/WPF 不能混用，更新后必须一起重启。本次没有新增 PLC ADS 符号，也没有修改 `G.refer[1..7]`、计划回退或定位臂通道。

### 2026-07-27 — axis6 软件限位状态与 axis1 回退后先行（无 PLC ADS 符号变更）
- 作者：AI（Codex）。
- 涉及文件：`main.cpp`、`control_types.h`、`vis_server.h`、`AdsControlUI/VisProtocol.cs`、`AdsControlUI/VisPipeClient.cs`、WPF 参数输入区。
- 行为变化：axis6 的 `670 mm from-left` 保护完全在上位机控制层计算，未增加 ADS 读写符号。预测到 axis6 手柄/计划回退/axis1 联动回退的最终目标越限时，不写对应 PLC `return_cmd`，只阻断当前危险动作并在后续周期重新评估；实际位置越限时继续冻结相关链路，直到回到限制内。现有 PLC 与 NC 的硬限位、`G.refer[1..7]` 及定位臂通道均不改变。axis1 回退完成后的自动先行量仅经本地命名管道写入 C++ 配置，不通过 ADS 下传到 PLC。
- 协议变化：新增 `SetAxis1PostReturnLead=24`，`param1 = mm * 1000`；当前允许范围为 `[0,10]`，命令号未改变。`VisState` 末尾追加 `bool axis6_soft_limit_hold`。Pack=1 布局由 327 字节扩展为 **328 字节**，C++ 与 WPF 必须同时更新并重启，禁止与 327 字节版本混用。

### （此处持续追加）

### 2026-07-29 — 统一记录 QPC 时间与按需位置补读（历史实现，已被 100 Hz 快照版替代）
- 作者：AI（Codex）。
- 涉及文件：`plc_io.cpp`、`main.cpp`、`experiment_recorder.*`。
- 行为变化：该版本首次引入 ADS 调用前后 QPC 中点和统一记录；其独立时隙截取与位置补读策略现已由 2026-08-03 的统一快照队列替代。
- 接口影响：当时没有修改 TwinCAT ADS 符号；本地管道布局后续先扩展到 499 字节，当前为 877 字节。

### 2026-07-28 — 轴3/5/6几何默认值与 axis6 窗口调整（无 ADS 符号变更）
- 作者：AI（Codex）。
- 行为变化：上位机启动最终默认值改为 axis1/3/5/6=`20/649/649/650 mm`，标准启动中间 axis5/6 间距改为 `15 mm`，运行时 axis6 窗口改为 `[axis5+1,axis5+21]`。axis1/axis6 在重启或重同步后已到达/略微越过触发边时，可由新的有效同向输入重触发换手。
- 接口影响：无新增或修改 ADS 符号；不改变 `G.refer[1..7]`、计划回退结构、TwinCAT 状态机、NC 映射、定位臂通道或 328 字节 WPF/C++ 管道布局。

### 2026-08-11 — 计划回退状态 Notification 与 axis1 自动先行
- 作者：AI（Codex）。
- 通讯变化：该阶段先把 axis1 计划回退的 Busy/Done/Error/ErrorId 改为 OnChange Notification，实际位置与 `NcToPlc.ActVelo` 使用 100 Hz 快照；随后同日的统一协调器改造把 axis6 也纳入相同 Notification 链，当前 axis1/axis6 都不再按拍发起专项状态 Sum Read。
- 运动变化：普通导管正向递送的 axis1 回退完成后自动执行 `[0,10] mm` 配置先行，默认 `1 mm`；取消先行前 `40 ms` 和到位后 `30 ms` 固定计时，改用新鲜快照、输出 generation 与连续到位样本驱动；回退前电缸 `50 ms` 稳定等待保留。
- 接口影响：继续使用 `SetAxis1PostReturnLead=24`，命令号、`VisState` 与 PLC ADS 符号布局不变。

### 2026-08-11 — 双轴 Notification 与统一计划回退协调器
- 作者：AI（Codex）。
- 通讯变化：axis6 的 Busy/Done/Error/ErrorId 与 axis1 一样改为 OnChange Notification，并各自锁存 ACK、完成、故障及最近非零错误码边沿；六种模式的回退由 `PlannedReturnCoordinator` 统一消费这些事件。
- 重试与交接：夹爪输出 generation 应用后才开始 50ms 稳定计时；计划动作使用 `Req=FALSE + 参数` 与 `Req=TRUE` 两阶段批量写。请求写入失败、ACK 超时或启动前错误仅在所有腿尚未启动时允许一次安全清理重试；动作一旦 ACK/启动就绝不重复下发。正常完成统一等待下一份 100 Hz 新鲜快照并非阻塞重建，再等待输出 generation 应用；活动动作取消则由 PLC 状态 40 的 `MC_Stop` 受控收尾。
- 接口影响：继续复用原 `G.return_cmd[1..7]` 和现有命名管道命令；未增加 PLC ADS 符号或 WPF 状态字段，pack(1) `VisState` 仍为 **877 字节**。

### 2026-08-11 — 固定容量回退队列与状态 40 错误闭环
- 作者：AI（Codex）。
- 通讯变化：`Prepare/Commit/Clear` 使用 32 槽 SPSC 命令环和 128 槽结果环异步进入 100 Hz ADS 线程；Clear 提交时建立 axis1/axis6 逐轴序号屏障，未开始的同轴旧命令不会越过取消命令落到 PLC。
- 取消与诊断：`CancelWait` 等待精确 Clear、PLC 停稳、新鲜快照 rebase 及已写入 `refer` 的 motion output generation；执行、取消和普通导管自动先行共享 5 秒安全上限。失败 Commit 的部分成功/未知腿会被保守锁存并禁止自动重发。PLC 状态 40 会把 `MC_Stop.Error/ErrorID` 暴露为回退 Error Notification，同时保持 Busy 和受控停止重试；主动取消不产生 Done。
- 互斥与接口：力过渡实验和计划回退在两边入口互相拒绝；未增加 ADS 符号、WPF 字段或运行时依赖。
