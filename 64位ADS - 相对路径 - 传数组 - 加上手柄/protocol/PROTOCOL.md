# 主从远程通信协议 v1

适用范围：**主端**（MasterConsole，只有界面和手柄）⇄ **从端/数据端**（机器人附近的电脑，运行 `ADS.exe`，经本地网线直连倍福 PLC）。两端通过 SD-WAN 异地组网。

> 本文是协议唯一权威定义。C++（`protocol/cpp/remote_protocol.h`）、C#（`master/MasterConsole.Protocol`）、Python 测试向量（`tools/gen_protocol_vectors.py`）三份实现必须同时修改、同时通过对拍。

## 1. 设计原则

1. **手柄与界面在主端，PLC/ADS/力传感器/相机在从端。** 广域网只承载“意图”和“状态”，不承载伺服闭环。
2. **按住类动作随高频帧携带。** 手柄输入、注射器推拉都放在 UDP 控制帧里，断网后从端因帧超时自动归零，不依赖“松开”事件能送达。
3. **一次性命令走 TCP。** 准备位置、力反馈开关、电缸、Y 阀等有确认回执（已接收/已拒绝/已完成）。
4. **超时降级，不主动进故障。** 帧超时后从端把手柄输入视为无效、注射器方向置 0、力反馈输出置 0；是否进入保持由从端既有逻辑决定。
5. **单写者控制权。** 同一时刻只有一个主端持有控制租约。

## 2. 端口与传输

| 通道 | 方向 | 传输 | 默认端口 | 频率 |
|---|---|---|---|---|
| 命令/事件 | 双向 | TCP，长度前缀 JSON | 32000（从端监听） | 事件驱动 |
| ControlFrame | 主→从 | UDP | 32001（从端监听） | 100 Hz |
| HapticFrame | 从→主 | UDP | 回到 ControlFrame 的源地址端口 | 100 Hz |
| StatusFrame | 从→主 | UDP | 同上 | 15 Hz |

从端把主端最近一个**通过校验**的 ControlFrame 的源 IP:端口作为 UDP 回传目标，因此主端穿越 NAT/SD-WAN 时不需要开放入站端口。

> 与文档 VIR-SW-ARCH-001 中 A 系统（HTTP 7998 + UDP 31001/31002）的关系：沿用“低频命令 + 高频 UDP 流”的分层，但命令通道改为 TCP 长连接 JSON，因为 HTTP 无法由从端主动推送事件；端口避开 A 系统端口，避免两套系统同机时冲突。

## 3. UDP 二进制帧（小端，无填充）

不使用 `Marshal`/结构体直接转换，两端都按字段逐个读写，避免 ABI 与对齐问题。

### 3.1 帧头（16 字节）

| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | u16 | magic | 0x4956（字节序列 `56 49`） |
| 2 | u8 | version | 1 |
| 3 | u8 | type | 1=Control，2=Haptic，3=Status |
| 4 | u32 | session | 会话号，由从端在握手时分配 |
| 8 | u32 | seq | 发送方单调递增序号 |
| 12 | u32 | ts_ms | 发送方单调时钟毫秒 |

### 3.2 帧尾（8 字节）

`MAC = HMAC-SHA256(session_key, 帧头 ‖ 载荷)` 的前 8 字节。

接收方处理顺序：长度 → magic/version → type → session → MAC → seq。任一步失败直接丢弃，不回应。`seq` 必须严格大于上一个已接受序号（按 u32 回绕比较：`(int)(seq - last) > 0`），否则视为重放/乱序丢弃。

### 3.3 ControlFrame（type=1，载荷 55 字节，整帧 79）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | HandleSample | 手柄 A（序列号 582） |
| 26 | HandleSample | 手柄 B（序列号 587） |
| 52 | i8[2] | 注射器 1/2 方向：-1 拉，0 停，+1 推 |
| 54 | i8 | 轴4点动方向：-1 后退，0 停，+1 前进 |

HandleSample（26 字节）：

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u8 | buttons（`buttons2` 位掩码） |
| 1 | u8 | valid（1=本次采样有效） |
| 2 | i32[2] | encoders |
| 10 | f32[2] | joints |
| 18 | f32[2] | vels |

手柄 A、B 指**物理序列号** 582、587（不是导管/导丝角色，角色映射由从端 `swap_handle_roles` 决定）。字段与本地 SDK 的 `buttons2 / encoders2 / fJoints2 / fVels2` 一一对应。主端对未打开或本次读取失败的手柄必须置 `valid=0`。

从端只在主端持有控制权时才使用手柄采样，并且只使用 250 ms 内的最新一帧：采样过期等同于“手柄断开”，从端沿用既有的手柄软保持逻辑（保持最后参考、丢弃故障期间增量、恢复后自动重建基准），不会进入故障。

### 3.4 HapticFrame（type=2，载荷 26 字节，整帧 50）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u32 | echo_ts_ms：回显最近收到的 ControlFrame.ts_ms |
| 4 | u16 | hold_ms：从收到该帧到发出本帧的停留毫秒 |
| 6 | HapticOut | 手柄 A |
| 16 | HapticOut | 手柄 B |

HapticOut（10 字节）：u8 enable，i8 axis，f32 force_n，f32 torque_nm。axis 为力作用的 SDK 轴（从端 `axial_force_axis`，当前为 1）。主端收到后在对应轴上调用手柄 `sendForce`。从端每收到一个合法 ControlFrame 就回一个 HapticFrame（约 100 Hz）；未持有控制权时 enable 恒为 0。**主端 200 ms 内未收到有效 HapticFrame 时，必须把两只手柄的力输出置 0。**

RTT = `now_ms − echo_ts_ms − hold_ms`。

### 3.5 StatusFrame（type=3，载荷 118 字节，整帧 142）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u32 | echo_ts_ms |
| 4 | u16 | hold_ms |
| 6 | u32 | flags |
| 10 | i32 | mode（`guidewire_mode`，仅显示） |
| 14 | i32 | phase（`startup_phase`） |
| 18 | i32 | selfcheck_status |
| 22 | i32 | ads_state |
| 26 | u16[4] | cylinder_cmd（电缸 1–4 当前命令值） |
| 34 | u8 | cylinder_manual_mask（位0..3=电缸1..4 手动覆盖） |
| 35 | i8[2] | injector_active（注射器当前动作方向） |
| 37 | i8 | axis4_active（轴4当前点动方向） |
| 38 | f32[5] | force_582_f, force_582_n, force_587_f, force_587_n, clean_force_n |
| 58 | f32 | ads_actual_hz |
| 62 | f32[7] | axis_pos（预留，界面暂不显示） |
| 90 | f32[7] | axis_from_left（预留，界面暂不显示） |

flags 位定义：

| 位 | 含义 | 位 | 含义 |
|---|---|---|---|
| 0 | control_active | 6 | yvalve_closed |
| 1 | estop_hold | 7 | lease_held（本主端持有控制权） |
| 2 | self_check_done | 8 | startup_completed |
| 3 | ff_enabled | 9 | ads_healthy |
| 4 | cal_zeroed | 10 | ff_zeroing（力反馈开启前自动零点采集中） |
| 5 | host_comm_timeout | | |

## 4. TCP 命令通道

帧格式：`u32 小端长度 ‖ UTF-8 JSON`，长度上限 64 KiB。每条消息含 `"t"` 类型字段。

### 4.1 握手与认证（不在链路上传输 token）

```
M→S {"t":"hello","proto":1,"client":"MasterConsole","nonce_c":"<16B hex>"}
S→M {"t":"challenge","nonce_s":"<16B hex>"}
M→S {"t":"auth","mac":"<hex>"}          mac = HMAC-SHA256(token, "auth"‖nonce_c‖nonce_s)
S→M {"t":"hello_ack","session":<u32>,"udp_port":32001}
```

双方各自派生 `session_key = HMAC-SHA256(token, "key"‖nonce_c‖nonce_s)[0..16]`，用于 UDP 帧 MAC。`token` 是现场配置的预共享密钥（见 `config/`），不进入版本库。

> 当前版本 TCP 内容为明文 JSON，机密性依赖 SD-WAN 隧道加密。后续可在 TCP 上加 TLS，不改变本协议字段。

### 4.2 控制权

```
M→S {"t":"acquire","id":1}      S→M {"t":"ack","id":1,"state":"done"}   或 rejected + reason
M→S {"t":"release","id":2}
```

未持有控制权时，除 `hello/acquire/ping` 外的命令一律 `rejected`，且从端忽略 ControlFrame 中的手柄与注射器内容。TCP 断开或 ControlFrame 持续超时 2 s，从端释放租约。

### 4.3 命令

所有命令形如 `{"t":"cmd","id":<n>,"name":"...",...}`，从端回 `ack`：`state` ∈ `accepted`（已接收，执行中）、`done`、`rejected`（含 `reason`）。**客户端不得把 `accepted` 当作执行完成。**

| name | 参数 | 从端对应既有动作 |
|---|---|---|
| `prepare_position` | `catheter_mm`（5–95），`wire_mm`（10–639） | `SetSelfCheckAxisPos` + `StartSelfCheck`（进入器械准备位置） |
  | `start_control` | — | `SelectDirectControl`（已到达准备位置后，在当前位置直接进入手柄控制） |
  | `refresh_handles_begin` | — | 从端先暂停远程手柄输入、清除旧采样/力输出，并在主循环确认保持后返回 `accepted/done` |
  | `refresh_handles` | `success_mask`（1=582，2=587）和 `after_seq`（主端刷新后控制帧序号） | 仅接受 `after_seq` 之后的新 UDP 采样；从端重建手柄/机器人基准后才返回 `done`。部分成功保持双手柄输入暂停 |
| `force_feedback` | `enable`（bool） | 开启前自动零点采集（`ZeroForceSensor`），完成后 `ToggleForceFeedback`；零点采集期间状态帧 `ff_zeroing=1` |
| `cylinder` | `index`（1–4），`engaged`（bool） | engaged=true：电缸 1/3 写 2000，电缸 2/4 写 10（`SetCylinderManualPosition`）；engaged=false：`ResetCylinderManual` 恢复原状态 |
| `yvalve` | `closed`（bool） | `SetYValveOpen(!closed)` |
| `arm_manual_enable` | `enable`（bool） | `SetArmManualEnable` |
| `arm_axis_enable` | `axis`（1–5），`enable`（bool） | `SetArmAxisEnable` |
| `arm_axis_reset` | `axis`（1–5） | `RequestArmAxisReset` |
| `arm_axis_jog` | `axis`（1–5），`direction`（-1/0/1） | `SetArmAxisJog`；主端按住期间周期重发 |
| `arm_cartesian_jog` | `mode`（1–5），`speed`（mm/s 或 °/s ×1000） | `SetArmCartesianJog`；主端按住期间周期重发 |
| `arm_cartesian_parameter` | `field`（0–3），`value`（单位 ×1000） | `SetArmCartesianParameter` |
| `arm_program_zero` | — | `ReturnArmProgramZero` |
| `arm_stop` | — | `StopArmCartesian` + 清零各轴点动 |
| `arm_cartesian_alive` | — | `KeepArmCartesianAlive` |
| `ping` | — | 返回 `pong`，用于 TCP 保活（5 s） |

注射器推拉和轴4点动**不是**命令：它们是“按住才动”的动作，随 ControlFrame 持续发送方向，松手或断网后从端因帧超时自动归零，见 3.3 节。

> 旧本地调试台（AdsControlUI）的轴4点动是“点击开始、再点停止”并由界面周期重发；远程版改为按住式，避免广域网下动作被锁存。

已移除的旧功能（不进入远程协议）：实时力窗口、重力补偿、力反馈保持、手动零点采集按钮。

### 4.4 事件（从→主）

`{"t":"event","level":"info|warn|error","text":"..."}`：自检完成、命令被拒绝、ADS 重连等，主端显示在日志区。

## 5. 超时与降级矩阵

| 现象 | 检测方 | 处理 |
|---|---|---|
| ControlFrame 超过 200 ms 未到 | 从端 | 手柄输入视为无效，注射器与轴4点动方向置 0，力反馈输出置 0 |
| ControlFrame 超过 2 s 未到或 TCP 断开 | 从端 | 释放控制租约；恢复需主端重新握手与 acquire |
| HapticFrame 超过 200 ms 未到 | 主端 | 本地手柄力输出置 0，链路区告警 |
| StatusFrame 超过 1 s 未到 | 主端 | 界面标记“数据过期”，禁用所有操作按钮 |
| MAC/序号校验失败 | 双方 | 丢弃并计数，不回应 |

这些门限为初始值，需结合停止距离与风险分析确认（文档 13.3 节）。

## 6. 版本与对拍

- 协议字段或长度变化必须同时递增 `version`，并重新生成测试向量。
- `python tools/gen_protocol_vectors.py` 生成 `protocol/test_vectors/*`；C# 的 `MasterConsole.Protocol.Tests` 读取同一批向量逐字节比对。
- 帧长常量：Control 79、Haptic 50、Status 142。
