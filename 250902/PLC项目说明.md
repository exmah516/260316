# PLC 项目说明 / 项目内维护文档

> 本文档是倍福 TwinCAT 3 PLC 侧（`250902.sln`）的顶层入口。PLC 侧全部代码只有一个活跃 PLC 项目 `Untitled2`（Untitled1、Untitled3 分别是空壳与 TwinSAFE 占位，非当前主链路）。上位机侧文档见 `../64位ADS - 相对路径 - 传数组 - 加上手柄/项目架构总览.md`。
>
> 修改任何 PLC POU、DUT、GVL、或改变与上位机的 ADS 契约，请同步更新本文档与相关子文档，并在最末"变更日志"追加条目。

更新时间：2026-07-06
适用工程：`250902\250902.sln`（PLC 项目：`250902\Untitled2`）
对应代码版本：2026-07-06 主分支（基于 `MAIN.TcPOU` / `SelfCheck.TcPOU` / `handle.TcPOU` / `init.TcPOU` / `reset.TcPOU` / `err.TcPOU` / `clear_err.TcPOU` / `G.TcGVL` / `state.TcDUT` / `ST_AxisPlannedReturnCmd.TcDUT` 当前状态校对）

***

## 1. 项目简介

血管介入手术机器人 PLC 侧程序。承担：

- 7 个 EtherCAT 伺服轴的**使能、寻参自检、错误处理、外部设定点输出**；
- 5 个电缸（含 Y 阀电缸）与 2 组力传感器 IO 的**总线 I/O 映射**；
- 与上位机（`64位ADS - 相对路径 - 传数组 - 加上手柄\ADS.exe`）之间通过 **ADS 通讯**交换 refer/actual 位置、电缸控制量、握手位、力采样电压；
- 顶层运行状态由**主状态机** `G.gen_state`（`state.TcDUT`）调度：上电 → 自检 → 正常跟随 → 故障/复位。

入口 POU：`Untitled2\POUs\MAIN.TcPOU`。任务周期由 `PlcTask.TcTTO` 决定（1 ms 循环）。

PLC 侧**不直接决定运动模式**（导管/导丝/协同、启动准备阶段、爬行状态机等全部在上位机）；PLC 只负责：
1. 拿到 `G.refer[]` 后按速度限幅 + 滑动平均 + 二阶滤波形成实际设定点；
2. 通过 `MC_ExtSetPointGenFeed` 下发给 NC；
3. 处理**计划回退**（`ST_AxisPlannedReturnCmd`）作为快退阶段的专用通道；
4. 处理 axis4 手动点动；
5. 处理错误/急停保持并通过握手位通知上位机。

## 2. 术语对照表

与上位机文档 `项目架构总览.md §1.5` 保持一致。以下补充 PLC 侧独有术语：

### 2.1 顶层状态（`state.TcDUT`）

| 枚举值 | 含义 | 主链路使用 |
|---|---|---|
| `_init` | 上电初始化，7 轴 MC_Power 上电 | 是 |
| `_self_check` | 自检寻参，寻左限位并回到工作位 | 是 |
| `_handle` | 正常跟随，接受上位机 refer 输出 | 是 |
| `_clear_err` | 错误清除后过渡到 `_handle` | 是 |
| `_err` | 错误保持态 | 是 |
| `_reset` | 复位过渡 | 是 |
| `_jog1` / `_jog2` / `_jog3` | 历史遗留状态 | 否，主 CASE 未使用 |
| `_trans` / `_trans_wait` | 历史遗留状态 | 否 |

### 2.2 三层"位置"字段（PLC 内部）

| 字段 | 基准 | 表达式 | 用途 |
|---|---|---|---|
| **NC 绝对** | 机械原点 | `G.axis[i].NcToPlc.ActPos` | PLC 内部与限位判断 |
| **相对** | `G.init_pos[i]` | `G.Act_pos[i] = ActPos - init_pos[i]` | 上位机读写基线 |
| **距左限位** | `G.leftlimit[i]` | `G.act_pos_from_left[i] = ActPos - leftlimit[i]` | 监测/门控 |

`G.refer[i]` 与 `G.Act_pos[i]` 使用相对坐标；`G.leftlimit[i]` 用绝对坐标；`G.axisN_return_cmd.TargetAbs` 用绝对坐标。这与上位机文档 `§1.5.4` 的三套基准一一对应。

### 2.3 使能/回退相关握手位

| 变量 | 方向 | 语义 |
|---|---|---|
| `G.self_check_done` | PLC→上位 | 自检完成 |
| `G.handle_reinit_req` | PLC→上位（上位读后清除） | 请求上位机做一次全量重同步 |
| `G.handle_reinit_done` | PLC 内部 | handle POU 已完成重初始化 |
| `G.estop_hold_req` | PLC→上位 | PLC 处于保持态，上位机应停写 refer |
| `G.selfcheck_reset_req` | init→SelfCheck | 请求 SelfCheck 重新初始化内部状态 |
| `G.axis1_fast_return` / `G.axis6_fast_retract` | 上位→PLC | 快退阶段旁路 PLC 平滑层 |
| `G.startup_smoothing_bypass` | 上位→PLC | 启动准备阶段旁路二阶滤波 |
| `G.axisN_return_cmd.Req` | 上位→PLC | 计划回退触发 |

## 3. 硬件对象与轴映射

7 个 NC 轴全部由 `G.axis[1..7]` 承载。此外还有 1 个不属于 PLC 轴控的**物理夹持底座**，其上装有电缸1。

### 3.1 轴分工

| 轴 | 类型 | 分工 | 上位机手柄 |
|---|---|---|---|
| **轴1** | 平移 | 导管主推进 | handle_axis1（导管手柄） |
| **轴2** | 旋转 | 导管旋转 | handle_axis1 |
| **轴3** | 平移 | 与轴1 同步，其机构上装 **电缸5** 控制 Y 阀 | 跟随轴1（非独立控制） |
| **轴4** | 旋转 | 球囊推进/撤出，PLC 侧通过 `axis4_fwd_req`/`axis4_rev_req` 手动点动 | UI 按钮或手柄键 |
| **轴5** | 平移 | 类似"可平移的夹持底座"，其上装 **电缸3** 与轴6 上的电缸4 交替夹导丝 | 跟随轴1（非独立控制） |
| **轴6** | 平移 | 导丝主推进，其上装 **电缸4** | handle_axis6（导丝手柄） |
| **轴7** | 旋转 | 导丝旋转 | handle_axis6 |

### 3.2 电缸/Y 阀映射

| 电缸 | PLC 变量 | 安装位置 | 主要作用 |
|---|---|---|---|
| 电缸1 | `G.cylinder1_value` | 物理夹持底座（非轴） | 夹持导管 |
| 电缸2 | `G.cylinder2_value` | 轴1 机构 | 与电缸1 交替夹持导管，实现导管蠕动 |
| 电缸3 | `G.cylinder3_value` | 轴5 机构 | 夹持导丝 |
| 电缸4 | `G.cylinder4_value` | 轴6 机构 | 与电缸3 交替夹持导丝，实现导丝蠕动 |
| 电缸5 | `G.cylinder5_value` / `G.cylinder5_cmd` / `G.cylinder5_press_req` | 轴3 机构 | 控制 Y 阀开合 |

**cylinder1..4** 由上位机直接写 WORD 目标值；**cylinder5** 采用双层：上位机写 `G.cylinder5_press_req`（BOOL），`handle` POU 每周期把该请求映射为 `G.cylinder5_value = 0`（按下 → 夹紧）或 `2000`（释放 → 打开）。这一层封装是为了把 Y 阀操作压缩为一个语义清晰的按键请求。

### 3.3 力传感器 IO

| 变量 | 类型 | 说明 |
|---|---|---|
| `G.ft_1_value` | INT (%I*) | 第一路扭矩通道 |
| `G.fn_1_value` | INT (%I*) | 第一路轴向力通道 |
| `G.ft_2_value` | INT (%I*) | 第二路扭矩通道 |
| `G.fn_2_value` | INT (%I*) | 第二路轴向力通道 |

对应倍福 TwinCAT 拓扑中的 `nDataIn2[0..1]` 等 EtherCAT 输入。PLC 侧仅做输入映射，不解算/滤波；上位机通过 ADS 备份读，主路径走 TCP 采集卡。见 `../64位ADS - 相对路径 - 传数组 - 加上手柄/力反馈说明.md`。

## 4. 目录结构

```
250902/                                # PLC 项目根，与上位机 .sln 平级
├── 250902.sln                         # 解决方案文件
├── 250902/                            # TwinCAT 系统项目
│   ├── 250902.tsproj                  # 系统配置（NC 任务、I/O 映射）
│   ├── Untitled1/                     # 空 PLC 项目（占位，不承载逻辑）
│   ├── Untitled2/                     # ★ 活跃 PLC 项目
│   │   ├── Untitled2.plcproj
│   │   ├── PlcTask.TcTTO              # 1ms 任务定义
│   │   ├── GVLs/
│   │   │   └── G.TcGVL                # 全局变量（详见 PLC全局变量与ADS接口.md）
│   │   ├── DUTs/
│   │   │   ├── state.TcDUT            # 顶层状态枚举
│   │   │   └── ST_AxisPlannedReturnCmd.TcDUT  # 计划回退命令结构
│   │   └── POUs/
│   │       ├── MAIN.TcPOU             # 主程序（状态调度 + Actions）
│   │       ├── init.TcPOU             # 上电初始化
│   │       ├── SelfCheck.TcPOU        # 自检寻参
│   │       ├── handle.TcPOU           # ★ 正常跟随控制
│   │       ├── handle(备份).TcPOU     # 历史备份，不参与编译
│   │       ├── err.TcPOU              # 错误保持态
│   │       ├── reset.TcPOU            # 复位过渡
│   │       ├── clear_err.TcPOU        # 错误清除
│   │       ├── move_edge.TcPOU        # 保留，未在 MAIN 中调用
│   │       └── 下一步工作.txt          # 历史备忘
│   └── Untitled3/                     # TwinSAFE 项目（安全逻辑，独立配置）
├── TwinCAT Measurement Project*/      # 波形示教工程（诊断用）
├── PLC项目说明.md                     # ★ 本文档
├── PLC状态机与流程说明.md              # 状态机与各 POU 详解
├── PLC全局变量与ADS接口.md             # GVL 与 ADS 契约
├── Project_Documentation.txt          # 历史文档（保留但已过时）
└── Project_Code_Analysis_Report.txt   # 历史文档（保留但已过时）
```

`handle(备份).TcPOU`、`move_edge.TcPOU`、`Project_Documentation.txt`、`Project_Code_Analysis_Report.txt`、`report.txt` 均为**历史遗留**：与当前主链路可能矛盾，仅作演进参考。以本文档与 `PLC状态机与流程说明.md` / `PLC全局变量与ADS接口.md` 为准。

## 5. 与上位机的分工契约

以"控制指令"为线索梳理 PLC 与上位机各自的责任边界：

| 决策项 | 决策方 | 落地字段 |
|---|---|---|
| 手柄输入采样、滤波、按键解码 | 上位机 | — |
| 模式判定（导管/导丝独立/协同） | 上位机 | — |
| 爬行状态机、窗口重建、快退触发 | 上位机 | `G.axisN_return_cmd.*` + `G.axis1_fast_return` / `G.axis6_fast_retract` |
| 启动准备序列 | 上位机 | `G.startup_smoothing_bypass` + `G.cylinderN_value` + `G.v_limit` |
| 力反馈标定、映射、下发手柄 | 上位机 | — |
| 目标位置计算（相对坐标） | 上位机 | `G.refer[7]` |
| 轴使能（MC_Power） | PLC | 内部 |
| 寻参自检（左限位） | PLC | 记录 `G.leftlimit[]`、置 `G.self_check_done` |
| refer → 设定点的**滑动平均 + 速度限幅 + 二阶滤波** | PLC | `handle` POU 内部 |
| 计划回退运动执行（MC_MoveAbsolute） | PLC | `handle` POU 内 `fb_return_move_abs` |
| axis4 手动点动执行 | PLC | `handle` POU 内 `fb_axis4_move_velocity` |
| 软限位（右侧） | PLC | `slimit_enable` + `rslimit_distance` |
| 错误保持 & 恢复策略 | PLC | `_err` / `_reset` / `_clear_err` |

### 5.1 控制链路时序

```
上位机主循环 (~1kHz)                  PLC 任务 (1ms)
──────────────────────                ────────────────
读手柄 + 生成 refer[7]     ── ADS ─►  G.refer[7]
                                       ↓
                                       handle POU:
                                        ├─ 速度限幅   → G.ref_slow
                                        ├─ 滑动平均   → G.act_h
                                        ├─ 二阶滤波   → traj_pos (若使能)
                                        └─ 输出       → MC_ExtSetPointGenFeed
                                       ↓
                                       NC → EtherCAT → 伺服
                                       ↓
G.Act_pos ◄── ADS ───────────────      更新 Act_pos / act_pos_from_left
```

### 5.2 快退（计划回退）时序

```
上位机检测到爬行触发                    PLC handle POU 内的 return_state[i]
──────────────────────                ─────────────────────────────────
axis1_fast_return := TRUE
G.axisN_return_cmd.TargetAbs/*
G.axisN_return_cmd.Req := TRUE   ─►    return_state = 10 → 20 → 30
                                        ├─ 10: 禁用 SetPointGen
                                        ├─ 20: 触发 MC_MoveAbsolute
                                        └─ 30: 完成后重建平滑基准 +
                                               重新使能 SetPointGen
                                       ↓
                                       G.axisN_return_cmd.Done := TRUE
                                                                    ─► 上位机检测到 Done
                                                                       → 清 Req
                                                                       → 恢复 refer
                                                                       → 单轴 sync
```

计划回退阶段 `G.axis1_fast_return` / `G.axis6_fast_retract` 为 TRUE 时，PLC 的 `handle` POU 走 `return_handoff_bypass` 分支：`avg_window := 1`、`ramp_counter := 0`、跳过二阶滤波，让 refer 直接透传给设定点发生器（末端仍受 `MC_MoveAbsolute` 主导，refer 只是防止基准漂移）。

## 6. 顶层状态机导航图

```
                            ┌───────────┐  首周期 / PLC 上电
                            │  _init    │◄──────────────────┐
                            └─────┬─────┘                    │
                                  │ 上电成功                  │
                                  ▼                          │
                            ┌───────────┐                    │
                            │_self_check│                    │
                            └─────┬─────┘                    │
                                  │ 全轴 stu=10              │
                                  ▼                          │
              ┌───────────────►┌───────────┐                 │
              │                │  _handle  │                 │
              │                └─┬───────┬─┘                 │
              │  轴错误          │       │  clear_err 完成    │
              │                  ▼       ▲                   │
              │              ┌───────┐   │                   │
              │              │ _err  │───┘                   │
              │              └───┬───┘                       │
              │                  │ 错误已清除                 │
              │                  ▼                           │
              │              ┌────────────┐                  │
              │              │ _clear_err │──────────────────┘
              │              └────────────┘  非首启或未自检时
              │                              → 回 _init
              │
              └─── _reset（未在 MAIN 主 CASE 中主动跳入，供外部调用）
```

详细跳转条件、每态动作、参数值见 `PLC状态机与流程说明.md`。

## 7. 关键参数速查

| 参数 | 值 | 位置 | 含义 |
|---|---|---|---|
| PLC 任务周期 | 1 ms | `PlcTask.TcTTO` | 主循环节拍 |
| 自检逼近速度 | `vel_scan = 30.0` | `SelfCheck.TcPOU` | 向左限位低速逼近速度 |
| 自检回位速度 | `vel_back = 50.0` | `SelfCheck.TcPOU` | 记录左限位后向工作位回退速度 |
| 自检回位目标（距左限位） | `[96, 0, 280, 0, 430, 580, 0]` | `SelfCheck.TcPOU::init_target_from_left` | 轴1/3/5/6 工作位 |
| 目标轴（平移轴）掩码 | `[T, F, T, F, T, T, F]` | `SelfCheck.TcPOU::target_axes` | 参与自检寻参的轴 |
| 错峰启动延迟 | `[0, 0, 300, 0, 600, 900, 0] ms` | `SelfCheck.TcPOU::scan_stagger_delay` | 各轴逼近启动错峰 |
| 错峰回位延迟 | `[3, 0, 2, 0, 1, 0, 0] s` | `SelfCheck.TcPOU::stagger_delay` | 各轴回位启动错峰 |
| 速度上限 | `v_limit = [7.5, 7.5, 1.5, 1.5, 1.5, 7.5, 1.5]` | `G.TcGVL` | 每拍 refer 增量上限（可被上位机改） |
| 右软限位距左限位 | `[99, 0, 666, 0, 688, 688, 0]` | `handle.TcPOU::rslimit_distance` | 参与软限位的距离 |
| 软限位使能 | `[T, F, T, F, T, T, F]` | `handle.TcPOU::slimit_enable` | 仅 4 个平移轴启用 |
| 滑动平均窗口（正常） | 20 | `handle.TcPOU::avg_window_follow` | 平滑窗口长度 |
| 二阶滤波频率 | `traj_wn = 60.0 rad/s` | `handle.TcPOU` | 二阶带宽 |
| 二阶滤波使能 | `[T, F, T, F, T, T, F]` | `handle.TcPOU::traj_enable` | 仅 4 个平移轴启用 |
| axis4 手动速度 | `30 deg/s` | `handle.TcPOU::axis4_manual_velocity` | 球囊推进速度 |

## 8. 文档索引

| 文档 | 覆盖范围 | 何时查阅 |
|---|---|---|
| **PLC项目说明.md**（本文档） | 顶层入口、术语、目录、硬件映射、状态机总览、契约 | 第一次接手 PLC、新增模块 |
| **PLC状态机与流程说明.md** | 各 POU 详解、状态跳转矩阵、SelfCheck 步骤、handle 三段式、错误处理链 | 改 POU、调参、排查跳转异常 |
| **PLC全局变量与ADS接口.md** | GVL 完整表、ADS 符号契约（对偶上位机 `ADS通讯说明.md`）、握手协议 | 新增/删除 PLC 符号、调 ADS |

对应上位机侧：

| 上位机文档 | 位置 |
|---|---|
| 项目架构总览 | `../64位ADS - 相对路径 - 传数组 - 加上手柄/项目架构总览.md` |
| ADS 通讯说明 | `../64位ADS - 相对路径 - 传数组 - 加上手柄/ADS通讯说明.md` |
| 运动流程说明 | `../64位ADS - 相对路径 - 传数组 - 加上手柄/运动流程说明.md` |
| 力反馈说明 | `../64位ADS - 相对路径 - 传数组 - 加上手柄/力反馈说明.md` |
| 可视化界面架构说明 | `../64位ADS - 相对路径 - 传数组 - 加上手柄/可视化界面架构说明.md` |

## 9. 维护总约定

任何修改 PLC 侧代码或 ADS 契约的工作，**必须**：

1. 先查阅对应子文档的"维护要求"小节，按其清单更新文档。
2. 变更日志必填字段：日期 / 变更人（AI 模型版本或工程师 initials）/ 涉及文件与 POU / 行为变化要点 / 是否影响与上位机的契约。
3. **跨侧改动**（新增/改语义/改类型的 GVL 变量、改 return_cmd FB、改握手位）：在本文档 §5 契约表 + `PLC全局变量与ADS接口.md` 符号表 + 上位机 `ADS通讯说明.md §3` 三处同步。
4. **删除/重命名 GVL 变量**：全工程 grep 确认无引用（PLC 侧 `Untitled2/**/*.TcPOU`、上位机 `plc_io.cpp` 与 `main.cpp`）。
5. **新增 POU 或改状态枚举**：`state.TcDUT` + `PLC状态机与流程说明.md §2` 状态跳转矩阵同步。

## 10. 变更日志

### 2026-07-06 — 文档初版（无代码变更）
- 作者：AI（Claude，应用户要求梳理）。
- 内容：基于现网代码（`MAIN.TcPOU` / `SelfCheck.TcPOU` / `handle.TcPOU` / `init.TcPOU` / `reset.TcPOU` / `err.TcPOU` / `clear_err.TcPOU` / `G.TcGVL` / `state.TcDUT` / `ST_AxisPlannedReturnCmd.TcDUT`）编写第一版说明。
- 明确了 7 轴 + 5 电缸的物理分工（用户澄清后）：物理夹持底座（含电缸1）不受 PLC 轴控；电缸5 通过 `cylinder5_press_req` 二值化控制 Y 阀；axis4 球囊推进由上位机通过 `axis4_fwd_req/rev_req` 手动点动。
- 建立与上位机文档的锚点交叉引用体系。
- 与历史文档（`Project_Documentation.txt` 2026-03-23 版）的主要差异：
  - `init_target_from_left` 更新为 `[96,0,280,0,430,580,0]`（历史文档为 `[96,0,280,0,530,685,0]`）；
  - handle POU 已引入二阶滤波（`traj_wn=60`、`traj_enable`），旧文档只描述滑动平均；
  - 新增了 axis4 手动控制 FB 与 `ST_AxisPlannedReturnCmd` 结构；
  - `SelfCheck` 中错峰逻辑改为 `scan_stagger_delay` + `stagger_delay` 两级；
  - 完成后自检不再直接切 handle，而是通过 `handle_reinit_req` 与上位机握手。
- 未触碰任何源文件。

### （此处持续追加）
