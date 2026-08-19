# C++ 上位机项目代码说明

更新时间：2026-08-03
适用工程：`ADS.vcxproj` + `AdsControlUI/AdsControlUI.csproj`

本文是当前代码导读。运动细节见 [运动流程说明.md](运动流程说明.md)，力链路见 [力反馈说明.md](力反馈说明.md)，记录和相机见 [实验记录与相机说明.md](实验记录与相机说明.md)。

## 1. 两个进程

| 进程 | 技术 | 职责 |
|---|---|---|
| `ADS.exe` | C++17 / x64 | ADS、运动状态机、手柄、力采样/反馈、PI、实验记录、相机、命名管道服务 |
| `AdsControlUI.exe` | WPF / .NET Framework 4.7.2 / x64 | 操作界面、状态显示、参数输入和实时弹窗 |

WPF 不直接连接 PLC、DAQ 或相机。两个进程只通过本地命名管道 `\\.\pipe\ADS_Control_Vis` 通信；相机像素另走本地共享内存。

## 2. C++ 模块

| 文件 | 责任 |
|---|---|
| `main.cpp` | 初始化、控制主循环、跨模块编排和 UI 命令分发 |
| `control_types.h/.cpp` | 公共控制状态、配置和小型工具函数 |
| `ADSComm.cpp` + `ADS/Include/*` | TwinCAT ADS 底层：单项读写、按句柄 Sum Read/Sum Write、Notifications、超时/状态与句柄缓存 |
| `ads_communication.h/.cpp` | 100 Hz ADS 工作线程、统一快照/输出、运行期专项请求队列、QPC 中点、软保持、自动重连、PLC 重启识别、主机看门狗与诊断 |
| `plc_io.h/.cpp` | ADS 符号表和强类型专项接口；正常模式下从 `ads_service` 复制位置/力快照，并把运行期低频请求交给通信线程 |
| `motion_sync.*` | 坐标换算、窗口、基线和重同步 |
| `guidewire_mode.*` | 导丝独立/协同模式进入退出 |
| `startup_sequence.*` | 标准装卸和中断恢复启动准备 |
| `delivery_tracking.*` | 换手欠账与受限单向 PI；没有磁盘职责 |
| `tcp_force_daq.*` | 可选 TCP 力 DAQ worker，只发布最新帧 |
| `force_calibration.h` | 纯净力公式和反馈标定链 |
| `force_feedback.*` | 力反馈输出门控、快退锁存和下发值 |
| `force_transition_experiment.*` | 力过渡实验状态机；可无日志运行 |
| `async_csv_writer.h` | 通用 SPSC 后台 CSV writer |
| `experiment_recorder.*` | 会话目录、QPC 调度、CSV 和元数据总控 |
| `action4_camera.*` | Media Foundation 探测、预览、MP4 和帧旁车 |
| `vis_server.*` | 499 字节状态快照（含 ADS 诊断）和版本化命令管道 |

三个旧独立日志模块已经删除。不要重新在启动、PI 或 TCP 回调里创建独立日志。

## 3. 启动流程

正常 `main()` 依次完成：

1. 设置 UTF-8 控制台并处理传感器重新标定/手柄监视等独立工具模式；
2. 创建 `ExperimentRecorder`，其相机 worker 和预览共享内存随对象初始化；
3. 打开一只或两只 FLCatheter 手柄并建立逻辑角色映射；
4. 连接 ADS，本地路由失败时按现有配置尝试外部路由，读取应用名；PLC 必须已经处于 `ADSSTATE_RUN`；
5. 启动 `VisServer`，读取初始坐标/启动状态，并按配置启动可选 TCP DAQ（ADS 仍是默认力源）；
6. 启动 `AdsCommunicationService`，注册 OnChange Notifications、解析快速符号、建立坐标缓存并等待首份 100 Hz 快照；
7. 进入由 ADS 快照唤醒的控制主循环。

连接和重连只检查 PLC 状态，不自动切换 RUN，也不调用 TwinCAT Activate Configuration。本轮没有 I/O、NC 或 PDO 映射变更。

进程启动、C/S 控制启动、力反馈开启和 PI 开启都不会创建记录文件。只有命令 25 能开始完整会话。

## 4. 主循环

主循环以 100 Hz ADS 快照为主要节拍：`wait_for_snapshot` 最多等待 20 ms，随后消费最新快照、队列和 Notification 状态。主要顺序：

```text
等待 AdsFastSnapshot / 排空记录队列
  -> ADS 状态、软保持、重连或 PLC 重启恢复边界
  -> 手柄 poll/滤波与按键边沿
  -> 暂停/急停/主机看门狗/自检/启动准备
  -> 模式、爬行/回退、PI 和 refer/cylinder 计算
  -> 力采样、纯净力和实验状态
  -> 发布 AdsOutputCommand，由通信线程下一拍 Sum Write
  -> force.csv 逐 100 Hz 快照入队
  -> motion.csv 取偶数快照序号形成 50 Hz
  -> 力过渡状态机/专用表入队
  -> 力反馈处理和手柄力输出
  -> 30 Hz VisState（含 ADS 诊断）推送与 VisCommand 消费
```

`read_plc_state` / `read_force_sample` 在正常模式只复制通信服务的最新有效快照，不做额外高频 ADS 请求；refer、快退、气缸、axis4 和平滑旁路先组成一份输出快照。计划回退、启动准备等运行期专项 ADS 操作也提交给通信服务串行执行，主循环不与 100 Hz worker 并发访问同一个 ADS 端口。任何主循环日志路径都只能调用 `try_enqueue`，目录、`fwrite`、flush、MP4 编码和 Finalize 由后台线程完成。

## 5. ADS 数据模型

`ADSComm` 提供线程安全的同步单项读写、按句柄 Sum Read/Sum Write、Notification 注册/注销和 symbol handle 缓存。业务代码新增 PLC 变量时，应先在 `plc_io.cpp::AdsSymbol` 声明或在通信服务的集中符号表中明确列出，不在主循环散落字符串。

### 5.1 100 Hz 快速快照

`AdsCommunicationService` 用 QPC 绝对截止时间每 10 ms 发起一次快速 Sum Read。整次调用前后 QPC 的中点写入 `AdsFastSnapshot::qpc_ticks`。同包包含：

```text
TaskInfo[1].CycleCount（首尾各一次）
TaskInfo[1].DcTaskTime
G.axis[1..7].NcToPlc.ActPos（首选）或 G.Act_pos[7]（整组回退）
G.ft_1_value / G.fn_1_value / G.fn_2_value / G.ft_2_value
G.estop_hold_req / G.host_comm_timeout
```

`init_pos` / `leftlimit` 在连接初始化时缓存，from-left 在上位机计算。位置只通过明确的 TwinCAT ADS 符号获取，不猜测 EtherCAT PDO 地址；本轮不修改 PDO/NC/I/O 映射，也不要求 Activate Configuration。

力默认读取原始 `INT16` 并按 `/1000.0` 转为伏特。`attempt_sequence` 是 C++ 主机尝试序号，PLC 不维护记录环形缓冲或 `write_sequence`。

### 5.2 输出、Sum Write 与 Notifications

同一通信线程在快速读取后执行一次 Sum Write：

- 每周期必写 `G.host_session_id` 和递增的 `G.host_heartbeat_sequence`；
- 运动有效时追加 `G.refer[7]`、`axis1_fast_return`、`axis6_fast_retract`；
- 气缸、axis4 请求和启动平滑旁路只在输出变化时追加；
- 看门狗恢复时追加一次 `G.host_recover_req=TRUE`。

自检、重初始化、急停/主机超时、启动资格、axis4 状态和 `gen_state` 使用 `ADSTRANS_SERVERONCHA` Notifications。注册后先 Sum Read 一次初值；ADS DLL 回调只复制数据，不从回调线程发起任何 ADS 请求。

计划回退仍是专项握手：主循环提交有界请求，通信线程先清 Req、一次 Sum Write 写参数，再在下一事务置 Req；Busy/Done/Error/ErrorId 也由通信线程一次 Sum Read 获取。

### 5.3 失败与恢复边界

- 第一个失败周期立即进入 `SoftHold`：保持最后参考、清力反馈、丢弃故障期间手柄增量；
- 距最后完整成功达到 100 ms 后关闭端口，并按 `250/500/1000/2000 ms` 退避重连；
- 同一 PLC 运行实例的链路重连会刷新坐标并重建位置/手柄基准，清除计划回退、屈曲恢复、力过渡和 PI 瞬态，但保留稳定模式和方向；
- 设备非 RUN、PLC DcTaskTime/CycleCount 回退或应用名变化按 PLC 重启/应用重载处理：清力零点、关闭力反馈和 PI、退出控制，必须重新自检、调零和人工启动；
- PLC 100 ms 主机看门狗通过 `host_session_id/host_heartbeat_sequence/host_recover_req/host_comm_timeout` 锁存超时、冻结参考并受控停止；心跳恢复后仍必须完成显式恢复和重初始化握手，不能直接续跑。

## 6. 两条力语义

### 6.1 纯净力

`calculate_clean_force` 只做装机零点扣除和本次 `F_direct` 映射：

```text
F = 0.614437208097 * (Fn_V - Fn0_V)
T = 0.703683250522 * (Ft_V - Ft0_V) * 3.0 * 0.001
```

其中 `Fn_V/Ft_V = raw/1000.0`，因此分别等价于传感器2、传感器1的 `p × (raw-raw_zero)`；`F_direct` 固定截距在动态调零中抵消。输出单位分别为 N 和 N·m。

它供 `force.csv`、力过渡专用表和“纯净力感”窗口使用，不受反馈开关或模式影响。

### 6.2 反馈力

`calibrate_force` / `process_force_feedback` 是实际手柄输出链，包含可选重力补偿、反馈比例、死区、限幅、控制/暂停门控和快退锁存。`ForceRealtimeWindow` 显示“实际下发值”和“反馈目标值”。

两条链不能混用。验证纯净值时切换重力补偿、快退、PI、模式或力反馈开关，纯净值应保持不变。

## 7. 主从位移 PI

`DeliveryTrackingController` 只维护换手欠账、控制条件和受限增益。当前契约：

- 控制器随程序运行，不依赖磁盘会话；
- PI 开启不创建文件；
- 停止统一记录不改变 PI 开关或积分状态；
- ADS 默认力源始终随 100 Hz 快速快照更新；仅使用 TCP 回退源且只有 PI 是消费者时，主循环将力健康消费节流到约 20 Hz；
- 力失效或原有安全条件失效时，PI 按自身规则关闭。
- axis1/axis6 默认均为 `Kp=0.5`、`Ki=2`、最大增益 `2`、欠账上界 `20mm`；`Ki` UI 范围为 `0-100`，最大增益上限为 `5`。

统一 `motion.csv` 只记录 7 轴 from-left 和两只手柄原始/滤波值，不再记录旧主从实验的大量中间列。

## 8. 统一实验记录

`ExperimentRecorder` 状态：

```text
Idle -> Starting -> Recording -> Stopping -> Idle
                  \-> Error（启动失败）
```

开始流程同步创建目录和两个主 CSV 表头；相机随后异步打开。相机缺失或失败不回滚会话。

停止流程使用独立 `stop_thread_`：先停止/封装视频，再排空专用表和两个主表，最后覆盖最终 `session.json`。程序退出调用同一路径并 join。

ADS 记录由 100 Hz 主机快照序号驱动：`force.csv` 每个尝试序号最多一行，`motion.csv` 取偶数序号形成 50 Hz。无效快照保留时间槽并写有效位/`NaN`；会话开始前帧、重复序号、序号空洞和主机快照队列丢帧均显式统计，不突发补写伪历史。

`session.json` 除目标采样率外，还写 ADS 连接状态、实际 Hz、数据龄、RTT、失败/连续失败、重连、PLC 重启，以及 force/motion/力过渡各自的首末序号、接受/无效/拒绝/空洞计数。

详细格式和错误语义见 [实验记录与相机说明.md](实验记录与相机说明.md)。

## 9. 力过渡实验

`ForceTransitionExperiment` 继续独立管理 axis1 实验状态机。磁盘规则只有一条：开始力过渡的那个瞬间若完整会话正在 Recording，调用 `start_force_transition_log()` 在当前目录创建顺序文件；否则实验无日志运行。

停止完整会话只让 `enqueue_force_transition` 停止接受数据，不调用状态机 abort。用户点击“停止实验”或原有安全条件失败才终止状态机。

## 10. Action 4

`Action4CameraRecorder` 独占 Media Foundation 视频源：

- VID/PID `2CA3:0023` 优先，名称 `OsmoAction4` 备用；
- 1080p/30 H.264、MJPEG，再到 720p/30；
- 无音频 H.264 MP4；
- 1280x720 BGRA 双缓冲预览；
- 每帧写 `video_frames.csv` 映射 QPC；
- 拔线后封装已有文件，当前会话不重连。

当前优先验证原生 H.264，但录制管线仍会解码 RGB32 后重编码。该限制及实机待验项目写在 [实验记录与相机说明.md](实验记录与相机说明.md)。

## 11. 本地可视化协议

### 11.1 状态

`VisState` 是 pack(1) 的 883 字节二进制结构。C++ 有 `static_assert`，C# 启动时检查固定 wire size。除运动、力反馈、PI、记录、相机和物理按钮字段外，末尾还包含 ADS 状态、定位臂五轴的上电/复位/点动状态和参数、axis4 手动点动状态，以及力反馈-保持状态。

WPF 顶部把状态翻译为“ADS 正常 / 单拍软保持 / 重连中 / PLC 已重启”等文本，同时显示实时量和累计计数。只有本地管道连接、ADS 状态为 Running 且 PLC 主机看门狗未超时时才显示健康。

### 11.2 命令

命令头固定 24 字节：magic、version、header size、type、两个 int 参数和 payload size。payload 最多 1024 字节；实验名称使用 UTF-8。

记录命令值：

```text
25 StartExperimentRecording
26 StopExperimentRecording
27 SetCameraPreview
28 SetCleanForceMonitor
29 SetArmManualEnable
30 SetArmAxisEnable
31 RequestArmAxisReset
32 SetArmAxisJog
33 SetArmJogParameter
34 SetAxis4ManualJog
```

7 和 20 是删除旧日志功能留下的空洞，禁止重新编号或复用。协议细节见 [可视化界面架构说明.md](可视化界面架构说明.md)。

## 12. WPF 数据流

```text
VisPipeClient 后台读线程
  -> 最新 VisState
  -> AdsControlViewModel（约 33 ms 刷新绑定）
  -> MainWindow
  -> 已打开的实时弹窗 AddState/OnState
```

`MainWindow.xaml.cs` 直接调用 ViewModel 方法发送命令，没有 RelayCommand。ScottPlot 仅供已有力过渡窗口；纯净力和反馈实时窗口沿用 WPF 图元。

## 13. 构建与协议配对

推荐构建：

```powershell
MSBuild.exe ADS.vcxproj /m /p:Configuration=Debug /p:Platform=x64
MSBuild.exe AdsControlUI\AdsControlUI.csproj /m /p:Configuration=Debug /p:Platform=x64
```

Release 同理。C++ 和 WPF 协议必须成对替换；499 字节版不能与旧 UI/C++ 混用。

## 14. 扩展检查

1. 不修改 PLC/ADS 契约，除非任务明确要求并同步 PLC 文档。
2. 控制主循环不做磁盘、编码或窗口线程等待。
3. 新 CSV row 复用 `AsyncCsvWriter`，写失败必须有可见状态。
4. 新 `VisState` 字段只末尾追加并同步 wire size。
5. 新命令使用新枚举值；文本走 UTF-8 payload。
6. 自动化或实验退出不隐式改变无关的 PI、力反馈或运动模式。
7. 修改相机后同时测试 missing、拔线、Finalize 和 15 分钟压力场景。
