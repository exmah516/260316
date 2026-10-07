# 轴 6 回退结束时短暂急停：复核记录

日期：2026-10-07。基于工作区 HEAD `999555f` 及现有未提交内容进行只读审计；本次未修改运动参数、PLC 控制逻辑或现有工作区改动。

## 当前结论

尚未确定首次触发源。不能继续把运动参数、正常完成清 Req、完成回调阻塞或重复调用 SetPointGen 当成已经证实的根因。

用户提供的“同参数此前正常”是版本回归排查依据；目前还缺少最后正常版本和现场实际下载的 PLC 工程，不能把任一历史提交自行认定为正常基线。

## 已核实和排除的推断

1. 参数对比：`39cd4be`、`2ad6f64`、`999555f` 的轴 6 回退速度、加速度、减速度、加加速度均为 `400 / 4800 / 4800 / 70000`。没有参数改变的证据。

2. 正常完成清 Req 不会直接触发取消状态 40：上位机要求本次 Done 新事件且 Busy 为 FALSE，再等待更新的位置快照，最后把清 Req 和交接输出一并发布。包含状态 40 的 PLC 版本在置 Done 的同一扫描已把 return_state 置为 0。因此“正常清 Req 被当成取消”的先前推断不成立。

   - [本次完成事件判定](<D:/Work_files/Vessel intervention Robot/260316/64位ADS - 相对路径 - 传数组 - 加上手柄/main.cpp:2375>)
   - [等待新快照](<D:/Work_files/Vessel intervention Robot/260316/64位ADS - 相对路径 - 传数组 - 加上手柄/main.cpp:3002>)
   - [发布清 Req](<D:/Work_files/Vessel intervention Robot/260316/64位ADS - 相对路径 - 传数组 - 加上手柄/main.cpp:4518>)
   - [PLC 完成分支](<D:/Work_files/Vessel intervention Robot/260316/250902/250902/Untitled2/POUs/handle.TcPOU:667>)

3. 完成位置重建走的是 `rebase_axis6_after_return()`，其实现重建本地快照、窗口和滤波状态，没有同步 ADS 读取或 Sleep。旧摘要中以 `sync_axis6()` 解释当前正常收尾路径不准确。未发现回退完成特有的阻塞调用证据。

   - [当前位置重建实现](<D:/Work_files/Vessel intervention Robot/260316/64位ADS - 相对路径 - 传数组 - 加上手柄/motion_sync.cpp:247>)

4. MAIN 与 handle 同周期调用 SetPointGen 的结构，在两套 PLC 工程的 `39cd4be` 中已存在，在 `d0c480d`、`2ad6f64`、`999555f` 中仍存在。它可以作为待验证结构风险，但不是已定位的新回归。MAIN 的 Position 和 PositionType 输入引用实例自身字段，并非固定写入 0。

5. 上位机显示“PLC 急停状态：开启”只表示订阅的 `G.estop_hold_req` 发生变化，消息不包含触发轴或故障类型，不能据此认定碰到了限位。

   - [提示输出位置](<D:/Work_files/Vessel intervention Robot/260316/64位ADS - 相对路径 - 传数组 - 加上手柄/main.cpp:1722>)

## 一秒解除的证据边界

[main.cpp 的恢复请求](<D:/Work_files/Vessel intervention Robot/260316/64位ADS - 相对路径 - 传数组 - 加上手柄/main.cpp:1743>)在急停期间每隔 1000 ms 调用 `request_watchdog_recovery()`。这是恢复时间特征的一个解释，但不是固定延时：首次请求可能立即发送，实际解除还取决于 PLC 分支和故障恢复条件。

所以，“约一秒后解除”不能单独证明最初由看门狗触发。轴错误的自动复位流程同样需要结合第一次故障时的状态确认。

## 必须区分的两套源码

| 工程 | 当前 handle 看门狗 | 通信超时分支直接置急停 |
|---|---|---|
| `D:\Work_files\Vessel intervention Robot\260316\下位机工程程序\250902\250902.tsproj` | 2 s | 否；正常已初始化 handle 的故障入口是上电状态或轴/功能块错误 |
| `D:\Work_files\Vessel intervention Robot\260316\250902\250902\250902.tsproj` | 100 ms | 是 |

依据：

- [第一套看门狗](<D:/Work_files/Vessel intervention Robot/260316/下位机工程程序/250902/Untitled2/POUs/handle.TcPOU:113>)、[故障入口](<D:/Work_files/Vessel intervention Robot/260316/下位机工程程序/250902/Untitled2/POUs/handle.TcPOU:216>)。
- [第二套看门狗](<D:/Work_files/Vessel intervention Robot/260316/250902/250902/Untitled2/POUs/handle.TcPOU:145>)、[通信超时入口](<D:/Work_files/Vessel intervention Robot/260316/250902/250902/Untitled2/POUs/handle.TcPOU:326>)。

目录标签和通信改造实施指南不能证明现场下载的是哪一套代码。当前本机未观察到可确认下载工程的 TwinCAT 编辑器窗口。

## 首次故障取证要求

项目内两份 `Device Log messages.log` 均为空，没有能够重放本次症状的现场记录。本轮未连接或操作机器人，也未获得实际复现信号；静态检查不能作为现场修复通过的证明。

确认现场工程后，应在第一次急停置位之前锁存原因，保留到手动读取结束，避免自动复位抹掉首个错误。记录对象至少包括：

- 触发分支、PLC 周期号、`gen_state`、`host_comm_timeout`。
- 全部业务轴 1..7 的 `power_output.Done/Error/ErrorID`、`axis.Status.Error/ErrorID/DriveDeviceError`、`reset_output.Error/ErrorID`，不能只记录轴 6。
- 轴 6 的 `return_state`、Req/Busy/Done/Error/ErrorId、`fb_return_move_abs` 的 Done/Busy/Error/CommandAborted/ErrorID。
- 轴 6 SetPointGen Enable/Disable 实例本身的 Execute/Done/Busy/Error/ErrorID/Enabled，以及 ActPos、ActVelo、参考位置。

触发前后再保留连续 PLC 周期的数据，以确定是先出现轴故障、上电状态失效还是控制权交接异常。第一次原因必须在 `err` 自动复位、`handle_reinit_req` 初始化或参考位置重建之前采集。暂不通过减速、延迟清 Req 或重构功能块调用去掩盖尚未识别的触发源。
