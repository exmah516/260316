# 导管轴1前进暂停（2026-10-09）

导管模式勾选“启用前进暂停”，填写从每段准备起点前进的距离和停留秒数，再执行原有准备、取零、开始流程。默认关闭，参数初值为10 mm、3 s。距离须在 `(0, 准备位置－触发位置]` 内；时间为0.001～60 s，精度1 ms。

各周期及最终前向段经过该距离时暂停一次。最终段较短时直接完成；恰好到暂停点时停留后完成。停留从定位功能块报告Done后开始，不释放夹爪、不停止记录。阶段编号仍为4或9，`wait_action=3` 表示停留。

## 源码与接口

- PLC：`260316/250902/250902/Untitled2/POUs/ProgrammedDeliveryExperiment.TcPOU` 与同工程 `GVLs/G.TcGVL`。实际入口为MAIN调用的独立功能块；本次不向handle添加第二套控制。
- `PROGRAM_PREPARE` 新增 `forward_pause_enabled=0/1`、`forward_pause_distance_mm`、`forward_pause_duration_ms`。旧命令缺省关闭；非导管模式不得启用。
- ADS对应 `G.program_test_forward_pause_enabled`（BOOL）、`G.program_test_forward_pause_distance_mm`（LREAL）、`G.program_test_forward_pause_duration_ms`（UDINT）。所有配置写入成功后单独发送准备请求；缺失符号或批量写入失败不会发送该请求。
- PLC准备时锁存暂停参数；非法参数报 `16#7208`。每段先定位到暂停点，再使用TON计时，最后以原运动参数定位到终点。中止、错误及重新准备清除等待状态。
- `experiment.json` 增加同名三个配置字段；CSV列和阶段编号不变。界面与后端握手版本更新为 `20261009.1`，防止新界面误连旧后端。

## 离线验证

在实验目录运行 `powershell -ExecutionPolicy Bypass -File tools/build_forward_pause_tests.ps1`。测试不连接ADS或驱动设备，运动功能块使用模拟实现。

- 执行生产PLC状态机，覆盖多周期、终点停留、最终段长度边界、1 ms和60 s等待、非法参数、停留中中止/故障、重新准备及配置锁存。
- 执行生产PLC分块记录器，模拟两次35 s停留；完整输出71,421点，短时32768点缓存满后分块记录仍连续。
- 编译并验证生产C++管道解析、配置校验与ADS写入；新字段类型对照目标PLC声明逐项检查，并模拟每个暂停字段写入失败。
- 将PLC输出送入生产C++记录器，逐行核对归档序号、毫秒时间戳、原始力值及暂停配置元数据。

后端标准构建产物位于 `x64/Debug/`，独立验证构建位于 `x64/Debug_pause/`；测试及归档位于后者的 `verification/` 子目录。WPF使用标准 `AdsControlUI/bin/Debug/net472/` 输出，供后端现有启动逻辑查找。

## PLC更新前需要核对的既有差异

本目录现有后端要求 `G.program_interface_version=20261006`，并读取 `G.selfcheck_status`、`G.program_test_handle_cycle` 等接口；文档指定的PLC源码没有这些声明。因此全量 `tools/test_ads_contract.py` 校验当前失败，这不是暂停字段造成的。不能只添加同名空变量绕过会话/自检检查；应先确认设备实际使用的PLC工程，并核对相应实现。

本次没有修改既有会话和自检协议，没有下载PLC或操作设备。须将暂停逻辑合入与后端匹配的实际PLC工程，完成TwinCAT编译，再在安全的台架条件下验证到位精度、等待时长及中止行为。当前离线测试不替代实机验证。
