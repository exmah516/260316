# 血管介入机器人 · ADS 项目

主从分离架构：**主端**只有界面和手柄；**从端（数据端）** 放在机器人附近，经网线直连倍福 PLC。两端通过 SD-WAN 异地组网。

## 目录结构

```
.
├─ ADS.sln / ADS.vcxproj     从端 C++ 上位机（当前源码仍在根目录，见“待办”）
├─ *.cpp / *.h               从端源码：ADS 通信、运动、力反馈、记录、相机、本地 UI 桥接
├─ ADS/  手柄/               ADS 库、手柄 SDK
├─ AdsControlUI/             从端本地调试台（WPF，命名管道，保留自检/定位臂/记录等调试功能）
├─ master/                   主端
│  ├─ MasterConsole/            主端界面（WPF）：显示区、状态区、操作区、日志、链路区
│  ├─ MasterConsole.Protocol/   协议 C# 实现（编解码、认证、命令 JSON）
│  ├─ MasterConsole.Protocol.Tests/  协议对拍自检
│  └─ MasterConsole.sln
├─ protocol/                 协议权威定义
│  ├─ PROTOCOL.md               字节布局、握手、命令、超时矩阵
│  ├─ cpp/remote_protocol.h     从端 C++ 结构体（带 static_assert 长度检查）
│  └─ test_vectors/             固定样本帧与认证向量（由 tools 脚本生成）
├─ config/                   配置模板（真实密钥不入库）
├─ tools/                    脚本（协议向量生成等）
├─ docs/                     说明文档：architecture / guides / experiments / images / logs
├─ data/                     历史实验 CSV（不入库）
└─ records/                  运行时记录目录（程序写入，路径固定，勿移动）
```

## 主端界面分区

- 顶部：连接、控制权、急停、力反馈、从端 ADS 状态；模式与阶段（仅显示）；连接/申请控制权按钮。
- 显示区：影像占位框、URDF 模型占位框、事件日志。
- 状态区：模式（仅显示）、手柄 582/587 的力与扭矩、纯净力；七轴位置/行程预留。
- 操作区：进入器械准备位置（两个位置输入）、力反馈开关（开启前自动零点采集）、电缸 1–4、Y 阀关闭、注射器 1/2 按住推拉。
- 底部链路区（小号字体）：会话、往返时延、状态数据龄、触觉帧龄、状态帧率、丢弃帧、从端 ADS。

已移除（不进入远程协议）：实时力窗口、重力补偿、力反馈保持、手动零点采集按钮。

## 当前状态

- 从端：`remote_gateway.cpp/.h` 已实现并接入 `main.cpp`（TCP 32000 命令、UDP 32001 控制/状态）。命令翻译成既有 `VisCommand`，与本地调试台共用 `main.cpp` 的命令循环。注射器与轴4点动租约统一为 500 ms。
- 主端：`RemoteRobotLink` 已实现；找到 `remote.token` 就连真实从端，否则退回模拟链路。
- 尚未实现：手柄采样上行与触觉帧下行（控制帧里的手柄字段目前只校验不使用，触觉帧龄显示为空）、影像、URDF。
- 所有新增 C++ 与 C# 代码均未在本机编译过（沙箱没有 MSVC），需在 Visual Studio 中编译。

## 本机联调（同一台电脑）

1. 复制 `config/remote.token.sample` 为 `remote.token`（至少 16 个字符），放在 ADS.exe 的工作目录，同时放在 `config/` 下供主端读取。两份内容必须一致。没有这个文件时从端网关不启用。
2. 编译并运行 `ADS.exe`，控制台应出现“远程网关：已启动”。
3. 运行 MasterConsole。默认连 `127.0.0.1`；跨机用 `--host <从端地址>`，`--sim` 强制模拟，`--token <文件>` 指定密钥。
4. 先点连接，再点申请控制权；操作区在持有控制权之前保持锁定。
5. 涉及运动的命令（准备位置、电缸、注射器、轴4）联调时请在场并保证急停可用。

网关监听 0.0.0.0，已有 HMAC 挑战认证，但没有加密；跨网络使用时只放在 SD-WAN 内网，不要映射到公网。

## 如何验证协议

1. `python tools/gen_protocol_vectors.py` 重新生成向量。
2. 在 Visual Studio 打开 `master/MasterConsole.sln`，运行 `MasterConsole.Protocol.Tests`，退出码 0 即与 Python 实现逐字节一致。

## 待办

1. 在 VS 中编译 `master/MasterConsole.sln`，修正可能的编译错误，目视检查界面。
2. 联调网关：先本机，再 SD-WAN；做断网、重放、控制帧超时测试。
3. 手柄迁移：主端读取手柄，通过 ControlFrame 发送；从端按 HandleSample 驱动原有运动逻辑。
4. 拆分 `main.cpp`（约 5000 行）并把 C++ 源码移入 `slave/`，同步修改 `ADS.vcxproj` 路径。
5. 影像和 URDF 控件接入占位框。
