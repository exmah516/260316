## 现状结论（基于现有文件）
- 当前报错出现在 ADS 初始化阶段：在 [ADSComm.cpp](file:///c:/Users/admin1/Desktop/251230_nopid/64位ADS%20-%20相对路径%20-%20传数组%20-%20加上手柄/ADSComm.cpp#L113-L160) 调用 `AdsSyncReadStateReqEx` 返回 6，导致 OpenComm 失败。
- 结合公开资料，ADS 错误码 6 常见含义为“目标端口不存在/未找到（target port not found）”，通常由目标 AMS NetID/端口不对、TwinCAT/PLC Runtime 未启动、或路由未建立导致。[参考](https://github.com/Beckhoff/ADS/issues/103)
- 手柄侧单位/方向：`showinfo()` 把 `fJoints2[0] * 1000` 打印为 J 值（mm），说明 `fJoints2[0]` 本体更接近“米”。[手柄.cpp](file:///c:/Users/admin1/Desktop/251230_nopid/64位ADS%20-%20相对路径%20-%20传数组%20-%20加上手柄/手柄/手柄.cpp#L49-L58)

## PLC 代码梳理（需要先在磁盘中找到文件）
- 在当前工作区内未检索到任何 `.TcPOU` 文件（只看到 [下一步工作.txt](file:///c:/Users/admin1/Desktop/251230_nopid/250902/Untitled2/POUs/下一步工作.txt)）。
- 我将先全盘在 `250902` 目录下定位 TwinCAT 工程文件（例如 `.tsproj/.plcproj/.TcPOU/.TcGVL` 等）并读取 `SelfCheck`、`handle` 等 POUs，提取：
  - 全局变量 `G.*` 的定义位置与类型
  - 电缸输出/轴位置数组（如 `nDataOut1[]`）与 ADS 变量映射关系
  - 自检/回退阶段对电缸/轴的强制写入逻辑

## ADS 连接问题修复（不改 PLC 的前提）
- 增强诊断信息：打印“本机 AMS NetID / 目标 AMS NetID / 目标 port”，并把错误码解析为更明确的含义（至少区分：路由错误、端口不存在、目标未启动）。
- 增加可配置目标：支持从命令行或配置文件指定 `--ams 1.2.3.4.1.1 --port 851`，避免写死 `169.254.*`。
- 增加端口探测回退：在 `851` 失败时，按 TwinCAT 常见 PLC 端口序列（例如 851/801/811/821/831）尝试读取状态，快速判断“是端口不对还是路由/Runtime 不在”。
- 增加“只测手柄不连 ADS”的启动选项（例如 `--no-ads`），方便先验证手柄侧逻辑与按钮映射。

## 验证方式
- 新增一个最小化诊断可执行（或在现有 exe 增加诊断模式）：仅做 `AdsPortOpenEx -> AdsGetLocalAddressEx -> AdsSyncReadStateReqEx/ReadDeviceInfo`，输出结果后退出，用于快速定位是端口/路由/Runtime 问题。
- 运行后给出下一步建议：
  - 若探测到 851 存在：继续读取 `G.*` 符号，验证 PLC 符号可见性。
  - 若所有端口都“not found”：提示检查 TwinCAT Runtime 是否 RUN、PLC 是否下载/启动、路由是否建立。

## 后续（在 ADS 正常后再做）
- 在不改按钮/初始化逻辑前提下，实现您描述的“蠕动递送”状态机，并把 10mm 作为参数化常量（可配置），同时统计“手柄前推累计量”与“轴6累计递送量”。