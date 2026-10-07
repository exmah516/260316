# 步骤 5：主端直接接收独立 DSA ScreenCut JPEG

实施日期：2026-10-07

## 旧协议核实

已读取旧工程 `A/ScreenCut/ScreenCut/MainWindow.xaml.cs`，确认：

- 流 0 发送到 UDP `36661`，命令接收端口 `36662`。
- 流 1 发送到 UDP `36663`，命令接收端口 `36664`。
- 每帧先发送 5 字节：`流编号(1) + JPEG长度(4，大端)`。
- JPEG 数据随后按最多 `8192` 字节的数据报发送，不能按单个 UDP 包解码。
- 控制命令固定为 2 字节：流 0 快发为 `[1,0]`，流 1 快发为 `[0,1]`；慢发为 `[0,0]`。

没有修改或合并旧 ScreenCut 工程。

部署时旧 ScreenCut 的 `txtIP` 应填写主端 IP；主端 `dsa_screencut.ini` 的 `screenCutIp` 用于快慢控制回路，应填写 DSA 电脑上 ScreenCut 的 IP。

## 目标工程改动

- `master/MasterConsole/Services/DsaJpegReceiver.cs`
  - 后台监听 36661/36663，可由 `config/dsa_screencut.ini` 配置。
  - 每路独立重组 JPEG，校验长度上限、SOI/EOI，5 秒未完成则丢弃当前帧。
  - 只保留完整 JPEG 交给显示层；不转发、不重编码。
  - 向 ScreenCut 的 36662/36664 发送两字节快慢控制命令。
- `master/MasterConsole/Controls/DsaImageView.xaml(.cs)`
  - 接收完整 JPEG 字节；后台解码 `BitmapImage` 并 `Freeze`。
  - UI 定时器只交接最新已解码图像到 WPF，保持比例并显示等待/在线/过期/失败。
- `master/MasterConsole/Views/MainWindow.xaml(.cs)`
  - 启动/关闭独立 DSA 接收器；显示两路接收状态；增加“DSA快发/DSA慢发”按钮。
- `config/dsa_screencut.ini.example`
  - 提供端口、ScreenCut IP 和 JPEG 最大长度配置样例。

## 离线验证

```powershell
dotnet build master/MasterConsole/MasterConsole.csproj -c Debug -p:Platform=x64 --no-restore
dotnet build master/MasterConsole/MasterConsole.csproj -c Release -p:Platform=x64 --no-restore
```

Debug/Release 均通过，0 错误；既有 `_linkStatsText` 警告仍存在。离线启动后确认主端占用 UDP 36661 和 36663；使用假发送端分别发送协议头和 JPEG 数据报后进程保持响应并可正常退出。没有运行真实 DSA ScreenCut，未声称实机影像联调成功。
