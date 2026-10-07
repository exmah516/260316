# 步骤 1：主端显示旧模型和旧离线默认值

日期：2026-10-07  
范围：仅接入旧 URDF 静态显示；未修改 StatusFrame、remote_gateway、PLC、控制权、力反馈或手柄逻辑。

## 已实施

- 将 A 的 `build/Debug/bin/net472/URDF` 按原层级复制到 `master/MasterConsole/URDF`。
- 入口固定为程序输出目录下的 `URDF/urdf.urdf`；运行时使用 `AppDomain.CurrentDomain.BaseDirectory`，不依赖当前工作目录。
- 新增 `Controls/RobotViewport.xaml(.cs)`：WPF `WindowsFormsHost` 承载旧版 `RenderWindowControl`，复用旧 `UrdfStruct.cs` 的 `Load/AddModel/MoveJoint/UpdateMove` 和旧 `Math4wpf.dll` 的 `UrdfMath`。
- 旧 `UrdfStruct.cs` 复制到 `Services/UrdfStruct.cs`，保持 `Rz*Ry*Rx`、4×4 行布局、`m1*m2` 顺序及 URDF 原点/轴向/颜色/几何参数。加载前逐项检查 URDF 的网格引用，缺失时在模型区域显示具体文件路径。
- 机器人模型占位区已替换为 `RobotViewport`；状态层和“恢复视角”按钮置于原生 VTK 子窗口之上；窗口关闭时释放模型控件。
- 初始姿态显式设置为 9 个动态关节全 0，对应旧 `MsStatus.GetJointValue` 无数据默认；状态显示“旧模型默认姿态 / 未接入 DSA 机械臂”。未接入 B 的 `AxisPos`、`AxisFromLeft`、`arm_act_pos`。
- 相机复用旧 `MonitorWnd.ViewX`：位置 `(5.8,0.7,0)`，焦点 `(0,0.7,0)`，向上 `(0,0,1)`，裁剪范围 `(0.1,1000)`；提供“恢复视角”按钮。
- `MasterConsole.csproj` 保持 `net472/x64`，启用 WPF/Windows Forms，引用 `Activiz.NET.x64 5.8.0`、`Math4wpf.dll`，并把 ActiViz 同版本原生 DLL 复制到输出目录。

## 资源核对

- URDF 入口与 12 个 STL 网格逐项与 A 对拍，文件内容 SHA-256 全部一致。
- 输出目录 `master/MasterConsole/bin/x64/Debug/net472` 已包含 URDF、`Math4wpf.dll`、`Kitware.VTK.dll`、`Kitware.mummy.Runtime.dll` 及 ActiViz 原生 DLL；`HCNetSDK` 未在本步骤复制。
- 旧 `Math4wpf` 可直接作为现成 x64 DLL 使用，因此没有新增 C# 数学替代实现。

## 构建与检查

执行：

```powershell
dotnet restore master/MasterConsole/MasterConsole.csproj -p:Platform=x64
dotnet build master/MasterConsole/MasterConsole.csproj --no-restore -c Debug -p:Platform=x64
dotnet build master/MasterConsole/MasterConsole.csproj --no-restore -c Release -p:Platform=x64
```

结果：Debug 和 Release 的 x64 构建均成功，均为 0 个错误；各有 `MainViewModel._linkStatsText` 未使用警告 2 个（主项目与临时 WPF 项目各 1 个），与本步骤无关。

离线启动烟测：

```powershell
Start-Process master/MasterConsole/bin/x64/Debug/net472/MasterConsole.exe -ArgumentList '--sim'
```

结果：进程正常启动并保持响应，窗口标题为“血管介入机器人 · 主端控制台”；未连接真实 ADS/PLC/机器人。另以 `C:\Windows` 为工作目录启动，进程仍保持响应；通过输出目录存在性和 14 个 URDF 文件逐项哈希对拍。

## 未完成/待验证

- 本环境未能取得可用的原生窗口截图输出，因此“实际模型截图”和视觉确认标记为待完成；进程启动烟测已通过。
- 尚未在不同工作目录启动并人工观察模型画面，也未故意删除单个网格做弹窗/状态文字视觉测试。
- ActiViz/VTK DLL 已随输出复制，但未做显卡、不同 DPI、最小窗口和重复打开/关闭的实机视觉验收。
- 海康、DSA 截屏、中继和独立 DSA 机械臂状态均不属于本步骤。
