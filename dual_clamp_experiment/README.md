# 双机构夹持扰动实验版

本目录是独立的实验控制器，不会初始化原程序的手柄、定位臂、注射器或轴15控制。

## 工程

- `DualClampExperiment.sln`：C++ ADS后端。
- `AdsControlUI/AdsControlUI.sln`：WPF可视化界面。
- `DualClampExperiment.vcxproj`：x64 C++工程，依赖原工程中的 `ADS\x64\lib\TcAdsDll.lib` 和 `ADS\x64\TcAdsDll.dll`。

## 轴和电缸

- 轴1：平移，实验夹爪为电缸2。
- 轴2：轴1侧周向角度。
- 轴6：平移，实验夹爪为电缸4。
- 轴7：轴6侧周向角度。
- 普通空闲状态下电缸1、电缸3由PLC保持正常输送值；程序递送模式可通过配合开关让电缸1或电缸3随运动端同步切换。

## PLC接口

`250902/250902/Untitled2` 中保留旧 `DualClampExperiment.TcPOU`，并新增 `ProgrammedDeliveryExperiment.TcPOU` 与 `G.program_test_*` 接口。程序模式1为导管测试，模式2为导丝测试；两套新接口不改旧 `dual_clamp_*` 命令和状态格式。
PLC任务周期为1 ms；长时间记录使用2个512点分块交替传输，32768点数组仅作为PLC短时诊断缓存。采样包含运动状态、电缸命令和四路力感原始值；C++按“取零→0~50 g斜率→机构安装校正”生成校正量，并在CSV中追加N、N·mm和串扰解耦字段，另生成`events.csv`。

界面中的操作顺序为“等待PLC自动自检完成”→“选择模式”→“准备定位”→“开始测试”。自检完成后轴2、轴7已按原程序绝对定位到0度；界面不会发送自检请求，也不会自动准备定位或自动开始实验。导管模式默认使用轴1距左限位23 mm、触发位置3 mm，两个位置可在界面修改；导丝模式由轴5输入位置，轴6自动使用轴5+21 mm。夹紧/释放WORD固定沿用原程序，不在界面中重新判定。

旧模式只选择运动端：选择轴1时轴6为固定端，选择轴6时轴1为固定端。程序模式完全由PLC状态机控制：导管只使能轴1、轴2，导丝只使能轴5、轴6、轴7；每周期按前向触发、释放、回退、重新夹紧执行，最后周期额外执行一次最终前向段。导管模式可选电缸1是否参与电缸2的同步配合，导丝模式可选电缸3是否参与电缸4的同步配合；默认开启，关闭时对应电缸保持400。实时状态和曲线按当前模式只显示相关轴与相关力感通道，`fn`/`ft`单位均为N；CSV中的扭矩字段单位为N·mm，重力补偿暂不执行。

## 编译

```powershell
$msbuild = 'D:\Work_software\VS2022\MSBuild\Current\Bin\MSBuild.exe'
& $msbuild '.\dual_clamp_experiment\DualClampExperiment.vcxproj' /p:Configuration=Debug /p:Platform=x64 /m
dotnet build '.\dual_clamp_experiment\AdsControlUI\AdsControlUI.csproj' -c Debug -p:Platform=x64
```

TwinCAT PLC工程需要在安装 TwinCAT XAE 的 Visual Studio 中打开 `250902.sln` 后执行 PLC Build；本机命令行未提供独立的 TwinCAT PLC编译器。
