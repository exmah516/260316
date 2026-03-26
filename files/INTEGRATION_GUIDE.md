# 血管介入机器人 Qt 界面 — 接入指南

## 项目结构

```
CatheterRobotUI/
├── CatheterRobotUI.pro          ← Qt 项目文件
└── src/
    ├── main.cpp                 ← Qt 应用入口
    ├── SharedState.h            ← ★ 控制线程 ↔ UI 线程的数据桥梁
    ├── ControlEngine.h/.cpp     ← ★ 你的 main.cpp 重构后的容器
    ├── ControlThread.h/.cpp     ← 独立线程运行 ControlEngine
    ├── MainWindow.h/.cpp        ← 主界面(6个功能区)
    ├── AxisDisplayWidget.h      ← 单轴位置条形控件
    ├── ForcePlotWidget.h        ← 力信号实时曲线
    ├── ForceRecorder.h          ← 力信号 CSV 录制
    └── legacy/                  ← ★ 放你现有的代码(新建此目录)
        ├── Handle.h
        ├── Handle.cpp           ← 原 手柄.cpp, 改名为英文
        ├── ADSComm1.h
        └── ADSComm.cpp
```

## 线程模型

```
┌─────────────────┐       SharedState        ┌─────────────────┐
│   ControlThread  │  ←─── mutex 保护 ───→   │    UI Thread     │
│                  │                          │                  │
│  ControlEngine   │  ControlToUI (状态)→     │   MainWindow     │
│   .tick() 10ms   │  ←UIToControl (命令)     │   QTimer 30ms    │
│                  │                          │                  │
│  Handle 582/587  │  ForceSample 队列→       │  ForcePlotWidget │
│  CADSComm        │                          │  ForceRecorder   │
└─────────────────┘                          └─────────────────┘
```

规则：
- 控制线程**不碰**任何 Qt 控件
- UI 线程**不调用**手柄/ADS 函数
- 所有跨线程数据都经过 `SharedState` 的 mutex

---

## 接入步骤(共 11 步)

### 准备工作

1. 安装 Qt 5.15+ 或 Qt 6.x (需包含 Charts 模块)
2. 在项目目录下新建 `src/legacy/` 目录
3. 把你的文件复制进去:
   - `Handle.h` → `src/legacy/Handle.h`
   - `手柄.cpp` → `src/legacy/Handle.cpp` (改为英文文件名)
   - `ADSComm1.h` → `src/legacy/ADSComm1.h`
   - `ADSComm.cpp` → `src/legacy/ADSComm.cpp`

4. 修改 `CatheterRobotUI.pro`:
   - 取消 `INCLUDEPATH` 和 `LIBS` 的注释, 改为你本机的实际路径
   - 取消 legacy 文件的注释

5. 在 `ControlEngine.h` 顶部:
   ```cpp
   // 删掉这两行前向声明:
   class Handle;
   class CADSComm;
   // 替换为:
   #include "legacy/Handle.h"
   #include "legacy/ADSComm1.h"
   ```

### 步骤 1-4: 初始化代码迁移

打开 `ControlEngine.cpp` 的 `init()` 函数, 按照 TODO 注释,
把你 `main.cpp` 对应行号的代码**取消注释并适配**:

| TODO 标记 | 对应 main.cpp 行号 | 说明 |
|-----------|-------------------|------|
| 接入步骤 1 | 244~263 | 创建并初始化两个 Handle 对象 |
| 接入步骤 2 | 265~283 | 连接 ADS (本地优先, 远程备选) |
| 接入步骤 3 | 526~584 | 首次读取 PLC, 同步手柄零位 |
| 接入步骤 4 | 77~115, 214~504 | CrawlState 和相关常量/lambda→成员 |

**步骤 4 详解 — lambda 转成员函数:**

你 main.cpp 中有大量 lambda (sync_all, sync_axis1, sync_axis6 等),
需要转成 ControlEngine 的成员函数:

```cpp
// main.cpp 中:
auto sync_all = [&](int samples) -> bool { ... };

// 改为 ControlEngine.h 中声明:
bool syncAll(int samples);

// ControlEngine.cpp 中实现:
bool ControlEngine::syncAll(int samples) {
    // lambda 体原封不动, 只是把 capture 的变量换成成员变量
    // 例如: plc_act_pos → plc_act_pos_
    //       handle_axis1 → handle_axis1_->
    //       ads → ads_->
}
```

需要转换的 lambda 列表:

| main.cpp 中的 lambda | 转为成员函数 |
|---------------------|------------|
| `read_plc_state` | `readPlcState()` (已提供) |
| `write_refer` | `writeRefer()` (已提供) |
| `load_pos_from_actual` | `loadPosFromActual()` (已提供) |
| `apply_cmd_force` | `applyCmdForce(double)` |
| `sync_all` | `syncAll(int samples)` |
| `sync_axis1` | `syncAxis1(int, bool, int)` |
| `sync_axis6` | `syncAxis6(int, bool, bool, int)` |
| `release_axis6_to_follow` | `releaseAxis6ToFollow()` |
| `capture_axis1_follow_baseline` | `captureAxis1FollowBaseline()` |
| `clear_plc_reinit_req` | `clearPlcReinitReq()` |
| `axis1_start_abs` | `axis1StartAbs()` |
| `axis1_end_abs` | `axis1EndAbs()` |

同时, 以下 main.cpp 局部变量需要变成类成员:

```cpp
// 在 ControlEngine.h 的 private 区添加:

// 来自 main.cpp 的常量(建议保持 const)
static constexpr double k_handle_to_mm_base = 500.0 * (75.0 / 50.0);
static constexpr double axis_push_sign = -1.0;
static constexpr double axis_rot_scale_deg = 57.29577951308232;
// ... 其余 crawl 相关常量 ...

// 来自 main.cpp 的状态变量
CrawlState axis1_crawl_;
CrawlState axis6_crawl_;
double axis3_base_rel_ = 0.0;
double axis5_base_rel_ = 0.0;
double axis6_mirror_base_rel_ = 0.0;
bool pause_pressed_prev_ = false;
bool axis6_arm_pressed_prev_ = false;
// ... 其余状态变量 ...
```

> **注意**: CrawlState 结构体已经在你的 main.cpp 中定义, 把它移到
> `ControlEngine.h` 或单独的头文件中。

### 步骤 5-11: tick() 循环体迁移

打开 `ControlEngine.cpp` 的 `tick()` 函数, 按 TODO 注释逐段搬入。

**核心改动点:**

#### 步骤 5: 手柄轮询(直接搬)
```cpp
handle_axis1_->poll();
handle_axis6_->poll();
```

#### 步骤 6: 控制模式(新增)
```cpp
// 根据 587 按钮判断, 你需要确定哪个 bit 对应哪个模式
const unsigned char btn587 = handle_axis6_->buttons2;
if (btn587 & YOUR_GUIDEWIRE_BUTTON)
    current_mode_ = ControlMode::Guidewire;
else if (btn587 & YOUR_BOTH_BUTTON)
    current_mode_ = ControlMode::GuidewireCatheter;
else
    current_mode_ = ControlMode::Catheter;
```

#### 步骤 7: 暂停逻辑(合并 UI + 手柄)
```cpp
// 原来只有手柄按钮控制暂停, 现在 UI 也可以
const bool ui_pause = cmd.cmd_pause;  // 来自 applyUICommands
const bool handle_pause = (handle_axis1_->buttons2 & axis1_pause_button_mask) != 0;
// 两者任一为 true 都暂停
```

#### 步骤 9: 力反馈(修改增益)
```cpp
// 原来:
const double axial_gain = 1.0 / 1000.0;
// 改为:
const double axial_gain = (1.0 / 1000.0) * force_gain_axial_;

// 原来:
torque_force = clamp_double(torque_force, -1.0, 1.0);
// 后面加:
torque_force *= force_gain_torque_;
```

#### 步骤 10: 手柄灵敏度
```cpp
// 原来:
const double axis1_hand_delta_mm =
    (handle_axis1.fJoints2[0] - axis1_crawl.handle_ref)
    * k_handle_to_mm * axis_push_sign;
// 改为:
UIToControl cmd = shared_.readUICommands();
const double effective_k = k_handle_to_mm_base * cmd.handle_sensitivity;
const double axis1_hand_delta_mm =
    (handle_axis1_->fJoints2[0] - axis1_crawl_.handle_ref)
    * effective_k * axis_push_sign;
```

### 完成后删除

把所有 TODO 注释中的 `/* ... */` 注释块解开后, 删掉临时的:
- `class Handle;` 前向声明
- `class CADSComm;` 前向声明
- 所有 `return true;` 占位符

---

## 数据流总结

```
用户拖动"球囊轴速度"滑块
  → MainWindow::onAxis4VelocityChanged()
    → shared_.modifyUICommand(设置 axis4_velocity)
      → ControlEngine::applyUICommands() 读取
        → ads_->ADSWrite("G.v_limit[4]", ...)
          → PLC handle 功能块使用新速度
```

```
PLC 实际位置变化
  → ControlEngine::readPlcState() 每轮读取
    → ControlEngine::publishState() 写入 SharedState
      → MainWindow::onRefreshTimer() 每 30ms 读取
        → AxisDisplayWidget::setPosition() 更新条形图
```

```
力传感器产生信号
  → PLC G.fn_value / G.ft_value
    → ControlEngine::tick() ADS 读取, 滤波, 映射
      → 实时: publishState() → ForcePlotWidget 曲线
      → 录制: pushForceSample() → ForceRecorder → CSV 文件
      → 反馈: handle_axis1_->setforce_axis() → 手柄电机
```

---

## 编译与运行

```bash
# 在项目目录打开 Qt Creator 或命令行:
qmake CatheterRobotUI.pro
nmake          # MSVC
# 或
mingw32-make   # MinGW
```

确保:
- FLCatheter SDK 的 DLL 在 PATH 或 exe 同目录
- TwinCAT ADS 的 DLL 可访问
- TwinCAT 运行时已启动(本地或远程)

---

## 后续扩展建议

1. **机构示意图**: 在轴位置面板叠加一个简化的 2D 示意图(QGraphicsView)
2. **日志面板**: 替代 stdout, 用 QPlainTextEdit 显示控制线程的日志
3. **参数持久化**: 用 QSettings 保存速度/增益/窗口大小等参数
4. **报警历史**: 记录 PLC 错误码和时间戳, 显示在独立面板
5. **国际化**: 用 Qt Linguist 做中英文切换
