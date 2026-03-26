## 现有项目逻辑（我已读完，作为改动基线）

### PLC（c:\Users\admin1\Desktop\251230_nopid\250902\）
- **状态机**：`G.gen_state`，定义在 [state.TcDUT](file:///c:/Users/admin1/Desktop/251230_nopid/250902/250902/Untitled2/DUTs/state.TcDUT)。
- **主调度**：每周期更新 7 轴对象后，按 `G.gen_state` 调用 `init / SelfCheck / handle / err / clear_err / reset`，[MAIN.TcPOU](file:///c:/Users/admin1/Desktop/251230_nopid/250902/250902/Untitled2/POUs/MAIN.TcPOU)。
- **init**：7 轴上电，超时 `T#5S`，成功后 `G.init_pos[i]=ActPos` 并转 `_self_check`，[init.TcPOU](file:///c:/Users/admin1/Desktop/251230_nopid/250902/250902/Untitled2/POUs/init.TcPOU)。
- **SelfCheck（现在）**：
  - 目标轴 `target_axes=[1,3,5,6]`，逼近右限位速度 `vel_scan=10`，回退速度 `vel_back=50`。
  - 回退顺序 `seq_order=[1,3,5,6]`（串行放行），回退目标是 `rightlimit + back_dist`。
  - 关键常量：`back_dist=[-60,0,-200,0,-130,-60,0]`；到限位判据 100ms；Stop: Decel 500/Jerk 1000。
  - 自检启动时写电缸：`cylinder1_value=1500`、`cylinder2_value=0`。
  - 完成后：`self_check_done=TRUE`、`handle_reinit_req=TRUE`、并 `gen_state=_handle`。
  - 见 [SelfCheck.TcPOU](file:///c:/Users/admin1/Desktop/251230_nopid/250902/250902/Untitled2/POUs/SelfCheck.TcPOU)。
- **handle（现在）**：
  - 上位机写 `G.refer[1..7]`（相对坐标），PLC 先做软限位裁剪，再按 `G.v_limit` 限速生成 `G.ref_slow`，再做窗口平均生成 `G.act_h`，最终下发 `MC_ExtSetPointGenFeed(Position:=G.act_h+G.init_pos)`。
  - `G.v_limit=[7.5,7.5,1.5,1.5,1.5,7.5,1.5]`。
  - 轴6快速回撤标志：`G.axis6_fast_retract` 时，轴6用 `v_step=1000` 且 `avg_window=1`。
  - 见 [G.TcGVL](file:///c:/Users/admin1/Desktop/251230_nopid/250902/250902/Untitled2/GVLs/G.TcGVL) 与 [handle.TcPOU](file:///c:/Users/admin1/Desktop/251230_nopid/250902/250902/Untitled2/POUs/handle.TcPOU)。

### 手柄上位机（c:\Users\admin1\Desktop\251230_nopid\64位ADS - 相对路径 - 传数组 - 加上手柄）
- **按钮位定义**：b0=0x01、b5=0x20、b6=0x40、b7=0x80；base 常见 0x06（bit1/bit2 固定 1），见 [按钮说明.txt](file:///c:/Users/admin1/Desktop/251230_nopid/按钮说明.txt)。
- **ADS 交互变量**：读写 `G.Act_pos / G.refer / G.cylinder1_value / G.cylinder2_value / G.fn_value / G.ft_value / G.self_check_done / G.handle_reinit_req / G.estop_hold_req / G.axis6_fast_retract`。
- **当前运动逻辑**：轴6（`pos[5]`）为蠕动控制、轴7（`pos[6]`）由编码器映射旋转；蠕动参数在 [main.cpp:L218-L234](file:///c:/Users/admin1/Desktop/251230_nopid/64%E4%BD%8DADS%20-%20%E7%9B%B8%E5%AF%B9%E8%B7%AF%E5%BE%84%20-%20%E4%BC%A0%E6%95%B0%E7%BB%84%20-%20%E5%8A%A0%E4%B8%8A%E6%89%8B%E6%9F%84/main.cpp#L218-L234)。

---

## 我理解的“目标逻辑”（按你这条消息逐条落地，不加额外功能）

### A. 电缸命名/映射（你已在 PLC 侧完成接线与更名，本轮只按新语义使用）
- `cylinder1_value`：夹持机构电缸（逻辑同原轴5电缸）：**1000 张开、100 闭合**。
- `cylinder2_value`：轴1器械电缸（逻辑同原轴6电缸）：**500 闭合、0 张开**。
- 原电缸1/2 改名为电缸3/4，并接到 `cylinder3_value/cylinder4_value`。
  - 我理解：后续 b6 流程会同时控制 “电缸1、3” 以及 “电缸2、4”，所以 PLC 侧必须能被 ADS 写到 `cylinder3_value/cylinder4_value`。

### B. SelfCheck：保留逼近右限位，但回退不再按顺序
- 仍对轴1/3/5/6 做右限位逼近并记录 `G.rightlimit[]`。
- 回退阶段：四轴**同时**回退到“距右限位指定距离”的位置：
  - 轴1：右限位 - 5mm（即 `rightlimit[1] - 5`）
  - 轴3：右限位 - 413mm
  - 轴5：右限位 - 253.7mm
  - 轴6：右限位 - 74.2mm
- 旋转轴（2/4/7）仍按原逻辑去绝对 0。

### C. b6（0x40）一次性按钮流程
- b6 只在“第一次按下”有效（上升沿触发并锁存已用）。
- 触发后动作序列：
  1) 电缸1 + 电缸3 夹紧（各自闭合值），等待 100ms。
  2) 轴1 到右限位 - 56mm 并停；轴3/5/6 分别到右限位 - 80/-40/-20 并停。
  3) 电缸1 + 电缸3 松开；电缸2 + 电缸4 夹紧。
- 我理解这应由 **PLC 执行绝对运动**（MC_MoveAbsolute），上位机只负责“检测 b6 上升沿并置位触发变量”，避免上位机自己算绝对坐标/同步。

### D. 新蠕动逻辑：手柄控制夹持机构+轴1；轴3镜像轴1的正常段
- 手柄推拉（`fJoints2[0]`）→ **轴1平移**；手柄旋转（`fJoints2[1]`）→ **轴2旋转**。
- 蠕动端点定义（都用“距轴1右限位的距离”表达，且预留接口改区间）：
  - 起始端 `D_start = 56mm`（轴1回退后停在这里；正常段起点）
  - 终止端 `D_end = D_start - segment_len`；当前 `segment_len=10mm` → `D_end=46mm`
- 正常控制区间（轴1在 [46,56] 之间）：
  - 电缸1松开、电缸2夹紧保持不变；轴1随手柄；轴2随旋转；
  - 轴3要“复刻轴1的位移/方向/速度加速度”（本质就是跟随同一套 refer 变化），但 **不复刻碰端点后的快速回跳**。
- 触碰端点触发蠕动回跳：
  - 手柄前推触到终止端 46：电缸1夹紧、电缸2松开；等待电缸动作时间；轴1以最大速度回到起始端 56。
  - 手柄回拉触到起始端 56：电缸1夹紧、电缸2松开；等待电缸动作时间；轴1以最大速度去终止端 46。
  - 回跳期间：轴3不执行回跳，但需要 **暂停**（保持其参考不再更新），直到轴1回跳完成再恢复。

---

## 实施计划（你确认后我再开始改代码）

### 1) PLC：修改 SelfCheck 回退逻辑（并行回退 + 新距离）
- 编辑 [SelfCheck.TcPOU](file:///c:/Users/admin1/Desktop/251230_nopid/250902/250902/Untitled2/POUs/SelfCheck.TcPOU)
  - 将 `back_dist` 改为 `[-5, 0, -413, 0, -253.7, -74.2, 0]`。
  - 移除 `seq_order/seq_idx` 的串行放行；在 all_reach_limit 后让轴1/3/5/6同时进入 SetPosition+MoveAbsolute。

### 2) PLC：扩展 GVL/增加 b6 触发与新电缸变量
- 编辑 [G.TcGVL](file:///c:/Users/admin1/Desktop/251230_nopid/250902/250902/Untitled2/GVLs/G.TcGVL)
  - 增加 `cylinder3_value AT %Q* : WORD`、`cylinder4_value AT %Q* : WORD`（按你现有接线）。
  - 增加 `b6_req : BOOL`、`b6_done : BOOL`、`b6_used : BOOL`（一次性触发/完成反馈）。
  - 增加 `axis1_fast_return : BOOL`（仅用于轴1端点回跳时让 PLC 放宽限速/平均，实现“最大速度”）。

### 3) PLC：实现 b6 一次性流程（建议作为新 FB + MAIN 分发或在 handle 内分支）
- 新增一个 POU（例如 `b6_action.TcPOU`）并在 `MAIN` 中加一个状态或在 `_handle` 内先判断 `b6_req`。
- 使用 `MC_MoveAbsolute` 目标 = `G.rightlimit[i] - D`；完成判定用 `Done` 或 `abs(ActPos-target)<tol`。
- 电缸动作按你给的数值 + 定时 `100ms`。

### 4) 上位机：按新按钮表修改 b6 检测与触发
- 编辑 [main.cpp](file:///c:/Users/admin1/Desktop/251230_nopid/64%E4%BD%8DADS%20-%20%E7%9B%B8%E5%AF%B9%E8%B7%AF%E5%BE%84%20-%20%E4%BC%A0%E6%95%B0%E7%BB%84%20-%20%E5%8A%A0%E4%B8%8A%E6%89%8B%E6%9F%84/main.cpp)
  - 识别 b6（mask 0x40）上升沿；仅首次写 `G.b6_req := TRUE`。
  - 之后依赖 PLC 的 `b6_done/b6_used`，上位机不再二次触发。

### 5) 上位机：替换运动控制为“轴1/轴2 蠕动 + 轴3镜像”
- 将现有轴6/轴7 蠕动段改为：
  - `pos[0]` 控轴1；`pos[1]` 控轴2；`pos[2]` 镜像轴3。
  - 端点用“距右限位的距离”参数：`D_start=56`、`segment_len=10`（可改），`D_end=D_start-segment_len`。
  - 端点触发回跳时置位 `G.axis1_fast_return=TRUE` 以获得最大速度；回跳完成后清零。
  - 轴3在回跳期间保持其 refer 不更新，等轴1回跳完成再恢复。

如果你认可上面这份“目标逻辑理解”，我再开始进入编码阶段。