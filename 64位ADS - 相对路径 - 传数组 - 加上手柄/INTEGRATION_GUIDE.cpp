// ============================================================
// 集成指南：如何将 ForceSignalProcessor.h 接入 main.cpp
// ============================================================
//
// 总共只需要改 main.cpp 的 3 个位置，不超过 20 行。
//
// ============================================================
// 改动 1：在 main.cpp 顶部加一行 include
// ============================================================
//
// 在 #include "Handle.h" 下面加：
//
//   #include "ForceSignalProcessor.h"
//

// ============================================================
// 改动 2：在 main() 函数中声明处理器实例
// ============================================================
//
// 在 main() 函数开头，和其他 state 变量放在一起（比如紧挨着
// ForceFeedbackState ff; 的下面），加一行：
//
//   ForceSignalProcessor force_processor;
//

// ============================================================
// 改动 3：在主循环中，process_force_feedback() 调用之前插入处理
// ============================================================
//
// 找到主循环中调用 process_force_feedback() 的位置。
// 在它前面插入以下代码：
//
//   // --- 力信号在线处理 ---
//   if (force_sample.valid)
//   {
//       force_processor.update(
//           force_sample.fn_1_value,
//           force_sample.ft_1_value,
//           force_sample.fn_2_value,
//           force_sample.ft_2_value,
//           current_push_pull_code,    // 你已有的 push_pull_code 变量
//           current_rot_sign_code,     // 你已有的 rot_sign_code 变量
//           fast_move_active           // axis1_fast_return || axis6_fast_retract
//       );
//
//       // 用处理后的值覆盖原始采样（这样下游映射函数自动吃到干净值）
//       force_sample.fn_1_value = static_cast<short>(force_processor.output.fn_1);
//       force_sample.ft_1_value = static_cast<short>(force_processor.output.ft_1);
//       force_sample.fn_2_value = static_cast<short>(force_processor.output.fn_2);
//       force_sample.ft_2_value = static_cast<short>(force_processor.output.ft_2);
//   }
//
//   // 然后正常调用 process_force_feedback(...)，不需要改它的任何代码

// ============================================================
// 注意事项
// ============================================================
//
// 1) ForceSignalProcessor.h 是纯头文件，没有 .cpp，不需要改编译脚本。
//    把 .h 文件放在和 main.cpp 同目录即可。
//
// 2) 补偿后的值从 double 转回 short 会有截断。
//    如果你想保留更高精度，可以改 map_fn1_to_force_582() 等函数
//    接受 double 而不是 short。但这是可选优化，不影响基本功能。
//
// 3) 如果你想在 CSV 日志中同时记录原始值和补偿后值，
//    可以在 ForceLogState 的写入行中追加列：
//      force_processor.output.fn_1,
//      force_processor.output.conf_fn_1,
//      force_processor.filter_fn_1.zero_estimate
//    这样就能在论文中画出"原始 vs 补偿 vs 零位跟踪"三线图。
//
// 4) 参数调节指南（先用默认值跑一次，再根据效果调）：
//
//    spike_k (默认 3.5)
//      - 太小 → 正常力变化被误判为尖峰
//      - 太大 → 真正的异常尖峰漏检
//      - 建议范围：3.0 ~ 5.0
//
//    spike_window (默认 32)
//      - 采样率高的话可以加大到 64
//      - 太大会增加延迟（中值计算的滞后）
//
//    quiet_min_samples (默认 20)
//      - 太小 → 短暂停顿就触发零位更新，可能不稳定
//      - 太大 → 需要很长静止才更新，零位跟踪太慢
//      - 根据你的采样率调：如果 100Hz 则 20 = 200ms 静止
//
//    zero_alpha (默认 0.3)
//      - 正常工作时的零位跟踪平滑度
//      - 0.1 = 非常保守，0.5 = 比较激进
//
//    zero_alpha_fast (默认 0.8)
//      - 复位刚结束后的快速锁定系数
//      - 因为你说 1-2 次往复后零位就稳定了，所以这里用大系数快速跟上
//
// 5) 置信度的使用：
//    后续做安全控制时，可以在 process_force_feedback() 中
//    根据 force_processor.output.conf_fn_1 来调节力反馈增益。
//    例如：mapped_582_f *= force_processor.output.conf_fn_1;
//    这样复位期间主端力反馈自动减弱，操作者不会感受到尖峰。
