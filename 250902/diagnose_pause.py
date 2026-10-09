#!/usr/bin/env python3
"""诊断前向暂停功能是否触发"""
import pyads

# 连接PLC
plc = pyads.Connection('5.135.207.190.1.1', pyads.PORT_TC3PLC1)
plc.open()

print("监控前向暂停相关变量（按Ctrl+C退出）\n")

try:
    while True:
        # 读取关键变量
        enabled = plc.read_by_name('G.program_test_forward_pause_enabled', pyads.PLCTYPE_BOOL)
        distance = plc.read_by_name('G.program_test_forward_pause_distance_mm', pyads.PLCTYPE_LREAL)
        duration = plc.read_by_name('G.program_test_forward_pause_duration_ms', pyads.PLCTYPE_UDINT)
        phase = plc.read_by_name('G.program_test_phase', pyads.PLCTYPE_USINT)
        wait_action = plc.read_by_name('G.program_test_wait_action', pyads.PLCTYPE_USINT)

        # 读取轴位置
        axis1_pos = plc.read_by_name('G.axis[1].NcToPlc.ActPos', pyads.PLCTYPE_LREAL)
        axis6_pos = plc.read_by_name('G.axis[6].NcToPlc.ActPos', pyads.PLCTYPE_LREAL)

        # 读取目标位置
        trigger_target = plc.read_by_name('G.program_test_trigger_target_abs', pyads.PLCTYPE_LREAL)

        # 计算暂停触发位置
        pause_position = trigger_target - distance

        print(f"\r启用:{enabled} | 距离:{distance}mm | 时长:{duration}ms | "
              f"阶段:{phase} | 等待动作:{wait_action} | "
              f"轴1:{axis1_pos:.2f} | 轴6:{axis6_pos:.2f} | "
              f"触发位:{trigger_target:.2f} | 暂停位:{pause_position:.2f}    ", end='')

        import time
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\n监控结束")
finally:
    plc.close()
