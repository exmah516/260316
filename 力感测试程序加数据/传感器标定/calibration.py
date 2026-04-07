import pyads
import time
import numpy as np
import csv
from datetime import datetime

# 配置
PLC_PORT = 851
VAR_NAME = "G.fn_value"
GRAVITY = 9.80665  # m/s^2

# 标准本地回环 AMS Net ID
# 如果失败，我们将尝试在 C++ 代码中找到的那个: '169.254.119.135.1.1'
LOCAL_NET_ID = '127.0.0.1.1.1' 
FALLBACK_NET_ID = '169.254.119.135.1.1'

def connect_to_plc():
    """尝试连接到 PLC。"""
    print(f"正在连接到 PLC (端口 {PLC_PORT})...")
    
    # 首先尝试本地回环
    try:
        plc = pyads.Connection(LOCAL_NET_ID, PLC_PORT)
        plc.open()
        # 读取状态以验证连接
        plc.read_state()
        print(f"已连接，使用 NetID: {LOCAL_NET_ID}")
        return plc
    except Exception as e:
        print(f"连接到 {LOCAL_NET_ID} 失败: {e}")
    
    # 尝试备用 NetID
    try:
        print(f"正在尝试备用 NetID: {FALLBACK_NET_ID}...")
        plc = pyads.Connection(FALLBACK_NET_ID, PLC_PORT)
        plc.open()
        plc.read_state()
        print(f"已连接，使用 NetID: {FALLBACK_NET_ID}")
        return plc
    except Exception as e:
        print(f"连接到 {FALLBACK_NET_ID} 失败: {e}")
    
    return None

def main():
    # 1. 连接到 PLC
    plc = connect_to_plc()
    if not plc:
        print("无法连接到 PLC。请检查 TwinCAT 是否正在运行且路由已配置。")
        return

    # 2. 定义标定点
    # 添加 0g 作为基准
    weights_g = [0, 1, 2, 5, 10, 20, 50, 100, 200]
    
    sensor_data = []  # 存储平均传感器值
    force_data = []   # 存储计算出的力 (N)
    calibration_stats = [] # 存储详细统计信息

    print("\n" + "="*50)
    print("力传感器标定助手")
    print(f"待标定变量: {VAR_NAME}")
    print("="*50 + "\n")

    try:
        # 3. 测量循环
        for weight in weights_g:
            input(f">> 请放置 {weight}g 砝码 (或 0 表示空载) 并按回车键开始测量...")
            
            print(f"   正在测量 {weight}g (持续 5 秒)...")
            print("   实时读数: ", end="", flush=True)
            
            # 收集数据约 5 秒
            samples = []
            start_time = time.time()
            duration = 5.0 # 秒
            
            while time.time() - start_time < duration:
                try:
                    # 从 PLC 读取 INT 值
                    val = plc.read_by_name(VAR_NAME, pyads.PLCTYPE_INT)
                    samples.append(val)
                    print(f"{val} ", end="", flush=True) # 实时显示数值
                except pyads.ADSError as e:
                    print(f"\n   读取 {VAR_NAME} 出错: {e}")
                time.sleep(0.05) # 20Hz 采样率
            
            print() # 换行
            
            if not samples:
                print("\n   未收集到数据！跳过此点。")
                continue
            
            # 数据处理：剔除异常值
            # 1. 计算中位数
            median_val = np.median(samples)
            
            # 2. 排除与中位数绝对差值超过 300 的数值
            valid_samples = [x for x in samples if abs(x - median_val) <= 300]
            
            # 统计剔除情况
            total_count = len(samples)
            valid_count = len(valid_samples)
            removed_count = total_count - valid_count
            
            if valid_count == 0:
                print(f"   警告: 所有数据均被视为异常值剔除！(中位数: {median_val})")
                print("   跳过此点。")
                continue

            avg_val = np.mean(valid_samples)
            std_val = np.std(valid_samples)
            
            # 计算力: F = m * g
            force_n = (weight / 1000.0) * GRAVITY
            
            sensor_data.append(avg_val)
            force_data.append(force_n)
            
            calibration_stats.append({
                'weight': weight,
                'total_count': total_count,
                'valid_count': valid_count,
                'removed_count': removed_count,
                'avg_val': avg_val,
                'std_val': std_val,
                'force_n': force_n
            })
            
            print(f"   采集完成。原始数据: {total_count} 个, 有效数据: {valid_count} 个 (剔除 {removed_count} 个)")
            print(f"   平均值: {avg_val:.2f} (标准差: {std_val:.2f}), 对应力: {force_n:.4f} N")
            
    except KeyboardInterrupt:
        print("\n\n用户中断标定。")
    finally:
        plc.close()

    # 4. 数据分析与拟合
    if len(sensor_data) < 2:
        print("\n标定数据点不足。")
        return

    print("\n" + "-"*50)
    print("正在计算标定参数...")
    
    x = np.array(sensor_data) # 传感器数值
    y = np.array(force_data)  # 力 (N)
    
    # 线性拟合: Force = k * Sensor + b
    # deg=1 表示线性
    slope, intercept = np.polyfit(x, y, 1)
    
    print("-" * 50)
    print("标定结果:")
    print(f"斜率 (k):     {slope:.8f}  (N / 单位)")
    print(f"截距 (b):     {intercept:.8f}  (N)")
    print("-" * 50)
    print(f"公式: Force (N) = {slope:.6e} * {VAR_NAME} + ({intercept:.6e})")
    print("-" * 50)
    
    # 5. 验证表
    print("\n" + "="*90)
    print("标定数据汇总与验证表")
    print("="*90)
    header = f"{'重量(g)':<8} | {'总数':<6} | {'有效':<6} | {'传感器均值':<12} | {'标准差':<10} | {'实际力(N)':<10} | {'预测力(N)':<10} | {'误差(N)':<10}"
    print(header)
    print("-" * len(header))

    for stat in calibration_stats:
        weight = stat['weight']
        s_val = stat['avg_val']
        act_f = stat['force_n']
        pred_f = slope * s_val + intercept
        error = pred_f - act_f
        
        print(f"{weight:<8} | {stat['total_count']:<6} | {stat['valid_count']:<6} | {s_val:<12.2f} | {stat['std_val']:<10.2f} | {act_f:<10.4f} | {pred_f:<10.4f} | {error:<10.4f}")
    print("-" * len(header))

    print("\n在代码中使用:")
    print(f"force_newton = {slope:.8f} * {VAR_NAME} + {intercept:.8f}")

    # 6. 保存到文件
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"calibration_result_{timestamp}.csv"
    
    print(f"\n正在保存结果到文件: {filename} ...")
    
    try:
        with open(filename, 'w', newline='', encoding='utf-8-sig') as f: # utf-8-sig 方便 Excel 打开中文不乱码
            writer = csv.writer(f)
            
            # 写入元数据和公式
            writer.writerow(["标定时间", timestamp])
            writer.writerow(["变量名", VAR_NAME])
            writer.writerow(["拟合公式", f"Force (N) = {slope:.8f} * {VAR_NAME} + {intercept:.8f}"])
            writer.writerow(["斜率 (k)", slope])
            writer.writerow(["截距 (b)", intercept])
            writer.writerow([]) # 空行分隔
            
            # 写入表头
            header_csv = ['重量(g)', '总样本数', '有效样本数', '传感器均值', '标准差', '实际力(N)', '预测力(N)', '误差(N)']
            writer.writerow(header_csv)
            
            # 写入数据行
            for stat in calibration_stats:
                weight = stat['weight']
                s_val = stat['avg_val']
                act_f = stat['force_n']
                pred_f = slope * s_val + intercept
                error = pred_f - act_f
                
                writer.writerow([
                    weight,
                    stat['total_count'],
                    stat['valid_count'],
                    f"{s_val:.2f}",
                    f"{stat['std_val']:.2f}",
                    f"{act_f:.4f}",
                    f"{pred_f:.4f}",
                    f"{error:.4f}"
                ])
                
        print(f"保存成功！文件位于: {filename}")
        
    except Exception as e:
        print(f"保存文件失败: {e}")

if __name__ == "__main__":
    main()
