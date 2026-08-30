"""
力传感器标定助手 (适配当前下位机工程程序 G.TcGVL 变量规范)
支持通道:
  - 传感器 1 / G.ft_1_value (导管侧向/扭矩通道)
  - 传感器 2 / G.fn_1_value (导管轴向力通道)
  - 传感器 3 / G.fn_2_value (导丝轴向力通道)
  - 传感器 4 / G.ft_2_value (导丝侧向/扭矩通道)
  - 四通道同步采集与串扰监测模式
"""

import argparse
import csv
from datetime import datetime
import os
import sys
import time
import numpy as np

try:
    import pyads
except ImportError:
    pyads = None

# ==================== 下位机 ADS 接口标准配置 ====================
PLC_PORT = 851
GRAVITY = 9.80665  # m/s^2

# 标准 AMS Net ID (优先尝试本地回环，失败时自动尝试实机 Net ID)
LOCAL_NET_ID = '127.0.0.1.1.1'
FALLBACK_NET_ID = '169.254.119.135.1.1'

# 下位机工程程序中的 4 路力传感器标准变量字典
SENSOR_CHANNELS = {
    1: {
        'id': 1,
        'var_name': 'G.ft_1_value',
        'desc': '传感器 1 / G.ft_1_value (导管从端侧向力/扭矩通道)',
        'file_prefix': '传感器1标定数据'
    },
    2: {
        'id': 2,
        'var_name': 'G.fn_1_value',
        'desc': '传感器 2 / G.fn_1_value (导管从端轴向推拉力通道)',
        'file_prefix': '传感器2标定数据'
    },
    3: {
        'id': 3,
        'var_name': 'G.fn_2_value',
        'desc': '传感器 3 / G.fn_2_value (导丝主端轴向推拉力通道)',
        'file_prefix': '传感器3标定数据'
    },
    4: {
        'id': 4,
        'var_name': 'G.ft_2_value',
        'desc': '传感器 4 / G.ft_2_value (导丝主端侧向力/扭矩通道)',
        'file_prefix': '传感器4标定数据'
    }
}

WEIGHT_PRESETS = {
    '1': ([0, 2, 5, 10, 20, 50], '常用微力量程 (0 ~ 50g，共6点)'),
    '2': ([0, 1, 2, 5, 10, 20, 50, 100, 200], '完整全量程 (0 ~ 200g，共9点)'),
}


def connect_to_plc(net_id: str = None):
    """尝试连接到 TwinCAT PLC。"""
    if pyads is None:
        print("错误: 未检测到 pyads 模块。请在控制台执行 `pip install pyads` 后再运行此脚本。")
        return None

    candidates = [net_id] if net_id else [LOCAL_NET_ID, FALLBACK_NET_ID]
    for target in candidates:
        if not target:
            continue
        try:
            print(f"正在连接 PLC (NetID: {target}, 端口: {PLC_PORT})...")
            plc = pyads.Connection(target, PLC_PORT)
            plc.open()
            plc.read_state()
            print(f"-> ADS 连接成功！NetID: {target}")
            return plc
        except Exception as e:
            print(f"-> 连接到 {target} 失败: {e}")
    return None


def verify_plc_symbols(plc):
    """检查下位机中是否存在 4 路力传感器变量。"""
    print("\n正在核对下位机变量接口...")
    all_ok = True
    for ch_id, ch_info in SENSOR_CHANNELS.items():
        var = ch_info['var_name']
        try:
            val = plc.read_by_name(var, pyads.PLCTYPE_INT)
            print(f"  [OK] {var:<15} (当前原始读数: {val} counts) - {ch_info['desc']}")
        except Exception as e:
            print(f"  [FAIL] {var:<15} 读取失败: {e}")
            all_ok = False
    return all_ok


def select_channel_interactive() -> int:
    """交互式通道选择。"""
    print("\n" + "=" * 60)
    print("请选择需要标定的力传感器通道 (对应当前下位机工程 G.TcGVL)：")
    print("=" * 60)
    for ch_id, info in SENSOR_CHANNELS.items():
        print(f"  [{ch_id}] {info['desc']}")
    print("  [5] 四通道同步采集与串扰监测 (同时采集全部 4 路通道)")
    print("=" * 60)

    while True:
        choice = input("请输入选择编号 (1-5, 默认 3 - 导丝轴向力): ").strip()
        if not choice:
            return 3
        if choice in ['1', '2', '3', '4', '5']:
            return int(choice)
        print("输入无效，请输入 1 到 5 之间的数字。")


def select_weights_interactive():
    """交互式选择标定砝码序列。"""
    print("\n" + "=" * 60)
    print("请选择标定砝码阶梯序列：")
    print("=" * 60)
    print(f"  [1] {WEIGHT_PRESETS['1'][1]}: {WEIGHT_PRESETS['1'][0]}")
    print(f"  [2] {WEIGHT_PRESETS['2'][1]}: {WEIGHT_PRESETS['2'][0]}")
    print("  [3] 自定义输入砝码列表 (以逗号或空格分隔，如 0, 5, 10, 20, 50)")
    print("=" * 60)

    while True:
        choice = input("请输入选择编号 (1-3, 默认 1 - 0~50g): ").strip()
        if not choice or choice == '1':
            return WEIGHT_PRESETS['1'][0]
        if choice == '2':
            return WEIGHT_PRESETS['2'][0]
        if choice == '3':
            custom = input("请输入砝码质量 (g)，例如 '0, 2, 5, 10, 20, 50': ").strip()
            try:
                weights = [float(w.strip()) for w in custom.replace(',', ' ').split() if w.strip()]
                if len(weights) >= 2 and 0 in weights:
                    return sorted(weights)
                elif len(weights) >= 2:
                    return sorted([0.0] + weights)
                print("至少需要 2 个以上的标定点，且建议包含 0g 基准！")
            except Exception as e:
                print(f"输入格式错误: {e}")
        else:
            print("输入无效，请输入 1、2 或 3。")


def measure_point(plc, target_channel: int, weight_g: float, duration_s: float = 5.0, sample_interval_s: float = 0.05):
    """采集单点多通道数据（持续 duration_s 秒，约 20Hz 采样率）。"""
    all_channel_samples = {ch_id: [] for ch_id in SENSOR_CHANNELS.keys()}
    
    start_time = time.time()
    print(f"   正在采集 {weight_g}g 数据 (持续 {duration_s:.1f} 秒)... 实时读数: ", end="", flush=True)

    read_count = 0
    while time.time() - start_time < duration_s:
        try:
            # 读取全部 4 路通道
            for ch_id, ch_info in SENSOR_CHANNELS.items():
                val = plc.read_by_name(ch_info['var_name'], pyads.PLCTYPE_INT)
                all_channel_samples[ch_id].append(val)

            # 回显主目标通道读数
            if target_channel in SENSOR_CHANNELS:
                main_val = all_channel_samples[target_channel][-1]
                if read_count % 4 == 0:
                    print(f"{main_val} ", end="", flush=True)
            elif target_channel == 5:
                # 4 通道同步模式
                if read_count % 5 == 0:
                    print(f"[{all_channel_samples[1][-1]},{all_channel_samples[2][-1]},{all_channel_samples[3][-1]},{all_channel_samples[4][-1]}] ", end="", flush=True)
            read_count += 1
        except Exception as e:
            print(f"\n   ADS 读取异常: {e}")
        time.sleep(sample_interval_s)

    print()
    actual_dur = time.time() - start_time
    actual_hz = read_count / actual_dur if actual_dur > 0 else 0

    # 对各通道进行离群值剔除处理 (中位数 ± 300 counts)
    point_stats = {'weight_g': weight_g, 'force_n': (weight_g / 1000.0) * GRAVITY, 'actual_hz': actual_hz, 'channels': {}}

    for ch_id, raw_samples in all_channel_samples.items():
        if not raw_samples:
            point_stats['channels'][ch_id] = {
                'total': 0, 'valid': 0, 'mean': 0.0, 'std': 0.0, 'median': 0.0
            }
            continue

        median_val = float(np.median(raw_samples))
        valid_samples = [x for x in raw_samples if abs(x - median_val) <= 300.0]
        if not valid_samples:
            valid_samples = raw_samples

        mean_val = float(np.mean(valid_samples))
        std_val = float(np.std(valid_samples))
        point_stats['channels'][ch_id] = {
            'total': len(raw_samples),
            'valid': len(valid_samples),
            'mean': mean_val,
            'std': std_val,
            'median': median_val
        }

    return point_stats


def run_calibration(channel_choice: int, weights_g: list, net_id: str = None):
    """执行完整标定流程。"""
    plc = connect_to_plc(net_id)
    if not plc:
        print("错误: 无法连接至 PLC，请检查 TwinCAT 运行状态及 ADS 路由配置。")
        return

    # 核对变量
    if not verify_plc_symbols(plc):
        print("\n警告: 部分传感器变量在 PLC 中未找到，请检查 PLC 程序是否已激活！")

    is_all_channels = (channel_choice == 5)
    main_ch = channel_choice if not is_all_channels else 3 # 同步模式默认分析通道3
    main_info = SENSOR_CHANNELS[main_ch]

    print("\n" + "=" * 70)
    print("力传感器多点标定助手")
    if is_all_channels:
        print("标定模式: 四通道同步采集模式")
    else:
        print(f"目标标定通道: {main_info['desc']}")
        print(f"PLC 变量名: {main_info['var_name']}")
    print(f"砝码序列 (g): {weights_g}")
    print("=" * 70 + "\n")

    point_results = []
    try:
        for w in weights_g:
            input(f">> 请放置 {w}g 砝码 (或 0g 空载) 并按 [回车键 Enter] 开始采样...")
            stats = measure_point(plc, channel_choice, w, duration_s=5.0, sample_interval_s=0.05)
            point_results.append(stats)

            m_stat = stats['channels'][main_ch]
            print(f"   -> 采集完成: 样本 {m_stat['valid']}/{m_stat['total']}, 均值: {m_stat['mean']:.2f} count, 波动(std): {m_stat['std']:.2f}, 对应实际力: {stats['force_n']:.4f} N")

    except KeyboardInterrupt:
        print("\n\n标定被用户中断。")
    finally:
        try:
            plc.close()
        except Exception:
            pass

    if len(point_results) < 2:
        print("\n有效标定数据点不足 (< 2)，无法拟合。")
        return

    # 数据分析与线性拟合
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    channels_to_fit = list(SENSOR_CHANNELS.keys()) if is_all_channels else [channel_choice]

    print("\n" + "=" * 80)
    print("标定拟合结果与误差分析")
    print("=" * 80)

    fit_summaries = {}
    for ch_id in channels_to_fit:
        ch_info = SENSOR_CHANNELS[ch_id]
        x_raw = np.array([pt['channels'][ch_id]['mean'] for pt in point_results])
        y_force = np.array([pt['force_n'] for pt in point_results])

        # 线性拟合: Force(N) = k * Raw + b
        slope, intercept = np.polyfit(x_raw, y_force, 1)
        y_pred = slope * x_raw + intercept
        residuals = y_pred - y_force
        r2 = 1.0 - (np.sum(residuals**2) / np.sum((y_force - np.mean(y_force))**2)) if np.var(y_force) > 0 else 0.0
        rmse = float(np.sqrt(np.mean(residuals**2)))
        max_err = float(np.max(np.abs(residuals)))
        span = float(np.max(x_raw) - np.min(x_raw))

        fit_summaries[ch_id] = {
            'slope': slope,
            'intercept': intercept,
            'r2': r2,
            'rmse': rmse,
            'max_err': max_err,
            'span': span,
            'x_raw': x_raw,
            'y_force': y_force,
            'y_pred': y_pred,
            'residuals': residuals
        }

        print(f"\n--- 通道: {ch_info['desc']} ---")
        print(f"拟合公式: Force (N) = {slope:.8e} * {ch_info['var_name']} + ({intercept:.8e})")
        print(f"斜率 (k):     {slope:.12f} N/count")
        print(f"截距 (b):     {intercept:.8f} N")
        print(f"拟合优度 R^2: {r2:.6f}, RMSE: {rmse*1000:.3f} mN, 最大绝对残差: {max_err*1000:.3f} mN, 计数跨度: {span:.1f} counts")

        print("-" * 75)
        header = f"{'砝码(g)':<8} | {'传感器均值(count)':<18} | {'标准差':<8} | {'实际力(N)':<10} | {'预测力(N)':<10} | {'残差(mN)':<10}"
        print(header)
        print("-" * 75)
        for i, pt in enumerate(point_results):
            w = pt['weight_g']
            raw_m = pt['channels'][ch_id]['mean']
            raw_s = pt['channels'][ch_id]['std']
            act_f = pt['force_n']
            pred_f = y_pred[i]
            res_mN = residuals[i] * 1000.0
            print(f"{w:<8.1f} | {raw_m:<18.2f} | {raw_s:<8.2f} | {act_f:<10.4f} | {pred_f:<10.4f} | {res_mN:<+10.3f}")
        print("-" * 75)

    # 保存结果到 CSV 文件
    csv_filename = f"{main_info['file_prefix']}_{timestamp}.csv" if not is_all_channels else f"四通道标定数据_{timestamp}.csv"
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), csv_filename)

    try:
        with open(save_path, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(["标定生成时间", timestamp])
            writer.writerow(["标定通道", "四通道同步模式" if is_all_channels else main_info['desc']])
            writer.writerow(["PLC接口变量", main_info['var_name'] if not is_all_channels else "G.ft_1/fn_1/fn_2/ft_2"])
            for ch_id in channels_to_fit:
                s = fit_summaries[ch_id]
                writer.writerow([f"拟合公式_{SENSOR_CHANNELS[ch_id]['var_name']}", f"Force (N) = {s['slope']:.12f} * {SENSOR_CHANNELS[ch_id]['var_name']} + {s['intercept']:.8f}"])
                writer.writerow([f"斜率_k_{SENSOR_CHANNELS[ch_id]['var_name']}", f"{s['slope']:.12f}"])
                writer.writerow([f"截距_b_{SENSOR_CHANNELS[ch_id]['var_name']}", f"{s['intercept']:.8f}"])
                writer.writerow([f"R2_{SENSOR_CHANNELS[ch_id]['var_name']}", f"{s['r2']:.6f}"])
            writer.writerow([])

            # 表头
            headers = ['重量(g)', '实际力(N)']
            for ch_id in SENSOR_CHANNELS.keys():
                name = SENSOR_CHANNELS[ch_id]['var_name']
                headers.extend([f'{name}_均值', f'{name}_标准差', f'{name}_有效样本'])
            writer.writerow(headers)

            for pt in point_results:
                row = [pt['weight_g'], f"{pt['force_n']:.6f}"]
                for ch_id in SENSOR_CHANNELS.keys():
                    c_stat = pt['channels'][ch_id]
                    row.extend([f"{c_stat['mean']:.3f}", f"{c_stat['std']:.3f}", c_stat['valid']])
                writer.writerow(row)

        print(f"\n标定数据表已成功保存至:\n  {save_path}")
    except Exception as e:
        print(f"\n保存 CSV 失败: {e}")

    # 打印可在 C++ 代码中使用的常量建议
    print("\n" + "=" * 80)
    print("可在 C++ / 上位机代码中直接复制的标定参数定义：")
    print("=" * 80)
    for ch_id in channels_to_fit:
        s = fit_summaries[ch_id]
        var = SENSOR_CHANNELS[ch_id]['var_name'].replace('G.', '')
        print(f"constexpr double k_{var}_slope = {s['slope']:.12f}; // N / count")


def main():
    parser = argparse.ArgumentParser(description="TwinCAT ADS 力传感器多通道多点标定助手")
    parser.add_argument('--channel', type=int, choices=[1, 2, 3, 4, 5], default=None, help="标定通道 (1:ft1, 2:fn1, 3:fn2, 4:ft2, 5:全部)")
    parser.add_argument('--weights', type=str, default=None, help="标定砝码列表，以逗号分隔，如 '0,2,5,10,20,50'")
    parser.add_argument('--net-id', type=str, default=None, help="TwinCAT AMS Net ID，默认尝试本地和实机")
    args = parser.parse_args()

    if args.channel is None:
        ch = select_channel_interactive()
    else:
        ch = args.channel

    if args.weights is None:
        weights = select_weights_interactive()
    else:
        weights = sorted([float(w.strip()) for w in args.weights.split(',') if w.strip()])
        if 0.0 not in weights:
            weights = [0.0] + weights

    run_calibration(ch, weights, net_id=args.net_id)


if __name__ == '__main__':
    main()
