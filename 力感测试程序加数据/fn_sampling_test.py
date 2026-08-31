"""
实时力传感器高速采样与波形监控工具
适配下位机工程程序标准变量：
  - G.ft_1_value (导管侧向/扭矩通道)
  - G.fn_1_value (导管轴向力通道)
  - G.fn_2_value (导丝轴向力通道)
  - G.ft_2_value (导丝侧向/扭矩通道)
"""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import time
from typing import List, Optional, Tuple

import numpy as np

try:
    import pyads
except ImportError:
    pyads = None

PLC_PORT = 851
NET_IDS = ["169.254.119.135.1.1", "127.0.0.1.1.1"]

SENSOR_CHANNELS = {
    1: {'var_name': 'G.ft_1_value', 'desc': '传感器 1 (导管从端侧向/扭矩)'},
    2: {'var_name': 'G.fn_1_value', 'desc': '传感器 2 (导管从端轴向推拉力)'},
    3: {'var_name': 'G.fn_2_value', 'desc': '传感器 3 (导丝主端轴向推拉力)'},
    4: {'var_name': 'G.ft_2_value', 'desc': '传感器 4 (导丝主端侧向/扭矩)'}
}


@dataclass
class Sample:
    t_s: float
    t_wall: str
    value_raw: Optional[int]
    read_ok: bool
    ads_error: str


class LivePlot:
    def __init__(self, duration_s: float, var_name: str):
        import matplotlib.pyplot as plt

        self._plt = plt
        self._fig, self._ax = plt.subplots(figsize=(10, 4.5), dpi=120)
        (self._line,) = self._ax.plot([], [], linewidth=1.2, color='#0D9488', label=var_name)
        self._time_text = self._ax.text(
            0.98,
            0.98,
            "0.000 s",
            transform=self._ax.transAxes,
            ha="right",
            va="top",
            fontsize=10,
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8)
        )

        self._ax.set_title(f"{var_name} 实时读数波形", fontsize=12, fontweight='bold')
        self._ax.set_xlabel("时间 (s)", fontsize=10)
        self._ax.set_ylabel("传感器原始计数 (count)", fontsize=10)
        self._ax.grid(True, alpha=0.3)
        self._ax.legend(loc="lower left")
        self._ax.set_xlim(0.0, float(duration_s))
        plt.tight_layout()
        plt.show(block=False)

    def is_open(self) -> bool:
        return self._plt.fignum_exists(self._fig.number)

    def update(self, t: np.ndarray, v: np.ndarray, t_now: float) -> None:
        if t.size == 0:
            return
        self._line.set_data(t, v)
        if t_now > self._ax.get_xlim()[1]:
            self._ax.set_xlim(0.0, max(t_now, float(t[-1])))
        if v.size > 0:
            v_min = float(np.min(v))
            v_max = float(np.max(v))
            if v_min == v_max:
                v_min -= 10.0
                v_max += 10.0
            margin = max(10.0, (v_max - v_min) * 0.15)
            self._ax.set_ylim(v_min - margin, v_max + margin)
        self._time_text.set_text(f"{t_now:.3f} s")
        self._fig.canvas.draw_idle()
        self._plt.pause(0.001)

    def close(self) -> None:
        try:
            self._plt.close(self._fig)
        except Exception:
            pass


def connect_plc(net_id: Optional[str] = None):
    if pyads is None:
        raise RuntimeError("未检测到 pyads 模块。请在控制台执行 `pip install pyads` 后再运行此脚本。")

    candidates = [net_id] if net_id else NET_IDS
    for candidate in candidates:
        if not candidate:
            continue
        plc = None
        try:
            print(f"正在连接 PLC: {candidate}:{PLC_PORT} ...")
            plc = pyads.Connection(candidate, PLC_PORT)
            plc.open()
            state = plc.read_state()
            print(f"-> ADS 连接成功！NetID: {candidate} (状态: {state})")
            return plc
        except Exception as e:
            print(f"-> 连接 {candidate} 失败: {e}")
            if plc:
                try:
                    plc.close()
                except Exception:
                    pass
    raise RuntimeError("无法建立与 PLC 的 ADS 连接。请检查 TwinCAT 路由配置。")


def run_sampling(var_name: str, duration_s: float = 10.0, live_plot: bool = True, net_id: Optional[str] = None):
    plc = connect_plc(net_id)
    plot = LivePlot(duration_s, var_name) if live_plot else None

    samples: List[Sample] = []
    t0 = time.perf_counter()
    last_plot_time = 0.0

    print(f"\n开始高速采样: {var_name} (时长: {duration_s:.1f} s)... 按 Ctrl+C 提前停止。")
    try:
        while True:
            t_now = time.perf_counter() - t0
            if t_now >= duration_s:
                break
            wall = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            try:
                val = plc.read_by_name(var_name, pyads.PLCTYPE_INT)
                samples.append(Sample(t_s=t_now, t_wall=wall, value_raw=val, read_ok=True, ads_error=""))
            except Exception as e:
                samples.append(Sample(t_s=t_now, t_wall=wall, value_raw=None, read_ok=False, ads_error=str(e)))

            if plot is not None and (t_now - last_plot_time) >= 0.03:
                last_plot_time = t_now
                ok_samples = [s for s in samples if s.read_ok and s.value_raw is not None]
                if ok_samples:
                    ts = np.array([s.t_s for s in ok_samples])
                    vs = np.array([s.value_raw for s in ok_samples])
                    plot.update(ts, vs, t_now)

    except KeyboardInterrupt:
        print("\n采样被用户中断。")
    finally:
        if plc:
            try:
                plc.close()
            except Exception:
                pass
        if plot:
            plot.close()

    # 统计分析
    ok_samples = [s for s in samples if s.read_ok and s.value_raw is not None]
    print("\n" + "=" * 60)
    print("采样统计结果")
    print("=" * 60)
    print(f"总采样点数:   {len(samples)}")
    print(f"有效样本数:   {len(ok_samples)} ({len(ok_samples)/max(1,len(samples))*100:.1f}%)")
    actual_dur = samples[-1].t_s if samples else 0.0
    print(f"实际采样时长: {actual_dur:.3f} s")
    if actual_dur > 0:
        print(f"实际采样频率: {len(samples)/actual_dur:.1f} Hz")

    if ok_samples:
        vs = np.array([s.value_raw for s in ok_samples])
        print(f"原始读数均值: {np.mean(vs):.2f} counts")
        print(f"标准差 (噪声): {np.std(vs):.2f} counts")
        print(f"最小值 / 最大值: {np.min(vs)} / {np.max(vs)} counts")
        print(f"跨度 (Span):  {np.max(vs) - np.min(vs)} counts")

    # 保存 CSV
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_name = f"fn_test_{var_name.replace('G.', '')}_{timestamp}.csv"
    with open(out_name, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(["time_s", "timestamp", "value_raw", "read_ok", "error"])
        for s in samples:
            w.writerow([f"{s.t_s:.6f}", s.t_wall, s.value_raw if s.value_raw is not None else "", s.read_ok, s.ads_error])
    print(f"\n详细采样记录已保存至: {out_name}")


def main():
    parser = argparse.ArgumentParser(description="TwinCAT ADS 传感器高速采样与监控")
    parser.add_argument("--channel", type=int, choices=[1, 2, 3, 4], default=3, help="传感器通道编号 (1:ft1, 2:fn1, 3:fn2, 4:ft2, 默认 3)")
    parser.add_argument("--duration", type=float, default=10.0, help="采样时长 (秒，默认 10.0)")
    parser.add_argument("--no-plot", action="store_true", help="禁用实时绘图")
    parser.add_argument("--net-id", type=str, default=None, help="TwinCAT AMS Net ID")
    args = parser.parse_args()

    var_name = SENSOR_CHANNELS[args.channel]['var_name']
    run_sampling(var_name, duration_s=args.duration, live_plot=not args.no_plot, net_id=args.net_id)


if __name__ == '__main__':
    main()
