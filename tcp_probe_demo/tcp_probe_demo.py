#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
TCP 采样探针（ch0/ch1 实时波形版）

功能：
1) TCP 采样 6 通道数据（ch0~ch5）
2) 实时显示 ch0/ch1 波形窗口（中文标签）
3) 保存 ch0/ch1 的 CSV
4) 保存 ch0/ch1 的静态波形图
"""

import argparse
import csv
import socket
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import matplotlib

try:
    matplotlib.use("TkAgg")
except Exception:
    # 后端不可切换时保留默认设置。
    pass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager


QUERY_CMD = bytes([0x00, 0x03, 0x00, 0x00, 0x00, 0x06, 0x01, 0x04, 0x00, 0x40, 0x00, 0x08])
RESP_LEN = 25
CHANNEL_COUNT = 6
CHANNEL_OFFSET = 9

# 你提供的校准映射参数
CH0_SCALE = 0.00108104
CH0_OFFSET = -0.31384209
CH1_SCALE = 0.00106976
CH1_OFFSET = -0.63060292

# 样本结构：wall_ts_s, mono_ms, frame_idx, ch0, ch1
SampleRow = Tuple[float, float, int, float, float]
LivePlotState = Dict[str, Any]


def configure_matplotlib_chinese() -> None:
    """配置中文字体和负号显示，尽量避免中文乱码。"""
    candidates = [
        "Microsoft YaHei",
        "SimHei",
        "SimSun",
        "Noto Sans CJK SC",
        "PingFang SC",
        "WenQuanYi Zen Hei",
        "Arial Unicode MS",
    ]
    available = {f.name for f in font_manager.fontManager.ttflist}
    picked = [name for name in candidates if name in available]

    if picked:
        # 将可用中文字体放在首位，保留默认字体作为兜底。
        default_fonts = list(plt.rcParams.get("font.sans-serif", []))
        merged_fonts = picked + [f for f in default_fonts if f not in picked]
        plt.rcParams["font.family"] = "sans-serif"
        plt.rcParams["font.sans-serif"] = merged_fonts
        print(f"[INFO] 中文字体: {picked[0]}")
    else:
        print("[WARN] 未检测到常见中文字体，中文可能显示为方框。")

    plt.rcParams["axes.unicode_minus"] = False


def recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("socket closed by peer")
        data.extend(chunk)
    return bytes(data)


def parse_raw_channels(resp: bytes) -> Optional[List[int]]:
    if len(resp) != RESP_LEN:
        return None

    raw_vals: List[int] = []
    for i in range(CHANNEL_COUNT):
        hi = resp[CHANNEL_OFFSET + i * 2]
        lo = resp[CHANNEL_OFFSET + i * 2 + 1]
        raw = int.from_bytes(bytes([hi, lo]), byteorder="big", signed=True)
        raw_vals.append(raw)
    return raw_vals


def map_raw_channels(raw_vals: List[int]) -> List[float]:
    mapped_vals: List[float] = []
    for idx, raw in enumerate(raw_vals):
        if idx == 0:
            mapped_vals.append(CH0_SCALE * raw + CH0_OFFSET)
        elif idx == 1:
            mapped_vals.append(CH1_SCALE * raw + CH1_OFFSET)
        else:
            # 其它通道沿用原始逻辑（raw/1000）。
            mapped_vals.append(raw / 1000.0)
    return mapped_vals


def init_live_plot(max_points: int = 2000) -> Optional[LivePlotState]:
    backend = matplotlib.get_backend().lower()
    non_interactive_backends = {"agg", "cairo", "pdf", "pgf", "ps", "svg", "template"}
    if backend in non_interactive_backends or "matplotlib_inline" in backend:
        print(f"[WARN] 当前 matplotlib 后端不支持实时窗口（backend={backend}），已跳过实时绘图。")
        return None

    plt.ion()
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

    line_ch0, = axes[0].plot([], [], linewidth=0.9, color="#1f77b4")
    axes[0].set_ylabel("切向力")
    axes[0].grid(alpha=0.3)

    line_ch1, = axes[1].plot([], [], linewidth=0.9, color="#ff7f0e")
    axes[1].set_ylabel("轴向力")
    axes[1].set_xlabel("时间 (ms)")
    axes[1].grid(alpha=0.3)

    fig.suptitle("实时波形：切向力 与 轴向力")
    fig.tight_layout()

    try:
        fig.canvas.manager.set_window_title("TCP 实时波形")
    except Exception:
        pass

    plt.show(block=False)
    return {
        "fig": fig,
        "axes": axes,
        "line_ch0": line_ch0,
        "line_ch1": line_ch1,
        "t": deque(maxlen=max_points),
        "ch0": deque(maxlen=max_points),
        "ch1": deque(maxlen=max_points),
        "lock": threading.Lock(),
        "disabled": False,
    }


def append_live_point(state: Optional[LivePlotState], t_ms: float, ch0: float, ch1: float) -> None:
    if state is None or state["disabled"]:
        return
    with state["lock"]:
        state["t"].append(t_ms)
        state["ch0"].append(ch0)
        state["ch1"].append(ch1)


def redraw_live_plot(state: LivePlotState) -> None:
    with state["lock"]:
        t_vals = list(state["t"])
        ch0_vals = list(state["ch0"])
        ch1_vals = list(state["ch1"])

    if not t_vals:
        return

    state["line_ch0"].set_data(t_vals, ch0_vals)
    state["line_ch1"].set_data(t_vals, ch1_vals)

    x0 = t_vals[0]
    x1 = t_vals[-1]
    if x1 <= x0:
        x1 = x0 + 1.0

    for ax in state["axes"]:
        ax.set_xlim(x0, x1)
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)

    state["fig"].canvas.draw_idle()
    state["fig"].canvas.flush_events()


def drive_live_plot(state: Optional[LivePlotState], done_event: threading.Event, fps: float) -> None:
    if state is None:
        done_event.wait()
        return

    interval_s = 1.0 / max(fps, 1.0)
    fig = state["fig"]

    while not done_event.is_set():
        if not plt.fignum_exists(fig.number):
            state["disabled"] = True
            print("[WARN] 实时窗口已关闭，采集继续进行，不再刷新图像。")
            done_event.wait()
            return
        redraw_live_plot(state)
        plt.pause(interval_s)

    if not state["disabled"] and plt.fignum_exists(fig.number):
        redraw_live_plot(state)
        plt.pause(0.05)
    plt.ioff()


def finalize_live_plot(state: Optional[LivePlotState]) -> None:
    if state is None:
        return
    fig = state["fig"]
    if plt.fignum_exists(fig.number):
        plt.close(fig)


def open_socket(ip: str, port: int, timeout_s: float) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout_s)
    # 关闭 Nagle，减少小包发送时的等待。
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except OSError:
        pass
    sock.connect((ip, port))
    return sock


def collect_samples(
    ip: str,
    port: int,
    seconds: float,
    timeout_ms: int,
    live_plot: Optional[LivePlotState] = None,
) -> Tuple[List[SampleRow], Dict[str, int]]:
    timeout_s = max(timeout_ms, 1) / 1000.0
    end_t = time.perf_counter() + max(seconds, 0.1)

    samples: List[SampleRow] = []
    counters = {
        "connect_fail": 0,
        "io_timeout": 0,
        "io_error": 0,
        "bad_length": 0,
    }

    frame_idx = 0
    sock: Optional[socket.socket] = None
    start_mono_ms: Optional[float] = None

    try:
        while time.perf_counter() < end_t:
            if sock is None:
                try:
                    sock = open_socket(ip, port, timeout_s)
                except OSError:
                    counters["connect_fail"] += 1
                    time.sleep(0.02)
                    continue

            try:
                sock.sendall(QUERY_CMD)
                resp = recv_exact(sock, RESP_LEN)
            except socket.timeout:
                counters["io_timeout"] += 1
                continue
            except (OSError, ConnectionError):
                counters["io_error"] += 1
                try:
                    sock.close()
                except OSError:
                    pass
                sock = None
                continue

            raw_vals = parse_raw_channels(resp)
            if raw_vals is None:
                counters["bad_length"] += 1
                continue

            mapped_vals = map_raw_channels(raw_vals)

            wall_ts_s = time.time()
            mono_ms = time.perf_counter() * 1000.0
            if start_mono_ms is None:
                start_mono_ms = mono_ms

            ch0 = mapped_vals[0]
            ch1 = mapped_vals[1]
            samples.append((wall_ts_s, mono_ms, frame_idx, ch0, ch1))

            append_live_point(live_plot, mono_ms - start_mono_ms, ch0, ch1)
            frame_idx += 1
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    return samples, counters


def run_collection_worker(
    ip: str,
    port: int,
    seconds: float,
    timeout_ms: int,
    live_plot: Optional[LivePlotState],
    done_event: threading.Event,
    result_holder: Dict[str, Any],
) -> None:
    try:
        samples, counters = collect_samples(
            ip=ip,
            port=port,
            seconds=seconds,
            timeout_ms=timeout_ms,
            live_plot=live_plot,
        )
        result_holder["samples"] = samples
        result_holder["counters"] = counters
    finally:
        done_event.set()


def write_csv(csv_path: Path, samples: List[SampleRow]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow([
            "wall_time_iso",
            "mono_ms",
            "frame_idx",
            "切向力",
            "轴向力",
        ])
        for wall_ts_s, mono_ms, frame_idx, ch0, ch1 in samples:
            wall_iso = datetime.fromtimestamp(wall_ts_s).isoformat(timespec="milliseconds")
            writer.writerow([wall_iso, mono_ms, frame_idx, ch0, ch1])


def save_plot(run_dir: Path, mono_ms: np.ndarray, ch0: np.ndarray, ch1: np.ndarray) -> Optional[Path]:
    if mono_ms.size == 0 or ch0.size == 0 or ch1.size == 0:
        return None

    t = mono_ms - mono_ms[0]
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

    axes[0].plot(t, ch0, linewidth=0.9, color="#1f77b4")
    axes[0].set_ylabel("切向力")
    axes[0].grid(alpha=0.3)

    axes[1].plot(t, ch1, linewidth=0.9, color="#ff7f0e")
    axes[1].set_ylabel("轴向力")
    axes[1].set_xlabel("时间 (ms)")
    axes[1].grid(alpha=0.3)

    fig.suptitle("波形图：切向力 与 轴向力")
    fig.tight_layout()
    plot_path = run_dir / "wave_ch0_ch1.png"
    fig.savefig(plot_path, dpi=120)
    plt.close(fig)
    return plot_path


def estimate_hz(samples: List[SampleRow]) -> float:
    if len(samples) < 2:
        return float("nan")
    total_s = (samples[-1][1] - samples[0][1]) / 1000.0
    if total_s <= 1e-12:
        return float("nan")
    return (len(samples) - 1) / total_s


def main() -> int:
    parser = argparse.ArgumentParser(description="TCP 采样探针（ch0/ch1 实时波形版）")
    parser.add_argument("--ip", default="192.168.1.30", help="采集卡 IP")
    parser.add_argument("--port", type=int, default=502, help="采集卡端口")
    parser.add_argument("--seconds", type=float, default=120.0, help="采集时长（秒）")
    parser.add_argument("--outdir", default="", help="输出目录（默认脚本目录/out）")
    parser.add_argument("--timeout-ms", type=int, default=300, help="socket 超时（毫秒）")
    parser.add_argument("--live-plot", dest="live_plot", action="store_true", help="开启实时波形窗口（默认开启）")
    parser.add_argument("--no-live-plot", dest="live_plot", action="store_false", help="关闭实时波形窗口")
    parser.add_argument("--max-live-points", type=int, default=2000, help="实时窗口最多显示点数")
    parser.add_argument("--live-fps", type=float, default=20.0, help="实时窗口刷新帧率（越高越流畅，默认 20）")
    parser.set_defaults(live_plot=True)
    args = parser.parse_args()

    configure_matplotlib_chinese()

    script_dir = Path(__file__).resolve().parent
    base_out = Path(args.outdir).resolve() if args.outdir else (script_dir / "out")
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = base_out / f"run_{run_id}"
    run_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] target={args.ip}:{args.port}, seconds={args.seconds}, timeout_ms={args.timeout_ms}")
    print(f"[INFO] output_dir={run_dir}")

    live_plot = init_live_plot(max_points=args.max_live_points) if args.live_plot else None
    done_event = threading.Event()
    result_holder: Dict[str, Any] = {}

    worker = threading.Thread(
        target=run_collection_worker,
        kwargs={
            "ip": args.ip,
            "port": args.port,
            "seconds": args.seconds,
            "timeout_ms": args.timeout_ms,
            "live_plot": live_plot,
            "done_event": done_event,
            "result_holder": result_holder,
        },
        daemon=True,
    )
    worker.start()

    drive_live_plot(live_plot, done_event=done_event, fps=args.live_fps)
    worker.join()
    finalize_live_plot(live_plot)

    samples = result_holder.get("samples", [])
    counters = result_holder.get(
        "counters",
        {"connect_fail": 0, "io_timeout": 0, "io_error": 0, "bad_length": 0},
    )

    csv_path = run_dir / f"tcp_probe_ch0_ch1_{run_id}.csv"
    write_csv(csv_path, samples)

    if samples:
        mono_ms = np.array([row[1] for row in samples], dtype=np.float64)
        ch0 = np.array([row[3] for row in samples], dtype=np.float64)
        ch1 = np.array([row[4] for row in samples], dtype=np.float64)
    else:
        mono_ms = np.array([], dtype=np.float64)
        ch0 = np.array([], dtype=np.float64)
        ch1 = np.array([], dtype=np.float64)

    plot_path = save_plot(run_dir, mono_ms, ch0, ch1)
    hz = estimate_hz(samples)

    print("[INFO] 采集完成")
    print(f"[INFO] samples={len(samples)}")
    if np.isfinite(hz):
        print(f"[INFO] effective_hz={hz:.2f}")
    else:
        print("[INFO] effective_hz=nan")
    print(
        f"[INFO] connect_fail={counters['connect_fail']}, "
        f"io_timeout={counters['io_timeout']}, "
        f"io_error={counters['io_error']}, "
        f"bad_length={counters['bad_length']}"
    )
    print(f"[INFO] csv={csv_path}")
    if plot_path is not None:
        print(f"[INFO] plot={plot_path}")
    else:
        print("[INFO] plot=无有效样本，未生成图像")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[INFO] interrupted by user")
        raise SystemExit(130)
