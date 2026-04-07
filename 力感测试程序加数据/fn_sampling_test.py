import csv
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import pyads


PLC_PORT = 851
VAR_NAME = "G.ft_value"
DURATION_S = 10.0
SLEEP_S = 0.0
LIVE_PLOT = True
PLOT_UPDATE_MIN_INTERVAL_S = 0.02

NET_IDS = ["127.0.0.1.1.1", "169.254.119.135.1.1"]


@dataclass
class Sample:
    t_s: float
    t_wall: str
    value_raw: Optional[int]
    read_ok: bool
    ads_error: str


class LivePlot:
    def __init__(self, duration_s: float):
        import matplotlib.pyplot as plt

        self._plt = plt
        self._fig, self._ax = plt.subplots(figsize=(10, 4), dpi=120)
        (self._line,) = self._ax.plot([], [], linewidth=1.0, label=VAR_NAME)
        self._time_text = self._ax.text(
            0.98,
            0.98,
            "0.000 s",
            transform=self._ax.transAxes,
            ha="right",
            va="top",
        )

        self._ax.set_title(f"{VAR_NAME} realtime")
        self._ax.set_xlabel("time (s)")
        self._ax.set_ylabel("sensor value (raw)")
        self._ax.grid(True, alpha=0.25)
        self._ax.legend(loc="lower left")
        self._ax.set_xlim(0.0, float(duration_s))
        plt.tight_layout()
        plt.show(block=False)

    def is_open(self) -> bool:
        return self._plt.fignum_exists(self._fig.number)

    def update(self, t: np.ndarray, v: np.ndarray, t_now: float) -> None:
        if t.size == 0:
            return
        ok = np.isfinite(v)
        self._line.set_data(t[ok], v[ok])
        self._time_text.set_text(f"{t_now:.3f} s")

        if np.any(ok):
            v_ok = v[ok]
            vmin = float(np.min(v_ok))
            vmax = float(np.max(v_ok))
            if np.isfinite(vmin) and np.isfinite(vmax):
                if abs(vmax - vmin) < 1e-9:
                    pad = 1.0
                else:
                    pad = (vmax - vmin) * 0.08
                self._ax.set_ylim(vmin - pad, vmax + pad)

        self._fig.canvas.draw_idle()
        self._plt.pause(0.001)

    def close(self) -> None:
        try:
            self._plt.close(self._fig)
        except Exception:
            pass


def _try_connect(net_id: str) -> Optional[pyads.Connection]:
    try:
        plc = pyads.Connection(net_id, PLC_PORT)
        plc.open()
        plc.read_state()
        return plc
    except Exception:
        try:
            plc.close()
        except Exception:
            pass
        return None


def connect_to_plc() -> Tuple[pyads.Connection, str]:
    last_err = None
    for net_id in NET_IDS:
        plc = _try_connect(net_id)
        if plc is not None:
            return plc, net_id
        last_err = f"Failed to connect using NetID={net_id}"
    raise RuntimeError(last_err or "Failed to connect")


def read_fn_value(plc: pyads.Connection) -> Tuple[float, str]:
    try:
        val = plc.read_by_name(VAR_NAME, pyads.PLCTYPE_INT)
        return int(val), ""
    except Exception as e:
        return None, str(e)


def compute_rates(t_s: np.ndarray) -> dict:
    if t_s.size < 2:
        return {"duration_s": float(t_s[-1]) if t_s.size == 1 else 0.0, "n": int(t_s.size)}
    duration_s = float(t_s[-1] - t_s[0])
    dt = np.diff(t_s)
    dt = dt[np.isfinite(dt) & (dt > 0)]
    if dt.size == 0 or duration_s <= 0:
        return {"duration_s": duration_s, "n": int(t_s.size)}
    return {
        "duration_s": duration_s,
        "n": int(t_s.size),
        "rate_hz_by_total": float((t_s.size - 1) / duration_s),
        "rate_hz_by_mean_dt": float(1.0 / float(np.mean(dt))),
        "rate_hz_by_median_dt": float(1.0 / float(np.median(dt))),
        "dt_ms_mean": float(np.mean(dt) * 1000.0),
        "dt_ms_median": float(np.median(dt) * 1000.0),
        "dt_ms_min": float(np.min(dt) * 1000.0),
        "dt_ms_max": float(np.max(dt) * 1000.0),
    }


def save_csv_detail(path: Path, samples: List[Sample]) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["idx", "t_s", "t_ms", "t_wall", "value_raw", "read_ok", "ads_error"])
        for i, s in enumerate(samples):
            w.writerow(
                [
                    i,
                    f"{s.t_s:.6f}",
                    int(round(s.t_s * 1000.0)),
                    s.t_wall,
                    "" if s.value_raw is None else s.value_raw,
                    1 if s.read_ok else 0,
                    s.ads_error,
                ]
            )


def save_csv_report(path: Path, report: dict) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["key", "value"])
        for k, v in report.items():
            w.writerow([k, v])


def save_plot(path: Path, samples: List[Sample]) -> None:
    import matplotlib.pyplot as plt

    t = np.array([s.t_s for s in samples], dtype=float)
    v = np.array([float("nan") if s.value_raw is None else float(s.value_raw) for s in samples], dtype=float)
    ok = np.isfinite(v)

    plt.figure(figsize=(12, 5), dpi=120)
    if np.any(ok):
        plt.plot(t[ok], v[ok], linewidth=1.0, label=VAR_NAME)
    plt.title(f"{VAR_NAME} raw value over time")
    plt.xlabel("time (s)")
    plt.ylabel("sensor value (raw)")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path)
    plt.close()


def main() -> None:
    print(f"Connecting to PLC (Port {PLC_PORT})...")
    plc, net_id = connect_to_plc()
    print(f"Connected. NetID={net_id}  Var={VAR_NAME}")

    try:
        input("按回车开始测试（10秒）...")
        start_wall = datetime.now()
        t0 = time.perf_counter()
        samples: List[Sample] = []
        live = LivePlot(DURATION_S) if LIVE_PLOT else None
        last_plot_update_t = -1.0

        while True:
            now = time.perf_counter()
            t_rel = now - t0
            if t_rel >= DURATION_S:
                break

            t_wall = datetime.now().isoformat(timespec="milliseconds")
            val_raw, err = read_fn_value(plc)
            read_ok = (val_raw is not None) and (err == "")
            samples.append(Sample(t_s=float(t_rel), t_wall=t_wall, value_raw=val_raw, read_ok=bool(read_ok), ads_error=err))

            if live is not None:
                if not live.is_open():
                    live = None
                else:
                    if (last_plot_update_t < 0) or ((t_rel - last_plot_update_t) >= PLOT_UPDATE_MIN_INTERVAL_S):
                        ts = np.array([s.t_s for s in samples], dtype=float)
                        vs = np.array([float("nan") if s.value_raw is None else float(s.value_raw) for s in samples], dtype=float)
                        live.update(ts, vs, float(t_rel))
                        last_plot_update_t = float(t_rel)

            if SLEEP_S > 0:
                time.sleep(SLEEP_S)
        end_wall = datetime.now()

    finally:
        try:
            plc.close()
        except Exception:
            pass
        try:
            if "live" in locals() and live is not None:
                live.close()
        except Exception:
            pass

    if not samples:
        print("未采集到数据。")
        return

    ts = np.array([s.t_s for s in samples], dtype=float)
    vs = np.array([float("nan") if s.value_raw is None else float(s.value_raw) for s in samples], dtype=float)
    ok_mask = np.isfinite(vs)
    read_error_count = int(np.sum([1 for s in samples if not s.read_ok]))
    read_error_ratio = float(read_error_count / len(samples))

    rates = compute_rates(ts)

    stats = {}
    if int(np.sum(ok_mask)) > 0:
        v_valid = vs[ok_mask]
        stats = {
            "value_mean": float(np.mean(v_valid)),
            "value_std": float(np.std(v_valid)),
            "value_min": float(np.min(v_valid)),
            "value_max": float(np.max(v_valid)),
        }

    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(__file__).resolve().parent
    base = out_dir / f"fn_test_{ts_str}"
    csv_detail = base.with_suffix(".csv")
    csv_report = out_dir / f"fn_test_{ts_str}_report.csv"
    png_path = base.with_suffix(".png")

    report = {
        "var_name": VAR_NAME,
        "plc_port": PLC_PORT,
        "net_id": net_id,
        "start_wall": (start_wall.isoformat(timespec="milliseconds") if "start_wall" in locals() else ""),
        "end_wall": (end_wall.isoformat(timespec="milliseconds") if "end_wall" in locals() else ""),
        "duration_s": rates.get("duration_s", float(ts[-1])),
        "samples_total": len(samples),
        "read_error_count": read_error_count,
        "read_error_ratio": read_error_ratio,
        "sleep_s": SLEEP_S,
        "live_plot": int(bool(LIVE_PLOT)),
        "plot_update_min_interval_s": PLOT_UPDATE_MIN_INTERVAL_S,
        **rates,
        **stats,
    }

    save_csv_detail(csv_detail, samples)
    save_csv_report(csv_report, report)
    save_plot(png_path, samples)

    print("")
    print("测试完成：")
    print(f"- 详细数据CSV: {csv_detail}")
    print(f"- 统计报告CSV: {csv_report}")
    print(f"- 测试图PNG:   {png_path}")
    print("")
    print("关键结果：")
    if "rate_hz_by_total" in rates:
        print(f"- 采样频率(总时长): {rates['rate_hz_by_total']:.2f} Hz")
        print(f"- 采样频率(均值dt): {rates['rate_hz_by_mean_dt']:.2f} Hz")
        print(f"- 采样频率(中位dt): {rates['rate_hz_by_median_dt']:.2f} Hz")
        print(f"- dt(ms): mean={rates['dt_ms_mean']:.3f}  median={rates['dt_ms_median']:.3f}  min={rates['dt_ms_min']:.3f}  max={rates['dt_ms_max']:.3f}")
    print(f"- 读取错误(ADS异常/读取失败): {read_error_count}/{len(samples)} ({read_error_ratio*100:.2f}%)")
    if int(np.sum(ok_mask)) > 0:
        print(f"- 数值统计: mean={stats['value_mean']:.3f}  std={stats['value_std']:.3f}  min={stats['value_min']:.3f}  max={stats['value_max']:.3f}")


if __name__ == "__main__":
    main()
