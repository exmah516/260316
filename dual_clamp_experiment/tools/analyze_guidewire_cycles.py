"""导丝实验记录的周期级分析。

该脚本只读取 records 下的原始 CSV/JSON，在 records\analysis 下生成派生数据、图表和报告。
"""

import csv
import html
import json
import re
import warnings
from collections import defaultdict
from pathlib import Path

import matplotlib
import numpy as np
from scipy.signal import savgol_filter

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.stats import spearmanr

warnings.filterwarnings("ignore", message="Glyph .* missing from font")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial Unicode MS", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


BASE = Path(r"D:\Work_files\Vessel intervention Robot\260316\dual_clamp_experiment\x64\Debug\records")
OUT = BASE / "analysis"
DERIVED = OUT / "derived_acceleration"
OUT.mkdir(parents=True, exist_ok=True)
DERIVED.mkdir(parents=True, exist_ok=True)

# 力响应的基线与响应窗口。10 ms 避开命令切换边缘；100 ms 表示短时扰动带。
FORCE_WINDOW_S = 0.100
EDGE_GUARD_S = 0.010
MIN_WINDOW_S = 0.040


def read_csv(path):
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows, fields):
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def value(rows, key):
    result = []
    for row in rows:
        try:
            result.append(float(row.get(key, "")))
        except (TypeError, ValueError):
            result.append(np.nan)
    return np.asarray(result, dtype=float)


def as_float(obj, key, fallback=np.nan):
    try:
        return float(obj[key])
    except (KeyError, TypeError, ValueError):
        return fallback


def safe_name(name):
    return re.sub(r"[^A-Za-z0-9_-]+", "_", name)[:120]


def finite_or_blank(number):
    return number if np.isfinite(number) else ""


def median_or_blank(numbers):
    values = np.asarray([x for x in numbers if np.isfinite(x)], dtype=float)
    return float(np.median(values)) if values.size else ""


def mean_or_blank(numbers):
    values = np.asarray([x for x in numbers if np.isfinite(x)], dtype=float)
    return float(np.mean(values)) if values.size else ""


def range_or_blank(numbers):
    values = np.asarray([x for x in numbers if np.isfinite(x)], dtype=float)
    return float(np.max(values) - np.min(values)) if values.size else ""


def parse_name_parameters(name, meta):
    """优先使用 JSON；旧记录没有参数时，只把目录名作为弱标签。"""
    result = {
        "parameter_source": "json" if "return_velocity_mm_s" in meta else "directory_label",
        "return_velocity_set_mm_s": as_float(meta, "return_velocity_mm_s"),
        "return_acceleration_set_mm_s2": as_float(meta, "return_acceleration_mm_s2"),
        "return_jerk_set_mm_s3": as_float(meta, "return_jerk_mm_s3"),
        "forward_velocity_set_mm_s": as_float(meta, "forward_velocity_mm_s"),
        "forward_acceleration_set_mm_s2": as_float(meta, "forward_acceleration_mm_s2"),
        "release_wait_set_ms": as_float(meta, "release_wait_ms"),
        "reclamp_wait_set_ms": as_float(meta, "reclamp_wait_ms"),
    }
    explicit = {
        "return_velocity_set_mm_s": r"速度(-?\d+(?:\.\d+)?)",
        "return_acceleration_set_mm_s2": r"加速度(-?\d+(?:\.\d+)?)",
        "return_jerk_set_mm_s3": r"jerk(-?\d+(?:\.\d+)?)",
    }
    for field, pattern in explicit.items():
        if not np.isfinite(result[field]):
            match = re.search(pattern, name, re.IGNORECASE)
            if match:
                result[field] = float(match.group(1))

    # 早期目录使用“正常推-速度-加速度-减速度-Jerk”形式。
    legacy = re.search(r"正常推-(\d+(?:\.\d+)?)-(\d+(?:\.\d+)?)-(\d+(?:\.\d+)?)-(\d+(?:\.\d+)?)", name)
    if legacy:
        if not np.isfinite(result["return_velocity_set_mm_s"]):
            result["return_velocity_set_mm_s"] = float(legacy.group(1))
        if not np.isfinite(result["return_acceleration_set_mm_s2"]):
            result["return_acceleration_set_mm_s2"] = float(legacy.group(2))
        if not np.isfinite(result["return_jerk_set_mm_s3"]):
            result["return_jerk_set_mm_s3"] = float(legacy.group(4))

    result["directory_has_gripper_label"] = "有夹爪" in name or "有导丝" in name or "有导管" in name
    result["directory_no_wait_label"] = "不等待" in name
    return result


def event_map(events):
    result = []
    for event in events:
        try:
            result.append(
                {
                    "name": event.get("event_name", ""),
                    "time": float(event.get("plc_time_us", "0")) / 1e6,
                    "cycle": int(float(event.get("cycle_index", "0"))),
                    "phase": int(float(event.get("phase", "0"))),
                }
            )
        except (TypeError, ValueError):
            continue
    return sorted(result, key=lambda item: item["time"])


def events_of(events, event_name, cycle=None, after=-np.inf):
    return [event for event in events if event["name"] == event_name and (cycle is None or event["cycle"] == cycle) and event["time"] >= after]


def first_event(events, event_name, cycle=None, after=-np.inf):
    matches = events_of(events, event_name, cycle, after)
    return matches[0] if matches else None


def next_boundary(events, after_time):
    candidates = [event for event in events if event["time"] > after_time and event["name"] in {"ForwardStart", "FinalForwardStart", "RecordStop"}]
    return candidates[0] if candidates else None


def build_cycles(events):
    cycles = []
    for forward in [event for event in events if event["name"] == "ForwardStart"]:
        cycle = forward["cycle"]
        release = first_event(events, "ReleaseStart", cycle, forward["time"])
        returned = first_event(events, "ReturnStart", cycle, release["time"] if release else forward["time"])
        reclamp = first_event(events, "ReclampStart", cycle, returned["time"] if returned else forward["time"])
        if not (release and returned and reclamp):
            continue
        boundary = next_boundary(events, reclamp["time"])
        cycles.append(
            {
                "cycle_index": cycle,
                "forward": forward["time"],
                "release": release["time"],
                "return": returned["time"],
                "reclamp": reclamp["time"],
                "next_boundary": boundary["time"] if boundary else np.nan,
                "next_boundary_name": boundary["name"] if boundary else "",
            }
        )
    return cycles


def derive_acceleration(t, velocity):
    valid = np.isfinite(t) & np.isfinite(velocity)
    raw = np.full_like(velocity, np.nan)
    smooth = np.full_like(velocity, np.nan)
    if valid.sum() < 5:
        return raw, smooth
    tv = t[valid]
    vv = velocity[valid]
    delta = float(np.median(np.diff(tv)))
    if not np.isfinite(delta) or delta <= 0:
        delta = 0.001
    raw[valid] = np.gradient(vv, tv)
    # 11 ms 三阶 Savitzky-Golay 微分：抑制 1 ms 速度采样噪声，保留运动段趋势。
    window = min(11, len(vv) if len(vv) % 2 == 1 else len(vv) - 1)
    if window >= 5:
        smooth[valid] = savgol_filter(vv, window_length=window, polyorder=3, deriv=1, delta=delta)
    else:
        smooth[valid] = raw[valid]
    return raw, smooth


def nearest_time_value(t, data, when):
    if not np.isfinite(when):
        return np.nan
    index = int(np.argmin(np.abs(t - when)))
    return data[index]


def segment_profile(t, position, velocity, acceleration, start, end):
    if not (np.isfinite(start) and np.isfinite(end) and end > start):
        return {key: "" for key in ("duration_ms", "distance_mm", "velocity_peak_mm_s", "velocity_mean_mm_s", "acceleration_p95_mm_s2", "acceleration_peak_mm_s2", "deceleration_p95_mm_s2", "deceleration_peak_mm_s2")}
    core_start = start + EDGE_GUARD_S
    core_end = end - EDGE_GUARD_S
    core = (t >= core_start) & (t <= core_end)
    if core.sum() < 5:
        core = (t >= start) & (t <= end)
    vv = np.abs(velocity[core])
    distance = nearest_time_value(t, position, end) - nearest_time_value(t, position, start)
    direction = 1.0 if distance >= 0.0 else -1.0
    aligned_acceleration = acceleration[core] * direction
    accelerating = aligned_acceleration[aligned_acceleration > 0.0]
    decelerating = -aligned_acceleration[aligned_acceleration < 0.0]
    return {
        "duration_ms": (end - start) * 1000.0,
        "distance_mm": distance,
        "velocity_peak_mm_s": finite_or_blank(np.nanmax(vv)) if np.isfinite(vv).any() else "",
        "velocity_mean_mm_s": finite_or_blank(np.nanmean(vv)) if np.isfinite(vv).any() else "",
        "acceleration_p95_mm_s2": finite_or_blank(np.nanquantile(accelerating, 0.95)) if accelerating.size else "",
        "acceleration_peak_mm_s2": finite_or_blank(np.nanmax(accelerating)) if accelerating.size else "",
        "deceleration_p95_mm_s2": finite_or_blank(np.nanquantile(decelerating, 0.95)) if decelerating.size else "",
        "deceleration_peak_mm_s2": finite_or_blank(np.nanmax(decelerating)) if decelerating.size else "",
    }


def force_response(t, signal, event_time_s, previous_time_s, next_time_s):
    """事件前后窗口。后窗口自动截断到下一阶段前，避免混入下一动作。"""
    if not np.isfinite(event_time_s):
        return {key: "" for key in ("available", "pre_duration_ms", "post_duration_ms", "mean_shift", "range", "std", "mad", "abs_integral_Ns")}
    pre_start = max(previous_time_s + EDGE_GUARD_S, event_time_s - FORCE_WINDOW_S)
    pre_end = event_time_s - EDGE_GUARD_S
    post_start = event_time_s + EDGE_GUARD_S
    post_end = min(next_time_s - EDGE_GUARD_S, event_time_s + FORCE_WINDOW_S)
    before = signal[(t >= pre_start) & (t <= pre_end)]
    after = signal[(t >= post_start) & (t <= post_end)]
    before = before[np.isfinite(before)]
    after = after[np.isfinite(after)]
    pre_duration = (pre_end - pre_start) * 1000.0
    post_duration = (post_end - post_start) * 1000.0
    if before.size < 10 or after.size < 10 or pre_duration < MIN_WINDOW_S * 1000.0 or post_duration < MIN_WINDOW_S * 1000.0:
        return {
            "available": False,
            "pre_duration_ms": max(0.0, pre_duration),
            "post_duration_ms": max(0.0, post_duration),
            "mean_shift": "",
            "range": "",
            "std": "",
            "mad": "",
            "abs_integral_Ns": "",
        }
    baseline = float(np.mean(before))
    centered = after - baseline
    return {
        "available": True,
        "pre_duration_ms": pre_duration,
        "post_duration_ms": post_duration,
        "mean_shift": float(np.mean(centered)),
        "range": float(np.max(after) - np.min(after)),
        "std": float(np.std(after)),
        "mad": float(np.median(np.abs(after - np.median(after)))),
        "abs_integral_Ns": float(np.trapezoid(np.abs(centered), dx=(post_duration / 1000.0) / max(1, after.size - 1))),
    }


def add_force_fields(target, prefix, fn, ft):
    target[f"{prefix}_window_available"] = fn["available"] and ft["available"]
    target[f"{prefix}_pre_ms"] = fn["pre_duration_ms"]
    target[f"{prefix}_post_ms"] = fn["post_duration_ms"]
    for output, key in (("fn_shift_N", "mean_shift"), ("fn_range_N", "range"), ("fn_std_N", "std"), ("fn_abs_integral_Ns", "abs_integral_Ns")):
        target[f"{prefix}_{output}"] = fn[key]
    for output, key in (("ft_shift_N", "mean_shift"), ("ft_range_N", "range"), ("ft_std_N", "std"), ("ft_abs_integral_Ns", "abs_integral_Ns")):
        target[f"{prefix}_{output}"] = ft[key]


def aggregate_cycle_field(cycles, field):
    return median_or_blank([as_float(cycle, field) for cycle in cycles])


def write_derived_acceleration(stem, rows, t, velocity, recorded_acc, raw_acc, smooth_acc):
    derived_rows = []
    for index, row in enumerate(rows):
        derived_rows.append(
            {
                "sample_index": row.get("sample_index", index),
                "plc_time_us": row.get("plc_time_us", ""),
                "phase": row.get("phase", ""),
                "event_sequence": row.get("event_sequence", ""),
                "cycle_index": row.get("cycle_index", ""),
                "axis6_vel_mm_s": finite_or_blank(velocity[index]),
                "axis6_acc_recorded_mm_s2": finite_or_blank(recorded_acc[index]),
                "axis6_acc_derived_raw_mm_s2": finite_or_blank(raw_acc[index]),
                "axis6_acc_derived_smooth_mm_s2": finite_or_blank(smooth_acc[index]),
            }
        )
    output = DERIVED / f"{stem}_axis6_acceleration.csv"
    write_csv(output, derived_rows, list(derived_rows[0]) if derived_rows else [])
    return output.name


def plot_record(path, title, t, fn, ft, pos, vel, recorded_acc, smooth_acc, events):
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True, constrained_layout=True)
    axes[0].plot(t, fn, color="#1565c0", lw=0.65, label="fn2 decoupled (N)")
    axes[0].plot(t, ft, color="#ef6c00", lw=0.65, label="ft2 calibrated (N)")
    axes[0].set_ylabel("Force (N)")
    axes[0].legend(loc="upper right")
    axes[1].plot(t, pos, color="#2e7d32", lw=0.65, label="axis6 position")
    axes[1].set_ylabel("Position (mm)")
    axes[1].legend(loc="upper right")
    axes[2].plot(t, vel, color="#6a1b9a", lw=0.65, label="axis6 velocity")
    axes[2].set_ylabel("Velocity (mm/s)")
    axes[2].legend(loc="upper right")
    axes[3].plot(t, smooth_acc, color="#c62828", lw=0.65, label="derived acceleration (11 ms SG)")
    if np.isfinite(recorded_acc).any() and np.nanmax(np.abs(recorded_acc)) > 0:
        axes[3].plot(t, recorded_acc, color="#777", lw=0.45, alpha=0.6, label="recorded acceleration")
    axes[3].set_ylabel("Acceleration (mm/s²)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="upper right")
    for event in events:
        if event["time"] <= 0 or event["name"] not in {"ReleaseStart", "ReturnStart", "ReclampStart", "FinalForwardStart"}:
            continue
        for axis in axes:
            axis.axvline(event["time"], color="#555", alpha=0.20, lw=0.65)
        axes[0].text(event["time"], 0.98, event["name"], rotation=90, va="top", ha="right", transform=axes[0].get_xaxis_transform(), fontsize=7)
    fig.suptitle(title, fontsize=11)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def html_number(value, digits=5):
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return "—"


def write_record_html(path, record, cycles, image_name, derived_name):
    cycle_rows = []
    for cycle in cycles:
        cycle_rows.append(
            "<tr>"
            f"<td>{cycle['cycle_index']}</td>"
            f"<td>{html_number(cycle['forward_velocity_peak_mm_s'], 2)}</td>"
            f"<td>{html_number(cycle['return_velocity_peak_mm_s'], 2)}</td>"
            f"<td>{html_number(cycle['return_acceleration_p95_mm_s2'], 1)}</td>"
            f"<td>{html_number(cycle['return_duration_ms'], 1)}</td>"
            f"<td>{html_number(cycle['return_fn_shift_N'])}</td>"
            f"<td>{html_number(cycle['return_fn_range_N'])}</td>"
            f"<td>{html_number(cycle['return_ft_shift_N'])}</td>"
            f"<td>{html_number(cycle['return_ft_range_N'])}</td>"
            f"<td>{html_number(cycle['reclamp_fn_range_N']) if cycle['reclamp_window_available'] else '不可分离'}</td>"
            "</tr>"
        )
    status_note = "正常收尾" if record["status"] == "Completed" else ("动作序列完整但归档状态异常" if record["action_complete"] else "动作序列不完整")
    body = f"""<!doctype html><meta charset='utf-8'><title>{html.escape(record['name'])}</title>
<style>body{{font-family:Arial,'Microsoft YaHei',sans-serif;margin:22px;color:#202020}}table{{border-collapse:collapse}}th,td{{border:1px solid #aaa;padding:4px 7px;white-space:nowrap}}img{{max-width:100%;border:1px solid #bbb}}.warn{{color:#a00000}}</style>
<h1>{html.escape(record['name'])}</h1>
<p>状态：<b class='{'warn' if record['status'] != 'Completed' else ''}'>{record['status']}</b>；{status_note}。<br>
实际循环数：{record['cycle_count_actual']}；CSV角度：{html_number(record['angle_deg'], 2)}°；回退实际峰值速度中位数：{html_number(record['return_velocity_peak_median_mm_s'], 2)} mm/s；回退导出加速度P95中位数：{html_number(record['return_acceleration_p95_median_mm_s2'], 1)} mm/s²。</p>
<p>派生加速度文件：<a href='derived_acceleration/{html.escape(derived_name)}'>{html.escape(derived_name)}</a>。加速度通过轴6速度按PLC时间戳微分，使用11 ms三阶Savitzky-Golay平滑。</p>
<img src='{html.escape(image_name)}' alt='force and kinematics plot'>
<h2>每个往复周期</h2>
<table><tr><th>周期</th><th>前向峰值速度</th><th>回退峰值速度</th><th>回退加速度P95</th><th>回退时长 ms</th><th>回退fn均值变化 N</th><th>回退fn波动带 N</th><th>回退ft均值变化 N</th><th>回退ft波动带 N</th><th>重夹紧fn波动带 N</th></tr>{''.join(cycle_rows)}</table>
<p>力窗口采用动作前后各最多100 ms，且自动在下一动作前截断。对于无等待记录，重夹紧后约1 ms即进入下一前向运动，无法把“重夹紧本身”与“下一段前向”分离，因此该列标为不可分离。</p>"""
    path.write_text(body, encoding="utf-8")


def get_group_rows(records, predicate):
    return [record for record in records if record["analysis_usable"] and predicate(record)]


def group_summary(name, factor, records):
    cycles = [cycle for record in records for cycle in record["cycles"]]
    return {
        "analysis": name,
        "factor_level": factor,
        "record_count": len(records),
        "cycle_count": len(cycles),
        "status_completed_records": sum(record["status"] == "Completed" for record in records),
        "return_velocity_peak_median_mm_s": aggregate_cycle_field(cycles, "return_velocity_peak_mm_s"),
        "return_acceleration_p95_median_mm_s2": aggregate_cycle_field(cycles, "return_acceleration_p95_mm_s2"),
        "return_fn_shift_median_N": aggregate_cycle_field(cycles, "return_fn_shift_N"),
        "return_fn_range_median_N": aggregate_cycle_field(cycles, "return_fn_range_N"),
        "return_fn_abs_integral_median_Ns": aggregate_cycle_field(cycles, "return_fn_abs_integral_Ns"),
        "return_ft_shift_median_N": aggregate_cycle_field(cycles, "return_ft_shift_N"),
        "return_ft_range_median_N": aggregate_cycle_field(cycles, "return_ft_range_N"),
        "reclamp_fn_range_median_N": aggregate_cycle_field(cycles, "reclamp_fn_range_N"),
    }


def build_controlled_groups(records):
    groups = []
    # 0°、有夹爪、五次、151 ms等待的早期批次：目录只改变了速度标签。
    for speed in (250.0, 500.0, 750.0, 1000.0):
        rows = get_group_rows(
            records,
            lambda r, speed=speed: r["directory_has_gripper_label"] and r["cycle_count_actual"] == 5 and abs(r["angle_deg"]) < 1.0
            and abs(r["return_wait_median_ms"] - 151.0) < 5.0 and np.isfinite(r["return_velocity_set_mm_s"])
            and abs(r["return_velocity_set_mm_s"] - speed) < 0.1 and np.isfinite(r["return_acceleration_set_mm_s2"])
            and abs(r["return_acceleration_set_mm_s2"] - 1000.0) < 0.1,
        )
        if rows:
            groups.append(group_summary("早期批次：速度标签对比（其它目录标签固定）", f"目录速度={speed:g}", rows))

    # 固定速度2000、加速度5000的三条十周期记录：Jerk 是唯一明确写出的变化量。
    for jerk in (150000.0, 200000.0, 250000.0):
        rows = get_group_rows(
            records,
            lambda r, jerk=jerk: r["cycle_count_actual"] == 10 and abs(r["angle_deg"]) < 1.0
            and np.isfinite(r["return_velocity_set_mm_s"]) and abs(r["return_velocity_set_mm_s"] - 2000.0) < 0.1
            and np.isfinite(r["return_acceleration_set_mm_s2"]) and abs(r["return_acceleration_set_mm_s2"] - 5000.0) < 0.1
            and np.isfinite(r["return_jerk_set_mm_s3"]) and abs(r["return_jerk_set_mm_s3"] - jerk) < 0.1
            and abs(r["return_wait_median_ms"] - 151.0) < 5.0,
        )
        if rows:
            groups.append(group_summary("十周期批次：固定速度/加速度下的Jerk标签对比", f"目录Jerk={jerk:g}", rows))

    # 明确同为速度2000、加速度1000、0°的有等待与无等待记录。
    for wait_name, predicate in (
        ("等待约151 ms", lambda r: abs(r["return_wait_median_ms"] - 151.0) < 5.0),
        ("无等待（约1 ms）", lambda r: r["return_wait_median_ms"] < 5.0),
    ):
        rows = get_group_rows(
            records,
            lambda r, predicate=predicate: r["cycle_count_actual"] == 10 and abs(r["angle_deg"]) < 1.0
            and np.isfinite(r["return_velocity_set_mm_s"]) and abs(r["return_velocity_set_mm_s"] - 2000.0) < 0.1
            and np.isfinite(r["return_acceleration_set_mm_s2"]) and abs(r["return_acceleration_set_mm_s2"] - 1000.0) < 0.1
            and predicate(r),
        )
        if rows:
            groups.append(group_summary("十周期批次：等待时间对比", wait_name, rows))

    # 速度2000、0°、151 ms等待、十周期的加速度系列。部分目录没有Jerk标签，因此为探索性比较。
    for acceleration in (500.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0):
        rows = get_group_rows(
            records,
            lambda r, acceleration=acceleration: r["cycle_count_actual"] == 10 and abs(r["angle_deg"]) < 1.0
            and np.isfinite(r["return_velocity_set_mm_s"]) and abs(r["return_velocity_set_mm_s"] - 2000.0) < 0.1
            and np.isfinite(r["return_acceleration_set_mm_s2"]) and abs(r["return_acceleration_set_mm_s2"] - acceleration) < 0.1
            and abs(r["return_wait_median_ms"] - 151.0) < 5.0,
        )
        if rows:
            groups.append(group_summary("十周期批次：加速度标签系列（Jerk部分缺失，探索性）", f"目录加速度={acceleration:g}", rows))
    return groups


def plot_controlled(groups):
    if not groups:
        return
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    sections = [
        ("早期批次：速度标签", "早期批次：速度标签对比（其它目录标签固定）", "return_velocity_peak_median_mm_s", "Actual return peak velocity (mm/s)"),
        ("早期批次：速度与fn扰动", "早期批次：速度标签对比（其它目录标签固定）", "return_fn_range_median_N", "Return fn2 band median (N)"),
        ("固定加速度下：Jerk", "十周期批次：固定速度/加速度下的Jerk标签对比", "return_acceleration_p95_median_mm_s2", "Derived return acceleration P95 (mm/s²)"),
        ("加速度标签与重夹紧fn", "十周期批次：加速度标签系列（Jerk部分缺失，探索性）", "reclamp_fn_range_median_N", "Reclamp fn2 band median (N)"),
    ]
    for axis, (title, section, key, ylabel) in zip(axes.ravel(), sections):
        selected = [group for group in groups if group["analysis"] == section]
        if not selected:
            axis.set_title(title + " (no matched records)")
            axis.axis("off")
            continue
        x = np.arange(len(selected))
        y = [float(group[key]) if group[key] != "" else np.nan for group in selected]
        axis.bar(x, y, color="#2e7d32")
        axis.set_xticks(x, [group["factor_level"] for group in selected], rotation=15, ha="right")
        axis.set_ylabel(ylabel)
        axis.set_title(title)
        for index, group in enumerate(selected):
            axis.text(index, y[index], f"n={group['cycle_count']} cycles\n{group['record_count']} rec", ha="center", va="bottom", fontsize=8)
    fig.suptitle("Guidewire controlled-comparison summaries")
    fig.savefig(OUT / "controlled_variable_effects.png", dpi=170)
    plt.close(fig)


def write_report(records, groups):
    usable = [record for record in records if record["analysis_usable"]]
    completed = [record for record in usable if record["status"] == "Completed"]
    status_abnormal = [record for record in usable if record["status"] != "Completed"]
    total_cycles = sum(len(record["cycles"]) for record in usable)
    all_return = [cycle for record in usable for cycle in record["cycles"]]
    wait_group = [group for group in groups if group["analysis"] == "十周期批次：等待时间对比"]
    speed_group = [group for group in groups if group["analysis"] == "早期批次：速度标签对比（其它目录标签固定）"]
    jerk_group = [group for group in groups if group["analysis"] == "十周期批次：固定速度/加速度下的Jerk标签对比"]
    acceleration_group = [group for group in groups if group["analysis"] == "十周期批次：加速度标签系列（Jerk部分缺失，探索性）"]

    acceleration_attainment = []
    acceleration_x = []
    acceleration_return_fn = []
    acceleration_return_ft = []
    acceleration_reclamp_fn = []
    for group in acceleration_group:
        try:
            setting = float(group["factor_level"].split("=")[1])
            actual = float(group["return_acceleration_p95_median_mm_s2"])
            acceleration_attainment.append(f"设定{setting:g}→实测{actual:.1f} mm/s²（{actual / setting * 100.0:.0f}%）")
            acceleration_x.append(actual)
            acceleration_return_fn.append(float(group["return_fn_range_median_N"]))
            acceleration_return_ft.append(float(group["return_ft_range_median_N"]))
            acceleration_reclamp_fn.append(float(group["reclamp_fn_range_median_N"]))
        except (IndexError, ValueError, KeyError, TypeError):
            pass
    if len(acceleration_x) >= 3:
        return_fn_rho = spearmanr(acceleration_x, acceleration_return_fn).statistic
        return_ft_rho = spearmanr(acceleration_x, acceleration_return_ft).statistic
        reclamp_fn_rho = spearmanr(acceleration_x, acceleration_reclamp_fn).statistic
    else:
        return_fn_rho = return_ft_rho = reclamp_fn_rho = np.nan

    def group_lines(groups, fields):
        if not groups:
            return ["该对照组在现有记录中不存在。"]
        result = []
        for group in groups:
            values = "；".join(f"{label}={html_number(group[key], digits)}" for label, key, digits in fields)
            result.append(f"- {group['factor_level']}：{group['record_count']} 条记录、{group['cycle_count']} 个周期；{values}。")
        return result

    lines = [
        "# 导丝往复实验：周期级控制变量分析",
        "",
        f"分析日期：2026-09-01。共读取 {len(records)} 条导丝记录。动作序列完整、可参与运动-力分析的记录为 **{len(usable)}** 条，其中正常 Completed **{len(completed)}** 条，动作完成后被标为 Aborted/Error 的 **{len(status_abnormal)}** 条；合计 **{total_cycles}** 个完整往复周期。原始 CSV、JSON 和事件文件未改写。",
        "",
        "## 窗口定义",
        "力响应不是取单点峰值。每个回退或重夹紧事件使用事件前最多100 ms作为基线、事件后最多100 ms作为响应窗口，并在事件两侧留10 ms保护带。响应窗口会在下一动作前自动截断，避免把下一段运动混入当前事件。窗口不足40 ms时该事件标记为不可分离。",
        "",
        "因此，常规的释放/重夹紧等待约151 ms时，100 ms窗口能够覆盖电缸切换后的主要短时波动；无等待记录中，重夹紧后约1 ms就进入下一前向段，重夹紧与下一段运动物理上无法仅凭本批1 kHz数据分离，不能把合并波动归因给夹紧本身。",
        "",
        "## 速度与加速度是否达到设定值",
        "轴6原始 `axis6_acc_mm_s2` 在本批记录中全部为0，因此不使用该列。已在每个实验对应的 `derived_acceleration` CSV 中补充两列：原始速度中心差分加速度，以及用于统计的11 ms三阶Savitzky-Golay平滑导数加速度。回退段把沿回退方向的正加速与制动分开输出；“回退加速度P95”只取正加速段，避免把减速度误判为设置的加速度。",
        f"所有可用周期的回退峰值速度中位数为 **{html_number(median_or_blank([as_float(c, 'return_velocity_peak_mm_s') for c in all_return]), 2)} mm/s**；回退导出加速度P95中位数为 **{html_number(median_or_blank([as_float(c, 'return_acceleration_p95_mm_s2') for c in all_return]), 1)} mm/s²**。",
        "速度设定值是上限，不保证短距离运动一定达到。导丝回退距离约20 mm，且Jerk和加速度限制会形成三角或S曲线，因此即使目录写入500、1000或2000 mm/s，实际轨迹也可能在到达目标前已开始减速。需要同时看每条图和周期表中的“回退峰值速度/导出加速度”，不能只根据UI输入值判断。",
        "早期速度标签250、500、750、1000 mm/s的实际回退峰值速度均约111.25 mm/s；因此分别只达到标签值的约44%、22%、15%和11%。速度上限在这些短行程条件下没有被触及。",
        "",
        "## 控制变量对比",
        "### 1. 早期五周期批次：仅速度标签变化",
        *group_lines(speed_group, [("实际回退峰值速度", "return_velocity_peak_median_mm_s", 2), ("回退fn波动带", "return_fn_range_median_N", 5), ("回退ft波动带", "return_ft_range_median_N", 5)]),
        "这组中，目录速度标签虽从250到1000变化，但实测回退峰值速度如果仍集中在约111 mm/s，说明20 mm行程和Jerk/加速度约束使速度上限没有被达到。此时“设置速度”对力波动的可辨识影响应很小，因为实际运动曲线几乎没有改变；不能据此声称速度本身无效，只能说明该设置区间未转化为不同实际速度。",
        "",
        "### 2. 十周期批次：固定速度2000、加速度5000下的Jerk标签",
        *group_lines(jerk_group, [("实际回退加速度P95", "return_acceleration_p95_median_mm_s2", 1), ("回退fn波动带", "return_fn_range_median_N", 5), ("回退ft波动带", "return_ft_range_median_N", 5), ("重夹紧fn波动带", "reclamp_fn_range_median_N", 5)]),
        "这一组是现有数据中最接近单因素的Jerk比较。每个条件只有一个母记录、10个周期；周期可用于估计同一条件下的波动带和周期趋势，但它们共享同一次取零、装夹和系统状态，不能当作10个完全独立的重复实验。",
        "",
        "### 3. 十周期批次：等待时间",
        *group_lines(wait_group, [("实际回退峰值速度", "return_velocity_peak_median_mm_s", 2), ("回退fn波动带", "return_fn_range_median_N", 5), ("回退ft波动带", "return_ft_range_median_N", 5)]),
        "等待时间可以影响回退开始前电缸与器械是否已稳定。无等待组在释放后约1 ms即开始回退，因此既没有独立的“释放后基线”，也没有独立的“重夹紧后窗口”；它只能作为“释放并立即回退/重夹紧并立即前向”的组合流程分析，不能与151 ms等待组做纯回退或纯重夹紧的数值对照。",
        "",
        "### 4. 十周期批次：加速度标签系列（探索性）",
        *group_lines(acceleration_group, [("实际回退峰值速度", "return_velocity_peak_median_mm_s", 2), ("实际正加速P95", "return_acceleration_p95_median_mm_s2", 1), ("回退fn波动带", "return_fn_range_median_N", 5), ("重夹紧fn波动带", "reclamp_fn_range_median_N", 5)]),
        "该系列的位置、角度、速度标签和151 ms等待保持一致，但500~4000的目录没有写Jerk，因此它用于识别“实际加速/残余振动的关联”，不是严格的单因素因果对比。",
        "加速度达到情况：" + "；".join(acceleration_attainment) + "。500~4000 mm/s²基本能达到对应正加速水平；5000 mm/s²仅达到约80%~84%，符合短行程/Jerk约束下尚未充分爬升即进入后续轨迹的情况。",
        f"以六个加速度条件的记录中位数计算，实际正加速度与回退fn波动带的Spearman相关为 {return_fn_rho:.3f}，与回退ft波动带为 {return_ft_rho:.3f}，均未显示单调关系；与重夹紧fn波动带为 {reclamp_fn_rho:.3f}。后者很强，但Jerk配置不完整、每个条件仅一个母记录，因此该数值应作为“优先复测信号”，不能直接写成独立因果效应。",
        "",
        "## 哪些因素目前最值得关注",
        "1. **动作阶段**：回退开始和重新夹紧必须分开分析。回退窗口可稳定量化；重夹紧只有在后续等待至少40 ms时才能独立量化。",
        "2. **实际运动曲线，而不是UI设定数字**：现有数据表明许多高速度设定在20 mm回退中没有达到设定上限。影响力扰动的应是实测峰值速度、导出加速度和Jerk形成的速度曲线。",
        "3. **Jerk/加速度组合**：固定标称加速度5000的三条Jerk记录可用于优先复测；如果导出加速度P95不同，且回退或重夹紧力带随之重复变化，才可以建立参数-扰动关系。加速度系列中，重夹紧fn波动随实际加速度升高而明显增大是当前最值得复测的现象，但Jerk未完整记录，暂不写成因果结论。",
        "4. **等待时间**：这是目前可以从事件表可靠识别的因素，但无等待下释放与回退、重夹紧与下一前向运动都耦合，因此应把“无等待”视为组合流程，而非纯电缸夹紧因素。",
        "5. **角度、夹爪状态**：当前90°仅2条完整记录、无夹爪仅1条完整记录，且运动参数不完全匹配；不适合给出控制变量结论。重力补偿未做，角度比较尤其应谨慎。",
        "",
        "## 输出文件",
        "- `cycle_metrics.csv`：每一个往复周期的实际速度、导出加速度、回退和重夹紧力响应指标。",
        "- `record_metrics.csv`：每条实验记录的周期中位数、动作完整性及可用性判断。",
        "- `controlled_variable_analysis.csv`：按可匹配条件汇总的控制变量对比。",
        "- `derived_acceleration/*.csv`：每条记录的轴6导出加速度，原始记录不变。",
        "- 单条HTML/PNG：力、位置、速度、导出加速度和动作时间戳。",
        "",
        "## 后续实验建议",
        "要得到论文级因果结论，应让程序在 experiment.json 中强制写入每一次的前向/回退速度、加速度、减速度、Jerk、等待时间、位置和配合开关；每次只改变一个参数，并至少进行3次重新装夹/重新取零的独立重复。每条独立重复内的多个周期可作为重复测量，用于估计漂移和周期内变异。",
    ]
    (OUT / "导丝周期级分析报告.md").write_text("\n".join(lines), encoding="utf-8")


def build_index(records):
    table_rows = []
    for record in records:
        table_rows.append(
            "<tr>"
            f"<td>{html.escape(record['status'])}</td>"
            f"<td>{'是' if record['action_complete'] else '否'}</td>"
            f"<td>{record['cycle_count_actual']}</td>"
            f"<td>{html_number(record['angle_deg'], 2)}</td>"
            f"<td>{html_number(record['return_velocity_peak_median_mm_s'], 2)}</td>"
            f"<td>{html_number(record['return_acceleration_p95_median_mm_s2'], 1)}</td>"
            f"<td>{html_number(record['return_fn_range_median_N'])}</td>"
            f"<td><a href='{html.escape(record['plot_html'])}'>HTML</a> / <a href='{html.escape(record['plot_png'])}'>PNG</a></td>"
            "</tr>"
        )
    page = f"""<!doctype html><meta charset='utf-8'><title>导丝周期级实验分析</title>
<style>body{{font-family:Arial,'Microsoft YaHei',sans-serif;margin:22px;color:#202020}}table{{border-collapse:collapse;font-size:13px}}th,td{{border:1px solid #aaa;padding:4px 7px}}img{{max-width:100%;border:1px solid #bbb}}</style>
<h1>导丝周期级实验分析</h1>
<p><a href='导丝周期级分析报告.md'>分析报告</a> | <a href='cycle_metrics.csv'>周期指标CSV</a> | <a href='record_metrics.csv'>记录指标CSV</a> | <a href='controlled_variable_analysis.csv'>控制变量汇总CSV</a></p>
<p>力响应使用动作前后最多100 ms窗口，并在下一动作前自动截断。加速度由速度微分导出，原始CSV不修改。</p>
<h2>控制变量汇总</h2><img src='controlled_variable_effects.png' alt='controlled comparisons'>
<h2>各实验记录</h2><table><tr><th>归档状态</th><th>动作完整</th><th>周期数</th><th>CSV角度°</th><th>回退峰值速度</th><th>回退导出加速度P95</th><th>回退fn波动带</th><th>图</th></tr>{''.join(table_rows)}</table>"""
    (OUT / "index.html").write_text(page, encoding="utf-8")


def main():
    records = []
    all_cycles = []
    for directory in sorted(BASE.iterdir()):
        if not directory.is_dir() or directory.name == "analysis":
            continue
        json_path = directory / "experiment.json"
        samples_path = directory / "samples_1khz.csv"
        events_path = directory / "events.csv"
        if not (json_path.exists() and samples_path.exists()):
            continue
        try:
            meta = json.loads(json_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if meta.get("mode") != "guidewire":
            continue
        rows = read_csv(samples_path)
        if not rows:
            continue
        events = event_map(read_csv(events_path)) if events_path.exists() else []
        t = value(rows, "plc_time_us") / 1e6
        if not np.isfinite(t).any() or np.nanmax(t) <= 0:
            t = np.arange(len(rows), dtype=float) / 1000.0
        pos = value(rows, "axis6_pos_mm")
        vel = value(rows, "axis6_vel_mm_s")
        rec_acc = value(rows, "axis6_acc_mm_s2")
        fn = value(rows, "fn2_decoupled_delta_N")
        ft = value(rows, "ft2_cal_delta_N")
        raw_acc, smooth_acc = derive_acceleration(t, vel)
        cycles_raw = build_cycles(events)
        stem = safe_name(directory.name)
        derived_file = write_derived_acceleration(stem, rows, t, vel, rec_acc, raw_acc, smooth_acc)

        cycles = []
        for cycle in cycles_raw:
            forward_profile = segment_profile(t, pos, vel, smooth_acc, cycle["forward"], cycle["release"])
            return_profile = segment_profile(t, pos, vel, smooth_acc, cycle["return"], cycle["reclamp"])
            row = {
                "record_name": directory.name,
                "record_status": meta.get("status", ""),
                "cycle_index": cycle["cycle_index"],
                "forward_start_s": cycle["forward"],
                "release_start_s": cycle["release"],
                "return_start_s": cycle["return"],
                "reclamp_start_s": cycle["reclamp"],
                "next_boundary_s": cycle["next_boundary"],
                "next_boundary_name": cycle["next_boundary_name"],
            }
            row.update({f"forward_{key}": item for key, item in forward_profile.items()})
            row.update({f"return_{key}": item for key, item in return_profile.items()})
            return_fn = force_response(t, fn, cycle["return"], cycle["release"], cycle["reclamp"])
            return_ft = force_response(t, ft, cycle["return"], cycle["release"], cycle["reclamp"])
            add_force_fields(row, "return", return_fn, return_ft)
            next_time = cycle["next_boundary"] if np.isfinite(cycle["next_boundary"]) else cycle["reclamp"]
            reclamp_fn = force_response(t, fn, cycle["reclamp"], cycle["return"], next_time)
            reclamp_ft = force_response(t, ft, cycle["reclamp"], cycle["return"], next_time)
            add_force_fields(row, "reclamp", reclamp_fn, reclamp_ft)
            cycles.append(row)
            all_cycles.append(row)

        forward_count = len([event for event in events if event["name"] == "ForwardStart"])
        final_forward = any(event["name"] == "FinalForwardStart" for event in events)
        action_complete = forward_count > 0 and len(cycles) == forward_count and final_forward
        parameters = parse_name_parameters(directory.name, meta)
        angle = float(np.nanmedian(value(rows, "axis7_angle_deg"))) if np.isfinite(value(rows, "axis7_angle_deg")).any() else as_float(meta, "zero_axis7_angle_deg", 0.0)
        record = {
            "name": directory.name,
            "status": meta.get("status", ""),
            "reason": meta.get("reason", ""),
            "sample_count": meta.get("sample_count", len(rows)),
            "duration_s": meta.get("duration_s", ""),
            "action_complete": action_complete,
            "analysis_usable": action_complete,
            "cycle_count_actual": len(cycles),
            "angle_deg": angle,
            "return_wait_median_ms": median_or_blank([(cycle["return_start_s"] - cycle["release_start_s"]) * 1000.0 for cycle in cycles]),
            "reclamp_wait_median_ms": median_or_blank([(cycle["next_boundary_s"] - cycle["reclamp_start_s"]) * 1000.0 for cycle in cycles if np.isfinite(cycle["next_boundary_s"])]),
            "return_velocity_peak_median_mm_s": aggregate_cycle_field(cycles, "return_velocity_peak_mm_s"),
            "return_acceleration_p95_median_mm_s2": aggregate_cycle_field(cycles, "return_acceleration_p95_mm_s2"),
            "return_acceleration_peak_median_mm_s2": aggregate_cycle_field(cycles, "return_acceleration_peak_mm_s2"),
            "return_fn_range_median_N": aggregate_cycle_field(cycles, "return_fn_range_N"),
            "return_ft_range_median_N": aggregate_cycle_field(cycles, "return_ft_range_N"),
            "reclamp_fn_range_median_N": aggregate_cycle_field(cycles, "reclamp_fn_range_N"),
            "reclamp_ft_range_median_N": aggregate_cycle_field(cycles, "reclamp_ft_range_N"),
            "plot_html": f"{stem}_cycle_analysis.html",
            "plot_png": f"{stem}_cycle_analysis.png",
            "derived_acceleration_csv": derived_file,
            "cycles": cycles,
        }
        record.update(parameters)
        plot_record(OUT / record["plot_png"], directory.name, t, fn, ft, pos, vel, rec_acc, smooth_acc, events)
        write_record_html(OUT / record["plot_html"], record, cycles, record["plot_png"], derived_file)
        records.append(record)

    cycle_fields = list(all_cycles[0]) if all_cycles else []
    record_fields = [field for field in records[0] if field != "cycles"] if records else []
    write_csv(OUT / "cycle_metrics.csv", all_cycles, cycle_fields)
    write_csv(OUT / "record_metrics.csv", records, record_fields)
    groups = build_controlled_groups(records)
    group_fields = list(groups[0]) if groups else []
    write_csv(OUT / "controlled_variable_analysis.csv", groups, group_fields)
    plot_controlled(groups)
    write_report(records, groups)
    build_index(records)
    print(json.dumps({"records": len(records), "cycles": len(all_cycles), "usable_records": sum(record["analysis_usable"] for record in records), "output": str(OUT)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
