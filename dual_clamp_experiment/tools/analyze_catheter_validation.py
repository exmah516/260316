"""Analyze catheter-only timing and load-path parameters from recorded CSV files."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np


GROUPS = {
    "D04": 0.0,
    "E_C01": 0.02,
    "E_C02": 0.04,
    "E_C03": 0.06,
    "E_C04": 0.08,
}


def load_record(path: Path):
    csv_path = path / "samples_1khz.csv"
    with csv_path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < 1000:
        return None
    names = (
        "plc_time_us",
        "phase",
        "cycle_index",
        "axis1_pos_mm",
        "axis1_vel_mm_s",
        "axis1_acc_mm_s2",
        "fn1_cal_delta_N",
    )
    return {
        name: np.asarray([float(row[name]) for row in rows], dtype=float)
        for name in names
    }


def moving_average(values: np.ndarray, width: int) -> np.ndarray:
    if width <= 1:
        return values.copy()
    return np.convolve(values, np.ones(width) / width, mode="same")


def correlation_lag(reference: np.ndarray, delayed: np.ndarray, max_lag: int = 100):
    best = (-1.0, 0)
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            first = reference[:-lag or None]
            second = delayed[lag:]
        else:
            first = reference[-lag:]
            second = delayed[:lag]
        valid = np.isfinite(first) & np.isfinite(second)
        if valid.sum() < 100:
            continue
        first = first[valid] - first[valid].mean()
        second = second[valid] - second[valid].mean()
        denominator = np.linalg.norm(first) * np.linalg.norm(second)
        if denominator == 0:
            continue
        coefficient = float(first @ second / denominator)
        if coefficient > best[0]:
            best = (coefficient, lag)
    return best


def velocity_lags(data):
    time = (data["plc_time_us"] - data["plc_time_us"][0]) * 1e-6
    dt = float(np.median(np.diff(time)))
    position_velocity = moving_average(np.gradient(data["axis1_pos_mm"], dt), 5)
    lags = []
    phases = data["phase"].astype(int)
    cycles = data["cycle_index"].astype(int)
    for cycle in sorted(set(cycles)):
        indices = np.where((cycles == cycle) & (phases == 4))[0]
        if len(indices) < 300:
            continue
        margin = 50
        local = np.arange(margin, len(indices) - margin)
        local = local[
            (np.abs(position_velocity[indices][local]) > 2.0)
            & (np.abs(data["axis1_vel_mm_s"][indices][local]) > 2.0)
        ]
        if len(local) < 100:
            continue
        coefficient, lag = correlation_lag(
            position_velocity[indices][local],
            data["axis1_vel_mm_s"][indices][local],
        )
        lags.append({"cycle": int(cycle), "lag_samples": lag, "correlation": coefficient})
    return dt, lags


def phase4_values(data):
    time = data["plc_time_us"]
    phase = data["phase"].astype(int)
    cycles = data["cycle_index"].astype(int)
    force = data["fn1_cal_delta_N"]
    baseline_values = force[(phase == 3) & (cycles == 1)]
    baseline = float(np.median(baseline_values)) if len(baseline_values) else 0.0
    values = []
    for cycle in sorted(set(cycles)):
        indices = np.where((cycles == cycle) & (phase == 4))[0]
        if len(indices) < 300:
            continue
        elapsed = (time[indices] - time[indices[0]]) * 1e-6
        stable = force[indices][elapsed >= 0.4]
        if len(stable):
            values.append(float(np.median(stable) - baseline))
    return baseline, values


def fit_group_load(group_values):
    x = np.asarray([GROUPS[group] * 9.80665 for group in GROUPS], dtype=float)
    y = np.asarray([np.median(group_values[group]) for group in GROUPS], dtype=float)
    design = np.column_stack((x, np.ones_like(x)))
    alpha, beta = np.linalg.lstsq(design, y, rcond=None)[0]
    prediction = design @ np.asarray([alpha, beta])
    residual = y - prediction
    denominator = np.sum((y - y.mean()) ** 2)
    return {
        "alpha_N_per_N": float(alpha),
        "beta_N": float(beta),
        "mae_N": float(np.mean(np.abs(residual))),
        "rmse_N": float(np.sqrt(np.mean(residual ** 2))),
        "r2": float(1.0 - np.sum(residual ** 2) / denominator),
        "group_medians_N": {
            group: float(np.median(group_values[group])) for group in GROUPS
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--records",
        type=Path,
        default=Path(
            r"D:\Work_files\Vessel intervention Robot\260316\dual_clamp_experiment\x64\Debug\records"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("analysis") / "catheter_validation_20261005",
    )
    args = parser.parse_args()

    records = []
    for path in sorted(args.records.iterdir()):
        if not path.is_dir():
            continue
        match = re.search(r"(D04|E_C0[1-4])", path.name)
        group = match.group(1) if match else (
            "NEW_20260927" if path.name.startswith("20260927_223") else None
        )
        if group is None:
            continue
        data = load_record(path)
        if data is None:
            continue
        dt, lags = velocity_lags(data)
        baseline, values = phase4_values(data)
        records.append(
            {
                "group": group,
                "record": path.name,
                "sample_period_s": dt,
                "velocity_lags": lags,
                "phase3_baseline_N": baseline,
                "phase4_delta_values_N": values,
            }
        )

    lag_samples = [
        item["lag_samples"]
        for record in records
        for item in record["velocity_lags"]
    ]
    group_values = {group: [] for group in GROUPS}
    for record in records:
        if record["group"] in group_values and record["phase4_delta_values_N"]:
            group_values[record["group"]].append(
                float(np.median(record["phase4_delta_values_N"]))
            )
    # ponytail: group medians are intentionally used for this small, noisy dataset.
    load_fit = fit_group_load(group_values)
    result = {
        "records": records,
        "record_count": len(records),
        "velocity_delay_s": float(np.median(lag_samples) * 0.001),
        "velocity_delay_q1_s": float(np.percentile(lag_samples, 25) * 0.001),
        "velocity_delay_q3_s": float(np.percentile(lag_samples, 75) * 0.001),
        "velocity_delay_samples": {
            "count": len(lag_samples),
            "median": float(np.median(lag_samples)),
            "q1": float(np.percentile(lag_samples, 25)),
            "q3": float(np.percentile(lag_samples, 75)),
        },
        "load_fit": load_fit,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "catheter_validation.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    report = [
        "# 导管机构验证参数",
        "",
        f"- 分析记录：{len(records)} 条；有效 Phase 4 速度窗口：{len(lag_samples)} 个。",
        f"- 采样周期：{result['records'][0]['sample_period_s'] * 1000:.3f} ms。",
        f"- 速度延迟：{result['velocity_delay_s'] * 1000:.1f} ms "
        f"(四分位 {result['velocity_delay_q1_s'] * 1000:.1f}–"
        f"{result['velocity_delay_q3_s'] * 1000:.1f} ms)。",
        "",
        "| 负载组 | 额定外力/N | Phase 4 相对基线中位数/N |",
        "|---|---:|---:|",
    ]
    for group, mass in GROUPS.items():
        report.append(
            f"| {group} | {mass * 9.80665:.4f} | "
            f"{load_fit['group_medians_N'][group]:.4f} |"
        )
    report.extend(
        [
            "",
            f"- 组中位数回归：斜率 {load_fit['alpha_N_per_N']:.4f}，"
            f"截距 {load_fit['beta_N']:.4f} N，"
            f"MAE {load_fit['mae_N']:.4f} N，R² {load_fit['r2']:.4f}。",
            "- 该负载回归只作为传力验证记录，当前未启用实时逆重构；"
            "60 g 组不单调，不能据此扣除真实外载荷。",
        ]
    )
    (args.output / "catheter_validation.md").write_text(
        "\n".join(report) + "\n", encoding="utf-8"
    )
    print(json.dumps(result["velocity_delay_samples"], ensure_ascii=False))
    print(json.dumps(load_fit, ensure_ascii=False))


if __name__ == "__main__":
    main()
