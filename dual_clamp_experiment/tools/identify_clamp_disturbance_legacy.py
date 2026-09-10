"""ARCHIVED NONCAUSAL PROTOTYPE. Do not deploy these parameters.

This is an offline tool. It does not change the real-time controller and does not
claim to identify contact stiffness, damping, or the actuator trajectory. The
identified output is the change of the two measured force channels relative to
the local pre-event force level during release, return, and reclamp stages.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np


PHASES = (5, 6, 7)
PHASE_NAMES = {5: "release", 6: "return", 7: "reclamp"}


def read_csv(path: Path) -> List[dict]:
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def number(row: dict, key: str, default: float = np.nan) -> float:
    try:
        value = float(row.get(key, ""))
        return value if np.isfinite(value) else default
    except (TypeError, ValueError):
        return default


def load_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError):
        return {}


def parse_legacy_name(name: str) -> dict:
    out = {}
    patterns = {
        "angle_deg": r"(-?\d+(?:\.\d+)?)度",
        "speed_label": r"速度(-?\d+(?:\.\d+)?)",
        "acc_label": r"加速度(-?\d+(?:\.\d+)?)",
        "jerk_label": r"jerk(-?\d+(?:\.\d+)?)",
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, name, re.IGNORECASE)
        if match:
            out[key] = float(match.group(1))
    out["has_gripper"] = int("有夹爪" in name or "有导管" in name or "有导丝" in name)
    out["has_external_load"] = int(any(token in name for token in ("负载", "砝码", "挂", "20g", "60g", "80g", "100g", "120g")))
    out["wait_label"] = int("不等待" not in name and ("等待" in name or "等" in name))
    return out


def finite_median(values: np.ndarray, fallback: float = 0.0) -> float:
    values = values[np.isfinite(values)]
    return float(np.median(values)) if values.size else fallback


def fill_nan(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float).copy()
    good = np.isfinite(values)
    if good.all():
        return values
    if not good.any():
        return np.zeros_like(values)
    indices = np.arange(values.size)
    values[~good] = np.interp(indices[~good], indices[good], values[good])
    return values


def smooth(values: np.ndarray, window: int = 11) -> np.ndarray:
    if values.size < 3:
        return values.copy()
    window = min(window, values.size if values.size % 2 else values.size - 1)
    if window < 3:
        return values.copy()
    kernel = np.ones(window, dtype=float) / window
    padded = np.pad(values, (window // 2, window // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def derivative(values: np.ndarray, time: np.ndarray) -> np.ndarray:
    values = fill_nan(values)
    time = fill_nan(time)
    if values.size < 3:
        return np.zeros_like(values)
    dt = np.gradient(time)
    dt = np.where(np.abs(dt) > 1e-9, dt, 1e-3)
    return np.gradient(values) / dt


def event_times(events: Sequence[dict]) -> Dict[str, List[Tuple[int, float]]]:
    result: Dict[str, List[Tuple[int, float]]] = {}
    for row in events:
        name = row.get("event_name", "")
        if not name:
            continue
        cycle = int(number(row, "cycle_index", 0))
        time = number(row, "plc_time_us", np.nan) / 1e6
        if np.isfinite(time):
            result.setdefault(name, []).append((cycle, time))
    return result


def nearest_event_time(events: Dict[str, List[Tuple[int, float]]], name: str, cycle: int) -> float | None:
    candidates = [t for c, t in events.get(name, []) if c == cycle]
    return candidates[0] if candidates else None


def robust_scale(x: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    center = np.nanmedian(x, axis=0)
    scale = np.nanmedian(np.abs(x - center), axis=0) * 1.4826
    scale = np.where(np.isfinite(scale) & (scale > 1e-9), scale, 1.0)
    return np.nan_to_num(x, nan=0.0, posinf=0.0, neginf=0.0), center, scale


def ridge_fit(x: np.ndarray, y: np.ndarray, alpha: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    x_clean, center, scale = robust_scale(x)
    xs = (x_clean - center) / scale
    # A tiny diagonal term also protects against rank deficiency when one
    # experimental factor is constant in the available records.
    reg = np.eye(xs.shape[1]) * max(alpha, 0.0)
    reg[0, 0] = 1e-10
    reg += np.eye(xs.shape[1]) * 1e-10
    beta = np.linalg.solve(xs.T @ xs + reg, xs.T @ y)
    return beta, center, scale


def predict(x: np.ndarray, beta: np.ndarray, center: np.ndarray, scale: np.ndarray) -> np.ndarray:
    x = np.nan_to_num(x, nan=0.0, posinf=0.0, neginf=0.0)
    return ((x - center) / scale) @ beta


def csv_write(path: Path, rows: Iterable[dict], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def prepare_record(directory: Path, mode: str, condition: str) -> dict | None:
    meta = load_json(directory / "experiment.json")
    if meta.get("status") != "Completed":
        return None
    samples_path = directory / "samples_1khz.csv"
    events_path = directory / "events.csv"
    if not samples_path.exists() or not events_path.exists():
        return None
    if mode == "catheter" and meta.get("mode") not in ("catheter", None):
        return None
    if mode == "guidewire" and meta.get("mode") not in ("guidewire", None):
        return None

    arm_mm = float(meta.get("tangential_arm_mm", 37.0) or 37.0)
    if arm_mm <= 0:
        arm_mm = 37.0
    if mode == "catheter":
        axis, angle, moving, fixed = "axis1", "axis2_angle_deg", "cylinder2_cmd", "cylinder1_cmd"
        y_keys = ("fn1_decoupled_delta_N", "torque1_decoupled_delta_Nmm")
    else:
        axis, angle, moving, fixed = "axis6", "axis7_angle_deg", "cylinder4_cmd", "cylinder3_cmd"
        y_keys = ("fn2_decoupled_delta_N", "torque2_decoupled_delta_Nmm")

    name_values = parse_legacy_name(directory.name)
    if condition == "clamp_empty" and (not name_values["has_gripper"] or name_values["has_external_load"]):
        return None
    rows = read_csv(samples_path)
    events = event_times(read_csv(events_path))
    if not rows or not events.get("ReleaseStart"):
        return None

    t = np.asarray([number(r, "plc_time_us") / 1e6 for r in rows])
    phase = np.asarray([number(r, "phase", 0) for r in rows], dtype=int)
    cycle = np.asarray([number(r, "cycle_index", 0) for r in rows], dtype=int)
    pos = np.asarray([number(r, f"{axis}_pos_mm") for r in rows])
    vel = np.asarray([number(r, f"{axis}_vel_mm_s") for r in rows])
    acc_recorded = np.asarray([number(r, f"{axis}_acc_mm_s2") for r in rows])
    vel = fill_nan(vel)
    pos = fill_nan(pos)
    t = fill_nan(t)
    acc = acc_recorded.copy()
    if np.count_nonzero(np.isfinite(acc) & (np.abs(acc) > 1e-6)) < max(10, acc.size // 100):
        acc = derivative(smooth(vel, 11), t)
    else:
        acc = fill_nan(acc)
    jerk = derivative(smooth(acc, 11), t)
    angle_values = fill_nan(np.asarray([number(r, angle) for r in rows]))
    moving_values = fill_nan(np.asarray([number(r, moving) for r in rows]))
    fixed_values = fill_nan(np.asarray([number(r, fixed) for r in rows]))
    y = np.column_stack([fill_nan(np.asarray([number(r, key) for r in rows])) for key in y_keys])
    # The recorder stores the decoupled transverse channel as an equivalent
    # torque. Convert it back to the transverse force channel for modeling.
    y[:, 1] /= arm_mm

    samples = []
    for stage in PHASES:
        starts = events.get({5: "ReleaseStart", 6: "ReturnStart", 7: "ReclampStart"}[stage], [])
        for this_cycle, start in starts:
            # Use the measured phase label and a compact event window.  The upper
            # bound is the next event, so different stages remain separable.
            candidates = [tt for _, tt in sum(events.values(), []) if tt > start + 1e-6]
            stop = min(candidates) if candidates else start + 0.25
            mask = (phase == stage) & (cycle == this_cycle) & (t >= start) & (t < stop)
            indices = np.flatnonzero(mask)
            if indices.size < 8:
                continue
            pre = (t >= start - 0.10) & (t < start - 0.01) & (cycle == this_cycle)
            if np.count_nonzero(pre) < 5:
                pre = (t >= start - 0.15) & (t < start) & (cycle == this_cycle)
            baseline = np.median(y[pre], axis=0) if np.count_nonzero(pre) else y[indices[0]]
            rel_t = t[indices] - start
            for idx, elapsed in zip(indices, rel_t):
                samples.append({
                    "record": directory.name,
                    "mode": mode,
                    "stage": stage,
                    "cycle": int(this_cycle),
                    "time": float(t[idx]),
                    "elapsed": float(elapsed),
                    "v": float(vel[idx]),
                    "a": float(acc[idx]),
                    "jerk": float(jerk[idx]),
                    "moving": float(moving_values[idx]),
                    "fixed": float(fixed_values[idx]),
                    "angle": float(angle_values[idx]),
                    "y0": baseline,
                    "y": y[idx] - baseline,
                    "angle_label": name_values.get("angle_deg", float(angle_values[idx])),
                })
    if not samples:
        return None
    return {"directory": directory.name, "samples": samples, "meta": meta}


def feature_vector(sample: dict) -> np.ndarray:
    stage = int(sample["stage"])
    elapsed = min(max(sample["elapsed"], 0.0), 0.5)
    return np.asarray([
        1.0,
        sample["v"], abs(sample["v"]), sample["a"], sample["jerk"],
        sample["moving"], sample["fixed"],
        math.sin(math.radians(sample["angle_label"])),
        math.cos(math.radians(sample["angle_label"])),
        float(stage == 5), float(stage == 6), float(stage == 7),
        elapsed, elapsed * elapsed,
        sample["v"] * sample["a"],
    ], dtype=float)


def metrics(y: np.ndarray, yp: np.ndarray) -> dict:
    err = yp - y
    result = {}
    for j, name in enumerate(("axial_N", "transverse_tangential_N")):
        e = err[:, j]
        yy = y[:, j]
        ss = np.sum((yy - np.mean(yy)) ** 2)
        result[name] = {
            "rmse": float(np.sqrt(np.mean(e ** 2))),
            "mae": float(np.mean(np.abs(e))),
            "r2": float(1.0 - np.sum(e ** 2) / ss) if ss > 1e-12 else float("nan"),
            "max_abs_error": float(np.max(np.abs(e))) if e.size else float("nan"),
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Identify clamp-stage equivalent force disturbances.")
    parser.add_argument("--records", type=Path, default=Path(r"D:\Work_files\Vessel intervention Robot\260316\dual_clamp_experiment\x64\Debug\records"))
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--mode", choices=("catheter", "guidewire", "both"), default="both")
    parser.add_argument("--condition", choices=("clamp_empty", "all"), default="clamp_empty",
                        help="Use empty clamped records by default; use all to include no-clamp and loaded records.")
    parser.add_argument("--alpha", type=float, default=10.0)
    parser.add_argument("--test-fraction", type=float, default=0.25)
    args = parser.parse_args()
    output_root = args.output or (args.records / "identification")
    output_root.mkdir(parents=True, exist_ok=True)

    modes = ("catheter", "guidewire") if args.mode == "both" else (args.mode,)
    summary = {"records_root": str(args.records), "condition": args.condition, "models": []}
    for mode in modes:
        records = []
        for directory in sorted(p for p in args.records.iterdir() if p.is_dir()):
            record = prepare_record(directory, mode, args.condition)
            if record:
                records.append(record)
        if not records:
            summary["models"].append({"mode": mode, "status": "no_usable_records"})
            continue

        rng = np.random.default_rng(20260907)
        order = np.arange(len(records))
        rng.shuffle(order)
        test_count = max(1, int(round(len(records) * args.test_fraction))) if len(records) > 1 else 0
        test_ids = set(order[:test_count].tolist())
        train_records = [r for i, r in enumerate(records) if i not in test_ids]
        test_records = [r for i, r in enumerate(records) if i in test_ids]
        if not train_records:
            train_records, test_records = records, []

        train_samples = [s for r in train_records for s in r["samples"]]
        test_samples = [s for r in test_records for s in r["samples"]]
        x_train = np.vstack([feature_vector(s) for s in train_samples])
        y_train = np.vstack([s["y"] for s in train_samples])
        beta, center, scale = ridge_fit(x_train, y_train, args.alpha)
        x_test = np.vstack([feature_vector(s) for s in test_samples]) if test_samples else np.empty((0, x_train.shape[1]))
        y_test = np.vstack([s["y"] for s in test_samples]) if test_samples else np.empty((0, 2))
        pred_train = predict(x_train, beta, center, scale)
        pred_test = predict(x_test, beta, center, scale) if test_samples else np.empty((0, 2))

        predictions = []
        all_samples = [(s, "train") for s in train_samples] + [(s, "test") for s in test_samples]
        for (sample, split), pred in zip(all_samples, np.vstack([pred_train, pred_test])):
            predictions.append({
                "record": sample["record"], "mode": mode, "stage": PHASE_NAMES[sample["stage"]],
                "cycle": sample["cycle"], "time_s": sample["time"], "elapsed_s": sample["elapsed"],
                "v_mm_s": sample["v"], "a_mm_s2": sample["a"], "jerk_mm_s3": sample["jerk"],
                "moving_cylinder_cmd": sample["moving"], "fixed_cylinder_cmd": sample["fixed"],
                "angle_deg": sample["angle"], "set": split,
                "axial_target_delta_N": sample["y"][0], "transverse_target_delta_N": sample["y"][1],
                "axial_prediction_delta_N": pred[0], "transverse_prediction_delta_N": pred[1],
            })

        mode_dir = output_root / mode
        mode_dir.mkdir(parents=True, exist_ok=True)
        csv_write(mode_dir / "sample_predictions.csv", predictions, list(predictions[0]))
        params = {
            "mode": mode, "alpha": args.alpha, "output_names": ["axial_force_N", "transverse_tangential_force_N"], "feature_names": [
                "bias", "velocity", "abs_velocity", "acceleration", "jerk",
                "moving_cylinder_cmd", "fixed_cylinder_cmd", "sin_angle", "cos_angle",
                "release", "return", "reclamp", "elapsed", "elapsed_squared", "velocity_times_acceleration",
            ],
            "beta": beta.tolist(), "feature_center": center.tolist(), "feature_scale": scale.tolist(),
            "stages": PHASE_NAMES, "target_definition": "axial and transverse force channels minus local pre-event median",
            "transverse_force_conversion": "torque_decoupled_delta_Nmm / tangential_arm_mm",
        }
        (mode_dir / "parameters.json").write_text(json.dumps(params, ensure_ascii=False, indent=2), encoding="utf-8")
        summary_entry = {
            "mode": mode, "status": "ok", "record_count": len(records),
            "train_record_count": len(train_records), "test_record_count": len(test_records),
            "sample_count": len(train_samples) + len(test_samples),
            "train_metrics": metrics(y_train, pred_train),
            "test_metrics": metrics(y_test, pred_test) if test_samples else None,
            "train_records": [r["directory"] for r in train_records],
            "test_records": [r["directory"] for r in test_records],
        }
        summary["models"].append(summary_entry)

    (output_root / "fit_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    report = ["# Clamp disturbance identification", "", f"Records: `{args.records}`", ""]
    for model in summary["models"]:
        report.append(f"## {model['mode']}")
        if model.get("status") != "ok":
            report.append("No usable completed records were found.")
            continue
        report.append(f"Completed records used: {model['record_count']}; train/test: {model['train_record_count']}/{model['test_record_count']}.")
        for split in ("train_metrics", "test_metrics"):
            if model.get(split):
                report.append(f"{split}: `{json.dumps(model[split], ensure_ascii=False)}`")
        report.append("")
    (output_root / "model_report.md").write_text("\n".join(report), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
