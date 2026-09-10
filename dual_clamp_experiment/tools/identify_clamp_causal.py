"""Causal, motion-only disturbance regression and C++ parameter export.

Targets use the same installed force channels as GET_PROGRAM, not decoupled
torque. A pre-event median is a reference, never independent force ground truth.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
VERSION = "causal-motion-v1"
NAMES = ["bias", "velocity", "abs_velocity", "acceleration", "jerk",
         "moving_cmd", "fixed_cmd", "sin_angle", "cos_angle",
         "release", "return", "reclamp", "elapsed", "elapsed_squared", "velocity_acceleration"]
STAGES = (5, 6, 7)
TAU = 0.003
MAX_DT = 0.003
WARMUP = 0.010


def calibration():
    text = (ROOT / "ForceCalibration.h").read_text(encoding="utf-8-sig")
    names = ("kInstallationAxialGain", "kInstallationTorqueGainNmmPerN", "kTangentialArmMm",
             "kFn1SensorSlopeNPerCount", "kFt1SensorSlopeNPerCount", "kFn2SensorSlopeNPerCount", "kFt2SensorSlopeNPerCount")
    return [float(re.search(r"\b" + name + r"\s*=\s*([0-9.eE+-]+)", text)[1]) for name in names]


class CausalFeatures:
    """State is advanced once per new timestamp; no force input is used."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = None
        self.a = self.j = 0.0
        self.start = self.stage_start = 0.0
        self.phase = -1
        self.cycle = -1

    def update(self, t, velocity, moving, fixed, angle, phase, cycle):
        if not all(math.isfinite(x) for x in (t, velocity, moving, fixed, angle)):
            self.reset()
            return None
        if self.previous is not None and t <= self.previous[0]:
            return None
        if self.previous is None or t - self.previous[0] > MAX_DT + 1e-12:
            self.reset()
            self.start = t
        if phase != self.phase or cycle != self.cycle:
            self.stage_start = t
            self.phase, self.cycle = phase, cycle
        if self.previous is not None:
            dt = t - self.previous[0]
            alpha = dt / (TAU + dt)
            old_a = self.a
            self.a += alpha * ((velocity - self.previous[1]) / dt - self.a)
            self.j += alpha * ((self.a - old_a) / dt - self.j)
        self.previous = (t, velocity)
        if t - self.start < WARMUP - 1e-12:
            return None
        elapsed = min(max(t - self.stage_start, 0.0), 0.5)
        rad = angle * math.pi / 180.0
        return np.array([1., velocity, abs(velocity), self.a, self.j, moving, fixed,
                         math.sin(rad), math.cos(rad), float(phase == 5),
                         float(phase == 6), float(phase == 7), elapsed,
                         elapsed * elapsed, velocity * self.a])


def read_rows(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def value(row, *names):
    for key in names:
        if key in row:
            try:
                return float(row[key])
            except (TypeError, ValueError):
                return float("nan")
    return float("nan")


def load_record(directory, gains):
    meta = json.loads((directory / "experiment.json").read_text(encoding="utf-8-sig"))
    if meta.get("status") != "Completed":
        raise ValueError("not_completed")
    if meta.get("data_complete") is False or any(meta.get(k) for k in ("record_overflow", "sample_gap", "ads_error")):
        raise ValueError("recording_fault")
    name = directory.name
    if any(s in name for s in ("不好使", "故障", "失败")):
        raise ValueError("fault_label")
    if not any(s in name for s in ("有夹爪", "有导管", "有导丝")):
        raise ValueError("clamp_condition_unconfirmed")
    if any(s in name for s in ("负载", "砝码", "挂")) or re.search(r"\d+\s*g\b", name, re.I):
        raise ValueError("external_load")
    mode = meta.get("mode")
    if mode not in ("catheter", "guidewire"):
        raise ValueError("unsupported_mode")
    side, axis, rotation, moving, fixed = ((1, 1, 2, 2, 1) if mode == "catheter" else (2, 6, 7, 4, 3))
    rows = read_rows(directory / "samples_1khz.csv")
    if len(rows) < 20:
        raise ValueError("short_record")
    required = [f"fn{side}_sensor_N", f"ft{side}_sensor_N"]
    if not all(k in rows[0] for k in required):
        raise ValueError("sensor_force_fields_missing")
    old_slopes = meta.get("sensor_slopes_N_per_count", {})
    slopes = [old_slopes.get(f"fn{side}", 0), old_slopes.get(f"ft{side}", 0)]
    if not all(isinstance(s, (float, int)) and math.isfinite(s) and s > 0 for s in slopes):
        raise ValueError("sensor_calibration_missing")
    timestamp = np.array([value(r, "plc_time_us") for r in rows])
    if not np.all(np.isfinite(timestamp)) or np.any(np.diff(timestamp) <= 0):
        raise ValueError("nonmonotonic_timestamp")
    events = read_rows(directory / "events.csv")
    complete = set()
    for cycle in {int(value(e, "cycle_index")) for e in events}:
        times = {}
        for e in events:
            if int(value(e, "cycle_index")) == cycle:
                times.setdefault(e["event_name"], value(e, "plc_time_us") * 1e-6)
        ordered = [times.get(k, np.nan) for k in ("ForwardStart", "ReleaseStart", "ReturnStart", "ReclampStart")]
        if np.all(np.isfinite(ordered)) and np.all(np.diff(ordered) > 0):
            complete.add(cycle)
    if not complete:
        raise ValueError("no_complete_event_cycles")
    state = CausalFeatures()
    data, history, baselines, phase_previous = [], [], {}, None
    for row in rows:
        t = value(row, "plc_time_us") * 1e-6
        phase, cycle = int(value(row, "phase")), int(value(row, "cycle_index"))
        inputs = (t, value(row, f"axis{axis}_vel_mm_s", f"axis{axis}_velocity_mm_s"),
                  value(row, f"cylinder{moving}_cmd"), value(row, f"cylinder{fixed}_cmd"),
                  value(row, f"axis{rotation}_angle_deg"), phase, cycle)
        x = state.update(*inputs)
        # Reapply the current installed gains to already sensor-calibrated forces.
        y = np.array([value(row, required[0]) * gains[0] * gains[3 + 2*(side-1)] / slopes[0],
                      value(row, required[1]) * gains[1] / gains[2] * gains[4 + 2*(side-1)] / slopes[1]])
        key = (cycle, phase)
        if key != phase_previous:
            candidates = [yy for tt, cc, yy in history if cc == cycle and t - .100 <= tt < t - .010]
            if len(candidates) >= 5:
                baselines[key] = np.median(candidates, axis=0)
            phase_previous = key
        history = [(tt, cc, yy) for tt, cc, yy in history if tt >= t - .15]
        if np.all(np.isfinite(y)):
            history.append((t, cycle, y))
        data.append(dict(record=name, mode=mode, time=t, phase=phase, cycle=cycle,
                         inputs=inputs, x=x, raw=y, baseline=baselines.get(key),
                         usable=cycle in complete and phase in STAGES and key in baselines
                         and x is not None and np.all(np.isfinite(y))))
    # Require actual samples of all three event stages, not only event metadata.
    good_cycles = {c for c in complete if all(any(d["usable"] and d["cycle"] == c and d["phase"] == p
                                                 for d in data) for p in STAGES)}
    for d in data:
        d["usable"] = d["usable"] and d["cycle"] in good_cycles
    if not good_cycles:
        raise ValueError("insufficient_event_samples")
    return dict(name=name, mode=mode, data=data, cycles=sorted(good_cycles))


def fit(samples, alpha=10.):
    x = np.vstack([d["x"] for d in samples])
    y = np.vstack([d["raw"] - d["baseline"] for d in samples])
    center = np.median(x, axis=0)
    scale = np.std(x, axis=0)
    scale[scale < 1e-9] = 1.
    center[0], scale[0] = 0., 1.
    z = (x - center) / scale
    # Equal total influence per complete cycle, irrespective of dwell duration.
    keys = [(s["record"], s["cycle"]) for s in samples]
    counts = {k: keys.count(k) for k in set(keys)}
    weight = np.array([1. / counts[k] for k in keys])
    weight *= len(weight) / weight.sum()
    penalty = np.eye(len(NAMES)) * math.sqrt(alpha)
    penalty[0, 0] = 0.
    beta = np.linalg.lstsq(np.vstack([z * np.sqrt(weight[:, None]), penalty]),
                          np.vstack([y * np.sqrt(weight[:, None]), np.zeros((len(NAMES), 2))]),
                          rcond=None)[0]
    return dict(center=center.tolist(), scale=scale.tolist(), beta=beta.tolist(),
                feature_min=x.min(axis=0).tolist(), feature_max=x.max(axis=0).tolist(),
                rank=int(np.linalg.matrix_rank(z)))


def predict(model, x):
    if x is None or model is None:
        return np.zeros(2)
    return ((x - model["center"]) / model["scale"]) @ np.asarray(model["beta"])


def metrics(samples, model):
    if not samples:
        return None
    y = np.vstack([d["raw"] - d["baseline"] for d in samples])
    p = np.vstack([predict(model, d["x"]) for d in samples])
    out = {}
    for j, channel in enumerate(("fn_N", "ft_N")):
        error = y[:, j] - p[:, j]
        denominator = np.sum((y[:, j] - y[:, j].mean()) ** 2)
        out[channel] = dict(rmse=float(np.sqrt(np.mean(error ** 2))),
                            uncompensated_rmse=float(np.sqrt(np.mean(y[:, j] ** 2))),
                            max_abs_residual=float(np.max(np.abs(error))),
                            r2=float(1 - np.sum(error ** 2) / denominator) if denominator > 1e-15 else None)
    return out


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def json_write(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def export_header(path, models, gains, version):
    def arr(values):
        return "{" + ", ".join(format(float(v), ".17g") for v in values) + "}"
    lines = ["// Generated by tools/identify_clamp_causal.py; do not edit.",
             "#pragma once", '#include "ClampDisturbance.h"', "namespace clampmodel {",
             f'inline constexpr const char* kVersion = "{version}";',
             "inline constexpr double kCalibration[7] = " + arr(gains) + ";"]
    for mode in ("catheter", "guidewire"):
        model = models.get(mode)
        if model is None:
            lines.append(f"inline const Parameters k{mode.title()}{{}};")
        else:
            matrix = "{" + ", ".join(arr(row) for row in model["beta"]) + "}"
            lines.append(f"inline const Parameters k{mode.title()}{{true, {arr(model['center'])}, "
                         f"{arr(model['scale'])}, {matrix}}};")
    lines.append("} // namespace clampmodel")
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("both",), default="both",
                        help="Both parameter slots are exported together to prevent stale mixed versions.")
    parser.add_argument("--records", type=Path, default=ROOT / "x64/Debug/records")
    parser.add_argument("--output", type=Path, default=ROOT / "x64/Debug/records/identification_causal")
    parser.add_argument("--header", type=Path, default=ROOT / "ClampDisturbanceParameters.h")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    gains = calibration()
    audit, records = [], []
    for directory in sorted(args.records.iterdir()):
        if not (directory / "experiment.json").is_file():
            continue
        try:
            record = load_record(directory, gains)
            records.append(record)
            audit.append(dict(record=directory.name, included=True, reason="eligible", cycles=len(record["cycles"])))
        except (ValueError, OSError, KeyError) as exc:
            audit.append(dict(record=directory.name, included=False, reason=str(exc), cycles=0))
    summary = dict(version=VERSION, calibration=gains,
                   force_definition="fn_sensor_N * axial_gain; ft_sensor_N * torque_gain / arm_mm",
                   tau_s=TAU, warmup_s=WARMUP, max_dt_s=MAX_DT,
                   target="per-event pre-event median reference; not independent external force truth",
                   models={}, audit=audit)
    deployed = {}
    predictions, event_metrics = [], []
    for mode in ("catheter", "guidewire"):
        candidates = [r for r in records if r["mode"] == mode]
        if len(candidates) < 4:
            summary["models"][mode] = dict(available=False, reason="fewer_than_4_eligible_records",
                                           records=[r["name"] for r in candidates])
            continue
        rng = np.random.default_rng(20260907)
        order = rng.permutation(len(candidates))
        ntest = max(1, round(len(order) * .25))
        test_ids = set(order[:ntest].tolist())
        train = [d for i, r in enumerate(candidates) if i not in test_ids for d in r["data"] if d["usable"]]
        test = [d for i, r in enumerate(candidates) if i in test_ids for d in r["data"] if d["usable"]]
        model = fit(train)
        mode_report = dict(available=True, experimental=True, parameters=model,
                           train_records=[r["name"] for i, r in enumerate(candidates) if i not in test_ids],
                           test_records=[r["name"] for i, r in enumerate(candidates) if i in test_ids],
                           train=metrics(train, model), test=metrics(test, model),
                           complete_cycles=sum(len(r["cycles"]) for r in candidates))
        # Secondary split intentionally measures within-session repeatability only.
        all_samples = train + test
        cycle_keys = sorted({(d["record"], d["cycle"]) for d in all_samples})
        rng.shuffle(cycle_keys)
        cycle_test = set(cycle_keys[:max(1, round(len(cycle_keys) * .25))])
        ct = [d for d in all_samples if (d["record"], d["cycle"]) not in cycle_test]
        cv = [d for d in all_samples if (d["record"], d["cycle"]) in cycle_test]
        mode_report["within_session_cycle_test"] = metrics(cv, fit(ct))
        mode_report["within_session_test_cycles"] = sorted(cycle_test)
        summary["models"][mode] = mode_report
        deployed[mode] = model  # Keep the held-out set genuinely unseen by exported parameters.
        for i, r in enumerate(candidates):
            split = "test" if i in test_ids else "train"
            for d in r["data"]:
                p = predict(model, d["x"]) if d["phase"] in STAGES else np.zeros(2)
                if not np.all(np.isfinite(d["raw"])):
                    continue
                predictions.append(dict(record=r["name"], mode=mode, split=split, cycle=d["cycle"],
                    phase=d["phase"], time_s=d["time"], fn_original_N=d["raw"][0], ft_original_N=d["raw"][1],
                    fn_prediction_N=p[0], ft_prediction_N=p[1],
                    fn_corrected_N=d["raw"][0]-p[0], ft_corrected_N=d["raw"][1]-p[1],
                    input_valid=d["x"] is not None, labeled=d["usable"]))
            for c in r["cycles"]:
                for phase in STAGES:
                    samples = [d for d in r["data"] if d["usable"] and d["cycle"] == c and d["phase"] == phase]
                    result = metrics(samples, model)
                    if result:
                        for ch, vals in result.items():
                            event_metrics.append(dict(record=r["name"], mode=mode, split=split,
                                                      cycle=c, phase=phase, channel=ch, **vals))
    stamp = hashlib.sha256(json.dumps(deployed, sort_keys=True).encode()).hexdigest()[:12]
    summary["version"] = VERSION + "-" + stamp
    json_write(args.output / "fit_summary.json", summary)
    write_csv(args.output / "record_audit.csv", audit)
    write_csv(args.output / "sample_predictions.csv", predictions)
    write_csv(args.output / "event_metrics.csv", event_metrics)
    export_header(args.header, deployed, gains, summary["version"])
    for mode in ("catheter", "guidewire"):
        json_write(args.output / f"parameters_{mode}.json",
                   dict(version=summary["version"], feature_names=NAMES, calibration=gains,
                        **summary["models"][mode]))
    # Fixed-model replay fixture for C++ numerical parity, includes normal phases.
    if records:
        r = next((r for r in records if r["mode"] in deployed), records[0])
        fixture = []
        model = deployed.get(r["mode"])
        for d in r["data"]:
            if not all(math.isfinite(v) for v in d["inputs"]) or not np.all(np.isfinite(d["raw"])):
                continue
            p = predict(model, d["x"]) if d["phase"] in STAGES else np.zeros(2)
            fixture.append(dict(zip(["t","v","moving","fixed","angle","phase","cycle"], d["inputs"]),
                fn=d["raw"][0], ft=d["raw"][1], expected_fn=p[0], expected_ft=p[1],
                mode=1 if r["mode"] == "catheter" else 2,
                valid=int(model is not None and d["x"] is not None)))
        write_csv(args.output / "parity_fixture.csv", fixture)
    lines = ["# 因果扰动模型", "", f"版本：{summary['version']}",
             "仅为事件前参考水平的预测，不是独立真实外力测量。模型未使用未来样本。",
             "输出力定义与当前界面一致；原始记录未改写。按文件留出测试，部署参数未重拟合测试集。", ""]
    for mode, report in summary["models"].items():
        lines.extend([f"## {mode}", json.dumps({k: v for k, v in report.items() if k in
                     ("available", "reason", "complete_cycles", "train", "test", "within_session_cycle_test")},
                     ensure_ascii=False, indent=2)])
    (args.output / "model_report.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({mode: {k: v for k, v in report.items() if k in
                     ("available", "reason", "complete_cycles", "test")} for mode, report in summary["models"].items()},
                     ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
