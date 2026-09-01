import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_guidewire_cycles import (
    BASE,
    build_cycles,
    derive_acceleration,
    event_map,
    force_response,
    read_csv,
    segment_profile,
    value,
)


def analyze(directory, axis, fn_key, ft_key):
    meta = json.loads((directory / "experiment.json").read_text(encoding="utf-8"))
    rows = read_csv(directory / "samples_1khz.csv")
    events = event_map(read_csv(directory / "events.csv"))
    t = value(rows, "plc_time_us") / 1e6
    if not np.isfinite(t).any() or np.nanmax(t) <= 0:
        t = np.arange(len(rows), dtype=float) / 1000.0
    pos = value(rows, f"axis{axis}_pos_mm")
    vel = value(rows, f"axis{axis}_vel_mm_s")
    rec_acc = value(rows, f"axis{axis}_acc_mm_s2")
    _, smooth_acc = derive_acceleration(t, vel)
    fn = value(rows, fn_key)
    ft = value(rows, ft_key)
    cycles = []
    for base_cycle in build_cycles(events):
        forward = segment_profile(t, pos, vel, smooth_acc, base_cycle["forward"], base_cycle["release"])
        ret = segment_profile(t, pos, vel, smooth_acc, base_cycle["return"], base_cycle["reclamp"])
        next_boundary = base_cycle["next_boundary"] if np.isfinite(base_cycle["next_boundary"]) else base_cycle["reclamp"]
        rf = force_response(t, fn, base_cycle["return"], base_cycle["release"], base_cycle["reclamp"])
        rt = force_response(t, ft, base_cycle["return"], base_cycle["release"], base_cycle["reclamp"])
        cf = force_response(t, fn, base_cycle["reclamp"], base_cycle["return"], next_boundary)
        ct = force_response(t, ft, base_cycle["reclamp"], base_cycle["return"], next_boundary)
        cycles.append({"forward": forward, "return": ret, "rf": rf, "rt": rt, "cf": cf, "ct": ct})
    return meta, rows, events, rec_acc, smooth_acc, cycles


def fmedian(cycles, group, key):
    vals = [c[group][key] for c in cycles if c[group].get(key, "") not in ("", None) and np.isfinite(float(c[group][key]))]
    return float(np.median(vals)) if vals else np.nan


def main():
    for mode, axis, fn_key, ft_key in (("catheter", 1, "fn1_decoupled_delta_N", "ft1_cal_delta_N"), ("guidewire", 6, "fn2_decoupled_delta_N", "ft2_cal_delta_N")):
        print("\n===", mode, "axis", axis)
        for directory in sorted(BASE.iterdir()):
            if not directory.is_dir() or directory.name == "analysis":
                continue
            jp = directory / "experiment.json"
            if not jp.exists():
                continue
            try:
                meta = json.loads(jp.read_text(encoding="utf-8"))
            except Exception:
                continue
            if meta.get("mode") != mode or meta.get("status") != "Completed":
                continue
            try:
                m, rows, events, rec_acc, smooth_acc, cycles = analyze(directory, axis, fn_key, ft_key)
            except Exception as exc:
                print("ERROR", directory.name, exc)
                continue
            angle = np.nanmedian(value(rows, f"axis{7 if mode == 'guidewire' else 2}_angle_deg"))
            print(directory.name)
            print("  cycles", len(cycles), "angle", round(float(angle), 3), "sample", m.get("sample_count"), "set_return_v", m.get("return_velocity_mm_s"), "set_return_a", m.get("return_acceleration_mm_s2"))
            print("  actual return v", round(fmedian(cycles, "return", "velocity_peak_mm_s"), 3), "a95", round(fmedian(cycles, "return", "acceleration_p95_mm_s2"), 1), "a_peak", round(fmedian(cycles, "return", "acceleration_peak_mm_s2"), 1))
            print("  return fn range", round(fmedian(cycles, "rf", "range"), 5), "ft range", round(fmedian(cycles, "rt", "range"), 5), "reclamp fn range", round(fmedian(cycles, "cf", "range"), 5), "ft range", round(fmedian(cycles, "ct", "range"), 5))


if __name__ == "__main__":
    main()
