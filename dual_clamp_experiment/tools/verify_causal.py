"""Verify built C++ parity/recording tests; create offline UI fixtures.

Runs only dedicated test executables. Never starts the controller or connects ADS.
"""
import csv
import json
import subprocess
from pathlib import Path
import numpy as np
from identify_clamp_causal import ROOT, read_rows, write_csv

OUT = ROOT / "x64/Debug/records/identification_causal"
BIN = ROOT / "x64/Debug_causal"


def main():
    result = subprocess.run([str(BIN / "test_clamp_causal.exe"), str(OUT / "parity_fixture.csv")],
                            check=True, capture_output=True)
    timing = json.loads(result.stdout)
    record_test = subprocess.run([str(BIN / "test_clamp_recording.exe")], check=True, capture_output=True)
    directories = [Path(p) for p in record_test.stdout.decode("utf-8").splitlines() if p.strip()]
    checked = []
    for directory in directories:
        original = read_rows(directory / "samples_1khz.csv")
        derived = read_rows(directory / "causal_force.csv")
        meta = json.loads((directory / "causal_model.json").read_text())
        assert len(original) == len(derived) == 600
        side = 1 if "catheter" in directory.name else 2
        for a, b in zip(original, derived):
            assert a["sample_index"] == b["sample_index"] and a["plc_time_us"] == b["plc_time_us"]
            for ch in ("fn", "ft"):
                np.testing.assert_allclose(float(a[f"{ch}{side}_cal_delta_N"]), float(b[f"{ch}_original_N"]), atol=1e-9)
                np.testing.assert_allclose(float(b[f"{ch}_original_N"]) - float(b[f"{ch}_prediction_N"]),
                                           float(b[f"{ch}_corrected_N"]), atol=1e-12)
                if int(b["phase"]) not in (5, 6, 7):
                    assert float(b[f"{ch}_prediction_N"]) == 0.
        assert meta["available"]
        assert meta["parameter_source"] == "guidewire"
        assert meta["cross_mode_preview"] == (side == 1)
        assert any(float(b["fn_prediction_N"]) != 0 for b in derived)
        checked.append(str(directory))
    # Explicitly synthetic catheter fixture tests unavailable-mode UI, not efficacy.
    fixture = read_rows(OUT / "parity_fixture.csv")
    for row in fixture:
        row["mode"] = "1"
        row["expected_fn"] = row["expected_ft"] = "0"
        row["valid"] = "0"
    write_csv(OUT / "synthetic_catheter_unavailable_fixture.csv", fixture)
    timing["recording_tests"] = checked
    timing["note"] = "Optimized standalone C++ timing; not PLC hard real-time or end-to-end latency."
    (OUT / "verification.json").write_text(json.dumps(timing, indent=2), encoding="utf-8")
    print(json.dumps(timing, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
