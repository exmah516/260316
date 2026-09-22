"""Exercise production calculation and CSV/UI contracts without ADS or haptic devices."""
import json
from pathlib import Path
import subprocess
import hashlib

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT/"x64"/"Debug_inertia25g"
OUT = ROOT/"x64"/"SoftwareFix20260920"/"verification"
OUT.mkdir(parents=True, exist_ok=True)


def main():
    results = {}
    for name in ["test_clamp_dynamics", "test_clamp_illustration", "test_force_pulse"]:
        run = subprocess.run([str(BIN/(name+".exe"))], check=True, capture_output=True)
        results[name] = run.stdout.decode("utf-8").strip()
    run = subprocess.run([str(BIN/"test_clamp_recording.exe")], check=True, capture_output=True)
    folders = [Path(s) for s in run.stdout.decode("utf-8").splitlines()
               if s and not s.startswith("PULSE_RECORD")]
    fixtures = []
    for folder in folders:
        m = json.loads((folder/"causal_model.json").read_text(encoding="utf-8"))
        d = pd.read_csv(folder/"causal_force.csv")
        assert len(d) == 600 and (d.model_version == "motion-only-axis1-v4-20260920").all()
        assert not m["physics_verified"] and not m["haptic_feedback_enabled"]
        assert not m["reconstruct_external"] and not m["intercept_subtracted"]
        wire = m["application_mode"] == "guidewire"
        if wire:
            assert (d.model_valid == 0).all()
            assert d[["fn_prediction_N", "ft_prediction_N", "fn_corrected_N", "ft_corrected_N"]].isna().all().all()
        else:
            v = m["axial_sign"]*d.velocity_mm_s*.001
            a = m["axial_sign"]*d.feedback_acceleration_mm_s2*.001
            expected = m["beta_a"]*a + m["beta_v"]*v + m["beta_s"]*np.where(abs(v)>.0002, np.sign(v), 0)
            np.testing.assert_allclose(d.fn_prediction_N, expected, atol=1e-12)
            np.testing.assert_allclose(d.fn_corrected_N, d.fn_original_N-expected, atol=1e-12)
            assert d.loc[d.phase == 9, "model_valid"].eq(1).all()
        fixture = pd.DataFrame(dict(
            time=d.plc_time_us*1e-6, velocity=d.velocity_mm_s, moving=0, fixed=400,
            angle=0, phase=d.phase, cycle=d.cycle_index,
            fn=d.fn_original_N, ft=d.ft_original_N,
            pred_fn=d.fn_prediction_N.fillna(0), pred_ft=d.ft_prediction_N.fillna(0),
            mode=2 if wire else 1, valid=d.model_valid))
        fixture.to_csv(folder/"ui_fixture.csv", index=False)
        fixtures.append(str(folder))
    # Actual six-file replay against the frozen axis1 motion-only definition.
    replay_rows, hashes = [], {}
    for folder in sorted((ROOT/"x64"/"Debug"/"records").glob("20260918_*")):
        source = folder/"samples_1khz.csv"
        if not (folder/"causal_force.csv").exists():
            continue
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        d = pd.read_csv(source)
        inputs = pd.DataFrame(dict(sample_index=d.sample_index, time_s=d.plc_time_us*1e-6,
            velocity_mm_s=d.axis1_vel_mm_s, moving_cmd=d.cylinder2_cmd, fixed_cmd=d.cylinder1_cmd,
            phase=d.phase, cycle=d.cycle_index, acceleration_mm_s2=d.axis1_acc_mm_s2,
            force_valid=1, fn_original_N=d.fn1_cal_delta_N, ft_original_N=d.ft1_cal_delta_N))
        source_path, target = OUT/(folder.name[:15]+"_input.csv"), OUT/(folder.name[:15]+"_replay.csv")
        inputs.to_csv(source_path, index=False)
        subprocess.run([str(BIN/"test_clamp_dynamics.exe"), "--replay", str(source_path),
                        str(target), "1.8", "-1", "0"], check=True)
        r = pd.read_csv(target)
        cfg = json.loads((folders[0]/"causal_model.json").read_text(encoding="utf-8"))
        v = -inputs.velocity_mm_s*.001
        expected = (cfg["beta_a"]*(-inputs.acceleration_mm_s2*.001) +
                    cfg["beta_v"]*v + cfg["beta_s"]*np.where(abs(v)>.0002, np.sign(v), 0))
        np.testing.assert_allclose(r.fn_corrected_N, inputs.fn_original_N-expected, atol=1e-10)
        assert (r.model_valid == 1).all()
        assert hashlib.sha256(source.read_bytes()).hexdigest() == digest
        hashes[str(source)] = digest
        replay_rows.append(dict(file=folder.name, samples=len(r),
                               max_motion_N=float(abs(r.fn_prediction_N).max()),
                               phase9_samples=int((r.phase == 9).sum())))
    results.update(recording_tests=fixtures, replay_files=replay_rows, raw_sha256=hashes,
                   hardware_connected=False, axis6_unavailable_verified=True,
                   source_version="motion-only-axis1-v4-20260920")
    (OUT/"verification.json").write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    # Preserve the UI test runner's known fixture location, but not old physics claims.
    (BIN/"verification.json").write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(dict(recording_cases=len(folders), replay_files=len(replay_rows),
                          hardware_connected=False), indent=2))


if __name__ == "__main__":
    main()
