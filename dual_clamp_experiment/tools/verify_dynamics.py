"""25 g生产计算核心、记录一致性及输入错误检查；不启动ADS。"""
import json
import math
from pathlib import Path
import subprocess
import tempfile

import numpy as np
from replay_clamp_dynamics import BIN, read_rows, normalize, write_csv, run_core, vector, write_ui_fixture


def main():
    run = subprocess.run([str(BIN / "test_clamp_dynamics.exe")], check=True, capture_output=True)
    result = json.loads(run.stdout)
    illustration = subprocess.run([str(BIN / "test_clamp_illustration.exe")], check=True, capture_output=True)
    result["model2_tests"] = illustration.stdout.decode("utf-8").strip()
    run = subprocess.run([str(BIN / "test_clamp_recording.exe")], check=True, capture_output=True)
    directories = [Path(p) for p in run.stdout.decode("utf-8").splitlines() if p]
    model2_reference = {}
    for directory in directories:
        snapshot = json.loads((directory / "causal_model.json").read_text(encoding="utf-8"))
        assert snapshot["mass_kg"] == .025 and not snapshot["axial_sign_verified"]
        assert not snapshot["ft_compensation_enabled"] and not snapshot["velocity_fallback"]
        assert snapshot["disturbance_intercept_N"] == 0
        rows = read_rows(directory / "causal_force.csv")
        original = read_rows(directory / "samples_1khz.csv")
        assert len(rows) == len(original) == 600
        for row, raw in zip(rows, original):
            assert row["sample_index"] == raw["sample_index"] and row["plc_time_us"] == raw["plc_time_us"]
            assert row["ft_corrected_N"] == row["ft_original_N"] and float(row["ft_prediction_N"]) == 0
            assert row["model_version"] == snapshot["version"]
            a = float(row["feedback_acceleration_mm_s2"])
            sensor = snapshot["axial_sign"] * .025 * a * .001
            assert abs(sensor - float(row["sensor_prediction_N"])) < 1e-12
            display = sensor * snapshot["installation_axial_gain"]
            expected = display if row["model_gate"] == "1" else 0
            assert abs(float(row["fn_prediction_N"])-expected) < 1e-12
            assert abs(float(row["fn_corrected_N"]) - (float(row["fn_original_N"])-expected)) < 1e-12
            assert row["physics_verified"] == "0"
        inputs, metadata, _ = normalize(directory)
        normalized = directory / "verification_input.csv"
        write_csv(normalized, inputs)
        replayed = run_core(normalized, directory / "verification_replay.csv",
                            snapshot["installation_axial_gain"], snapshot["axial_sign"], snapshot["validation_mode"])
        for key in ("fn_prediction_N", "ft_prediction_N", "fn_corrected_N", "ft_corrected_N", "model_gate", "model_valid",
                    "sensor_prediction_N", "display_prediction_N", "used_acceleration_m_s2"):
            np.testing.assert_allclose(vector(rows, key), vector(replayed, key), atol=1e-10)
        m2 = read_rows(directory / "model2_illustration.csv")
        current = vector(m2, "fn_illustration_N")
        if metadata["mode"] in model2_reference:
            np.testing.assert_array_equal(current, model2_reference[metadata["mode"]])
        model2_reference[metadata["mode"]] = current
        write_ui_fixture(directory / "ui_fixture.csv", inputs, replayed, metadata["mode"])
    # 缺少元数据、列或非数加速度必须暴露，不能猜测标定或改用差分。
    with tempfile.TemporaryDirectory(prefix="inertia25g_") as temporary:
        temp = Path(temporary)
        (temp / "experiment.json").write_text('{"mode":"catheter"}', encoding="utf-8")
        try:
            normalize(temp)
            raise AssertionError("missing gain accepted")
        except KeyError:
            pass
        bad = dict(sample_index=0, time_s=0, velocity_mm_s=0, moving_cmd=0, fixed_cmd=0,
                   phase=6, cycle=1, acceleration_mm_s2=math.nan, force_valid=1,
                   fn_original_N=1, ft_original_N=2)
        write_csv(temp / "input.csv", [bad])
        invalid = run_core(temp / "input.csv", temp / "output.csv", 2.85)[0]
        assert invalid["model_valid"] == "0" and float(invalid["fn_prediction_N"]) == 0
        assert invalid["model_status"] == "invalid_acceleration"
        del bad["acceleration_mm_s2"]
        write_csv(temp / "input.csv", [bad])
        missing = subprocess.run([str(BIN / "test_clamp_dynamics.exe"), "--replay",
                                  str(temp / "input.csv"), str(temp / "missing_output.csv"),
                                  "2.85", "1", "0"], capture_output=True)
        assert missing.returncode != 0 and b"missing replay column" in missing.stderr
        # 标定层级不一致必须报错；不得以当前常量替换历史增益。
        example = dict(original[0])
        example["fn2_cal_delta_N"] = "123"
        (temp / "experiment.json").write_text(json.dumps(metadata), encoding="utf-8")
        write_csv(temp / "samples_1khz.csv", [example])
        try:
            normalize(temp)
            raise AssertionError("calibration mismatch accepted")
        except ValueError as error:
            assert "标定关系不一致" in str(error)
    result["recording_tests"] = [str(p) for p in directories]
    result["hardware_connected"] = False
    result["model2_independent_of_sign_and_validation"] = True
    (BIN / "verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        if error.stderr:
            print(error.stderr.decode("utf-8", errors="replace"))
        raise
