"""核对当前手柄外源记录的结构、数值、五轴字段及事件；只读取离线结果。"""
import argparse
import csv
import json
import math
from pathlib import Path


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        assert key not in result, f"Duplicate JSON key: {key}"
        result[key] = value
    return result


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"), object_pairs_hook=unique_object)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("verification", type=Path)
    args = parser.parse_args()
    report = read_json(args.verification / "record_tests.json")
    directory = Path(report["record_directory"])
    metadata = read_json(directory / "experiment.json")
    assert metadata["mode"] == "external_validation"
    assert metadata["record_schema"] == "external-validation-v2"
    assert metadata["status"] == "Completed" and metadata["data_complete"]
    assert metadata["sample_count"] == report["sample_count"]
    assert metadata["axis6_total_forward_mm"] == 60
    assert metadata["axis6_expected_end_from_left_mm"] == 391
    assert metadata["final_forward_distance_mm"] == 0
    assert metadata["external_motion"]["control"] == "handle"
    assert metadata["external_motion"]["serial"] == 582
    assert metadata["external_motion"]["axis7_copies_axis2"]
    assert metadata["cylinder1_coupling_enabled"] and not metadata["cylinder3_coupling_enabled"]
    assert metadata["external_reference"]["assumed_accurate"]
    assert not metadata["external_reference"]["accuracy_verified"]
    assert not metadata["external_reference"]["model_compensated"]
    assert "force_pulse_guard" not in metadata
    model = read_json(directory / "causal_model.json")
    assert model["application_mode"] == "external_validation" and not model["validation_mode"]
    with (directory / "samples_1khz.csv").open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        assert len(reader.fieldnames) == len(set(reader.fieldnames)) == 61
        samples = list(reader)
    with (args.verification / "plc_trace.csv").open(encoding="utf-8", newline="") as stream:
        trace = list(csv.DictReader(stream))
    assert len(samples) == report["sample_count"]
    assert len(samples) == len(trace)
    assert int(samples[-1]["phase"]) == 10 and int(samples[-1]["cycle_index"]) == 3
    assert all(int(row["phase"]) != 9 for row in samples)
    held_axis5 = None
    for i, row in enumerate(samples):
        assert None not in row and all(value is not None for value in row.values())
        assert int(row["sample_index"]) == i and int(row["plc_time_us"]) == i * 1000
        assert int(row["cylinder4_cmd"]) == 500 and int(row["cylinder3_cmd"]) == 400
        assert row["fn1_raw"] != row["fn2_raw"] and row["ft1_raw"] != row["ft2_raw"]
        for axis in (1, 2, 5, 6, 7):
            unit = "deg" if axis in (2, 7) else "mm"
            for quantity, suffix in (("pos", unit), ("vel", unit + "_s"), ("acc", unit + "_s2")):
                assert math.isclose(float(row[f"axis{axis}_{quantity}_{suffix}"]),
                                    float(trace[i][f"axis{axis}_{quantity}"]), abs_tol=1e-9)
        assert float(row["axis7_pos_deg"]) == float(row["axis2_pos_deg"])
        if int(row["phase"]) in (5, 6, 7):
            if held_axis5 is None:
                held_axis5 = float(row["axis5_pos_mm"])
            assert math.isclose(float(row["axis5_pos_mm"]), held_axis5, abs_tol=1e-9)
        else:
            held_axis5 = None
        if int(row["phase"]) in (5, 6, 7):
            assert float(row["axis6_vel_mm_s"]) == 0 and int(row["sync_state"]) in (0, 4)
        if int(row["phase"]) == 6:
            assert int(row["cylinder1_cmd"]) == 5 and int(row["cylinder2_cmd"]) == 5
        for side in (1, 2):
            f = float(row[f"fn{side}_cal_delta_N"])
            t = float(row[f"torque{side}_cal_delta_Nmm"])
            assert math.isclose(float(row[f"fn{side}_decoupled_delta_N"]), .996063*f + .037854*t, abs_tol=1e-10)
            assert math.isclose(float(row[f"torque{side}_decoupled_delta_Nmm"]), -.103597*f + .996063*t, abs_tol=1e-10)
    with (directory / "causal_force.csv").open(encoding="utf-8", newline="") as stream:
        derived = list(csv.DictReader(stream))
    assert len(derived) == len(samples)
    assert [x["plc_time_us"] for x in derived] == [x["plc_time_us"] for x in samples]
    with (directory / "events.csv").open(encoding="utf-8", newline="") as stream:
        events = list(csv.DictReader(stream))
    for name in ("AwaitHandleBaseline", "HandleFollowing", "ReferenceHold"):
        assert sum(x["event_name"] == name for x in events) == 3
    assert not any("Gear" in event["event_name"] for event in events)
    for name in ("ReleaseStart", "ReturnStart", "ReclampStart"):
        assert sum(x["event_name"] == name for x in events) == 3
    print(json.dumps({"result": "passed", "rows": len(samples), "columns": 61,
                      "events": len(events), "hardware_connected": False}))


if __name__ == "__main__":
    main()
