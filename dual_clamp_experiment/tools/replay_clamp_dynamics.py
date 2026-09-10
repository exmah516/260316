"""25 g反馈惯性离线回放；生产C++核心计算，历史文件只读。"""
import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / "x64/Debug_inertia25g"
VERSION = "inertia-feedback-v2-m025"


def read_rows(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    if not rows:
        raise ValueError("没有可写入的回放记录")
    with Path(path).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def numeric(row, *names):
    for name in names:
        if name in row:
            return float(row[name]) if row[name].strip() else float("nan")
    raise ValueError("缺少字段：" + " / ".join(names))


def normalize(directory):
    directory = Path(directory)
    metadata = json.loads((directory / "experiment.json").read_text(encoding="utf-8-sig"))
    mode = metadata["mode"]
    if mode not in ("catheter", "guidewire"):
        raise ValueError("只支持导管/导丝记录")
    gain = float(metadata["installation_axial_gain"])
    if not math.isfinite(gain) or gain <= 0:
        raise ValueError("缺少有效安装标定增益")
    side, axis, moving, fixed = (1, 1, 2, 1) if mode == "catheter" else (2, 6, 4, 3)
    rows = read_rows(directory / "samples_1khz.csv")
    if not rows:
        raise ValueError("空记录")
    result = []
    calibration_error = 0
    for row in rows:
        fn = numeric(row, f"fn{side}_cal_delta_N")
        ft = numeric(row, f"ft{side}_cal_delta_N")
        sensor = numeric(row, f"fn{side}_sensor_N")
        if math.isfinite(fn) and math.isfinite(sensor):
            calibration_error = max(calibration_error, abs(fn - gain * sensor))
        result.append(dict(
            sample_index=int(row["sample_index"]), time_s=float(row["plc_time_us"]) * 1e-6,
            velocity_mm_s=numeric(row, f"axis{axis}_vel_mm_s", f"axis{axis}_velocity_mm_s"),
            moving_cmd=numeric(row, f"cylinder{moving}_cmd"),
            fixed_cmd=numeric(row, f"cylinder{fixed}_cmd"),
            phase=int(row["phase"]), cycle=int(row["cycle_index"]),
            acceleration_mm_s2=numeric(row, f"axis{axis}_acc_mm_s2", f"axis{axis}_acceleration_mm_s2"),
            force_valid=int(all(math.isfinite(x) for x in (fn, ft, sensor))),
            fn_original_N=fn, ft_original_N=ft))
    if calibration_error > 1e-8:
        raise ValueError(f"传感器层与显示层标定关系不一致：{calibration_error:g} N")
    return result, metadata, calibration_error


def run_core(normalized, output, gain, sign=1, validation=False):
    subprocess.run([str(BIN / "test_clamp_dynamics.exe"), "--replay", str(normalized),
                    str(output), repr(gain), str(sign), str(int(validation))], check=True)
    return read_rows(output)


def vector(rows, key):
    return np.array([float(row[key]) for row in rows])


def correlation(a, b):
    finite = np.isfinite(a) & np.isfinite(b)
    if finite.sum() < 3 or np.std(a[finite]) < 1e-12 or np.std(b[finite]) < 1e-12:
        return None
    return float(np.corrcoef(a[finite], b[finite])[0, 1])


def segments(inputs, predictions):
    t, v, a = (vector(inputs, key) for key in ("time_s", "velocity_mm_s", "acceleration_mm_s2"))
    f, r = (vector(predictions, key) for key in ("fn_original_N", "fn_corrected_N"))
    # 此阈值只用于报告分段，不参与模型计算，不删除任何采样。
    finite_a = np.abs(a[np.isfinite(a)])
    threshold = max(1.0, .02 * float(finite_a.max())) if len(finite_a) else 1.0
    labels = []
    for velocity, acc in zip(v, a):
        if not (math.isfinite(velocity) and math.isfinite(acc)):
            labels.append("invalid")
        elif abs(velocity) < .1 and abs(acc) <= threshold:
            labels.append("stationary")
        elif abs(acc) <= threshold:
            labels.append("constant_speed")
        else:
            labels.append("accelerating" if velocity * acc >= 0 else "decelerating")
    result = []
    start = 0
    for end in range(1, len(inputs) + 1):
        if end < len(inputs) and labels[end] == labels[start] and inputs[end]["cycle"] == inputs[start]["cycle"]:
            continue
        index = slice(start, end)
        entry = dict(kind=labels[start], cycle=inputs[start]["cycle"], start_s=float(t[start]),
                     end_s=float(t[end-1]), samples=end-start,
                     velocity_mean_mm_s=float(np.mean(v[index])),
                     acceleration_peak_mm_s2=float(np.max(np.abs(a[index]))),
                     residual_mean_N=float(np.mean(r[index])), residual_std_N=float(np.std(r[index])),
                     original_mean_N=float(np.mean(f[index])), original_std_N=float(np.std(f[index])),
                     acceleration_force_same_sample_correlation=correlation(a[index], f[index]),
                     peak_force_minus_acceleration_time_ms=None)
        if end-start >= 3 and labels[start] in ("accelerating", "decelerating"):
            # 峰值时差仅是描述指标，未对曲线或模型输入作平移。
            ia = start + int(np.argmax(np.abs(a[index])))
            local_force = f[index] - f[start]
            if np.all(np.isfinite(local_force)):
                iff = start + int(np.argmax(np.abs(local_force)))
                entry["peak_force_minus_acceleration_time_ms"] = float((t[iff]-t[ia])*1000)
        result.append(entry)
        start = end
    return result, threshold


def speed_groups(all_segments):
    groups = {}
    for segment in all_segments:
        if segment["kind"] != "constant_speed" or segment["end_s"]-segment["start_s"] < .02:
            continue
        speed = segment["velocity_mean_mm_s"]
        key = (1 if speed >= 0 else -1, round(abs(speed)/5)*5)
        groups.setdefault(key, []).append(segment)
    return [dict(direction=key[0], speed_bin_mm_s=key[1], segment_count=len(values),
                 residual_mean_N=float(np.mean([s["residual_mean_N"] for s in values])),
                 between_segment_std_N=float(np.std([s["residual_mean_N"] for s in values])))
            for key, values in sorted(groups.items())]


def plot_record(path, inputs, predictions, gain):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = vector(inputs, "time_s")
    fig, axes = plt.subplots(4, 1, figsize=(12, 8), sharex=True, constrained_layout=True)
    axes[0].plot(t, vector(predictions, "fn_original_N"), label="Original installed delta", color="#147d75", lw=.8)
    axes[0].plot(t, vector(predictions, "fn_corrected_N"), label="25 g processed", color="#dc3456", lw=.8)
    axes[0].set_ylabel("Display fn (N)")
    axes[1].plot(t, vector(predictions, "fn_original_N") / gain, label="Original sensor delta", color="#147d75", lw=.8)
    axes[1].plot(t, vector(predictions, "sensor_prediction_N"), label="Ungated sensor prediction", color="#dc3456", lw=.8)
    axes[1].set_ylabel("Sensor fn (N)")
    axes[2].plot(t, vector(inputs, "acceleration_mm_s2"), color="#7753a0", label="Recorded ActAcc; no added filter", lw=.8)
    axes[2].set_ylabel("a (mm/s^2)")
    axes[3].plot(t, vector(inputs, "velocity_mm_s"), color="#397aa8", label="Recorded velocity", lw=.8)
    axes[3].set_ylabel("v (mm/s)")
    axes[3].set_xlabel("Nominal PLC sample time (s), no shift")
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(loc="upper right", fontsize=8)
    fig.suptitle("25 g inertia preview | sign +1 UNVERIFIED | full-record computational replay")
    fig.savefig(path, dpi=140)
    plt.close(fig)


def write_ui_fixture(path, inputs, predictions, mode):
    result = []
    for row, model in zip(inputs, predictions):
        if not all(math.isfinite(float(model[key])) for key in ("fn_original_N", "ft_original_N")):
            continue
        result.append(dict(t=row["time_s"], v=row["velocity_mm_s"], moving=row["moving_cmd"],
                           fixed=row["fixed_cmd"], angle=0, phase=row["phase"], cycle=row["cycle"],
                           fn=row["fn_original_N"], ft=row["ft_original_N"],
                           expected_fn=model["fn_prediction_N"], expected_ft=0,
                           mode=2 if mode == "guidewire" else 1, valid=model["model_valid"],
                           m2fn=model["model2_fn_N"], m2ft=model["model2_ft_N"],
                           m2fnvalid=model["model2_fn_valid"], m2ftvalid=model["model2_ft_valid"]))
    write_csv(path, result)


def replay_record(directory, output):
    source = Path(directory)
    tracked = [p for p in source.iterdir() if p.is_file()]
    before = {p.name: digest(p) for p in tracked}
    inputs, meta, calibration_error = normalize(source)
    target = output / source.name
    target.mkdir()
    normalized = target / "normalized_input.csv"
    write_csv(normalized, inputs)
    variants = {}
    for validation in (False, True):
        for sign in (1, -1):
            name = ("full" if validation else "normal") + ("_plus" if sign == 1 else "_minus")
            variants[name] = run_core(normalized, target / f"{name}.csv",
                                      float(meta["installation_axial_gain"]), sign, validation)
    for prefix in ("normal", "full"):
        plus, minus = variants[prefix+"_plus"], variants[prefix+"_minus"]
        np.testing.assert_allclose(vector(plus, "fn_prediction_N"), -vector(minus, "fn_prediction_N"), atol=1e-12)
    for rows in variants.values():
        np.testing.assert_allclose(vector(rows, "ft_original_N"), vector(rows, "ft_corrected_N"), equal_nan=True)
        # 模型2独立于质量、符号和验证门控，不参与以下物理模型统计。
        np.testing.assert_allclose(vector(rows, "model2_fn_N"), vector(variants["normal_plus"], "model2_fn_N"), equal_nan=True)
    chosen = variants["full_plus"]
    seg, threshold = segments(inputs, chosen)
    write_csv(target / "segments.csv", seg)
    plot_record(target / "comparison.png", inputs, chosen, float(meta["installation_axial_gain"]))
    write_ui_fixture(target / "ui_fixture.csv", inputs, variants["normal_plus"], meta["mode"])
    t, v, a = (vector(inputs, key) for key in ("time_s", "velocity_mm_s", "acceleration_mm_s2"))
    dt = np.diff(t)
    backward = np.divide(np.diff(v), dt, out=np.full_like(dt, np.nan), where=dt > 0)
    after = {p.name: digest(p) for p in tracked}
    if before != after:
        raise AssertionError("历史源文件发生变化")
    summary = dict(record=source.name, mode=meta["mode"], samples=len(inputs), model_version=VERSION,
                   mass_kg=.025, sign=1, sign_verified=False, physics_verified=False,
                   historical_open_jaw_conditions_verified=False, full_gate_is_computational_test_only=True,
                   installation_axial_gain=meta["installation_axial_gain"],
                   calibration_max_error_N=calibration_error, source_hashes=before, source_unchanged=True,
                   time_basis="sample index * nominal 1 ms; hardware synchronization unknown",
                   nominal_dt_min_ms=float(dt.min()*1000), nominal_dt_max_ms=float(dt.max()*1000),
                   acceleration_all_zero=bool(np.all(a == 0)),
                   acceleration_velocity_difference_correlation=correlation(a[1:], backward),
                   velocity_from_acceleration_rmse_mm_s=float(np.sqrt(np.mean((v[0]+np.cumsum(np.r_[0,a[1:]*dt])-v)**2))),
                   segmentation_acceleration_threshold_mm_s2=threshold,
                   max_abs_sensor_prediction_N=float(np.max(np.abs(vector(chosen, "sensor_prediction_N")))),
                   max_abs_display_prediction_N=float(np.max(np.abs(vector(chosen, "display_prediction_N")))),
                   speed_groups=speed_groups(seg))
    (target / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("records", nargs="*", type=Path)
    args = parser.parse_args()
    records = args.records or [ROOT / "x64/Debug/records" / name for name in
        ("20260907_194143_experiment", "20260907_194244_experiment",
         "20260907_194347_experiment", "20260907_194424_experiment")]
    if not args.records:
        records += sorted((ROOT / "x64/Debug/records").glob("20260831_214022_*"))
    output = BIN / "offline" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True)
    summaries = [replay_record(directory, output) for directory in records]
    (output / "replay_summary.json").write_text(json.dumps(summaries, ensure_ascii=False, indent=2), encoding="utf-8")
    report = ["# 25 g 惯性模型离线回放", "",
              "本报告仅验证计算链路，不代表已完成无器械、夹爪张开实机验证。",
              "所有处理直接使用记录的反馈加速度；无差分兜底、无新增滤波、无尖峰剔除、无时间平移。",
              "原始文件SHA-256回放前后相同。模型2未进入模型辨识或残差评价。",
              "", "## 数值与标定", "",
              "|记录|模式|样本|标定增益|最大传感器预测 N|最大显示增量 N|", "|---|---|---:|---:|---:|---:|"]
    for s in summaries:
        report.append(f"|{s['record']}|{s['mode']}|{s['samples']}|{s['installation_axial_gain']}|"
                      f"{s['max_abs_sensor_prediction_N']:.6f}|{s['max_abs_display_prediction_N']:.6f}|")
    report += ["", "## 反馈与同步", "",
               "PLC G.axis[1]映射NC Axis 6，G.axis[6]映射NC Axis 11。记录读取NcToPlc.ActAcc/ActVelo。",
               "运动与力在同一PLC记录循环中复制，但力来自EtherCAT输入；PLC记录时间为采样序号×1000 us，"
               "没有各传感器的硬件采样时刻，不能确认内部滤波、刷新延迟或物理同步。",
               "速度后向差分和加速度积分只用于一致性诊断，不用作补偿输入。峰值时差仅描述，不修正时间。",
               ""]
    for s in summaries:
        corr = s["acceleration_velocity_difference_correlation"]
        report.append(f"- {s['record']}：加速度全零={s['acceleration_all_zero']}；"
                      f"与速度差分同点相关={corr if corr is not None else '不可计算'}；"
                      f"积分速度RMSE={s['velocity_from_acceleration_rmse_mm_s']:.3f} mm/s。")
    report += ["", "## 分段检查", "",
               "每条记录另存normal/full与±1四种计算结果；full仅测试全程门控，不证明历史记录符合无器械条件。",
               "segments.csv逐段给出加速/减速/匀速/静止残差、同点相关和峰值时差；所有采样均保留。",
               "匀速统计按方向和5 mm/s速度组汇总，至少20 ms的连续匀速段才用于组间比较；"
               "分段阈值单独记录，短段仍在segments.csv中。没有足够重复段时不宣称存在速度阻力。",
               "本次不拟合速度项或有效质量，也不依据幅值减小或平滑程度选择符号。",
               "", "## 未验证假设", "",
               "两侧25 g组件与基座同步运动、惯性载荷经过轴向传感器、轴与力的正方向、反馈加速度特性、"
               "真实时间同步仍未验证。真正验证需在人工确认无器械、夹爪全程张开条件下进行轴向往复，"
               "重复比较加减速的符号、幅值和时间对应，以及多速度匀速残差。",
               ""]
    for s in summaries:
        report += [f"## {s['record']}", "", f"![原始与预测对照]({s['record']}/comparison.png)", ""]
    (output / "report.md").write_text("\n".join(report), encoding="utf-8")
    print(str(output))


if __name__ == "__main__":
    main()
