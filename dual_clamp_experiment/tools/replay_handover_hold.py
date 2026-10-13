"""Replay the production C++ tracker; independently check the spatial policy.

Figure contract: compare raw and held display, including the discontinuity at
recovery. Two stacked channels preserve force/torque units; no physical
disturbance rejection or estimation accuracy is implied by a flat held trace.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import random
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
SEED = 20261013
COLS = ["sample_index", "plc_time_us", "phase", "cycle_index", "axis1_from_left_mm",
        "fn1_cal_delta_N", "torque1_cal_delta_Nmm", "position_reference_valid"]


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def runs(values):
    starts = np.r_[0, np.flatnonzero(np.diff(values) != 0) + 1]
    return [(int(a), int(b), int(values[a])) for a, b in zip(starts, np.r_[starts[1:], len(values)])]


def oracle(frame, trigger):
    """按整段位置和滚动窗口独立求解，不复制逐点状态机。"""
    phase = frame.phase.to_numpy()
    x = frame.axis1_from_left_mm.to_numpy()
    force = frame.fn1_cal_delta_N.to_numpy()
    torque = frame.torque1_cal_delta_Nmm.to_numpy()
    time = frame.plc_time_us.to_numpy() * 1e-6
    expected = np.column_stack([force, torque])
    expected[(phase == 0) | (phase >= 10)] = np.nan
    holding = np.zeros(len(frame), dtype=bool)
    cycles = []
    spans = runs(phase)
    for k, (start, end, p) in enumerate(spans):
        if p != 4 or k + 1 >= len(spans) or spans[k + 1][2] != 5:
            continue
        remaining = x[start:end] - trigger
        inside = (remaining >= 2) & (remaining <= 5)
        window = pd.Series(force[start:end]).rolling(50)
        stable = (window.max() - window.min() <= .1 + 1e-12).to_numpy()
        stable = stable & (pd.Series(inside.astype(int)).rolling(50).sum().to_numpy() == 50)
        choices = np.flatnonzero(stable | (remaining <= 2))
        assert len(choices), "record cannot reach sampling window"
        anchor = start + choices[0]
        source = "stable_window" if stable[choices[0]] else "forced_at_2mm"
        progress = np.maximum.accumulate(x[anchor:end][0] - x[anchor:end])
        picks = anchor + np.searchsorted(progress + 1e-9, np.arange(1, 21) * .05)
        assert picks[-1] < end and len(np.unique(picks)) == 20, "insufficient spatial samples"
        mean = np.array([force[picks].mean(), torque[picks].mean()])
        boundary = next((a for a, b, q in spans[k + 1:] if q == 0 or q >= 10), len(frame))
        next_forward = next(((a, b) for a, b, q in spans[k + 1:] if q in (4, 9) and a < boundary), None)
        release = boundary
        if next_forward:
            a, b = next_forward
            reached = np.flatnonzero(x[a] - x[a:b] >= 2 - 1e-9)
            if len(reached):
                release = a + reached[0]
        expected[end:release] = mean
        holding[end:release] = True
        raw = np.column_stack([force, torque])[end:release]
        recovered = release < boundary
        cycles.append({
            "cycle": len(cycles) + 1, "source": source,
            "reference_start_index": int(frame.sample_index.iloc[anchor]),
            "sample_indices": frame.sample_index.iloc[picks].astype(int).tolist(),
            "reference_fn_N": mean[0], "reference_torque_Nmm": mean[1],
            "sample_window_s": time[picks[-1]] - time[anchor],
            "hold_start_row": end, "hold_end_row_exclusive": release,
            "hold_start_s": time[end],
            "hold_end_s": time[release] if release < len(time) else time[-1] + .001,
            "hold_duration_s": (release - end) * .001,
            "recovered": recovered,
            "raw_fn_peak_to_peak_N": np.ptp(raw[:, 0]),
            "raw_torque_peak_to_peak_Nmm": np.ptp(raw[:, 1]),
            "held_fn_peak_to_peak_N": 0.0, "held_torque_peak_to_peak_Nmm": 0.0,
            "entry_jump_fn_N": abs(mean[0] - force[end - 1]),
            "entry_jump_torque_Nmm": abs(mean[1] - torque[end - 1]),
            "recovery_jump_fn_N": abs(force[release] - mean[0]) if recovered else np.nan,
            "recovery_jump_torque_Nmm": abs(torque[release] - mean[1]) if recovered else np.nan,
        })
    return expected, holding, cycles


def production(frame, trigger, executable):
    payload = frame[["plc_time_us", "sample_index", "phase", "cycle_index", "axis1_from_left_mm"]].copy()
    payload["trigger"] = trigger
    payload["force"] = frame.fn1_cal_delta_N
    payload["torque"] = frame.torque1_cal_delta_Nmm
    payload["valid"] = frame.position_reference_valid
    payload["catheter"] = 1
    result = subprocess.run([str(executable), "--replay"], input=payload.to_csv(index=False, header=False, sep=" "),
                            text=True, encoding="utf-8", capture_output=True, check=True)
    return pd.read_csv(io.StringIO(result.stdout))


def export_figure(frame, result, cycles, title, base, zoom=None):
    from audit_panel_alignment import require_matplotlib_panel_alignment
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    fig.subplots_adjust(left=.095, right=.98, bottom=.11, top=.82, hspace=.24)
    fig.suptitle(title, fontsize=11, y=.975)
    time = frame.plc_time_us.to_numpy() * 1e-6
    for ax, raw, held, label in zip(axes, [frame.fn1_cal_delta_N, frame.torque1_cal_delta_Nmm],
                                    [result.handover_fn_N, result.handover_torque_Nmm], ["轴向力 (N)", "扭矩 (N·mm)"]):
        ax.plot(time, raw, color="#0072B2", lw=.6, label="原始实测")
        ax.plot(time, held, color="#111111", lw=1.2, label="换手冻结显示")
        for cycle in cycles:
            ax.axvspan(cycle["hold_start_s"], cycle["hold_end_s"], color="#999999", alpha=.15, lw=0)
            if cycle["recovered"]:
                ax.axvline(cycle["hold_end_s"], color="#D55E00", lw=.75, ls="--")
        ax.set_ylabel(label)
        ax.grid(axis="y", color="#dddddd", lw=.45)
        ax.margins(x=0)
        if zoom:
            ax.set_xlim(*zoom)
            mask = (time >= zoom[0]) & (time <= zoom[1])
            values = np.r_[np.asarray(raw)[mask], np.asarray(held)[mask]]
            lo, hi = np.nanmin(values), np.nanmax(values)
            pad = max(.02, .08 * (hi - lo))
            ax.set_ylim(lo - pad, hi + pad)
    axes[-1].set_xlabel("PLC 记录时间 (s)")
    handles, labels = axes[0].get_legend_handles_labels()
    handles += [Patch(facecolor="#dddddd"), plt.Line2D([], [], color="#D55E00", ls="--", lw=.8)]
    labels += ["冻结时段", "前送满 2 mm，恢复实测"]
    fig.legend(handles, labels, ncol=4, loc="upper center", bbox_to_anchor=(.54, .93), frameon=False)
    fig.canvas.draw()
    require_matplotlib_panel_alignment(fig, json_out=str(base) + ".alignment.json", strict=True)
    fig.savefig(str(base) + ".png", dpi=300)
    fig.savefig(str(base) + ".pdf", dpi=300)
    fig.savefig(str(base) + ".svg")
    plt.close(fig)


def ui_fixture(frame, result, arm, destination):
    """沿用现有 UI 回放协议前23列，追加7列冻结字段。"""
    # 优先显示第一次恢复，因为该处可能暴露最大的跳变。
    released = np.flatnonzero(result.handover_status == "resumed")
    end = min(len(frame), int(released[0]) + 700) if len(released) else len(frame)
    data = frame.iloc[max(0, end - 9500):end].reset_index(drop=True)
    held = result.iloc[max(0, end - 9500):end].reset_index(drop=True)
    fixture = pd.DataFrame(np.zeros((len(data), 30)), columns=[f"c{i}" for i in range(30)])
    fixture["c0"] = data.plc_time_us * 1e-6
    fixture["c5"] = data.phase
    fixture["c7"] = fixture["c13"] = fixture["c17"] = data.fn1_cal_delta_N
    fixture["c8"] = fixture["c14"] = fixture["c18"] = data.torque1_cal_delta_Nmm / arm
    for column in (11, 12, 15, 16):
        fixture[f"c{column}"] = 1
    fixture["c23"] = held.handover_fn_N.fillna(0)
    fixture["c24"] = held.handover_torque_Nmm.fillna(0) / arm
    for output, source in ((25, "handover_valid"), (26, "handover_reference_valid"), (27, "handover_holding")):
        fixture[f"c{output}"] = held[source].astype(int)
    fixture["c28"] = held.handover_source.map({"none": 0, "stable_window": 1, "forced_at_2mm": 2})
    states = ["inactive", "live", "searching", "sampling", "reference_ready", "holding", "resumed",
              "no_reference", "invalid_input", "sample_gap", "ended", "spatial_undersampled"]
    fixture["c29"] = held.handover_status.map({state: i for i, state in enumerate(states)})
    # 协议布尔及整数使用1/0字面量，不输出1.0。
    for column in (5, 11, 12, 15, 16, 19, 20, 21, 22, 25, 26, 27, 28, 29):
        fixture[f"c{column}"] = fixture[f"c{column}"].astype(int)
    fixture.to_csv(destination, index=False)
    for width in (13, 17, 23):
        fixture.iloc[:, :width].to_csv(destination.with_name(f"ui_legacy_{width}.csv"), index=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--records", type=Path, default=ROOT / "x64/Debug/records")
    parser.add_argument("--output", type=Path, default=ROOT / "analysis/handover_hold_20261013")
    parser.add_argument("--executable", type=Path, default=ROOT / "x64/Debug_handover/verification/test_handover_hold.exe")
    parser.add_argument("--figure-tools", type=Path, default=Path.home() / ".codex/skills/nature-figure/scripts")
    args = parser.parse_args()
    sys.path.insert(0, str(args.figure_tools))
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["Microsoft YaHei", "DejaVu Sans"],
                         "font.size": 9, "pdf.fonttype": 42, "svg.fonttype": "none",
                         "axes.spines.top": False, "axes.spines.right": False, "axes.unicode_minus": False})
    eligible = sorted(path for path in args.records.iterdir() if path.is_dir()
                      and "20261009_140151" <= path.name[:15] <= "20261009_184620"
                      and (path / "samples_1khz.csv").is_file()
                      and (path / "samples_1khz.csv").stat().st_size > 1000)
    selected = sorted(random.Random(SEED).sample(eligible, 5))
    args.output.mkdir(parents=True, exist_ok=True)
    summaries, all_cycles, manifest = [], [], []
    for record in selected:
        path = record / "samples_1khz.csv"
        hashes = {p.name: sha256(p) for p in record.glob("*.csv")}
        metadata = json.loads((record / "experiment.json").read_text(encoding="utf-8-sig"))
        assert metadata["mode"] == "catheter", "axis1/channel1 validation only"
        frame = pd.read_csv(path, usecols=COLS)
        assert np.all(np.diff(frame.plc_time_us) == 1000) and np.all(np.diff(frame.sample_index) == 1)
        assert frame.position_reference_valid.eq(1).all() and np.isfinite(frame.to_numpy()).all()
        trigger = metadata["axis1_trigger_from_left_mm"]
        result = production(frame, trigger, args.executable)
        expected, holding, cycles = oracle(frame, trigger)
        np.testing.assert_array_equal(result.sample_index, frame.sample_index)
        np.testing.assert_array_equal(result.handover_holding.to_numpy().astype(bool), holding)
        np.testing.assert_allclose(result[["handover_fn_N", "handover_torque_Nmm"]], expected, atol=1e-11, rtol=1e-11, equal_nan=True)
        for cycle in cycles:
            at_hold = result.iloc[cycle["hold_start_row"]]
            assert at_hold.handover_source == cycle["source"] and at_hold.handover_sample_count == 20
            assert at_hold.handover_reference_start_index == cycle["reference_start_index"]
            cycle["record"] = record.name
        destination = args.output / record.name[:15]
        destination.mkdir(exist_ok=True)
        combined = pd.concat([frame, result.drop(columns="sample_index")], axis=1)
        combined.to_csv(destination / "replay.csv", index=False)
        pd.DataFrame(cycles).drop(columns="sample_indices").to_csv(destination / "cycles.csv", index=False)
        serializable = [{key: None if isinstance(value, float) and not np.isfinite(value) else value
                         for key, value in cycle.items()} for cycle in cycles]
        (destination / "sample_indices.json").write_text(json.dumps(serializable, ensure_ascii=False, indent=2,
                                                                   allow_nan=False, default=lambda x: x.item()), encoding="utf-8")
        export_figure(frame, result, cycles, record.name, destination / "force_torque_hold")
        first_recovery = next((c["hold_end_s"] for c in cycles if c["recovered"]), None)
        if first_recovery is not None:
            export_figure(frame, result, cycles, record.name + " | 首次恢复局部", destination / "first_recovery",
                          (first_recovery - .3, first_recovery + .5))
        ui_fixture(frame, result, metadata["tangential_arm_mm"], destination / "ui_fixture.csv")
        rows = pd.DataFrame(cycles)
        summaries.append({"record": record.name, "samples": len(frame), "holds": len(rows),
                          "stable_windows": int(rows.source.eq("stable_window").sum()),
                          "forced_windows": int(rows.source.eq("forced_at_2mm").sum()),
                          "recoveries": int(rows.recovered.sum()),
                          "median_hold_s": rows.hold_duration_s.median(),
                          "max_recovery_jump_fn_N": rows.recovery_jump_fn_N.max(),
                          "max_recovery_jump_torque_Nmm": rows.recovery_jump_torque_Nmm.max(),
                          "median_raw_hold_peak_to_peak_fn_N": rows.raw_fn_peak_to_peak_N.median(),
                          "median_raw_hold_peak_to_peak_torque_Nmm": rows.raw_torque_peak_to_peak_Nmm.median(),
                          "cpp_python_parity": True})
        after = {p.name: sha256(p) for p in record.glob("*.csv")}
        assert hashes == after, "original CSV changed"
        manifest.append({"record": record.name, "sha256_before": hashes, "sha256_after": after,
                         "sampling_Hz": 1000, "mode": metadata["mode"], "trigger_mm": trigger})
        all_cycles.extend(cycles)
        print(f"{record.name}: {len(frame)} samples, {len(cycles)} holds, parity PASS", flush=True)
    summary = pd.DataFrame(summaries)
    summary.to_csv(args.output / "summary.csv", index=False)
    pd.DataFrame(all_cycles).drop(columns="sample_indices").to_csv(args.output / "all_cycles.csv", index=False)
    (args.output / "manifest.json").write_text(json.dumps({"seed": SEED, "eligible_count": len(eligible),
        "selected_count": len(selected), "production_header_sha256": sha256(ROOT / "HandoverHold.h"),
        "figure_contract": "Paired raw/held comparison; frozen display is not physical disturbance rejection. Full traces and first recovery; no decimation.",
        "inputs": manifest}, ensure_ascii=False, indent=2), encoding="utf-8")
    table = "| 记录时间 | 稳定/强制 | 恢复次数 | 最大恢复跳变 N | 最大恢复跳变 N·mm |\n|---|---:|---:|---:|---:|\n"
    for row in summaries:
        table += f"| {row['record'][9:15]} | {row['stable_windows']}/{row['forced_windows']} | {row['recoveries']} | {row['max_recovery_jump_fn_N']:.4f} | {row['max_recovery_jump_torque_Nmm']:.4f} |\n"
    cycles = pd.DataFrame(all_cycles)
    report = f"""# 换手冻结显示验证

验证范围：固定种子 {SEED}，在2026年10月9日14:01:51至18:46:20的{len(eligible)}个有效记录中随机抽取5组，全部为导管模式、轴1、通道1；文件名中的导丝不改变该硬件映射。

实现结果：生产C++与独立Python按段回放逐点一致，共{int(summary.samples.sum())}帧、25次冻结、20次恢复。PLC时间间隔全部1000微秒。所有输入CSV的SHA-256前后相同。原始标定数值直接使用，不再额外乘1.7。

{table}
恢复跳变中位数：轴向{cycles.recovery_jump_fn_N.median():.5f} N，扭矩{cycles.recovery_jump_torque_Nmm.median():.4f} N·mm。排除每组首次冻结后的恢复，其余恢复最大跳变：{cycles.loc[cycles.cycle > 1, 'recovery_jump_fn_N'].max():.4f} N、{cycles.loc[cycles.cycle > 1, 'recovery_jump_torque_Nmm'].max():.4f} N·mm。首次与后续差异保留在图中，未剔除异常大跳变。

每轮在剩余5至2毫米内寻找连续50帧轴向峰峰值不超过0.1 N的窗口；之后前进1毫米，按每0.05毫米取一个真实帧，共20帧，两通道同时取值。未找到稳定窗时，剩余2毫米处开始采样。到Phase4进入Phase5才开始冻结，持续至下一段Phase4或Phase9实际前送满2毫米。实际步进跨越多个空间采样格时本轮参考无效，不重复使用同一帧。

冻结期内显示峰峰值为零，这是保持常数的定义，不是力估计精度或真实扰动抑制率。轴向与扭矩的真实变化都被隐藏；正常前送其余部分不处理，仍可有高频波动。恢复为直接切回实测值，首轮可能出现较大跳变。末轮没有后续前送时标记为未恢复，在终止采样处结束曲线，不制造回接。

界面：导管模式右侧轴向力图图例中的“换手冻结显示”，默认关闭，黑线同时用于轴向与ft比较。勾选时比较曲线保留原始实测，暂不叠加周期脉冲替代；取消后恢复原有显示选择。程序持续保存独立派生列，勾选只控制显示。扭矩以N·mm记录，界面ft沿用37毫米力臂换算N。

记录：`replay.csv`保留输入列和新增冻结列；`cycles.csv`记录窗口、均值、跳变与来源；`sample_indices.json`给出20个真实采样索引。`force_torque_hold`为完整上下通道对比，`first_recovery`为首次恢复局部，均提供PNG和矢量PDF。`manifest.json`保存随机抽样、输入哈希及生产代码版本哈希。

本次只验证离线数据与软件，未启动机器人或实机验证。PLC动作、原始测量、旧模型和手柄反馈通道未接入冻结输出。
"""
    (args.output / "REPORT.md").write_text(report, encoding="utf-8")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
