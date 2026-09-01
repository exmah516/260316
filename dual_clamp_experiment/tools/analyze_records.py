import csv
import json
import math
import os
import re
import statistics
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


BASE = Path(r"D:\Work_files\Vessel intervention Robot\260316\dual_clamp_experiment\x64\Debug\records")
OUT = BASE / "analysis"
OUT.mkdir(parents=True, exist_ok=True)


def load_csv(path):
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def floats(rows, key):
    vals = []
    for row in rows:
        try:
            vals.append(float(row.get(key, "")))
        except (TypeError, ValueError):
            vals.append(float("nan"))
    return np.asarray(vals, dtype=float)


def parse_name(name):
    values = {}
    patterns = {
        "angle_deg": r"(?P<v>-?\d+(?:\.\d+)?)度",
        "speed": r"速度(?P<v>-?\d+(?:\.\d+)?)",
        "acceleration": r"加速度(?P<v>-?\d+(?:\.\d+)?)",
        "jerk": r"jerk(?P<v>-?\d+(?:\.\d+)?)",
    }
    for key, pattern in patterns.items():
        m = re.search(pattern, name, re.IGNORECASE)
        if m:
            values[key] = float(m.group("v"))
    nums = re.findall(r"(?<![A-Za-z])(?P<v>\d+(?:\.\d+)?)", name)
    if "speed" not in values and "加速度" not in name:
        for token in ("250", "500", "750", "1000", "2000"):
            if token in name:
                values.setdefault("legacy_numeric", float(token))
                break
    values["has_gripper"] = "有夹爪" in name or "有导丝" in name or "有导管" in name
    values["has_wire_label"] = "有导丝" in name or "无夹爪" in name
    values["wait_label"] = "不等待" not in name and ("等待" in name or "等" in name)
    return values


def event_rows(path):
    if not path.exists():
        return []
    return load_csv(path)


def event_time(events, name):
    for row in events:
        if row.get("event_name") == name:
            try:
                return float(row.get("plc_time_us", "0")) / 1e6
            except ValueError:
                return None
    return None


def robust_mad(x):
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    med = np.median(x)
    return float(np.median(np.abs(x - med)))


def event_metrics(t, signal, events, window=0.1):
    result = {}
    finite = np.isfinite(signal)
    if finite.sum() == 0:
        return result
    for label in ("ForwardStart", "ReleaseStart", "ReturnStart", "ReclampStart", "FinalForwardStart"):
        et = event_time(events, label)
        if et is None:
            continue
        before = signal[(t >= et - window) & (t < et) & finite]
        after = signal[(t >= et) & (t < et + window) & finite]
        if before.size == 0 or after.size == 0:
            continue
        baseline = float(np.mean(before))
        result[label] = {
            "before_mean": baseline,
            "after_mean": float(np.mean(after)),
            "delta_mean": float(np.mean(after) - baseline),
            "before_std": float(np.std(before)),
            "after_std": float(np.std(after)),
            "after_min": float(np.min(after)),
            "after_max": float(np.max(after)),
            "after_range": float(np.max(after) - np.min(after)),
            "after_mad": robust_mad(after),
        }
    return result


def plot_record(out_path, display_name, rows, events, meta):
    t = floats(rows, "plc_time_us") / 1e6
    if not np.isfinite(t).any() or np.nanmax(t) <= 0:
        t = np.arange(len(rows), dtype=float) / 1000.0
    fn = floats(rows, "fn2_decoupled_delta_N")
    ft = floats(rows, "ft2_cal_delta_N")
    pos = floats(rows, "axis6_pos_mm")
    vel = floats(rows, "axis6_vel_mm_s")
    acc = floats(rows, "axis6_acc_mm_s2")
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True, constrained_layout=True)
    axes[0].plot(t, fn, linewidth=0.7, color="#1565c0", label="fn2 解耦轴向力 (N)")
    axes[0].plot(t, ft, linewidth=0.7, color="#ef6c00", label="ft2 校正切向力 (N)")
    axes[0].set_ylabel("力 (N)")
    axes[0].legend(loc="upper right", ncol=2)
    axes[1].plot(t, pos, linewidth=0.7, color="#2e7d32", label="轴6位置")
    axes[1].set_ylabel("位置 (mm)")
    axes[1].legend(loc="upper right")
    axes[2].plot(t, vel, linewidth=0.7, color="#6a1b9a", label="轴6速度")
    axes[2].set_ylabel("速度 (mm/s)")
    axes[2].legend(loc="upper right")
    axes[3].plot(t, acc, linewidth=0.7, color="#c62828", label="轴6加速度")
    axes[3].set_ylabel("加速度 (mm/s²)")
    axes[3].set_xlabel("时间 (s)")
    axes[3].legend(loc="upper right")
    for row in events:
        try:
            et = float(row.get("plc_time_us", "0")) / 1e6
        except ValueError:
            continue
        if et <= 0:
            continue
        label = row.get("event_name", "")
        if label in {"ReleaseStart", "ReturnStart", "ReclampStart", "FinalForwardStart"}:
            for ax in axes:
                ax.axvline(et, color="#555555", alpha=0.25, linewidth=0.7)
            axes[0].text(et, 0.98, label, rotation=90, va="top", ha="right", transform=axes[0].get_xaxis_transform(), fontsize=7)
    fig.suptitle(display_name, fontsize=11)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def html_record(path, title, png_name, meta, metrics):
    rows = []
    for event, values in metrics.items():
        rows.append(
            f"<tr><td>{event}</td><td>{values['delta_mean']:.5f}</td><td>{values['after_range']:.5f}</td><td>{values['after_std']:.5f}</td><td>{values['after_mad']:.5f}</td></tr>"
        )
    status = meta.get("status", "")
    note = "正常完成" if status == "Completed" else "异常/人工终止候选，需结合事件表复核"
    body = f"""<!doctype html><meta charset='utf-8'><title>{title}</title>
<style>body{{font-family:Arial,'Microsoft YaHei',sans-serif;margin:24px;color:#222}} img{{max-width:100%;border:1px solid #ccc}} table{{border-collapse:collapse}}td,th{{border:1px solid #bbb;padding:5px 8px}} .warn{{color:#a33}}</style>
<h1>{title}</h1><p>状态：<b class='{'warn' if status != 'Completed' else ''}'>{status}</b>；判定：{note}</p>
<p>样本数：{meta.get('sample_count','')}；时长：{meta.get('duration_s','')} s；零点：{meta.get('zero_done','')}</p>
<img src='{png_name}' alt='record plot'>
<h2>事件窗口变化（事件前后各100 ms）</h2>
<table><tr><th>事件</th><th>fn2均值变化 N</th><th>fn2后窗范围 N</th><th>fn2后窗标准差 N</th><th>fn2后窗MAD N</th></tr>{''.join(rows)}</table>
<p>图中 fn2 为串扰解耦后的轴向力，ft2 为机构校正后的切向力；原始数据和其它校正列仍在同目录 CSV 中。</p>"""
    path.write_text(body, encoding="utf-8")


def main():
    records = []
    for directory in sorted(BASE.iterdir()):
        if not directory.is_dir() or directory.name == "analysis":
            continue
        jp = directory / "experiment.json"
        sp = directory / "samples_1khz.csv"
        ep = directory / "events.csv"
        if not (jp.exists() and sp.exists()):
            continue
        try:
            meta = json.loads(jp.read_text(encoding="utf-8"))
        except Exception:
            continue
        if meta.get("mode") != "guidewire":
            continue
        rows = load_csv(sp)
        if not rows:
            continue
        events = event_rows(ep)
        t = floats(rows, "plc_time_us") / 1e6
        if not np.isfinite(t).any() or np.nanmax(t) <= 0:
            t = np.arange(len(rows), dtype=float) / 1000.0
        fn = floats(rows, "fn2_decoupled_delta_N")
        ft = floats(rows, "ft2_cal_delta_N")
        fn_metrics = event_metrics(t, fn, events)
        ft_metrics = event_metrics(t, ft, events)
        name = directory.name
        stem = re.sub(r"[^A-Za-z0-9_-]+", "_", name)[:100]
        png = OUT / f"{stem}.png"
        html = OUT / f"{stem}.html"
        plot_record(png, name, rows, events, meta)
        html_record(html, name, png.name, meta, fn_metrics)
        event_return = [m for e, m in fn_metrics.items() if e == "ReturnStart"]
        event_reclamp = [m for e, m in fn_metrics.items() if e == "ReclampStart"]
        row = {
            "name": name,
            "status": meta.get("status", ""),
            "sample_count": int(meta.get("sample_count", len(rows))),
            "duration_s": float(meta.get("duration_s", 0) or 0),
            "angle_deg": parse_name(name).get("angle_deg", meta.get("zero_axis7_angle_deg", "")),
            "speed_name": parse_name(name).get("speed", parse_name(name).get("legacy_numeric", "")),
            "acceleration_name": parse_name(name).get("acceleration", ""),
            "jerk_name": parse_name(name).get("jerk", ""),
            "wait_label": parse_name(name).get("wait_label", False),
            "zero_std_fn2": float(meta.get("fn2_zero_std", 0) or 0),
            "zero_std_ft2": float(meta.get("ft2_zero_std", 0) or 0),
            "fn_return_delta_N": event_return[0]["delta_mean"] if event_return else "",
            "fn_reclamp_delta_N": event_reclamp[0]["delta_mean"] if event_reclamp else "",
            "ft_return_delta_N": ft_metrics.get("ReturnStart", {}).get("delta_mean", ""),
            "ft_reclamp_delta_N": ft_metrics.get("ReclampStart", {}).get("delta_mean", ""),
            "fn_return_range_N": event_return[0]["after_range"] if event_return else "",
            "fn_reclamp_range_N": event_reclamp[0]["after_range"] if event_reclamp else "",
            "ft_return_range_N": ft_metrics.get("ReturnStart", {}).get("after_range", ""),
            "ft_reclamp_range_N": ft_metrics.get("ReclampStart", {}).get("after_range", ""),
            "plot_html": html.name,
            "plot_png": png.name,
        }
        records.append(row)

    fields = list(records[0].keys()) if records else []
    with (OUT / "guidewire_summary.csv").open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)

    completed = [r for r in records if r["status"] == "Completed"]
    abnormal = [r for r in records if r["status"] != "Completed"]
    report = OUT / "导丝实验分析报告.md"
    lines = [
        "# 导丝模式实验记录分析",
        "",
        f"分析目录：`{BASE}`。本报告只读取原始 `experiment.json`、`samples_1khz.csv` 和 `events.csv`，未修改任何原始记录。共发现 **{len(records)}** 条导丝记录，其中状态为 Completed 的 **{len(completed)}** 条，Aborted/Error 的 **{len(abnormal)}** 条。",
        "",
        "## 记录名称可靠性",
        "目录名称只能作为人工标签，不能作为唯一参数来源。部分名称与实际 JSON/事件不一致，例如名称写 90°，JSON 中 `zero_axis7_angle_deg` 或轴7采样可能是另一值；名称中的速度、加速度、Jerk 也可能缺失或只是用户备注。正式比较应以 CSV 的实际轴6速度/加速度、事件表和 JSON 状态为准。",
        "",
        "## 完成状态与异常记录",
        "Completed 且包含 `ForwardStart → ReleaseStart → ReturnStart → ReclampStart → FinalForwardStart → RecordStop` 的记录，作为正常可比样本。Aborted/Error 记录不直接用于参数均值；如果它们已经包含完整往复事件和接近完整样本数，应标记为“动作可能已完成后人工终止”，可用于排查程序未自动收尾，但不能和正常完成样本混合统计。",
        "",
        "## 力突变分析口径",
        "对每个 `ReturnStart`（回退开始）和 `ReclampStart`（重新夹紧开始）事件，分别取事件前 100 ms 与事件后 100 ms。报告均值变化、后窗范围、标准差和 MAD。这里的“突变”不只看单点峰值：后窗范围和 MAD 用来描述事件周围持续波动；同一周期可结合 HTML 图查看力变化是否在回退前已开始、是否在重新夹紧后恢复。",
        "",
        "## 当前数据能支持的结论",
        "1. 轴向力变化应优先看 `fn2_decoupled_delta_N`，切向力变化看 `ft2_cal_delta_N`；两者都已扣除取零并经过相应标定。",
        "2. 回退相关变化需要与轴6速度、加速度和电缸3/4命令对齐。若 `ReturnStart` 附近力带明显扩大，而空载/无夹爪记录扩大较小，可认为夹持切换或回退运动共同造成附加扰动；仅凭单次记录不能区分两者贡献。",
        "3. 角度效应目前只能做描述性比较，因为尚未进行重力补偿，且部分目录名称和实际角度标签可能不一致；应以 CSV 的 `axis7_angle_deg` 和 JSON 零点角度复核。",
        "4. 加速度、Jerk、等待时间的影响必须按同一角度、同一位置窗口和同一夹持配置分组比较。当前记录中多因素同时变化、且存在 Aborted/Error，因此暂时不能给出严格因果结论，只能给出趋势和候选解释。",
        "",
        "## 生成文件",
        "- `guidewire_summary.csv`：每条导丝记录的状态、参数标签和回退/重夹紧窗口统计。",
        "- 每条记录对应一个 `.png` 和 `.html`：包含 fn2、ft2、轴6位置/速度/加速度，并标出释放、回退、重夹紧等事件。",
        "- 本报告不覆盖原始记录；异常记录仍保留在原目录。",
        "",
        "## 下一步建议",
        "后续实验应固定轴5/轴6位置、角度和电缸配合，只改变一个因素；每个条件至少保留 3 次状态为 Completed 的记录。程序应修复完成状态未自动收尾的问题，否则将继续产生“数据完整但状态 Aborted”的混杂样本。",
    ]
    report.write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({"records": len(records), "completed": len(completed), "abnormal": len(abnormal), "output": str(OUT)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
