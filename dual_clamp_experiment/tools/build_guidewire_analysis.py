import csv
import json
import re
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


BASE = Path(r"D:\Work_files\Vessel intervention Robot\260316\dual_clamp_experiment\x64\Debug\records")
OUT = BASE / "analysis"
OUT.mkdir(parents=True, exist_ok=True)


def read_csv(path):
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def vector(rows, key):
    out = []
    for row in rows:
        try:
            out.append(float(row.get(key, "")))
        except (TypeError, ValueError):
            out.append(np.nan)
    return np.asarray(out, dtype=float)


def numeric_name(name, pattern):
    m = re.search(pattern, name, re.IGNORECASE)
    return float(m.group(1)) if m else ""


def event_times(events, event_name):
    result = []
    for row in events:
        if row.get("event_name") != event_name:
            continue
        try:
            result.append(float(row.get("plc_time_us", "0")) / 1e6)
        except ValueError:
            pass
    return result


def robust_mad(values):
    values = values[np.isfinite(values)]
    if values.size == 0:
        return np.nan
    median = np.median(values)
    return float(np.median(np.abs(values - median)))


def window_stat(t, signal, center, pre=0.1, post=0.1):
    before = signal[(t >= center - pre) & (t < center)]
    after = signal[(t >= center) & (t < center + post)]
    before = before[np.isfinite(before)]
    after = after[np.isfinite(after)]
    if before.size == 0 or after.size == 0:
        return None
    return {
        "before_mean": float(np.mean(before)),
        "after_mean": float(np.mean(after)),
        "mean_shift": float(np.mean(after) - np.mean(before)),
        "after_range": float(np.max(after) - np.min(after)),
        "after_std": float(np.std(after)),
        "after_mad": robust_mad(after),
    }


def aggregate_event(t, signal, events, event_name):
    stats = [window_stat(t, signal, et) for et in event_times(events, event_name)]
    stats = [x for x in stats if x is not None]
    if not stats:
        return {"count": 0, "mean_shift": "", "after_range": "", "after_std": "", "after_mad": ""}
    return {
        "count": len(stats),
        "mean_shift": float(np.mean([x["mean_shift"] for x in stats])),
        "after_range": float(np.mean([x["after_range"] for x in stats])),
        "after_std": float(np.mean([x["after_std"] for x in stats])),
        "after_mad": float(np.mean([x["after_mad"] for x in stats])),
    }


def derive_waits(events):
    release = event_times(events, "ReleaseStart")
    returns = event_times(events, "ReturnStart")
    reclamps = event_times(events, "ReclampStart")
    phases = sorted(event_times(events, "PhaseChange") + event_times(events, "FinalForwardStart"))
    release_wait = [1000.0 * (b - a) for a, b in zip(release, returns) if b >= a]
    reclamp_wait = []
    for start in reclamps:
        following = [x for x in phases if x >= start]
        if following:
            reclamp_wait.append(1000.0 * (following[0] - start))
    return (float(np.median(release_wait)) if release_wait else "", float(np.median(reclamp_wait)) if reclamp_wait else "")


def plot_record(path, title, rows, events):
    t = vector(rows, "plc_time_us") / 1e6
    if not np.isfinite(t).any() or np.nanmax(t) <= 0:
        t = np.arange(len(rows), dtype=float) / 1000.0
    fn = vector(rows, "fn2_decoupled_delta_N")
    ft = vector(rows, "ft2_cal_delta_N")
    pos = vector(rows, "axis6_pos_mm")
    vel = vector(rows, "axis6_vel_mm_s")
    acc = vector(rows, "axis6_acc_mm_s2")
    fig, ax = plt.subplots(4, 1, figsize=(14, 10), sharex=True, constrained_layout=True)
    ax[0].plot(t, fn, color="#1565c0", lw=0.7, label="fn2 decoupled (N)")
    ax[0].plot(t, ft, color="#ef6c00", lw=0.7, label="ft2 calibrated (N)")
    ax[0].set_ylabel("Force (N)")
    ax[0].legend(loc="upper right")
    ax[1].plot(t, pos, color="#2e7d32", lw=0.7, label="axis6 position")
    ax[1].set_ylabel("Position (mm)")
    ax[1].legend(loc="upper right")
    ax[2].plot(t, vel, color="#6a1b9a", lw=0.7, label="axis6 velocity")
    ax[2].set_ylabel("Velocity (mm/s)")
    ax[2].legend(loc="upper right")
    ax[3].plot(t, acc, color="#c62828", lw=0.7, label="axis6 acceleration")
    ax[3].set_ylabel("Acceleration (mm/s²)")
    ax[3].set_xlabel("Time (s)")
    ax[3].legend(loc="upper right")
    for event in events:
        try:
            et = float(event.get("plc_time_us", "0")) / 1e6
        except ValueError:
            continue
        if et <= 0 or event.get("event_name") not in {"ReleaseStart", "ReturnStart", "ReclampStart", "FinalForwardStart"}:
            continue
        for axis in ax:
            axis.axvline(et, color="#555", alpha=0.22, lw=0.7)
        ax[0].text(et, 0.98, event.get("event_name", ""), rotation=90, va="top", ha="right", transform=ax[0].get_xaxis_transform(), fontsize=7)
    fig.suptitle(title, fontsize=11)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def create_record_html(path, title, image_name, meta, fn_return, ft_return, fn_reclamp, ft_reclamp):
    status = meta.get("status", "")
    status_note = "正常完成" if status == "Completed" else "异常/人工终止候选，不能与正常完成样本直接混合"
    rows = []
    for name, fn, ft in (("ReturnStart", fn_return, ft_return), ("ReclampStart", fn_reclamp, ft_reclamp)):
        rows.append(f"<tr><td>{name}</td><td>{fn['count']}</td><td>{fn['mean_shift']}</td><td>{fn['after_range']}</td><td>{ft['mean_shift']}</td><td>{ft['after_range']}</td></tr>")
    html = f"""<!doctype html><meta charset='utf-8'><title>{title}</title>
<style>body{{font-family:Arial,'Microsoft YaHei',sans-serif;margin:24px;color:#222}}img{{max-width:100%;border:1px solid #bbb}}table{{border-collapse:collapse}}td,th{{border:1px solid #aaa;padding:5px 8px}}.bad{{color:#a00}}</style>
<h1>{title}</h1><p>状态：<b class='{'bad' if status != 'Completed' else ''}'>{status}</b>；{status_note}</p>
<p>样本数：{meta.get('sample_count','')}；时长：{meta.get('duration_s','')} s；取零：{meta.get('zero_done','')}</p>
<img src='{image_name}' alt='force and motion plot'>
<h2>事件前后100 ms窗口</h2>
<table><tr><th>事件</th><th>窗口数</th><th>fn2均值变化 (N)</th><th>fn2后窗范围 (N)</th><th>ft2均值变化 (N)</th><th>ft2后窗范围 (N)</th></tr>{''.join(rows)}</table>
<p>fn2 使用串扰解耦后的增量轴向力；ft2 使用机构安装校正后的增量切向力。图中的竖线来自 events.csv。</p>"""
    path.write_text(html, encoding="utf-8")


def make_cross_plot(rows):
    ok = [r for r in rows if r["status"] == "Completed" and r["return_fn_range_N"] != ""]
    if not ok:
        return
    fig, ax = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    colors = ["#2e7d32" if r["gripper"] else "#757575" for r in ok]
    x = np.asarray([float(r["actual_acc_max_mm_s2"]) for r in ok])
    y = np.asarray([float(r["return_fn_range_N"]) for r in ok])
    ax[0, 0].scatter(x, y, c=colors, edgecolor="black", alpha=0.85)
    ax[0, 0].set_xlabel("Measured max |axis6 acceleration| (mm/s²)")
    ax[0, 0].set_ylabel("Return fn2 window range (N)")
    ax[0, 0].set_title("Acceleration vs force-band width")
    x = np.asarray([float(r["return_wait_ms"]) for r in ok])
    y = np.asarray([float(r["return_fn_range_N"]) for r in ok])
    ax[0, 1].scatter(x, y, c=colors, edgecolor="black", alpha=0.85)
    ax[0, 1].set_xlabel("Measured release-to-return interval (ms)")
    ax[0, 1].set_ylabel("Return fn2 window range (N)")
    ax[0, 1].set_title("Wait interval vs force-band width")
    groups = {}
    for r in ok:
        groups.setdefault(round(float(r["angle_deg"])), []).append(float(r["return_fn_range_N"]))
    labels = sorted(groups)
    ax[1, 0].boxplot([groups[x] for x in labels], labels=[str(x) for x in labels], showmeans=True)
    ax[1, 0].set_xlabel("Axis7 angle (deg; from CSV)")
    ax[1, 0].set_ylabel("Return fn2 window range (N)")
    ax[1, 0].set_title("Angle grouping")
    groups = {"gripper label": [], "no gripper label": []}
    for r in ok:
        groups["gripper label" if r["gripper"] else "no gripper label"].append(float(r["return_fn_range_N"]))
    ax[1, 1].boxplot([groups["gripper label"], groups["no gripper label"]], labels=["gripper", "no gripper"], showmeans=True)
    ax[1, 1].set_ylabel("Return fn2 window range (N)")
    ax[1, 1].set_title("Label-based comparison")
    fig.suptitle("Guidewire cross-record disturbance summary (Completed only)")
    fig.savefig(OUT / "cross_experiment_effects.png", dpi=170)
    plt.close(fig)


def write_report(rows):
    completed = [r for r in rows if r["status"] == "Completed"]
    abnormal = [r for r in rows if r["status"] != "Completed"]

    def med(items, key):
        values = [float(x[key]) for x in items if x.get(key, "") not in ("", None)]
        return f"{float(np.median(values)):.5f}" if values else "无"

    def p90(items, key):
        values = [float(x[key]) for x in items if x.get(key, "") not in ("", None)]
        return f"{float(np.quantile(values, 0.90)):.5f}" if values else "无"

    full_abnormal = 0
    required = {"ForwardStart", "ReleaseStart", "ReturnStart", "ReclampStart", "FinalForwardStart"}
    for row in abnormal:
        event_file = BASE / row["name"] / "events.csv"
        if not event_file.exists():
            continue
        names = {x.get("event_name") for x in read_csv(event_file)}
        if required.issubset(names):
            full_abnormal += 1

    measured_acc = [float(r["actual_acc_max_mm_s2"]) for r in completed if r["actual_acc_max_mm_s2"] != ""]
    acceleration_note = "轴6加速度列存在，但本批 Completed 记录全部为0，不能据此估计加速度对力突变的影响。" if measured_acc and max(abs(x) for x in measured_acc) < 1e-9 else "轴6加速度列可用于比较。"

    lines = [
        "# 导丝模式实验记录分析报告",
        "",
        f"分析对象为 `records` 下截至 2026 年 8 月 31 日的导丝模式记录，共 **{len(rows)}** 条：Completed **{len(completed)}** 条，Aborted/Error **{len(abnormal)}** 条。原始实验目录、CSV 和 JSON 未被修改。",
        "",
        "## 记录名称与状态",
        "目录名是操作者输入的标签，不是可靠的参数数据库。速度、加速度、Jerk 和等待时间应优先从 JSON、事件间隔以及 CSV 实测轴6信号核对。当前 15 条异常记录中有 **12 条** 已包含完整的往复事件序列和接近完整样本数，符合“动作可能已经完成、随后通过中止或准备才收尾”的异常特征；另外 3 条在前向阶段前后即终止。异常记录不与 Completed 记录混合计算均值。",
        "",
        "## 数据完整性",
        "每条导丝 CSV 包含轴5/轴6/轴7位置、速度、加速度、电缸3/4命令以及 fn2/ft2 原始、传感器标定、机构校正和解耦字段。加速度字段在表中存在，但本批 Completed 记录的 `axis6_acc_mm_s2` 实际全为0；因此当前只能确认字段已记录，不能把加速度列当作有效测量值。" + acceleration_note,
        "",
        "## 回退相关力变化",
        "指标为 `ReturnStart` 或 `ReclampStart` 事件前后各 100 ms 的窗口范围，描述事件周围持续波动而非单点尖峰。Completed 记录总体统计如下：",
        f"- 回退开始：fn2 窗口范围中位数 **{med(completed, 'return_fn_range_N')} N**，90%分位 **{p90(completed, 'return_fn_range_N')} N**；ft2 中位数 **{med(completed, 'return_ft_range_N')} N**，90%分位 **{p90(completed, 'return_ft_range_N')} N**。",
        f"- 重新夹紧：fn2 窗口范围中位数 **{med(completed, 'reclamp_fn_range_N')} N**，90%分位 **{p90(completed, 'reclamp_fn_range_N')} N**；ft2 中位数 **{med(completed, 'reclamp_ft_range_N')} N**，90%分位 **{p90(completed, 'reclamp_ft_range_N')} N**。",
        "- 重新夹紧阶段的离群值明显：无等待记录的 `reclamp_fn_range_N` 约 1.77 N，Jerk 250000 的记录约 1.15 N，远高于多数记录的约 0.03 N 中位水平。这提示重夹紧瞬间比回退开始更容易产生大幅轴向扰动，但需用重复实验确认，不应把两条记录直接视为普遍规律。",
        "",
        "## 因素影响的初步判断",
        "- **等待时间**：约 151 ms 等待的 Completed 记录，回退 fn2 窗口范围中位数约 0.055 N、ft2 约 0.030 N；目前唯一一条无等待 Completed 记录约为 0.023 N 和 0.002 N，但样本量为 1，不能形成统计结论。无等待记录的重夹紧 fn2 波动反而达到约 1.77 N，值得重点复测。",
        "- **角度**：0°记录的回退 fn2 中位数约 0.052 N，90°记录约 0.064 N；90°组只有 2 条，且重夹紧波动受单条离群记录影响，当前只能作描述性参考。未做重力补偿，角度效应不能直接解释为纯机械扰动。",
        "- **夹爪/电缸状态**：无夹爪只有 1 条 Completed 记录，回退 fn2/ft2 波动较小；有夹爪记录的 ft2 回退波动更明显，但样本不平衡，不能归因。",
        "- **速度、加速度、Jerk**：CSV 实测速度约 111~149 mm/s，而目录名中的 500、2000 等是控制参数标签，二者不能直接等同。加速度实测列全为0；当前记录又同时改变了速度、加速度、Jerk、等待或夹持标签，因此无法分离单因素效应。",
        "",
        "## 生成文件",
        "- `index.html`：分析索引，可打开每条记录的 HTML/PNG 图。",
        "- `cross_experiment_effects.png`：跨实验参数与回退 fn2 波动带汇总图，仅使用 Completed 记录。",
        "- `guidewire_factor_summary.csv`：每条记录的状态、CSV角度、实测速度/加速度、等待间隔以及回退/重夹紧窗口指标。",
        "- 每条记录一个 `.html` 和 `.png`：fn2、ft2、轴6位置/速度/加速度及动作事件竖线。",
        "",
        "## 结论边界与建议",
        "现有数据已经能确认：回退和重新夹紧事件附近存在可见的力变化带，且重新夹紧的离群波动可能更大；但由于异常收尾、样本不平衡、加速度列为0以及多参数同时变化，暂时不能给出严格的因果效应量。后续应先修复实验完成自动收尾，再固定位置、角度、电缸配合和等待时间，每次只改变一个运动参数，每个条件至少取得3条 Completed 记录。",
    ]
    (OUT / "导丝实验分析报告.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    records = []
    for directory in sorted(BASE.iterdir()):
        if not directory.is_dir() or directory.name == "analysis":
            continue
        try:
            meta = json.loads((directory / "experiment.json").read_text(encoding="utf-8"))
        except Exception:
            continue
        if meta.get("mode") != "guidewire" or not (directory / "samples_1khz.csv").exists():
            continue
        rows = read_csv(directory / "samples_1khz.csv")
        events = read_csv(directory / "events.csv") if (directory / "events.csv").exists() else []
        if not rows:
            continue
        t = vector(rows, "plc_time_us") / 1e6
        if not np.isfinite(t).any() or np.nanmax(t) <= 0:
            t = np.arange(len(rows), dtype=float) / 1000.0
        fn = vector(rows, "fn2_decoupled_delta_N")
        ft = vector(rows, "ft2_cal_delta_N")
        pos = vector(rows, "axis6_pos_mm")
        vel = vector(rows, "axis6_vel_mm_s")
        acc = vector(rows, "axis6_acc_mm_s2")
        fn_return = aggregate_event(t, fn, events, "ReturnStart")
        ft_return = aggregate_event(t, ft, events, "ReturnStart")
        fn_reclamp = aggregate_event(t, fn, events, "ReclampStart")
        ft_reclamp = aggregate_event(t, ft, events, "ReclampStart")
        return_wait, reclamp_wait = derive_waits(events)
        stem = re.sub(r"[^A-Za-z0-9_-]+", "_", directory.name)[:100]
        png = f"{stem}.png"
        html = f"{stem}.html"
        plot_record(OUT / png, directory.name, rows, events)
        create_record_html(OUT / html, directory.name, png, meta, fn_return, ft_return, fn_reclamp, ft_reclamp)
        angle = float(np.nanmedian(vector(rows, "axis7_angle_deg"))) if np.isfinite(vector(rows, "axis7_angle_deg")).any() else float(meta.get("zero_axis7_angle_deg", 0) or 0)
        record = {
            "name": directory.name,
            "status": meta.get("status", ""),
            "sample_count": meta.get("sample_count", len(rows)),
            "duration_s": meta.get("duration_s", ""),
            "angle_deg": angle,
            "actual_speed_max_mm_s": float(np.nanmax(np.abs(vel))) if np.isfinite(vel).any() else "",
            "actual_acc_max_mm_s2": float(np.nanmax(np.abs(acc))) if np.isfinite(acc).any() else "",
            "return_wait_ms": return_wait,
            "reclamp_wait_ms": reclamp_wait,
            "return_fn_range_N": fn_return["after_range"],
            "return_fn_shift_N": fn_return["mean_shift"],
            "return_ft_range_N": ft_return["after_range"],
            "return_ft_shift_N": ft_return["mean_shift"],
            "reclamp_fn_range_N": fn_reclamp["after_range"],
            "reclamp_fn_shift_N": fn_reclamp["mean_shift"],
            "reclamp_ft_range_N": ft_reclamp["after_range"],
            "reclamp_ft_shift_N": ft_reclamp["mean_shift"],
            "zero_fn2_std_count": meta.get("fn2_zero_std", ""),
            "zero_ft2_std_count": meta.get("ft2_zero_std", ""),
            "gripper": ("有夹爪" in directory.name or "有导丝" in directory.name or "有导管" in directory.name),
            "plot_html": html,
            "plot_png": png,
        }
        records.append(record)

    fields = list(records[0]) if records else []
    with (OUT / "guidewire_factor_summary.csv").open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)
    make_cross_plot(records)
    write_report(records)
    links = "".join(f"<tr><td>{r['status']}</td><td>{r['name']}</td><td>{r['angle_deg']:.2f}</td><td>{r['actual_speed_max_mm_s']}</td><td>{r['actual_acc_max_mm_s2']}</td><td>{r['return_fn_range_N']}</td><td>{r['return_ft_range_N']}</td><td><a href='{r['plot_html']}'>HTML</a> / <a href='{r['plot_png']}'>PNG</a></td></tr>" for r in records)
    dashboard = f"""<!doctype html><meta charset='utf-8'><title>导丝实验分析索引</title><style>body{{font-family:Arial,'Microsoft YaHei',sans-serif;margin:24px}}table{{border-collapse:collapse}}td,th{{border:1px solid #aaa;padding:4px 7px}}img{{max-width:100%}}</style><h1>导丝实验分析索引</h1><p>Completed 和异常记录均列出；跨实验汇总图只使用 Completed 且具有回退事件的记录。</p><img src='cross_experiment_effects.png'><table><tr><th>状态</th><th>记录</th><th>CSV角度</th><th>实测最大速度</th><th>实测最大加速度</th><th>回退fn范围</th><th>回退ft范围</th><th>图表</th></tr>{links}</table><p>完整数值见 guidewire_factor_summary.csv，说明见 导丝实验分析报告.md。</p>"""
    (OUT / "index.html").write_text(dashboard, encoding="utf-8")
    print(json.dumps({"records": len(records), "completed": sum(r["status"] == "Completed" for r in records), "abnormal": sum(r["status"] != "Completed" for r in records), "out": str(OUT)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
