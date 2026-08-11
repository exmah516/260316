"""绘制旧版力传感器 CSV 中的零点差值与轴1/轴2位置。

该脚本用于旧版单文件记录格式；新会话目录请优先使用 play_session.py
或 plot_force_motion.py。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
PROJECT_DIRECTORY = SCRIPT_DIRECTORY.parent if SCRIPT_DIRECTORY.name == "records" else SCRIPT_DIRECTORY
DEFAULT_INPUT = PROJECT_DIRECTORY / "Force_sensor_20260616_103210.csv"


def main() -> int:
    parser = argparse.ArgumentParser(description="绘制旧版力传感器 CSV")
    parser.add_argument("input_csv", nargs="?", type=Path, default=DEFAULT_INPUT)
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="图片输出目录；默认与输入 CSV 同目录",
    )
    args = parser.parse_args()
    input_csv = args.input_csv.resolve()
    output_dir = args.output_dir.resolve() if args.output_dir else input_csv.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(input_csv)
    required = {
        "fn_1_zero_v", "ft_1_zero_v", "fn_1_raw_v", "ft_1_raw_v",
        "tick_ms", "axis1_pos_abs_mm", "axis2_pos_abs_mm",
    }
    missing = sorted(required - set(df.columns))
    if missing:
        raise ValueError(f"CSV 缺少字段：{', '.join(missing)}")

    nonzero = (df["fn_1_zero_v"] != 0) | (df["ft_1_zero_v"] != 0)
    indices = df.index[nonzero]
    if len(indices) == 0:
        raise ValueError("未找到非零的传感器零点，请检查数据源")

    first_index = int(indices[0])
    print(f"检测到首个有效零点行：{first_index}")
    data = df.iloc[first_index:].copy()
    data["fn_delta_v"] = data["fn_1_raw_v"] - data["fn_1_zero_v"]
    data["ft_delta_v"] = data["ft_1_raw_v"] - data["ft_1_zero_v"]
    data["t_sec"] = (data["tick_ms"] - data["tick_ms"].iloc[0]) / 1000.0

    plt.rcParams["font.sans-serif"] = [
        "SimHei", "Microsoft YaHei", "Noto Sans CJK JP", "DejaVu Sans"
    ]
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["font.size"] = 12

    _draw_pair(
        data,
        signal_column="fn_delta_v",
        signal_label="FN1 电压差",
        position_column="axis1_pos_abs_mm",
        position_label="轴1绝对位置",
        position_unit="mm",
        output_path=output_dir / "d_minus_f_with_axis1.png",
    )
    _draw_pair(
        data,
        signal_column="ft_delta_v",
        signal_label="FT1 电压差",
        position_column="axis2_pos_abs_mm",
        position_label="轴2绝对位置",
        position_unit="mm",
        output_path=output_dir / "e_minus_g_with_axis2.png",
    )
    print(f"图表已保存到：{output_dir}")
    return 0


def _draw_pair(
    data: pd.DataFrame,
    signal_column: str,
    signal_label: str,
    position_column: str,
    position_label: str,
    position_unit: str,
    output_path: Path,
) -> None:
    fig, signal_axis = plt.subplots(figsize=(8, 5), dpi=300)
    signal_line = signal_axis.plot(
        data["t_sec"], data[signal_column],
        color="#1f77b4", linewidth=1.5, label=signal_label,
    )
    signal_axis.set_xlabel("时间 (s)")
    signal_axis.set_ylabel("电压差 (V)")
    signal_axis.grid(True, linestyle="--", alpha=0.4)

    position_axis = signal_axis.twinx()
    position_line = position_axis.plot(
        data["t_sec"], data[position_column],
        color="#d62728", linewidth=1.2, linestyle="--", label=position_label,
    )
    position_axis.set_ylabel(f"{position_label} ({position_unit})")
    lines = signal_line + position_line
    signal_axis.legend(lines, [line.get_label() for line in lines], loc="upper right")
    signal_axis.set_title(f"{signal_label}与{position_label}")
    fig.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
