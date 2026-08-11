"""交互查看 FN1/FT1 与轴1/轴2位置。

默认打开 2026-08-04 第一次完整实验。左键拖动曲线可平移时间轴，
鼠标滚轮缩放时间窗口；可见窗口变化后 Y 轴会自动重算，便于观察局部趋势。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from play_session import _load_session, _number_column, _time_seconds, _valid_column


RECORDS_DIRECTORY = Path(__file__).resolve().parent
DEFAULT_SESSION = RECORDS_DIRECTORY / "20260804" / "224852_完整实验流程第一次"
HANDLE_RADIUS_M = 3.0 * 0.001


def _clean_fn_force(force) -> np.ndarray:
    """读取 FN1 纯净轴向力，单位 N。"""
    values = _number_column(force, "clean_force_n")
    values[~_valid_column(force, "calibrated_valid")] = np.nan
    return values


def _clean_ft_force(force) -> np.ndarray:
    """把 FT1 纯净手柄力矩还原为传感器切向力，单位 N。"""
    values = _number_column(force, "clean_handle_torque_nm") / HANDLE_RADIUS_M
    values[~_valid_column(force, "calibrated_valid")] = np.nan
    return values


class ForceMotionViewer:
    def __init__(self, session: dict, window_seconds: float):
        import matplotlib as mpl
        import matplotlib.pyplot as plt

        mpl.rcParams["font.sans-serif"] = [
            "Microsoft YaHei", "SimHei", "Arial Unicode MS", "DejaVu Sans"
        ]
        mpl.rcParams["axes.unicode_minus"] = False

        self.force = session["force"]
        self.motion = session["motion"]
        self.force_t = _time_seconds(self.force)
        self.motion_t = _time_seconds(self.motion)
        finite_times = np.concatenate((
            self.force_t[np.isfinite(self.force_t)],
            self.motion_t[np.isfinite(self.motion_t)],
        ))
        if not len(finite_times):
            raise ValueError("会话中没有有效 elapsed_us")
        self.t_min = float(np.min(finite_times))
        self.t_max = float(np.max(finite_times))
        duration = max(0.001, self.t_max - self.t_min)
        self.default_span = min(duration, max(0.5, float(window_seconds)))
        self.view_left = self.t_min
        self.view_right = min(self.t_max, self.t_min + self.default_span)
        self._drag_state = None
        self.data_lines = {}

        self.fig, (self.force_ax, self.position_ax) = plt.subplots(
            2, 1, figsize=(14, 8), sharex=True, constrained_layout=True
        )
        self.fig.canvas.manager.set_window_title(
            f"FN1/FT1 与轴1/轴2 - {session['directory'].name}"
        )
        self.force_ax.set_title("FN1 轴向力 / FT1 切向力")
        self.force_ax.set_ylabel("力 (N)")
        self.force_ax.grid(True, alpha=0.25)

        fn1 = _clean_fn_force(self.force)
        ft1 = _clean_ft_force(self.force)
        if np.isfinite(fn1).any():
            line, = self.force_ax.plot(
                self.force_t, fn1, lw=0.9, color="#1769aa", label="FN1 轴向力"
            )
            self._register_line(self.force_ax, line)
        if np.isfinite(ft1).any():
            line, = self.force_ax.plot(
                self.force_t, ft1, lw=0.9, color="#d97706", label="FT1 切向力"
            )
            self._register_line(self.force_ax, line)
        if self.force_ax.lines:
            self.force_ax.legend(loc="upper right")

        position_valid = _valid_column(self.motion, "position_valid")
        axis1 = _number_column(self.motion, "axis1_from_left_mm")
        axis2 = _number_column(self.motion, "axis2_from_left_mm")
        axis1[~position_valid] = np.nan
        axis2[~position_valid] = np.nan
        axis1_line, = self.position_ax.plot(
            self.motion_t, axis1, lw=0.9, color="#1769aa", label="轴1位置"
        )
        self.position_axis2 = self.position_ax.twinx()
        axis2_line, = self.position_axis2.plot(
            self.motion_t, axis2, lw=0.9, color="#d97706", label="轴2位置"
        )
        self.position_ax.set_title("轴1 / 轴2 位置")
        self.position_ax.set_ylabel("轴1位置 (mm)")
        self.position_axis2.set_ylabel("轴2位置 (deg)")
        self.position_ax.set_xlabel("会话时间 (s)")
        self.position_ax.grid(True, alpha=0.25)
        self.position_ax.legend(
            (axis1_line, axis2_line),
            (axis1_line.get_label(), axis2_line.get_label()),
            loc="upper right",
        )
        self._register_line(self.position_ax, axis1_line)
        self._register_line(self.position_axis2, axis2_line)

        self.axes = (self.force_ax, self.position_ax, self.position_axis2)
        self.fig.text(
            0.5, 0.005,
            "左键拖动：平移 · 滚轮：缩放时间 · 0：显示全程 · R：恢复默认窗口",
            ha="center", va="bottom",
        )
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_move)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._set_view(self.view_left, self.view_right)

    def _register_line(self, axis, line):
        self.data_lines.setdefault(axis, []).append(line)

    def _autoscale_y(self, axis):
        visible_parts = []
        for line in self.data_lines.get(axis, ()):
            x = np.asarray(line.get_xdata(), dtype=float)
            y = np.asarray(line.get_ydata(), dtype=float)
            mask = np.isfinite(x) & np.isfinite(y)
            mask &= (x >= self.view_left) & (x <= self.view_right)
            if np.any(mask):
                visible_parts.append(y[mask])
        if not visible_parts:
            return
        visible = np.concatenate(visible_parts)
        low = float(np.min(visible))
        high = float(np.max(visible))
        padding = max((high - low) * 0.08, max(abs(low), abs(high), 1.0) * 1e-4)
        axis.set_ylim(low - padding, high + padding)

    def _set_view(self, left: float, right: float):
        duration = max(0.001, self.t_max - self.t_min)
        span = min(duration, max(0.5, float(right) - float(left)))
        left = max(self.t_min, min(float(left), self.t_max - span))
        self.view_left = left
        self.view_right = left + span
        self.force_ax.set_xlim(self.view_left, self.view_right)
        self.position_ax.set_xlim(self.view_left, self.view_right)
        for axis in self.data_lines:
            self._autoscale_y(axis)
        self.fig.canvas.draw_idle()

    def _on_scroll(self, event):
        if event.inaxes not in self.axes or event.xdata is None:
            return
        old_span = self.view_right - self.view_left
        factor = 0.8 if event.button == "up" else 1.25
        duration = max(0.5, self.t_max - self.t_min)
        new_span = min(duration, max(0.5, old_span * factor))
        ratio = (float(event.xdata) - self.view_left) / old_span
        new_left = float(event.xdata) - ratio * new_span
        self._set_view(new_left, new_left + new_span)

    def _on_press(self, event):
        if event.button != 1 or event.inaxes not in self.axes:
            return
        width = max(1.0, float(event.inaxes.bbox.width))
        self._drag_state = (float(event.x), self.view_left, self.view_right, width)

    def _on_move(self, event):
        if self._drag_state is None or event.x is None:
            return
        start_x, left, right, width = self._drag_state
        shift = (float(event.x) - start_x) * (right - left) / width
        self._set_view(left - shift, right - shift)

    def _on_release(self, _event):
        self._drag_state = None

    def _on_key(self, event):
        if event.key == "0":
            self._set_view(self.t_min, self.t_max)
        elif event.key in ("r", "R"):
            self._set_view(self.t_min, self.t_min + self.default_span)

    def show(self):
        import matplotlib.pyplot as plt
        plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="交互查看 FN1/FT1 与轴1/轴2位置")
    parser.add_argument(
        "session_dir", nargs="?", type=Path, default=DEFAULT_SESSION,
        help="会话目录；省略时打开 2026-08-04 第一次完整实验",
    )
    parser.add_argument(
        "--window-seconds", type=float, default=20.0,
        help="初始时间窗口，默认 20 秒",
    )
    args = parser.parse_args()
    try:
        session = _load_session(args.session_dir)
        viewer = ForceMotionViewer(session, args.window_seconds)
    except (OSError, ValueError) as exc:
        print(f"错误：{exc}")
        return 2
    viewer.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
