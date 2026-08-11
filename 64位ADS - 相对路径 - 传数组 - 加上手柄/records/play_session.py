"""实验会话同步播放器。

用法：
    python records\\play_session.py "records\\20260729\\195045_测试"
    python records\\play_session.py "records\\20260729\\195045_测试" --check

播放器只读取会话文件，不修改任何 CSV 或视频文件。视频解码使用
opencv-python；没有视频或没有第二路力字段时，界面会显示明确提示。
曲线区支持左键拖动时间轴、滚轮缩放；默认只显示 20 秒，避免整段实验压缩在一屏。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd

try:
    import cv2
except ImportError:  # 允许在只有 CSV 的机器上使用 --check
    cv2 = None


def _number_column(frame: pd.DataFrame, name: str) -> np.ndarray:
    """读取数值列；旧会话没有的列统一返回 NaN。"""
    if name not in frame.columns:
        return np.full(len(frame), np.nan, dtype=float)
    # pandas 可能返回只读 NumPy 视图；后续需原地写入 NaN，因此统一复制。
    return pd.to_numeric(frame[name], errors="coerce").to_numpy(dtype=float).copy()


def _valid_column(frame: pd.DataFrame, name: str) -> np.ndarray:
    if name not in frame.columns:
        return np.zeros(len(frame), dtype=bool)
    values = frame[name].astype(str).str.strip().str.lower()
    return values.isin(("1", "true", "yes")).to_numpy(dtype=bool)


def _time_seconds(frame: pd.DataFrame) -> np.ndarray:
    return _number_column(frame, "elapsed_us") / 1_000_000.0


def _finite_time(values: Iterable[np.ndarray]) -> tuple[float, float]:
    points = [x[np.isfinite(x)] for x in values if len(x)]
    points = [x for x in points if len(x)]
    if not points:
        return 0.0, 1.0
    return float(min(x.min() for x in points)), float(max(x.max() for x in points))


def _interval_summary(t: np.ndarray) -> str:
    d = np.diff(t) * 1000.0
    d = d[np.isfinite(d) & (d > 0)]
    if not len(d):
        return "无有效时间戳"
    return (
        f"中位 {np.median(d):.3f} ms, P95 {np.percentile(d, 95):.3f} ms, "
        f"最大 {d.max():.3f} ms"
    )


def _derivative(t: np.ndarray, x: np.ndarray, gap_limit_s: float = 0.1) -> np.ndarray:
    """用相邻有效位置计算速度，长时间断点不跨越。"""
    result = np.full(len(x), np.nan, dtype=float)
    if len(x) < 2:
        return result
    dt = np.diff(t)
    dx = np.diff(x)
    good = np.isfinite(dt) & (dt > 0) & (dt <= gap_limit_s)
    good &= np.isfinite(dx)
    result[1:][good] = dx[good] / dt[good]
    return result


def _clean_or_raw(
    frame: pd.DataFrame,
    clean_name: str,
    raw_name: str,
    valid_name: str = "calibrated_valid",
) -> tuple[np.ndarray, str]:
    clean = _number_column(frame, clean_name)
    valid = _valid_column(frame, valid_name)
    clean[~valid] = np.nan
    if np.isfinite(clean).sum() >= 2:
        return clean, "清洗值"
    return _number_column(frame, raw_name), "原始电压"


def _force_columns(channel: int) -> tuple[str, str, str, str, str, str]:
    if channel == 1:
        return (
            "clean_force_n",
            "clean_handle_torque_nm",
            "fn_1_raw_v",
            "ft_1_raw_v",
            "FN1",
            "FT1",
        )
    return (
        "clean_force_2_n",
        "clean_handle_torque_2_nm",
        "fn_2_raw_v",
        "ft_2_raw_v",
        "FN2",
        "FT2",
    )


def _load_session(session_dir: Path) -> dict:
    if not session_dir.is_dir():
        raise FileNotFoundError(f"会话目录不存在：{session_dir}")
    force_path = session_dir / "force.csv"
    motion_path = session_dir / "motion.csv"
    if not force_path.exists() or not motion_path.exists():
        raise FileNotFoundError("会话目录必须包含 force.csv 和 motion.csv")
    force = pd.read_csv(force_path, na_values=("NaN", "nan", "NAN"))
    motion = pd.read_csv(motion_path, na_values=("NaN", "nan", "NAN"))
    metadata = {}
    metadata_path = session_dir / "session.json"
    if metadata_path.exists():
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            metadata = {}

    frames = pd.DataFrame()
    frames_path = session_dir / "video_frames.csv"
    if frames_path.exists():
        frames = pd.read_csv(frames_path)

    return {
        "directory": session_dir,
        "force": force,
        "motion": motion,
        "metadata": metadata,
        "frames": frames,
        "video_path": session_dir / "video.mp4",
    }


def _print_check(session: dict) -> int:
    force = session["force"]
    motion = session["motion"]
    frames = session["frames"]
    force_t = _time_seconds(force)
    motion_t = _time_seconds(motion)
    frame_t = (
        _number_column(frames, "callback_elapsed_us") / 1_000_000.0
        if len(frames)
        else np.array([], dtype=float)
    )
    start, end = _finite_time((force_t, motion_t, frame_t))
    print(f"会话：{session['directory']}")
    print(f"时间范围：{start:.3f} - {end:.3f} s ({end - start:.3f} s)")
    print(f"force.csv：{len(force)} 行，{_interval_summary(force_t)}")
    print(f"motion.csv：{len(motion)} 行，{_interval_summary(motion_t)}")
    if len(frames):
        print(f"video_frames.csv：{len(frames)} 行，{_interval_summary(frame_t)}")
    else:
        print("video_frames.csv：缺失")

    common = 0
    if len(force_t) and len(motion_t):
        common = len(set(np.rint(force_t * 1_000_000).astype(np.int64)) &
                     set(np.rint(motion_t * 1_000_000).astype(np.int64)))
    print(f"力/位置精确相同时间戳：{common}/{len(motion_t)}")

    for channel in (1, 2):
        clean_force, clean_torque, raw_force, raw_torque, fn_label, ft_label = _force_columns(channel)
        present = raw_force in force.columns and raw_torque in force.columns
        clean_values = _number_column(force, clean_force)
        clean_valid = _valid_column(force, "calibrated_valid")
        clean_present = bool(np.isfinite(clean_values[clean_valid]).any())
        print(
            f"{fn_label}/{ft_label}："
            f"{'存在' if present else '缺失'}，"
            f"{'有有效清洗值' if clean_present else '无有效清洗值'}"
        )

    video_path = session["video_path"]
    if not video_path.exists():
        print("视频：缺失")
    elif cv2 is None:
        print("视频：文件存在，但未安装 opencv-python，播放器无法解码")
    else:
        cap = cv2.VideoCapture(str(video_path))
        opened = bool(cap.isOpened())
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if opened else 0
        fps = float(cap.get(cv2.CAP_PROP_FPS)) if opened else 0.0
        cap.release()
        print(f"视频：{'可解码' if opened else '无法解码'}，帧数 {frame_count}，FPS {fps:.3f}")
    return 0


class _VideoReader:
    def __init__(self, video_path: Path, timing: pd.DataFrame):
        self.path = video_path
        self.timing = timing.copy()
        self.cap = None
        self.last_frame = -1
        self.available = False
        self.error = ""
        if not video_path.exists():
            self.error = "video.mp4 缺失"
            return
        if cv2 is None:
            self.error = "缺少 opencv-python"
            return
        self.cap = cv2.VideoCapture(str(video_path))
        self.available = bool(self.cap.isOpened())
        if not self.available:
            self.error = "video.mp4 无法解码"
        if self.available and "callback_elapsed_us" in self.timing.columns:
            self.timing["_t"] = pd.to_numeric(
                self.timing["callback_elapsed_us"], errors="coerce"
            ) / 1_000_000.0
            self.timing["_frame"] = pd.to_numeric(
                self.timing["frame_index"], errors="coerce"
            ).fillna(-1).astype(int)
            self.timing = self.timing[np.isfinite(self.timing["_t"])]
        else:
            self.timing = pd.DataFrame()

    def frame_at(self, elapsed_s: float):
        if not self.available:
            return None
        if len(self.timing):
            times = self.timing["_t"].to_numpy()
            idx = int(np.searchsorted(times, elapsed_s, side="right") - 1)
            idx = max(0, min(idx, len(self.timing) - 1))
            frame_index = int(self.timing.iloc[idx]["_frame"])
        else:
            fps = float(self.cap.get(cv2.CAP_PROP_FPS)) or 30.0
            frame_index = max(0, int(round(elapsed_s * fps)))
        if frame_index == self.last_frame:
            return None
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ok, frame = self.cap.read()
        if not ok:
            return None
        self.last_frame = frame_index
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def close(self):
        if self.cap is not None:
            self.cap.release()


class SessionPlayer:
    def __init__(self, session: dict, speed: float = 1.0, window_seconds: float = 20.0):
        import matplotlib as mpl
        import matplotlib.pyplot as plt
        from matplotlib.gridspec import GridSpec
        from matplotlib.widgets import Button, Slider

        # 优先使用 Windows 常见中文字体，避免界面标题和提示显示为方框。
        mpl.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial Unicode MS", "DejaVu Sans"]
        mpl.rcParams["axes.unicode_minus"] = False
        self.session = session
        self.force = session["force"]
        self.motion = session["motion"]
        self.force_t = _time_seconds(self.force)
        self.motion_t = _time_seconds(self.motion)
        self.frame_t = (
            _number_column(session["frames"], "callback_elapsed_us") / 1_000_000.0
            if len(session["frames"])
            else np.array([], dtype=float)
        )
        self.t_min, self.t_max = _finite_time((self.force_t, self.motion_t, self.frame_t))
        self.current = self.t_min
        self.speed = max(0.05, float(speed))
        self.playing = True
        self._last_clock = time.perf_counter()
        self._changing_slider = False
        self.plot_axes = []
        self.cursor_lines = []
        self.axis_data_lines = {}
        self.follow_view = True
        self._drag_state = None
        total_duration = max(0.001, self.t_max - self.t_min)
        self.default_view_seconds = min(total_duration, max(0.5, float(window_seconds)))
        self.view_left = self.t_min
        self.view_right = min(self.t_max, self.t_min + self.default_view_seconds)
        self.video = _VideoReader(session["video_path"], session["frames"])

        self.fig = plt.figure(figsize=(16, 9), constrained_layout=True)
        self.fig.canvas.manager.set_window_title(
            f"实验同步播放器 - {session['directory'].name}"
        )
        grid = GridSpec(2, 3, figure=self.fig, width_ratios=(1.35, 1.0, 1.8))
        self.video_ax = self.fig.add_subplot(grid[:, 0])
        self.video_ax.set_title("Action 4 视频")
        self.video_ax.axis("off")
        self.video_image = self.video_ax.imshow(np.zeros((2, 2, 3), dtype=np.uint8))
        self.video_text = self.video_ax.text(
            0.5, 0.5, "", transform=self.video_ax.transAxes,
            ha="center", va="center", color="white", fontsize=12
        )

        self._make_force_panel(self.fig.add_subplot(grid[0, 1]), 1)
        self._make_force_panel(self.fig.add_subplot(grid[1, 1]), 2)
        self._make_motion_panel(grid[0, 2], (1, 2))
        self._make_motion_panel(grid[1, 2], (6, 7))

        slider_ax = self.fig.add_axes((0.12, 0.025, 0.63, 0.025))
        self.slider = Slider(slider_ax, "时间 (s)", self.t_min, self.t_max, valinit=self.current)
        self.slider.on_changed(self._on_slider)
        button_ax = self.fig.add_axes((0.78, 0.018, 0.10, 0.04))
        self.pause_button = Button(button_ax, "暂停")
        self.pause_button.on_clicked(self._toggle_play)
        self.status_text = self.fig.text(0.90, 0.035, "", ha="center", va="center", fontsize=9)

        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.fig.canvas.mpl_connect("button_press_event", self._on_button_press)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_mouse_move)
        self.fig.canvas.mpl_connect("button_release_event", self._on_button_release)
        self.timer = self.fig.canvas.new_timer(interval=33)
        self.timer.add_callback(self._on_timer)
        self._set_view(self.view_left, self.view_right)
        self._update(self.current)
        self.timer.start()

    def _register_cursor(self, axis):
        self.plot_axes.append(axis)
        self.cursor_lines.append(axis.axvline(self.current, color="black", lw=0.8, alpha=0.7))

    def _register_data_line(self, axis, line):
        self.axis_data_lines.setdefault(axis, []).append(line)

    def _make_force_panel(self, axis, channel: int):
        force = self.force
        clean_force, clean_torque, raw_force, raw_torque, fn_label, ft_label = _force_columns(channel)
        has_raw = raw_force in force.columns and raw_torque in force.columns
        if channel == 2 and not has_raw:
            axis.text(0.5, 0.5, "当前 force.csv 未记录 FN2 / FT2", ha="center", va="center")
            axis.set_title("FN2 / FT2")
            axis.set_xlabel("会话时间 (s)")
            self._register_cursor(axis)
            return

        fn, fn_mode = _clean_or_raw(force, clean_force, raw_force)
        ft, ft_mode = _clean_or_raw(force, clean_torque, raw_torque)
        t = self.force_t
        axis.plot(t, fn, color="#1769aa", lw=0.8, label=f"{fn_label} ({fn_mode})")
        axis.set_title(f"{fn_label} / {ft_label}")
        axis.set_xlabel("会话时间 (s)")
        axis.set_ylabel("FN (N)" if fn_mode == "清洗值" else "FN (V)")
        axis.grid(True, alpha=0.25)
        twin = axis.twinx()
        fn_line = axis.lines[-1]
        ft_line, = twin.plot(t, ft, color="#d97706", lw=0.8, label=f"{ft_label} ({ft_mode})")
        twin.set_ylabel("FT (N·m)" if ft_mode == "清洗值" else "FT (V)")
        lines = axis.lines + twin.lines
        axis.legend(lines, [line.get_label() for line in lines], loc="upper right", fontsize=8)
        self._register_data_line(axis, fn_line)
        self._register_data_line(twin, ft_line)
        self._register_cursor(axis)
        self._register_cursor(twin)

    def _make_motion_panel(self, grid_cell, axes: tuple[int, int]):
        subgrid = grid_cell.subgridspec(2, 1, hspace=0.15)
        pos_axis = self.fig.add_subplot(subgrid[0])
        speed_axis = self.fig.add_subplot(subgrid[1], sharex=pos_axis)
        a, b = axes
        t = self.motion_t
        positions = {}
        speeds = {}
        for axis_number in axes:
            value = _number_column(self.motion, f"axis{axis_number}_from_left_mm")
            value[~_valid_column(self.motion, "position_valid")] = np.nan
            positions[axis_number] = value
            speeds[axis_number] = _derivative(t, value)

        left_pos, = pos_axis.plot(t, positions[a], color="#1769aa", lw=0.8, label=f"axis{a} 位置")
        right_pos_axis = pos_axis.twinx()
        right_pos, = right_pos_axis.plot(t, positions[b], color="#d97706", lw=0.8, label=f"axis{b} 位置")
        pos_axis.set_ylabel(f"axis{a} ({'deg' if a in (2, 7) else 'mm'})")
        right_pos_axis.set_ylabel(f"axis{b} ({'deg' if b in (2, 7) else 'mm'})")
        pos_axis.grid(True, alpha=0.25)
        pos_axis.set_title(f"axis{a} / axis{b} 位置")
        pos_axis.legend((left_pos, right_pos), (left_pos.get_label(), right_pos.get_label()), fontsize=8, loc="upper right")

        left_speed, = speed_axis.plot(t, speeds[a], color="#1769aa", lw=0.8, label=f"axis{a} 速度")
        right_speed_axis = speed_axis.twinx()
        right_speed, = right_speed_axis.plot(t, speeds[b], color="#d97706", lw=0.8, label=f"axis{b} 速度")
        speed_axis.set_ylabel(f"axis{a} 速度")
        right_speed_axis.set_ylabel(f"axis{b} 速度")
        speed_axis.set_xlabel("会话时间 (s)")
        speed_axis.grid(True, alpha=0.25)
        speed_axis.legend((left_speed, right_speed), (left_speed.get_label(), right_speed.get_label()), fontsize=8, loc="upper right")
        self._register_data_line(pos_axis, left_pos)
        self._register_data_line(right_pos_axis, right_pos)
        self._register_data_line(speed_axis, left_speed)
        self._register_data_line(right_speed_axis, right_speed)
        self._register_cursor(pos_axis)
        self._register_cursor(right_pos_axis)
        self._register_cursor(speed_axis)
        self._register_cursor(right_speed_axis)

    def _autoscale_axis(self, axis):
        values = []
        for line in self.axis_data_lines.get(axis, ()):
            x = np.asarray(line.get_xdata(), dtype=float)
            y = np.asarray(line.get_ydata(), dtype=float)
            mask = np.isfinite(x) & np.isfinite(y)
            mask &= (x >= self.view_left) & (x <= self.view_right)
            if np.any(mask):
                values.append(y[mask])
        if not values:
            return
        visible = np.concatenate(values)
        low = float(np.min(visible))
        high = float(np.max(visible))
        padding = max((high - low) * 0.08, max(abs(low), abs(high), 1.0) * 1e-4)
        axis.set_ylim(low - padding, high + padding)

    def _set_view(self, left, right):
        duration = max(0.001, self.t_max - self.t_min)
        span = min(duration, max(0.5, float(right) - float(left)))
        left = float(left)
        if left < self.t_min:
            left = self.t_min
        if left + span > self.t_max:
            left = self.t_max - span
        self.view_left = left
        self.view_right = left + span
        for axis in dict.fromkeys(self.plot_axes):
            axis.set_xlim(self.view_left, self.view_right)
        for axis in self.axis_data_lines:
            self._autoscale_axis(axis)
        self.fig.canvas.draw_idle()

    def _center_view(self, center, span=None):
        if span is None:
            span = self.view_right - self.view_left
        self._set_view(center - span * 0.5, center + span * 0.5)

    def _on_scroll(self, event):
        if event.inaxes not in self.plot_axes or event.xdata is None:
            return
        old_span = self.view_right - self.view_left
        factor = 0.8 if event.button == "up" else 1.25
        new_span = min(max(0.5, old_span * factor), max(0.5, self.t_max - self.t_min))
        ratio = (float(event.xdata) - self.view_left) / old_span if old_span > 0 else 0.5
        new_left = float(event.xdata) - ratio * new_span
        self.follow_view = False
        self._set_view(new_left, new_left + new_span)

    def _on_button_press(self, event):
        if event.button != 1 or event.inaxes not in self.plot_axes:
            return
        width = max(1.0, float(event.inaxes.bbox.width))
        self._drag_state = (float(event.x), self.view_left, self.view_right, width)
        self.follow_view = False
        if self.playing:
            self._toggle_play()

    def _on_mouse_move(self, event):
        if self._drag_state is None or event.x is None:
            return
        start_x, left, right, width = self._drag_state
        shift = (float(event.x) - start_x) * (right - left) / width
        self._set_view(left - shift, right - shift)

    def _on_button_release(self, _event):
        self._drag_state = None

    def _toggle_play(self, _event=None):
        self.playing = not self.playing
        self._last_clock = time.perf_counter()
        self.pause_button.label.set_text("暂停" if self.playing else "播放")
        self.fig.canvas.draw_idle()

    def _on_slider(self, value):
        if not self._changing_slider:
            self.current = float(value)
            self.follow_view = True
            self._center_view(self.current, self.default_view_seconds)
            self._update(self.current)

    def _seek(self, delta):
        self.current = min(self.t_max, max(self.t_min, self.current + delta))
        self._update(self.current)

    def _on_key(self, event):
        if event.key == " ":
            self._toggle_play()
        elif event.key in ("left", "shift+left"):
            self._seek(-5.0 if event.key.startswith("shift") else -1.0)
        elif event.key in ("right", "shift+right"):
            self._seek(5.0 if event.key.startswith("shift") else 1.0)
        elif event.key == "home":
            self.current = self.t_min
            self._update(self.current)
        elif event.key == "end":
            self.current = self.t_max
            self._update(self.current)
        elif event.key in ("f", "F"):
            self.follow_view = True
            self._center_view(self.current, self.default_view_seconds)
        elif event.key == "0":
            self.follow_view = False
            self._set_view(self.t_min, self.t_max)

    def _on_timer(self):
        now = time.perf_counter()
        if self.playing:
            self.current = min(self.t_max, self.current + (now - self._last_clock) * self.speed)
            if self.current >= self.t_max:
                self.playing = False
                self.pause_button.label.set_text("播放")
        self._last_clock = now
        self._update(self.current)

    def _update(self, elapsed_s):
        if self.follow_view:
            span = self.view_right - self.view_left
            if elapsed_s < self.view_left or elapsed_s > self.view_right - span * 0.08:
                self._center_view(elapsed_s, self.default_view_seconds)
        for line in self.cursor_lines:
            line.set_xdata((elapsed_s, elapsed_s))
        frame = self.video.frame_at(elapsed_s)
        if frame is not None:
            self.video_image.set_data(frame)
            self.video_text.set_text("")
        elif not self.video.available:
            self.video_text.set_text(self.video.error)
        self.status_text.set_text(f"{elapsed_s:.3f} s")
        if not self._changing_slider:
            self._changing_slider = True
            self.slider.set_val(elapsed_s)
            self._changing_slider = False
        self.fig.canvas.draw_idle()

    def show(self):
        import matplotlib.pyplot as plt
        try:
            plt.show()
        finally:
            self.video.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Action 4、力感和位置同步播放器")
    parser.add_argument("session_dir", type=Path, help="包含 force.csv/motion.csv 的会话目录")
    parser.add_argument("--speed", type=float, default=1.0, help="播放速度，默认 1.0")
    parser.add_argument(
        "--window-seconds", type=float, default=20.0,
        help="曲线默认显示的时间窗口，默认 20 秒",
    )
    parser.add_argument("--check", action="store_true", help="只检查数据和视频，不打开窗口")
    args = parser.parse_args()
    try:
        session = _load_session(args.session_dir)
    except (OSError, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2
    if args.check:
        return _print_check(session)
    if cv2 is None and session["video_path"].exists():
        print("提示：未安装 opencv-python，曲线仍可显示，但视频无法解码。")
    player = SessionPlayer(session, args.speed, args.window_seconds)
    player.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
