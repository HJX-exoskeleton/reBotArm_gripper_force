#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪触觉闭环界面：12×30 热力图与电机/触觉实时曲线版。"""

from __future__ import annotations

import os
import signal
import sys
import time
from collections import deque
from typing import Optional

# 当前运行环境的 ~/.config 可能只读，使用可写缓存避免 Matplotlib 启动警告和延迟。
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-rebotarm")

import matplotlib

matplotlib.use("Qt5Agg")

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QGroupBox, QVBoxLayout

from gripper_ui_integrated_tactile import (
    CONTACT_CLIP,
    DEFAULT_CLOSE_TORQUE,
    GripperUI as BaseGripperUI,
    OPEN_TORQUE_LIMIT,
    VIS_COLS,
    VIS_ROWS,
)


PLOT_RATE = 20.0
PLOT_HISTORY_SECONDS = 20.0
MAX_MOTOR_POINTS = int(PLOT_RATE * PLOT_HISTORY_SECONDS)
MAX_TACTILE_POINTS = int(30.0 * PLOT_HISTORY_SECONDS)
POSITION_MIN = -5.8
POSITION_MAX = 0.0


def set_dynamic_ylim(axis, values, minimum_padding: float) -> None:
    if not values:
        return
    low, high = min(values), max(values)
    padding = max(minimum_padding, (high - low) * 0.12)
    axis.set_ylim(low - padding, high + padding)


class GripperVisualizationUI(BaseGripperUI):
    """复用主控制界面的硬件与闭环逻辑，仅增加无阻塞可视化。"""

    def __init__(self):
        self.plot_start = time.monotonic()
        self.latest_tactile_matrix = np.zeros((VIS_ROWS, VIS_COLS), dtype=np.float32)

        self.motor_times = deque(maxlen=MAX_MOTOR_POINTS)
        self.positions = deque(maxlen=MAX_MOTOR_POINTS)
        self.velocities = deque(maxlen=MAX_MOTOR_POINTS)
        self.actual_torques = deque(maxlen=MAX_MOTOR_POINTS)
        self.command_torques = deque(maxlen=MAX_MOTOR_POINTS)

        self.tactile_times = deque(maxlen=MAX_TACTILE_POINTS)
        self.force_features = deque(maxlen=MAX_TACTILE_POINTS)
        self.force_targets = deque(maxlen=MAX_TACTILE_POINTS)
        self.contact_peaks = deque(maxlen=MAX_TACTILE_POINTS)

        super().__init__()
        self.setWindowTitle("reBotArm 夹爪触觉闭环 · 可视化")
        self.resize(1450, 920)

    def _build_ui(self) -> None:
        super()._build_ui()
        plot_group = QGroupBox("FlexiTac 12×30 与闭环状态")
        plot_layout = QVBoxLayout(plot_group)

        self.figure = Figure(figsize=(13, 6), constrained_layout=True)
        self.canvas = FigureCanvas(self.figure)
        grid = self.figure.add_gridspec(4, 2, width_ratios=(1.15, 1.55))

        self.heat_axis = self.figure.add_subplot(grid[:, 0])
        self.position_axis = self.figure.add_subplot(grid[0, 1])
        self.velocity_axis = self.figure.add_subplot(grid[1, 1], sharex=self.position_axis)
        self.torque_axis = self.figure.add_subplot(grid[2, 1], sharex=self.position_axis)
        self.force_axis = self.figure.add_subplot(grid[3, 1], sharex=self.position_axis)

        self.heat_image = self.heat_axis.imshow(
            self.latest_tactile_matrix,
            cmap="turbo",
            interpolation="bilinear",
            origin="upper",
            vmin=0.0,
            vmax=CONTACT_CLIP,
            aspect="auto",
        )
        self.heat_axis.set_title(f"FlexiTac contact map ({VIS_ROWS}×{VIS_COLS})")
        self.heat_axis.set_xlabel("Column")
        self.heat_axis.set_ylabel("Row")
        colorbar = self.figure.colorbar(self.heat_image, ax=self.heat_axis, fraction=0.046)
        colorbar.set_label("Contact value")

        self.position_line, = self.position_axis.plot([], [], "#d32f2f", label="Position")
        self.velocity_line, = self.velocity_axis.plot([], [], "#388e3c", label="Velocity")
        self.actual_torque_line, = self.torque_axis.plot(
            [], [], "#1976d2", label="Actual torque"
        )
        self.command_torque_line, = self.torque_axis.plot(
            [], [], "k--", linewidth=1.0, label="Command torque"
        )
        self.force_line, = self.force_axis.plot([], [], "#7b1fa2", label="Robust force")
        self.target_line, = self.force_axis.plot([], [], "k--", label="Target")
        self.peak_line, = self.force_axis.plot(
            [], [], color="#f57c00", alpha=0.65, label="Peak"
        )

        self.position_axis.set_ylabel("Position\n(rad)")
        self.velocity_axis.set_ylabel("Velocity\n(rad/s)")
        self.torque_axis.set_ylabel("Torque\n(N·m)")
        self.force_axis.set_ylabel("Tactile")
        self.force_axis.set_xlabel("Time (s)")
        self.position_axis.set_ylim(POSITION_MIN - 0.2, POSITION_MAX + 0.2)
        self.torque_axis.set_ylim(OPEN_TORQUE_LIMIT - 0.05, DEFAULT_CLOSE_TORQUE + 0.05)
        self.force_axis.set_ylim(0.0, CONTACT_CLIP + 5.0)

        for axis in (
            self.position_axis, self.velocity_axis, self.torque_axis, self.force_axis
        ):
            axis.grid(True, linestyle="--", alpha=0.35)
            axis.legend(loc="upper right", fontsize=8)
        for axis in (self.position_axis, self.velocity_axis, self.torque_axis):
            axis.tick_params(labelbottom=False)

        plot_layout.addWidget(self.canvas)
        self.centralWidget().layout().addWidget(plot_group, stretch=1)

        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.update_visualization)
        self.plot_timer.start(round(1000.0 / PLOT_RATE))

    def on_tactile_update(
        self, force: float, peak: float, cells: int, matrix, timestamp: float
    ) -> None:
        super().on_tactile_update(force, peak, cells, matrix, timestamp)
        array = np.asarray(matrix, dtype=np.float32)
        if array.shape == (VIS_ROWS, VIS_COLS):
            self.latest_tactile_matrix = array.copy()
        elapsed = time.monotonic() - self.plot_start
        self.tactile_times.append(elapsed)
        self.force_features.append(force)
        self.force_targets.append(self.target_force_spin.value())
        self.contact_peaks.append(peak)

    def on_motor_state(
        self,
        position: float,
        velocity: float,
        torque: float,
        command: float,
        mode: str,
        enabled: bool,
    ) -> None:
        super().on_motor_state(position, velocity, torque, command, mode, enabled)
        elapsed = time.monotonic() - self.plot_start
        self.motor_times.append(elapsed)
        self.positions.append(position)
        self.velocities.append(velocity)
        self.actual_torques.append(torque)
        self.command_torques.append(command)

    def update_visualization(self) -> None:
        """只在 Qt 主线程以固定20 Hz更新绘图，不参与控制与串口读取。"""
        self.heat_image.set_data(self.latest_tactile_matrix)

        if len(self.motor_times) > 1:
            self.position_line.set_data(self.motor_times, self.positions)
            self.velocity_line.set_data(self.motor_times, self.velocities)
            self.actual_torque_line.set_data(self.motor_times, self.actual_torques)
            self.command_torque_line.set_data(self.motor_times, self.command_torques)
            set_dynamic_ylim(self.velocity_axis, self.velocities, 0.1)

        if len(self.tactile_times) > 1:
            self.force_line.set_data(self.tactile_times, self.force_features)
            self.target_line.set_data(self.tactile_times, self.force_targets)
            self.peak_line.set_data(self.tactile_times, self.contact_peaks)

        latest_times = []
        if self.motor_times:
            latest_times.append(self.motor_times[-1])
        if self.tactile_times:
            latest_times.append(self.tactile_times[-1])
        if latest_times:
            right = max(latest_times)
            left = max(0.0, right - PLOT_HISTORY_SECONDS)
            for axis in (
                self.position_axis, self.velocity_axis, self.torque_axis, self.force_axis
            ):
                axis.set_xlim(left, max(left + 1.0, right))

        self.canvas.draw_idle()

    def clear_plot_history(self) -> None:
        for data in (
            self.motor_times, self.positions, self.velocities, self.actual_torques,
            self.command_torques, self.tactile_times, self.force_features,
            self.force_targets, self.contact_peaks,
        ):
            data.clear()
        self.plot_start = time.monotonic()

    def rebaseline(self) -> None:
        super().rebaseline()
        self.latest_tactile_matrix.fill(0.0)
        self.clear_plot_history()

    def cleanup(self) -> None:
        if hasattr(self, "plot_timer"):
            self.plot_timer.stop()
        super().cleanup()


app_instance: Optional[QApplication] = None
main_window: Optional[GripperVisualizationUI] = None


def signal_handler(signum, _frame) -> None:
    print(f"\n收到信号 {signum}，正在安全退出……")
    if main_window is not None:
        QTimer.singleShot(0, main_window.close)


def main() -> int:
    global app_instance, main_window
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    app_instance = QApplication(sys.argv)
    app_instance.setStyle("Fusion")
    signal_timer = QTimer()
    signal_timer.start(250)
    signal_timer.timeout.connect(lambda: None)
    main_window = GripperVisualizationUI()
    main_window.show()
    return app_instance.exec_()


if __name__ == "__main__":
    sys.exit(main())
