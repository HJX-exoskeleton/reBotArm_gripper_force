#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基于 reBotArm_control_py.actuator.Gripper 的夹爪控制界面。"""

from __future__ import annotations

import math
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

# 允许从仓库内直接运行本文件，无需安装 reBotArm_control_py 包。
PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROL_PACKAGE_ROOT = PROJECT_ROOT / "Servo_control" / "reBotArm_control_py"
if str(CONTROL_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_PACKAGE_ROOT))

from actuator.gripper import Gripper, load_cfg  # noqa: E402


DEFAULT_CONFIG = PROJECT_ROOT / "Servo_control" / "config" / "gripper.yaml"
CONTROL_RATE = 50.0
UI_FEEDBACK_RATE = 10.0
POSITION_GUARD = 0.15


class MotorWorker(QThread):
    """独占 Gripper/motorbridge 的硬件线程。"""

    initialized = pyqtSignal(str, float, float, float)
    state_updated = pyqtSignal(float, float, float, object, bool)
    error_occurred = pyqtSignal(str)
    stopped = pyqtSignal()

    def __init__(self, config_path: Path):
        super().__init__()
        self.config_path = config_path
        self._lock = threading.Lock()
        self._running = True
        self._enable_requested = False
        self._target: Optional[float] = None
        self._velocity_limit = 3.0
        self._last_position = 0.0
        loaded = load_cfg(str(config_path))
        cfg = loaded["gripper"]
        self._channel = loaded["channel"]
        self._cfg = cfg
        self._lower, self._upper = sorted((cfg.position_open, cfg.position_close))
        self._margin = cfg.safety_margin

    def request_enable(self) -> None:
        with self._lock:
            self._enable_requested = True

    def request_disable(self) -> None:
        with self._lock:
            self._enable_requested = False
            self._target = None

    def set_target(self, target: float, velocity_limit: float) -> None:
        with self._lock:
            self._target = min(self._upper, max(self._lower, float(target)))
            self._velocity_limit = max(0.1, float(velocity_limit))

    def hold_position(self) -> None:
        with self._lock:
            self._target = min(self._upper, max(self._lower, self._last_position))

    def request_stop(self) -> None:
        with self._lock:
            self._running = False
            self._enable_requested = False
            self._target = None

    def _snapshot(self):
        with self._lock:
            return self._running, self._enable_requested, self._target, self._velocity_limit

    def run(self) -> None:
        gripper = None
        enabled = False
        violation_reported = False
        feedback_divider = max(1, round(CONTROL_RATE / UI_FEEDBACK_RATE))
        cycle = 0

        try:
            # Controller 的创建、模式切换和后续调用均在本线程内进行。
            gripper = Gripper(str(self.config_path))
            gripper.disable()
            if not gripper.mode_pos_vel():
                raise RuntimeError("夹爪切换到 POS_VEL 模式失败")

            self._velocity_limit = gripper.default_velocity_limit
            self.initialized.emit(
                f"{self._channel} | {self._cfg.vendor} {self._cfg.model} | "
                f"ID 0x{self._cfg.motor_id:02X}/0x{self._cfg.feedback_id:02X}",
                self._lower,
                self._upper,
                self._margin,
            )

            period = 1.0 / CONTROL_RATE
            next_tick = time.monotonic()
            while True:
                running, want_enabled, target, velocity_limit = self._snapshot()
                if not running:
                    break

                if want_enabled and not enabled:
                    if gripper.enable():
                        enabled = True
                    else:
                        with self._lock:
                            self._enable_requested = False
                        self.error_occurred.emit("电机使能失败，请检查电机状态和通信")
                elif not want_enabled and enabled:
                    gripper.disable()
                    enabled = False

                if enabled and target is not None:
                    gripper.pos_vel(target, velocity_limit)
                    position, velocity, torque = gripper.get_state(request=False)
                else:
                    position, velocity, torque = gripper.get_state(request=True)

                if not all(math.isfinite(value) for value in (position, velocity, torque)):
                    raise RuntimeError("电机反馈包含 NaN/Inf")

                with self._lock:
                    self._last_position = position

                outside = (
                    position < self._lower - POSITION_GUARD
                    or position > self._upper + POSITION_GUARD
                )
                if outside and not violation_reported:
                    if enabled:
                        gripper.disable()
                        enabled = False
                    with self._lock:
                        self._enable_requested = False
                        self._target = None
                    violation_reported = True
                    self.error_occurred.emit(
                        f"位置 {position:.3f} rad 超出机械范围 "
                        f"[{self._lower:.3f}, {self._upper:.3f}]，已失能"
                    )
                elif not outside:
                    violation_reported = False

                cycle += 1
                if cycle % feedback_divider == 0:
                    self.state_updated.emit(position, velocity, torque, target, enabled)

                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()

        except Exception as exc:
            self.error_occurred.emit(str(exc))
        finally:
            if gripper is not None:
                try:
                    gripper.disconnect()
                except Exception:
                    pass
            self.stopped.emit()


class MotorControlUI(QMainWindow):
    def __init__(self, config_path: Path = DEFAULT_CONFIG):
        super().__init__()
        self.config_path = config_path.resolve()
        self.worker: Optional[MotorWorker] = None
        self.hardware_ready = False
        self.motor_enabled = False
        self._closing = False

        try:
            gripper_cfg = load_cfg(str(self.config_path))["gripper"]
            self.position_open = gripper_cfg.position_open
            self.position_close = gripper_cfg.position_close
            self.lower, self.upper = sorted((self.position_open, self.position_close))
            self.safety_margin = gripper_cfg.safety_margin
            self.default_vlim = gripper_cfg.vlim
        except Exception as exc:
            self.position_open, self.position_close = -5.8, 0.0
            self.lower, self.upper = -5.8, 0.0
            self.safety_margin = 0.1
            self.default_vlim = 3.0
            self._config_error = str(exc)
        else:
            self._config_error = None

        self._build_ui()
        if self._config_error is None:
            self._start_worker()
        else:
            self._show_initialization_error(self._config_error)

    def _build_ui(self) -> None:
        self.setWindowTitle("reBotArm 达妙夹爪控制")
        self.resize(700, 560)
        self.setStyleSheet("""
            QMainWindow { background: #f3f5f7; }
            QGroupBox { font-size:14px; font-weight:bold; background:white;
                border:1px solid #cfd8dc; border-radius:8px;
                margin-top:10px; padding-top:12px; }
            QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 5px; }
            QPushButton { font-size:15px; min-height:44px; border-radius:6px; }
            QPushButton:disabled { color:#888; background:#ddd; }
            QDoubleSpinBox { min-height:32px; font-size:14px; }
        """)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(24, 20, 24, 24)
        layout.setSpacing(14)

        title = QLabel("reBotArm 夹爪安全控制")
        title.setFont(QFont("Sans Serif", 22, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        self.connection_label = QLabel("正在通过 motorbridge 连接……")
        self.connection_label.setAlignment(Qt.AlignCenter)
        self.connection_label.setStyleSheet("color:#ef6c00;font-weight:bold;")
        layout.addWidget(self.connection_label)

        state_group = QGroupBox("实时反馈")
        state_grid = QGridLayout(state_group)
        self.position_label = QLabel("-- rad")
        self.velocity_label = QLabel("-- rad/s")
        self.torque_label = QLabel("-- N·m")
        self.target_label = QLabel("无")
        for column, (name, widget) in enumerate((
            ("位置", self.position_label), ("速度", self.velocity_label),
            ("力矩", self.torque_label), ("目标", self.target_label),
        )):
            state_grid.addWidget(QLabel(f"{name}："), 0, column)
            state_grid.addWidget(widget, 1, column)
        layout.addWidget(state_group)

        settings_group = QGroupBox("POS_VEL 参数")
        settings_grid = QGridLayout(settings_group)
        self.target_spin = QDoubleSpinBox()
        self.target_spin.setRange(self.lower, self.upper)
        self.target_spin.setDecimals(3)
        self.target_spin.setSingleStep(0.1)
        self.target_spin.setValue((self.lower + self.upper) / 2.0)
        self.target_spin.setSuffix(" rad")
        self.velocity_spin = QDoubleSpinBox()
        self.velocity_spin.setRange(0.1, 5.0)
        self.velocity_spin.setDecimals(2)
        self.velocity_spin.setSingleStep(0.1)
        self.velocity_spin.setValue(self.default_vlim)
        self.velocity_spin.setSuffix(" rad/s")
        settings_grid.addWidget(QLabel("目标位置："), 0, 0)
        settings_grid.addWidget(self.target_spin, 0, 1)
        settings_grid.addWidget(QLabel("速度上限："), 1, 0)
        settings_grid.addWidget(self.velocity_spin, 1, 1)
        settings_grid.addWidget(QLabel("YAML 机械范围："), 2, 0)
        settings_grid.addWidget(
            QLabel(f"{self.lower:.2f} ～ {self.upper:.2f} rad，安全余量 {self.safety_margin:.2f} rad"),
            2, 1,
        )
        layout.addWidget(settings_group)

        enable_row = QHBoxLayout()
        self.enable_button = QPushButton("使能电机")
        self.disable_button = QPushButton("立即失能")
        self.enable_button.setStyleSheet("background:#fb8c00;color:white;")
        self.disable_button.setStyleSheet("background:#c62828;color:white;")
        self.enable_button.clicked.connect(self.enable_motor)
        self.disable_button.clicked.connect(self.disable_motor)
        enable_row.addWidget(self.enable_button)
        enable_row.addWidget(self.disable_button)
        layout.addLayout(enable_row)

        motion_row = QHBoxLayout()
        self.open_button = QPushButton(f"完全张开\n{self.position_open:.1f} rad")
        self.move_button = QPushButton("移动到目标")
        self.hold_button = QPushButton("保持当前位置")
        self.close_button = QPushButton(f"完全闭合\n{self.position_close:.1f} rad")
        self.open_button.setStyleSheet("background:#43a047;color:white;")
        self.move_button.setStyleSheet("background:#00897b;color:white;")
        self.hold_button.setStyleSheet("background:#5e35b1;color:white;")
        self.close_button.setStyleSheet("background:#1e88e5;color:white;")
        self.open_button.clicked.connect(lambda: self.move_to(self.position_open))
        self.move_button.clicked.connect(lambda: self.move_to(self.target_spin.value()))
        self.hold_button.clicked.connect(self.hold_position)
        self.close_button.clicked.connect(lambda: self.move_to(self.position_close))
        for button in (self.open_button, self.move_button, self.hold_button, self.close_button):
            motion_row.addWidget(button)
        layout.addLayout(motion_row)

        self.status_label = QLabel("状态：初始化中")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        self._update_buttons()

    def _start_worker(self) -> None:
        try:
            self.worker = MotorWorker(self.config_path)
        except Exception as exc:
            self._show_initialization_error(str(exc))
            return
        self.worker.initialized.connect(self.on_initialized)
        self.worker.state_updated.connect(self.on_state_updated)
        self.worker.error_occurred.connect(self.on_motor_error)
        self.worker.stopped.connect(self.on_worker_stopped)
        self.worker.start()

    def on_initialized(self, description: str, lower: float, upper: float, margin: float) -> None:
        self.hardware_ready = True
        self.connection_label.setText(f"已连接：{description}")
        self.connection_label.setStyleSheet("color:#2e7d32;font-weight:bold;")
        self.status_label.setText("状态：POS_VEL 就绪，电机已失能")
        self._update_buttons()

    def enable_motor(self) -> None:
        if self.worker is None or not self.hardware_ready:
            return
        answer = QMessageBox.question(
            self, "确认使能", "请确认夹爪运动范围内没有人员或障碍物。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.worker.request_enable()
            self.status_label.setText("状态：正在使能……")

    def disable_motor(self) -> None:
        if self.worker is not None:
            self.worker.request_disable()
        self.motor_enabled = False
        self.status_label.setText("状态：正在失能……")
        self._update_buttons()

    def move_to(self, target: float) -> None:
        if self.worker is None or not self.motor_enabled:
            QMessageBox.warning(self, "未使能", "请先使能电机。")
            return
        target = min(self.upper, max(self.lower, float(target)))
        self.target_spin.setValue(target)
        self.worker.set_target(target, self.velocity_spin.value())
        self.status_label.setText(f"状态：移动到 {target:.3f} rad")

    def hold_position(self) -> None:
        if self.worker is None or not self.motor_enabled:
            QMessageBox.warning(self, "未使能", "请先使能电机。")
            return
        self.worker.hold_position()
        self.status_label.setText("状态：保持当前位置")

    def on_state_updated(
        self, position: float, velocity: float, torque: float, target, enabled: bool
    ) -> None:
        self.position_label.setText(f"{position:.4f} rad")
        self.velocity_label.setText(f"{velocity:.4f} rad/s")
        self.torque_label.setText(f"{torque:.4f} N·m")
        self.target_label.setText("无" if target is None else f"{target:.3f} rad")
        if enabled != self.motor_enabled:
            self.motor_enabled = enabled
            self.status_label.setText("状态：已使能" if enabled else "状态：已失能")
            self._update_buttons()

    def on_motor_error(self, message: str) -> None:
        self.motor_enabled = False
        self.status_label.setText("状态：错误/已失能")
        self.connection_label.setStyleSheet("color:#c62828;font-weight:bold;")
        self._update_buttons()
        if not self._closing:
            QMessageBox.critical(self, "夹爪错误", message)

    def on_worker_stopped(self) -> None:
        self.hardware_ready = False
        self.motor_enabled = False
        self._update_buttons()

    def _show_initialization_error(self, message: str) -> None:
        self.connection_label.setText("初始化失败")
        self.connection_label.setStyleSheet("color:#c62828;font-weight:bold;")
        self.status_label.setText("状态：不可用")
        self._update_buttons()
        QTimer.singleShot(0, lambda: QMessageBox.critical(self, "初始化错误", message))

    def _update_buttons(self) -> None:
        self.enable_button.setEnabled(self.hardware_ready and not self.motor_enabled)
        self.disable_button.setEnabled(self.hardware_ready and self.motor_enabled)
        for button in (self.open_button, self.move_button, self.hold_button, self.close_button):
            button.setEnabled(self.hardware_ready and self.motor_enabled)

    def cleanup(self) -> None:
        if self._closing:
            return
        self._closing = True
        if self.worker is not None and self.worker.isRunning():
            self.worker.request_stop()
            self.worker.wait(3500)

    def closeEvent(self, event) -> None:
        reply = QMessageBox.question(
            self, "确认退出", "退出将失能夹爪并关闭 motorbridge，确定继续吗？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.cleanup()
            event.accept()
        else:
            event.ignore()


app_instance: Optional[QApplication] = None
main_window: Optional[MotorControlUI] = None


def signal_handler(signum, _frame) -> None:
    print(f"\n收到信号 {signum}，请求关闭界面……")
    if main_window is not None:
        QTimer.singleShot(0, main_window.close)


def main() -> int:
    global app_instance, main_window
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    app_instance = QApplication(sys.argv)
    app_instance.setStyle("Fusion")
    timer = QTimer()
    timer.start(250)
    timer.timeout.connect(lambda: None)
    main_window = MotorControlUI()
    main_window.show()
    return app_instance.exec_()


if __name__ == "__main__":
    sys.exit(main())
