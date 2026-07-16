#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""达妙夹爪 + FlexiTac 30×12 触觉闭环控制界面。"""

from __future__ import annotations

import math
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import serial
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
    QProgressBar,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROL_PACKAGE_ROOT = PROJECT_ROOT / "Servo_control" / "reBotArm_control_py"
if str(CONTROL_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_PACKAGE_ROOT))

from actuator.gripper import Gripper, load_cfg  # noqa: E402


GRIPPER_CONFIG = PROJECT_ROOT / "Servo_control" / "config" / "gripper.yaml"

# tactile_30_12_update.py 的当前硬件协议
TACTILE_PORT = "/dev/ttyUSB1"
TACTILE_BAUD = 2_000_000
TACTILE_ROWS = 16
TACTILE_COLS = 32
TACTILE_FRAME_BYTES = TACTILE_ROWS * TACTILE_COLS
TACTILE_MAGIC = b"\xAA\x55"
VIS_ROWS = 12
VIS_COLS = 30
BASELINE_FRAMES = 30
TACTILE_THRESHOLD = 20.0
CONTACT_CLIP = 100.0
CONTACT_CELL_THRESHOLD = 1.0
TACTILE_STALE_TIMEOUT = 0.35
FORCE_TOP_CELLS = 30
FORCE_FILTER_ALPHA = 0.35
BASELINE_DRIFT_ALPHA = 0.002
BASELINE_QUIET_PERCENTILE = 8.0
TACTILE_UI_RATE = 30.0
# 连续触觉反馈区间：从“开始接触”平滑过渡到“明显按压”。
CONTACT_BLEND_START_CELLS = 10
CONTACT_BLEND_FULL_CELLS = 30
CONTACT_BLEND_START_PEAK = 15.0
CONTACT_BLEND_FULL_PEAK = 40.0
# PID模式的最大力矩变化速度，避免闭合/张开命令瞬间反向。
PID_TORQUE_SLEW_RATE = 0.8  # N·m/s

MOTOR_RATE = 100.0
# 软限位只清零继续向外的力矩，不失能；硬限位才视为反馈/零点异常。
POSITION_GUARD = 0.03
HARD_POSITION_FAULT_MARGIN = 0.75
# 新夹爪机构静摩擦较大。默认值用于可靠起步，LIMIT 仍作为统一硬保护。
OPEN_TORQUE_LIMIT = -0.3
CLOSE_TORQUE_LIMIT = 0.3
DEFAULT_OPEN_TORQUE = -0.23
DEFAULT_CLOSE_TORQUE = 0.24


def smoothstep(value: float, start: float, end: float) -> float:
    """返回0～1且端点一阶连续的平滑权重。"""
    if end <= start:
        raise ValueError("smoothstep end 必须大于 start")
    x = float(np.clip((value - start) / (end - start), 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


def tactile_contact_blend(contact_cells: int, peak: float) -> float:
    """点数和峰值必须共同增大，才逐渐提高主动释放权重。"""
    cells_weight = smoothstep(
        contact_cells, CONTACT_BLEND_START_CELLS, CONTACT_BLEND_FULL_CELLS
    )
    peak_weight = smoothstep(
        peak, CONTACT_BLEND_START_PEAK, CONTACT_BLEND_FULL_PEAK
    )
    return min(cells_weight, peak_weight)


def slew_limit(current: float, target: float, max_delta: float) -> float:
    """限制单周期力矩变化量。"""
    return current + float(np.clip(target - current, -max_delta, max_delta))


def extract_next_frame(buffer: bytearray):
    """初始化时按顺序提取一帧，行为与 tactile_30_12_update.py 一致。"""
    index = buffer.find(TACTILE_MAGIC)
    if index < 0:
        return None, bytearray(buffer[-1:]) if len(buffer) > 1 else buffer
    end = index + len(TACTILE_MAGIC) + TACTILE_FRAME_BYTES
    if len(buffer) < end:
        return None, bytearray(buffer[index:])
    return bytes(buffer[index + 2:end]), bytearray(buffer[end:])


def extract_latest_frame(buffer: bytearray):
    """实时阶段只取最新完整帧，主动丢弃积压帧以降低延迟。"""
    positions = []
    start = 0
    while True:
        index = buffer.find(TACTILE_MAGIC, start)
        if index < 0:
            break
        positions.append(index)
        start = index + 2

    if not positions:
        return None, bytearray(buffer[-1:]) if len(buffer) > 1 else buffer
    for index in reversed(positions):
        end = index + 2 + TACTILE_FRAME_BYTES
        if len(buffer) >= end:
            return bytes(buffer[index + 2:end]), bytearray(buffer[end:])
    return None, bytearray(buffer[positions[-1]:])


class TactileWorker(QThread):
    """FlexiTac 二进制采集、基线扣除和特征计算线程。"""

    status_updated = pyqtSignal(str)
    baseline_progress = pyqtSignal(int, int)
    tactile_updated = pyqtSignal(float, float, int, object, float)
    ready_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)

    def __init__(self, port: str = TACTILE_PORT, baud: int = TACTILE_BAUD):
        super().__init__()
        self.port = port
        self.baud = baud
        self._running = True
        self._rebaseline = False
        self._lock = threading.Lock()

    def request_rebaseline(self) -> None:
        with self._lock:
            self._rebaseline = True

    def request_stop(self) -> None:
        with self._lock:
            self._running = False

    def _flags(self):
        with self._lock:
            running, rebaseline = self._running, self._rebaseline
            self._rebaseline = False
        return running, rebaseline

    @staticmethod
    def _decode(frame_bytes: bytes) -> np.ndarray:
        return np.frombuffer(frame_bytes, dtype=np.uint8).reshape(
            TACTILE_ROWS, TACTILE_COLS
        ).astype(np.float32)

    def run(self) -> None:
        device = None
        try:
            device = serial.Serial(self.port, self.baud, timeout=0.005)
            device.reset_input_buffer()
            try:
                device.set_buffer_size(rx_size=262144, tx_size=262144)
            except (AttributeError, OSError):
                pass

            buffer = bytearray()
            baseline_samples = []
            baseline = None
            filtered_force = None
            last_emit_time = 0.0
            self.status_updated.emit("请保持阵列无接触，正在采集基线")
            self.ready_changed.emit(False)

            while True:
                running, rebaseline = self._flags()
                if not running:
                    break
                if rebaseline:
                    baseline_samples.clear()
                    baseline = None
                    filtered_force = None
                    buffer.clear()
                    device.reset_input_buffer()
                    self.ready_changed.emit(False)
                    self.status_updated.emit("请保持阵列无接触，正在重新采集基线")

                waiting = device.in_waiting
                chunk = device.read(waiting if waiting > 0 else 4096)
                if not chunk:
                    continue
                buffer.extend(chunk)
                if len(buffer) > 50000:
                    buffer = buffer[-50000:]

                if baseline is None:
                    while len(baseline_samples) < BASELINE_FRAMES:
                        frame_bytes, new_buffer = extract_next_frame(buffer)
                        if frame_bytes is None:
                            break
                        buffer = new_buffer
                        baseline_samples.append(self._decode(frame_bytes))
                        self.baseline_progress.emit(len(baseline_samples), BASELINE_FRAMES)
                    if len(baseline_samples) == BASELINE_FRAMES:
                        baseline = np.median(np.stack(baseline_samples), axis=0).astype(np.float32)
                        self.ready_changed.emit(True)
                        self.status_updated.emit("触觉在线，实时解算中")
                    continue

                frame_bytes, buffer = extract_latest_frame(buffer)
                if frame_bytes is None:
                    continue
                raw = self._decode(frame_bytes)
                delta = raw - baseline
                # 无接触时缓慢跟踪温漂；有接触时冻结基线，避免把真实压力吃掉。
                if float(np.percentile(np.abs(delta), 95)) < BASELINE_QUIET_PERCENTILE:
                    baseline *= 1.0 - BASELINE_DRIFT_ALPHA
                    baseline += BASELINE_DRIFT_ALPHA * raw

                contact = delta - TACTILE_THRESHOLD
                np.clip(contact, 0.0, CONTACT_CLIP, out=contact)
                crop = contact[-VIS_ROWS:, 1:-1]

                # 取最强的若干接触点求均值，比全阵列求和更不受接触面积和边缘噪声影响。
                flat = crop.ravel()
                top_cells = np.partition(flat, flat.size - FORCE_TOP_CELLS)[-FORCE_TOP_CELLS:]
                robust_force = float(np.mean(top_cells))
                if filtered_force is None:
                    filtered_force = robust_force
                else:
                    filtered_force += FORCE_FILTER_ALPHA * (robust_force - filtered_force)
                force_peak = float(np.max(crop))
                contact_cells = int(np.count_nonzero(crop > CONTACT_CELL_THRESHOLD))
                timestamp = time.monotonic()
                # 限制 Qt 事件频率，防止高帧率串口数据堆积导致闭环延迟。
                if timestamp - last_emit_time >= 1.0 / TACTILE_UI_RATE:
                    last_emit_time = timestamp
                    self.tactile_updated.emit(
                        filtered_force, force_peak, contact_cells, crop.copy(), timestamp
                    )

        except (serial.SerialException, OSError, ValueError) as exc:
            self.ready_changed.emit(False)
            self.error_occurred.emit(f"触觉串口 {self.port}: {exc}")
        finally:
            if device is not None and device.is_open:
                device.close()


class ForcePID:
    """带积分限幅和微分低通的触觉力 PID。"""

    def __init__(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.derivative = 0.0
        self.last_time = time.monotonic()

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.derivative = 0.0
        self.last_time = time.monotonic()

    def compute(self, target, measurement, kp, ki, kd) -> float:
        now = time.monotonic()
        dt = min(0.1, max(1e-4, now - self.last_time))
        error = target - measurement
        self.integral = float(np.clip(self.integral + error * dt, -200.0, 200.0))
        raw_derivative = (error - self.previous_error) / dt
        self.derivative = 0.2 * raw_derivative + 0.8 * self.derivative
        output = kp * error + ki * self.integral + kd * self.derivative
        self.previous_error = error
        self.last_time = now
        # 触觉闭环闭合方向严格限制为 DEFAULT_CLOSE_TORQUE。
        return float(np.clip(output, OPEN_TORQUE_LIMIT, DEFAULT_CLOSE_TORQUE))


class MotorWorker(QThread):
    """独占 Gripper/motorbridge，执行手动力矩或触觉 PID。"""

    initialized = pyqtSignal(str)
    state_updated = pyqtSignal(float, float, float, float, str, bool)
    pid_updated = pyqtSignal(float, float)
    error_occurred = pyqtSignal(str)
    stopped = pyqtSignal()

    def __init__(self, config_path: Path):
        super().__init__()
        self.config_path = config_path
        cfg = load_cfg(str(config_path))
        gc = cfg["gripper"]
        self._description = (
            f"{cfg['channel']} | {gc.vendor} {gc.model} | "
            f"ID 0x{gc.motor_id:02X}/0x{gc.feedback_id:02X}"
        )
        self._lower, self._upper = sorted((gc.position_open, gc.position_close))
        self._lock = threading.Lock()
        self._running = True
        self._enable_requested = False
        self._mode = "idle"
        self._manual_torque = 0.0
        self._force = 0.0
        self._tactile_peak = 0.0
        self._contact_cells = 0
        self._force_timestamp = 0.0
        self._target_force = 25.0
        self._kp = 0.012
        self._ki = 0.0005
        self._kd = 0.0

    def request_enable(self) -> None:
        with self._lock:
            self._enable_requested = True

    def request_disable(self) -> None:
        with self._lock:
            self._enable_requested = False
            self._mode = "idle"
            self._manual_torque = 0.0

    def set_manual_torque(self, torque: float) -> None:
        with self._lock:
            self._manual_torque = float(np.clip(torque, OPEN_TORQUE_LIMIT, CLOSE_TORQUE_LIMIT))
            self._mode = "manual"

    def stop_control(self) -> None:
        with self._lock:
            self._mode = "idle"
            self._manual_torque = 0.0

    def update_tactile(
        self, force: float, peak: float, contact_cells: int, timestamp: float
    ) -> None:
        with self._lock:
            self._force = float(force)
            self._tactile_peak = float(peak)
            self._contact_cells = int(contact_cells)
            self._force_timestamp = float(timestamp)

    def start_force_control(self, target: float, kp: float, ki: float, kd: float) -> None:
        with self._lock:
            self._target_force, self._kp, self._ki, self._kd = target, kp, ki, kd
            self._mode = "pid"

    def update_pid(self, target: float, kp: float, ki: float, kd: float) -> None:
        with self._lock:
            self._target_force, self._kp, self._ki, self._kd = target, kp, ki, kd

    def request_stop(self) -> None:
        with self._lock:
            self._running = False
            self._enable_requested = False
            self._mode = "idle"

    def _snapshot(self):
        with self._lock:
            return (
                self._running, self._enable_requested, self._mode, self._manual_torque,
                self._force, self._tactile_peak, self._contact_cells,
                self._force_timestamp, self._target_force,
                self._kp, self._ki, self._kd,
            )

    def run(self) -> None:
        gripper = None
        enabled = False
        pid = ForcePID()
        previous_mode = "idle"
        watchdog_reported = False
        position_violation_reported = False
        smooth_pid_command = 0.0
        cycle = 0
        try:
            gripper = Gripper(str(self.config_path))
            gripper.disable()
            if not gripper.mode_mit(kp=0.0, kd=0.0):
                raise RuntimeError("夹爪切换到 MIT 力矩模式失败")
            self.initialized.emit(self._description)

            period = 1.0 / MOTOR_RATE
            next_tick = time.monotonic()
            while True:
                (running, want_enabled, mode, manual_torque, force, tactile_peak,
                 contact_cells, force_timestamp, target_force, kp, ki, kd) = self._snapshot()
                if not running:
                    break

                if want_enabled and not enabled:
                    if gripper.enable():
                        enabled = True
                    else:
                        with self._lock:
                            self._enable_requested = False
                        self.error_occurred.emit("电机使能失败")
                elif not want_enabled and enabled:
                    gripper.disable()
                    enabled = False

                if mode != previous_mode:
                    pid.reset()
                    smooth_pid_command = 0.0
                    previous_mode = mode

                command = 0.0
                if enabled and mode == "manual":
                    command = manual_torque
                elif enabled and mode == "pid":
                    age = time.monotonic() - force_timestamp
                    if force_timestamp <= 0 or age > TACTILE_STALE_TIMEOUT:
                        with self._lock:
                            self._mode = "idle"
                        if not watchdog_reported:
                            watchdog_reported = True
                            self.error_occurred.emit("触觉数据超时，闭环已停止并清零力矩")
                    else:
                        watchdog_reported = False
                        pid_command = pid.compute(target_force, force, kp, ki, kd)
                        contact_blend = tactile_contact_blend(
                            contact_cells, tactile_peak
                        )
                        # 接触增强时从PID输出连续混合到张开力矩，不再二值跳变。
                        target_command = (
                            (1.0 - contact_blend) * pid_command
                            + contact_blend * DEFAULT_OPEN_TORQUE
                        )
                        smooth_pid_command = slew_limit(
                            smooth_pid_command,
                            target_command,
                            PID_TORQUE_SLEW_RATE / MOTOR_RATE,
                        )
                        command = smooth_pid_command

                # MIT 纯力矩控制，同时主动获取位置反馈用于机械限位保护。
                if enabled:
                    position, _, _ = gripper.get_state(request=True)
                    if position <= self._lower + POSITION_GUARD and command < 0:
                        command = 0.0
                        smooth_pid_command = 0.0
                    if position >= self._upper - POSITION_GUARD and command > 0:
                        command = 0.0
                        smooth_pid_command = 0.0
                    if mode == "pid":
                        # 显示经过软限位处理后真正下发的闭环力矩。
                        self.pid_updated.emit(force, command)
                    gripper.mit(0.0, 0.0, kp=0.0, kd=0.0, tau=command)
                    position, velocity, torque = gripper.get_state(request=False)
                else:
                    position, velocity, torque = gripper.get_state(request=True)

                if not all(math.isfinite(v) for v in (position, velocity, torque)):
                    raise RuntimeError("电机反馈包含 NaN/Inf")
                position_outside = (
                    position < self._lower - HARD_POSITION_FAULT_MARGIN
                    or position > self._upper + HARD_POSITION_FAULT_MARGIN
                )
                if position_outside and not position_violation_reported:
                    if enabled:
                        gripper.disable()
                        enabled = False
                    with self._lock:
                        self._enable_requested = False
                        self._mode = "idle"
                    self.error_occurred.emit(
                        f"电机位置 {position:.3f} rad 严重超出机械范围 "
                        f"[{self._lower}, {self._upper}]，已失能"
                    )
                    position_violation_reported = True
                elif not position_outside:
                    position_violation_reported = False

                cycle += 1
                if cycle % 10 == 0:
                    self.state_updated.emit(position, velocity, torque, command, mode, enabled)

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


class GripperUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.motor_worker: Optional[MotorWorker] = None
        self.tactile_worker: Optional[TactileWorker] = None
        self.motor_ready = False
        self.motor_enabled = False
        self.tactile_ready = False
        self._closing = False
        self._build_ui()
        QTimer.singleShot(0, self.start_workers)

    def _build_ui(self) -> None:
        self.setWindowTitle("reBotArm 夹爪触觉闭环")
        self.resize(760, 600)
        self.setStyleSheet("""
            QMainWindow { background:#f3f5f7; }
            QGroupBox { font-size:14px; font-weight:bold; background:white;
                border:1px solid #cfd8dc; border-radius:7px; margin-top:9px; padding-top:11px; }
            QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; }
            QPushButton { min-height:38px; font-size:14px; border-radius:5px; }
            QPushButton:disabled { color:#888; background:#ddd; }
            QDoubleSpinBox { min-height:30px; }
        """)
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        title = QLabel("达妙夹爪 · FlexiTac 触觉控制")
        title.setFont(QFont("Sans Serif", 20, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        status_group = QGroupBox("系统状态")
        status_grid = QGridLayout(status_group)
        self.motor_status = QLabel("电机：初始化中")
        self.tactile_status = QLabel("触觉：初始化中")
        self.baseline_progress = QProgressBar()
        self.baseline_progress.setRange(0, BASELINE_FRAMES)
        status_grid.addWidget(self.motor_status, 0, 0)
        status_grid.addWidget(self.tactile_status, 0, 1)
        status_grid.addWidget(QLabel("触觉基线："), 1, 0)
        status_grid.addWidget(self.baseline_progress, 1, 1)
        layout.addWidget(status_group)

        feedback_group = QGroupBox("实时反馈")
        feedback_grid = QGridLayout(feedback_group)
        self.position_label = QLabel("-- rad")
        self.motor_torque_label = QLabel("-- N·m")
        self.force_label = QLabel("0.0")
        self.peak_label = QLabel("0.0")
        self.cells_label = QLabel("0 / 360")
        for column, (name, widget) in enumerate((
            ("电机位置", self.position_label), ("电机力矩", self.motor_torque_label),
            ("稳健力特征", self.force_label), ("触觉峰值", self.peak_label),
            ("接触点数", self.cells_label),
        )):
            feedback_grid.addWidget(QLabel(name), 0, column)
            feedback_grid.addWidget(widget, 1, column)
        layout.addWidget(feedback_group)

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

        self.tabs = QTabWidget()
        self.tabs.addTab(self._manual_tab(), "手动力矩")
        self.tabs.addTab(self._pid_tab(), "触觉 PID 闭环")
        layout.addWidget(self.tabs)
        self._update_buttons()

    def _manual_tab(self) -> QWidget:
        widget = QWidget()
        layout = QGridLayout(widget)
        self.open_torque_spin = QDoubleSpinBox()
        self.open_torque_spin.setRange(OPEN_TORQUE_LIMIT, 0.0)
        self.open_torque_spin.setDecimals(3)
        self.open_torque_spin.setSingleStep(0.01)
        self.open_torque_spin.setValue(DEFAULT_OPEN_TORQUE)
        self.open_torque_spin.setSuffix(" N·m")
        self.close_torque_spin = QDoubleSpinBox()
        self.close_torque_spin.setRange(0.0, CLOSE_TORQUE_LIMIT)
        self.close_torque_spin.setDecimals(3)
        self.close_torque_spin.setSingleStep(0.01)
        self.close_torque_spin.setValue(DEFAULT_CLOSE_TORQUE)
        self.close_torque_spin.setSuffix(" N·m")
        self.open_button = QPushButton("张开")
        self.close_button = QPushButton("闭合")
        self.stop_button = QPushButton("力矩归零/停止")
        self.open_button.clicked.connect(lambda: self.set_manual(self.open_torque_spin.value()))
        self.close_button.clicked.connect(lambda: self.set_manual(self.close_torque_spin.value()))
        self.stop_button.clicked.connect(self.stop_control)
        layout.addWidget(QLabel("张开力矩"), 0, 0)
        layout.addWidget(self.open_torque_spin, 1, 0)
        layout.addWidget(self.open_button, 2, 0)
        layout.addWidget(QLabel("闭合力矩"), 0, 1)
        layout.addWidget(self.close_torque_spin, 1, 1)
        layout.addWidget(self.close_button, 2, 1)
        layout.addWidget(self.stop_button, 3, 0, 1, 2)
        return widget

    def _pid_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        grid = QGridLayout()
        self.target_force_spin = QDoubleSpinBox()
        self.target_force_spin.setRange(0.0, CONTACT_CLIP)
        self.target_force_spin.setValue(25.0)
        self.target_force_spin.setSingleStep(1.0)
        self.kp_spin = self._gain_spin(0.012, 5)
        self.ki_spin = self._gain_spin(0.0005, 6)
        self.kd_spin = self._gain_spin(0.0, 7)
        for row, (name, spin) in enumerate((
            ("目标稳健力特征", self.target_force_spin), ("Kp", self.kp_spin),
            ("Ki", self.ki_spin), ("Kd", self.kd_spin),
        )):
            grid.addWidget(QLabel(name), row, 0)
            grid.addWidget(spin, row, 1)
        layout.addLayout(grid)
        self.pid_output_label = QLabel("闭环输出：0.000 N·m")
        self.pid_output_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.pid_output_label)
        row = QHBoxLayout()
        self.start_pid_button = QPushButton("启动触觉闭环")
        self.stop_pid_button = QPushButton("停止闭环")
        self.rebaseline_button = QPushButton("重新采集基线")
        self.start_pid_button.clicked.connect(self.start_pid)
        self.stop_pid_button.clicked.connect(self.stop_control)
        self.rebaseline_button.clicked.connect(self.rebaseline)
        row.addWidget(self.start_pid_button)
        row.addWidget(self.stop_pid_button)
        row.addWidget(self.rebaseline_button)
        layout.addLayout(row)
        for spin in (self.target_force_spin, self.kp_spin, self.ki_spin, self.kd_spin):
            spin.valueChanged.connect(self.update_pid_parameters)
        return widget

    @staticmethod
    def _gain_spin(value: float, decimals: int) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 1.0)
        spin.setDecimals(decimals)
        spin.setSingleStep(10 ** (-decimals + 1))
        spin.setValue(value)
        return spin

    def start_workers(self) -> None:
        try:
            self.motor_worker = MotorWorker(GRIPPER_CONFIG)
            self.motor_worker.initialized.connect(self.on_motor_initialized)
            self.motor_worker.state_updated.connect(self.on_motor_state)
            self.motor_worker.pid_updated.connect(self.on_pid_update)
            self.motor_worker.error_occurred.connect(self.on_motor_error)
            self.motor_worker.stopped.connect(self.on_motor_stopped)
            self.motor_worker.start()
        except Exception as exc:
            self.on_motor_error(str(exc))

        self.tactile_worker = TactileWorker()
        self.tactile_worker.status_updated.connect(
            lambda message: self.tactile_status.setText(f"触觉：{message}")
        )
        self.tactile_worker.baseline_progress.connect(self.baseline_progress.setValue)
        self.tactile_worker.tactile_updated.connect(self.on_tactile_update)
        self.tactile_worker.ready_changed.connect(self.on_tactile_ready)
        self.tactile_worker.error_occurred.connect(self.on_tactile_error)
        self.tactile_worker.start()

    def on_motor_initialized(self, description: str) -> None:
        self.motor_ready = True
        self.motor_status.setText(f"电机：已连接 {description}，MIT 就绪/已失能")
        self._update_buttons()

    def on_tactile_ready(self, ready: bool) -> None:
        self.tactile_ready = ready
        self._update_buttons()

    def on_tactile_update(
        self, force: float, peak: float, cells: int, _matrix, timestamp: float
    ) -> None:
        self.force_label.setText(f"{force:.1f}")
        self.peak_label.setText(f"{peak:.1f}")
        self.cells_label.setText(f"{cells} / {VIS_ROWS * VIS_COLS}")
        if self.motor_worker is not None:
            self.motor_worker.update_tactile(force, peak, cells, timestamp)

    def enable_motor(self) -> None:
        if self.motor_worker is None:
            return
        answer = QMessageBox.question(
            self, "确认使能", "请确认夹爪范围内没有人员或障碍物。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.motor_worker.request_enable()
            self.motor_status.setText("电机：正在使能")

    def disable_motor(self) -> None:
        if self.motor_worker is not None:
            self.motor_worker.request_disable()
        self.motor_enabled = False
        self._update_buttons()

    def set_manual(self, torque: float) -> None:
        if self.motor_worker is not None and self.motor_enabled:
            self.motor_worker.set_manual_torque(torque)

    def start_pid(self) -> None:
        if not self.motor_enabled:
            QMessageBox.warning(self, "未使能", "请先使能电机。")
            return
        if not self.tactile_ready:
            QMessageBox.warning(self, "触觉未就绪", "请等待基线采集完成。")
            return
        self.motor_worker.start_force_control(
            self.target_force_spin.value(), self.kp_spin.value(),
            self.ki_spin.value(), self.kd_spin.value(),
        )
        self._update_buttons(pid_active=True)

    def update_pid_parameters(self) -> None:
        if self.motor_worker is not None:
            self.motor_worker.update_pid(
                self.target_force_spin.value(), self.kp_spin.value(),
                self.ki_spin.value(), self.kd_spin.value(),
            )

    def stop_control(self) -> None:
        if self.motor_worker is not None:
            self.motor_worker.stop_control()
        self.pid_output_label.setText("闭环输出：0.000 N·m")
        self._update_buttons(pid_active=False)

    def rebaseline(self) -> None:
        self.stop_control()
        self.tactile_ready = False
        self.baseline_progress.setValue(0)
        if self.tactile_worker is not None:
            self.tactile_worker.request_rebaseline()
        self._update_buttons()

    def on_motor_state(
        self, position: float, _velocity: float, torque: float,
        _command: float, mode: str, enabled: bool,
    ) -> None:
        self.position_label.setText(f"{position:.4f} rad")
        self.motor_torque_label.setText(f"{torque:.4f} N·m")
        self.motor_enabled = enabled
        self.motor_status.setText(f"电机：{'已使能' if enabled else '已失能'} | {mode}")
        self._update_buttons(pid_active=(mode == "pid"))

    def on_pid_update(self, force: float, output: float) -> None:
        self.force_label.setText(f"{force:.1f}")
        self.pid_output_label.setText(f"闭环输出：{output:.4f} N·m")

    def on_motor_error(self, message: str) -> None:
        self.motor_enabled = False
        self.motor_status.setText("电机：错误/已失能")
        self._update_buttons()
        if not self._closing:
            QMessageBox.critical(self, "电机错误", message)

    def on_tactile_error(self, message: str) -> None:
        self.tactile_ready = False
        self.tactile_status.setText("触觉：连接错误")
        self.stop_control()
        if not self._closing:
            QMessageBox.critical(self, "触觉错误", message)

    def on_motor_stopped(self) -> None:
        self.motor_ready = False
        self.motor_enabled = False
        self._update_buttons()

    def _update_buttons(self, pid_active: bool = False) -> None:
        self.enable_button.setEnabled(self.motor_ready and not self.motor_enabled)
        self.disable_button.setEnabled(self.motor_ready and self.motor_enabled)
        for button in (self.open_button, self.close_button, self.stop_button):
            button.setEnabled(self.motor_ready and self.motor_enabled and not pid_active)
        self.start_pid_button.setEnabled(
            self.motor_ready and self.motor_enabled and self.tactile_ready and not pid_active
        )
        self.stop_pid_button.setEnabled(self.motor_enabled and pid_active)
        self.rebaseline_button.setEnabled(self.tactile_worker is not None and not pid_active)

    def cleanup(self) -> None:
        if self._closing:
            return
        self._closing = True
        if self.motor_worker is not None and self.motor_worker.isRunning():
            self.motor_worker.request_stop()
        if self.tactile_worker is not None and self.tactile_worker.isRunning():
            self.tactile_worker.request_stop()
        if self.motor_worker is not None:
            self.motor_worker.wait(3500)
        if self.tactile_worker is not None:
            self.tactile_worker.wait(1500)

    def closeEvent(self, event) -> None:
        self.cleanup()
        event.accept()


app_instance: Optional[QApplication] = None
main_window: Optional[GripperUI] = None


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
    timer = QTimer()
    timer.start(250)
    timer.timeout.connect(lambda: None)
    main_window = GripperUI()
    main_window.show()
    return app_instance.exec_()


if __name__ == "__main__":
    sys.exit(main())
