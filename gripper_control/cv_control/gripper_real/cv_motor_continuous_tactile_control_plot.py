#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉位置意图 + FlexiTac 力反馈的夹爪连续遥操作。

 运行：

  python3 gripper_control/cv_control/gripper_real/cv_motor_continuous_tactile_control_plot.py

  指定摄像头：

  python3 gripper_control/cv_control/gripper_real/cv_motor_continuous_tactile_control_plot.py --camera 0

  追求更低视觉延迟：

  python3 gripper_control/cv_control/gripper_real/cv_motor_continuous_tactile_control_plot.py --no-plot

"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import serial

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-rebotarm")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import HandTrackingModule as htm


PROJECT_ROOT = Path(__file__).resolve().parents[3]
GRIPPER_CONTROL_ROOT = PROJECT_ROOT / "gripper_control"
CONTROL_PACKAGE_ROOT = PROJECT_ROOT / "Servo_control" / "reBotArm_control_py"
for path in (GRIPPER_CONTROL_ROOT, CONTROL_PACKAGE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from actuator.gripper import Gripper  # noqa: E402
from cv_control.gripper_real.cv_motor_continuous_control_plot import (  # noqa: E402
    filter_pinch_ratio,
    normalized_pinch_ratio,
    ratio_to_position,
)
from gripper_ui_integrated_tactile import (  # noqa: E402
    BASELINE_DRIFT_ALPHA,
    BASELINE_FRAMES,
    BASELINE_QUIET_PERCENTILE,
    CONTACT_CELL_THRESHOLD,
    CONTACT_CLIP,
    FORCE_FILTER_ALPHA,
    FORCE_TOP_CELLS,
    TACTILE_BAUD,
    TACTILE_COLS,
    TACTILE_FRAME_BYTES,
    TACTILE_MAGIC,
    TACTILE_PORT,
    TACTILE_ROWS,
    TACTILE_STALE_TIMEOUT,
    TACTILE_THRESHOLD,
    VIS_COLS,
    VIS_ROWS,
    extract_latest_frame,
    extract_next_frame,
    slew_limit,
    tactile_contact_blend,
)


GRIPPER_CONFIG = PROJECT_ROOT / "Servo_control" / "config" / "gripper.yaml"
P_OPEN = -5.8
P_CLOSE = 0.0
POSITION_FAULT_MARGIN = 0.75
CONTROL_RATE = 100.0
VELOCITY_LIMIT = 6.0
CONTACT_VELOCITY_LIMIT = 0.4  # 0.8
STARTUP_POSITION = 0.0
STARTUP_VELOCITY_LIMIT = 2.0
STARTUP_POSITION_TOLERANCE = 0.05
STARTUP_TIMEOUT = 8.0
CONTROLLER_READY_TIMEOUT = STARTUP_TIMEOUT + 4.0
VISUAL_TARGET_SLEW_RATE = 6.0  # 与POS_VEL速度上限一致
TACTILE_ACTIVE_CELLS = 3
TACTILE_ACTIVE_PEAK = 5.0
CONTACT_BLEND_ALPHA = 0.03  # 0.05
TACTILE_RELEASE_GAIN = 0.02  # 0.04  # rad / 触觉特征单位
TACTILE_BLEND_RELEASE = 0.06  # 0.40  # 完全接触权重对应的额外张开量(rad)
TACTILE_CLOSE_ALLOWANCE = 0.06  # 0.12  # 接触后最多继续闭合的行程(rad)
TACTILE_MAX_RELEASE = 0.08  # 1.50   # 最大触觉退让行程(rad)
TACTILE_OFFSET_RATE = 0.35  # 1.20   # 触觉退让变化速度(rad/s)
CONTACT_HOLD_TIME = 0.8  # 0.30     # 接触短暂丢失时仍保持锚点(s)
HAND_LOSS_TIMEOUT = 0.35
TARGET_FORCE = 18.0
PLOT_RATE = 6.0
PLOT_HISTORY = 15.0
MAX_POINTS = round(PLOT_RATE * PLOT_HISTORY)


def desired_tactile_offset(force: float, contact_blend: float, active: bool) -> float:
    """计算相对接触锚点的位置修正，正值只允许极小的继续闭合量。"""
    if not active:
        return 0.0
    force_correction = TACTILE_RELEASE_GAIN * (TARGET_FORCE - float(force))
    correction = (
        force_correction
        - TACTILE_BLEND_RELEASE * float(np.clip(contact_blend, 0.0, 1.0))
    )
    return float(np.clip(
        correction, -TACTILE_MAX_RELEASE, TACTILE_CLOSE_ALLOWANCE
    ))


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.visual_target: Optional[float] = None
        self.visual_time = 0.0
        self.tactile_force = 0.0
        self.tactile_peak = 0.0
        self.contact_cells = 0
        self.tactile_matrix = np.zeros((VIS_ROWS, VIS_COLS), dtype=np.float32)
        self.tactile_time = 0.0
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.command_position = 0.0
        self.command_torque = 0.0
        self.error: Optional[str] = None

    def set_visual_target(self, target: float) -> None:
        with self.lock:
            self.visual_target = float(np.clip(target, P_OPEN, P_CLOSE))
            self.visual_time = time.monotonic()

    def set_tactile(self, force, peak, cells, matrix, timestamp) -> None:
        with self.lock:
            self.tactile_force = float(force)
            self.tactile_peak = float(peak)
            self.contact_cells = int(cells)
            self.tactile_matrix = matrix.copy()
            self.tactile_time = float(timestamp)

    def snapshot(self):
        with self.lock:
            return {
                "visual_target": self.visual_target,
                "visual_time": self.visual_time,
                "force": self.tactile_force,
                "peak": self.tactile_peak,
                "cells": self.contact_cells,
                "matrix": self.tactile_matrix.copy(),
                "tactile_time": self.tactile_time,
                "position": self.position,
                "velocity": self.velocity,
                "torque": self.torque,
                "command_position": self.command_position,
                "command_torque": self.command_torque,
                "error": self.error,
            }


class TactileReader(threading.Thread):
    """按 tactile_30_12_update.py 协议读取最新触觉帧。"""

    def __init__(self, state: SharedState, stop_event: threading.Event, port: str):
        super().__init__(daemon=True)
        self.state = state
        self.stop_event = stop_event
        self.port = port
        self.ready = threading.Event()
        self.error: Optional[str] = None

    @staticmethod
    def decode(payload: bytes) -> np.ndarray:
        return np.frombuffer(payload, dtype=np.uint8).reshape(
            TACTILE_ROWS, TACTILE_COLS
        ).astype(np.float32)

    def run(self) -> None:
        device = None
        try:
            device = serial.Serial(self.port, TACTILE_BAUD, timeout=0.005)
            device.reset_input_buffer()
            buffer = bytearray()
            baseline_frames = []
            print("触觉：请保持阵列无接触，正在采集30帧基线……")

            while not self.stop_event.is_set() and len(baseline_frames) < BASELINE_FRAMES:
                chunk = device.read(device.in_waiting or 4096)
                if not chunk:
                    continue
                buffer.extend(chunk)
                while len(baseline_frames) < BASELINE_FRAMES:
                    payload, new_buffer = extract_next_frame(buffer)
                    if payload is None:
                        break
                    buffer = new_buffer
                    baseline_frames.append(self.decode(payload))

            if len(baseline_frames) != BASELINE_FRAMES:
                raise RuntimeError("触觉基线采集未完成")
            baseline = np.median(np.stack(baseline_frames), axis=0).astype(np.float32)
            filtered_force = None
            print("触觉：基线完成，等待第一帧实时反馈……")

            while not self.stop_event.is_set():
                chunk = device.read(device.in_waiting or 4096)
                if not chunk:
                    continue
                buffer.extend(chunk)
                if len(buffer) > 50000:
                    buffer = buffer[-50000:]
                payload, buffer = extract_latest_frame(buffer)
                if payload is None:
                    continue

                raw = self.decode(payload)
                delta = raw - baseline
                if float(np.percentile(np.abs(delta), 95)) < BASELINE_QUIET_PERCENTILE:
                    baseline *= 1.0 - BASELINE_DRIFT_ALPHA
                    baseline += BASELINE_DRIFT_ALPHA * raw
                contact = delta - TACTILE_THRESHOLD
                np.clip(contact, 0.0, CONTACT_CLIP, out=contact)
                crop = contact[-VIS_ROWS:, 1:-1]
                flat = crop.ravel()
                top = np.partition(flat, flat.size - FORCE_TOP_CELLS)[-FORCE_TOP_CELLS:]
                robust_force = float(np.mean(top))
                if filtered_force is None:
                    filtered_force = robust_force
                else:
                    filtered_force += FORCE_FILTER_ALPHA * (robust_force - filtered_force)
                self.state.set_tactile(
                    filtered_force,
                    float(np.max(crop)),
                    int(np.count_nonzero(crop > CONTACT_CELL_THRESHOLD)),
                    crop,
                    time.monotonic(),
                )
                if not self.ready.is_set():
                    # 只有第一帧实时数据及时间戳已写入后，才允许启动电机线程。
                    self.ready.set()
                    print("触觉：第一帧实时反馈就绪。")

        except Exception as exc:
            self.error = str(exc)
            with self.state.lock:
                self.state.error = f"触觉错误: {exc}"
            self.ready.set()
            self.stop_event.set()
        finally:
            if device is not None and device.is_open:
                device.close()


class FusionController(threading.Thread):
    """100 Hz视觉位置意图与触觉反馈平滑融合控制。"""

    def __init__(self, state: SharedState, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.state = state
        self.stop_event = stop_event
        self.ready = threading.Event()

    def _move_to_startup_position(self, gripper: Gripper) -> tuple[float, float, float]:
        """限速移动到启动零位，到位后才允许进入正常控制。"""
        target = float(np.clip(STARTUP_POSITION, P_OPEN, P_CLOSE))
        deadline = time.monotonic() + STARTUP_TIMEOUT
        position, velocity, torque = gripper.get_state(request=True)

        while not self.stop_event.is_set():
            if not all(math.isfinite(value) for value in (position, velocity, torque)):
                raise RuntimeError("零位初始化时电机反馈包含NaN/Inf")
            if position < P_OPEN - POSITION_FAULT_MARGIN or position > P_CLOSE + POSITION_FAULT_MARGIN:
                raise RuntimeError(f"零位初始化时位置 {position:.3f} rad 严重越界")

            with self.state.lock:
                self.state.position, self.state.velocity, self.state.torque = position, velocity, torque
                self.state.command_position = target

            if abs(position - target) <= STARTUP_POSITION_TOLERANCE:
                return position, velocity, torque
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"夹爪零位初始化超时：当前 {position:.3f} rad，目标 {target:.3f} rad"
                )

            gripper.pos_vel(target, STARTUP_VELOCITY_LIMIT)
            position, velocity, torque = gripper.get_state(request=False)
            time.sleep(1.0 / CONTROL_RATE)

        raise RuntimeError("夹爪零位初始化被中止")

    def run(self) -> None:
        gripper = None
        try:
            gripper = Gripper(str(GRIPPER_CONFIG))
            gripper.disable()
            if not gripper.mode_pos_vel():
                raise RuntimeError("切换POS_VEL模式失败")
            if not gripper.enable():
                raise RuntimeError("电机使能失败")
            print(f"电机：正在限速初始化到 {STARTUP_POSITION:.1f} rad 零位……")
            position, velocity, torque = self._move_to_startup_position(gripper)
            print("电机：零位初始化完成。")
            visual_target = float(np.clip(STARTUP_POSITION, P_OPEN, P_CLOSE))
            smooth_visual_target = visual_target
            smooth_contact_blend = 0.0
            tactile_offset = 0.0
            contact_anchor: Optional[float] = None
            last_contact_time = 0.0
            with self.state.lock:
                self.state.position, self.state.velocity, self.state.torque = position, velocity, torque
                self.state.command_position = visual_target
            self.ready.set()

            period = 1.0 / CONTROL_RATE
            next_tick = time.monotonic()
            while not self.stop_event.is_set():
                data = self.state.snapshot()
                now = time.monotonic()
                if data["visual_target"] is not None and now - data["visual_time"] <= HAND_LOSS_TIMEOUT:
                    visual_target = data["visual_target"]
                    smooth_visual_target = slew_limit(
                        smooth_visual_target,
                        visual_target,
                        VISUAL_TARGET_SLEW_RATE / CONTROL_RATE,
                    )
                else:
                    # 手势丢失时停止未完成动作并保持当前反馈位置。
                    visual_target = float(np.clip(position, P_OPEN, P_CLOSE))
                    smooth_visual_target = visual_target

                if data["tactile_time"] <= 0 or now - data["tactile_time"] > TACTILE_STALE_TIMEOUT:
                    raise RuntimeError("触觉反馈超时")

                raw_blend = tactile_contact_blend(data["cells"], data["peak"])
                smooth_contact_blend += CONTACT_BLEND_ALPHA * (
                    raw_blend - smooth_contact_blend
                )
                tactile_active = (
                    data["cells"] >= TACTILE_ACTIVE_CELLS
                    and data["peak"] >= TACTILE_ACTIVE_PEAK
                )
                if tactile_active:
                    if contact_anchor is None:
                        # 首次接触位置成为力控基准，防止继续追赶CV完全闭合目标。
                        contact_anchor = float(np.clip(position, P_OPEN, P_CLOSE))
                    last_contact_time = now
                contact_mode = (
                    contact_anchor is not None
                    and now - last_contact_time <= CONTACT_HOLD_TIME
                )
                desired_offset = desired_tactile_offset(
                    data["force"], smooth_contact_blend, contact_mode
                )
                tactile_offset = slew_limit(
                    tactile_offset,
                    desired_offset,
                    TACTILE_OFFSET_RATE / CONTROL_RATE,
                )
                if contact_mode:
                    contact_target = float(np.clip(
                        contact_anchor + tactile_offset, P_OPEN, P_CLOSE
                    ))
                    # CV主动张开优先；CV闭合不得越过触觉锚点目标。
                    command_position = min(smooth_visual_target, contact_target)
                    velocity_limit = CONTACT_VELOCITY_LIMIT
                else:
                    contact_anchor = None
                    command_position = smooth_visual_target
                    velocity_limit = VELOCITY_LIMIT
                gripper.pos_vel(command_position, velocity_limit)
                position, velocity, torque = gripper.get_state(request=False)
                if not all(math.isfinite(value) for value in (position, velocity, torque)):
                    raise RuntimeError("电机反馈包含NaN/Inf")
                if position < P_OPEN - POSITION_FAULT_MARGIN or position > P_CLOSE + POSITION_FAULT_MARGIN:
                    raise RuntimeError(f"位置 {position:.3f} rad 严重越界")
                with self.state.lock:
                    self.state.position, self.state.velocity, self.state.torque = position, velocity, torque
                    self.state.command_position = command_position
                    # 保留该通道用于界面显示触觉产生的位置退让量。
                    self.state.command_torque = tactile_offset

                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()

        except Exception as exc:
            with self.state.lock:
                self.state.error = f"电机控制错误: {exc}"
            self.ready.set()
            self.stop_event.set()
        finally:
            if gripper is not None:
                try:
                    gripper.disconnect()
                except Exception:
                    pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="视觉-触觉连续控制达妙夹爪")
    parser.add_argument("--camera", type=int, default=0, help="摄像头编号，默认0")
    parser.add_argument("--tactile-port", default=TACTILE_PORT, help="触觉串口")
    parser.add_argument("--no-plot", action="store_true", help="关闭曲线窗口以降低延迟")
    parser.add_argument("--yes", action="store_true", help="跳过启动确认")
    return parser


def make_dashboard():
    plt.style.use("fast")
    figure, axes = plt.subplots(3, 1, figsize=(7, 7), sharex=True, dpi=90)
    position_line, command_position_line = axes[0].plot([], [], "r-", [], [], "m--")
    force_line, = axes[1].plot([], [], color="darkorange")
    target_force_line, = axes[1].plot([], [], "g--")
    torque_line, = axes[2].plot([], [], "b-")
    axes[0].set_ylabel("Position (rad)")
    axes[1].set_ylabel("Robust tactile")
    axes[2].set_ylabel("Torque (N·m)")
    axes[2].set_xlabel("Time (s)")
    axes[0].set_ylim(P_OPEN - 0.2, P_CLOSE + 0.2)
    axes[1].set_ylim(0, CONTACT_CLIP + 5)
    axes[2].set_ylim(-0.4, 0.4)
    for axis in axes:
        axis.grid(True, linestyle=":", alpha=0.5)
    figure.tight_layout()
    return figure, axes, (
        position_line, command_position_line, force_line, target_force_line,
        torque_line,
    )


def render_figure(figure):
    figure.canvas.draw()
    return cv2.cvtColor(np.asarray(figure.canvas.buffer_rgba()), cv2.COLOR_RGBA2BGR)


def render_tactile(matrix: np.ndarray, force: float, peak: float, cells: int) -> np.ndarray:
    gray = np.clip(matrix * 2.55, 0, 255).astype(np.uint8)
    image = cv2.resize(gray, (600, 240), interpolation=cv2.INTER_CUBIC)
    image = cv2.applyColorMap(image, cv2.COLORMAP_TURBO)
    cv2.rectangle(image, (0, 0), (600, 38), (0, 0, 0), -1)
    cv2.putText(
        image, f"Force {force:.1f}  Peak {peak:.1f}  Cells {cells}",
        (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2,
    )
    return image


def main() -> int:
    args = build_parser().parse_args()
    if not args.yes:
        answer = input("请保持触觉无接触并确认夹爪周围安全。[y/N] ").strip().lower()
        if answer not in {"y", "yes"}:
            return 0

    stop_event = threading.Event()
    state = SharedState()

    def handle_signal(signum, _frame):
        print(f"\n收到信号 {signum}，正在安全退出……")
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    tactile = TactileReader(state, stop_event, args.tactile_port)
    tactile.start()
    if not tactile.ready.wait(5.0) or tactile.error:
        stop_event.set()
        tactile.join(timeout=1.0)
        print(f"触觉初始化失败: {tactile.error or '超时'}", file=sys.stderr)
        return 1

    controller = FusionController(state, stop_event)
    controller.start()
    if not controller.ready.wait(CONTROLLER_READY_TIMEOUT) or state.snapshot()["error"]:
        stop_event.set()
        controller.join(timeout=3.0)
        tactile.join(timeout=1.0)
        print(state.snapshot()["error"] or "电机初始化超时", file=sys.stderr)
        return 1

    camera = None
    figure = axes = lines = plot_image = None
    try:
        detector = htm.handDetector(
            maxHands=1, model_complexity=0, detectionCon=0.70, trackCon=0.75
        )
        camera = cv2.VideoCapture(args.camera)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_FPS, 60)
        if not camera.isOpened():
            raise RuntimeError(f"无法打开摄像头 {args.camera}")
        if not args.no_plot:
            figure, axes, lines = make_dashboard()

        buffers = [deque(maxlen=MAX_POINTS) for _ in range(6)]
        times, positions, command_positions, forces, force_targets, torques = buffers
        start = previous = time.monotonic()
        last_plot = 0.0
        filtered_ratio = None
        print("视觉-触觉控制已启动；按q或Ctrl+C退出。")

        while not stop_event.is_set():
            ok, image = camera.read()
            if not ok:
                raise RuntimeError("摄像头读取失败")
            image = cv2.flip(image, 1)
            image = detector.findHands(image, draw=True)
            landmarks = detector.findPosition(image, draw=False)
            ratio = normalized_pinch_ratio(landmarks)
            if ratio is not None:
                filtered_ratio = filter_pinch_ratio(filtered_ratio, ratio)
                target = ratio_to_position(filtered_ratio)
                state.set_visual_target(target)

                thumb = tuple(landmarks[4][1:3])
                index = tuple(landmarks[8][1:3])
                center = (
                    (thumb[0] + index[0]) // 2,
                    (thumb[1] + index[1]) // 2,
                )
                cv2.line(image, thumb, index, (255, 0, 255), 3)
                cv2.circle(image, thumb, 7, (255, 0, 255), cv2.FILLED)
                cv2.circle(image, index, 7, (255, 0, 255), cv2.FILLED)
                cv2.circle(image, center, 8, (0, 255, 255), cv2.FILLED)
                cv2.putText(
                    image,
                    f"Pinch {filtered_ratio:.2f} -> {target:.2f} rad",
                    (max(5, center[0] - 110), max(25, center[1] - 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2,
                )
            else:
                filtered_ratio = None
                cv2.putText(image, "Hand lost: hold", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)

            now = time.monotonic()
            fps = 1.0 / max(1e-6, now - previous)
            previous = now
            data = state.snapshot()
            if data["error"]:
                raise RuntimeError(data["error"])
            cv2.putText(image, f"FPS {fps:.0f}", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.putText(
                image,
                f"Pos {data['position']:.2f}  Target {data['command_position']:.2f}  "
                f"TacOffset {data['command_torque']:.3f} rad",
                (10, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2,
            )
            cv2.imshow("Visuo-Tactile Gripper", image)
            cv2.imshow("FlexiTac 12x30", render_tactile(
                data["matrix"], data["force"], data["peak"], data["cells"]
            ))

            if figure is not None and now - last_plot >= 1.0 / PLOT_RATE:
                elapsed = now - start
                for buffer, value in zip(buffers, (
                    elapsed, data["position"], data["command_position"], data["force"],
                    TARGET_FORCE, data["torque"],
                )):
                    buffer.append(value)
                for line, values in zip(lines, (
                    positions, command_positions, forces, force_targets, torques,
                )):
                    line.set_data(times, values)
                left, right = times[0], max(times[0] + 1.0, times[-1])
                for axis in axes:
                    axis.set_xlim(left, right)
                plot_image = render_figure(figure)
                last_plot = now
            if plot_image is not None:
                cv2.imshow("Motor & Tactile Dashboard", plot_image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        return 0

    except Exception as exc:
        print(f"运行错误: {exc}", file=sys.stderr)
        return 1
    finally:
        stop_event.set()
        controller.join(timeout=4.0)
        tactile.join(timeout=1.5)
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()
        if figure is not None:
            plt.close(figure)
        print("电机已失能，触觉串口和摄像头已关闭。")


if __name__ == "__main__":
    sys.exit(main())
