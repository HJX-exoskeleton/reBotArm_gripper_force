#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基于手指捏合比例的达妙夹爪连续遥操作与实时曲线。"""

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

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-rebotarm")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import HandTrackingModule as htm


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CONTROL_PACKAGE_ROOT = PROJECT_ROOT / "Servo_control" / "reBotArm_control_py"
if str(CONTROL_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_PACKAGE_ROOT))

from actuator.gripper import Gripper  # noqa: E402


GRIPPER_CONFIG = PROJECT_ROOT / "Servo_control" / "config" / "gripper.yaml"
P_OPEN = -5.8
P_CLOSE = 0.0
POSITION_FAULT_MARGIN = 0.75

# 拇指-食指距离 / 手腕-中指掌指关节距离，消除手与摄像头距离的影响。
PINCH_RATIO_CLOSE = 0.25
PINCH_RATIO_OPEN = 1.45
PINCH_FILTER_ALPHA_SLOW = 0.30
PINCH_FILTER_ALPHA_FAST = 0.72
PINCH_FAST_DELTA = 0.08

CONTROL_RATE = 100.0
VELOCITY_LIMIT = 6.0
STARTUP_POSITION = 0.0
STARTUP_VELOCITY_LIMIT = 2.0
STARTUP_POSITION_TOLERANCE = 0.05
STARTUP_TIMEOUT = 8.0
HARDWARE_READY_TIMEOUT = STARTUP_TIMEOUT + 4.0
HAND_LOSS_TIMEOUT = 0.35
PLOT_RATE = 6.0
PLOT_HISTORY = 12.0
MAX_POINTS = round(PLOT_RATE * PLOT_HISTORY)


class HardwareWorker(threading.Thread):
    """独占 Gripper/motorbridge 的位置控制线程。"""

    def __init__(self, config_path: Path, velocity_limit: float = VELOCITY_LIMIT):
        super().__init__(daemon=True)
        self.config_path = config_path
        self.velocity_limit = float(velocity_limit)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._target: Optional[float] = None
        self._target_time = 0.0
        self._position = 0.0
        self._velocity = 0.0
        self._torque = 0.0
        self._command = 0.0
        self._enabled = False
        self._error: Optional[str] = None

    @staticmethod
    def clamp_position(position: float) -> float:
        return min(P_CLOSE, max(P_OPEN, float(position)))

    def set_target(self, position: float) -> None:
        with self._lock:
            self._target = self.clamp_position(position)
            self._target_time = time.monotonic()

    def request_stop(self) -> None:
        self._stop_event.set()

    def wait_ready(self, timeout: float) -> bool:
        self._ready_event.wait(timeout)
        with self._lock:
            return self._enabled and self._error is None

    def snapshot(self):
        with self._lock:
            return {
                "position": self._position,
                "velocity": self._velocity,
                "torque": self._torque,
                "command": self._command,
                "enabled": self._enabled,
                "error": self._error,
            }

    def _move_to_startup_position(self, gripper: Gripper) -> tuple[float, float, float]:
        """限速移动到启动零位，到位后才允许进入手势控制。"""
        target = self.clamp_position(STARTUP_POSITION)
        deadline = time.monotonic() + STARTUP_TIMEOUT
        position, velocity, torque = gripper.get_state(request=True)

        while not self._stop_event.is_set():
            if not all(math.isfinite(value) for value in (position, velocity, torque)):
                raise RuntimeError("零位初始化时电机反馈包含 NaN/Inf")
            if (
                position < P_OPEN - POSITION_FAULT_MARGIN
                or position > P_CLOSE + POSITION_FAULT_MARGIN
            ):
                raise RuntimeError(f"零位初始化时位置 {position:.3f} rad 严重越界")

            with self._lock:
                self._position, self._velocity, self._torque = position, velocity, torque
                self._command = target

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
            gripper = Gripper(str(self.config_path))
            gripper.disable()
            if not gripper.mode_pos_vel():
                raise RuntimeError("切换 POS_VEL 模式失败")
            if not gripper.enable():
                raise RuntimeError("电机使能失败")

            print(f"电机：正在限速初始化到 {STARTUP_POSITION:.1f} rad 零位……")
            position, velocity, torque = self._move_to_startup_position(gripper)
            print("电机：零位初始化完成。")
            startup_target = self.clamp_position(STARTUP_POSITION)
            with self._lock:
                self._position, self._velocity, self._torque = position, velocity, torque
                self._command = startup_target
                self._target = startup_target
                self._target_time = time.monotonic()
                self._enabled = True
            self._ready_event.set()

            held_after_loss = False
            period = 1.0 / CONTROL_RATE
            next_tick = time.monotonic()
            while not self._stop_event.is_set():
                with self._lock:
                    target = self._target
                    target_age = time.monotonic() - self._target_time

                # 手势丢失时立即以当前反馈为新目标，停止尚未完成的运动。
                if target is None or target_age > HAND_LOSS_TIMEOUT:
                    if not held_after_loss:
                        target = self.clamp_position(position)
                        with self._lock:
                            self._target = target
                        held_after_loss = True
                else:
                    held_after_loss = False

                target = self.clamp_position(target)
                gripper.pos_vel(target, self.velocity_limit)
                position, velocity, torque = gripper.get_state(request=False)
                if not all(math.isfinite(value) for value in (position, velocity, torque)):
                    raise RuntimeError("电机反馈包含 NaN/Inf")
                if (
                    position < P_OPEN - POSITION_FAULT_MARGIN
                    or position > P_CLOSE + POSITION_FAULT_MARGIN
                ):
                    raise RuntimeError(f"位置 {position:.3f} rad 严重超出机械行程")

                with self._lock:
                    self._position, self._velocity, self._torque = position, velocity, torque
                    self._command = target

                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()

        except Exception as exc:
            with self._lock:
                self._error = str(exc)
                self._enabled = False
            self._ready_event.set()
        finally:
            if gripper is not None:
                try:
                    gripper.disconnect()
                except Exception:
                    pass
            with self._lock:
                self._enabled = False


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="视觉手势连续控制达妙夹爪")
    parser.add_argument("--camera", type=int, default=0, help="摄像头编号，默认0")
    parser.add_argument("--velocity", type=float, default=VELOCITY_LIMIT, help="电机速度上限 rad/s")
    parser.add_argument("--no-plot", action="store_true", help="不显示电机曲线窗口")
    parser.add_argument("--yes", action="store_true", help="跳过电机使能前确认")
    return parser


def normalized_pinch_ratio(landmarks) -> Optional[float]:
    """返回归一化捏合比例；关键点缺失或手掌尺度异常时返回None。"""
    if len(landmarks) <= 9:
        return None
    thumb = np.array(landmarks[4][1:3], dtype=np.float64)
    index = np.array(landmarks[8][1:3], dtype=np.float64)
    wrist = np.array(landmarks[0][1:3], dtype=np.float64)
    middle_mcp = np.array(landmarks[9][1:3], dtype=np.float64)
    palm_scale = float(np.linalg.norm(middle_mcp - wrist))
    if palm_scale < 10.0:
        return None
    return float(np.linalg.norm(index - thumb) / palm_scale)


def ratio_to_position(ratio: float) -> float:
    ratio = float(np.clip(ratio, PINCH_RATIO_CLOSE, PINCH_RATIO_OPEN))
    return float(np.interp(
        ratio,
        (PINCH_RATIO_CLOSE, PINCH_RATIO_OPEN),
        (P_CLOSE, P_OPEN),
    ))


def filter_pinch_ratio(previous: Optional[float], current: float) -> float:
    """变化大时快速跟随，变化小时加强滤波抑制抖动。"""
    if previous is None:
        return float(current)
    delta = float(current - previous)
    alpha = PINCH_FILTER_ALPHA_FAST if abs(delta) >= PINCH_FAST_DELTA else PINCH_FILTER_ALPHA_SLOW
    return previous + alpha * delta


def make_plot():
    plt.style.use("fast")
    figure, axes = plt.subplots(3, 1, figsize=(7, 6), sharex=True, dpi=90)
    figure.suptitle("Gripper Teleoperation Dashboard", fontsize=12, fontweight="bold")
    position_line, = axes[0].plot([], [], "r-", label="Actual")
    command_line, = axes[0].plot([], [], color="purple", linestyle="--", label="Command")
    velocity_line, = axes[1].plot([], [], "g-", label="Velocity")
    torque_line, = axes[2].plot([], [], "b-", label="Torque")
    axes[0].set_ylabel("Position (rad)")
    axes[1].set_ylabel("Velocity (rad/s)")
    axes[2].set_ylabel("Torque (N·m)")
    axes[2].set_xlabel("Time (s)")
    axes[0].set_ylim(P_OPEN - 0.2, P_CLOSE + 0.2)
    for axis in axes:
        axis.grid(True, linestyle=":", alpha=0.6)
        axis.legend(loc="upper right")
    figure.tight_layout()
    return figure, axes, (position_line, command_line, velocity_line, torque_line)


def figure_to_bgr(figure) -> np.ndarray:
    figure.canvas.draw()
    rgba = np.asarray(figure.canvas.buffer_rgba())
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)


def main() -> int:
    args = build_parser().parse_args()
    if not math.isfinite(args.velocity) or args.velocity <= 0:
        print("--velocity 必须是大于0的有限数", file=sys.stderr)
        return 2
    stop_event = threading.Event()

    def handle_signal(signum, _frame):
        print(f"\n收到信号 {signum}，正在安全退出……")
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    print("视觉映射：手指捏合→0 rad闭合，手指张开→-5.8 rad张开")
    if not args.yes:
        answer = input("请确认夹爪运动范围内没有人员或障碍物。[y/N] ").strip().lower()
        if answer not in {"y", "yes"}:
            print("已取消，电机未使能。")
            return 0

    hardware = HardwareWorker(GRIPPER_CONFIG, args.velocity)
    camera = None
    hardware.start()
    if not hardware.wait_ready(HARDWARE_READY_TIMEOUT):
        error = hardware.snapshot()["error"] or "电机初始化超时"
        hardware.request_stop()
        hardware.join(timeout=3.0)
        print(f"电机初始化失败: {error}", file=sys.stderr)
        return 1

    times = deque(maxlen=MAX_POINTS)
    positions = deque(maxlen=MAX_POINTS)
    commands = deque(maxlen=MAX_POINTS)
    velocities = deque(maxlen=MAX_POINTS)
    torques = deque(maxlen=MAX_POINTS)
    figure = axes = lines = None
    plot_image = None

    try:
        if not args.no_plot:
            figure, axes, lines = make_plot()

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

        start_time = time.monotonic()
        previous_frame_time = start_time
        last_plot_time = 0.0
        filtered_ratio = None
        print("系统已启动；按 q 或 Ctrl+C 退出。")

        while not stop_event.is_set():
            success, image = camera.read()
            if not success:
                raise RuntimeError("摄像头读取失败")
            image = cv2.flip(image, 1)
            image = detector.findHands(image, draw=True)
            landmarks = detector.findPosition(image, draw=False)

            ratio = normalized_pinch_ratio(landmarks)
            if ratio is not None:
                filtered_ratio = filter_pinch_ratio(filtered_ratio, ratio)
                target = ratio_to_position(filtered_ratio)
                hardware.set_target(target)

                thumb = tuple(landmarks[4][1:3])
                index = tuple(landmarks[8][1:3])
                center = ((thumb[0] + index[0]) // 2, (thumb[1] + index[1]) // 2)
                cv2.line(image, thumb, index, (255, 0, 255), 3)
                cv2.circle(image, center, 7, (255, 0, 255), cv2.FILLED)
                cv2.putText(
                    image, f"Ratio {filtered_ratio:.2f} -> {target:.2f} rad",
                    (max(5, center[0] - 100), max(25, center[1] - 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2,
                )
            else:
                filtered_ratio = None
                cv2.putText(
                    image, "Hand lost: holding current position", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2,
                )

            now = time.monotonic()
            fps = 1.0 / max(1e-6, now - previous_frame_time)
            previous_frame_time = now
            state = hardware.snapshot()
            if state["error"]:
                raise RuntimeError(state["error"])
            cv2.putText(image, f"FPS {fps:.0f}", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.putText(
                image,
                f"Motor pos {state['position']:.3f}  target {state['command']:.3f}",
                (10, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2,
            )

            if figure is not None and now - last_plot_time >= 1.0 / PLOT_RATE:
                elapsed = now - start_time
                times.append(elapsed)
                positions.append(state["position"])
                commands.append(state["command"])
                velocities.append(state["velocity"])
                torques.append(state["torque"])
                position_line, command_line, velocity_line, torque_line = lines
                position_line.set_data(times, positions)
                command_line.set_data(times, commands)
                velocity_line.set_data(times, velocities)
                torque_line.set_data(times, torques)
                left, right = times[0], max(times[0] + 1.0, times[-1])
                for axis in axes:
                    axis.set_xlim(left, right)
                if velocities:
                    vmax = max(0.2, max(abs(v) for v in velocities) * 1.15)
                    axes[1].set_ylim(-vmax, vmax)
                if torques:
                    tmax = max(0.1, max(abs(v) for v in torques) * 1.15)
                    axes[2].set_ylim(-tmax, tmax)
                plot_image = figure_to_bgr(figure)
                last_plot_time = now

            cv2.imshow("Normalized Hand Gripper Control", image)
            if plot_image is not None:
                cv2.imshow("Motor Dashboard", plot_image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        return 0

    except Exception as exc:
        print(f"运行错误: {exc}", file=sys.stderr)
        return 1
    finally:
        hardware.request_stop()
        hardware.join(timeout=4.0)
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()
        if figure is not None:
            plt.close(figure)
        print("电机已失能，摄像头和窗口已关闭。")


if __name__ == "__main__":
    sys.exit(main())
