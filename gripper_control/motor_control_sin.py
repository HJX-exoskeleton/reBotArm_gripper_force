#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""达妙夹爪安全正弦位置控制与实时状态显示。

  正常运行：

  python3 gripper_control/motor_control_sin.py

  运行10秒、不显示曲线：

  python3 gripper_control/motor_control_sin.py --duration 10 --no-plot

  调整正弦频率：

  python3 gripper_control/motor_control_sin.py --frequency 0.1


"""

from __future__ import annotations

import argparse
import math
import multiprocessing as mp
import queue
import signal
import sys
import time
from pathlib import Path

import serial

from DM_CAN import Control_Type, DM_variable, Motor, MotorControl
from motor_pos_detection_read import BAUDRATE, DEFAULT_CONFIG, load_motor_config


P_CLOSE = 0.0
P_OPEN = -5.8
DEFAULT_MARGIN = 0.10
DEFAULT_FREQUENCY = 0.15
DEFAULT_RATE = 50.0
DEFAULT_VELOCITY = 300  # control_pos_force 速度值放大 100 倍，即 3 rad/s
DEFAULT_CURRENT = 300   # 电流标幺值放大 10000 倍，即最大电流的 3%
POSITION_GUARD = 0.15
PLOT_STOP = None

running = True


def signal_handler(signum, _frame):
    global running
    print(f"\n[退出] 收到信号 {signum}，正在失能电机……")
    running = False


def put_latest(data_queue: mp.Queue, sample) -> None:
    """队列满时丢弃旧样本，绘图不能拖慢电机控制循环。"""
    try:
        data_queue.put_nowait(sample)
    except queue.Full:
        try:
            data_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            data_queue.put_nowait(sample)
        except queue.Full:
            pass


def real_time_plotter(data_queue: mp.Queue) -> None:
    import matplotlib.pyplot as plt
    from collections import deque

    max_len = 500
    times = deque(maxlen=max_len)
    commands = deque(maxlen=max_len)
    positions = deque(maxlen=max_len)
    velocities = deque(maxlen=max_len)
    torques = deque(maxlen=max_len)

    fig, (ax_pos, ax_vel, ax_tau) = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    fig.canvas.manager.set_window_title("Gripper sinusoidal position control")
    line_cmd, = ax_pos.plot([], [], "k--", label="Command")
    line_pos, = ax_pos.plot([], [], "r-", label="Position")
    line_vel, = ax_vel.plot([], [], "g-", label="Velocity")
    line_tau, = ax_tau.plot([], [], "b-", label="Torque")

    ax_pos.set_ylabel("Position (rad)")
    ax_pos.set_ylim(P_OPEN - 0.2, P_CLOSE + 0.2)
    ax_vel.set_ylabel("Velocity (rad/s)")
    ax_tau.set_ylabel("Torque (N·m)")
    ax_tau.set_xlabel("Time (s)")
    for axis in (ax_pos, ax_vel, ax_tau):
        axis.grid(True, linestyle="--", alpha=0.5)
        axis.legend(loc="upper right")

    plt.tight_layout()
    plt.show(block=False)

    active = True
    while active and plt.fignum_exists(fig.number):
        received = False
        try:
            while True:
                sample = data_queue.get_nowait()
                if sample is PLOT_STOP:
                    active = False
                    break
                timestamp, command, position, velocity, torque = sample
                times.append(timestamp)
                commands.append(command)
                positions.append(position)
                velocities.append(velocity)
                torques.append(torque)
                received = True
        except queue.Empty:
            pass

        if received and len(times) > 1:
            line_cmd.set_data(times, commands)
            line_pos.set_data(times, positions)
            line_vel.set_data(times, velocities)
            line_tau.set_data(times, torques)
            left, right = times[0], times[-1]
            if right <= left:
                right = left + 0.1
            ax_pos.set_xlim(left, right)
            for axis, values in ((ax_vel, velocities), (ax_tau, torques)):
                low, high = min(values), max(values)
                padding = max(0.1, (high - low) * 0.1)
                axis.set_ylim(low - padding, high + padding)
            fig.canvas.draw_idle()

        plt.pause(0.02)

    plt.close(fig)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="新结构夹爪安全正弦位置控制")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="夹爪 YAML 配置")
    parser.add_argument("--port", help="覆盖 YAML 中的串口设备")
    parser.add_argument("--frequency", type=float, default=DEFAULT_FREQUENCY, help="正弦频率 Hz")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE, help="控制频率 Hz")
    parser.add_argument("--margin", type=float, default=DEFAULT_MARGIN, help="两端安全余量 rad")
    parser.add_argument("--duration", type=float, default=0.0, help="运行秒数；0 表示持续运行")
    parser.add_argument("--no-plot", action="store_true", help="不启动实时曲线窗口")
    parser.add_argument("--yes", action="store_true", help="跳过动作前安全确认")
    return parser


def validate_args(args) -> None:
    for name in ("frequency", "rate"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"--{name} 必须是大于 0 的有限数")
    if not math.isfinite(args.margin) or not 0 <= args.margin < (P_CLOSE - P_OPEN) / 2:
        raise ValueError(f"--margin 必须在 [0, {(P_CLOSE - P_OPEN) / 2:.2f}) 内")
    if not math.isfinite(args.duration) or args.duration < 0:
        raise ValueError("--duration 必须大于等于 0")


def main() -> int:
    global running
    args = build_parser().parse_args()
    try:
        validate_args(args)
        cfg = load_motor_config(args.config.resolve())
    except ValueError as exc:
        print(f"参数或配置错误: {exc}", file=sys.stderr)
        return 2

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    lower = P_OPEN + args.margin
    upper = P_CLOSE - args.margin
    center = (lower + upper) / 2.0
    amplitude = (upper - lower) / 2.0
    port = args.port or cfg["channel"]

    print("=== 新结构夹爪正弦位置控制 ===")
    print(f"硬件: {port}, DM{cfg['model']}, ID=0x{cfg['motor_id']:02X}/0x{cfg['feedback_id']:02X}")
    print(f"机械范围: [{P_OPEN:.2f}, {P_CLOSE:.2f}] rad")
    print(f"命令范围: [{lower:.2f}, {upper:.2f}] rad（余量 {args.margin:.2f} rad）")
    print(f"频率: {args.frequency:.3f} Hz，控制频率: {args.rate:.1f} Hz")

    if not args.yes:
        answer = input("请清空夹爪周围物体，确认开始往复运动。[y/N] ").strip().lower()
        if not running:
            print("已取消，电机未使能。")
            return 130
        if answer not in {"y", "yes"}:
            print("已取消，电机未使能。")
            return 0

    data_queue = None
    plot_process = None
    serial_device = None
    control = None
    motor = Motor(cfg["motor_type"], cfg["motor_id"], cfg["feedback_id"])

    try:
        serial_device = serial.Serial(port, BAUDRATE, timeout=0.05)
        control = MotorControl(serial_device)
        control.addMotor(motor)
        control.disable(motor)

        master_id = control.read_motor_param(motor, DM_variable.MST_ID)
        if master_id is None:
            raise RuntimeError("未收到电机响应，请检查串口、CAN 接线及电机 ID")
        if not control.switchControlMode(motor, Control_Type.Torque_Pos):
            raise RuntimeError("切换 Torque_Pos 控制模式失败")

        if not args.no_plot:
            data_queue = mp.Queue(maxsize=200)
            plot_process = mp.Process(target=real_time_plotter, args=(data_queue,), daemon=True)
            plot_process.start()

        control.enable(motor)
        start = time.monotonic()
        next_tick = start
        period = 1.0 / args.rate
        print("电机已使能，开始运动；按 Ctrl+C 安全退出。\n")

        while running:
            elapsed = time.monotonic() - start
            if args.duration > 0 and elapsed >= args.duration:
                break

            target = center + amplitude * math.sin(2.0 * math.pi * args.frequency * elapsed)
            # 数值保护：即使浮点计算出现边界误差，也不允许命令越界。
            target = min(upper, max(lower, target))
            control.control_pos_force(motor, target, DEFAULT_VELOCITY, DEFAULT_CURRENT)

            position = motor.getPosition()
            velocity = motor.getVelocity()
            torque = motor.getTorque()
            if not all(math.isfinite(value) for value in (position, velocity, torque)):
                raise RuntimeError("电机反馈包含 NaN/Inf，已停止运动")
            if position < P_OPEN - POSITION_GUARD or position > P_CLOSE + POSITION_GUARD:
                raise RuntimeError(f"位置 {position:.3f} rad 超出机械范围，已紧急停止")

            print(
                f"\r时间 {elapsed:7.2f}s | 目标 {target:7.3f} | "
                f"位置 {position:7.3f} | 速度 {velocity:7.3f} | 力矩 {torque:7.3f}",
                end="",
                flush=True,
            )
            if data_queue is not None:
                put_latest(data_queue, (elapsed, target, position, velocity, torque))

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

        print("\n运动结束，正在失能电机。")
        return 0

    except (serial.SerialException, OSError, RuntimeError) as exc:
        print(f"\n运行错误: {exc}", file=sys.stderr)
        return 1
    finally:
        if control is not None:
            try:
                control.disable(motor)
            except Exception:
                pass
        if serial_device is not None and serial_device.is_open:
            serial_device.close()
        if data_queue is not None:
            put_latest(data_queue, PLOT_STOP)
        if plot_process is not None:
            plot_process.join(timeout=1.0)
            if plot_process.is_alive():
                plot_process.terminate()
                plot_process.join(timeout=1.0)


if __name__ == "__main__":
    mp.freeze_support()
    sys.exit(main())
