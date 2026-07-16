#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""当前夹爪结构的端点回零/验证程序。

新夹爪的绝对位置定义：闭合 0.0 rad，张开 -5.8 rad。程序使用与
DM_Motor_test.py 相同的 DM_CAN 串口链路和 Torque_Pos 控制模式。
"""

from __future__ import annotations

import argparse
import sys
import time

import serial

from DM_CAN import Control_Type, DM_variable, Motor, MotorControl
from motor_pos_detection_read import BAUDRATE, DEFAULT_CONFIG, load_motor_config


P_CLOSE = 0.0
P_OPEN = -5.8

# control_pos_force() 中速度放大 100 倍、电流标幺值放大 10000 倍。
MOVE_VELOCITY = 300       # 3.0 rad/s
CURRENT_LIMIT = 300       # 最大电流的 3%
POSITION_TOLERANCE = 0.08 # rad
VELOCITY_TOLERANCE = 0.15 # rad/s
SETTLE_SAMPLES = 10
CONTROL_RATE = 50.0       # Hz
MOVE_TIMEOUT = 8.0        # s


def move_to_endpoint(
    motor: Motor,
    control: MotorControl,
    target: float,
    name: str,
) -> float:
    """在软件限位内移动到已知端点，并等待位置稳定。"""
    if not P_OPEN <= target <= P_CLOSE:
        raise ValueError(f"目标 {target} rad 超出软件限位 [{P_OPEN}, {P_CLOSE}]")

    period = 1.0 / CONTROL_RATE
    deadline = time.monotonic() + MOVE_TIMEOUT
    settled = 0

    while time.monotonic() < deadline:
        control.control_pos_force(motor, target, MOVE_VELOCITY, CURRENT_LIMIT)
        position = motor.getPosition()
        velocity = motor.getVelocity()
        error = target - position

        print(
            f"\r{name}: 目标 {target:>6.2f} rad | "
            f"位置 {position:>7.3f} rad | 误差 {error:>+7.3f} rad | "
            f"速度 {velocity:>7.3f} rad/s",
            end="",
            flush=True,
        )

        if abs(error) <= POSITION_TOLERANCE and abs(velocity) <= VELOCITY_TOLERANCE:
            settled += 1
            if settled >= SETTLE_SAMPLES:
                print(f"\n{name}端点到达，实测位置: {position:.4f} rad")
                return position
        else:
            settled = 0

        time.sleep(period)

    raise RuntimeError(f"{name}动作超时，最后位置 {motor.getPosition():.4f} rad")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="新结构夹爪端点回零与验证")
    parser.add_argument("--config", type=str, default=str(DEFAULT_CONFIG), help="夹爪 YAML 配置")
    parser.add_argument("--port", help="覆盖 YAML 中的串口设备")
    parser.add_argument(
        "--yes",
        action="store_true",
        help="跳过启动前的安全确认",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        from pathlib import Path

        cfg = load_motor_config(Path(args.config).resolve())
    except ValueError as exc:
        print(f"配置错误: {exc}", file=sys.stderr)
        return 2

    port = args.port or cfg["channel"]
    motor = Motor(cfg["motor_type"], cfg["motor_id"], cfg["feedback_id"])
    serial_device = None
    control = None

    print("新夹爪位置范围：")
    print(f"  完全闭合 P_CLOSE = {P_CLOSE:.1f} rad")
    print(f"  完全张开 P_OPEN  = {P_OPEN:.1f} rad")
    print(f"  总行程 Delta_P   = {P_CLOSE - P_OPEN:.1f} rad")
    print(
        f"硬件配置：{port}, DM{cfg['model']}, "
        f"ID=0x{cfg['motor_id']:02X}/0x{cfg['feedback_id']:02X}"
    )

    if not args.yes:
        answer = input("\n请清空夹爪周围物体，确认可以先张开、再闭合。[y/N] ").strip().lower()
        if answer not in {"y", "yes"}:
            print("已取消，电机未使能。")
            return 0

    try:
        serial_device = serial.Serial(port, BAUDRATE, timeout=0.05)
        control = MotorControl(serial_device)
        control.addMotor(motor)

        # 先失能并验证通信，防止继承其他程序遗留的运动命令。
        control.disable(motor)
        master_id = control.read_motor_param(motor, DM_variable.MST_ID)
        if master_id is None:
            raise RuntimeError("未收到电机响应，请检查串口、CAN 接线和电机 ID")

        if not control.switchControlMode(motor, Control_Type.Torque_Pos):
            raise RuntimeError("切换 Torque_Pos 控制模式失败")
        control.enable(motor)

        print("\n[阶段 1/2] 移动到完全张开位置")
        measured_open = move_to_endpoint(motor, control, P_OPEN, "张开")

        print("\n[阶段 2/2] 移动到完全闭合位置")
        measured_close = move_to_endpoint(motor, control, P_CLOSE, "闭合")

        print("\n回零验证完成：")
        print(f"  张开位置: {measured_open:.4f} rad（设定 {P_OPEN:.1f}）")
        print(f"  闭合位置: {measured_close:.4f} rad（设定 {P_CLOSE:.1f}）")
        print(f"  实测行程: {measured_close - measured_open:.4f} rad")
        return 0

    except KeyboardInterrupt:
        print("\n用户中断，正在失能电机。")
        return 130
    except (serial.SerialException, OSError, RuntimeError, ValueError) as exc:
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


if __name__ == "__main__":
    sys.exit(main())
