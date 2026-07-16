#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""达妙夹爪电机位置读取工具。

使用与 DM_Motor_test.py 相同的 DM_CAN 串口通信链路。程序不会使能电机，
适合在手动转动夹爪时读取张开/闭合机械限位位置。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any

import serial
import yaml

from DM_CAN import DM_Motor_Type, DM_variable, Motor, MotorControl


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1] / "Servo_control" / "config" / "gripper.yaml"
)
BAUDRATE = 921600

MOTOR_TYPES = {
    "4310": DM_Motor_Type.DM4310,
    "4310_48v": DM_Motor_Type.DM4310_48V,
    "4340": DM_Motor_Type.DM4340,
    "4340_48v": DM_Motor_Type.DM4340_48V,
    "6006": DM_Motor_Type.DM6006,
    "8006": DM_Motor_Type.DM8006,
    "8009": DM_Motor_Type.DM8009,
    "10010l": DM_Motor_Type.DM10010L,
    "10010": DM_Motor_Type.DM10010,
    "h3510": DM_Motor_Type.DMH3510,
    "h6215": DM_Motor_Type.DMH6215,
    "g6220": DM_Motor_Type.DMG6220,
}


def parse_int(value: Any) -> int:
    """同时接受 YAML 整数、十进制字符串和 0x 前缀字符串。"""
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def load_motor_config(config_path: Path) -> dict[str, Any]:
    try:
        with config_path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except FileNotFoundError as exc:
        raise ValueError(f"配置文件不存在: {config_path}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"YAML 格式错误: {exc}") from exc

    grippers = data.get("gripper")
    if not isinstance(grippers, list) or not grippers:
        raise ValueError("配置中缺少非空的 gripper 列表")

    gripper = grippers[0]
    try:
        result = {
            "channel": str(data["channel"]),
            "motor_id": parse_int(gripper["motor_id"]),
            "feedback_id": parse_int(gripper["feedback_id"]),
            "model": gripper.get("model", "4310"),
        }
    except KeyError as exc:
        raise ValueError(f"配置缺少字段: {exc.args[0]}") from exc

    model_name = str(result["model"]).lower().replace("dm", "")
    try:
        result["motor_type"] = MOTOR_TYPES[model_name]
    except KeyError as exc:
        raise ValueError(f"不支持的达妙电机型号: {result['model']}") from exc
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="安全读取达妙夹爪电机位置")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="夹爪 YAML 配置文件")
    parser.add_argument("--port", help="覆盖 YAML 中的串口设备")
    parser.add_argument("--rate", type=float, default=20.0, help="刷新频率，默认 20 Hz")
    parser.add_argument("--timeout", type=float, default=0.05, help="串口读取超时，默认 0.05 s")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0:
        print("错误: --rate 必须是大于 0 的有限数", file=sys.stderr)
        return 2
    if not math.isfinite(args.timeout) or args.timeout < 0:
        print("错误: --timeout 必须是大于等于 0 的有限数", file=sys.stderr)
        return 2

    try:
        cfg = load_motor_config(args.config.resolve())
    except ValueError as exc:
        print(f"配置错误: {exc}", file=sys.stderr)
        return 2

    port = args.port or cfg["channel"]
    motor = Motor(cfg["motor_type"], cfg["motor_id"], cfg["feedback_id"])
    serial_device = None
    control = None

    print("正在初始化电机控制器（只读/失能模式）...")
    print(
        f"配置: port={port}, model=DM{cfg['model']}, "
        f"motor_id=0x{cfg['motor_id']:02X}, feedback_id=0x{cfg['feedback_id']:02X}"
    )

    try:
        # MotorControl 会重新打开传入的 serial.Serial，与已验证的测试代码保持一致。
        serial_device = serial.Serial(port, BAUDRATE, timeout=args.timeout)
        control = MotorControl(serial_device)
        control.addMotor(motor)

        # 明确下发失能命令，避免电机保留上一个程序的使能状态。
        control.disable(motor)
        time.sleep(0.05)

        # 读取一个不会改变电机状态的参数，尽早发现串口、CAN ID 或反馈 ID 错误。
        reported_master_id = control.read_motor_param(motor, DM_variable.MST_ID)
        if reported_master_id is None:
            raise RuntimeError("未收到电机响应，请检查串口、供电、CAN 接线及电机 ID")

        print("\n电机通信正常，且已失能。请用手转动夹爪：")
        print("  完全张开时记录 P_OPEN，完全闭合时记录 P_CLOSE；Ctrl+C 退出。\n")

        period = 1.0 / args.rate
        next_tick = time.monotonic()
        while True:
            # getPosition() 只是读取本地缓存；必须先主动请求状态才能获得新位置。
            control.refresh_motor_status(motor)
            position = motor.getPosition()
            velocity = motor.getVelocity()
            torque = motor.getTorque()
            print(
                f"\r位置: {position:>9.4f} rad | "
                f"速度: {velocity:>8.4f} rad/s | 力矩: {torque:>8.4f} Nm",
                end="",
                flush=True,
            )

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                # 通信耗时超过周期时直接重新对齐，避免累计漂移和忙循环。
                next_tick = time.monotonic()

    except KeyboardInterrupt:
        print("\n\n读取结束。")
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


if __name__ == "__main__":
    sys.exit(main())


# 0： 闭合 ， -5.8 张开