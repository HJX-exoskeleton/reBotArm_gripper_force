#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HLS3606 舵机回归初始位置脚本
==================================
功能:
  1. 将舵机安全地移动到预设的初始/零点位置
  2. 支持单舵机和多舵机模式
  3. 可调节回零速度和加速度
  4. 实时显示位置变化和运动状态
  5. 到达目标位置后自动释放扭矩 (可选)

使用方法:
  python hls3606_return_home.py                           # 默认参数
  python hls3606_return_home.py --ids 1,2,3               # 指定舵机ID
  python hls3606_return_home.py --home-pos 2048,2048      # 指定各舵机零点 (raw值)
  python hls3606_return_home.py --speed 30 --acc 20       # 指定回零速度、加速度
  python hls3606_return_home.py --no-release              # 不回零后不释放扭矩
  python hls3606_return_home.py --reset                   # 先执行舵机复位再回零
"""

import sys
import os
import time
import argparse
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scservo_sdk import *

# ==================== 默认配置参数 ====================
SERIAL_PORT = "/dev/ttyACM1"
BAUDRATE = 1000000

# 默认舵机 ID 列表
DEFAULT_SERVO_IDS = [1]

# 默认零点位置 (raw 值, 0-4095 对应 0-360°)
# 2048 = 180° (中间位置)
DEFAULT_HOME_POSITIONS = {1: 2048}

# 回零运动参数
# 速度: raw 值, 实际速度 = raw * 0.732 rpm
# 加速度: raw 值, 实际加速度 = raw * 8.7 deg/s²
DEFAULT_HOME_SPEED = 20       # ~14.64 rpm
DEFAULT_HOME_ACC = 10         # ~87 deg/s²
DEFAULT_TORQUE = 500          # 扭矩限制


def init_communication(port_name, baudrate):
    """初始化串口通信"""
    port_handler = PortHandler(port_name)
    packet_handler = hls(port_handler)

    if not port_handler.openPort():
        print(f"❌ 串口 {port_name} 打开失败!")
        return None, None

    if not port_handler.setBaudRate(baudrate):
        print(f"❌ 波特率 {baudrate} 设置失败!")
        port_handler.closePort()
        return None, None

    print(f"✅ 串口连接成功 ({port_name} @ {baudrate}bps)")
    return port_handler, packet_handler


def enable_torque(packet_handler, servo_ids):
    """使能所有舵机扭矩"""
    for sid in servo_ids:
        comm_result, error = packet_handler.write1ByteTxRx(sid, HLS_TORQUE_ENABLE, 1)
        if comm_result != COMM_SUCCESS:
            print(f"  ⚠️  [ID:{sid:03d}] 扭矩使能失败: {packet_handler.getTxRxResult(comm_result)}")
    time.sleep(0.1)


def disable_torque(packet_handler, servo_ids):
    """释放所有舵机扭矩"""
    for sid in servo_ids:
        packet_handler.write1ByteTxRx(sid, HLS_TORQUE_ENABLE, 0)


def read_current_positions(packet_handler, servo_ids):
    """读取当前所有舵机位置"""
    positions = {}
    for sid in servo_ids:
        pos, comm_result, error = packet_handler.ReadPos(sid)
        if comm_result == COMM_SUCCESS:
            positions[sid] = pos
        else:
            print(f"  ⚠️  [ID:{sid:03d}] 读取位置失败")
            positions[sid] = None
    return positions


def wait_until_arrived(packet_handler, servo_ids, timeout=10.0):
    """等待所有舵机到达目标位置 (运动完成)"""
    start_time = time.time()
    arrived = {sid: False for sid in servo_ids}

    while not all(arrived.values()):
        elapsed = time.time() - start_time
        if elapsed > timeout:
            print(f"\n⚠️  等待超时 ({timeout}s), 以下舵机可能未到达:")
            for sid, a in arrived.items():
                if not a:
                    print(f"    [ID:{sid:03d}] 仍在运动")
            return False

        for sid in servo_ids:
            if arrived[sid]:
                continue
            moving, comm_result, error = packet_handler.ReadMoving(sid)
            if comm_result == COMM_SUCCESS and moving == 0:
                arrived[sid] = True
                pos, _, _, _ = packet_handler.ReadPosSpeed(sid)
                if comm_result == COMM_SUCCESS and pos is not None:
                    angle = pos * 360.0 / 4095.0
                    print(f"  ✅ [ID:{sid:03d}] 已到达, 位置: {pos} ({angle:.1f}°)")

        if not all(arrived.values()):
            # 打印仍在运动的舵机
            moving_ids = [str(sid) for sid, a in arrived.items() if not a]
            print(f"\r  ⏳ 等待舵机 [ID:{','.join(moving_ids)}] ... {elapsed:.1f}s", end="")
            time.sleep(0.2)

    print("")  # 换行
    return True


def move_to_home(packet_handler, servo_ids, home_positions, speed, acc, torque):
    """移动舵机到零点位置"""
    print(f"\n🏠 开始回归初始位置...")
    print(f"  速度: {speed} (raw) / {speed * 0.732:.1f} rpm")
    print(f"  加速度: {acc} (raw) / {acc * 8.7:.1f} deg/s²")

    # 先读取当前位置
    print("\n📊 当前位置:")
    current_pos = read_current_positions(packet_handler, servo_ids)
    for sid in servo_ids:
        pos = current_pos.get(sid)
        if pos is not None:
            angle = pos * 360.0 / 4095.0
            home_angle = home_positions[sid] * 360.0 / 4095.0
            delta = home_angle - angle
            print(f"  [ID:{sid:03d}] 当前: {pos} ({angle:.1f}°) → 目标: {home_positions[sid]} ({home_angle:.1f}°) | Δ={delta:+.1f}°")
        else:
            print(f"  [ID:{sid:03d}] 读取失败")

    # 发送位置指令
    print(f"\n🚀 发送回零指令...")
    for sid in servo_ids:
        target_pos = home_positions[sid]
        comm_result, error = packet_handler.WritePosEx(sid, target_pos, speed, acc, torque)
        if comm_result != COMM_SUCCESS:
            print(f"  ❌ [ID:{sid:03d}] 指令发送失败: {packet_handler.getTxRxResult(comm_result)}")
        else:
            target_angle = target_pos * 360.0 / 4095.0
            print(f"  ✅ [ID:{sid:03d}] 目标位置: {target_pos} ({target_angle:.1f}°)")

    # 等待到达
    return wait_until_arrived(packet_handler, servo_ids)


def reset_servos(packet_handler, servo_ids):
    """复位舵机 (清除圈数计数)"""
    print(f"\n🔄 执行舵机复位...")
    for sid in servo_ids:
        comm_result, error = packet_handler.reSet(sid)
        if comm_result != COMM_SUCCESS:
            print(f"  ❌ [ID:{sid:03d}] 复位失败: {packet_handler.getTxRxResult(comm_result)}")
        else:
            print(f"  ✅ [ID:{sid:03d}] 复位成功")
        time.sleep(0.5)
    # 复位后等待舵机重启
    print("  ⏳ 等待舵机重启...")
    time.sleep(2.0)


def print_final_status(packet_handler, servo_ids):
    """打印最终位置状态"""
    print(f"\n📊 最终位置确认:")
    for sid in servo_ids:
        pos, comm_result, error = packet_handler.ReadPos(sid)
        if comm_result == COMM_SUCCESS:
            angle = pos * 360.0 / 4095.0
            print(f"  [ID:{sid:03d}] 位置: {pos} ({angle:.1f}°) ✅")
        else:
            print(f"  [ID:{sid:03d}] 读取失败 ⚠️")


def main():
    parser = argparse.ArgumentParser(
        description="HLS3606 舵机回归初始位置",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python hls3606_return_home.py
  python hls3606_return_home.py --ids 1,2,3 --home-pos 2048,2048,1024
  python hls3606_return_home.py --speed 30 --acc 20
  python hls3606_return_home.py --reset --no-release
        """
    )
    parser.add_argument("--port", type=str, default=SERIAL_PORT, help=f"串口设备 (默认: {SERIAL_PORT})")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE, help=f"波特率 (默认: {BAUDRATE})")
    parser.add_argument("--ids", type=str, default="1", help="舵机ID列表, 逗号分隔 (默认: 1)")
    parser.add_argument("--home-pos", type=str, default=None, help="各舵机零点位置(raw值), 逗号分隔 (默认: 各2048)")
    parser.add_argument("--speed", type=int, default=DEFAULT_HOME_SPEED, help=f"回零速度 raw 值 (默认: {DEFAULT_HOME_SPEED})")
    parser.add_argument("--acc", type=int, default=DEFAULT_HOME_ACC, help=f"回零加速度 raw 值 (默认: {DEFAULT_HOME_ACC})")
    parser.add_argument("--torque", type=int, default=DEFAULT_TORQUE, help=f"扭矩限制 (默认: {DEFAULT_TORQUE})")
    parser.add_argument("--no-release", action="store_true", help="到达后不释放扭矩 (保持使能)")
    parser.add_argument("--reset", action="store_true", help="先执行舵机复位再回零")
    parser.add_argument("--timeout", type=float, default=15.0, help="等待超时时间秒 (默认: 15)")
    args = parser.parse_args()

    # 解析舵机 ID
    servo_ids = [int(x.strip()) for x in args.ids.split(",")]

    # 解析零点位置
    if args.home_pos:
        home_raw = [int(x.strip()) for x in args.home_pos.split(",")]
        if len(home_raw) != len(servo_ids):
            print(f"❌ --home-pos 数量 ({len(home_raw)}) 与 --ids 数量 ({len(servo_ids)}) 不匹配")
            sys.exit(1)
        home_positions = {sid: pos for sid, pos in zip(servo_ids, home_raw)}
    else:
        home_positions = {sid: DEFAULT_HOME_POSITIONS.get(sid, 2048) for sid in servo_ids}

    print("=" * 50)
    print("  HLS3606 回归初始位置")
    print("=" * 50)
    print(f"  舵机: {servo_ids}")
    for sid in servo_ids:
        print(f"    [ID:{sid:03d}] 零点: {home_positions[sid]} ({home_positions[sid] * 360.0 / 4095.0:.1f}°)")
    print(f"  串口: {args.port}")

    # 初始化通信
    print(f"\n[1] 初始化串口通信...")
    port_handler, packet_handler = init_communication(args.port, args.baudrate)
    if port_handler is None:
        sys.exit(1)

    try:
        # 可选: 复位舵机
        if args.reset:
            reset_servos(packet_handler, servo_ids)

        # 使能扭矩
        print(f"\n[2] 使能舵机扭矩...")
        enable_torque(packet_handler, servo_ids)

        # 回零
        print(f"\n[3] 回归初始位置...")
        success = move_to_home(packet_handler, servo_ids, home_positions,
                               args.speed, args.acc, args.torque)

        if success:
            print(f"\n✅ 所有舵机已到达初始位置!")
            print_final_status(packet_handler, servo_ids)
        else:
            print(f"\n⚠️  部分舵机未能在超时时间内到达")

        # 释放扭矩
        if not args.no_release:
            print(f"\n[4] 释放舵机扭矩...")
            disable_torque(packet_handler, servo_ids)
            print("  ✅ 扭矩已释放 (舵机可自由转动)")
        else:
            print(f"\n[4] 保持扭矩使能 (舵机锁止在当前位置)")

        print(f"\n🎉 回零任务完成!")

    except KeyboardInterrupt:
        print(f"\n⚠️  用户中断, 正在释放扭矩...")
        disable_torque(packet_handler, servo_ids)
    except Exception as e:
        print(f"\n❌ 异常: {e}")
        import traceback
        traceback.print_exc()
        disable_torque(packet_handler, servo_ids)
    finally:
        port_handler.closePort()
        print("🔌 串口已关闭")


if __name__ == "__main__":
    main()
