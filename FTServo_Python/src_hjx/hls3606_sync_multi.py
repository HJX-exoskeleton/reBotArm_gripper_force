#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HLS3606 多舵机同步控制脚本
==================================
功能:
  1. 使用 SyncWrite 同步写多个舵机位置
  2. 使用 SyncRead 同步读取多个舵机状态
  3. 支持自定义多舵机运动序列
  4. 实时显示同步误差
  5. 支持示教/回放模式

同步写 vs 逐个写:
  - SyncWrite: 一次指令同时更新所有舵机位置 (延迟更低, 同步性更好)
  - 逐个 WritePosEx: 逐个发送, 有串行延迟

使用方法:
  python hls3606_sync_multi.py                                   # 默认同步控制
  python hls3606_sync_multi.py --mode sync_pos                   # 同步位置控制
  python hls3606_sync_multi.py --mode sync_read                  # 同步读取测试
  python hls3606_sync_multi.py --mode wave                       # 波浪运动
  python hls3606_sync_multi.py --mode relay                      # 接力运动
"""

import sys
import os
import time
import argparse
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scservo_sdk import *

# ==================== 配置参数 ====================
SERIAL_PORT = "/dev/ttyACM1"
BAUDRATE = 1000000

# 默认舵机 ID 范围
DEFAULT_SERVO_IDS = [7]

# 运动参数
DEFAULT_SPEED = 40
DEFAULT_ACC = 20
DEFAULT_TORQUE = 500

# 位置范围
POS_MIN = 512    # ~45°
POS_MID = 2048   # ~180°
POS_MAX = 3584   # ~315°


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
        packet_handler.write1ByteTxRx(sid, HLS_TORQUE_ENABLE, 1)
    time.sleep(0.1)


def disable_torque(packet_handler, servo_ids):
    """释放所有舵机扭矩"""
    for sid in servo_ids:
        packet_handler.write1ByteTxRx(sid, HLS_TORQUE_ENABLE, 0)


def sync_write_positions(packet_handler, servo_ids, positions, speed, acc, torque):
    """
    使用 SyncWrite 同步写入多个舵机位置

    Args:
        packet_handler: hls 协议处理器
        servo_ids: 舵机ID列表
        positions: 位置列表 (与 servo_ids 一一对应)
        speed: 速度
        acc: 加速度
        torque: 扭矩
    """
    for sid, pos in zip(servo_ids, positions):
        if not packet_handler.SyncWritePosEx(sid, pos, speed, acc, torque):
            print(f"  ⚠️  [ID:{sid:03d}] SyncWrite addParam 失败")

    comm_result = packet_handler.groupSyncWrite.txPacket()
    if comm_result != COMM_SUCCESS:
        print(f"  ❌ SyncWrite txPacket 失败: {packet_handler.getTxRxResult(comm_result)}")

    packet_handler.groupSyncWrite.clearParam()
    return comm_result == COMM_SUCCESS


def sync_read_positions(packet_handler, servo_ids):
    """
    使用 SyncRead 同步读取多个舵机位置

    Returns:
        dict: {servo_id: position}
    """
    group_sync_read = GroupSyncRead(packet_handler, HLS_PRESENT_POSITION_L, 4)

    for sid in servo_ids:
        if not group_sync_read.addParam(sid):
            print(f"  ⚠️  [ID:{sid:03d}] SyncRead addParam 失败")

    comm_result = group_sync_read.txRxPacket()
    if comm_result != COMM_SUCCESS:
        print(f"  ❌ SyncRead txRxPacket 失败: {packet_handler.getTxRxResult(comm_result)}")
        group_sync_read.clearParam()
        return {}

    positions = {}
    for sid in servo_ids:
        data_result, error = group_sync_read.isAvailable(sid, HLS_PRESENT_POSITION_L, 2)
        if data_result:
            pos = group_sync_read.getData(sid, HLS_PRESENT_POSITION_L, 2)
            positions[sid] = pos
        else:
            print(f"  ⚠️  [ID:{sid:03d}] SyncRead 数据不可用")
            positions[sid] = None

    group_sync_read.clearParam()
    return positions


# ==================== 运动模式 ====================

def mode_sync_position_control(packet_handler, servo_ids, args):
    """
    同步位置控制模式:
    所有舵机同时移动到相同或成比例的位置
    """
    print("\n" + "=" * 50)
    print("  模式: 同步位置控制")
    print("=" * 50)
    print(f"  舵机: {servo_ids}")
    print(f"  运动范围: {POS_MIN} - {POS_MAX}")

    positions_list = [
        [POS_MID] * len(servo_ids),               # 全部中间
        [POS_MAX] * len(servo_ids),               # 全部最大
        [POS_MIN] * len(servo_ids),               # 全部最小
        [POS_MID] * len(servo_ids),               # 全部回中
    ]

    print("\n开始同步位置控制 (Ctrl+C 停止)...")
    cycle = 0
    try:
        while True:
            for i, positions in enumerate(positions_list):
                print(f"\n  🔄 第 {cycle + 1} 轮, 步骤 {i + 1}/{len(positions_list)}")
                for sid, pos in zip(servo_ids, positions):
                    angle = pos * 360.0 / 4095.0
                    print(f"    [ID:{sid:03d}] → {pos} ({angle:.1f}°)")

                success = sync_write_positions(packet_handler, servo_ids, positions,
                                               args.speed, args.acc, args.torque)
                if not success:
                    print("    ⚠️  同步写入失败")

                # 等待运动完成
                time.sleep(args.step_delay)

                # 同步读取确认
                actual = sync_read_positions(packet_handler, servo_ids)
                print(f"    实际位置:", end="")
                for sid in servo_ids:
                    if actual.get(sid) is not None:
                        print(f" ID{sid}:{actual[sid]}", end="")
                print("")

            cycle += 1

    except KeyboardInterrupt:
        print("\n⚠️  用户中断")


def mode_wave(packet_handler, servo_ids, args):
    """
    波浪运动模式:
    舵机之间以相位差做正弦波运动, 模仿蛇形/波浪
    """
    print("\n" + "=" * 50)
    print("  模式: 波浪运动")
    print("=" * 50)

    n_servos = len(servo_ids)
    phase_offsets = [2.0 * np.pi * i / n_servos for i in range(n_servos)]
    amplitude = (POS_MAX - POS_MIN) // 2
    offset = POS_MID

    print(f"  舵机数量: {n_servos}")
    print(f"  相位差: {[f'{o:.1f}rad' for o in phase_offsets]}")
    print(f"  振幅: {amplitude}")
    print(f"  周期: {args.wave_period}s")

    print("\n开始波浪运动 (Ctrl+C 停止)...")
    start_time = time.time()
    cycle = 0

    try:
        while True:
            t = time.time() - start_time
            positions = []
            for i, sid in enumerate(servo_ids):
                phase = 2.0 * np.pi * t / args.wave_period + phase_offsets[i]
                pos = int(offset + amplitude * np.sin(phase))
                pos = np.clip(pos, POS_MIN, POS_MAX)
                positions.append(pos)

            sync_write_positions(packet_handler, servo_ids, positions,
                                 args.speed, args.acc, args.torque)

            if cycle % 20 == 0:
                pos_str = " | ".join([f"ID{sid}: {p * 360.0 / 4095.0:.0f}°" for sid, p in zip(servo_ids, positions)])
                print(f"\r  t={t:.1f}s | {pos_str}", end="")

            time.sleep(args.wave_dt)
            cycle += 1

    except KeyboardInterrupt:
        print("\n⚠️  用户中断")


def mode_relay(packet_handler, servo_ids, args):
    """
    接力运动模式:
    舵机依次运动, 前一个到达后下一个才开始
    """
    print("\n" + "=" * 50)
    print("  模式: 接力运动")
    print("=" * 50)
    print(f"  舵机: {servo_ids}")

    print("\n开始接力运动 (Ctrl+C 停止)...")
    try:
        while True:
            # 顺序接力: 1→2→3→4
            for sid in servo_ids:
                print(f"\n  ▶️  [ID:{sid:03d}] 运动到 {POS_MAX}...")
                packet_handler.WritePosEx(sid, POS_MAX, args.speed, args.acc, args.torque)
                # 等待到达
                while True:
                    moving, comm_result, _ = packet_handler.ReadMoving(sid)
                    if comm_result == COMM_SUCCESS and moving == 0:
                        break
                    time.sleep(0.1)
                pos, _, _, _ = packet_handler.ReadPosSpeed(sid)
                if comm_result == COMM_SUCCESS:
                    print(f"    ✅ [ID:{sid:03d}] 到达: {pos}")

            time.sleep(args.step_delay)

            # 逆序接力: 4→3→2→1
            for sid in reversed(servo_ids):
                print(f"\n  ◀️  [ID:{sid:03d}] 运动到 {POS_MIN}...")
                packet_handler.WritePosEx(sid, POS_MIN, args.speed, args.acc, args.torque)
                while True:
                    moving, comm_result, _ = packet_handler.ReadMoving(sid)
                    if comm_result == COMM_SUCCESS and moving == 0:
                        break
                    time.sleep(0.1)
                pos, _, _, _ = packet_handler.ReadPosSpeed(sid)
                if comm_result == COMM_SUCCESS:
                    print(f"    ✅ [ID:{sid:03d}] 到达: {pos}")

            time.sleep(args.step_delay)

    except KeyboardInterrupt:
        print("\n⚠️  用户中断")


def mode_sync_read_test(packet_handler, servo_ids, args):
    """
    同步读取测试:
    持续同步读取所有舵机状态并显示
    """
    print("\n" + "=" * 50)
    print("  模式: 同步读取测试")
    print("=" * 50)
    print(f"  舵机: {servo_ids}")

    print("\n开始同步读取 (Ctrl+C 停止)...")
    print(f"{'Time':>8s} |", end="")
    for sid in servo_ids:
        print(f" ID{sid:03d}(deg) |", end="")
    print("")

    start_time = time.time()
    try:
        while True:
            t = time.time() - start_time
            positions = sync_read_positions(packet_handler, servo_ids)

            print(f"  {t:6.2f}s |", end="")
            for sid in servo_ids:
                pos = positions.get(sid)
                if pos is not None:
                    angle = pos * 360.0 / 4095.0
                    print(f" {angle:>10.1f} |", end="")
                else:
                    print(f" {'?':>10} |", end="")
            print("")

            time.sleep(args.read_interval)

    except KeyboardInterrupt:
        print("\n⚠️  用户中断")


# ==================== 主函数 ====================

def main():
    parser = argparse.ArgumentParser(
        description="HLS3606 多舵机同步控制",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
运动模式:
  sync_pos   - 同步位置控制 (所有舵机同时运动)
  sync_read  - 同步读取测试 (持续读取并显示)
  wave       - 波浪运动 (相位差正弦波)
  relay      - 接力运动 (依次运动)

示例:
  python hls3606_sync_multi.py --mode sync_pos
  python hls3606_sync_multi.py --mode wave --ids 1,2,3,4
  python hls3606_sync_multi.py --mode relay --ids 1,2,3
  python hls3606_sync_multi.py --mode sync_read --ids 1,2 --read-interval 0.1
        """
    )
    parser.add_argument("--port", type=str, default=SERIAL_PORT, help=f"串口设备 (默认: {SERIAL_PORT})")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE, help=f"波特率 (默认: {BAUDRATE})")
    parser.add_argument("--ids", type=str, default="7", help="舵机ID列表 (默认: 7)")
    parser.add_argument("--mode", type=str, default="sync_pos",
                        choices=["sync_pos", "sync_read", "wave", "relay"],
                        help="运动模式 (默认: sync_pos)")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help=f"运动速度 raw (默认: {DEFAULT_SPEED})")
    parser.add_argument("--acc", type=int, default=DEFAULT_ACC, help=f"加速度 raw (默认: {DEFAULT_ACC})")
    parser.add_argument("--torque", type=int, default=DEFAULT_TORQUE, help=f"扭矩限制 (默认: {DEFAULT_TORQUE})")
    parser.add_argument("--step-delay", type=float, default=2.0, help="每步间等待时间秒 (默认: 2.0)")
    parser.add_argument("--wave-period", type=float, default=4.0, help="波浪运动周期秒 (默认: 4.0)")
    parser.add_argument("--wave-dt", type=float, default=0.05, help="波浪运动控制周期秒 (默认: 0.05)")
    parser.add_argument("--read-interval", type=float, default=0.5, help="同步读取间隔秒 (默认: 0.5)")
    args = parser.parse_args()

    servo_ids = [int(x.strip()) for x in args.ids.split(",")]

    print("=" * 60)
    print(f"  HLS3606 多舵机同步控制 - {args.mode}")
    print("=" * 60)
    print(f"  舵机: {servo_ids}")
    print(f"  串口: {args.port}")

    # 初始化通信
    print(f"\n[1] 初始化通信...")
    port_handler, packet_handler = init_communication(args.port, args.baudrate)
    if port_handler is None:
        sys.exit(1)

    try:
        # 使能扭矩
        print(f"\n[2] 使能扭矩...")
        enable_torque(packet_handler, servo_ids)

        # 先移动到中间位置
        print(f"  移动到初始位置 ({POS_MID})...")
        mid_positions = [POS_MID] * len(servo_ids)
        sync_write_positions(packet_handler, servo_ids, mid_positions, 15, 10, args.torque)
        time.sleep(1.5)

        # 根据模式执行
        if args.mode == "sync_pos":
            mode_sync_position_control(packet_handler, servo_ids, args)
        elif args.mode == "sync_read":
            mode_sync_read_test(packet_handler, servo_ids, args)
        elif args.mode == "wave":
            mode_wave(packet_handler, servo_ids, args)
        elif args.mode == "relay":
            mode_relay(packet_handler, servo_ids, args)

    except KeyboardInterrupt:
        print("\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print(f"\n[3] 安全退出...")
        # 移动到中间位置
        mid_positions = [POS_MID] * len(servo_ids)
        sync_write_positions(packet_handler, servo_ids, mid_positions, 30, 20, args.torque)
        time.sleep(1.0)

        disable_torque(packet_handler, servo_ids)
        port_handler.closePort()
        print(f"🎉 程序安全退出")


if __name__ == "__main__":
    main()
