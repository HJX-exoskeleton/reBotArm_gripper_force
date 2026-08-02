#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HLS3606 舵机 ID 与中位校准工具
================================
功能:
  1. 读取当前舵机 ID 和零点偏移
  2. 修改舵机 ID
  3. 中位校准: 将当前位置设为指定值 (如 2048 = 180°)
  4. 清除零点偏移 (恢复到出厂)

⚠️ 注意:
  - 修改 ID 时总线上只能接一个舵机, 否则会同时改多个
  - EEPROM 写入有寿命 (~10万次), 不要频繁写入
  - 修改后需要重新上电或复位才能完全生效

使用方法:
  python hls3606_config.py --read                           # 读取当前配置
  python hls3606_config.py --set-id 2                       # 将舵机ID改为2 (旧ID=1)
  python hls3606_config.py --set-id 3 --old-id 1            # 指定旧ID
  python hls3606_config.py --calibrate 2048                 # 校准: 将当前位置设为180°(2048)
  python hls3606_config.py --calibrate 2048 --id 1          # 校准指定ID的舵机
  python hls3606_config.py --reset-offset                   # 清除零点偏移
"""

import sys
import os
import time
import argparse

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scservo_sdk import *

# ==================== 默认配置 ====================
SERIAL_PORT = "/dev/ttyACM1"
BAUDRATE = 1000000
DEFAULT_ID = 1


def init_communication(port_name, baudrate):
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


def read_config(packet_handler, servo_id):
    """读取舵机当前配置"""
    print(f"\n📋 [ID:{servo_id:03d}] 当前配置:")
    print("-" * 40)

    # Ping 获取型号
    model, comm_result, error = packet_handler.ping(servo_id)
    if comm_result == COMM_SUCCESS:
        print(f"  型号: {model}")

    # 读取 ID (地址 5)
    current_id, comm_result, error = packet_handler.read1ByteTxRx(servo_id, HLS_ID)
    if comm_result == COMM_SUCCESS:
        print(f"  ID: {current_id}")
    else:
        print(f"  ID: 读取失败 ({packet_handler.getTxRxResult(comm_result)})")

    # 读取波特率 (地址 6)
    baud_code, comm_result, error = packet_handler.read1ByteTxRx(servo_id, HLS_BAUD_RATE)
    if comm_result == COMM_SUCCESS:
        baud_map = {0: "1M", 1: "0.5M", 2: "250K", 3: "128K",
                    4: "115200", 5: "76800", 6: "57600", 7: "38400"}
        baud_str = baud_map.get(baud_code, f"未知({baud_code})")
        print(f"  波特率: {baud_str}")

    # 读取零点偏移 (地址 31-32)
    offset, comm_result, error = packet_handler.read2ByteTxRx(servo_id, HLS_OFS_L)
    if comm_result == COMM_SUCCESS:
        print(f"  零点偏移: {offset}")
    else:
        print(f"  零点偏移: 读取失败 ({packet_handler.getTxRxResult(comm_result)})")

    # 读取角度限制 (地址 9-12)
    min_angle, comm_result, error = packet_handler.read2ByteTxRx(servo_id, HLS_MIN_ANGLE_LIMIT_L)
    if comm_result == COMM_SUCCESS:
        print(f"  最小角度: {min_angle} ({min_angle * 360.0 / 4095.0:.1f}°)")
    max_angle, comm_result, error = packet_handler.read2ByteTxRx(servo_id, HLS_MAX_ANGLE_LIMIT_L)
    if comm_result == COMM_SUCCESS:
        print(f"  最大角度: {max_angle} ({max_angle * 360.0 / 4095.0:.1f}°)")

    # 读取模式
    mode, comm_result, error = packet_handler.read1ByteTxRx(servo_id, HLS_MODE)
    if comm_result == COMM_SUCCESS:
        mode_map = {0: "位置模式", 1: "轮式模式"}
        print(f"  模式: {mode_map.get(mode, f'未知({mode})')}")

    # 读取当前位置
    pos, comm_result, error = packet_handler.ReadPos(servo_id)
    if comm_result == COMM_SUCCESS:
        print(f"  当前位置: {pos} ({pos * 360.0 / 4095.0:.1f}°)")
    else:
        print(f"  当前位置: 读取失败 ({packet_handler.getTxRxResult(comm_result)})")

    print("-" * 40)


def set_servo_id(packet_handler, old_id, new_id):
    """修改舵机 ID

    HLS_ID 寄存器在地址 5 (EEPROM), 写入前需先解锁 EEPROM.
    写入新 ID 后, 舵机会立即响应新 ID.
    """
    if new_id < 0 or new_id > 252:
        print(f"❌ 无效的 ID: {new_id}, 范围 0-252")
        return False

    print(f"\n🔧 修改舵机 ID: {old_id} → {new_id}")
    print(f"  ⚠️  确认总线上只有一个舵机!")

    # 1. 解锁 EEPROM
    comm_result, error = packet_handler.unLockEprom(old_id)
    if comm_result != COMM_SUCCESS:
        print(f"  ❌ EEPROM 解锁失败: {packet_handler.getTxRxResult(comm_result)}")
        return False
    print(f"  ✅ EEPROM 已解锁")
    time.sleep(0.05)

    # 2. 写入新 ID
    comm_result, error = packet_handler.write1ByteTxRx(old_id, HLS_ID, new_id)
    if comm_result != COMM_SUCCESS:
        print(f"  ❌ ID 写入失败: {packet_handler.getTxRxResult(comm_result)}")
        packet_handler.LockEprom(old_id)
        return False
    print(f"  ✅ ID 已写入: {new_id}")
    time.sleep(0.05)

    # 3. 锁定 EEPROM
    comm_result, error = packet_handler.LockEprom(new_id)
    if comm_result != COMM_SUCCESS:
        print(f"  ⚠️  EEPROM 锁定失败: {packet_handler.getTxRxResult(comm_result)}")
    else:
        print(f"  ✅ EEPROM 已锁定")

    # 4. 验证
    time.sleep(0.1)
    model, comm_result, error = packet_handler.ping(new_id)
    if comm_result == COMM_SUCCESS:
        print(f"  ✅ 新 ID [{new_id}] 验证成功, 型号: {model}")
        return True
    else:
        print(f"  ⚠️  新 ID [{new_id}] 验证失败, 可能需要重新上电")
        return False


def calibrate_mid_position(packet_handler, servo_id, target_pos):
    """中位校准: 将舵机当前位置设为指定的零点位置

    使用 reOfsCal 方法, 内部将当前位置写入偏移寄存器,
    使得当前位置的逻辑值 = target_pos.

    例如: 舵机物理当前位置在 1000, 执行 calibrate(2048) 后,
          物理1000 的位置会被读作 2048.
    """
    print(f"\n🎯 中位校准 [ID:{servo_id:03d}]")

    # 先读取校准前的位置
    old_pos, comm_result, _ = packet_handler.ReadPos(servo_id)
    if comm_result == COMM_SUCCESS:
        print(f"  校准前位置: {old_pos} ({old_pos * 360.0 / 4095.0:.1f}°)")
    print(f"  将当前位置设置为: {target_pos} ({target_pos * 360.0 / 4095.0:.1f}°)")

    # 执行校准
    comm_result, error = packet_handler.reOfsCal(servo_id, target_pos)
    if comm_result != COMM_SUCCESS:
        print(f"  ❌ 校准失败: {packet_handler.getTxRxResult(comm_result)}")
        return False
    if error != 0:
        print(f"  ⚠️  校准错误: {packet_handler.getRxPacketError(error)}")

    print(f"  ✅ 校准完成!")
    time.sleep(0.5)

    # 验证
    new_pos, comm_result, _ = packet_handler.ReadPos(servo_id)
    if comm_result == COMM_SUCCESS:
        new_angle = new_pos * 360.0 / 4095.0
        print(f"  校准后位置: {new_pos} ({new_angle:.1f}°)")
        error_deg = abs(new_pos - target_pos) * 360.0 / 4095.0
        if error_deg < 1:
            print(f"  ✅ 验证通过 (误差 {error_deg:.2f}°)")
        else:
            print(f"  ⚠️  误差较大: {error_deg:.2f}°")

    return True


def reset_offset(packet_handler, servo_id):
    """清除零点偏移, 恢复出厂设定"""
    print(f"\n🔄 清除零点偏移 [ID:{servo_id:03d}]")

    # 先将偏移设为 0
    offset_value = 0

    # 解锁 EEPROM
    comm_result, error = packet_handler.unLockEprom(servo_id)
    if comm_result != COMM_SUCCESS:
        print(f"  ❌ EEPROM 解锁失败: {packet_handler.getTxRxResult(comm_result)}")
        return False
    print(f"  ✅ EEPROM 已解锁")
    time.sleep(0.05)

    # 写入偏移 0
    comm_result, error = packet_handler.write2ByteTxRx(servo_id, HLS_OFS_L, offset_value)
    if comm_result != COMM_SUCCESS:
        print(f"  ❌ 偏移清零失败: {packet_handler.getTxRxResult(comm_result)}")
        packet_handler.LockEprom(servo_id)
        return False
    print(f"  ✅ 偏移已清零")
    time.sleep(0.05)

    # 锁定 EEPROM
    packet_handler.LockEprom(servo_id)
    print(f"  ✅ EEPROM 已锁定")

    # 验证
    time.sleep(0.3)
    offset, comm_result, _ = packet_handler.read2ByteTxRx(servo_id, HLS_OFS_L)
    if comm_result == COMM_SUCCESS:
        print(f"  当前偏移值: {offset}")
        if offset == 0:
            print(f"  ✅ 清零验证通过")
        else:
            print(f"  ⚠️  偏移未完全清零, 当前值: {offset}")

    return True


def main():
    parser = argparse.ArgumentParser(
        description="HLS3606 舵机 ID 与中位校准工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python hls3606_config.py --read                        # 读取当前配置
  python hls3606_config.py --set-id 2                    # 将ID改为2
  python hls3606_config.py --calibrate 2048              # 中位校准为180°
  python hls3606_config.py --reset-offset                # 清除零点偏移
        """
    )
    parser.add_argument("--port", type=str, default=SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=BAUDRATE)
    parser.add_argument("--id", type=int, default=DEFAULT_ID, help="舵机ID (默认: 1)")
    parser.add_argument("--read", action="store_true", help="读取当前配置")
    parser.add_argument("--set-id", type=int, metavar="NEW_ID", help="修改舵机ID为目标值")
    parser.add_argument("--old-id", type=int, default=None, help="修改ID时的旧ID (默认用 --id)")
    parser.add_argument("--calibrate", type=int, metavar="POS", help="中位校准: 将当前位置设为指定raw值 (如2048)")
    parser.add_argument("--reset-offset", action="store_true", help="清除零点偏移(恢复出厂)")
    args = parser.parse_args()

    # 如果没有指定任何操作, 默认执行读取
    if not any([args.read, args.set_id, args.calibrate, args.reset_offset]):
        args.read = True

    servo_id = args.id

    print("=" * 45)
    print("  HLS3606 舵机配置工具")
    print("=" * 45)

    # 初始化通信
    port_handler, packet_handler = init_communication(args.port, args.baudrate)
    if port_handler is None:
        sys.exit(1)

    try:
        # 1. 先读取
        if args.read or args.set_id or args.reset_offset:
            read_config(packet_handler, servo_id)

        # 2. 修改 ID
        if args.set_id is not None:
            old = args.old_id if args.old_id is not None else servo_id
            new = args.set_id
            if new == old:
                print(f"⚠️  新旧ID相同 ({old}), 无需修改")
            else:
                set_servo_id(packet_handler, old, new)
                # 验证
                read_config(packet_handler, new)

        # 3. 中位校准
        if args.calibrate is not None:
            calibrate_mid_position(packet_handler, servo_id, args.calibrate)
            read_config(packet_handler, servo_id)

        # 4. 清除偏移
        if args.reset_offset:
            reset_offset(packet_handler, servo_id)
            read_config(packet_handler, servo_id)

        print(f"\n🎉 操作完成!")

    except Exception as e:
        print(f"\n❌ 异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        port_handler.closePort()
        print("🔌 串口已关闭")


if __name__ == "__main__":
    main()
