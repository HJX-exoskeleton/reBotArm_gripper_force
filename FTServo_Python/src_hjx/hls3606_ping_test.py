#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HLS3606 舵机连接与 Ping 测试脚本
==================================
功能:
  1. 串口连接测试
  2. Ping 舵机，获取型号信息
  3. 读取当前位置、速度、电压、温度等状态
  4. 扭矩开关测试

使用方法:
  python hls3606_ping_test.py

"""

import sys
import os
import time

# 添加 SDK 路径
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scservo_sdk import *

# ==================== 配置参数 ====================
# 串口配置
SERIAL_PORT = "/dev/ttyACM1"   # 根据实际连接修改
BAUDRATE = 1000000             # HLS 默认波特率 1M

# 舵机 ID 列表 (可修改为实际使用的 ID)
SERVO_IDS = [7]


def init_communication(port_name, baudrate):
    """初始化串口通信"""
    port_handler = PortHandler(port_name)
    packet_handler = hls(port_handler)

    # 打开串口
    if not port_handler.openPort():
        print(f"❌ 串口 {port_name} 打开失败!")
        return None, None

    print(f"✅ 串口 {port_name} 打开成功")

    # 设置波特率
    if not port_handler.setBaudRate(baudrate):
        print(f"❌ 波特率 {baudrate} 设置失败!")
        port_handler.closePort()
        return None, None

    print(f"✅ 波特率设置为 {baudrate}")
    return port_handler, packet_handler


def ping_servo(packet_handler, servo_id):
    """Ping 舵机，获取型号"""
    model_number, comm_result, error = packet_handler.ping(servo_id)
    if comm_result != COMM_SUCCESS:
        print(f"  [ID:{servo_id:03d}] ❌ Ping 失败: {packet_handler.getTxRxResult(comm_result)}")
        return False
    if error != 0:
        print(f"  [ID:{servo_id:03d}] ⚠️  错误: {packet_handler.getRxPacketError(error)}")
        return False

    print(f"  [ID:{servo_id:03d}] ✅ Ping 成功, 型号: {model_number}")
    return True


def read_servo_status(packet_handler, servo_id):
    """读取舵机完整状态"""
    print(f"\n📊 [ID:{servo_id:03d}] 舵机状态:")

    # 读取当前位置和速度
    pos, speed, comm_result, error = packet_handler.ReadPosSpeed(servo_id)
    if comm_result == COMM_SUCCESS:
        angle_deg = pos * 360.0 / 4095.0
        speed_rpm = speed * 0.732
        print(f"  位置: {pos} (raw) / {angle_deg:.2f}°")
        print(f"  速度: {speed} (raw) / {speed_rpm:.2f} rpm")
    else:
        print(f"  ⚠️  读取位置速度失败: {packet_handler.getTxRxResult(comm_result)}")

    # 读取运动状态
    moving, comm_result, error = packet_handler.ReadMoving(servo_id)
    if comm_result == COMM_SUCCESS:
        status_text = "运动中" if moving else "静止"
        print(f"  运动状态: {status_text}")
    else:
        print(f"  ⚠️  读取运动状态失败: {packet_handler.getTxRxResult(comm_result)}")

    # 读取电压 (地址 62)
    voltage_val, comm_result, error = packet_handler.read1ByteTxRx(servo_id, HLS_PRESENT_VOLTAGE)
    if comm_result == COMM_SUCCESS:
        voltage = voltage_val * 0.1  # 0.1V 单位
        print(f"  电压: {voltage:.1f}V")
    else:
        print(f"  ⚠️  读取电压失败: {packet_handler.getTxRxResult(comm_result)}")

    # 读取温度 (地址 63)
    temp_val, comm_result, error = packet_handler.read1ByteTxRx(servo_id, HLS_PRESENT_TEMPERATURE)
    if comm_result == COMM_SUCCESS:
        print(f"  温度: {temp_val}°C")
    else:
        print(f"  ⚠️  读取温度失败: {packet_handler.getTxRxResult(comm_result)}")

    # 读取负载 (地址 60-61)
    load_val, comm_result, error = packet_handler.read2ByteTxRx(servo_id, HLS_PRESENT_LOAD_L)
    if comm_result == COMM_SUCCESS:
        load_percent = load_val * 100.0 / 1023.0
        print(f"  负载: {load_val} (raw) / {load_percent:.1f}%")
    else:
        print(f"  ⚠️  读取负载失败: {packet_handler.getTxRxResult(comm_result)}")


def test_torque(packet_handler, servo_id):
    """测试扭矩开关"""
    print(f"\n🔧 [ID:{servo_id:03d}] 扭矩开关测试:")

    # 开启扭矩
    comm_result, error = packet_handler.write1ByteTxRx(servo_id, HLS_TORQUE_ENABLE, 1)
    if comm_result == COMM_SUCCESS:
        print(f"  ✅ 扭矩已开启")
    else:
        print(f"  ❌ 扭矩开启失败: {packet_handler.getTxRxResult(comm_result)}")
        return

    time.sleep(0.5)

    # 读取扭矩状态确认
    torque_state, comm_result, error = packet_handler.read1ByteTxRx(servo_id, HLS_TORQUE_ENABLE)
    if comm_result == COMM_SUCCESS:
        state_map = {0: "释放", 1: "使能", 2: "刹车"}
        state_text = state_map.get(torque_state, f"未知({torque_state})")
        print(f"  当前扭矩状态: {state_text}")

    # 关闭扭矩
    comm_result, error = packet_handler.write1ByteTxRx(servo_id, HLS_TORQUE_ENABLE, 0)
    if comm_result == COMM_SUCCESS:
        print(f"  ✅ 扭矩已释放")


def main():
    print("=" * 50)
    print("  HLS3606 舵机连接与 Ping 测试")
    print("=" * 50)

    # 1. 初始化通信
    print("\n[1] 初始化串口通信...")
    port_handler, packet_handler = init_communication(SERIAL_PORT, BAUDRATE)
    if port_handler is None:
        print("\n💡 请检查:")
        print("   - 舵机是否正确连接到 USB 端口")
        print("   - 串口设备名是否正确 (当前: {})".format(SERIAL_PORT))
        print("   - 是否有其他程序占用了串口")
        print("   - 用户是否有串口读写权限 (sudo usermod -a -G dialout $USER)")
        sys.exit(1)

    try:
        # 2. Ping 测试
        print("\n[2] Ping 舵机...")
        all_ok = True
        for sid in SERVO_IDS:
            if not ping_servo(packet_handler, sid):
                all_ok = False
        if not all_ok:
            print("\n⚠️  部分舵机 Ping 失败，请检查舵机 ID 和连接")

        # 3. 读取状态
        print("\n[3] 读取舵机状态...")
        for sid in SERVO_IDS:
            read_servo_status(packet_handler, sid)

        # 4. 扭矩测试
        print("\n[4] 扭矩开关测试...")
        for sid in SERVO_IDS:
            test_torque(packet_handler, sid)

        print("\n" + "=" * 50)
        print("  ✅ 测试完成!")
        print("=" * 50)

    except KeyboardInterrupt:
        print("\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 释放所有舵机扭矩
        for sid in SERVO_IDS:
            packet_handler.write1ByteTxRx(sid, HLS_TORQUE_ENABLE, 0)
        port_handler.closePort()
        print("🔌 串口已关闭")


if __name__ == "__main__":
    main()
