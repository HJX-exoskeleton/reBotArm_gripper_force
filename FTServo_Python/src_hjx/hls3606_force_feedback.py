#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HLS3606 力反馈/柔顺控制测试脚本
==================================
原理:
  WritePosEx(id, pos, speed, acc, torque) 的 torque 参数设置的是
  扭矩上限（电流限制）。位置环 PID 仍然在保持目标位置，但较低的 torque
  会限制最大输出力矩，使人手可以拧开输出轴；松手后 PID 自动回正。

效果:
  - 高 torque (500~1000): 刚性位置保持，难以拧动
  - 中 torque (200~500): 有弹性的弹簧感
  - 低 torque (50~200): 柔软，极易拨动，松手缓慢回正

使用方法:
  python hls3606_force_feedback.py                          # 默认配置
  python hls3606_force_feedback.py --target 2048 --torque 150  # 自定义目标和扭矩
  python hls3606_force_feedback.py --interactive            # 交互模式: 键盘实时调扭矩

交互模式按键:
  q / Esc  - 退出
  ↑ / w    - 增加扭矩 (+50)
  ↓ / s    - 减小扭矩 (-50)
  ← / a    - 目标位置左移 (-100)
  → / d    - 目标位置右移 (+100)
  r        - 复位到初始位置
  Space    - 切换扭矩使能
"""

import sys
import os
import time
import argparse
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from collections import deque

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scservo_sdk import *

# ==================== 默认配置 ====================
SERIAL_PORT = "/dev/ttyACM1"
BAUDRATE = 1000000
SERVO_ID = 7

# 运动参数
DEFAULT_TARGET = 2048       # 默认目标位置 (~180°)
DEFAULT_TORQUE = 60        # 扭矩限制 (值越小越软)
DEFAULT_SPEED = 1000          # 回正速度
DEFAULT_ACC = 20            # 回正加速度

# 位置范围
POS_MIN = 512    # ~45°
POS_MID = 2048   # ~180°
POS_MAX = 3584   # ~315°

# 扭矩范围 (raw, 0-1023)
TORQUE_MIN = 20    # 极软, 几乎无力
TORQUE_MAX = 1000  # 最大限制
TORQUE_STEP = 50

# 绘图
PLOT_WINDOW = 15.0
PLOT_FPS = 30


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


# ==================== 绘图 ====================

def setup_plot(servo_id, target_pos, torque):
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 5))

    # 目标位置线
    target_deg = target_pos * 360.0 / 4095.0
    ax.axhline(y=target_deg, color="red", linestyle="--", linewidth=1.5, label=f"Target ({target_deg:.0f}°)")

    # 实际位置线
    actual_line, = ax.plot([], [], "b-", linewidth=1.5, label="Actual Position")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (deg)")
    ax.set_ylim(0, 360)
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend(loc="upper right")

    # 扭矩信息文本
    torque_text = ax.text(0.02, 0.95, f"Torque Limit: {torque}", transform=ax.transAxes,
                          fontsize=11, verticalalignment="top",
                          bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))

    title = fig.suptitle("HLS3606 Force Feedback — Twist the shaft!", fontsize=14, fontweight="bold")

    time_buf = deque(maxlen=int(PLOT_WINDOW * PLOT_FPS))
    pos_buf = deque(maxlen=int(PLOT_WINDOW * PLOT_FPS))

    return fig, ax, actual_line, torque_text, title, time_buf, pos_buf


def update_plot(actual_line, torque_text, title, time_buf, pos_buf,
                current_time, actual_pos, target_pos, torque):
    pos_deg = actual_pos * 360.0 / 4095.0
    time_buf.append(current_time)
    pos_buf.append(pos_deg)

    actual_line.set_xdata(list(time_buf))
    actual_line.set_ydata(list(pos_buf))

    torque_text.set_text(f"Torque Limit: {torque}  |  "
                         f"Target: {target_pos} ({target_pos * 360 / 4095:.0f}°)  |  "
                         f"Actual: {actual_pos} ({pos_deg:.0f}°)")
    deviation = abs(pos_deg - target_pos * 360.0 / 4095.0)
    if deviation > 3:
        title.set_text(f"HLS3606 Force Feedback — ⚡ Deflected {deviation:.0f}°!")
        title.set_color("orange")
    else:
        title.set_text("HLS3606 Force Feedback — ✓ On target")
        title.set_color("black")


# ==================== 交互控制 ====================

def on_key_press(event, state):
    """键盘事件处理"""
    key = event.key.lower()

    if key in ("q", "escape"):
        state["running"] = False
        print("\n🛑 退出中...")

    elif key in ("up", "w", "=", "+"):
        state["torque"] = min(TORQUE_MAX, state["torque"] + TORQUE_STEP)
        state["torque_changed"] = True
        print(f"\r  🔼 扭矩 + → {state['torque']}", end="")

    elif key in ("down", "s", "-", "_"):
        state["torque"] = max(TORQUE_MIN, state["torque"] - TORQUE_STEP)
        state["torque_changed"] = True
        print(f"\r  🔽 扭矩 - → {state['torque']}", end="")

    elif key in ("left", "a"):
        state["target"] = max(POS_MIN, state["target"] - 100)
        state["target_changed"] = True
        print(f"\r  ◀️  目标 ← {state['target']} ({state['target'] * 360 / 4095:.0f}°)", end="")

    elif key in ("right", "d"):
        state["target"] = min(POS_MAX, state["target"] + 100)
        state["target_changed"] = True
        print(f"\r  ▶️  目标 → {state['target']} ({state['target'] * 360 / 4095:.0f}°)", end="")

    elif key == "r":
        state["target"] = DEFAULT_TARGET
        state["torque"] = DEFAULT_TORQUE
        state["target_changed"] = True
        state["torque_changed"] = True
        print(f"\r  🔄 复位到默认: target={DEFAULT_TARGET}, torque={DEFAULT_TORQUE}", end="")

    elif key == " ":
        state["torque_enabled"] = not state["torque_enabled"]
        state["torque_changed"] = True
        status = "启" if state["torque_enabled"] else "释放"
        print(f"\r  🔧 扭矩{status}  → torque={state['torque'] if state['torque_enabled'] else 0}", end="")


# ==================== 主函数 ====================

def main():
    parser = argparse.ArgumentParser(
        description="HLS3606 力反馈/柔顺控制",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python hls3606_force_feedback.py
  python hls3606_force_feedback.py --target 2048 --torque 100
  python hls3606_force_feedback.py --interactive
  python hls3606_force_feedback.py --target 1024 --torque 80 --duration 120
        """
    )
    parser.add_argument("--port", type=str, default=SERIAL_PORT, help=f"串口设备 (默认: {SERIAL_PORT})")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE, help=f"波特率 (默认: {BAUDRATE})")
    parser.add_argument("--id", type=int, default=SERVO_ID, help=f"舵机ID (默认: {SERVO_ID})")
    parser.add_argument("--target", type=int, default=DEFAULT_TARGET, help=f"目标位置 raw (默认: {DEFAULT_TARGET})")
    parser.add_argument("--torque", type=int, default=DEFAULT_TORQUE, help=f"扭矩限制 (默认: {DEFAULT_TORQUE})")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help=f"回正速度 (默认: {DEFAULT_SPEED})")
    parser.add_argument("--acc", type=int, default=DEFAULT_ACC, help=f"回正加速度 (默认: {DEFAULT_ACC})")
    parser.add_argument("--duration", type=float, default=0, help="运行时长秒 (默认: 120, 0=无限)")
    parser.add_argument("--interactive", action="store_true", help="启用键盘交互模式")
    parser.add_argument("--no-torque-on-start", action="store_true", help="启动时不使能扭矩 (先自由转动)")
    args = parser.parse_args()

    servo_id = args.id
    target_pos = args.target
    torque_limit = args.torque

    print("=" * 60)
    print("  HLS3606 力反馈 / 柔顺控制")
    print("=" * 60)
    print(f"  舵机ID: {servo_id}")
    print(f"  目标位置: {target_pos} ({target_pos * 360.0 / 4095.0:.0f}°)")
    print(f"  扭矩限制: {torque_limit}")
    print(f"  交互模式: {'✅ 启用' if args.interactive else '❌ 关闭'}")
    print()
    print("  💡 说明:")
    print(f"     - 舵机会保持在 {target_pos * 360.0 / 4095.0:.0f}° 位置")
    print(f"     - 扭矩限制为 {torque_limit}, 低于此值的力矩会顺从")
    print(f"     - 试着用手拧输出轴, 观察曲线变化")
    print(f"     - 松手后舵机会自动回正")
    if args.interactive:
        print()
        print("  ⌨️  交互按键:")
        print("     ↑/w  增加扭矩  |  ↓/s  减小扭矩")
        print("     ←/a  目标左移  |  →/d  目标右移")
        print("     Space 切换扭矩  |  r    复位")
        print("     q/Esc 退出")

    # 初始化通信
    print(f"\n[1] 初始化通信...")
    port_handler, packet_handler = init_communication(args.port, args.baudrate)
    if port_handler is None:
        sys.exit(1)

    # 初始化绘图
    fig, ax, actual_line, torque_text, title, time_buf, pos_buf = \
        setup_plot(servo_id, target_pos, torque_limit)

    # 交互状态
    state = {
        "running": True,
        "target": target_pos,
        "torque": torque_limit,
        "torque_enabled": not args.no_torque_on_start,
        "torque_changed": False,
        "target_changed": False,
    }

    if args.interactive:
        fig.canvas.mpl_connect("key_press_event", lambda e: on_key_press(e, state))

    try:
        # 使能扭矩
        if state["torque_enabled"]:
            print(f"\n[2] 使能扭矩并移动到目标位置...")
            packet_handler.write1ByteTxRx(servo_id, HLS_TORQUE_ENABLE, 1)
            time.sleep(0.1)
            packet_handler.WritePosEx(servo_id, target_pos, args.speed, args.acc, torque_limit)
            time.sleep(1.5)  # 等待到达目标
        else:
            print(f"\n[2] 扭矩未使能, 舵机可自由转动")
            packet_handler.write1ByteTxRx(servo_id, HLS_TORQUE_ENABLE, 0)

        # 更新目标线
        ax.axhline(y=target_pos * 360.0 / 4095.0, color="red", linestyle="--", linewidth=1.5)
        ax.relim()
        ax.autoscale_view()

        print(f"\n[3] 开始力反馈监控... (Ctrl+C 或 q 停止)")
        print(f"  📊 实时绘图已打开 — 试着拧输出轴!")

        start_time = time.time()
        last_plot_time = 0
        plot_interval = 1.0 / PLOT_FPS
        last_status_time = 0

        while state["running"]:
            current_time = time.time() - start_time

            # 检查运行时长
            if args.duration > 0 and current_time > args.duration:
                print(f"\n⏰ 运行时长 ({args.duration}s) 已到")
                break

            # 处理状态变更
            if state["torque_changed"] or state["target_changed"]:
                torque_val = state["torque"] if state["torque_enabled"] else 0

                if state["torque_enabled"]:
                    packet_handler.write1ByteTxRx(servo_id, HLS_TORQUE_ENABLE, 1)
                    time.sleep(0.02)
                    packet_handler.WritePosEx(servo_id, state["target"],
                                              args.speed, args.acc, torque_val)
                else:
                    packet_handler.write1ByteTxRx(servo_id, HLS_TORQUE_ENABLE, 0)

                if state["target_changed"]:
                    # 更新目标线
                    ax.lines[0].set_ydata([target_pos * 360.0 / 4095.0, target_pos * 360.0 / 4095.0])

                state["torque_changed"] = False
                state["target_changed"] = False

            # 持续发送目标位置指令 (保持位置环激活, 确保松手回正)
            if state["torque_enabled"]:
                torque_val = state["torque"]
                packet_handler.WritePosEx(servo_id, state["target"],
                                          args.speed, args.acc, torque_val)

            # 读取实际位置
            pos, comm_result, error = packet_handler.ReadPos(servo_id)
            if comm_result != COMM_SUCCESS:
                time.sleep(0.01)
                continue

            # 更新曲线
            if current_time - last_plot_time >= plot_interval:
                update_plot(actual_line, torque_text, title, time_buf, pos_buf,
                            current_time, pos, state["target"],
                            state["torque"] if state["torque_enabled"] else 0)

                ax.set_xlim(max(0, current_time - PLOT_WINDOW),
                            max(PLOT_WINDOW, current_time + 0.5))
                fig.canvas.draw()
                fig.canvas.flush_events()
                last_plot_time = current_time

            # 终端状态输出
            if current_time - last_status_time >= 0.5:
                pos_deg = pos * 360.0 / 4095.0
                tgt_deg = state["target"] * 360.0 / 4095.0
                dev = pos_deg - tgt_deg
                torque_display = state["torque"] if state["torque_enabled"] else 0
                bar = "█" * int(abs(dev)) if abs(dev) < 30 else "█" * 30
                direction = "→" if dev > 1 else ("←" if dev < -1 else "●")
                print(f"\r  t={current_time:5.1f}s | target={tgt_deg:6.1f}° "
                      f"actual={pos_deg:6.1f}° dev={dev:+6.1f}° {direction} "
                      f"torque={torque_display:4d} | {bar}", end="")
                last_status_time = current_time

            time.sleep(0.02)  # ~50Hz

            # 检查窗口是否被关闭
            if not plt.fignum_exists(fig.number):
                print("\n🛑 绘图窗口已关闭")
                break

    except KeyboardInterrupt:
        print("\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[4] 安全退出...")
        # 先设高扭矩回目标位置, 防止突然释放
        if state["torque_enabled"]:
            packet_handler.WritePosEx(servo_id, state["target"], 30, 20, 500)
            time.sleep(0.8)

        packet_handler.write1ByteTxRx(servo_id, HLS_TORQUE_ENABLE, 0)
        port_handler.closePort()
        plt.ioff()
        plt.close("all")
        print("🎉 程序安全退出")


if __name__ == "__main__":
    main()
