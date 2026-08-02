#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HLS3606 自由编码器模式 — 无阻力角度反馈
==========================================
原理:
  释放舵机扭矩 (TORQUE_ENABLE = 0)，电机完全断电，输出轴可被人手
  自由转动，无任何阻力。同时舵机的磁编码器仍然可以读取当前位置，
  实现类似"旋转编码器"的效果——实时显示人为转动角度曲线。

使用方法:
  python hls3606_free_encoder.py                        # 默认配置
  python hls3606_free_encoder.py --duration 60          # 运行60秒
  python hls3606_free_encoder.py --save                 # 保存数据到CSV
  python hls3606_free_encoder.py --save --save-dir ./my_data
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

# 绘图
PLOT_WINDOW = 20.0   # 时间窗口 (秒)
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

def setup_plot(servo_id):
    plt.ion()
    fig, (ax_pos, ax_speed) = plt.subplots(2, 1, figsize=(12, 6), sharex=True,
                                           gridspec_kw={"height_ratios": [3, 1]})

    # 位置图
    pos_line, = ax_pos.plot([], [], "b-", linewidth=1.5, label="Position")
    ax_pos.set_ylabel("Position (deg)")
    ax_pos.set_ylim(0, 360)
    ax_pos.grid(True, linestyle="--", alpha=0.5)
    ax_pos.legend(loc="upper right")

    # 速度图
    speed_line, = ax_speed.plot([], [], "g-", linewidth=1.2, label="Speed")
    ax_speed.set_xlabel("Time (s)")
    ax_speed.set_ylabel("Speed (rpm)")
    ax_speed.set_ylim(-80, 80)       # 手动转动通常在这个范围
    ax_speed.grid(True, linestyle="--", alpha=0.5)
    ax_speed.axhline(y=0, color="k", linewidth=0.5)
    ax_speed.legend(loc="upper right")

    # 信息文本
    info_text = ax_pos.text(0.02, 0.95, "", transform=ax_pos.transAxes,
                            fontsize=11, verticalalignment="top",
                            bbox=dict(boxstyle="round", facecolor="lightblue", alpha=0.8))

    fig.suptitle(f"HLS3606 Free Encoder — Rotate shaft freely!", fontsize=14, fontweight="bold")

    # 数据缓冲区
    maxlen = int(PLOT_WINDOW * PLOT_FPS)
    data_buf = {
        "time": deque(maxlen=maxlen),
        "pos": deque(maxlen=maxlen),
        "speed": deque(maxlen=maxlen),
    }

    return fig, ax_pos, ax_speed, pos_line, speed_line, info_text, data_buf


def render(ax_speed, pos_line, speed_line, info_text, data_buf):
    times = list(data_buf["time"])
    pos_line.set_xdata(times)
    pos_line.set_ydata(list(data_buf["pos"]))
    speed_line.set_xdata(times)
    speeds = list(data_buf["speed"])
    speed_line.set_ydata(speeds)

    # 自动调整速度轴范围，避免超出后看不到
    if speeds:
        abs_max = max(abs(s) for s in speeds if s is not None)
        margin = max(abs_max * 1.2, 20)
        ax_speed.set_ylim(-margin, margin)


# ==================== CSV 保存 ====================

class DataLogger:
    def __init__(self, filepath):
        self.fp = open(filepath, "w")
        self.fp.write("timestamp,position_raw,position_deg,speed_rpm\n")

    def log(self, t, pos_raw, pos_deg, speed_rpm):
        self.fp.write(f"{t:.4f},{pos_raw},{pos_deg:.2f},{speed_rpm:.2f}\n")

    def close(self):
        self.fp.close()
        print(f"📁 数据已保存至: {self.fp.name}")


# ==================== 主函数 ====================

def main():
    parser = argparse.ArgumentParser(
        description="HLS3606 自由编码器模式 — 无阻力角度反馈",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", type=str, default=SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=BAUDRATE)
    parser.add_argument("--id", type=int, default=SERVO_ID)
    parser.add_argument("--duration", type=float, default=0,
                        help="运行时长秒 (默认: 0=无限)")
    parser.add_argument("--save", action="store_true", help="保存数据到CSV")
    parser.add_argument("--save-dir", type=str, default="./data_logs")
    args = parser.parse_args()

    servo_id = args.id

    print("=" * 55)
    print("  HLS3606 自由编码器模式")
    print("=" * 55)
    print(f"  舵机ID: {servo_id}")
    print(f"  💡 舵机扭矩已释放, 可自由转动输出轴")
    print(f"  📊 实时显示位置和速度曲线")

    # 初始化通信
    print(f"\n[1] 初始化通信...")
    port_handler, packet_handler = init_communication(args.port, args.baudrate)
    if port_handler is None:
        sys.exit(1)

    # 初始化绘图
    fig, ax_pos, ax_speed, pos_line, speed_line, info_text, data_buf = setup_plot(servo_id)

    # 数据保存
    logger = None
    if args.save:
        os.makedirs(args.save_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        logger = DataLogger(os.path.join(args.save_dir, f"free_encoder_{ts}.csv"))

    try:
        # 释放扭矩 — 关键步骤
        print(f"\n[2] 释放舵机扭矩...")
        packet_handler.write1ByteTxRx(servo_id, HLS_TORQUE_ENABLE, 0)
        print(f"  ✅ 扭矩已释放 — 输出轴可自由转动")

        # 先读一次初始位置
        init_pos, comm_result, _ = packet_handler.ReadPos(servo_id)
        if comm_result == COMM_SUCCESS:
            print(f"  初始位置: {init_pos} ({init_pos * 360.0 / 4095.0:.1f}°)")

        print(f"\n[3] 开始编码器监控... (Ctrl+C 停止)")
        print(f"  🖐️  试着转动输出轴!")

        start_time = time.time()
        last_plot_time = 0
        plot_interval = 1.0 / PLOT_FPS
        last_status_time = 0
        last_pos = None
        last_pos_time = None
        frame_count = 0

        while True:
            current_time = time.time() - start_time

            # 检查运行时长
            if args.duration > 0 and current_time > args.duration:
                print(f"\n⏰ 运行时长 ({args.duration}s) 已到")
                break

            # 读取位置和速度
            pos, speed, comm_result, error = packet_handler.ReadPosSpeed(servo_id)
            if comm_result != COMM_SUCCESS:
                time.sleep(0.01)
                continue

            pos_deg = pos * 360.0 / 4095.0
            speed_rpm = speed * 0.732

            # 计算转动速度（基于位置差分，更精确）
            if last_pos is not None and last_pos_time is not None:
                dt = current_time - last_pos_time
                if dt > 0.001:
                    # 处理圈数回绕
                    raw_diff = pos - last_pos
                    if raw_diff > 2048:
                        raw_diff -= 4096
                    elif raw_diff < -2048:
                        raw_diff += 4096
                    diff_rpm = (raw_diff * 360.0 / 4095.0) / (dt / 60.0)  # deg/s → rpm
                else:
                    diff_rpm = 0
            else:
                diff_rpm = 0

            last_pos = pos
            last_pos_time = current_time

            # 记录数据 (用位置差分速度, 比寄存器速度更平滑)
            data_buf["time"].append(current_time)
            data_buf["pos"].append(pos_deg)
            data_buf["speed"].append(diff_rpm)

            if logger:
                logger.log(current_time, pos, pos_deg, diff_rpm)

            # 渲染图表
            if current_time - last_plot_time >= plot_interval:
                render(ax_speed, pos_line, speed_line, info_text, data_buf)

                x_min = max(0, current_time - PLOT_WINDOW)
                x_max = max(PLOT_WINDOW, current_time + 0.5)
                ax_pos.set_xlim(x_min, x_max)

                info_text.set_text(f"Pos: {pos} raw / {pos_deg:.1f}°  |  "
                                   f"Speed: {diff_rpm:.1f} rpm  |  "
                                   f"⏱ {current_time:.1f}s")

                fig.canvas.draw()
                fig.canvas.flush_events()
                last_plot_time = current_time

            # 终端状态
            frame_count += 1
            if current_time - last_status_time >= 0.3:
                # 可视化转动方向
                if abs(diff_rpm) > 2:
                    direction = "↻" if diff_rpm > 0 else "↺"
                    bar_len = min(int(abs(diff_rpm)), 30)
                    bar = direction + "█" * bar_len
                else:
                    bar = "●"
                print(f"\r  t={current_time:5.1f}s | pos={pos_deg:6.1f}° "
                      f"speed={diff_rpm:+6.1f}rpm | {bar}   ",
                      end="", flush=True)
                last_status_time = current_time

            # 检查窗口
            if not plt.fignum_exists(fig.number):
                print("\n🛑 绘图窗口已关闭")
                break

            time.sleep(0.015)

    except KeyboardInterrupt:
        print("\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print(f"\n[4] 关闭 (共 {frame_count} 帧)...")
        if logger:
            logger.close()
        port_handler.closePort()
        plt.ioff()
        plt.close("all")
        print("🎉 程序安全退出")


if __name__ == "__main__":
    main()
