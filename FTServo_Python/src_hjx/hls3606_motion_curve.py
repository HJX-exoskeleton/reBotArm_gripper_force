#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HLS3606 舵机运动曲线同步绘制脚本
==================================
功能:
  1. 实时绘制舵机位置/速度曲线
  2. 支持多种运动轨迹: 正弦波、梯形、三角波、自定义序列
  3. 同步显示目标位置 vs 实际位置追踪
  4. 多舵机同步运动曲线对比
  5. 数据自动保存为 CSV 文件

使用方法:
  python hls3606_motion_curve.py                                    # 默认正弦波
  python hls3606_motion_curve.py --mode sine                        # 正弦波轨迹
  python hls3606_motion_curve.py --mode trapezoid                   # 梯形轨迹
  python hls3606_motion_curve.py --mode triangle                    # 三角波轨迹
  python hls3606_motion_curve.py --mode sweep                       # 扫频测试
  python hls3606_motion_curve.py --ids 1 --mode sine --save         # 单舵机+保存数据

轨迹模式说明:
  sine      - 正弦波: 在 min_pos~max_pos 之间做正弦往复运动
  trapezoid - 梯形波: 匀速往返, 有加速-匀速-减速阶段
  triangle  - 三角波: 恒速往返, 速度方向突变
  sweep     - 扫频测试: 频率逐渐增加的正弦波, 测试舵机响应带宽
"""

import sys
import os
import time
import argparse
import numpy as np
import matplotlib
matplotlib.use("TkAgg")  # 使用 TkAgg 后端, 支持实时刷新
import matplotlib.pyplot as plt
from collections import deque

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scservo_sdk import *

# ==================== 配置参数 ====================
SERIAL_PORT = "/dev/ttyACM1"
BAUDRATE = 1000000

# 舵机位置范围 (raw 值, 0-4095)
POS_MIN = 1024     # ~90°
POS_MID = 2048     # ~180°
POS_MAX = 3072     # ~270°
POS_RANGE = POS_MAX - POS_MIN  # 运动范围

# 绘图配置
PLOT_WINDOW = 10.0  # 时间窗口 (秒)
PLOT_FPS = 30       # 绘图刷新率

# 运动参数
DEFAULT_SPEED = 60     # raw, ~43.9 rpm
DEFAULT_ACC = 30       # raw, ~261 deg/s²
DEFAULT_TORQUE = 500


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
    """使能扭矩"""
    for sid in servo_ids:
        packet_handler.write1ByteTxRx(sid, HLS_TORQUE_ENABLE, 1)
    time.sleep(0.05)


def disable_torque(packet_handler, servo_ids):
    """释放扭矩"""
    for sid in servo_ids:
        packet_handler.write1ByteTxRx(sid, HLS_TORQUE_ENABLE, 0)


# ==================== 轨迹生成器 ====================

class TrajectoryGenerator:
    """运动轨迹生成器基类"""

    def __init__(self, min_pos=POS_MIN, max_pos=POS_MAX, mid_pos=POS_MID, period=4.0):
        """
        Args:
            min_pos: 最小位置 (raw)
            max_pos: 最大位置 (raw)
            mid_pos: 中间位置 (raw)
            period: 运动周期 (秒)
        """
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.mid_pos = mid_pos
        self.period = period
        self.amplitude = (max_pos - min_pos) / 2.0
        self.offset = mid_pos

    def get_target(self, t):
        """根据时间 t 获取目标位置 (子类实现)"""
        raise NotImplementedError


class SineTrajectory(TrajectoryGenerator):
    """正弦波轨迹"""

    def get_target(self, t):
        phase = 2.0 * np.pi * t / self.period
        pos = self.offset + self.amplitude * np.sin(phase - np.pi / 2)
        return int(np.clip(pos, self.min_pos, self.max_pos))


class TrapezoidTrajectory(TrajectoryGenerator):
    """梯形波轨迹: 加速-匀速-减速-停止-反向"""

    def __init__(self, *args, duty_ratio=0.3, **kwargs):
        """
        Args:
            duty_ratio: 匀速段占半周期的比例 (0~1), 越小越接近三角波
        """
        super().__init__(*args, **kwargs)
        self.duty = np.clip(duty_ratio, 0.05, 0.95)

    def get_target(self, t):
        half_period = self.period / 2.0
        phase = t % self.period

        # 加速段时间 (占半周期的 (1-duty)/2)
        ramp_time = half_period * (1 - self.duty) / 2.0
        # 匀速段时间
        const_time = half_period * self.duty

        if phase <= ramp_time:
            # 正向加速段
            frac = phase / ramp_time
            pos = self.min_pos + self.amplitude * 2 * frac
        elif phase <= ramp_time + const_time:
            # 正向匀速段
            pos = self.max_pos
        elif phase <= half_period + ramp_time:
            # 负向加速段 (减速+反向加速)
            frac = (phase - half_period) / ramp_time
            pos = self.max_pos - self.amplitude * 2 * frac
        elif phase <= half_period + ramp_time + const_time:
            # 负向匀速段
            pos = self.min_pos
        else:
            # 回到正向加速段
            frac = (phase - self.period + ramp_time) / ramp_time
            pos = self.min_pos + self.amplitude * 2 * frac

        return int(np.clip(pos, self.min_pos, self.max_pos))


class TriangleTrajectory(TrajectoryGenerator):
    """三角波轨迹 (恒速往返)"""

    def get_target(self, t):
        half_period = self.period / 2.0
        phase = t % self.period
        if phase < half_period:
            frac = phase / half_period
            pos = self.min_pos + self.amplitude * 2 * frac
        else:
            frac = (phase - half_period) / half_period
            pos = self.max_pos - self.amplitude * 2 * frac
        return int(np.clip(pos, self.min_pos, self.max_pos))


class SweepTrajectory(TrajectoryGenerator):
    """扫频轨迹: 频率逐渐增加的正弦波"""

    def __init__(self, *args, freq_start=0.1, freq_end=2.0, sweep_time=30.0, **kwargs):
        """
        Args:
            freq_start: 起始频率 (Hz)
            freq_end: 终止频率 (Hz)
            sweep_time: 扫频总时间 (秒)
        """
        super().__init__(*args, **kwargs)
        self.freq_start = freq_start
        self.freq_end = freq_end
        self.sweep_time = sweep_time

    def get_target(self, t):
        # 线性 chirp: 频率从 f0 到 f1 线性增长
        freq = self.freq_start + (self.freq_end - self.freq_start) * (t / self.sweep_time)
        phase = 2.0 * np.pi * freq * t
        pos = self.offset + self.amplitude * np.sin(phase - np.pi / 2)
        return int(np.clip(pos, self.min_pos, self.max_pos))


def create_trajectory(mode, period=4.0):
    """根据模式创建轨迹生成器"""
    if mode == "sine":
        return SineTrajectory(period=period)
    elif mode == "trapezoid":
        return TrapezoidTrajectory(period=period, duty_ratio=0.3)
    elif mode == "triangle":
        return TriangleTrajectory(period=period)
    elif mode == "sweep":
        return SweepTrajectory(sweep_time=30.0)
    else:
        raise ValueError(f"未知轨迹模式: {mode}")


# ==================== 实时绘图 ====================

def setup_plot(servo_ids, mode_name):
    """初始化实时绘图窗口"""
    plt.ion()
    n_servos = len(servo_ids)
    fig, axes = plt.subplots(n_servos, 1, figsize=(12, 3 + 2 * n_servos), sharex=True)

    # 单舵机时 axes 是标量, 包装成列表统一处理
    if n_servos == 1:
        axes = [axes]

    # 位置追踪图 (每个舵机一个子图)
    target_lines = {}
    actual_lines = {}
    for i, sid in enumerate(servo_ids):
        ax = axes[i]
        ax.set_ylabel(f"ID:{sid}\nPosition (deg)")
        ax.set_ylim(0, 360)
        ax.grid(True, linestyle="--", alpha=0.5)
        target_lines[sid], = ax.plot([], [], "r--", linewidth=1.5, label="Target")
        actual_lines[sid], = ax.plot([], [], "b-", linewidth=1.5, label="Actual")
        ax.legend(loc="upper right", fontsize=8)

    # 最后一个子图放 X 轴标签
    axes[-1].set_xlabel("Time (s)")

    fig.suptitle(f"HLS3606 Motion Curve - {mode_name.upper()} Mode", fontsize=14, fontweight="bold")
    plt.tight_layout()

    # 数据缓冲区
    data_buf = {
        "time": deque(maxlen=int(PLOT_WINDOW * PLOT_FPS)),
        "target": {sid: deque(maxlen=int(PLOT_WINDOW * PLOT_FPS)) for sid in servo_ids},
        "actual": {sid: deque(maxlen=int(PLOT_WINDOW * PLOT_FPS)) for sid in servo_ids},
    }

    return fig, axes, target_lines, actual_lines, data_buf


def record_data(data_buf, servo_ids, current_time, targets, actuals):
    """每个控制周期记录数据到缓冲区 (保证数据连续性)"""
    data_buf["time"].append(current_time)
    for sid in servo_ids:
        target_deg = targets[sid] * 360.0 / 4095.0
        actual_val = actuals.get(sid)
        if actual_val is not None:
            actual_deg = actual_val * 360.0 / 4095.0
        else:
            # 读取失败时使用上一次的值, 避免曲线断裂
            if len(data_buf["actual"][sid]) > 0:
                actual_deg = data_buf["actual"][sid][-1]
            else:
                actual_deg = target_deg
        data_buf["target"][sid].append(target_deg)
        data_buf["actual"][sid].append(actual_deg)


def render_plot(target_lines, actual_lines, data_buf, servo_ids):
    """将缓冲区数据渲染到图表 (按刷新率调用)"""
    times = list(data_buf["time"])
    for sid in servo_ids:
        target_lines[sid].set_xdata(times)
        target_lines[sid].set_ydata(list(data_buf["target"][sid]))
        actual_lines[sid].set_xdata(times)
        actual_lines[sid].set_ydata(list(data_buf["actual"][sid]))


def read_actual_positions(packet_handler, servo_ids):
    """读取实际位置"""
    positions = {}
    for sid in servo_ids:
        pos, comm_result, error = packet_handler.ReadPos(sid)
        if comm_result == COMM_SUCCESS:
            positions[sid] = pos
        else:
            positions[sid] = None
    return positions


# ==================== CSV 数据保存 ====================

class DataLogger:
    """CSV 数据记录器"""

    def __init__(self, filepath, servo_ids):
        self.filepath = filepath
        self.servo_ids = servo_ids
        self.fp = open(filepath, "w")
        # 写入表头
        header = ["timestamp"]
        for sid in servo_ids:
            header.extend([f"id{sid}_target_raw", f"id{sid}_actual_raw",
                           f"id{sid}_target_deg", f"id{sid}_actual_deg"])
        self.fp.write(",".join(header) + "\n")
        self.fp.flush()

    def log(self, timestamp, targets, actuals):
        row = [f"{timestamp:.4f}"]
        for sid in self.servo_ids:
            t_raw = targets.get(sid, 0)
            a_raw = actuals.get(sid, 0)
            t_deg = t_raw * 360.0 / 4095.0
            a_deg = a_raw * 360.0 / 4095.0
            row.extend([f"{t_raw}", f"{a_raw}", f"{t_deg:.2f}", f"{a_deg:.2f}"])
        self.fp.write(",".join(row) + "\n")

    def close(self):
        self.fp.close()
        print(f"📁 数据已保存至: {self.filepath}")


# ==================== 主函数 ====================

def main():
    parser = argparse.ArgumentParser(
        description="HLS3606 舵机运动曲线同步绘制",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
轨迹模式:
  sine      - 正弦波往复运动
  trapezoid - 梯形速度曲线
  triangle  - 三角波恒速往返
  sweep     - 扫频测试 (频率递增)

示例:
  python hls3606_motion_curve.py --mode sine
  python hls3606_motion_curve.py --mode trapezoid --period 6
  python hls3606_motion_curve.py --ids 1 --mode sweep --save
        """
    )
    parser.add_argument("--port", type=str, default=SERIAL_PORT, help=f"串口设备 (默认: {SERIAL_PORT})")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE, help=f"波特率 (默认: {BAUDRATE})")
    parser.add_argument("--ids", type=str, default="1", help="舵机ID列表 (默认: 1)")
    parser.add_argument("--mode", type=str, default="sine",
                        choices=["sine", "trapezoid", "triangle", "sweep"],
                        help="运动轨迹模式 (默认: sine)")
    parser.add_argument("--period", type=float, default=4.0, help="运动周期秒 (默认: 4.0)")
    parser.add_argument("--min-pos", type=int, default=POS_MIN, help=f"最小位置 raw (默认: {POS_MIN})")
    parser.add_argument("--max-pos", type=int, default=POS_MAX, help=f"最大位置 raw (默认: {POS_MAX})")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help=f"运动速度 raw (默认: {DEFAULT_SPEED})")
    parser.add_argument("--acc", type=int, default=DEFAULT_ACC, help=f"加速度 raw (默认: {DEFAULT_ACC})")
    parser.add_argument("--torque", type=int, default=DEFAULT_TORQUE, help=f"扭矩限制 (默认: {DEFAULT_TORQUE})")
    parser.add_argument("--duration", type=float, default=60.0, help="运行时长秒 (默认: 60, 0=无限)")
    parser.add_argument("--save", action="store_true", help="保存数据到 CSV 文件")
    parser.add_argument("--save-dir", type=str, default="./data_logs", help="数据保存目录 (默认: ./data_logs)")
    args = parser.parse_args()

    servo_ids = [int(x.strip()) for x in args.ids.split(",")]
    mode_name = args.mode

    print("=" * 60)
    print(f"  HLS3606 运动曲线同步绘制 - {mode_name.upper()} 模式")
    print("=" * 60)
    print(f"  舵机ID: {servo_ids}")
    print(f"  轨迹模式: {mode_name}")
    print(f"  运动周期: {args.period}s")
    print(f"  位置范围: {args.min_pos}-{args.max_pos} (raw)")

    # 初始化通信
    print(f"\n[1] 初始化通信...")
    port_handler, packet_handler = init_communication(args.port, args.baudrate)
    if port_handler is None:
        sys.exit(1)

    # 创建轨迹生成器
    trajectory = create_trajectory(mode_name, period=args.period)
    trajectory.min_pos = args.min_pos
    trajectory.max_pos = args.max_pos
    trajectory.mid_pos = (args.min_pos + args.max_pos) // 2
    trajectory.amplitude = (args.max_pos - args.min_pos) / 2.0
    trajectory.offset = trajectory.mid_pos

    # 数据保存
    logger = None
    if args.save:
        os.makedirs(args.save_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(args.save_dir, f"hls3606_{mode_name}_{timestamp}.csv")
        logger = DataLogger(filepath, servo_ids)

    # 初始化绘图
    fig, axes, target_lines, actual_lines, data_buf = setup_plot(servo_ids, mode_name)

    try:
        # 使能扭矩
        print(f"\n[2] 使能舵机扭矩...")
        enable_torque(packet_handler, servo_ids)

        # 先缓慢移动到中间位置
        print(f"  移动到中间位置 ({trajectory.mid_pos})...")
        for sid in servo_ids:
            packet_handler.WritePosEx(sid, trajectory.mid_pos, 15, 10, args.torque)
        time.sleep(2.0)

        # 主循环
        print(f"\n[3] 开始运动曲线追踪... (Ctrl+C 停止)")
        print(f"  📊 实时绘图窗口已打开")
        print(f"  {'📁 数据记录中...' if args.save else ''}")

        start_time = time.time()
        last_plot_time = 0
        plot_interval = 1.0 / PLOT_FPS
        frame_count = 0

        while True:
            current_time = time.time() - start_time

            # 检查运行时长
            if args.duration > 0 and current_time > args.duration:
                print(f"\n⏰ 运行时长 ({args.duration}s) 已到, 停止")
                break

            # 计算目标位置
            target_pos = trajectory.get_target(current_time)

            # 向所有舵机发送目标位置
            for sid in servo_ids:
                packet_handler.WritePosEx(sid, target_pos, args.speed, args.acc, args.torque)

            # 读取实际位置
            actual_positions = read_actual_positions(packet_handler, servo_ids)

            # 构建目标字典
            targets = {sid: target_pos for sid in servo_ids}

            # 每个控制周期都记录数据 (保证数据连续)
            record_data(data_buf, servo_ids, current_time, targets, actual_positions)

            # 记录CSV
            if logger:
                logger.log(current_time, targets, actual_positions)

            # 按刷新率渲染图表
            if current_time - last_plot_time >= plot_interval:
                render_plot(target_lines, actual_lines, data_buf, servo_ids)

                # 更新子图 X 轴范围
                x_min = max(0, current_time - PLOT_WINDOW)
                x_max = max(PLOT_WINDOW, current_time + 0.5)
                for i, sid in enumerate(servo_ids):
                    axes[i].set_xlim(x_min, x_max)

                fig.canvas.draw()
                fig.canvas.flush_events()
                last_plot_time = current_time

            # 打印状态
            frame_count += 1
            if frame_count % 50 == 0:
                t_deg = target_pos * 360.0 / 4095.0
                actual_str = " | ".join(
                    [f"ID{sid}: {actual_positions.get(sid, '?') * 360.0 / 4095.0:.1f}°" if actual_positions.get(sid)
                     else f"ID{sid}: ?" for sid in servo_ids]
                )
                print(f"\r  t={current_time:.1f}s | target={t_deg:.1f}° | {actual_str}", end="")

            time.sleep(0.02)  # ~50Hz 控制频率

    except KeyboardInterrupt:
        print(f"\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print(f"\n[4] 关闭...")
        # 停止运动, 回到中间位置
        for sid in servo_ids:
            packet_handler.WritePosEx(sid, trajectory.mid_pos, 30, 20, args.torque)
        time.sleep(1.0)

        # 释放扭矩
        disable_torque(packet_handler, servo_ids)

        if logger:
            logger.close()

        port_handler.closePort()
        plt.ioff()
        plt.close("all")
        print(f"🎉 程序安全退出 (共运行 {frame_count} 帧)")


if __name__ == "__main__":
    main()
