#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HLS3606 舵机遥操作达妙夹爪
==============================
严格对照 cv_motor_continuous_control_plot.py 的 Gripper 控制逻辑:
  - Gripper 类 + POS_VEL 模式
  - HardwareWorker 独立线程
  - 慢速零位初始化到闭合位置
  - 主线程读 HLS 舵机 + 绘图

用法: python hls_teleop_gripper.py
"""

import sys, os, time, math, argparse, threading
import numpy as np
from pathlib import Path
from typing import Optional

# ---- matplotlib ----
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-rebotarm")
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from collections import deque
plt.ion()  # 必须在创建任何 figure 之前开启交互模式

# ---- HLS SDK ----
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../FTServo_Python"))
from scservo_sdk import *

# ---- Gripper (精确对齐 cv_motor 引用) ----
PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROL_PKG = PROJECT_ROOT / "reBotArm_control_py"
sys.path.insert(0, str(CONTROL_PKG))
from actuator.gripper import Gripper

# ==================== 配置 (对齐 cv_motor) ====================
GRIPPER_CONFIG = str(PROJECT_ROOT / "config" / "gripper.yaml")

# 夹爪行程 (对齐 cv_motor: P_CLOSE=0.0, P_OPEN=-5.8)
P_OPEN = -5.8
P_CLOSE = 0.0
POSITION_FAULT_MARGIN = 0.75

# 控制参数 (对齐 cv_motor)
CONTROL_RATE = 100.0
VELOCITY_LIMIT = 6.0
STARTUP_POSITION = 0.0        # 闭合
STARTUP_VELOCITY_LIMIT = 2.0
STARTUP_POSITION_TOLERANCE = 0.05
STARTUP_TIMEOUT = 8.0
HAND_LOSS_TIMEOUT = 0.35

# HLS 舵机
HLS_PORT = "/dev/ttyACM1"
HLS_BAUDRATE = 1000000
HLS_ID = 7
SERVO_CLOSED_DEG = 180.0   # 中位=闭合
SERVO_OPEN_DEG = 90.0      # 左转=张开
SERVO_RANGE = 4095.0
SERVO_ANGLE = 360.0

# 绘图
PLOT_RATE = 6.0
PLOT_HISTORY = 12.0
MAX_POINTS = round(PLOT_RATE * PLOT_HISTORY)


def clamp(v, lo, hi): return max(lo, min(float(v), hi))
def raw_to_deg(r): return float(r) / SERVO_RANGE * SERVO_ANGLE


def servo_deg_to_gripper_pos(deg):
    """舵机角度 → 夹爪 rad (对齐参考映射)"""
    deg = clamp(deg, SERVO_OPEN_DEG, SERVO_CLOSED_DEG)
    norm = (SERVO_CLOSED_DEG - deg) / (SERVO_CLOSED_DEG - SERVO_OPEN_DEG)
    return P_CLOSE + norm * (P_OPEN - P_CLOSE)


# ==================== HLS ====================

class HLSServo:
    def __init__(self, port, baud, sid):
        self.ph = PortHandler(port)
        self.pk = hls(self.ph)
        self.sid = sid

    def connect(self):
        if not self.ph.openPort(): raise RuntimeError("HLS open fail")
        if not self.ph.setBaudRate(self.ph.baudrate): raise RuntimeError("HLS baud fail")

    def enable(self):
        self.pk.write1ByteTxRx(self.sid, HLS_TORQUE_ENABLE, 1)

    def release(self):
        self.pk.write1ByteTxRx(self.sid, HLS_TORQUE_ENABLE, 0)

    def move(self, raw, speed=30, acc=20, torq=500):
        self.pk.WritePosEx(self.sid, int(raw), int(speed), int(acc), int(torq))

    def read_raw(self):
        pos, res, _ = self.pk.ReadPos(self.sid)
        return pos if res == COMM_SUCCESS else None

    def close(self): self.ph.closePort()


# ==================== HardwareWorker (精确对齐 cv_motor) ====================

class HardwareWorker(threading.Thread):
    """独占 Gripper/motorbridge 的位置控制线程。"""

    def __init__(self, config_path: str):
        super().__init__(daemon=True)
        self.config_path = config_path
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._target: Optional[float] = None
        self._target_time = 0.0
        self._position = 0.0
        self._velocity = 0.0
        self._torque = 0.0
        self._command = 0.0
        self._enabled = False
        self._error: Optional[str] = None

    @staticmethod
    def clamp_position(position: float) -> float:
        return min(P_CLOSE, max(P_OPEN, float(position)))

    def set_target(self, position: float) -> None:
        with self._lock:
            self._target = self.clamp_position(position)
            self._target_time = time.monotonic()

    def request_stop(self) -> None:
        self._stop_event.set()

    def wait_ready(self, timeout: float) -> bool:
        self._ready_event.wait(timeout)
        with self._lock:
            return self._enabled and self._error is None

    def snapshot(self):
        with self._lock:
            return {
                "position": self._position,
                "velocity": self._velocity,
                "torque": self._torque,
                "command": self._command,
                "enabled": self._enabled,
                "error": self._error,
            }

    def _move_to_startup_position(self, gripper: Gripper):
        target = self.clamp_position(STARTUP_POSITION)
        deadline = time.monotonic() + STARTUP_TIMEOUT
        position, velocity, torque = gripper.get_state(request=True)

        while not self._stop_event.is_set():
            if not all(math.isfinite(v) for v in (position, velocity, torque)):
                raise RuntimeError("零位初始化时电机反馈包含 NaN/Inf")
            if position < P_OPEN - POSITION_FAULT_MARGIN or position > P_CLOSE + POSITION_FAULT_MARGIN:
                raise RuntimeError(f"零位初始化时位置 {position:.3f} rad 严重越界")

            with self._lock:
                self._position, self._velocity, self._torque = position, velocity, torque
                self._command = target

            if abs(position - target) <= STARTUP_POSITION_TOLERANCE:
                return position, velocity, torque
            if time.monotonic() >= deadline:
                raise RuntimeError(f"夹爪零位初始化超时：当前 {position:.3f} rad，目标 {target:.3f} rad")

            gripper.pos_vel(target, STARTUP_VELOCITY_LIMIT)
            position, velocity, torque = gripper.get_state(request=False)
            time.sleep(1.0 / CONTROL_RATE)

        raise RuntimeError("夹爪零位初始化被中止")

    def run(self) -> None:
        gripper = None
        try:
            gripper = Gripper(str(self.config_path))
            gripper.disable()
            if not gripper.mode_pos_vel():
                raise RuntimeError("切换 POS_VEL 模式失败")
            if not gripper.enable():
                raise RuntimeError("电机使能失败")

            print(f"  电机：限速初始化到 {STARTUP_POSITION:.1f} rad 零位……")
            position, velocity, torque = self._move_to_startup_position(gripper)
            print("  电机：零位初始化完成。")

            startup_target = self.clamp_position(STARTUP_POSITION)
            with self._lock:
                self._position = position
                self._velocity = velocity
                self._torque = torque
                self._command = startup_target
                self._target = startup_target
                self._target_time = time.monotonic()
                self._enabled = True
            self._ready_event.set()

            held_after_loss = False
            period = 1.0 / CONTROL_RATE
            next_tick = time.monotonic()

            while not self._stop_event.is_set():
                with self._lock:
                    target = self._target
                    target_age = time.monotonic() - self._target_time

                if target is None or target_age > HAND_LOSS_TIMEOUT:
                    if not held_after_loss:
                        target = self.clamp_position(position)
                        with self._lock:
                            self._target = target
                        held_after_loss = True
                else:
                    held_after_loss = False

                target = self.clamp_position(target)
                gripper.pos_vel(target, VELOCITY_LIMIT)
                position, velocity, torque = gripper.get_state(request=False)

                if not all(math.isfinite(v) for v in (position, velocity, torque)):
                    raise RuntimeError("电机反馈包含 NaN/Inf")
                if position < P_OPEN - POSITION_FAULT_MARGIN or position > P_CLOSE + POSITION_FAULT_MARGIN:
                    raise RuntimeError(f"位置 {position:.3f} rad 严重超出机械行程")

                with self._lock:
                    self._position = position
                    self._velocity = velocity
                    self._torque = torque
                    self._command = target

                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()

        except Exception as exc:
            with self._lock:
                self._error = str(exc)
                self._enabled = False
            self._ready_event.set()
        finally:
            if gripper is not None:
                try:
                    gripper.disconnect()
                except Exception:
                    pass
            with self._lock:
                self._enabled = False


# ==================== 绘图 (对齐 cv_motor make_plot) ====================

def make_plot():
    fig, axes = plt.subplots(4, 1, figsize=(8, 8), sharex=True, dpi=90)
    fig.suptitle("HLS3606 → Damiao Gripper Teleoperation", fontsize=12, fontweight="bold")

    # 舵机角度
    hls_line, = axes[0].plot([], [], "b-", label="HLS Angle")
    axes[0].axhline(y=SERVO_CLOSED_DEG, color="orange", linestyle=":", alpha=0.5, label=f"Close({SERVO_CLOSED_DEG}°)")
    axes[0].axhline(y=SERVO_OPEN_DEG, color="green", linestyle=":", alpha=0.5, label=f"Open({SERVO_OPEN_DEG}°)")
    axes[0].set_ylabel("HLS (deg)")
    axes[0].set_ylim(SERVO_OPEN_DEG - 10, SERVO_CLOSED_DEG + 10)
    axes[0].grid(True, linestyle=":", alpha=0.6)
    axes[0].legend(loc="upper right", fontsize=7)

    # 夹爪位置
    pos_line, = axes[1].plot([], [], "r-", label="Actual")
    cmd_line, = axes[1].plot([], [], color="purple", linestyle="--", label="Command")
    axes[1].axhline(y=P_CLOSE, color="orange", linestyle=":", alpha=0.5, label=f"Close({P_CLOSE})")
    axes[1].axhline(y=P_OPEN, color="green", linestyle=":", alpha=0.5, label=f"Open({P_OPEN})")
    axes[1].set_ylabel("Position (rad)")
    axes[1].set_ylim(P_OPEN - 0.5, P_CLOSE + 0.5)
    axes[1].grid(True, linestyle=":", alpha=0.6)
    axes[1].legend(loc="upper right", fontsize=7)

    # 夹爪速度
    vel_line, = axes[2].plot([], [], "g-", label="Velocity")
    axes[2].set_ylabel("Velocity (rad/s)")
    axes[2].grid(True, linestyle=":", alpha=0.6)
    axes[2].legend(loc="upper right", fontsize=7)

    # 夹爪力矩
    torq_line, = axes[3].plot([], [], "m-", label="Torque")
    axes[3].set_ylabel("Torque (Nm)")
    axes[3].set_xlabel("Time (s)")
    axes[3].grid(True, linestyle=":", alpha=0.6)
    axes[3].legend(loc="upper right", fontsize=7)

    fig.tight_layout()
    return fig, axes, hls_line, pos_line, cmd_line, vel_line, torq_line


# ==================== main ====================

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--hls-port", default=HLS_PORT)
    p.add_argument("--yes", action="store_true", help="跳过确认")
    args = p.parse_args()

    print("=" * 55)
    print("  HLS3606 → Damiao Gripper 遥操作")
    print("=" * 55)
    print(f"  舵机: {SERVO_CLOSED_DEG}°(闭) ~ {SERVO_OPEN_DEG}°(开)")
    print(f"  夹爪: {P_CLOSE}(闭) ~ {P_OPEN}(开) rad")
    print(f"  控制: POS_VEL, {CONTROL_RATE:.0f}Hz, vlim={VELOCITY_LIMIT}")
    print(f"  零位: {STARTUP_POSITION} rad (闭合)")

    if not args.yes:
        ans = input("确认夹爪运动范围内无障碍物。[y/N] ").strip().lower()
        if ans not in {"y", "yes"}:
            print("已取消。")
            return

    # [1] HLS
    print("\n[1] HLS3606...")
    hls = HLSServo(args.hls_port, HLS_BAUDRATE, HLS_ID)
    hls.connect()

    # [2] 启动 HardwareWorker (对齐 cv_motor)
    print("\n[2] 达妙夹爪 HardwareWorker...")
    hw = HardwareWorker(GRIPPER_CONFIG)
    hw.start()
    if not hw.wait_ready(STARTUP_TIMEOUT + 4.0):
        err = hw.snapshot()["error"] or "电机初始化超时"
        hw.request_stop()
        hw.join(timeout=3.0)
        raise RuntimeError(f"电机初始化失败: {err}")

    # [3] HLS 零位
    print(f"\n[3] HLS 零位...")
    mid_raw = int(SERVO_CLOSED_DEG / SERVO_ANGLE * SERVO_RANGE)
    hls.enable()
    time.sleep(0.1)
    hls.move(mid_raw, 30, 20, 500)
    time.sleep(1.5)
    hls.release()
    print(f"  ✅ HLS 中位={SERVO_CLOSED_DEG}°, 扭矩释放")
    time.sleep(1.0)

    # [4] 遥操作
    print(f"\n[4] 遥操作 (Ctrl+C 停止)")

    fig, axes, hls_line, pos_line, cmd_line, vel_line, torq_line = make_plot()

    times = deque(maxlen=MAX_POINTS)
    hls_angles = deque(maxlen=MAX_POINTS)
    positions = deque(maxlen=MAX_POINTS)
    commands = deque(maxlen=MAX_POINTS)
    velocities = deque(maxlen=MAX_POINTS)
    torques = deque(maxlen=MAX_POINTS)

    t0 = time.monotonic()
    last_plot = 0.0

    try:
        while True:
            now = time.monotonic() - t0

            # 读 HLS 舵机
            raw = hls.read_raw()
            if raw is not None:
                deg = raw_to_deg(raw)
                grip_target = servo_deg_to_gripper_pos(deg)
                hw.set_target(grip_target)
            else:
                deg = SERVO_CLOSED_DEG
                grip_target = P_CLOSE

            # 读夹爪状态
            state = hw.snapshot()
            if state["error"]:
                raise RuntimeError(state["error"])

            # 绘图 (对齐 cv_motor 的绘图频率)
            if now - last_plot >= 1.0 / PLOT_RATE:
                elapsed = now
                times.append(elapsed)
                positions.append(state["position"])
                commands.append(state["command"])
                velocities.append(state["velocity"])
                torques.append(state["torque"])
                hls_angles.append(deg)

                hls_line.set_data(times, hls_angles)
                pos_line.set_data(times, positions)
                cmd_line.set_data(times, commands)
                vel_line.set_data(times, velocities)
                torq_line.set_data(times, torques)

                left = max(0, times[-1] - PLOT_HISTORY) if times else 0
                right = max(left + 1.0, times[-1]) if times else 1.0
                for ax in axes:
                    ax.set_xlim(left, right)
                if velocities:
                    vmax = max(0.2, max(abs(v) for v in velocities) * 1.15)
                    axes[2].set_ylim(-vmax, vmax)
                if torques:
                    tmax = max(0.1, max(abs(v) for v in torques) * 1.15)
                    axes[3].set_ylim(-tmax, tmax)

                fig.canvas.draw()
                fig.canvas.flush_events()
                last_plot = now

            # 终端
            pos = state["position"]
            pct = (pos - P_OPEN) / (P_CLOSE - P_OPEN) * 100 if abs(P_CLOSE - P_OPEN) > 1e-6 else 0
            pct = clamp(pct, 0, 100)
            bar = "█" * int(pct / 4) + "░" * (25 - int(pct / 4))
            print(f"\r  t={now:5.1f}s | HLS={deg:5.1f}° → cmd={grip_target:+.3f} "
                  f"pos={pos:+.3f} vel={state['velocity']:+.2f} torq={state['torque']:+.3f} |{bar}|",
                  end="", flush=True)

            time.sleep(1.0 / CONTROL_RATE)

    except KeyboardInterrupt:
        print("\n⚠️ 中断")
    except Exception as e:
        print(f"\n❌ {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[5] 退出...")
        hw.request_stop()
        hw.join(timeout=4.0)
        hls.release()
        hls.close()
        plt.close("all")
        print("🎉 完成")


if __name__ == "__main__":
    main()
