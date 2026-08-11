#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HLS3606 遥操作达妙夹爪 — 力反馈版
=====================================
严格对照 cv_motor_continuous_control_plot.py 的 Gripper 控制 + HardwareWorker 线程。
力反馈: 空载柔顺, 夹爪力矩超阈值→HLS产生阻力+限制闭合。

用法: python hls_teleop_gripper_force.py
"""

import sys, os, time, math, argparse, threading
import numpy as np
from pathlib import Path
from typing import Optional

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-rebotarm")
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from collections import deque
plt.ion()

# ---- HLS SDK ----
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../FTServo_Python"))
from scservo_sdk import *

# ---- Gripper ----
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "reBotArm_control_py"))
from actuator.gripper import Gripper

# ==================== 配置 ====================
GRIPPER_CONFIG = str(PROJECT_ROOT / "config" / "gripper.yaml")

P_OPEN = -5.7  # -5.8
P_CLOSE = 0.0
POSITION_FAULT_MARGIN = 0.75

CONTROL_RATE = 100.0
VELOCITY_LIMIT = 6.0
STARTUP_POSITION = 0.0
STARTUP_VELOCITY_LIMIT = 2.0
STARTUP_POSITION_TOLERANCE = 0.05
STARTUP_TIMEOUT = 8.0
HAND_LOSS_TIMEOUT = 0.35

# HLS
HLS_PORT = "/dev/ttyACM1"
HLS_BAUDRATE = 1000000
HLS_ID = 7
SERVO_CLOSED_DEG = 180.0
SERVO_OPEN_DEG = 90.0
SERVO_RANGE = 4095.0
SERVO_ANGLE = 360.0

# 力反馈
FORCE_THRESHOLD = 0.35
TORQUE_BASE = 50
TORQUE_GRASP = 500  # 600
BACKOFF_MARGIN = 0.1

# 力反馈释放参数
RELEASE_RAW_THRESHOLD = 10     # 用户拧开 ~0.88° 即触发释放 (90°行程=1024 raw)
FORCE_RATIO_SATURATE = 2.5      # 力矩映射饱和倍数: threshold * 2.5 时达到最大阻力

# 夹爪扭矩上限 (独立于力反馈阈值, 防止夹碎物体)
TORQUE_LIMIT = 0.6              # Nm, 扭矩超过此值禁止继续闭合 (但始终允许张开)
TORQUE_BACKOFF_GAIN = 0.2      # rad/Nm, 超调回退增益: overshoot * gain = 回退弧度

# 绘图
PLOT_RATE = 6.0
PLOT_HISTORY = 12.0
MAX_POINTS = round(PLOT_RATE * PLOT_HISTORY)


def clamp(v, lo, hi): return max(lo, min(float(v), hi))
def raw_to_deg(r): return float(r) / SERVO_RANGE * SERVO_ANGLE
def deg_to_raw(d): return int(d / SERVO_ANGLE * SERVO_RANGE)


def servo_deg_to_gripper_pos(deg):
    deg = clamp(deg, SERVO_OPEN_DEG, SERVO_CLOSED_DEG)
    norm = (SERVO_CLOSED_DEG - deg) / (SERVO_CLOSED_DEG - SERVO_OPEN_DEG)
    return P_CLOSE + norm * (P_OPEN - P_CLOSE)


def gripper_pos_to_servo_deg(pos):
    norm = (pos - P_CLOSE) / (P_OPEN - P_CLOSE) if abs(P_OPEN - P_CLOSE) > 1e-6 else 0.0
    norm = clamp(norm, 0.0, 1.0)
    return SERVO_CLOSED_DEG - norm * (SERVO_CLOSED_DEG - SERVO_OPEN_DEG)


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

    def set_pos_torque(self, raw, torq, spd=20, acc=10):
        self.pk.WritePosEx(self.sid, int(raw), int(spd), int(acc), int(torq))

    def read_raw(self):
        pos, res, _ = self.pk.ReadPos(self.sid)
        return pos if res == COMM_SUCCESS else None

    def close(self): self.ph.closePort()


# ==================== HardwareWorker (对齐 cv_motor) ====================

class HardwareWorker(threading.Thread):
    def __init__(self, config_path: str, torque_limit: Optional[float] = None):
        super().__init__(daemon=True)
        self.config_path = config_path
        self._torque_limit = torque_limit  # 线程内扭矩上限, None=不启用
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
                "position": self._position, "velocity": self._velocity,
                "torque": self._torque, "command": self._command,
                "enabled": self._enabled, "error": self._error,
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
                raise RuntimeError(f"零位初始化超时：当前 {position:.3f} rad，目标 {target:.3f} rad")

            gripper.pos_vel(target, STARTUP_VELOCITY_LIMIT)
            position, velocity, torque = gripper.get_state(request=False)
            time.sleep(1.0 / CONTROL_RATE)

        raise RuntimeError("零位初始化被中止")

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
                self._position, self._velocity, self._torque = position, velocity, torque
                self._command = startup_target
                self._target = startup_target
                self._target_time = time.monotonic()
                self._enabled = True
            self._ready_event.set()

            held_after_loss = False
            period = 1.0 / CONTROL_RATE
            next_tick = time.monotonic()
            prev_abs_torque = 0.0  # 用于扭矩变化率预判

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

                # ── 动态限速 (扭矩余量 + 扭矩变化率预判) ──
                if self._torque_limit is not None:
                    abs_t = abs(torque)
                    torque_headroom = max(0.0, self._torque_limit - abs_t)
                    torque_rate = (abs_t - prev_abs_torque) / period  # Nm/s

                    # 基础限速: 扭矩越接近上限越慢
                    if torque_headroom < 0.03:
                        vlim = 0.8
                    elif torque_headroom < 0.08:
                        vlim = 1.5
                    elif torque_headroom < 0.15:
                        vlim = 2.5
                    elif torque_headroom < 0.25:
                        vlim = 4.0
                    else:
                        vlim = VELOCITY_LIMIT  # 6.0

                    # 扭矩变化率预判: 扭矩快速上升 → 提前减速, 防止冲击尖峰
                    if torque_rate > 80:        # >80 Nm/s, 刚性碰撞
                        vlim = min(vlim, 1.0)
                    elif torque_rate > 40:      # >40 Nm/s, 即将接触
                        vlim = min(vlim, 2.0)
                    elif torque_rate > 15:      # >15 Nm/s, 轻触
                        vlim = min(vlim, 3.5)

                    prev_abs_torque = abs_t
                else:
                    vlim = VELOCITY_LIMIT

                gripper.pos_vel(target, vlim)
                position, velocity, torque = gripper.get_state(request=False)

                # ── 线程内扭矩上限快速回退 (延迟 ~10ms, 远快于主循环) ──
                if self._torque_limit is not None and abs(torque) > self._torque_limit:
                    overshoot = abs(torque) - self._torque_limit
                    backoff = overshoot * TORQUE_BACKOFF_GAIN
                    target = self.clamp_position(position - backoff)
                    gripper.pos_vel(target, 1.0)  # 回退时慢速, 避免震荡
                    position, velocity, torque = gripper.get_state(request=False)
                    prev_abs_torque = abs(torque)  # 更新, 避免下周期误判

                if not all(math.isfinite(v) for v in (position, velocity, torque)):
                    raise RuntimeError("电机反馈包含 NaN/Inf")
                if position < P_OPEN - POSITION_FAULT_MARGIN or position > P_CLOSE + POSITION_FAULT_MARGIN:
                    raise RuntimeError(f"位置 {position:.3f} rad 严重超出机械行程")

                with self._lock:
                    self._position, self._velocity, self._torque = position, velocity, torque
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


# ==================== 绘图 ====================

def make_plot():
    fig, axes = plt.subplots(4, 1, figsize=(8, 8), sharex=True, dpi=90)
    fig.suptitle("HLS3606 ↔ Damiao Gripper — Force Feedback", fontsize=12, fontweight="bold")

    hls_line, = axes[0].plot([], [], "b-", label="HLS Actual")
    hls_tgt_line, = axes[0].plot([], [], "b--", lw=1.0, alpha=0.5, label="HLS Target")
    axes[0].axhline(y=SERVO_CLOSED_DEG, color="orange", ls=":", alpha=0.5, label=f"Close({SERVO_CLOSED_DEG}°)")
    axes[0].axhline(y=SERVO_OPEN_DEG, color="green", ls=":", alpha=0.5, label=f"Open({SERVO_OPEN_DEG}°)")
    axes[0].set_ylabel("HLS (deg)")
    axes[0].set_ylim(SERVO_OPEN_DEG - 10, SERVO_CLOSED_DEG + 10)
    axes[0].grid(True, ls=":", alpha=0.6)
    axes[0].legend(loc="upper right", fontsize=7)

    pos_line, = axes[1].plot([], [], "r-", label="Actual")
    cmd_line, = axes[1].plot([], [], color="purple", ls="--", label="Command")
    axes[1].axhline(y=P_CLOSE, color="orange", ls=":", alpha=0.5)
    axes[1].axhline(y=P_OPEN, color="green", ls=":", alpha=0.5)
    axes[1].set_ylabel("Position (rad)")
    axes[1].set_ylim(P_OPEN - 0.5, P_CLOSE + 0.5)
    axes[1].grid(True, ls=":", alpha=0.6)
    axes[1].legend(loc="upper right", fontsize=7)

    vel_line, = axes[2].plot([], [], "g-", label="Velocity")
    axes[2].set_ylabel("Velocity (rad/s)")
    axes[2].grid(True, ls=":", alpha=0.6)
    axes[2].legend(loc="upper right", fontsize=7)

    torq_line, = axes[3].plot([], [], "m-", label="Torque")
    axes[3].axhline(y=FORCE_THRESHOLD, color="red", ls="--", alpha=0.5, label=f"Thresh({FORCE_THRESHOLD})")
    axes[3].set_ylabel("Torque (Nm)")
    axes[3].set_xlabel("Time (s)")
    axes[3].grid(True, ls=":", alpha=0.6)
    axes[3].legend(loc="upper right", fontsize=7)

    fig.tight_layout()
    return fig, axes, hls_line, hls_tgt_line, pos_line, cmd_line, vel_line, torq_line


# ==================== main ====================

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--hls-port", default=HLS_PORT)
    p.add_argument("--force-threshold", type=float, default=FORCE_THRESHOLD)
    p.add_argument("--yes", action="store_true")
    args = p.parse_args()
    threshold = args.force_threshold

    print("=" * 55)
    print("  HLS3606 ↔ Damiao Gripper 力反馈遥操作")
    print("=" * 55)
    print(f"  舵机: {SERVO_CLOSED_DEG}°(闭) ~ {SERVO_OPEN_DEG}°(开)")
    print(f"  夹爪: {P_CLOSE}(闭) ~ {P_OPEN}(开) rad")
    print(f"  力反馈阈值: {threshold} Nm")
    print(f"  空载: HLS扭矩释放, 柔顺 | 抓取: HLS扭矩{TORQUE_GRASP}, 锁闭")

    if not args.yes:
        ans = input("确认无障碍物。[y/N] ").strip().lower()
        if ans not in {"y", "yes"}:
            print("已取消。")
            return

    # [1] HLS
    print("\n[1] HLS3606...")
    hls = HLSServo(args.hls_port, HLS_BAUDRATE, HLS_ID)
    hls.connect()

    # [2] HardwareWorker
    print("\n[2] 达妙夹爪 HardwareWorker...")
    hw = HardwareWorker(GRIPPER_CONFIG, torque_limit=TORQUE_LIMIT)
    hw.start()
    if not hw.wait_ready(STARTUP_TIMEOUT + 4.0):
        err = hw.snapshot()["error"] or "超时"
        hw.request_stop()
        hw.join(timeout=3.0)
        raise RuntimeError(f"电机初始化失败: {err}")

    # [3] HLS 零位
    print(f"\n[3] HLS 零位...")
    mid_raw = deg_to_raw(SERVO_CLOSED_DEG)
    hls.enable()
    time.sleep(0.1)
    hls.set_pos_torque(mid_raw, 500, 30, 20)
    time.sleep(1.5)
    hls.release()
    print(f"  ✅ HLS 中位={SERVO_CLOSED_DEG}°, 扭矩释放")
    time.sleep(1.0)

    # [4] 力反馈遥操作
    print(f"\n[4] 力反馈遥操作 (Ctrl+C 停止)")

    fig, axes, hls_line, hls_tgt_line, pos_line, cmd_line, vel_line, torq_line = make_plot()

    times = deque(maxlen=MAX_POINTS)
    hls_angles = deque(maxlen=MAX_POINTS)
    hls_targets = deque(maxlen=MAX_POINTS)
    positions = deque(maxlen=MAX_POINTS)
    commands = deque(maxlen=MAX_POINTS)
    velocities = deque(maxlen=MAX_POINTS)
    torques = deque(maxlen=MAX_POINTS)

    t0 = time.monotonic()
    last_plot = 0.0

    grasping = False
    grasp_pos = P_CLOSE
    grasp_hls_raw = mid_raw
    hls_target_raw = mid_raw   # 动态HLS目标位置, 跟随用户往OPEN方向移动
    hls_torque = 0
    torque_limited = False     # 扭矩上限锁定标志

    try:
        while True:
            now = time.monotonic() - t0

            # 读 HLS
            raw = hls.read_raw()
            if raw is not None:
                deg = raw_to_deg(raw)
                grip_target = servo_deg_to_gripper_pos(deg)
            else:
                deg = SERVO_CLOSED_DEG
                grip_target = P_CLOSE

            # 读夹爪
            state = hw.snapshot()
            if state["error"]:
                raise RuntimeError(state["error"])
            pos = state["position"]
            torq = state["torque"]
            abs_torque = abs(torq)

            # ── 力反馈状态机 ──
            if abs_torque > threshold and not grasping:
                grasping = True
                grasp_pos = pos
                grasp_hls_raw = raw if raw is not None else mid_raw
                hls_target_raw = grasp_hls_raw   # 动态目标初始化为抓取点
                print(f"\n  ⚡ GRASP! torque={abs_torque:.3f} > {threshold}")

            elif abs_torque <= threshold * 0.5 and grasping:
                grasping = False
                hls.release()
                hls_torque = 0
                print(f"\n  ✅ free (torque dropped)")

            # ── HLS 力反馈 (比例 + 方向感知) ──
            if grasping:
                current_raw = raw if raw is not None else grasp_hls_raw

                # 用户往 OPEN 方向拧: raw变小 (180°→90°, raw 2048→1024)
                # 动态目标跟随用户, 允许释放
                if current_raw < hls_target_raw:
                    hls_target_raw = current_raw

                # 检测用户拧开角度, 超过阈值直接释放舵机
                open_deviation = grasp_hls_raw - hls_target_raw
                if open_deviation > RELEASE_RAW_THRESHOLD:
                    grasping = False
                    hls.release()
                    hls_torque = 0
                    print(f"\n  ✅ released (user opened servo, dev={open_deviation})")
                else:
                    # 比例扭矩: 夹爪力矩 → HLS阻力, 力越大阻力越大
                    force_ratio = min(1.0, abs_torque / (threshold * FORCE_RATIO_SATURATE))
                    hls_torque = int(TORQUE_BASE + force_ratio * (TORQUE_GRASP - TORQUE_BASE))

                    if hls_torque > 0:
                        hls.enable()
                    hls.set_pos_torque(hls_target_raw, hls_torque, 30, 15)

            # ── 安全限制夹爪指令 (抓取时只能开不能继续闭) ──
            if grasping:
                grip_target = min(grip_target, grasp_pos + BACKOFF_MARGIN)
                grip_target = clamp(grip_target, P_OPEN, P_CLOSE)

            # ── 扭矩上限保护 (独立于力反馈, 主动比例回退) ──
            if abs_torque > TORQUE_LIMIT:
                overshoot = abs_torque - TORQUE_LIMIT
                backoff = overshoot * TORQUE_BACKOFF_GAIN
                grip_target = clamp(pos - backoff, P_OPEN, P_CLOSE)  # 主动回退
                if not torque_limited:
                    torque_limited = True
                    print(f"\n  ⛔ TORQUE LIMIT! torque={abs_torque:.3f} > {TORQUE_LIMIT}, "
                          f"overshoot={overshoot:.3f}, backoff={backoff:.3f}")
            elif abs_torque < TORQUE_LIMIT * 0.85 and torque_limited:
                torque_limited = False
                print(f"\n  ✅ torque ok ({abs_torque:.3f} < {TORQUE_LIMIT * 0.85:.3f})")

            hw.set_target(grip_target)

            # HLS 回正目标 (显示用)
            tgt_deg = gripper_pos_to_servo_deg(pos)

            # 绘图
            if now - last_plot >= 1.0 / PLOT_RATE:
                elapsed = now
                times.append(elapsed)
                hls_angles.append(deg)
                hls_targets.append(tgt_deg)
                positions.append(pos)
                commands.append(state["command"])
                velocities.append(state["velocity"])
                torques.append(abs_torque)

                hls_line.set_data(times, hls_angles)
                hls_tgt_line.set_data(times, hls_targets)
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
                    tmax = max(0.1, max(abs(v) for v in torques) * 1.15, threshold * 2)
                    axes[3].set_ylim(-0.1, tmax)

                fig.canvas.draw()
                fig.canvas.flush_events()
                last_plot = now

            # 终端
            pct = (pos - P_OPEN) / (P_CLOSE - P_OPEN) * 100 if abs(P_CLOSE - P_OPEN) > 1e-6 else 0
            pct = clamp(pct, 0, 100)
            st = "⚡GRASP" if grasping else "  free"
            st += " ⛔TLIM" if torque_limited else ""
            print(f"\r  t={now:5.1f}s | HLS={deg:5.1f}°→{tgt_deg:5.1f}° | "
                  f"pos={pos:+.3f} torq={abs_torque:.3f} | {st} | hls_t={hls_torque:3d}  ",
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
