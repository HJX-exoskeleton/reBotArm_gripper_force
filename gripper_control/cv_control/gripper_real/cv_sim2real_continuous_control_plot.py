#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo + 真机视觉触觉夹爪连续控制。

真机控制与 cv_motor_continuous_tactile_control_plot.py 共用同一套
夹爪归零、视觉映射、FlexiTac 融合和安全保护逻辑。CV先控制
MuJoCo 夹爪，仿真物理后的实际夹爪位置再映射为真机目标；
FlexiTac 在真机端执行最终的接触保护和位置退让。

  python3 gripper_control/cv_control/gripper_real/cv_sim2real_continuous_control_plot.py

  关闭曲线或仿真触觉窗口：

  python3 gripper_control/cv_control/gripper_real/cv_sim2real_continuous_control_plot.py \
    --no-plot \
    --no-sim-tactile

"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Optional

# Conda/OpenCV 可能扰乱Mesa的驱动搜索。在导入MuJoCo/GLFW前明确选择
# 系统软件渲染驱动，避免 iris/swrast_dri.so 加载失败。
os.environ.setdefault("LIBGL_DRIVERS_PATH", "/usr/lib/x86_64-linux-gnu/dri")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")

import cv2
import mujoco
import mujoco.viewer
import numpy as np

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-rebotarm")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import HandTrackingModule as htm


PROJECT_ROOT = Path(__file__).resolve().parents[3]
GRIPPER_CONTROL_ROOT = PROJECT_ROOT / "gripper_control"
if str(GRIPPER_CONTROL_ROOT) not in sys.path:
    sys.path.insert(0, str(GRIPPER_CONTROL_ROOT))

from cv_control.gripper_real.cv_motor_continuous_control_plot import (  # noqa: E402
    filter_pinch_ratio,
    normalized_pinch_ratio,
    ratio_to_position,
)
from cv_control.gripper_real.cv_motor_continuous_tactile_control_plot import (  # noqa: E402
    CONTROLLER_READY_TIMEOUT,
    HAND_LOSS_TIMEOUT,
    MAX_POINTS,
    P_CLOSE,
    P_OPEN,
    PLOT_RATE,
    TARGET_FORCE,
    FusionController,
    SharedState,
    TACTILE_PORT,
    TactileReader,
    make_dashboard,
    render_figure,
    render_tactile,
)


DEFAULT_XML_PATH = (
    PROJECT_ROOT
    / "mujoco"
    / "assets_robot_xml"
    / "rebotarm_gripper"
    / "rebotarm_sim_transfer_cube.xml"
)
SIM_GRIPPER_ACTUATOR = "gripper"
SIM_TACTILE_ROWS = 8
SIM_TACTILE_COLS = 16
SIM_TACTILE_POINTS = SIM_TACTILE_ROWS * SIM_TACTILE_COLS
SIM_CONTROL_RATE = 100.0
SIM_GRIPPER_ALPHA = 0.90
SIM_TACTILE_FORCE_MAX = 0.05


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MuJoCo-真机视觉触觉夹爪连续控制")
    parser.add_argument("--camera", type=int, default=0, help="摄像头编号，默认0")
    parser.add_argument("--tactile-port", default=TACTILE_PORT, help="FlexiTac 串口")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML_PATH, help="MuJoCo XML 路径")
    parser.add_argument("--no-plot", action="store_true", help="关闭曲线窗口")
    parser.add_argument("--no-sim-tactile", action="store_true", help="关闭仿真触觉热力图")
    parser.add_argument("--no-viewer", action="store_true", help="关闭MuJoCo 3D仿真画面")
    parser.add_argument("--yes", action="store_true", help="跳过启动安全确认")
    return parser


def real_to_sim_position(position: float, sim_close: float, sim_open: float) -> float:
    """将真机 [-5.8, 0] rad 映射到仿真 [open, close] m。"""
    position = float(np.clip(position, P_OPEN, P_CLOSE))
    return float(np.interp(position, (P_OPEN, P_CLOSE), (sim_open, sim_close)))


def sim_to_real_position(position: float, sim_close: float, sim_open: float) -> float:
    """将仿真夹爪实际位置 [close, open] m 映射回真机 rad。"""
    position = float(np.clip(position, sim_close, sim_open))
    return float(np.interp(position, (sim_close, sim_open), (P_CLOSE, P_OPEN)))


class SimTactileReader:
    """按名称缓存 MuJoCo 左右 8x16 力传感器地址。"""

    def __init__(self, model: mujoco.MjModel):
        self.addresses = {
            side: self._sensor_addresses(model, side) for side in ("left", "right")
        }

    @staticmethod
    def _sensor_addresses(model: mujoco.MjModel, side: str) -> np.ndarray:
        addresses = []
        for index in range(SIM_TACTILE_POINTS):
            name = f"touch_point_{side}_{index:03d}"
            try:
                sensor_id = model.sensor(name).id
            except KeyError as exc:
                raise RuntimeError(f"MuJoCo 缺少触觉传感器 {name}") from exc
            if int(model.sensor_dim[sensor_id]) != 3:
                raise RuntimeError(f"MuJoCo 触觉传感器 {name} 不是3维力数据")
            addresses.append(int(model.sensor_adr[sensor_id]))
        return np.asarray(addresses, dtype=np.int32)

    def read(self, data: mujoco.MjData, side: str) -> np.ndarray:
        values = np.empty(SIM_TACTILE_POINTS, dtype=np.float32)
        for index, address in enumerate(self.addresses[side]):
            values[index] = float(np.linalg.norm(data.sensordata[address:address + 3]))
        # 与 rebotarm_with_gripper_vis_rgb_8_16.py 的 column_major_xy 一致。
        return values.reshape(SIM_TACTILE_COLS, SIM_TACTILE_ROWS).T


class SimulationWorker(threading.Thread):
    """独立100 Hz推进MuJoCo，避免摄像头和UI阻塞夹爪映射。"""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        actuator_id: int,
        sim_close: float,
        sim_open: float,
        state: SharedState,
        stop_event: threading.Event,
        publish_actual_to_real: bool,
    ):
        super().__init__(daemon=True)
        self.model = model
        self.data = data
        self.actuator_id = actuator_id
        self.sim_close = sim_close
        self.sim_open = sim_open
        self.state = state
        self.stop_event = stop_event
        self.publish_actual_to_real = publish_actual_to_real
        self.data_lock = threading.Lock()
        self.error: Optional[str] = None
        self._command = real_to_sim_position(0.0, sim_close, sim_open)
        self._actual = self._command
        self._target_position = 0.0
        self._target_time = 0.0
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0:
            raise RuntimeError("MuJoCo夹爪执行器未连接到关节")
        self._qpos_adr = int(model.jnt_qposadr[joint_id])

    def set_target(self, position: float) -> None:
        with self.data_lock:
            self._target_position = float(np.clip(position, P_OPEN, P_CLOSE))
            self._target_time = time.monotonic()

    def snapshot(self) -> tuple[float, float]:
        with self.data_lock:
            return self._command, self._actual

    def run(self) -> None:
        period = 1.0 / SIM_CONTROL_RATE
        steps_per_tick = max(1, int(round(period / float(self.model.opt.timestep))))
        next_tick = time.monotonic()
        filtered_command = self._command
        held_after_loss = False
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                with self.data_lock:
                    target_position = self._target_position
                    target_age = now - self._target_time
                    actual_position = self._actual
                if self._target_time <= 0.0 or target_age > HAND_LOSS_TIMEOUT:
                    if not held_after_loss:
                        target_position = sim_to_real_position(
                            actual_position, self.sim_close, self.sim_open
                        )
                        with self.data_lock:
                            self._target_position = target_position
                        held_after_loss = True
                else:
                    held_after_loss = False

                target = real_to_sim_position(
                    target_position, self.sim_close, self.sim_open
                )
                filtered_command = (
                    SIM_GRIPPER_ALPHA * target
                    + (1.0 - SIM_GRIPPER_ALPHA) * filtered_command
                )
                with self.data_lock:
                    self.data.ctrl[self.actuator_id] = filtered_command
                    for _ in range(steps_per_tick):
                        mujoco.mj_step(self.model, self.data)
                    self._command = filtered_command
                    self._actual = float(np.clip(
                        self.data.qpos[self._qpos_adr], self.sim_close, self.sim_open
                    ))
                    actual_real_position = sim_to_real_position(
                        self._actual, self.sim_close, self.sim_open
                    )

                if self.publish_actual_to_real:
                    # Sim2Real串联：真机只接收仿真物理后的实际位置。
                    self.state.set_visual_target(actual_real_position)

                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    # UI占用数据锁较久时不追赶过期周期。
                    next_tick = time.monotonic()
        except Exception as exc:
            self.error = str(exc)
            with self.state.lock:
                self.state.error = f"MuJoCo仿真错误: {exc}"
            self.stop_event.set()


def render_sim_tactile(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """以demo中的固定0.05 N量程显示仿真左右触觉。"""
    peak = max(float(np.max(left)), float(np.max(right)))

    def colorize(matrix: np.ndarray) -> np.ndarray:
        gray = np.clip(matrix, 0.0, SIM_TACTILE_FORCE_MAX)
        gray = (gray / SIM_TACTILE_FORCE_MAX * 255.0).astype(np.uint8)
        image = cv2.applyColorMap(gray, cv2.COLORMAP_VIRIDIS)
        image = cv2.resize(image, (480, 240), interpolation=cv2.INTER_NEAREST)
        return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)

    left_image = colorize(left)
    right_image = colorize(right)
    combined = np.hstack((left_image, right_image))
    cv2.rectangle(combined, (0, 0), (combined.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(
        combined,
        f"Sim tactile  Left / Right   peak {peak:.4f} N",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )
    return combined


def main(coupling: str = "sim2real") -> int:
    if coupling not in {"sim2real", "parallel"}:
        raise ValueError(f"未知控制架构: {coupling}")
    args = build_parser().parse_args()
    xml_path = args.xml.expanduser().resolve()
    if not xml_path.is_file():
        print(f"MuJoCo XML 不存在: {xml_path}", file=sys.stderr)
        return 2
    if not args.yes:
        answer = input("请保持FlexiTac无接触并确认夹爪周围安全。[y/N] ").strip().lower()
        if answer not in {"y", "yes"}:
            return 0

    stop_event = threading.Event()
    state = SharedState()

    def handle_signal(signum, _frame):
        print(f"\n收到信号 {signum}，正在安全退出……")
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    tactile = TactileReader(state, stop_event, args.tactile_port)
    controller: Optional[FusionController] = None
    sim_worker: Optional[SimulationWorker] = None
    camera = None
    viewer = None
    figure = axes = lines = plot_image = None

    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(model)
        try:
            actuator_id = model.actuator(SIM_GRIPPER_ACTUATOR).id
        except KeyError as exc:
            raise RuntimeError(f"MuJoCo XML 缺少执行器 {SIM_GRIPPER_ACTUATOR!r}") from exc
        if not model.actuator_ctrllimited[actuator_id]:
            raise RuntimeError("MuJoCo 夹爪执行器未配置 ctrlrange")
        sim_close, sim_open = map(float, model.actuator_ctrlrange[actuator_id])
        if sim_close >= sim_open:
            raise RuntimeError(f"MuJoCo 夹爪 ctrlrange 无效: {sim_close}, {sim_open}")
        sim_tactile = None if args.no_sim_tactile else SimTactileReader(model)
        data.ctrl[actuator_id] = real_to_sim_position(0.0, sim_close, sim_open)
        mujoco.mj_forward(model, data)
        print(
            f"MuJoCo：已加载 {xml_path.name}，夹爪范围 "
            f"{sim_close:.3f}～{sim_open:.3f} m。"
        )

        # viewer 会创建OpenGL上下文，必须在启动真机前先确认它可用。
        if not args.no_viewer:
            viewer = mujoco.viewer.launch_passive(model, data)

        tactile.start()
        if not tactile.ready.wait(5.0) or tactile.error:
            raise RuntimeError(f"触觉初始化失败: {tactile.error or '超时'}")

        controller = FusionController(state, stop_event)
        controller.start()
        if not controller.ready.wait(CONTROLLER_READY_TIMEOUT) or state.snapshot()["error"]:
            raise RuntimeError(state.snapshot()["error"] or "电机初始化超时")

        sim_worker = SimulationWorker(
            model,
            data,
            actuator_id,
            sim_close,
            sim_open,
            state,
            stop_event,
            publish_actual_to_real=(coupling == "sim2real"),
        )
        sim_worker.start()

        detector = htm.handDetector(
            maxHands=1, model_complexity=0, detectionCon=0.70, trackCon=0.75
        )
        camera = cv2.VideoCapture(args.camera)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_FPS, 60)
        if not camera.isOpened():
            raise RuntimeError(f"无法打开摄像头 {args.camera}")

        if not args.no_plot:
            figure, axes, lines = make_dashboard()

        buffers = [deque(maxlen=MAX_POINTS) for _ in range(6)]
        times, positions, commands, forces, force_targets, torques = buffers
        start = previous = time.monotonic()
        last_plot = 0.0
        filtered_ratio = None
        architecture = "CV→Sim实际位置→真机" if coupling == "sim2real" else "CV→Sim/真机并行"
        print(f"{architecture} 控制已启动（MuJoCo 100 Hz）；按q或Ctrl+C退出。")

        while (
            not stop_event.is_set()
            and (viewer is None or viewer.is_running())
        ):
            ok, image = camera.read()
            if not ok:
                raise RuntimeError("摄像头读取失败")
            image = cv2.flip(image, 1)
            image = detector.findHands(image, draw=True)
            landmarks = detector.findPosition(image, draw=False)
            ratio = normalized_pinch_ratio(landmarks)
            if ratio is not None:
                filtered_ratio = filter_pinch_ratio(filtered_ratio, ratio)
                visual_target = ratio_to_position(filtered_ratio)
                sim_worker.set_target(visual_target)
                if coupling == "parallel":
                    # Sim-and-Real并行：CV目标直接同时发布给仿真和真机。
                    state.set_visual_target(visual_target)
                thumb = tuple(landmarks[4][1:3])
                index = tuple(landmarks[8][1:3])
                center = ((thumb[0] + index[0]) // 2, (thumb[1] + index[1]) // 2)
                cv2.line(image, thumb, index, (255, 0, 255), 3)
                cv2.circle(image, center, 8, (0, 255, 255), cv2.FILLED)
                cv2.putText(
                    image,
                    f"Pinch {filtered_ratio:.2f} -> {visual_target:.2f} rad",
                    (max(5, center[0] - 110), max(25, center[1] - 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                )
            else:
                filtered_ratio = None
                cv2.putText(
                    image, "Hand lost: hold", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2,
                )

            now = time.monotonic()
            snapshot = state.snapshot()
            if snapshot["error"]:
                raise RuntimeError(snapshot["error"])

            sim_command, sim_actual = sim_worker.snapshot()
            if viewer is not None:
                with sim_worker.data_lock:
                    viewer.sync()

            fps = 1.0 / max(1e-6, now - previous)
            previous = now
            cv2.putText(image, f"FPS {fps:.0f}", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.putText(
                image,
                f"Real {snapshot['position']:.2f}  Cmd {snapshot['command_position']:.2f} rad  "
                f"Sim cmd/actual {sim_command:.3f}/{sim_actual:.3f} m",
                (10, image.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 255, 0),
                2,
            )
            cv2.imshow("Sim2Real Visuo-Tactile Gripper", image)
            cv2.imshow("Real FlexiTac 12x30", render_tactile(
                snapshot["matrix"], snapshot["force"], snapshot["peak"], snapshot["cells"]
            ))

            if sim_tactile is not None:
                with sim_worker.data_lock:
                    sim_touch_left = sim_tactile.read(data, "left")
                    sim_touch_right = sim_tactile.read(data, "right")
                cv2.imshow(
                    "MuJoCo Tactile 8x16",
                    render_sim_tactile(sim_touch_left, sim_touch_right),
                )

            if figure is not None and now - last_plot >= 1.0 / PLOT_RATE:
                elapsed = now - start
                for buffer, value in zip(buffers, (
                    elapsed,
                    snapshot["position"],
                    snapshot["command_position"],
                    snapshot["force"],
                    TARGET_FORCE,
                    snapshot["torque"],
                )):
                    buffer.append(value)
                for line, values in zip(lines, (
                    positions, commands, forces, force_targets, torques,
                )):
                    line.set_data(times, values)
                left, right = times[0], max(times[0] + 1.0, times[-1])
                for axis in axes:
                    axis.set_xlim(left, right)
                plot_image = render_figure(figure)
                last_plot = now
            if plot_image is not None:
                cv2.imshow("Motor & Tactile Dashboard", plot_image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        return 0

    except Exception as exc:
        print(f"运行错误: {exc}", file=sys.stderr)
        return 1
    finally:
        stop_event.set()
        if sim_worker is not None:
            sim_worker.join(timeout=2.0)
        if controller is not None:
            controller.join(timeout=4.0)
        if tactile.is_alive():
            tactile.join(timeout=1.5)
        if camera is not None:
            camera.release()
        if viewer is not None:
            viewer.close()
        cv2.destroyAllWindows()
        if figure is not None:
            plt.close(figure)
        print("电机已失能，触觉串口、仿真、摄像头和窗口已关闭。")


if __name__ == "__main__":
    sys.exit(main())
