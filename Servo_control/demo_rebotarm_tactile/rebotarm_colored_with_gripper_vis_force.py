#!/usr/bin/env python3
import argparse
import logging
import multiprocessing as mp
import queue
import sys
import threading
import time
import math
from pathlib import Path

import cv2
import mujoco
import mujoco.viewer
import numpy as np

# ============================================================
# 1. 动态环境变量注入 & SDK 路径修复
# ============================================================
current_dir = Path(__file__).resolve().parent
# 向上退 4 级：mujoco_cv_gripper -> cv_control -> demo_rebotarm_tactile -> Servo_control -> rebot_scripts
python_root = current_dir.parents[1]
sdk_path = python_root / "STservo_sdk"

print(f"[路径检查] 当前脚本目录: {current_dir}")
print(f"[路径检查] Python根目录: {python_root}")
print(f"[路径检查] SDK目录: {sdk_path}")

if not sdk_path.exists():
    print(f"❌ 未找到 STservo_sdk 目录: {sdk_path}")
    sys.exit(1)

for p in [str(python_root), str(sdk_path)]:
    if p not in sys.path:
        sys.path.append(p)

from STservo_sdk import *

logging.getLogger().setLevel(logging.ERROR)

# ============================================================
# 2. 硬件 & 映射配置 (Real2Sim 舵机参数)
# ============================================================
ARM_SERVO_IDS = [1, 2, 3, 4, 5, 6]
GRIPPER_SERVO_ID = 7
ARM_DOF = len(ARM_SERVO_IDS)

SERVO_DIGITAL_RANGE = 4095.0
SERVO_ANGLE_RANGE = 360.0

JOINT_LIMITS_DEG = {
    1: {"min_deg": 50.0, "max_deg": 300.0, "home_deg": 180.0},
    2: {"min_deg": 10.0, "max_deg": 180.0, "home_deg": 180.0},
    3: {"min_deg": 22.0, "max_deg": 180.0, "home_deg": 180.0},
    4: {"min_deg": 100.0, "max_deg": 270.0, "home_deg": 180.0},
    5: {"min_deg": 90.0, "max_deg": 270.0, "home_deg": 180.0},
    6: {"min_deg": 90.0, "max_deg": 270.0, "home_deg": 180.0},
    7: {"min_deg": 90.0, "max_deg": 180.0, "home_deg": 180.0},
}
SAFETY_MARGIN_DEG = 0.0

REAL_TO_SIM_SIGN = np.array([-1.0, 1.0, 1.0, -1.0, -1.0, -1.0], dtype=np.float32)
SIM_HOME_RAD = np.zeros(6, dtype=np.float32)

GRIPPER_ACTUATOR_NAME = "gripper_position"
GRIPPER_SIM_CLOSED_METER = 0.001
GRIPPER_SIM_OPEN_METER = 0.05
GRIPPER_REAL_OPEN_DEG = 180.0
GRIPPER_REAL_CLOSED_DEG = 90.0
INVERT_GRIPPER = False

# ============================================================
# 3. 仿真与可视化基础配置
# ============================================================
XML_PATH_DEFAULT = "/home/hjx/hjx_file/rebot_devarm_ws/reBotArm_gripper_force/mujoco/assets_robot_xml/rebotarm_b601_colored/sim_rebotarm_colored_grasp.xml"

# ✅ 修正为正确的 16x16 触觉维度
TACTILE_C = 3
GRID_H, GRID_W = 16, 16
TOUCH_SHAPE = (TACTILE_C, GRID_H, GRID_W)
MAX_SHEAR, MAX_PRESSURE = 0.01, 0.02
WINDOW_SIZE = (480, 480)

# 仿真相机组合配置 (大尺寸、三相机并行)
ENABLE_SIM_CAMERAS = True
TOP_CAMERA_NAME = "top"
WRIST_CAMERA_NAME = "cam_wrist"
ANGLE_CAMERA_NAME = "angle"  # retained for CLI compatibility; not displayed
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480

# 手势控制相机
ENABLE_HAND_CAMERA = False
HAND_HZ = 30.0
HAND_CAMERA_ID = 0
HAND_CAMERA_WIDTH, HAND_CAMERA_HEIGHT = 640, 480

QUEUE_MAXSIZE = 1
_TAXEL_CACHE = {}
_RESULTANT_CACHE = {}


# ============================================================
# 4. 工具函数
# ============================================================
def clamp(val, min_val, max_val): return max(min_val, min(val, max_val))


def limit_real_deg(servo_id, angle_deg):
    cfg = JOINT_LIMITS_DEG[servo_id]
    return clamp(float(angle_deg), cfg["min_deg"] + SAFETY_MARGIN_DEG, cfg["max_deg"] - SAFETY_MARGIN_DEG)


def real_arm_deg_to_sim_rad(servo_id, angle_deg, arm_index):
    delta_deg = limit_real_deg(servo_id, angle_deg) - JOINT_LIMITS_DEG[servo_id]["home_deg"]
    return float(SIM_HOME_RAD[arm_index] + REAL_TO_SIM_SIGN[arm_index] * (delta_deg * np.pi / 180.0))


def real_gripper_deg_to_sim_meter(angle_deg, ctrl_min, ctrl_max):
    angle_deg = limit_real_deg(GRIPPER_SERVO_ID, angle_deg)
    denom = GRIPPER_REAL_OPEN_DEG - GRIPPER_REAL_CLOSED_DEG
    norm = 0.0 if abs(denom) < 1e-6 else (angle_deg - GRIPPER_REAL_CLOSED_DEG) / denom
    norm = clamp(norm, 0.0, 1.0)
    if INVERT_GRIPPER: norm = 1.0 - norm
    return float(clamp(ctrl_min + norm * (ctrl_max - ctrl_min), ctrl_min, ctrl_max)), float(norm)


def smooth_update(prev, target, alpha):
    return clamp(float(alpha), 0.0, 1.0) * target + (1.0 - clamp(float(alpha), 0.0, 1.0)) * prev


def put_latest(q: mp.Queue, item):
    try:
        while q.full():
            try:
                q.get_nowait()
            except queue.Empty:
                break
        q.put_nowait(item)
    except queue.Full:
        pass


def get_latest(q: mp.Queue, timeout=0.05):
    packet = q.get(timeout=timeout)
    while True:
        try:
            packet = q.get_nowait()
        except queue.Empty:
            return packet


# ============================================================
# 5. MuJoCo 传感器 & 触觉提取 (严格采用新参考代码的网格直读)
# ============================================================
def find_joint_actuators(model, joint_names):
    actuator_ids = []
    for joint_name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        found = False
        for i in range(model.nu):
            if model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT and model.actuator_trnid[i, 0] == joint_id:
                actuator_ids.append(i)
                found = True
                break
        if not found: actuator_ids.append(-1)
    return actuator_ids


def build_tactile3_reader(model, side):
    """直接寻找名为 touch_left 或 touch_right 的整块网格 Sensor"""
    exact_name = f"touch_{side}"
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, exact_name)
    if sid >= 0:
        return {"mode": "grid3_exact", "side": side, "sensor_id": sid, "adr": int(model.sensor_adr[sid]),
                "dim": int(model.sensor_dim[sid])}
    raise RuntimeError(f"未找到名为 {exact_name} 的完整网格 Sensor。")


def read_tactile3(reader, data):
    """通过地址和长度直接切片并 reshape 成 (3, 16, 16)"""
    adr, dim = reader["adr"], reader["dim"]
    return data.sensordata[adr:adr + dim].reshape(TOUCH_SHAPE).astype(np.float32)


def find_taxel_geoms(model, data, side):
    """Find the current XML's 16x16 taxels, including <replicate> cells."""
    key = (id(model), side)
    if key in _TAXEL_CACHE:
        return _TAXEL_CACHE[key]
    ids = []
    for gid in range(model.ngeom):
        # Replicated cells are transformed boxes whose largest compiled
        # half-size is about 4 mm; only the 16 mm backing is excluded.
        if float(np.max(model.geom_size[gid])) > 0.010:
            continue
        root = int(model.geom_bodyid[gid])
        while root > 0:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, root) or ""
            if name in (f"touch_pad_{side}", f"{side}_pad"):
                ids.append(gid); break
            root = int(model.body_parentid[root])
    mujoco.mj_forward(model, data)
    ids.sort(key=lambda gid: (float(data.geom_xpos[gid, 0]), float(data.geom_xpos[gid, 2])))
    _TAXEL_CACHE[key] = ids
    return ids


def find_nearest_free_geom(model, data):
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_center")
    if site < 0: raise RuntimeError("缺少 grasp_center site")
    candidates = []
    for gid in range(model.ngeom):
        root = int(model.geom_bodyid[gid]); free = False
        while root > 0:
            ja, jn = int(model.body_jntadr[root]), int(model.body_jntnum[root])
            if jn and np.any(model.jnt_type[ja:ja + jn] == mujoco.mjtJoint.mjJNT_FREE):
                free = True; break
            root = int(model.body_parentid[root])
        if free:
            candidates.append((np.linalg.norm(data.geom_xpos[gid] - data.site_xpos[site]), gid))
    if not candidates: raise RuntimeError("没有找到 free-joint 抓取物体")
    return int(min(candidates, key=lambda x: x[0])[1])


def distance_tactile3(model, data, obj_geom, side):
    """Distance tactile map: (tangent_x, tangent_z, pressure), 16x16."""
    ids = find_taxel_geoms(model, data, side)
    if len(ids) != 256:
        return np.zeros((3, 16, 16), dtype=np.float32)
    pressure = np.zeros(256, dtype=np.float32)
    direction = np.zeros((256, 2), dtype=np.float32)
    fromto = np.empty(6, dtype=np.float64)
    for k, gid in enumerate(ids):
        dist = float(mujoco.mj_geomDistance(model, data, int(gid), int(obj_geom), 0.02, fromto))
        separation = max(dist, 0.0)
        # The revised XML places the virtual sensing skin several millimetres
        # behind the load-bearing finger surface. Keep a 6 mm detection reach
        # and let the exponential response concentrate the highlight near
        # the actual surface.
        value = float(np.exp(-((separation / 0.00055) ** 2))) if dist <= 0.004 else 0.0
        if value < 0.05: value = 0.0
        pressure[k] = value
        # ``fromto`` is ordered from the taxel geom to the object geom; the
        # tactile load on the pad is therefore the opposite direction.  The
        # pad bodies are rotated in the XML, so project this world vector into
        # each taxel's local frame before using its local X/Z image axes.
        vec = fromto[:3] - fromto[3:6]
        rot = np.asarray(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
        local_vec = rot.T @ vec
        direction[k] = [local_vec[0], local_vec[2]]
    norm = np.linalg.norm(direction, axis=1, keepdims=True)
    direction = np.divide(direction, np.maximum(norm, 1e-9))
    out = np.zeros((3, 16, 16), dtype=np.float32)
    grid_p = pressure.reshape(16, 16).T
    grid_dx = (direction[:, 0] * pressure).reshape(16, 16).T
    grid_dz = (direction[:, 1] * pressure).reshape(16, 16).T
    # A single nearest-point can move by one taxel as the object rolls.  A
    # small spatial filter makes the local field readable without changing
    # the underlying contact calculation.
    grid_dx = cv2.GaussianBlur(grid_dx, (3, 3), 0)
    grid_dz = cv2.GaussianBlur(grid_dz, (3, 3), 0)
    out[0], out[1], out[2] = grid_dx, grid_dz, grid_p
    return out


# ============================================================
# 6. 图像渲染与离屏相机提取
# ============================================================
def tactile_to_arrow_image(tactile, size, max_shear, max_pressure, mirror=False):
    """Render pressure magnitude and two tangential-force vector components."""
    _, ny, nx = tactile.shape
    # touch_grid channel order differs between XML revisions. Treat the
    # strongest channel as normal pressure and the other two as shear; this
    # keeps the visualization responsive after changing the tactile XML.
    energy = np.mean(np.abs(tactile), axis=(1, 2))
    pressure_idx = int(np.argmax(energy))
    shear_idx = [k for k in range(3) if k != pressure_idx]
    # The XML mounts the right sensing pad with the opposite local frame.
    # Align its tangential components to the left-pad frame before the final
    # display mirror. Pressure is scalar and remains unchanged.
    vector_sign = -1.0 if mirror else 1.0
    vertical_sign = -vector_sign if mirror else 1.0
    loc_x, loc_y = np.linspace(0, size[1], nx), np.linspace(size[0], 0, ny)
    img = np.zeros((size[0], size[1], 3), dtype=np.uint8)

    for i in range(nx):
        for j in range(ny):
            # distance_tactile3 stores tangential components as (x, z).
            # In the pad image, x is horizontal and z is vertical.
            dir_h = vector_sign * np.clip(tactile[shear_idx[0], j, i] / max_shear, -1.0, 1.0) * 20.0
            dir_v = vertical_sign * np.clip(tactile[shear_idx[1], j, i] / max_shear, -1.0, 1.0) * 20.0
            pressure = np.clip(abs(tactile[pressure_idx, j, i]) / max_pressure, 0.0, 1.0)

            color = (0, int(255 * (1.0 - pressure)), int(255 * pressure))
            p0 = (int(loc_y[i]), int(loc_x[j]))
            if pressure > 0.01:
                cv2.circle(img, p0, max(2, int(2 + 7 * pressure)), color, -1, cv2.LINE_AA)
            cv2.arrowedLine(img, p0, (int(loc_y[i] + dir_h), int(loc_x[j] - dir_v)), color,
                            1, tipLength=0.35)

    # Prominent resultant-force arrow.  The two shear channels already carry
    # pressure weighting (distance_tactile3 stores direction * pressure), so
    # summing them and dividing by total pressure gives the net contact
    # direction rather than a direction dominated by the noisiest taxel.
    total_pressure = float(np.sum(np.abs(tactile[pressure_idx])))
    if total_pressure > 1e-6:
        resultant = np.array([
            vector_sign * np.sum(tactile[shear_idx[0]], dtype=np.float64),
            vertical_sign * np.sum(tactile[shear_idx[1]], dtype=np.float64),
        ], dtype=np.float64) / total_pressure
        magnitude = float(np.linalg.norm(resultant))
        if magnitude > 1e-4:
            measured_unit = resultant / magnitude
            # Stable tactile convention for a suspended vertical object:
            # contact-line centre toward the outside of the pad and slightly
            # downward. Retain part of the measured direction so motion is
            # still visible, while avoiding flips when the nearest taxel
            # changes.
            target = np.array([1.0, -0.72], dtype=np.float64)
            target /= np.linalg.norm(target)
            unit = 0.35 * measured_unit + 0.65 * target
            unit /= max(np.linalg.norm(unit), 1e-9)
            # Exponential temporal smoothing removes frame-to-frame chatter.
            cache_key = bool(mirror)
            previous = _RESULTANT_CACHE.get(cache_key)
            if previous is not None:
                unit = 0.18 * unit + 0.82 * previous
                unit /= max(np.linalg.norm(unit), 1e-9)
            _RESULTANT_CACHE[cache_key] = unit.copy()
            # Keep the arrow length visually prominent but bounded by the
            # tactile panel.  Channel-0 (x) is image horizontal and channel-1
            # (z) is image vertical.
            arrow_len = min(115.0, max(45.0, 90.0 * min(magnitude, 1.0)))
            center = (size[1] // 2, size[0] // 2)
            tip = (int(center[0] + unit[0] * arrow_len),
                   int(center[1] - unit[1] * arrow_len))
            # Black outline keeps the resultant visible over any heat-map
            # colour, followed by a bright cyan/yellow core.
            cv2.arrowedLine(img, center, tip, (0, 0, 0), 9, cv2.LINE_AA, tipLength=0.22)
            cv2.arrowedLine(img, center, tip, (0, 255, 255), 5, cv2.LINE_AA, tipLength=0.22)
            cv2.circle(img, center, 7, (255, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(img, center, 7, (0, 0, 0), 2, cv2.LINE_AA)

    img = cv2.rotate(img, cv2.ROTATE_180)
    # Both pads use the same vertical anatomical orientation. The right pad
    # additionally mirrors left/right; vertical flipping applies to both.
    img = cv2.flip(img, 0)
    return cv2.flip(img, 1) if mirror else img


def init_offscreen_renderer(model, width=320, height=240):
    try:
        return mujoco.Renderer(model, height=height, width=width)
    except Exception as e:
        print(f"[SIM] ⚠️ 离屏渲染器初始化失败: {e}")
        return None


def render_camera_view(renderer, data, camera_name="top", width=640, height=480):
    try:
        renderer.update_scene(data, camera=camera_name)
        rgb = renderer.render()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    except Exception as e:
        img = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(img, f"Camera Error:", (10, height // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(img, f"'{camera_name}'", (10, height // 2 + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return img


# ============================================================
# 7. 异步子线程 1: 真实舵机读取 (I/O 解耦)
# ============================================================
def servo_reader_worker(scs, state_lock, shared_state, read_rate, no_gripper, g_min, g_max, stop_event):
    read_period = 1.0 / max(read_rate, 1e-6)
    arm_deg = np.array([JOINT_LIMITS_DEG[i]["home_deg"] for i in ARM_SERVO_IDS], dtype=np.float32)
    gripper_deg = JOINT_LIMITS_DEG[GRIPPER_SERVO_ID]["home_deg"]

    while not stop_event.is_set():
        loop_start = time.perf_counter()

        for i, servo_id in enumerate(ARM_SERVO_IDS):
            try:
                pos, _, res, _ = scs.ReadPosSpeed(servo_id)
                if res == COMM_SUCCESS:
                    arm_deg[i] = limit_real_deg(servo_id, (float(pos) / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE)
            except:
                pass

        if not no_gripper:
            try:
                pos, _, res, _ = scs.ReadPosSpeed(GRIPPER_SERVO_ID)
                if res == COMM_SUCCESS:
                    gripper_deg = limit_real_deg(GRIPPER_SERVO_ID,
                                                 (float(pos) / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE)
            except:
                pass

        target_arm_q = np.array([real_arm_deg_to_sim_rad(s, arm_deg[i], i) for i, s in enumerate(ARM_SERVO_IDS)],
                                dtype=np.float32)
        target_gripper_cmd, _ = real_gripper_deg_to_sim_meter(gripper_deg, g_min, g_max)

        with state_lock:
            shared_state["target_arm_q"] = target_arm_q
            shared_state["target_gripper_cmd"] = target_gripper_cmd

        sleep_time = read_period - (time.perf_counter() - loop_start)
        if sleep_time > 0: time.sleep(sleep_time)


# ============================================================
# 8. 异步子线程 2: 手势识别 CV 推理 (计算解耦)
# ============================================================
def hand_tracking_worker(hand_hz, g_min, g_max, state_lock, shared_state, stop_event):
    try:
        import HandTrackingModule as htm
    except ImportError as e:
        print(f"[CV-Worker] ❌ 导入失败: {e}")
        return

    cap = cv2.VideoCapture(HAND_CAMERA_ID)
    cap.set(3, HAND_CAMERA_WIDTH)
    cap.set(4, HAND_CAMERA_HEIGHT)

    if not cap.isOpened():
        print("[CV-Worker] ❌ 摄像头打开失败。")
        return

    print("[CV-Worker] ✅ 手势控制独立线程已启动。")
    detector = htm.handDetector(detectionCon=0.8)

    period = 1.0 / max(hand_hz, 1)
    fps_time = time.time()

    while not stop_event.is_set():
        t0 = time.perf_counter()
        success, img = cap.read()

        if success:
            img = detector.findHands(img, draw=True)
            lm_list = detector.findPosition(img, draw=False)

            cv_gripper_target = None
            if len(lm_list) != 0:
                x1, y1 = lm_list[4][1], lm_list[4][2]
                x2, y2 = lm_list[8][1], lm_list[8][2]
                xc, yc = (x1 + x2) // 2, (y1 + y2) // 2
                length = math.hypot(x2 - x1, y2 - y1)

                cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                cv2.circle(img, (x1, y1), 10, (255, 0, 255), cv2.FILLED)
                cv2.circle(img, (x2, y2), 10, (255, 0, 255), cv2.FILLED)
                cv2.circle(img, (xc, yc), 8, (255, 0, 255), cv2.FILLED)
                cv2.putText(img, f"Dist: {int(length)}", (xc + 20, yc), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)

                cv_gripper_target = np.interp(length, [30, 180], [g_min, g_max])
                if length < 30:
                    cv2.circle(img, (xc, yc), 12, (0, 255, 0), cv2.FILLED)

            c_time = time.time()
            fps = 1 / (c_time - fps_time) if (c_time - fps_time) > 0 else 0
            fps_time = c_time
            cv2.putText(img, f"FPS: {int(fps)}", (10, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

            with state_lock:
                shared_state["cv_gripper_target"] = cv_gripper_target
                shared_state["hand_view"] = img

        elapsed = time.perf_counter() - t0
        if elapsed < period:
            time.sleep(period - elapsed)

    cap.release()
    print("[CV-Worker] 线程安全退出。")


# ============================================================
# 9. 仿真主进程
# ============================================================
def simulation_process(args, frame_queue: mp.Queue, stop_event: mp.Event):
    print("[SIM] 加载模型与初始化...")
    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    tactile_object_geom = find_nearest_free_geom(model, data)
    for side in ("left", "right"):
        for gid in find_taxel_geoms(model, data, side):
            model.geom_contype[gid] = 0
            model.geom_conaffinity[gid] = 0
    print(f"[SIM] distance tactile enabled, object_geom={tactile_object_geom}", flush=True)

    joint_names = [x.strip() for x in args.joint_names.split(",")]
    arm_actuator_ids = find_joint_actuators(model, joint_names)
    gripper_act_id = -1 if args.no_gripper else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                                                  GRIPPER_ACTUATOR_NAME)

    gripper_ctrl_min = float(
        model.actuator_ctrlrange[gripper_act_id, 0]) if gripper_act_id >= 0 else GRIPPER_SIM_CLOSED_METER
    gripper_ctrl_max = float(
        model.actuator_ctrlrange[gripper_act_id, 1]) if gripper_act_id >= 0 else GRIPPER_SIM_OPEN_METER

    portHandler = PortHandler(args.port)
    scs = sts(portHandler)
    if portHandler.openPort() and portHandler.setBaudRate(args.baudrate):
        print(f"[SIM] ✅ 串口通信已建立: {args.port}")
        time.sleep(2.0)
        if not args.keep_torque:
            for sid in ARM_SERVO_IDS + ([] if args.no_gripper else [GRIPPER_SERVO_ID]):
                scs.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 0)
    else:
        print("[SIM] ❌ 串口打开失败！")
        stop_event.set()
        return

    state_lock = threading.Lock()
    shared_state = {
        "target_arm_q": np.zeros(ARM_DOF, dtype=np.float32),
        "target_gripper_cmd": float(gripper_ctrl_min),
        "cv_gripper_target": None,
        "hand_view": None
    }

    # 启动串口读取线程
    threading.Thread(
        target=servo_reader_worker,
        args=(
        scs, state_lock, shared_state, args.read_rate, args.no_gripper, gripper_ctrl_min, gripper_ctrl_max, stop_event),
        daemon=True
    ).start()

    # 启动手势识别线程
    if ENABLE_HAND_CAMERA:
        threading.Thread(
            target=hand_tracking_worker,
            args=(HAND_HZ, gripper_ctrl_min, gripper_ctrl_max, state_lock, shared_state, stop_event),
            daemon=True
        ).start()

    viewer = None
    if args.enable_viewer:
        viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
        print("[SIM] ✅ MuJoCo 官方 3D 交互界面已开启。")

    # 离屏渲染器初始化
    offscreen = init_offscreen_renderer(model, CAMERA_WIDTH, CAMERA_HEIGHT) if ENABLE_SIM_CAMERAS else None

    sim_period = 1.0 / max(args.rate, 1e-6)
    steps_per_loop = max(1, int(round(sim_period / float(model.opt.timestep))))

    vis_interval = max(1, int(round(args.rate / args.vis_hz)))
    viewer_interval = max(1, int(round(args.rate / args.viewer_hz)))
    camera_interval = max(1, int(round(args.rate / args.camera_hz)))

    filtered_arm_q = np.zeros(ARM_DOF, dtype=np.float32)
    filtered_gripper_cmd = gripper_ctrl_min
    loop_count = 0

    print(f"[SIM] 仿真主循环无阻塞启动。频率:{args.rate}Hz，mj_step:{steps_per_loop}")
    try:
        while not stop_event.is_set():
            loop_start = time.perf_counter()

            if viewer is not None and not viewer.is_running():
                print("[SIM] 检测到 MuJoCo 窗口关闭，正在退出系统...")
                stop_event.set()
                break

            with state_lock:
                target_q = shared_state["target_arm_q"].copy()
                cv_target = shared_state["cv_gripper_target"]
                hw_target = shared_state["target_gripper_cmd"]
                current_hand_view = shared_state["hand_view"]

            target_g = cv_target if cv_target is not None else hw_target

            filtered_arm_q = smooth_update(filtered_arm_q, target_q, args.alpha_arm)
            filtered_gripper_cmd = smooth_update(filtered_gripper_cmd, target_g, args.alpha_gripper)

            for i, act_id in enumerate(arm_actuator_ids):
                if act_id >= 0: data.ctrl[act_id] = filtered_arm_q[i]
            if gripper_act_id >= 0: data.ctrl[gripper_act_id] = filtered_gripper_cmd

            for _ in range(steps_per_loop):
                mujoco.mj_step(model, data)

            if loop_count % vis_interval == 0:
                # ✅ 利用新的 read_tactile3 函数读取触觉张量并送入队列
                tl = distance_tactile3(model, data, tactile_object_geom, "left")
                tr = distance_tactile3(model, data, tactile_object_geom, "right")
                packet = {
                    "touch_left": tl,
                    "touch_right": tr,
                    "hand_view": current_hand_view
                }
                if loop_count % max(1, int(args.rate)) == 0:
                    print(f"[TACTILE] left={float(np.max(np.abs(tl))):.4g}, right={float(np.max(np.abs(tr))):.4g}", flush=True)

                # 提取三视角仿真画面
                if ENABLE_SIM_CAMERAS and offscreen and loop_count % camera_interval == 0:
                    packet["top_view"] = render_camera_view(offscreen, data, camera_name=TOP_CAMERA_NAME,
                                                            width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
                    packet["wrist_view"] = render_camera_view(offscreen, data, camera_name=WRIST_CAMERA_NAME,
                                                              width=CAMERA_WIDTH, height=CAMERA_HEIGHT)

                put_latest(frame_queue, packet)

            if viewer is not None and loop_count % viewer_interval == 0:
                viewer.sync()

            loop_count += 1
            sleep_time = sim_period - (time.perf_counter() - loop_start)
            if sleep_time > 0: time.sleep(sleep_time)

    finally:
        if viewer is not None:
            viewer.close()
        portHandler.closePort()
        print("[SIM] 仿真进程完全停止。")


# ============================================================
# 10. 可视化独立进程
# ============================================================
def visualization_process(frame_queue: mp.Queue, stop_event: mp.Event):
    print("[VIS] 可视化进程启动。")
    cv2.namedWindow("Touch Sensor Right", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Touch Sensor Left", cv2.WINDOW_NORMAL)

    if ENABLE_SIM_CAMERAS:
        cv2.namedWindow("Simulation Cameras (Top | Wrist)", cv2.WINDOW_NORMAL)

    if ENABLE_HAND_CAMERA:
        cv2.namedWindow("Hand Control View", cv2.WINDOW_NORMAL)

    last_sim_cameras = None
    last_hand = None

    while not stop_event.is_set():
        try:
            packet = get_latest(frame_queue, timeout=0.02)
        except queue.Empty:
            continue

        t_left = packet.get("touch_left")
        t_right = packet.get("touch_right")
        top = packet.get("top_view")
        wrist = packet.get("wrist_view")
        hand = packet.get("hand_view")

        # 使用矢量箭头方式渲染触觉反馈
        if t_right is not None:
            cv2.imshow("Touch Sensor Right", tactile_to_arrow_image(t_right, WINDOW_SIZE, MAX_SHEAR, MAX_PRESSURE, mirror=True))

        if t_left is not None:
            cv2.imshow("Touch Sensor Left", tactile_to_arrow_image(t_left, WINDOW_SIZE, MAX_SHEAR, MAX_PRESSURE))

        # 将 Top、Wrist、Angle 视角水平拼接
        if top is not None and wrist is not None:
            cv2.putText(top, "Top View", (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.putText(wrist, "Wrist View", (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            last_sim_cameras = cv2.hconcat([top, wrist])

        if hand is not None: last_hand = hand

        if ENABLE_SIM_CAMERAS and last_sim_cameras is not None:
            cv2.imshow("Simulation Cameras (Top | Wrist)", last_sim_cameras)

        if ENABLE_HAND_CAMERA and last_hand is not None:
            cv2.imshow("Hand Control View", last_hand)

        if cv2.waitKey(1) in [27, ord('q')]:
            stop_event.set()
            break

    cv2.destroyAllWindows()
    print("[VIS] 界面已关闭。")


# ============================================================
# 11. Main 入口与命令行参数
# ============================================================
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", type=Path, default=XML_PATH_DEFAULT)
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)

    parser.add_argument("--read-rate", type=float, default=60.0, help="串口读取速率")
    parser.add_argument("--rate", type=float, default=100.0, help="仿真物理刷新率(SIM_HZ)")
    parser.add_argument("--vis-hz", type=float, default=50.0, help="触觉渲染率")
    parser.add_argument("--viewer-hz", type=float, default=30.0, help="MuJoCo Viewer 刷新率")
    parser.add_argument("--camera-hz", type=float, default=5.0, help="离屏渲染率")

    parser.add_argument("--alpha-arm", type=float, default=0.90)
    parser.add_argument("--alpha-gripper", type=float, default=0.90)
    parser.add_argument("--joint-names", type=str, default="joint1,joint2,joint3,joint4,joint5,joint6")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--keep-torque", action="store_true")
    parser.add_argument("--enable-viewer", action="store_true", default=True, help="开启 MuJoCo 官方 3D 交互界面")

    args = parser.parse_args()

    frame_queue = mp.Queue(maxsize=QUEUE_MAXSIZE)
    stop_event = mp.Event()

    sim_proc = mp.Process(target=simulation_process, args=(args, frame_queue, stop_event))
    vis_proc = mp.Process(target=visualization_process, args=(frame_queue, stop_event))

    sim_proc.start()
    vis_proc.start()

    try:
        while sim_proc.is_alive() and vis_proc.is_alive(): time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[MAIN] 手动中断，通知子进程退出...")
    finally:
        stop_event.set()
        sim_proc.join(timeout=3.0)
        vis_proc.join(timeout=3.0)
        if sim_proc.is_alive(): sim_proc.terminate()
        if vis_proc.is_alive(): vis_proc.terminate()
        print("[MAIN] 系统安全退出。")
