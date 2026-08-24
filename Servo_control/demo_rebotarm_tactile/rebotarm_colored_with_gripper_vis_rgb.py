#!/usr/bin/env python3
import argparse
import logging
import multiprocessing as mp
import os
import queue
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import glfw
import mujoco
import mujoco.viewer
import numpy as np

# ============================================================
# 0. 动态环境变量注入 & SDK 路径修复
# ============================================================
current_dir = Path(__file__).resolve().parent
python_root = current_dir.parent.parent  # 向上两级：当前目录 -> Servo_control -> rebot_scripts
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
# 1. 硬件 & 映射配置 (Real2Sim 舵机参数)
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
# 2. 仿真与可视化基础配置
# ============================================================
XML_PATH_DEFAULT = "/home/hjx/hjx_file/rebot_devarm_ws/reBotArm_gripper_force/mujoco/assets_robot_xml/rebotarm_b601_colored/sim_rebotarm_colored_grasp.xml"

GRID_H, GRID_W = 16, 16
FORCE_MAX = 0.05
HEATMAP_DISPLAY_SIZE = 480

ENABLE_MUJOCO_VIEWER = True
ENABLE_TOP_CAMERA = True
CAMERA_NAME = "top"
ENABLE_WRIST_CAMERA = True
WRIST_CAMERA_NAME = "cam_wrist"
CAMERA_WIDTH, CAMERA_HEIGHT = 320, 240

QUEUE_MAXSIZE = 1
_TAXEL_GEOM_CACHE = {}


# ============================================================
# 3. 工具函数 (数学截断、单位转换与队列)
# ============================================================
def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))


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
    alpha = clamp(float(alpha), 0.0, 1.0)
    return alpha * target + (1.0 - alpha) * prev


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
# 4. 触觉 Sensor 工具 (单通道提取)
# ============================================================
def find_joint_actuators(model, joint_names):
    actuator_ids = []
    for joint_name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        found = False
        for i in range(model.nu):
            if model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT and model.actuator_trnid[i, 0] == joint_id:
                actuator_ids.append(i)
                found = True;
                break
        if not found: actuator_ids.append(-1)
    return actuator_ids


def find_sensors_by_keyword(model, keyword):
    return [i for i in range(model.nsensor) if
            keyword in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i) or "")]


def build_tactile_reader(model, keyword, grid_shape=(16, 16)):
    h, w = grid_shape
    sensor_ids = find_sensors_by_keyword(model, keyword)
    if len(sensor_ids) == h * w:
        dims = np.array([model.sensor_dim[sid] for sid in sensor_ids], dtype=np.int32)
        adrs = np.array([model.sensor_adr[sid] for sid in sensor_ids], dtype=np.int32)
        return {"mode": "grid_force", "keyword": keyword, "sensor_ids": sensor_ids, "adrs": adrs, "dims": dims,
                "grid_shape": grid_shape}
    if len(sensor_ids) == 1:
        sid = sensor_ids[0]
        return {"mode": "single_force", "keyword": keyword, "sensor_ids": sensor_ids, "adr": int(model.sensor_adr[sid]),
                "dim": int(model.sensor_dim[sid]), "grid_shape": grid_shape}
    raise RuntimeError(f"触觉 sensor 数量异常: keyword='{keyword}', found={len(sensor_ids)}")


def read_tactile(reader, data, force_max=1.0):
    mode, h, w = reader["mode"], reader["grid_shape"][0], reader["grid_shape"][1]
    if mode == "grid_force":
        adrs, dims = reader["adrs"], reader["dims"]
        if np.all(dims == 3):
            raw = data.sensordata[adrs[:, None] + np.array([0, 1, 2], dtype=np.int32)]
            return np.clip(np.linalg.norm(raw, axis=1), 0.0, force_max).reshape(h, w).astype(np.float32)
        values = np.zeros(h * w, dtype=np.float32)
        for k, adr in enumerate(adrs):
            dim = int(dims[k])
            raw = data.sensordata[adr:adr + dim]
            values[k] = np.clip(raw[0] if dim == 1 else np.linalg.norm(raw), 0.0, force_max)
        return values.reshape(h, w).astype(np.float32)
    elif mode == "single_force":
        raw = data.sensordata[reader["adr"]:reader["adr"] + reader["dim"]]
        return np.full((h, w), float(np.clip(raw[0] if reader["dim"] == 1 else np.linalg.norm(raw), 0.0, force_max)),
                       dtype=np.float32)
    return np.zeros((h, w), dtype=np.float32)


def disable_taxel_collisions(model):
    counts = {"left": 0, "right": 0}; backing = 0
    for gid in range(model.ngeom):
        gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                  int(model.geom_bodyid[gid])) or ""
        if gname in ("touch_base_left", "touch_base_right"):
            model.geom_contype[gid] = 0; model.geom_conaffinity[gid] = 0; backing += 1
        elif bname.startswith("touch_cell_left_"):
            model.geom_contype[gid] = 0; model.geom_conaffinity[gid] = 0; counts["left"] += 1
        elif bname.startswith("touch_cell_right_"):
            model.geom_contype[gid] = 0; model.geom_conaffinity[gid] = 0; counts["right"] += 1
    if counts["left"] not in (128, 256) or counts["right"] not in (128, 256):
        # Replicated cells have unnamed bodies; classify them by their
        # ancestor touch_pad body and 1 mm geometry size.
        counts = {"left": 0, "right": 0}
        for gid in range(model.ngeom):
            if float(np.max(model.geom_size[gid])) > 0.0011: continue
            root = int(model.geom_bodyid[gid])
            while root > 0:
                bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, root) or ""
                if bname in ("touch_pad_left", "touch_pad_right"):
                    model.geom_contype[gid] = 0; model.geom_conaffinity[gid] = 0
                    counts["left" if bname.endswith("left") else "right"] += 1
                    break
                root = int(model.body_parentid[root])
    if counts["left"] not in (128, 256) or counts["right"] not in (128, 256):
        raise RuntimeError(f"触觉 taxel geom 数量异常: {counts}")
    return counts, backing


def find_free_object_geom(model, data, grasp_site):
    candidates = []
    for gid in range(model.ngeom):
        body = int(model.geom_bodyid[gid]); root = body; free = False
        while root > 0:
            ja, jn = int(model.body_jntadr[root]), int(model.body_jntnum[root])
            if jn and np.any(model.jnt_type[ja:ja + jn] == mujoco.mjtJoint.mjJNT_FREE):
                free = True; break
            root = int(model.body_parentid[root])
        if free:
            candidates.append((float(np.linalg.norm(data.geom_xpos[gid] - data.site_xpos[grasp_site])), gid))
    if not candidates: raise RuntimeError("模型中没有找到 free-joint 抓取物体")
    return min(candidates, key=lambda x: x[0])[1]


def tactile_from_pad_proximity(model, data, obj_geom, side):
    cache_key = (id(model), side)
    if cache_key in _TAXEL_GEOM_CACHE:
        ids = _TAXEL_GEOM_CACHE[cache_key]
    else:
        ids = []
        prefix = f"touch_cell_{side}_"
        for gid in range(model.ngeom):
            body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid])) or ""
            if body.startswith(prefix): ids.append(gid)
        if len(ids) not in (128, 256):
            ids = []
            pad_name = f"touch_pad_{side}"
            for gid in range(model.ngeom):
                if float(np.max(model.geom_size[gid])) > 0.0011: continue
                root = int(model.geom_bodyid[gid])
                while root > 0:
                    bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, root) or ""
                    if bname == pad_name: ids.append(gid); break
                    root = int(model.body_parentid[root])
        mujoco.mj_forward(model, data)
        # The pad lies in the X-Z plane (Y is its normal). Sort by the two
        # in-plane axes, not world Y; sorting by Y scrambled the 16x16 map.
        ids.sort(key=lambda gid: (float(data.geom_xpos[gid, 0]), float(data.geom_xpos[gid, 2])))
        _TAXEL_GEOM_CACHE[cache_key] = ids
    # Older XMLs use MuJoCo <replicate>, which leaves cell bodies unnamed.
    # Recover those 1 mm cell geoms by walking to touch_pad_{side}.
    if len(ids) not in (128, 256):
        ids = []
        pad_name = f"touch_pad_{side}"
        for gid in range(model.ngeom):
            if float(np.max(model.geom_size[gid])) > 0.0011:
                continue
            bid = int(model.geom_bodyid[gid]); root = bid
            while root > 0:
                bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, root) or ""
                if bname == pad_name:
                    ids.append(gid); break
                root = int(model.body_parentid[root])
        mujoco.mj_forward(model, data)
        ids.sort(key=lambda gid: (float(data.geom_xpos[gid, 0]), float(data.geom_xpos[gid, 2])))
    if len(ids) not in (128, 256): return np.zeros((GRID_H, GRID_W), dtype=np.float32)
    dist = np.empty(len(ids), dtype=np.float32); fromto = np.empty(6, dtype=np.float64)
    for k, gid in enumerate(ids):
        dist[k] = mujoco.mj_geomDistance(model, data, int(gid), int(obj_geom), 0.02, fromto)
    sigma, reach = 0.00030, 0.00125
    values = np.exp(-np.square(np.maximum(dist, 0.0) / sigma)).astype(np.float32)
    values[dist > reach] = 0.0; values[values < 0.05] = 0.0
    if len(ids) == 128:
        # Backward-compatible 8x16 XMLs are expanded at the display
        # boundary so the public heatmap is always exactly 16x16.
        grid = np.repeat(values.reshape(8, 16), 2, axis=0)
    else:
        # ids are sorted with X as the outer axis and Z as the inner axis;
        # transpose so image rows follow vertical Z and columns follow X.
        grid = values.reshape(16, 16).T
    # Remove isolated taxel speckles caused by mesh-distance quantization,
    # without drawing any artificial lines into the image.
    return cv2.GaussianBlur(grid, (3, 3), 0)


def tactile_window(left, right):
    def as_16x16(a):
        a = np.nan_to_num(a).clip(0, 1)
        if a.shape == (8, 16):
            a = np.repeat(a, 2, axis=0)
        if a.shape != (16, 16):
            out = np.zeros((16, 16), dtype=np.float32)
            h, w = min(16, a.shape[0]), min(16, a.shape[1])
            out[:h, :w] = a[:h, :w]
            return out
        return a
    left = as_16x16(left)
    right = as_16x16(right)
    symmetric = 0.5 * (left + np.fliplr(right))
    def panel(a, label, mirror=False):
        im = cv2.applyColorMap((a * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
        im = cv2.resize(im, (160, 320), interpolation=cv2.INTER_NEAREST)
        if mirror: im = cv2.flip(im, 1)
        cv2.putText(im, label, (8, 24), cv2.FONT_HERSHEY_TRIPLEX, .62,
                    (255, 255, 255), 1, cv2.LINE_AA)
        return im
    out = np.hstack((panel(symmetric, "left"), panel(symmetric, "right", True)))
    return out


def tactile_square(a, label, mirror=False):
    """Render one independent 16x16 tactile panel as a square image."""
    a = np.nan_to_num(a).clip(0, 1)
    if a.shape == (8, 16):
        a = np.repeat(a, 2, axis=0)
    if a.shape != (16, 16):
        fixed = np.zeros((16, 16), dtype=np.float32)
        h, w = min(16, a.shape[0]), min(16, a.shape[1])
        fixed[:h, :w] = a[:h, :w]
        a = fixed
    im = cv2.applyColorMap((a * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    im = cv2.resize(im, (480, 480), interpolation=cv2.INTER_NEAREST)
    if mirror:
        im = cv2.flip(im, 1)
    cv2.putText(im, label, (12, 28), cv2.FONT_HERSHEY_TRIPLEX, .75,
                (255, 255, 255), 1, cv2.LINE_AA)
    return im


# ============================================================
# 5. 图像渲染
# ============================================================
def tactile_to_colormap(tactile, force_max=1.0, display_size=480):
    tactile_normalized = np.clip(tactile, 0.0, force_max) / force_max * 255.0
    colored = cv2.applyColorMap(tactile_normalized.astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    return cv2.resize(colored, (display_size, display_size), interpolation=cv2.INTER_NEAREST)


def init_offscreen_renderer(model, camera_name="top", width=320, height=240):
    if not glfw.init(): raise RuntimeError("GLFW 初始化失败")
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    window = glfw.create_window(width, height, "offscreen", None, None)
    glfw.make_context_current(window)
    cam = mujoco.MjvCamera()
    cam.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
    return window, cam, scene, context, mujoco.MjrRect(0, 0, width, height)


def make_fixed_camera(model, camera_name):
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cid < 0: return None
    cam = mujoco.MjvCamera(); cam.fixedcamid = cid; cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    return cam


def render_camera_view(model, data, window, cam, scene, context, viewport, width=320, height=240):
    glfw.make_context_current(window)
    mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(viewport, scene, context)
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    mujoco.mjr_readPixels(rgb, None, viewport, context)
    return cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR)


# ============================================================
# 6. 真实舵机读取线程
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
                if res == COMM_SUCCESS: arm_deg[i] = limit_real_deg(servo_id, (
                            float(pos) / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE)
            except:
                pass

        if not no_gripper:
            try:
                pos, _, res, _ = scs.ReadPosSpeed(GRIPPER_SERVO_ID)
                if res == COMM_SUCCESS: gripper_deg = limit_real_deg(GRIPPER_SERVO_ID, (
                            float(pos) / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE)
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
# 7. 仿真主进程 (包含物理计算与数据推送)
# ============================================================
def simulation_process(args, frame_queue: mp.Queue, stop_event: mp.Event):
    print("[SIM] 加载模型与初始化串口...")
    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    taxel_counts, backing_count = disable_taxel_collisions(model)
    grasp_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_center")
    tactile_object_geom = find_free_object_geom(model, data, grasp_site)

    joint_names = [x.strip() for x in args.joint_names.split(",")]
    arm_actuator_ids = find_joint_actuators(model, joint_names)
    gripper_act_id = -1 if args.no_gripper else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                                                  GRIPPER_ACTUATOR_NAME)

    gripper_ctrl_min = float(
        model.actuator_ctrlrange[gripper_act_id, 0]) if gripper_act_id >= 0 else GRIPPER_SIM_CLOSED_METER
    gripper_ctrl_max = float(
        model.actuator_ctrlrange[gripper_act_id, 1]) if gripper_act_id >= 0 else GRIPPER_SIM_OPEN_METER

    # 初始化串口
    portHandler = PortHandler(args.port)
    scs = sts(portHandler)
    if portHandler.openPort() and portHandler.setBaudRate(args.baudrate):
        print(f"[SIM] ✅ 串口通信已建立: {args.port}")
        time.sleep(2.0)
        if not args.keep_torque:
            for sid in ARM_SERVO_IDS + ([] if args.no_gripper else [GRIPPER_SERVO_ID]):
                scs.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 0)
    else:
        print("[SIM] ❌ 串口打开失败！");
        stop_event.set();
        return

    # 启动异步读取舵机线程
    state_lock = threading.Lock()
    shared_state = {
        "target_arm_q": np.zeros(ARM_DOF, dtype=np.float32),
        "target_gripper_cmd": float(gripper_ctrl_min)
    }
    threading.Thread(target=servo_reader_worker, args=(
        scs, state_lock, shared_state, args.read_rate, args.no_gripper, gripper_ctrl_min, gripper_ctrl_max, stop_event),
                     daemon=True).start()

    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False,
                                          show_right_ui=False) if ENABLE_MUJOCO_VIEWER else None
    offscreen = init_offscreen_renderer(model, CAMERA_NAME, CAMERA_WIDTH, CAMERA_HEIGHT) if ENABLE_TOP_CAMERA else None
    wrist_cam = make_fixed_camera(model, WRIST_CAMERA_NAME) if offscreen and ENABLE_WRIST_CAMERA else None

    # 控制步长与频率规划
    sim_period = 1.0 / max(args.rate, 1e-6)
    steps_per_loop = max(1, int(round(sim_period / float(model.opt.timestep))))

    vis_interval = max(1, int(round(args.rate / args.vis_hz)))
    viewer_interval = max(1, int(round(args.rate / args.viewer_hz)))
    camera_interval = max(1, int(round(args.rate / args.camera_hz)))

    filtered_arm_q = np.zeros(ARM_DOF, dtype=np.float32)
    filtered_gripper_cmd = gripper_ctrl_min
    loop_count = 0

    print(f"[SIM] 仿真已启动。 控制主频: {args.rate}Hz | mj_step 每次迭代: {steps_per_loop} | "
          f"distance tactile taxel={taxel_counts}, object_geom={tactile_object_geom}")
    try:
        while not stop_event.is_set():
            loop_start = time.perf_counter()
            if viewer and not viewer.is_running():
                stop_event.set();
                break

            # 1. 读取并平滑真实关节指令
            with state_lock:
                target_q = shared_state["target_arm_q"].copy()
                target_g = shared_state["target_gripper_cmd"]

            filtered_arm_q = smooth_update(filtered_arm_q, target_q, args.alpha_arm)
            filtered_gripper_cmd = smooth_update(filtered_gripper_cmd, target_g, args.alpha_gripper)

            # 2. 写入 data.ctrl 维持接触动力学
            for i, act_id in enumerate(arm_actuator_ids):
                if act_id >= 0: data.ctrl[act_id] = filtered_arm_q[i]
            if gripper_act_id >= 0: data.ctrl[gripper_act_id] = filtered_gripper_cmd

            # 3. 驱动物理引擎步进
            for _ in range(steps_per_loop):
                mujoco.mj_step(model, data)

            # 4. 提取触觉发给 OpenCV 可视化进程
            frame_packet = {}
            if loop_count % vis_interval == 0:
                frame_packet.update({
                    "touch_right": tactile_from_pad_proximity(model, data, tactile_object_geom, "right"),
                    "touch_left": tactile_from_pad_proximity(model, data, tactile_object_geom, "left")
                })

            if ENABLE_TOP_CAMERA and offscreen and loop_count % camera_interval == 0:
                win, cam, scn, ctx, vp = offscreen
                frame_packet["top_view"] = render_camera_view(model, data, win, cam, scn, ctx, vp, CAMERA_WIDTH, CAMERA_HEIGHT)
                if wrist_cam is not None:
                    frame_packet["wrist_view"] = render_camera_view(model, data, win, wrist_cam, scn, ctx, vp, CAMERA_WIDTH, CAMERA_HEIGHT)
            if frame_packet:
                put_latest(frame_queue, frame_packet)

            if viewer and loop_count % viewer_interval == 0: viewer.sync()

            # 5. 精确锁频机制 (替代第三方库)
            loop_count += 1
            sleep_time = sim_period - (time.perf_counter() - loop_start)
            if sleep_time > 0: time.sleep(sleep_time)

    finally:
        if viewer: viewer.close()
        if offscreen: glfw.destroy_window(offscreen[0]); glfw.terminate()
        portHandler.closePort()
        print("[SIM] 仿真进程完全停止。")


# ============================================================
# 8. 可视化进程 (专注绘制单通道热力图)
# ============================================================
def visualization_process(frame_queue: mp.Queue, stop_event: mp.Event):
    print("[VIS] 可视化进程启动。")
    cv2.namedWindow("Touch Heatmap - left", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Touch Heatmap - right", cv2.WINDOW_NORMAL)
    if ENABLE_TOP_CAMERA: cv2.namedWindow("Top Camera View", cv2.WINDOW_NORMAL)
    if ENABLE_WRIST_CAMERA: cv2.namedWindow("Wrist Camera View", cv2.WINDOW_NORMAL)

    last_top = None
    last_wrist = None

    while not stop_event.is_set():
        try:
            packet = get_latest(frame_queue, timeout=0.02)
        except queue.Empty:
            continue

        tr = packet.get("touch_right")
        tl = packet.get("touch_left")
        top = packet.get("top_view")
        wrist = packet.get("wrist_view")

        if tl is not None:
            cv2.imshow("Touch Heatmap - left", tactile_square(tl, "left"))
        if tr is not None:
            cv2.imshow("Touch Heatmap - right", tactile_square(tr, "right", mirror=True))
        if top is not None: last_top = top
        if ENABLE_TOP_CAMERA and last_top is not None: cv2.imshow("Top Camera View", last_top)
        if wrist is not None: last_wrist = wrist
        if ENABLE_WRIST_CAMERA and last_wrist is not None: cv2.imshow("Wrist Camera View", last_wrist)

        if cv2.waitKey(1) in [27, ord('q')]:
            stop_event.set();
            break

    cv2.destroyAllWindows()
    print("[VIS] 界面已关闭。")


# ============================================================
# 9. Main 入口与命令行参数解析
# ============================================================
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", type=str, default=XML_PATH_DEFAULT)
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)

    # 频率参数集合，默认值与你原代码内的宏观定义对齐
    parser.add_argument("--read-rate", type=float, default=60.0, help="串口轮询读取速度")
    parser.add_argument("--rate", type=float, default=100.0, help="仿真主循环写入控制信号的速度 (原SIM_HZ)")
    parser.add_argument("--vis-hz", type=float, default=20.0, help="OpenCV 热力图展示刷新率")
    parser.add_argument("--viewer-hz", type=float, default=30.0, help="MuJoCo Viewer 刷新率")
    parser.add_argument("--camera-hz", type=float, default=5.0, help="离屏相机渲染刷新率")

    # 滤波和配置参数
    parser.add_argument("--alpha-arm", type=float, default=0.90)
    parser.add_argument("--alpha-gripper", type=float, default=0.90)
    parser.add_argument("--joint-names", type=str, default="joint1,joint2,joint3,joint4,joint5,joint6")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--keep-torque", action="store_true")
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
