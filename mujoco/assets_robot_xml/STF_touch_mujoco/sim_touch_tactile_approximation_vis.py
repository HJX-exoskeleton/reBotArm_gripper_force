import time
import mujoco
import mujoco.viewer
import cv2
import numpy as np

GRID = 16
# The virtual taxel is 20 mm pitch with a 20 mm square footprint.  A broad
# Gaussian makes a line contact look like a thick band, so keep the sensing
# shell narrow and discard its low-level tail.
DETECT_RANGE = 0.006
SIGMA = 0.0010


def free_geom_ids(model):
    ids = []
    for gid in range(model.ngeom):
        root = int(model.geom_bodyid[gid])
        while root > 0:
            ja, jn = int(model.body_jntadr[root]), int(model.body_jntnum[root])
            if jn and np.any(model.jnt_type[ja:ja + jn] == mujoco.mjtJoint.mjJNT_FREE):
                ids.append(gid); break
            root = int(model.body_parentid[root])
    return ids


def taxel_ids(model, data):
    ids = []
    pad = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "sim_touch_pad_stf")
    for gid in range(model.ngeom):
        root = int(model.geom_bodyid[gid]); under_pad = False
        while root > 0:
            if root == pad: under_pad = True; break
            root = int(model.body_parentid[root])
        if under_pad and model.geom_contype[gid] == 0 and model.geom_size[gid, 2] < 0.005:
            ids.append(gid)
    ids.sort(key=lambda g: (float(data.geom_xpos[g, 1]), float(data.geom_xpos[g, 0])))
    if len(ids) != GRID * GRID:
        raise RuntimeError(f"expected 256 virtual taxels, found {len(ids)}")
    return ids


def distance_touch(model, data, taxels, object_geoms):
    """Return a 16x16 distance-based tactile pressure map."""
    values = np.zeros(GRID * GRID, dtype=np.float32)
    fromto = np.empty(6, dtype=np.float64)
    for k, tg in enumerate(taxels):
        nearest = DETECT_RANGE
        for og in object_geoms:
            dist = float(mujoco.mj_geomDistance(model, data, int(tg), int(og), DETECT_RANGE, fromto))
            nearest = min(nearest, max(dist, 0.0))
        # Traditional distance tactile response: closest is brightest and
        # the signal decays smoothly with the surface separation.
        values[k] = np.exp(-((nearest / SIGMA) ** 2)) if nearest < DETECT_RANGE else 0.0
    return values.reshape(GRID, GRID)

# 加载模型
m = mujoco.MjModel.from_xml_path('/home/hjx/hjx_file/rebot_devarm_ws/reBotArm_gripper_force/mujoco/assets_robot_xml/STF_touch_mujoco/xml/sim_touch_tactile_approximation_stf.xml')
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
taxels = taxel_ids(m, d)
object_geoms = free_geom_ids(m)
print(f"virtual taxels={len(taxels)}, object geoms={len(object_geoms)}")

# 启动渲染器并进入仿真循环
with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(m, d)

        touch_stf = distance_touch(m, d, taxels, object_geoms)

        # 将触觉传感器数据标准化到0到255之间
        touch_stf_normalized = np.clip(touch_stf, 0, 1) * 255

        # 使用OpenCV的COLORMAP_VIRIDIS来创建热力图效果
        touch_colored_stf = cv2.applyColorMap(touch_stf_normalized.astype(np.uint8), cv2.COLORMAP_VIRIDIS)

        # 调整热力图大小
        touch_colored_stf_resized = cv2.resize(touch_colored_stf, (480, 480))

        # 分别显示两个热力图在不同的窗口
        cv2.imshow("Touch Heatmap - Sensor", touch_colored_stf_resized)

        cv2.waitKey(1)

        # 同步仿真和渲染状态
        viewer.sync()

        # 简单的时间控制，保持与物理仿真步长同步
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
