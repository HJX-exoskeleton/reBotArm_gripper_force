import time
import numpy as np
import matplotlib.pyplot as plt
import h5py
import os
import argparse

from rustypot import Scs0009PyController
from tqdm import tqdm

# ==========================================
# 1. 解析命令行参数
# ==========================================
parser = argparse.ArgumentParser(description='SCS0009 Gripper Data Collection')
parser.add_argument('--task_name', action='store', type=str, help='任务名称 (如: control_test)', required=True)
parser.add_argument('--dataset_dir', action='store', type=str, help='数据集基础保存目录', required=True)
parser.add_argument('--episode_len', action='store', type=int, default=500, help='最大数据步数')
parser.add_argument('--dt', action='store', type=float, default=0.02, help='控制时间步长 (s)')
parser.add_argument('--fps', action='store', type=int, default=50, help='界面图表刷新帧率')
args = parser.parse_args()

# 舵机配置
ID_1 = 1
ID_2 = 2
MiddlePos_1 = 0
MiddlePos_2 = 0

c = Scs0009PyController(
    serial_port="/dev/ttyACM0",
    baudrate=115200,
    timeout=0.5,
)

# --- 全局绘图变量初始化 ---
plt.ion()
fig, ax = plt.subplots(figsize=(10, 5))
line1, = ax.plot([], [], label='Servo 1 (Right)', color='blue')
line2, = ax.plot([], [], label='Servo 2 (Left)', color='red')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (Degrees)')
ax.set_title(f'Gripper Real-time Position | Task: {args.task_name}')
ax.legend()
ax.grid(True)

time_data, pos1_data, pos2_data = [], [], []
start_time = time.time()
MAX_POINTS = 150

# 数据字典
data_dict = {
    '/time': [],
    '/observations/qpos': [],
}

# 全局采集进度条实例
collection_pbar = None


def wait_and_plot(duration):
    """
    等待并收集数据。
    返回 True 表示正常结束；返回 False 表示达到了 episode_len 需要停止采集。
    """
    global collection_pbar
    end_time = time.time() + duration
    plot_interval = 1.0 / args.fps if args.fps > 0 else args.dt
    last_plot_time = 0

    while time.time() < end_time:
        try:
            p1_raw = c.read_present_position(ID_1)
            p2_raw = c.read_present_position(ID_2)

            if p1_raw is not None and p2_raw is not None:
                # 优化：更鲁棒的数据提取方式
                p1_val = p1_raw[0] if isinstance(p1_raw, (list, tuple, np.ndarray)) else p1_raw
                p2_val = p2_raw[0] if isinstance(p2_raw, (list, tuple, np.ndarray)) else p2_raw

                p1_rad = float(p1_val)
                p2_rad = float(p2_val)

                current_time = time.time() - start_time

                # --- 保存数据到字典 ---
                data_dict['/time'].append(current_time)
                data_dict['/observations/qpos'].append([p1_rad, p2_rad])

                p1_deg = float(np.rad2deg(p1_rad))
                p2_deg = float(np.rad2deg(p2_rad))

                # --- 进度条更新机制 ---
                if collection_pbar is not None:
                    collection_pbar.update(1)
                    collection_pbar.set_postfix({
                        'Time': f"{current_time:.1f}s",
                        'ID1': f"{p1_deg:.1f}°",
                        'ID2': f"{p2_deg:.1f}°"
                    })

                # --- 检查是否达到最大步数 ---
                if len(data_dict['/time']) >= args.episode_len:
                    tqdm.write(f"\n[提示] 已达到最大采集步数 ({args.episode_len})，自动停止采集。")
                    return False

                # --- 图表数据更新 ---
                time_data.append(current_time)
                pos1_data.append(p1_deg)
                pos2_data.append(p2_deg)

                if len(time_data) > MAX_POINTS:
                    time_data.pop(0)
                    pos1_data.pop(0)
                    pos2_data.pop(0)

                # --- 控制图表刷新频率 ---
                if time.time() - last_plot_time >= plot_interval:
                    line1.set_xdata(time_data)
                    line1.set_ydata(pos1_data)
                    line2.set_xdata(time_data)
                    line2.set_ydata(pos2_data)

                    ax.relim()
                    ax.autoscale_view()
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    last_plot_time = time.time()

            else:
                tqdm.write(f"⚠️ [通信警告] 串口无有效返回 -> ID1: {p1_raw}, ID2: {p2_raw}")

        except Exception as e:
            tqdm.write(f"❌ [严重错误] {e}")

        time.sleep(args.dt)

    return True


def save_hdf5_dataset():
    """
    保存数据集至指定目录，包含独立的写入进度条和数据压缩
    """
    if len(data_dict['/time']) == 0:
        print("\n⚠️ 警告：没有收集到任何数据，跳过保存。")
        return

    save_dir = os.path.join(args.dataset_dir, args.task_name)
    os.makedirs(save_dir, exist_ok=True)

    episode_idx = 0
    while os.path.exists(os.path.join(save_dir, f"episode_{episode_idx}.hdf5")):
        episode_idx += 1

    filename = os.path.join(save_dir, f"episode_{episode_idx}.hdf5")
    print(f"\n正在将轨迹数据保存至: {filename}")

    keys_to_save = ['/time', '/observations/qpos']
    with h5py.File(filename, 'w') as f:
        for key in tqdm(keys_to_save, desc="写入HDF5", unit="模块", leave=False):
            # 优化：启用 gzip 压缩，极大减小文件体积
            f.create_dataset(
                key,
                data=np.array(data_dict[key], dtype=np.float32),
                compression="gzip",
                compression_opts=4
            )
            time.sleep(0.1)

    print(f"\n✅ 保存成功！共记录了 {len(data_dict['/time'])} 帧数据。")
    print(f"📊 数据集形状 - time: {len(data_dict['/time'])}, qpos: ({len(data_dict['/observations/qpos'])}, 2)")


def main():
    global collection_pbar

    print("\n" + "=" * 40)
    print(f"🚀 开始采集任务: {args.task_name}")
    print(f"⚙️  参数: dt={args.dt}s, fps={args.fps}, max_steps={args.episode_len}")
    print("🛑 按 Ctrl+C 可提前中断并安全保存")
    print("=" * 40 + "\n")

    c.write_torque_enable(1, 1)
    c.write_torque_enable(2, 1)

    collection_pbar = tqdm(total=args.episode_len, desc="采集进度", unit="step", dynamic_ncols=True)

    try:
        while True:
            CloseFinger()
            if not wait_and_plot(3): break

            OpenFinger()
            if not wait_and_plot(1): break

    except KeyboardInterrupt:
        tqdm.write("\n⚠️ 程序被用户手动中断，准备结束采集并保存数据...")

    finally:
        if collection_pbar is not None:
            collection_pbar.close()

        print("正在释放舵机扭矩...")
        c.write_torque_enable(1, 2)
        c.write_torque_enable(2, 2)

        save_hdf5_dataset()

        # 优化：优雅关闭所有图表窗口，避免卡死和 Exit code 137
        plt.ioff()
        plt.close('all')
        print("🎉 所有任务完成，程序安全退出。\n")


def CloseFinger():
    c.write_goal_speed(ID_1, 6)
    c.write_goal_speed(ID_2, 6)
    c.write_goal_position(ID_1, np.deg2rad(MiddlePos_1 + 90))
    c.write_goal_position(ID_2, np.deg2rad(MiddlePos_2 - 90))


def OpenFinger():
    c.write_goal_speed(ID_1, 6)
    c.write_goal_speed(ID_2, 6)
    c.write_goal_position(ID_1, np.deg2rad(MiddlePos_1 - 30))
    c.write_goal_position(ID_2, np.deg2rad(MiddlePos_2 + 30))


if __name__ == '__main__':
    main()

# python your_script.py --task_name control_test --dataset_dir ./datasets --episode_len 100

