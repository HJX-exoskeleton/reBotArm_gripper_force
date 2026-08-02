import time
import h5py
import argparse
import numpy as np
from tqdm import tqdm

from rustypot import Scs0009PyController

# ==========================================
# 解析命令行参数
# ==========================================
parser = argparse.ArgumentParser(description='SCS0009 Gripper Trajectory Replay')
parser.add_argument('--file', type=str, required=True,
                    help='要重播的 hdf5 文件路径 (例如: ./datasets/control_test/episode_0.hdf5)')
parser.add_argument('--speed', type=float, default=1.0, help='播放速度倍率 (默认 1.0, 2.0为两倍速, 0.5为慢放)')
args = parser.parse_args()

# 舵机配置
ID_1 = 1
ID_2 = 2

# 初始化控制器
c = Scs0009PyController(
    serial_port="/dev/ttyACM0",
    baudrate=115200,
    timeout=0.5,
)


def load_dataset(filepath):
    """从 HDF5 文件加载时间和位置数据"""
    print(f"📂 正在读取数据集: {filepath}")
    try:
        with h5py.File(filepath, 'r') as f:
            times = f['/time'][:]
            qpos = f['/observations/qpos'][:]
        return times, qpos
    except Exception as e:
        print(f"❌ 读取文件失败: {e}")
        exit(1)


def main():
    # 1. 加载数据
    times, qpos = load_dataset(args.file)
    total_frames = len(times)

    print("\n" + "=" * 40)
    print("▶️  准备开始轨迹重播")
    print(f"📊 总帧数: {total_frames}")
    print(f"⏱️  录制总时长: {times[-1]:.2f} 秒")
    print(f"⏩  播放倍速: {args.speed}x")
    print("=" * 40 + "\n")

    # 2. 舵机上电与初始化
    c.write_torque_enable(ID_1, 1)
    c.write_torque_enable(ID_2, 1)
    # 重播时为了尽可能贴合轨迹，建议将速度限制放开或设为最大
    c.write_goal_speed(ID_1, 0)  # 0 通常代表不受限/最快响应
    c.write_goal_speed(ID_2, 0)

    # 3. 初始复位：先让夹爪缓慢移动到录制的第一帧位置，防止突然抽搐
    print("正在移动至初始位置...")
    c.write_goal_position(ID_1, float(qpos[0][0]))
    c.write_goal_position(ID_2, float(qpos[0][1]))
    time.sleep(1.0)  # 等待机械臂/夹爪就位

    print("开始同步播放...")

    # 4. 核心重播循环
    start_play_time = time.time()

    try:
        # 使用 tqdm 显示重播进度条
        with tqdm(total=total_frames, desc="重播进度", unit="帧", dynamic_ncols=True) as pbar:
            for i in range(total_frames):
                # 获取当前帧的记录时间和目标位置
                recorded_time = times[i]
                target_pos_1 = float(qpos[i][0])
                target_pos_2 = float(qpos[i][1])

                # 【核心逻辑】：时间对齐控制 (考虑播放倍速)
                # 计算当前帧“应该”在什么时间点播放
                target_play_time = recorded_time / args.speed

                # 如果代码跑得比录制时间快，就在这里稍微等一下（自旋等待以保证极高精度）
                while (time.time() - start_play_time) < target_play_time:
                    # 使用 1 毫秒的微小休眠防止 CPU 100% 占用，同时保证精度
                    time.sleep(0.001)

                # 下发位置指令 (直接发送弧度)
                c.write_goal_position(ID_1, target_pos_1)
                c.write_goal_position(ID_2, target_pos_2)

                # 转换角度用于进度条显示
                deg_1 = np.rad2deg(target_pos_1)
                deg_2 = np.rad2deg(target_pos_2)

                # 更新进度条
                pbar.update(1)
                pbar.set_postfix({
                    'RealTime': f"{(time.time() - start_play_time):.2f}s",
                    'ID1': f"{deg_1:.1f}°",
                    'ID2': f"{deg_2:.1f}°"
                })

    except KeyboardInterrupt:
        print("\n⚠️ 重播被用户手动中止。")

    finally:
        print("\n正在释放舵机扭矩...")
        c.write_torque_enable(ID_1, 2)
        c.write_torque_enable(ID_2, 2)
        print("🎉 重播任务结束，程序安全退出。")


if __name__ == '__main__':
    main()

# 正常原速重播
# python replay.py --file ./datasets/control_test/episode_0.hdf5

# 两倍速极速重播
# python replay.py --file ./datasets/control_test/episode_0.hdf5 --speed 2.0
