import h5py
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os


def main():
    # ==========================================
    # 解析命令行参数
    # ==========================================
    parser = argparse.ArgumentParser(description='SCS0009 Dataset Offline Visualization')
    parser.add_argument('--file', type=str, required=True, help='要可视化的 hdf5 文件路径')
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"❌ 错误: 找不到文件 {args.file}")
        return

    print(f"📂 正在加载数据集: {args.file}")

    try:
        with h5py.File(args.file, 'r') as f:
            # 提取时间戳和位置矩阵
            time_data = f['/time'][:]
            qpos_data = f['/observations/qpos'][:]

        print(f"✅ 数据加载成功！共包含 {len(time_data)} 帧。")

    except Exception as e:
        print(f"❌ 读取文件失败: {e}")
        return

    # ==========================================
    # 数据处理：将弧度 (rad) 转换为角度 (deg)
    # qpos_data 的形状为 (N, 2)，其中 N 是帧数
    # ==========================================
    pos1_deg = np.rad2deg(qpos_data[:, 0])
    pos2_deg = np.rad2deg(qpos_data[:, 1])

    # ==========================================
    # 开始绘图
    # ==========================================
    print("📈 正在生成图表...")

    # 设置图表大小和清晰度
    plt.figure(figsize=(12, 6), dpi=100)

    # 绘制两条曲线
    plt.plot(time_data, pos1_deg, label='Servo 1', color='#1f77b4', linewidth=2)
    plt.plot(time_data, pos2_deg, label='Servo 2', color='#ff7f0e', linewidth=2)

    # 设置图表细节
    filename = os.path.basename(args.file)
    plt.title(f'Trajectory Offline Visualization: {filename}', fontsize=15, fontweight='bold')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Position (Degrees)', fontsize=12)

    # 优化网格显示
    plt.grid(True, linestyle='--', alpha=0.7)

    # 坐标轴留白优化
    plt.xlim([time_data[0], time_data[-1]])

    plt.legend(fontsize=11, loc='upper right')
    plt.tight_layout()  # 自动调整布局，防止标签被遮挡

    # 显示图表（这会弹出一个窗口，并阻塞直到你关闭它）
    plt.show()


if __name__ == '__main__':
    main()

# python plot_dataset.py --file ./datasets/control_test/episode_0.hdf5
