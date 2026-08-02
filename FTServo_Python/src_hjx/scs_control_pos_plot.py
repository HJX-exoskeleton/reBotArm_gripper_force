import time
import numpy as np
import matplotlib.pyplot as plt

from rustypot import Scs0009PyController

ID_1 = 1
ID_2 = 2
MiddlePos_1 = 0
MiddlePos_2 = 0

c = Scs0009PyController(
    serial_port="/dev/ttyACM0",
    baudrate=115200,
    timeout=0.5,
)

# --- 绘图初始化 ---
plt.ion()
fig, ax = plt.subplots(figsize=(10, 5))
line1, = ax.plot([], [], label='Servo 1 (Right)', color='blue')
line2, = ax.plot([], [], label='Servo 2 (Left)', color='red')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (Degrees)')
ax.set_title('Gripper Real-time Position')
ax.legend()
ax.grid(True)

time_data, pos1_data, pos2_data = [], [], []
start_time = time.time()
MAX_POINTS = 150


def wait_and_plot(duration):
    end_time = time.time() + duration
    while time.time() < end_time:
        try:
            p1_rad = c.read_present_position(ID_1)
            p2_rad = c.read_present_position(ID_2)

            if p1_rad is None or p2_rad is None:
                print(f"⚠️ [警告] 无法读取数据! ID_1: {p1_rad}, ID_2: {p2_rad}")
            else:
                current_time = time.time() - start_time

                # 【关键修复】：使用 float() 将 numpy 数组强制转换为纯数字
                p1_deg = float(np.rad2deg(p1_rad))
                p2_deg = float(np.rad2deg(p2_rad))

                # 现在这里不会再报错了
                print(f"✅ Time: {current_time:.2f}s | ID_1 Pos: {p1_deg:.2f}° | ID_2 Pos: {p2_deg:.2f}°")

                time_data.append(current_time)
                pos1_data.append(p1_deg)
                pos2_data.append(p2_deg)

                if len(time_data) > MAX_POINTS:
                    time_data.pop(0)
                    pos1_data.pop(0)
                    pos2_data.pop(0)

                line1.set_xdata(time_data)
                line1.set_ydata(pos1_data)
                line2.set_xdata(time_data)
                line2.set_ydata(pos2_data)

                ax.relim()
                ax.autoscale_view()

        except Exception as e:
            print(f"❌ [严重错误] 通信或代码异常: {e}")

        plt.pause(0.05)

def main():
    print("正在初始化舵机扭矩...")
    c.write_torque_enable(1, 1)
    c.write_torque_enable(2, 1)

    print("开始运行！请观察终端是否有报错信息...")
    try:
        while True:
            CloseFinger()
            wait_and_plot(3)

            OpenFinger()
            wait_and_plot(1)

    except KeyboardInterrupt:
        print("\n程序手动中断。")
        c.write_torque_enable(1, 2)
        c.write_torque_enable(2, 2)
        plt.ioff()
        plt.show()


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