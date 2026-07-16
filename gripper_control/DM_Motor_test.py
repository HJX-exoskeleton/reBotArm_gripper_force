import math  # 导入数学库，用于生成正弦波等数学运算
from DM_CAN import *  # 导入DM_CAN库中的所有内容，该库用于控制达妙电机
import serial  # 导入串口通信库
import time  # 导入时间操作库

# 创建电机对象
# 参数:
#   DM_Motor_Type.DM4310 - 电机型号为DM4310
#   0x01 - 电机的节点ID(CAN总线地址)
#   0x11 - 主站ID(控制器ID)
Motor1 = Motor(DM_Motor_Type.DM4310, 0x07, 0x17)

# 初始化串口连接
# 参数:
#   'COM9' - 串口号，根据实际连接修改
#   921600 - 波特率
#   timeout=0.5 - 读超时时间设置为0.5秒
serial_device = serial.Serial('/dev/ttyACM0', 921600, timeout=0.5)

# 创建电机控制器对象，绑定到刚才创建的串口设备
MotorControl1 = MotorControl(serial_device)

# 将电机添加到控制器，建立关联关系
MotorControl1.addMotor(Motor1)

# 切换电机的控制模式到位置-速度模式(POS_VEL)
# 该模式下可以同时控制电机的位置和速度
# if条件判断切换是否成功
# if MotorControl1.switchControlMode(Motor1, Control_Type.POS_VEL):
#     print("switch POS_VEL success")  # 打印成功信息

if MotorControl1.switchControlMode(Motor1, Control_Type.Torque_Pos):
    print("switch Torque_Pos success")  # 打印成功信息

# 读取并打印电机的子版本号参数
print("sub_ver:", MotorControl1.read_motor_param(Motor1, DM_variable.sub_ver))

# 读取并打印电机的减速比参数
print("Gr:", MotorControl1.read_motor_param(Motor1, DM_variable.Gr))

# 下面的代码块已被注释掉，功能是修改位置环比例增益参数
# if MotorControl1.change_motor_param(Motor1,DM_variable.KP_APR,54):
#     print("write success")

# 读取并打印电机的最大位置限制参数
print("PMAX:", MotorControl1.read_motor_param(Motor1, DM_variable.PMAX))

# 读取并打印电机的主站ID参数
print("MST_ID:", MotorControl1.read_motor_param(Motor1, DM_variable.MST_ID))

# 读取并打印电机的最大速度限制参数
print("VMAX:", MotorControl1.read_motor_param(Motor1, DM_variable.VMAX))

# 读取并打印电机的最大扭矩限制参数
print("TMAX:", MotorControl1.read_motor_param(Motor1, DM_variable.TMAX))

# 保存电机的当前参数到Flash存储器
# 确保断电后参数不会丢失
MotorControl1.save_motor_param(Motor1)

# 使能电机(解锁电机)，使电机可以响应控制指令
MotorControl1.enable(Motor1)

# 初始化循环计数器
i = 0

# 主控制循环，执行10000次
while i < 10000:
    # 生成一个基于当前时间的正弦波值，范围在-1到1之间

    # 循环计数器增加
    i = i + 1

    # 以下是三种控制方式：
    # 1. 位置-力控制模式
    MotorControl1.control_pos_force(Motor1, -2, 5000, 300)  # 0： 闭合 ， -5.8 张开

    # 控制循环延迟，实现1ms的控制周期
    time.sleep(0.001)

# 程序结束时关闭串口连接
# 释放系统资源
serial_device.close()
