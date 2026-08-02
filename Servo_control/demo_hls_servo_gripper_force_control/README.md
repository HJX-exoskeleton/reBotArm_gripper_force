# HLS3606 舵机遥操作达妙夹爪

使用 HLS3606 飞特舵机 (ID=7) 作为主手旋钮，遥操作控制达妙电机夹爪开合。

## 硬件连接

```
/dev/ttyACM1  ←→  HLS3606 舵机 (scservo_sdk, ID=7)    — 主手旋钮
/dev/ttyACM0  ←→  达妙电机夹爪 (motorbridge, 0x07)    — 从手执行
```

## 脚本

| 脚本 | 功能 | 力反馈 |
|------|------|--------|
| `hls_teleop_gripper.py` | 基础遥操作: 转动舵机→夹爪开合 | ❌ |
| `hls_teleop_gripper_force.py` | 力反馈遥操作: 夹爪受力→舵机变硬+锁闭 | ✅ |

## 架构

```
主线程                        HardwareWorker 线程
───────                       ──────────────────
读 HLS 舵机位置                Gripper POS_VEL 控制
  ↓                             ↓
映射为夹爪 rad                  pos_vel(target, vlim)
  ↓                             ↓
hw.set_target(rad) ──────────→ get_state(request=False)
  ↓                             ↓
hw.snapshot() ←────────────── 更新 position/velocity/torque
  ↓
绘图 + 终端显示
```

夹爪控制严格对照 `cv_motor_continuous_control_plot.py`：
- `Gripper` 类 (actuator.gripper)
- POS_VEL 模式
- `HardwareWorker` 独立线程
- 慢速零位初始化到闭合位置

## 使用方法

### 基础遥操作

```bash
python hls_teleop_gripper.py
python hls_teleop_gripper.py --yes    # 跳过安全确认
```

### 力反馈遥操作

```bash
python hls_teleop_gripper_force.py
python hls_teleop_gripper_force.py --force-threshold 0.3
python hls_teleop_gripper_force.py --yes
```

## 位置映射

```
HLS 舵机角度              夹爪位置
  90° (左转到底)    ←→   -5.8 rad (张开)
 135°               ←→   -2.9 rad
 180° (中位/零位)    ←→    0.0 rad (闭合)
```

零位 (180°) = 闭合, 逆时针左转向 90° = 张开。

## 绘图窗口

| 子图 | 内容 |
|------|------|
| 第1行 | HLS 舵机角度 + 开/闭合参考线 |
| 第2行 | 夹爪位置 (实际 红实线 / 指令 紫虚线) |
| 第3行 | 夹爪速度 |
| 第4行 | 夹爪力矩 (+ 力反馈阈值线) |

## 运行流程

```
[1] 初始化 HLS3606 舵机
[2] 启动 HardwareWorker 线程:
    - Gripper 类加载 gripper.yaml
    - disable → mode_pos_vel → enable
    - 慢速移动到零位 (0.0 rad, vlim=2.0)
[3] HLS 舵机到中位 180° + 释放扭矩
[4] 遥操作运行 (Ctrl+C 停止)
[5] 安全退出
```

## 力反馈原理 (仅 force 版)

```
夹爪力矩 (Nm)          状态               HLS 行为
  ≤ 0.25           空载 (free)         扭矩释放, 柔顺转动
  > 0.25           抓取 (GRASP)        扭矩 600, 锁在抓取点
  ≤ 0.125 (滞后)    脱离                 恢复释放柔顺

抓取时安全限制:
  - 夹爪指令 = min(舵机指令, 抓取位置 + 0.1)  — 只能张不能继续闭
  - 夹爪速度降至 2.0 rad/s                    — 防止过冲
```

## 控制参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `CONTROL_RATE` | 100 Hz | 控制循环频率 |
| `VELOCITY_LIMIT` | 6.0 | 遥操作速度上限 rad/s |
| `STARTUP_VELOCITY_LIMIT` | 2.0 | 零位初始化速度 |
| `STARTUP_POSITION_TOLERANCE` | 0.05 | 零位到位容差 rad |
| `FORCE_THRESHOLD` | 0.2 Nm | 力反馈触发阈值 |
| `TORQUE_GRASP` | 600 | 抓取时 HLS 扭矩 |
| `BACKOFF_MARGIN` | 0.1 rad | 抓取后允许的最大额外闭合量 |
