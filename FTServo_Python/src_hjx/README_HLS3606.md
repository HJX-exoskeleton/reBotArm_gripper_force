# HLS3606 舵机测试脚本

本目录包含 HLS3606 型号舵机的完整测试脚本集，基于 `scservo_sdk` (FTServo Python SDK) 的 `hls` 协议处理器。

## 环境要求

```bash
pip install numpy matplotlib pyserial
```

## 脚本概览

| 脚本 | 功能 | 适用场景 |
|------|------|----------|
| `hls3606_ping_test.py` | 连接/Ping/状态读取 | 初次连接验证 |
| `hls3606_return_home.py` | 回归初始位置 | 零点复位 |
| `hls3606_motion_curve.py` | 运动曲线实时绘制 | 轨迹追踪测试 |
| `hls3606_sync_multi.py` | 多舵机同步控制 | 多关节协同 |

---

## 1. hls3606_ping_test.py — 连接与状态测试

验证舵机通信是否正常，读取基本状态信息。

```bash
# 默认使用 /dev/ttyACM0, 测试 ID 1,2
python hls3606_ping_test.py
```

**功能:**
- 串口连接测试
- Ping 舵机获取型号
- 读取位置、速度、电压、温度、负载
- 扭矩开关测试

**修改默认参数:** 编辑脚本中的 `SERIAL_PORT`、`BAUDRATE`、`SERVO_IDS` 变量。

---

## 2. hls3606_return_home.py — 回归初始位置

将舵机安全地移动到预设零点位置。

```bash
# 默认回零 (ID 1,2 → 2048,2048)
python hls3606_return_home.py

# 指定舵机和零点位置
python hls3606_return_home.py --ids 1,2,3 --home-pos 2048,2048,1024

# 低速回零
python hls3606_return_home.py --speed 15 --acc 8

# 先复位舵机(清除圈数)再回零
python hls3606_return_home.py --reset

# 回零后保持扭矩 (不释放)
python hls3606_return_home.py --no-release
```

**参数说明:**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--port` | `/dev/ttyACM0` | 串口设备路径 |
| `--baudrate` | `1000000` | 波特率 (HLS 默认 1M) |
| `--ids` | `1,2` | 舵机 ID 列表 |
| `--home-pos` | `2048,...` | 各舵机零点 (raw值, 0-4095) |
| `--speed` | `20` | 回零速度 (~14.6 rpm) |
| `--acc` | `10` | 回零加速度 (~87 deg/s²) |
| `--torque` | `500` | 扭矩限制 |
| `--no-release` | False | 不回零后释放扭矩 |
| `--reset` | False | 先执行舵机复位 |
| `--timeout` | `15.0` | 等待超时 (秒) |

**位置换算:** `角度 = raw值 × 360 ÷ 4095`, 中间位置 2048 = 180°

---

## 3. hls3606_motion_curve.py — 运动曲线同步绘制

实时绘制舵机位置追踪曲线，支持多种运动轨迹模式。

```bash
# 正弦波轨迹 (默认)
python hls3606_motion_curve.py --mode sine

# 梯形速度曲线
python hls3606_motion_curve.py --mode trapezoid --period 6

# 三角波轨迹
python hls3606_motion_curve.py --mode triangle

# 扫频测试 (测试舵机响应带宽)
python hls3606_motion_curve.py --mode sweep

# 单舵机 + 保存数据
python hls3606_motion_curve.py --ids 1 --mode sine --save

# 自定义范围和时长
python hls3606_motion_curve.py --mode sine --min-pos 512 --max-pos 3584 --duration 120
```

**轨迹模式:**

| 模式 | 说明 |
|------|------|
| `sine` | 正弦波: 平滑往复运动 |
| `trapezoid` | 梯形波: 加速→匀速→减速, 模拟工业轨迹 |
| `triangle` | 三角波: 恒速往返, 测试速度反转 |
| `sweep` | 扫频: 频率递增正弦波, 测试响应带宽 |

**参数说明:**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--mode` | `sine` | 轨迹模式 |
| `--period` | `4.0` | 运动周期 (秒) |
| `--min-pos` | `1024` | 最小位置 (raw) |
| `--max-pos` | `3072` | 最大位置 (raw) |
| `--speed` | `60` | 运动速度 (~43.9 rpm) |
| `--acc` | `30` | 加速度 (~261 deg/s²) |
| `--duration` | `60` | 运行时长 (0=无限) |
| `--save` | False | 保存数据到 CSV |
| `--save-dir` | `./data_logs` | CSV 保存目录 |

**实时绘图窗口:**
- 上方子图: 每个舵机的目标位置(红虚线) vs 实际位置(蓝实线)
- 下方子图: 每个舵机的追踪误差 (度)

---

## 4. hls3606_sync_multi.py — 多舵机同步控制

使用 SyncWrite/SyncRead 实现多舵机同步运动。

```bash
# 同步位置控制
python hls3606_sync_multi.py --mode sync_pos --ids 1,2,3,4

# 波浪运动 (蛇形)
python hls3606_sync_multi.py --mode wave --ids 1,2,3,4 --wave-period 6

# 接力运动
python hls3606_sync_multi.py --mode relay --ids 1,2,3

# 同步读取测试
python hls3606_sync_multi.py --mode sync_read --ids 1,2 --read-interval 0.1
```

**运动模式:**

| 模式 | 说明 |
|------|------|
| `sync_pos` | 同步位置: 所有舵机同时运动到相同位置 |
| `wave` | 波浪运动: 相位差正弦波, 模仿蛇形/波浪 |
| `relay` | 接力运动: 逐个运动, 前一个完成后下一个开始 |
| `sync_read` | 同步读取: 持续读取并显示所有舵机状态 |

**参数说明:**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--mode` | `sync_pos` | 运动模式 |
| `--ids` | `1,2` | 舵机 ID 列表 |
| `--speed` | `40` | 运动速度 |
| `--acc` | `20` | 加速度 |
| `--step-delay` | `2.0` | 步间等待 (秒) |
| `--wave-period` | `4.0` | 波浪周期 (秒) |
| `--wave-dt` | `0.05` | 波浪控制周期 (秒) |
| `--read-interval` | `0.5` | 同步读取间隔 (秒) |

---

## HLS3606 通信协议要点

### SDK 核心类

```python
from scservo_sdk import *

# 串口处理器
portHandler = PortHandler("/dev/ttyACM0")
portHandler.openPort()
portHandler.setBaudRate(1000000)

# HLS 协议处理器
packetHandler = hls(portHandler)
```

### 常用 API

| 方法 | 功能 |
|------|------|
| `ping(id)` | Ping 舵机, 返回型号 |
| `WritePosEx(id, pos, speed, acc, torque)` | 位置控制 (带速度/加速度/扭矩) |
| `ReadPos(id)` | 读当前位置 |
| `ReadSpeed(id)` | 读当前速度 |
| `ReadPosSpeed(id)` | 同时读位置和速度 |
| `ReadMoving(id)` | 读运动状态 (0=停止) |
| `SyncWritePosEx(id, pos, speed, acc, torque)` | 添加同步写参数 |
| `groupSyncWrite.txPacket()` | 执行同步写 |
| `reSet(id)` | 舵机复位 |
| `reOfsCal(id, pos)` | 位置校准 |
| `WheelMode(id)` | 设置轮式模式 |
| `WriteSpec(id, speed, acc, torque)` | 速度控制 (轮式) |

### 单位换算

| 参数 | 公式 |
|------|------|
| 位置 | `角度(°) = raw × 360 / 4095` |
| 速度 | `转速(rpm) = raw × 0.732` |
| 加速度 | `角加速度(°/s²) = raw × 8.7` |
| 电压 | `电压(V) = raw × 0.1` |
| 负载 | `负载(%) = raw × 100 / 1023` |

### 位置范围

- Raw 值: **0 ~ 4095** (12-bit 分辨率)
- 角度: **0° ~ 360°**
- 中间位置: **2048 = 180°**

### 内存地址 (常用)

| 地址 | 名称 | 说明 | 访问 |
|------|------|------|------|
| 5 | HLS_ID | 舵机 ID | R/W |
| 6 | HLS_BAUD_RATE | 波特率 | R/W |
| 33 | HLS_MODE | 模式 (0=位置, 1=轮式) | R/W |
| 40 | HLS_TORQUE_ENABLE | 扭矩使能 (0=释放, 1=使能) | R/W |
| 41 | HLS_ACC | 加速度 | R/W |
| 42-43 | HLS_GOAL_POSITION | 目标位置 | R/W |
| 46-47 | HLS_GOAL_SPEED | 目标速度 | R/W |
| 56-57 | HLS_PRESENT_POSITION | 当前位置 | R |
| 58-59 | HLS_PRESENT_SPEED | 当前速度 | R |
| 62 | HLS_PRESENT_VOLTAGE | 当前电压 (×0.1V) | R |
| 63 | HLS_PRESENT_TEMPERATURE | 当前温度 (°C) | R |
| 66 | HLS_MOVING | 运动状态 (0=停止) | R |

---

## 硬件连接

```
电脑 USB ──► USB-TTL 转换器 ──► HLS3606 舵机
              TX ──► RX (舵机)
              RX ──► TX (舵机)
              GND ──► GND

Linux 设备路径: /dev/ttyACM0 或 /dev/ttyUSB0
```

**权限问题:** 如提示权限不足:
```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

---

## 旧脚本 (SCS0009 系列, 基于 rustypot)

以下脚本使用 `rustypot` 库, 适用于 SCS0009 系列舵机:

| 脚本 | 功能 |
|------|------|
| `scs_control_two.py` | 双舵机夹爪控制 |
| `scs_control_pos_plot.py` | 位置实时绘制 |
| `scs_control_pos_save.py` | 数据采集保存 |
| `scs_control_pos_replay.py` | 轨迹回放 |
| `scs_control_pos_read_data.py` | 离线数据可视化 |
| `sleep.py` | 舵机休眠 |
