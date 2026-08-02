# HLS_servo_sdk — 飞特 HLS 系列舵机 Python SDK 示例

本目录包含**飞特（FEETECH）HLS 系列舵机**的 Python 通信示例代码，所有示例基于父目录 `scservo_sdk` 核心库。

---

## 目录

| 文件 | 功能 |
|------|------|
| `ping.py` | 检测舵机在线状态，返回型号 |
| `read.py` | 循环读取舵机当前位置和速度 |
| `write.py` | 简单往复位置控制 |
| `read_write.py` | 写位置 + 轮询等待运动到位 |
| `reg_write.py` | 多舵机寄存器写 + 同步触发（多舵机同步运动） |
| `sync_read.py` | 批量同步读取多个舵机位置和速度 |
| `sync_write.py` | 批量同步写入多个舵机目标位置 |
| `wheel.py` | 轮式模式（速度模式）连续旋转控制 |
| `ofscal.py` | 位置偏移校准（零点设置） |
| `reset.py` | 舵机状态重置（清除多圈圈数） |

---

## 依赖

```
scservo_sdk（飞特官方核心库，位于父目录）
├── PortHandler      — 串口管理（Linux "COM口" 等）
├── hls              — HLS 协议包处理器
├── GroupSyncRead    — 批量同步读
├── GroupSyncWrite   — 批量同步写（由 hls 管理）
└── COMM_SUCCESS     — 通信状态常量
```

---

## 快速开始

### 1. 硬件连接

- 将舵机通过 USB 转 TTL/RS485 模块连接到主机
- 确认设备路径：`ls /dev/tty*`（常见：`/dev/ttyACM1`、`/dev/ttyUSB0`）
- 修改脚本中的设备路径为实际端口

### 2. 运行示例

```bash
cd HLS_servo_sdk
python ping.py
```

### 3. 典型用法模式

| 需求 | 参考示例 | 核心 API |
|------|----------|----------|
| 检查连接 | `ping.py` | `ping(id)` |
| 单舵机位置控制 | `write.py` | `WritePosEx(id, pos, speed, acc, torque)` |
| 等待运动到位 | `read_write.py` | `ReadMoving(id)` 循环轮询 |
| 多舵机同步启动 | `reg_write.py` | `RegWritePosEx()` + `RegAction()` |
| 批量读取状态 | `sync_read.py` | `GroupSyncRead` |
| 尾部写入指令 | `sync_write.py` | `SyncWritePosEx()` + `groupSyncWrite.txPacket()` |
| 速度/连续旋转 | `wheel.py` | `WheelMode()` + `WriteSpec()` |
| 零点校准 | `ofscal.py` | `reOfsCal(id, position)` |
| 清除圈数 | `reset.py` | `reSet(id)` |

---

## API 参考

### hls（协议包处理器）

| 方法 | 说明 | 参数 | 返回值 |
|------|------|------|--------|
| `ping(id)` | 检测舵机在线状态和型号 | `id`: 舵机 ID | `(model_number, comm_result, error)` |
| `ReadPosSpeed(id)` | 读取当前位置和速度 | `id`: 舵机 ID | `(position, speed, comm_result, error)` |
| `ReadMoving(id)` | 读取运动状态 | `id`: 舵机 ID | `(moving, comm_result, error)` — moving==0 表示停止 |
| `WritePosEx(id, pos, speed, acc, torque)` | 写目标位置（含速度/加速度/扭矩） | `id`: ID, `pos`: 位置, `speed`: 速度, `acc`: 加速度, `torque`: 扭矩 | `(comm_result, error)` |
| `RegWritePosEx(id, pos, speed, acc, torque)` | 寄存器写（暂存指令，等待触发） | 同上 | `(comm_result, error)` |
| `RegAction()` | 触发所有已寄存的写指令 | 无 | — |
| `SyncWritePosEx(id, pos, speed, acc, torque)` | 添加舵机到同步写缓冲区 | 同 `WritePosEx` | `True` / `False` |
| `reOfsCal(id, pos)` | 位置偏移校准 | `id`: 舵机 ID, `pos`: 目标位置值 | `(comm_result, error)` |
| `reSet(id)` | 重置舵机（清除多圈圈数） | `id`: 舵机 ID | `(comm_result, error)` |
| `WheelMode(id)` | 切换到轮式/速度模式 | `id`: 舵机 ID | `(comm_result, error)` |
| `WriteSpec(id, speed, acc, torque)` | 速度控制指令（需先进入轮式模式） | `id`: ID, `speed`: 速度（正=正转, 负=反转, 0=停止）, `acc`: 加速度, `torque`: 扭矩 | `(comm_result, error)` |
| `getTxRxResult(code)` | 获取通信状态描述 | `code`: 状态码 | 字符串 |
| `getRxPacketError(code)` | 获取应答包错误描述 | `code`: 错误码 | 字符串 |
| `scs_tohost(value, bits)` | 舵机数据 → 主机格式转换 | `value`: 原始值, `bits`: 位数 | 转换后的值 |

**属性：** `groupSyncWrite` — GroupSyncWrite 实例，用于同步写操作。

---

### GroupSyncRead（批量同步读）

| 方法 | 说明 |
|------|------|
| `addParam(id)` | 添加要读取的舵机 ID |
| `txRxPacket()` | 发送批量读请求并接收应答 |
| `isAvailable(id, addr, len)` | 检查指定舵机的数据是否可用 |
| `getData(id, addr, len)` | 获取读取到的数据 |
| `clearParam()` | 清空参数列表，为下一轮读取做准备 |

---

## 通信参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 波特率 | 1,000,000 (1 Mbps) | 固定波特率 |
| 位置范围 | 0–4095 | 12 位分辨率，对应 0–360° |
| 速度换算 | V × 0.732 rpm | V=60 → 43.92 rpm |
| 加速度换算 | A × 8.7 deg/s² | A=50 → 435 deg/s² |

### 运动时间估算公式

```
T = (P1 - P0) / (V × 50) + (V × 50) / (A × 100) + 0.05
```

---

## 轮式模式（`wheel.py`）

HLS 舵机支持两种工作模式，通过 `WheelMode()` 可切换：

| 模式 | 切换方法 | 控制方法 | 行为 |
|------|----------|----------|------|
| **位置模式**（默认） | — | `WritePosEx()` | 精准到达目标角度位置 |
| **轮式/速度模式** | `WheelMode(id)` | `WriteSpec()` | 连续旋转如普通电机 |

### 速度模式使用流程

```python
# 1. 切换到速度模式
packetHandler.WheelMode(1)

# 2. 速度控制
packetHandler.WriteSpec(1, 60, 50, 500)   # 正转: speed=60, acc=50, torque=500
time.sleep(5)
packetHandler.WriteSpec(1, 0, 50, 500)    # 停止: speed=0
time.sleep(2)
packetHandler.WriteSpec(1, -50, 50, 500)  # 反转: speed=-50
```

### 速度参数说明

- **speed**：正值为正转，负值为反转，0 为停止（换算：V × 0.732 rpm）
- **acc**：加速度（换算：A × 8.7 deg/s²）
- **torque**：扭矩限制（500 为满扭矩）

---

## 同步运动对比

### 寄存器模式（`reg_write.py` — 推荐）
- **特点**：先向各舵机下发目标 → `RegAction()` 统一触发 → 所有舵机**同时启动**
- **适用**：多舵机协调运动、轨迹同步

### 同步写模式（`sync_write.py`）
- **特点**：将多舵机指令打包为**单个通信包**发送
- **适用**：需要减少总线通信延迟的场景

---

## 常见问题

### 1. 设备权限拒绝
```bash
sudo chmod 666 /dev/ttyACM1
# 或添加用户到 dialout 组
sudo usermod -a -G dialout $USER
```

### 2. 通信失败
- 确认波特率为 1000000
- 确认舵机 ID 无误
- 检查供电电压和接线

### 3. 设备路径不确定
```bash
dmesg | grep tty     # 查看刚插入设备的内核日志
ls /dev/tty*         # 列出所有串口设备
```
