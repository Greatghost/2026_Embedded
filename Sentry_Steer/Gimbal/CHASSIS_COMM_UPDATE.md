# 云台→底盘 通信更新

> 2026-06-19 | 云台侧修改，底盘需配合解析新增 CAN 帧

---

## 一、新增：哨兵坐标 CAN 帧

### CAN 0x151（云台 → 底盘）

| 字节 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0-1 | `sentry_x_cm` | int16 LE | 哨兵 X 坐标 (cm) |
| 2-3 | `sentry_y_cm` | int16 LE | 哨兵 Y 坐标 (cm) |
| 4-7 | `reserved` | uint8[4] | 预留 0x00 |

**发送频率**：10Hz（每 50 个 GimbalTask 周期发一次）

**坐标来源**：上位机 → USB CDC TypeID=0x01 → STM32 解析校验后原值转发

**范围**：X: 0~2800 cm，Y: 0~1500 cm

---

## 二、修改：USB 下行帧格式

原有下行帧**帧长从 17 字节改为 18 字节**，新增 TypeID 字段。

### TypeID = 0x00（标准控制帧，18 bytes）

| 字节 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | `Head` | uint8 | 0x21 (`!`) |
| 1 | `TypeID` | uint8 | **0x00** (新增) |
| 2 | `Aim_v_x` | int8 | 导航 X 速度 (/50=m/s) |
| 3 | `Aim_v_y` | int8 | 导航 Y 速度 (/50=m/s) |
| 4-7 | `Aim_Yaw` | float | 自瞄 Yaw 角度 |
| 8-11 | `Aim_Pitch` | float | 自瞄 Pitch 角度 |
| 12 | `FireCode` | uint8 | 控制位域 (不变) |
| 13-16 | `SentryCmd` | uint32 | 裁判系统指令 (不变) |
| 17 | `tail` | uint8 | 0x00 |

### TypeID = 0x01（哨兵坐标帧，18 bytes）

| 字节 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | `Head` | uint8 | 0x21 (`!`) |
| 1 | `TypeID` | uint8 | **0x01** |
| 2-3 | `x_cm` | int16 LE | 哨兵 X 坐标 (cm) |
| 4-5 | `y_cm` | int16 LE | 哨兵 Y 坐标 (cm) |
| 6-15 | `reserved` | uint8[10] | 预留 0x00 |
| 16 | `CRC8` | uint8 | CRC8 校验 (poly=0x31, init=0xFF, 覆盖byte0-15) |

---

## 三、底盘需要做什么

1. **新增 CAN 0x151 接收** → 解析 int16 LE 的 X/Y 坐标
2. **USB 下行帧适配** → 帧长 17→18，`!` 后多一个字节 TypeID，根据 TypeID 区分解析
3. **CAN 0x150 (pack1) 不变** → 帧格式无改动
