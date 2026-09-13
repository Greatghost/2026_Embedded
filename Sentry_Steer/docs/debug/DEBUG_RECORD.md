# Debug Record — 2026-06-18

## 1. WBUS 遥控器旋转角速度

WBUS 遥控器通过 `DJIRemoteUpdate()` 控制旋转（小陀螺），`chassis_speed_w` 各模式取值：

| 拨杆组合 | 模式 | `chassis_speed_w` | 实际角速度 (MAX_YAW=16.0) |
|---|---|---|---|
| LEFT=Down, RIGHT=Mid | 检录小陀螺 | `0.5 × MAX_YAW_SPEED` | 8.0 rad/s |
| LEFT=Down, RIGHT=Up | 检录射击+旋转 | `0.75 × MAX_YAW_SPEED` | 12.0 rad/s |
| LEFT=Mid, RotateState=1 | NUC模式 | `0.5 × MAX_YAW_SPEED` | 8.0 rad/s |
| LEFT=Mid, RotateState=2 | NUC模式 | `0.75 × MAX_YAW_SPEED` | 12.0 rad/s |
| LEFT=Mid, RotateState=3 | NUC模式 | `1.0 × MAX_YAW_SPEED` | 16.0 rad/s |
| LEFT=Up | 遥控模式 | `0` | 不旋转 |

> `ROBOT = TIGER` → `MAX_YAW_SPEED = 16.0f` rad/s

---

## 2. `wbus_receiver` 数据流

```
WBUS接收机 ──USART3(100kbps)──▶ DMA双缓冲(sbus_rx_buf[2][36])
  └─ USART3_IRQHandler() 空闲中断 → RemoteReceive()
    └─ WBUS_Decode() → wbus_receiver.data.ch[0..15] (原始值 172~1811)
      └─ WBUS_UpdateRemoteController() → remote_controller.dji_remote.rc
        └─ WBUS(172~1811) → DJI(364~1684) 线性映射 + 死区
          └─ ActionTask@250Hz → get_control_info() → DJIRemoteUpdate()
            └─ chassis_solver.chassis_speed_w/x/y → CAN发送到电机
```

WBUS 通道映射（`WBUS_UpdateRemoteController`）：
- CH1→LEFT_CH_LR, CH2→LEFT_CH_UD (左摇杆)
- CH3→RIGHT_CH_UD, CH4→RIGHT_CH_LR (右摇杆)
- CH7→RIGHT_SW (三位拨杆)
- CH5+CH8 组合→LEFT_SW (双位拨杆: CH5下=下电, CH5上+CH8上=遥控, CH5上+CH8下=NUC)

---

## 3. Pitch / Yaw 电机 CAN ID

当前配置 `ROBOT = TIGER`，使用 **DM 达妙电机**：

| 电机 | CAN总线 | 发送 ID (STM32→Motor) | 接收 ID (Motor→STM32) | 电机类型 |
|---|---|---|---|---|
| **Pitch** | **CAN2** (PB5/PB6) | **0x06** | **0x11** | DM_MOTOR |
| **Big Yaw** | **CAN1** (PD0/PD1) | **0x05** | **0x10** | DM_MOTOR (DM8006) |

接收 ID 定义: [can_config.h](config/can_config.h)
- `PITCH_MOTOR_CAN_ID = 0x11`
- `BIG_YAW_MOTOR_CAN_ID = 0x10`

发送 ID: `MOTOR_STD_ID_LIST[5]=0x05` (DM_MOTOR_1), `[6]=0x06` (DM_MOTOR_2)

已删除：`SMALL_YAW_MOTOR_CAN_ID = 0x205` (GM6020)

**`pitch_recv` / `pitch_info` / `big_yaw_recv` / `big_yaw_info` 是旧的 GM6020 类型字段，DM 电机模式下永远为空。** 数据在 `DM_Pitch_Motor.P_Receive` 和 `DM_Big_Yaw_Motor.P_Receive`。

---

## 4. DM 电机反馈数据

**Pitch 电机 (CAN2)**: ✅ 正常
- `P_Receive = 297.52°`, `ERR_State = 1`, `Motor_Enable = 1`
- Kp=0, Kd=0 (MIT模式无阻尼，需要设置)

**Big Yaw 电机 (CAN1)**: ❌ 无数据
- `P_Receive = 0`, `ERR_State = 0`, `T_MOS = 0`
- `DM_Motor_Receive()` 从未被调用

---

## 5. Big Yaw CAN1 排查过程

### 排查步骤
1. `global_debugger.gimbal_debugger[0].recv_msgs_num` — Pitch 正常增长
2. `global_debugger.gimbal_debugger[2].recv_msgs_num` — Big Yaw **永远为 0**
3. `is_has_motor_data[0][5]` — STM32 在发送 Big Yaw 数据 ✓
4. `CAN1->ESR = 0xFFFF0017`: **TEC=255, REC=0** → CAN1 物理层断开

### CAN 分析仪结果

**CAN2 总线** (正常):
| ID | 说明 |
|---|---|
| 0x202 | 右摩擦轮反馈 |
| 0x11 | Pitch 电机反馈 |
| 0x201 | 左摩擦轮反馈 |
| 0x06 | Pitch 电机控制命令 |
| 0x210 | DJI 电机组 |
| 0x200 | DJI 电机组 |

**CAN1 总线** (异常):
| ID | 说明 |
|---|---|
| 0x201 | 左摩擦轮（**应该在 CAN2 上**） |
| 0x05/0x10/0x204 | **全部缺失** |

### 结论
**CAN1 物理线束上挂的是左摩擦轮(0x201)而非 Big Yaw 电机**。硬件接线与代码分配不一致，需要查 PCB 原理图确认 PD0/PD1 (CAN1) 和 PB5/PB6 (CAN2) 的实际连接。

---

## 6. Pitch 电机问题修复

### 问题1：进入射击模式向下冲到机械限位
**原因**: 软限位 `GIMBAL_ANGLE_MIN/MAX` 为旧 GM6020 标定值 (40°~90°)，DM 电机 `P_Receive=297°` 完全超出范围。
**修复**: 改为 `GIMBAL_ANGLE_MIN = 275.0f`, `GIMBAL_ANGLE_MAX = 315.0f`

### 问题2：剧烈振荡
**原因**: PID 为 GM6020 电压模式调参，DM 电机 MIT 模式下 Kp=Kd=0，外环增益过大会导致发散振荡。
**修复**:
- Angle PID: Kp 500→80, Ki 48→5, MaxOut 40→10
- Speed PID: Kp 15000→300, Ki 1200→20, MaxOut 35→10
- DM 内环: `Kp=15, Kd=5` (提供内部阻尼)
- 去掉速度环 Kd (DM 内环已有阻尼)

调参起点（后续需要手动调优）:
```c
// Gimbal.c TIGER 分支
PID_Init(&pitch_angle_pid, 80, 5, 0, 10, 0, 0, ...);
PID_Init(&pitch_speed_pid, 300, 20, 0, 10, 1, 0, ...);
DM_Pitch_Motor.Kp = 15;
DM_Pitch_Motor.Kd = 5;
```

---

## 7. 关键文件索引

| 文件 | 内容 |
|---|---|
| [ChassisSolver.c](application/src/ChassisSolver.c) | 遥控器控制逻辑、底盘速度解算 |
| [wbus_decoder.c](application/src/wbus_decoder.c) | WBUS 协议解码、通道映射 |
| [remote_control.c](application/src/remote_control.c) | 遥控器接收、模式设置 |
| [can_config.h](config/can_config.h) | CAN ID 定义、滤波器配置 |
| [can_send_config.c](config/can_send_config.c) | 电机通信配置 (`Motor_Config_Init`) |
| [gimbal_config.h](config/gimbal_config.h) | 云台角度限位、电机符号 |
| [Gimbal.h](application/inc/Gimbal.h) | `GimbalController` 结构体 |
| [Gimbal.c](application/src/Gimbal.c) | PID 初始化、云台计算 |
| [bsp_can.c](bsp/boards/src/bsp_can.c) | CAN 接收回调、滤波器初始化 |
| [GimbalTask.c](Task/src/GimbalTask.c) | 电机数据发送任务 |
| [ActionTask.c](Task/src/ActionTask.c) | 250Hz 控制循环 |
| [DM_Motor.c](components/motor/src/DM_Motor.c) | DM 电机 MIT 控制/接收 |
| [can.c](Src/can.c) | CAN 波特率配置 (1Mbps) |
| [robot_config.h](config/robot_config.h) | `ROBOT = TIGER` |

## 8. Bug 修复: Pitch 电机上电下冲 (2026-06-18)

**现象**: 上电后 Pitch 电机立即向下冲到 262° 机械限位。

**根因**: DM 电机 MIT 模式下，`P_des` 始终为 0 (BSS 零初始化)，编码后对应 DM 电机的 180° 位置。而电机实际在 297°，内部位置环 `Kp=15` 产生 `Kp×(180°-297°)` 的巨大向下回复力矩，远超外部 PID 的 `t_ff` 输出，导致电机被拉向机械限位。

**修复**: `Gimbal.c` → `Kp` 从 15 改为 **0**，禁用 DM 内部位置环，纯 MIT 力矩控制。外部 cascaded PID 完全负责位置控制，Kd=5 保留提供速度阻尼。

> 全项目 `P_des` 从未被赋值。`Motor_Data_Pack()` 仅设置 `t_ff`，未同步 `P_des`。如需将来使用 Kp>0，需在 `Motor_Data_Pack()` 中将当前位置回写 `P_des`。

---

## 9. 遗留问题

- [ ] **CAN1 硬件接线** — Big Yaw 电机未接到 CAN1，需要查 PCB 原理图修正物理接线
- [ ] **DM 电机返回 ID** — 确认 Big Yaw 电机是否配置为在 0x10 上返回数据（DM 默认在相同 ID 上收发）
- [ ] **Pitch PID 需在线调参** — 当前参数仅为安全的起调点，需要手动优化跟踪性能
- [ ] **重力补偿多项式** — 原为旧 GM6020 系统拟合，DM 电机下可能需要重新标定
