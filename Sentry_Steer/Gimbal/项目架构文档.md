# YP 云台 STM32 控制板 —— 项目架构文档

> 适用对象：机器人 YP 云台控制板（STM32 + FreeRTOS）
> 机型配置：`ROBOT = TIGER`（达妙 DM 电机方案）
> 本文档仅用于梳理代码结构与控制逻辑，不涉及任何代码改动。
>
> **最近更新（2026-07-19）**：
> - **2026-07-19**：**新增 `SHOOT_UNSTOPPABLE_AUTO_AIM_MODE` 射击模式**——当上位机下发 `sentry_cmd` 的 `current_posture==4`（强化进攻姿态，bit21-23 提取）时，`PCStateControl()` 切到该模式而非 `SHOOT_AUTO_AIM_MODE`。新模式与 AUTO_AIM 共享 `Shoot_Autoaim_Cal()` 全部辅瞄判定逻辑（FireCode 沿、AA_Shootable、掉线安全停），仅在 `Shoot_Pos_Cal()` 内部取更高弹频：`Shoot_IntervalTime=10`（约 50Hz，AUTO_AIM 为 25≈20Hz，手动为 50≈10Hz）。
> - **2026-07-19**：**UNSTOPPABLE 模式两项修订**——①摩擦轮补偿：`Shoot_Unstoppable_Cal()` 在 `setFrictionSpeed()` 基础上叠加 `UNSTOPPABLE_FRICTION_BOOST`（`application/inc/FrictionWheel.h`，2000 deg/s），抵消高弹频下子弹持续摩擦造成的转速下沉；②触发逻辑改写：原"左右摇杆任一轴都 >330"要求瞄准用的右摇杆也得推到一半以上，与瞄准冲突。现改为**仅由物理左摇杆（= `RIGHT_CH_UD`）上推 > 600** 触发，物理右摇杆（= `LEFT_CH_*`）完全解放用于 Yaw/Pitch 瞄准；左摇杆 > 330 但未到 600 时仍走 `SHOOT_FIRE_MODE` 单发路径（详见 §3.4 与 `遥控器模式映射表.md`）。
> - **2026-07-16**：清理独立 `BigYaw/` 子项目目录与 `power_cal/` 功率计算工具目录；新增 `downlink_control_frame.md`、`map_path_fragment_reassembly.md` 等协议文档；新增 `Pitch_Limit_Oscillation/` 限位震荡分析目录。
> - **2026-07-18**：**新增大 Yaw 模型前馈控制**（J/B/C 物理模型，`u_ff = gain*(J*α+B*ω+C*sat(ω/blend))`），参数入口 `config/gimbal_config.h`；**新增 GimbalSystemID 模块**（移植自 `crossing_hole-main`，自动辨识 Pitch G/B/C/J 与 Yaw B/C/J）；新增 `YAW_FEEDFORWARD_CALIBRATION.md`、`application/GimbalSystemID_PORTING.md`、`tools/fit_yaw_feedforward.py`；下行新增 `0x05` MPC 轨迹帧、上行新增 TypeID 11 云台动态反馈（已同步 §3.3）。
> - 云台⇄上位机上下行协议、云台⇄底盘 CAN 通信在 2026-07-11~13 发生较大变更，旧协议已不兼容，需上位机同步切换。

---

## 目录

1. [代码架构](#一代码架构)
2. [控制逻辑脉络](#二控制逻辑脉络)
3. [重要模块](#三重要模块)
   - [3.1 云台 YAW 与 Pitch 轴控制逻辑](#31-云台-yaw-与-pitch-轴控制逻辑)
   - [3.2 两路 CAN 通信](#32-两路-can-通信)
   - [3.3 云台与上位机通信](#33-云台与上位机通信)
   - [3.4 云台与遥控器遥控](#34-云台与遥控器遥控)
4. [重要的全局变量 / 结构体](#四重要的全局变量--结构体)
5. [附录：根目录文档索引](#五附录根目录文档索引)

---

## 一、代码架构

### 1.1 分层结构

整个工程按职责分为 5 层，自底向上：

```
┌───────────────────────────────────────────────────────────┐
│  Task/          任务层：FreeRTOS 各线程（控制主循环所在）     │
├───────────────────────────────────────────────────────────┤
│  application/   应用层：云台/底盘/上位机/遥控业务逻辑与结构体 │
├───────────────────────────────────────────────────────────┤
│  config/        配置层：机型选择、CAN ID、滤波器布局          │
├───────────────────────────────────────────────────────────┤
│  components/    组件层：算法(PID/EKF/TD) / 电机 / 传感器 / 工具│
├───────────────────────────────────────────────────────────┤
│  bsp/ + Drivers 板级支持包：CAN/USB/RC/PWM/SPI + HAL 库       │
└───────────────────────────────────────────────────────────┘
```

> 说明：`Drivers/`、`Middlewares/`、`MDK-ARM/`、`Inc/`、`Src/`（除 freertos.c）等为 CubeMX 生成的底层/工程文件，本文档不展开。

### 1.2 各层文件清单

**Task/ 层（FreeRTOS 任务）** —— 详见 [控制逻辑脉络](#二控制逻辑脉络)

| 文件 | 职责 |
|------|------|
| `Src/freertos.c` | 任务创建入口 `MX_FREERTOS_Init` |
| `Task/src/GimbalTask.c` | **云台电机控制核心**（500Hz） |
| `Task/src/ActionTask.c` | 状态切换 / 遥控器数据获取（250Hz） |
| `Task/src/ChassisTask.c` | 向底盘发送速度与坐标（1000Hz） |
| `Task/src/ShootTask.c` | 射击 / 摩擦轮 / 拨盘控制（由 GimbalTask 调用） |
| `Task/src/Offline_Task.c` | 离线检测（1000ms 周期） |
| `Task/src/INS_task.c` | IMU 姿态解算 |

**application/ 层（应用逻辑与结构体）**

| 模块 | 头文件 | 职责 |
|------|--------|------|
| 云台 | `application/inc/Gimbal.h` | `GimbalController` / `BigYawController` 结构体、PID 计算函数、大 Yaw 模型前馈 `BigYawModelFeedForward()` |
| 控制求解 | `application/inc/ChassisSolver.h` | 遥控/上位机 → 云台目标角度的分发逻辑 |
| 上位机通信 | `application/inc/pc_serial.h` | 收发协议：下行 6 帧（DownlinkTypeID 0x00~0x05）/ 上行 TypeID 0~11、`PC_Send` / `PCReceive` |
| 遥控 | `application/inc/remote_control.h` | 遥控器结构体、模式枚举 |
| 底盘下行 | `application/inc/ChassisSend.h` | CAN 0x150 / 0x151 / 0x15A 打包 |
| 底盘上行 | `application/inc/ChassisGet.h` | CAN 0x160 / 0x09x / 0x09F / 0x0A0 解析（`SentryDuration_t` / `DamageDiff_t`） |
| **系统辨识** | `application/inc/GimbalSystemID.h` | **新增**：`crossing_hole-main` 移植的 G/B/C/J 自动辨识状态机，独立上下文 `gimbal_sysid`，含 Pitch/Yaw 双轴 |
| **辨识配置** | `application/inc/GimbalSystemIDConfig.h` | **新增**：选择辨识轴/步骤（`GIMBAL_SYSID` / `GIMBAL_SYSID_STEP`）、 Pitch 老/新云台档案（OLD/NEW）、安全角度与运动参数 |

**config/ 层（配置）**

| 文件 | 职责 |
|------|------|
| `config/robot_config.h` | 机型选择：`#define ROBOT TIGER` |
| `config/can_config.h` | 所有 CAN ID、CAN1/CAN2 滤波器布局 |

**components/ 层（可复用组件，按文件名归类）**

- `algorithm/`：`pid`（PID 控制器）、`QuaternionEKF`（四元数 EKF 姿态解算）、`kalman_filter`、`TD`（跟踪微分器）、`my_filter`、`Observer`、`RLS_Identification` / `SystemIdentification`（系统辨识）、`CRC`/`algorithmOfCRC`（校验）、`user_lib`、`SignalGenerator`
- `devices/`：`BMI088`（六轴 IMU：driver/middleware/reg）、`ist8310driver`（磁力计）
- `motor/`：`DM_Motor`（达妙 MIT 模式）、`GM6020`、`M3508`、`M2006`、`Motor_Typdef`（电机类型枚举与通信配置）
- `tools/`：`debug`、`struct_typedef`、`tools`、`ZeroCheck`（过零检测）

**bsp/ 层（板级支持，按文件名归类）**

- `boards/`：`bsp_can`（CAN 收发/滤波器/接收回调 + `Can1SendSentryCmd`/`Can1SendMapPath`/`Can1SendCustomInfo` 下行转发）、`bsp_rc`（遥控器 WBUS 接收）、`bsp_dwt`（DWT 精确计时）、`bsp_PWM`、`bsp_spi`、`bsp_buzzer`、`bsp_led`
- `usb/`：`usb_device` / `usbd_cdc_if` / `usbd_conf` / `usbd_desc`（USB CDC 虚拟串口，与上位机通信）

**tools/ 层（离线标定工具，2026-07-18 新增）**

- `fit_yaw_feedforward.py`：大 Yaw 前馈参数离线最小二乘拟合脚本。`--mode bc` 拟合 B/C（含 bias 与摩擦淡入点 blend），`--mode j` 在已知 B/C 下拟合 J；输入 CSV 至少包含 `yaw_speed_dps, yaw_accel_dps2, yaw_torque_model_mnm` 三列。使用方法详见 `YAW_FEEDFORWARD_CALIBRATION.md`。

**Pitch_Limit_Oscillation/ 目录（2026-07-16 新增）**

- `analyze_oscillation.py` + 两份 CSV 数据 + 两张 PNG 图：分析 Pitch 轴在上下限位处的角度环/速度环震荡问题。

**Gravity_Compensation/ 目录（既有）**

- `gimbal_pitch_comp_v2.c` / `eda_v2_fit.py` 等：Pitch 重力补偿多项式拟合（Poly3 + 消除项）与 EDA 分析。

---

## 二、控制逻辑脉络

### 2.1 FreeRTOS 任务拓扑

`MX_FREERTOS_Init`（`Src/freertos.c`）中创建的任务：

| 任务 | 入口函数 | 栈 | 频率 | 优先级 | 作用 |
|------|----------|-----|------|--------|------|
| INSTask | `INS_task` | 512 | — | Realtime | BMI088 + IST8310 + EKF 姿态解算 |
| ActionTask | `ActionTask` | 256 | 250Hz | Realtime | 解析遥控/蓝牙，更新机器人状态与各模式 |
| GimbalTask | `GimbalTask` | 512 | 500Hz | Realtime | **云台控制主循环** |
| ChassisTask | `ChassisTask` | 512 | 1000Hz | Realtime | 向底盘发送速度(0x150)/坐标(0x151) |
| Offline_task | `Offline_task` | 128 | 1Hz | — | 各设备离线检测 |

> 注释未启用：`ShootTask`（射击逻辑改由 GimbalTask 内 `Shoot_Cal()` 调用）、`iwdgTask`、`SDCardTask`、`test`。

### 2.2 数据流总览

```
                ┌──────────────┐
   遥控器 WBUS  │              │
  ────────────▶ │  ActionTask  │  get_control_info()
   / 蓝牙       │  (250Hz)     │  ├─ DJIRemoteUpdate()
                └──────┬───────┘  ├─ DJIKeyMouseUpdate()
                       │          └─ blueToothStateUpdate()
                       │  写入
                       ▼
        remote_controller.{gimbal_action, shoot_action, ...}
        gimbal_controller.{target_pitch_angle, target_big_yaw_angle}
                       │
   上位机 USB          │  PCStateControl() 覆盖(自瞄/导航)
  ────────────────────┤
                       ▼
                ┌──────────────┐        ┌────────────────┐
   IMU 姿态     │  GimbalTask  │        │  各 Cal 函数    │
  ────────────▶ │  (500Hz)     │──────▶ │  Pitch/BigYaw   │  PID 三环 + 前馈
   gyro_*       │              │        │  Shoot          │
                └──────┬───────┘        └────────────────┘
                       │ Motor_Data_Pack() → t_ff / 电流
                       ▼
        ┌──────────────────────────────────┐
        │  CAN1: Big Yaw(0x05) / 拨盘(0x204) │
        │  CAN2: Pitch(0x06) / 摩擦轮        │
        └──────────────────────────────────┘
                       │ PC_Send(index)
                       ▼
                上位机(USB) / 底盘(CAN 0x150/0x151)
```

### 2.3 GimbalTask 主循环（500Hz，`xFrequency = 2`）

初始化：`FrictionWheel_Init() → GimbalPidInit() → TogglePidInit() → DM_Motor_Init() → GimbalSystemID_Init(&gimbal_controller) → GimbalTestInit()`

循环体：

```
updataSensors()            // 更新 IMU / 电机反馈
#if GIMBAL_SYSID
if (!gimbal_sysid.yaw.sysid_done || !gimbal_sysid.pitch.sysid_done)
    GimbalSystemID_Run()   // 推进系统辨识状态机（在控制模式计算前）
#endif
Big_Yaw_Bias_Cal()         // 大 Yaw 陀螺零偏解算
Schmitt_PID_changer()      // 施密特触发式 PID 参数切换
switch (remote_controller.gimbal_action) {
    GIMBAL_POWERDOWN     → 掉电
    GIMBAL_ACT_MODE      → 手动：Gimbal_Pitch_Calculate() + Gimbal_Big_Yaw_Calculate()
    GIMBAL_AUTO_AIM_MODE → 自瞄：目标角来自 pc_pitch / pc_yaw
    GIMBAL_SMALL_BUFF / BIG_BUFF_MODE → 小/大符
    GIMBAL_SI_MODE       → 系统辨识（Gimbal_SI_Cal() 当前为空，安全默认 GIMBAL_SYSID_DISABLED）
    GIMBAL_TEST_MODE     → 测试
}
Shoot_Cal()                // 摩擦轮 + 拨盘
Motor_Data_Pack()          // 按电机类型打包 CAN 帧（DM 电机设置 t_ff = 速度PID输出 + 模型前馈）
Motor_Data_Send_CAN1/2()   // 下发
PC_Send(index)             // 定时向上位机回传各类数据帧
```

> **2026-07-18 新增**：`GimbalSystemID_Init/Run` 来自 `crossing_hole-main` 移植，与普通控制器完全解耦——辨识状态保存在独立上下文 `gimbal_sysid`（不进入 `GimbalController`），辨识时强制 DM MIT `Kp/Kd/V_des=0`，输出仅走"辨识状态机速度参考 → 专用速度 PID → t_ff"。默认 `GIMBAL_SYSID_DISABLED` + `GIMBAL_SYSID_PORTING_CONFIRMED 0`，需手动把 `sysid_done` 改 0 才会运动。详见 `application/GimbalSystemID_PORTING.md`。

---

## 三、重要模块

### 3.1 云台 YAW 与 Pitch 轴控制逻辑

两轴均采用**达妙 DM 电机 MIT 模式**驱动，控制结构相同：**角度环 → 速度环 → （力矩/电流）** 三环串级 + 前馈。

#### 电机分配

| 轴 | 电机 | 所在 CAN | 接收 ID | 发送 ID | 控制方式 |
|----|------|----------|---------|---------|----------|
| Pitch | DM 电机 | **CAN2** | 0x11 | 0x06 | MIT 模式 |
| Big Yaw | DM 电机 | **CAN1** | 0x10 | 0x05 | MIT 模式 |

> **小 Yaw 已删除**：原小 Yaw 相关代码以 `// [SMALL_YAW_REMOVED]` 注释保留，目标角变量由 `target_small_yaw_angle` 统一改为 `target_big_yaw_angle`（详见 `SMALL_YAW_REMOVAL.md`）。

#### Pitch 轴（`Gimbal_Pitch_Calculate`）

- 反馈：IMU 解算的 `gyro_pitch_angle`（绝对角度） + 电机速度
- 目标：`gimbal_controller.target_pitch_angle`，经 `limitPitchAngle()` 限幅
- 控制：角度环输出速度期望 → 速度环输出力矩 → 叠加重力补偿前馈（见 `Gravity_Compensation/`）
- **DM MIT 参数**：Kp = 0（纯力矩控制，避免上电下冲），Kd = 5（保留阻尼）—— 见 `DEBUG_RECORD.md`

#### Big Yaw 轴（`Gimbal_Big_Yaw_Calculate`）

- 反馈：`big_yaw_controller` 中大 Yaw 陀螺数据（CAN2 0x166），经 `Big_Yaw_Bias_Cal()` 去零偏得 `dealed_big_yaw_gyro`
- 目标：`gimbal_controller.target_big_yaw_angle`
- **PID 参数**（见 `SMALL_YAW_REMOVAL.md`）：角度环 Kp = 15 / MaxOut = 120，速度环 Kp = 25 / MaxOut = 1500
- **大 Yaw 模型前馈**（2026-07-18 新增）：在速度 PID 输出上叠加基于 J/B/C 物理模型的力矩前馈 `u_ff`，最终统一乘 `GIMBAL_BIG_YAW_MOTOR_SIGN` 后写入 `DM_Big_Yaw_Motor.t_ff`。模型公式：

  ```text
  u_ff = gain * (J * alpha_ref + B * omega_ref + C * sat(omega_ref / friction_blend_dps))
  ```

  - `omega_ref` / `alpha_ref` 取自 `pos_big_yaw_td.dx / ddx`（参考速度/加速度，非实测）
  - 摩擦项 `C * sat(...)` 在 `±friction_blend_dps` 内线性淡入，零速时为 0
  - 模型输出有独立 `MAXOUT`；掉线清零、恢复首帧同步和大 Yaw 缓启动仍然有效
  - 参数入口在 `config/gimbal_config.h`，TIGER 当前实测值（三次自动辨识平均）：`J=1.346, B=0.0, C=325.5, blend=6 deg/s, gain=1.0`
  - 控制量单位为 DM 驱动的 mN·m 编码单位（DM 打包前 `t_ff/1000`）；详见 `YAW_FEEDFORWARD_CALIBRATION.md`

#### 辅助算法

- **TD 跟踪微分器**（`components/algorithm/TD`）：对目标角度做平滑微分，抑制阶跃冲击；当前 Yaw TD 参数 `r=20000, h0=0.01`
- **Schmitt_PID_changer**：根据误差大小施密特式切换 PID 参数组
- **前馈 / 重力补偿**：
  - **Pitch 轴**：使用多项式重力补偿（`PITCH_GRAVITY_COMP_ENABLE`，见 `Gravity_Compensation/`），旧通用速度前馈已被辨识得到的 Pitch J/B/C 物理模型接口替代（当前普通控制仍用多项式，辨识结果保留在 `GIMBAL_PITCH_SIN/COS/B/C/J` 宏中）
  - **Big Yaw 轴**：J/B/C 物理模型前馈（见上文）
- **系统辨识**（2026-07-18 新增）：`GimbalSystemID` 模块提供 Pitch（GRAVITY→BC→J）和 Yaw（BC→J）自动辨识；辨识时强制 DM MIT `Kp/Kd/V_des=0`，输出走专用速度 PID；默认禁用，详见 `application/GimbalSystemID_PORTING.md`

### 3.2 两路 CAN 通信

物理层：CAN1 = `hcan1`，CAN2 = `hcan2`，波特率均 1Mbps；配置集中在 `config/can_config.h`，收发与滤波器在 `bsp/boards/bsp_can`。

#### CAN1 挂载设备

| 设备 | 接收 ID | 发送 ID | 备注 |
|------|---------|---------|------|
| Big Yaw DM 电机 | 0x10 | 0x05 | 云台旋转 |
| 拨盘 M2006 | 0x204 | — | 供弹 |

> ⚠️ **遗留硬件问题**（`DEBUG_RECORD.md`）：Big Yaw 电机曾因 PCB 接线未接到 CAN1，导致 CAN1 物理层断开（TEC=255），需查原理图修正。

#### CAN2 挂载设备

| 设备 | 接收 ID | 发送 ID | 备注 |
|------|---------|---------|------|
| Pitch DM 电机 | 0x11 | 0x06 | 俯仰 |
| 摩擦轮 ×2 | 0x201 / 0x202 | — | 发射 |
| 大 Yaw 陀螺 | 0x166 | — | 大 Yaw 绝对角 |

#### 与底盘的 CAN 通信

| 方向 | CAN ID | 内容 |
|------|--------|------|
| 云台 → 底盘 | 0x150 | `Pack_InfantryMode()`：机器人状态 + yaw 角 + 速度 x/y/w（500Hz） |
| 云台 → 底盘 | 0x151 | `Pack_SentryCoord()`：哨兵坐标 x/y cm（10Hz，来自下行 0x04） |
| 云台 → 底盘 | 0x152 | `Can1SendMapPath()`：map_data 分段传输（105B 分 15 帧，来自下行 0x02，2026-07-18 新增） |
| 云台 → 底盘 | 0x15A | `Can1SendSentryCmd()`：SentryCmd 命令字（来自下行 0x01） |
| 底盘 → 云台 | 0x160 | `GimbalSendPack_1`：底盘状态（射击权限/阵营/电容/弹速，20Hz） |
| 底盘 → 云台 | 0x094 / 0x096 | 裁判数据（0x094 的 `Enemy_outpost` 数据源改 0x0003，均 20Hz） |
| 底盘 → 云台 | 0x097 | 血量（10Hz，敌方 `ID8` 基地血量启用，来自 0x0003） |
| 底盘 → 云台 | 0x098 / 0x099 / 0x09A | RFID/Buff(~9.5Hz) / 位置轮询(~9.5Hz，10 帧轮询 5 友+5 敌 IDs 1/2/3/4/7) / 底盘速度(20Hz) |
| 底盘 → 云台 | 0x09B / 0x09C / 0x09D | 射击(10Hz) / 哨兵信息(10Hz) / 子弹扩展(10Hz) |
| 底盘 → 云台 | 0x09E | 小地图机器人指令（`RobotCommand_ForSend`，事件触发 ≤10Hz，见 `CAN_0x09E_ROBOT_CMD.md`） |
| 底盘 → 云台 | **0x09F** | **`SentryDuration`：6 种哨兵姿态剩余秒数（0x020D 扩展，10Hz，新增）** |
| 底盘 → 云台 | **0x0A0** | **`DamageDiff`：伤害值差 int16（己方−敌方 HP 总和，0x0003 扩展，10Hz，新增）** |
| 底盘 → 云台 | 0x0A1 / 0x0A2 / 0x0A3 | `MotorOffline` / `UwbSteer` / `OutpostHP`：**发送函数已定义但 `JudgeDataCanSend` 未调度**（详见 `通信链路完整梳理.md` §9.2.4） |

> **2026-07-13 底盘 CAN 协议变更（基于裁判 V2.0.0）**，详见 `底盘CAN协议变更_云台适配说明_20260713.md`：
> - 新增 **0x09F** / **0x0A0** 两帧，CAN 滤波器已添加对应 ID（`config/can_config.h`）。
> - **0x09C** 哨兵信息帧 `sentry_info_2` 的 bit 15 由 `sentry_reserved2` 更名语义为 `sentry_is_enhanced_posture`（当前是否为强化姿态）。
> - **0x094** 的 `Enemy_outpost`（敌方前哨站血量，6bit）数据源由雷达改为 0x0003，解压公式 `HP = value/0.04 - 24`，此前注释为 0，现已启用。
> - **0x097** 的 `ID8`（敌方基地血量，6bit）此前硬编码 0，现从 0x0003 取 `enemy_base_HP/100`，实际需 ×100 还原。
> - 0x150/0x151/0x160/0x096/0x098/0x099/0x09A/0x09B/0x09D/0x09E/0x15A 均不变。

接收结构体见 `application/inc/ChassisGet.h`（`SentryDuration_t` / `DamageDiff_t` / `SentryInfoRecv_t`），全局变量 `sentry_duration` / `damage_diff` / `sentry_info_recv` 由 `ChassisGet.c` 写入。

#### 打包与下发

`Motor_Data_Pack()` 遍历 `motor_communication[]`，按电机类型（GM6020 / M3508 / M2006 / DM_MOTOR）分别打包；DM 电机设置 `t_ff`（前馈力矩）并调用 `DM_Motor_Control(..., DM_MIT_CONTROL)`，随后 `Motor_Data_Send_CAN1/2()` 发送。

### 3.3 云台与上位机通信

物理层：**USB CDC 虚拟串口**（`bsp/usb/usbd_cdc_if`）。协议选择 `COMMUNICATION_CHOOSE == COMMUNICATION_OF_SENTRY`，定义于 `application/inc/pc_serial.h`。

> **⚠️ 2026-07-11/12 下行协议重大重构（破坏性，需上位机同步切换）**：下行帧由旧版 2 种（0x00 控制 18B / 0x01 坐标 18B）重构为 5 种分包帧（DownlinkTypeID 0x00~0x04）。2026-07-18 以兼容方式新增 `0x05` MPC 轨迹帧，旧 `0x00` 仍为 13B；上行新增 TypeID 11 动态反馈。详见 `2026-07-11_上位机下发协议总览.md`。

#### 下行帧（上位机 → 云台，按 `DownlinkTypeID` 分发）

`PCReceive()` 改为 `switch(PCbuffer[1])` 按类型分发（`application/src/pc_serial.c`）。接收缓冲 `PCbuffer` 已扩至 **107B**（取最大帧 `MapPathFrame`）。所有下行帧 byte0 固定 `0x21`（`'!'`）、byte1 为 `DownlinkTypeID`：

| DownlinkTypeID | 帧名 | 总长 | 内容与去向 |
|----------------|------|------|-----------|
| `0x00` | `GimbalControlFrame` | 13B | 旧控制帧保持不变，继续承载底盘速度、角度和 FireCode |
| `0x01` | `SentryCommandFrame` | 6B | `sentry_cmd`(uint32 LE，RM2026 V2.0 0x0120) → `Can1SendSentryCmd()` 转发 CAN 0x15A；bit21-23 提取 `current_posture`(1~6) |
| `0x02` | `MapPathFragment` | 64B ×2 | **2026-07-18 协议变更**：上位机因 USB FS 单包 64B 限制，将原 107B MapPathFrame 拆为两个固定 64B 分片（fragment 0/1）。云台板 `MapPath_OnFragment()` 重组为 105B `map_data_t` payload → `Can1SendMapPath()` 通过 CAN 0x152 分 15 帧转发底盘 → 底盘封装 `0x0307` 发送裁判系统。详见 `map_path_fragment_reassembly.md` 与 `底盘侧map_data转发实现说明.md` |
| `0x03` | `CustomInfoFrame` | 36B | `0x0308 custom_info_t` 原样转发 → `Can1SendCustomInfo()`（已实现，采用 generation 双缓冲快照，`Can1ServiceCustomInfoTx()` 在 ChassisTask 中逐段发送 CAN 0x153 分 5 帧） |
| `0x04` | `SentryCoordinateFrame` | 17B | `x_cm/y_cm(int16 LE) + reserved[10] + crc8(poly=0x31,init=0xFF)` → 写 `sentry_position_x/y_cm`，转发底盘 CAN 0x151 |
| `0x05` | `GimbalTrajectoryFrame` | 26B | MPC 原子轨迹：Yaw/Pitch 的角度、角速度、角加速度（6×float32 LE） |

> **关键变更**：旧协议的 `0x01` 是坐标帧，新协议 `0x01` 是 **SentryCmd** —— 语义完全不同，**旧上位机 + 新固件会互相错解**；`SentryCmd` 不再夹在控制帧尾部，独立成 0x01。SentryCmd 姿态位宽由 bit21-22(1~3) 扩为 bit21-23(1~6)，能量机关确认位由 bit23 移到 bit24（基于 RM2026 V2.0）。

#### 上行帧（云台 → 上位机，`PC_Send(index)` 定时）

上行仍统一为 **15B**（1 head + 1 type + 12 data + 1 CRC8），`PC_dataType_enum`（USUAL_PC_DATA=0 … GIMBAL_DYNAMICS=11）分类，各帧按 index 取模控制频率：

| TypeID | 帧名 | 结构体 | 频率 | 备注 |
|--------|------|--------|------|------|
| 0 | 常规数据 | `PCSendData` | 250Hz | yaw/pitch/电量/弹量 |
| 1 | 裁判数据 | `PCSendDataJudge` | 25Hz | `Enemy_outpost` 数据源改 0x0003（见 §3.2） |
| 2 / 3 | 友/敌血量 | `PCSendDataBlood_1/2` | 10Hz | `E_base/ID8` 敌方基地血量启用（见 §3.2） |
| 4 | RFID/Buff | `PCSendDataRFIDAndBuff` | 5Hz | |
| 5 | 坐标 | `PCSendDataPosition` | 10Hz | |
| 6 | 扩展 | `PCSendDataExtended` | 10Hz | **`damage_difference` 来自新 CAN 0x0A0；姿态回读已移至 TypeID 7，本帧 `sentry_posture` 置 0** |
| 7 | 哨兵信息 | `PCSendDataSentry` | 10Hz | 0x020D + 弹速；bit15 = `sentry_is_enhanced_posture` |
| 8 | 弹量+RFID2 | `PCSendDataBulletAndRfid2` | 10Hz | |
| 9 | 小地图指令 | `PCSendDataRobotCmd` | 0.5Hz | 0x0303 原样转发 |
| 10 | 哨兵姿态时长 | `PCSendDataSentryDuration` | 10Hz | **新增**，来自新 CAN 0x09F（`index%50==45`） |
| 11 | 云台动态反馈 | `PCSendDataGimbalDynamics` | 约90~100Hz | Yaw/Pitch 角速度×10、角加速度×1、MCU tick、CRC8 |

> 上行 TypeID 0~10 帧结构不变；新增 TypeID 11 反馈角速度和角加速度。角度仍由 TypeID 0 以原频率反馈，消费者及原变量保持不变。

### 3.4 云台与遥控器遥控

物理层：**WBUS 遥控器**（USART3，100kbps，DMA 双缓冲，`bsp/boards/bsp_rc`）。数据结构与枚举定义于 `application/inc/remote_control.h`，分发逻辑在 `application/src/ChassisSolver.c`。

#### 控制入口

`ActionTask`（250Hz）循环调用 `get_control_info(&chassis_solver)`，按 `remote_controller.control_type` 分发：

| control_type | 处理函数 | 说明 |
|--------------|----------|------|
| DJI | `DJIRemoteUpdate()` | 摇杆 + 拨杆 |
| KEY_MOUSE | `DJIKeyMouseUpdate()` | 键鼠 |
| BLUE_TOOTH | `blueToothStateUpdate()` | 蓝牙 |

#### DJIRemoteUpdate（摇杆/拨杆）

- **左/右拨杆（s[2]）**：决定机器人状态、底盘模式、射击模式、云台模式（`GIMBAL_ACTION` / `SHOOT_ACTION` / `CONTROL_MODE_ACTION`）
- **摇杆（ch[4]）**：直接累加/设定 `gimbal_controller.target_big_yaw_angle` 与 `target_pitch_angle`

> **物理映射变更（见 `遥控器模式映射表.md`，2026-07-13）**：原三位拨杆 CH6 已损坏，**左拨杆 `LEFT_SW` 改由 CH5(SA) + CH8(SD) 合成**——CH5=Down 强制 `LEFT_SW=Down`；CH5=Up 且 CH8=Up → `LEFT_SW=Up`（遥控手动）；CH5=Up 且 CH8=Down → `LEFT_SW=Mid`（NUC/PC 模式）。右拨杆 `RIGHT_SW` 仍为 CH7(SC)。

#### DJIKeyMouseUpdate（键鼠）

- 鼠标 X/Y → Yaw / Pitch 目标角（系数 0.005°）；滚轮 Z → Pitch 微调
- 左键 → 开火（单击单发 / 按住连发）；右键 → 进入/退出自瞄；R+Z/X → 小/大符；E 松开 → Yaw+180°；Shift → 超级电容；Ctrl → 小陀螺；Q → 切换底盘形态；W/A/S/D → 底盘速度
- 键鼠模式经 WBUS 自动识别，与 `LEFT_SW=Mid/Up` 档位共存；`RIGHT_SW` 非 Mid 时切回遥控器模式

#### 模式枚举（remote_control.h）

- `GIMBAL_ACTION`（7 种）：POWERDOWN / ACT / AUTO_AIM / SMALL_BUFF / BIG_BUFF / SI / TEST
- `SHOOT_ACTION`（8 种）：POWERDOWN / CHECK / FIRE / TEST / AUTO_AIM / SUPPLY / **UNSTOPPABLE（双杆连射）** / **UNSTOPPABLE_AUTO_AIM（强化进攻姿态下的高速辅瞄射击，2026-07-19 新增）**
- `CONTROL_MODE_ACTION`（含 NAVIGATION_MODE）：NOT_CONTROL / NOT_FOLLOW_GIMBAL / FOLLOW_GIMBAL / CV_ROTATE / CHANGE_SPEED_FOLLOW / SPEED_FOLLOW / **NAVIGATION_MODE（导航模式，新增）**

> 完整 9 档位矩阵（SA/SD/SC 组合 → 云台/射击/底盘模式）、UNSTOPPABLE 触发条件、NUC 与导航模式区分（RIGHT_SW 是否由 Down 切换）等，详见 `遥控器模式映射表.md`。

##### UNSTOPPABLE 模式（2026-07-19 修订）

进入 `SHOOT_UNSTOPPABLE_MODE` 后由 `Shoot_Unstoppable_Cal()` 处理（`Task/src/ShootTask.c`），与普通 `SHOOT_FIRE_MODE` 的差异：

- **摩擦轮转速补偿**：在 `setFrictionSpeed(bullet_level)` 设定基础目标后，对 `friction_wheels.set_speed_l/r` 叠加 `UNSTOPPABLE_FRICTION_BOOST`（`application/inc/FrictionWheel.h`，当前 2000 deg/s），抵消高弹频下子弹持续摩擦造成的摩擦轮转速下沉，维持实际弹速。叠加方向与 `FrictionWheel_Set(-l, +r)` 的符号约定一致（`set_speed_l` 减小、`set_speed_r` 减小，左负右正在调用处取反）。
- **拨盘速度模式连转**：`toggle_controller.shoot_freq_speed = 3000.0f`（M2006 输出轴空载最高 500rpm = 3000°/s），用 `TOGGLE_SPEED` 速度环连续转动，配合 `autoReverse()` 自动反拨检测。

##### UNSTOPPABLE 触发逻辑（2026-07-19 修订）

`LEFT_SW=Up, RIGHT_SW=Up`（手动打弹档）下的两段式触发（`application/src/ChassisSolver.c` case `Up`）：

| 物理左摇杆上下（= `RIGHT_CH_UD` 偏差） | 射击模式 | `is_shoot` | 行为 |
|----------------------------------------|----------|-----------|------|
| > 600（接近到底） | `SHOOT_UNSTOPPABLE_MODE` | — | 连续高速射击 + 摩擦轮补偿 |
| 330 ~ 600（过半未到底） | `SHOOT_FIRE_MODE` | `TRUE` | 拨盘单发 |
| < 330（居中） | `SHOOT_FIRE_MODE` | `FALSE` | 待机 |

> **物理摇杆 ↔ 代码通道映射**（WBUS 接收机接线决定，与 DJI 遥控器左右约定相反，详见 `application/src/wbus_decoder.c:WBUS_UpdateRemoteController()`）：
> - **物理右摇杆** → `rc.ch[LEFT_CH_*]`（CH1/CH2）→ 控云台 Yaw/Pitch 目标角
> - **物理左摇杆** → `rc.ch[RIGHT_CH_*]`（CH3/CH4）→ 控底盘 x/y 速度与 `is_shoot`/UNSTOPPABLE 触发
>
> 因此 UNSTOPPABLE 触发用的是 `RIGHT_CH_UD`（代码命名），但物理上是**左摇杆**上推。
>
> **修订原因**：原触发条件要求"左右摇杆任一轴都 > 330"，意味着瞄准用的物理右摇杆也得推到一半以上才能进入 UNSTOPPABLE，与瞄准目标方向冲突。新逻辑让物理右摇杆完全解放，瞄准与连射可同时进行。

#### 上位机接管

`PCStateControl()`（`LEFT_SW=Mid, RIGHT_SW=Mid/Down` 比赛 / NUC 模式）覆盖遥控输入：设置 `chassis_speed_x/y/w`（来自 `NAV_cmd`）、`setGimbalAction(GIMBAL_AUTO_AIM_MODE)` 切自瞄；并通过 `FireCode` 的 bit[7:6]（`RotateState` 0~3）选择底盘旋转速度分档（0/8/12/16 rad/s，MAX=16），由 `PC_statecontrol` 写入。翻车保护：Pitch > 80° 或 Roll > 60° → `setAllModeOff()`。

##### 射击模式按 `current_posture` 分流（2026-07-19 新增）

`PCStateControl()` 内部根据下行 `0x01` 帧的 `sentry_cmd` bit21-23 提取的 `current_posture`（1~6 哨兵姿态，1=进攻 / 2=防御 / 3=移动 / 4=强化进攻 / 5=强化防御 / 6=强化移动）选择射击模式：

| `current_posture` | 射击模式 | 弹频 (Hz) | 说明 |
|--------------------|----------|-----------|------|
| 4（强化进攻） | `SHOOT_UNSTOPPABLE_AUTO_AIM_MODE` | ~50 | 复用 `Shoot_Autoaim_Cal()` 全部辅瞄判定逻辑，仅在 `Shoot_Pos_Cal()` 内部取 `Shoot_IntervalTime=10`（20ms 一发） |
| 其他（1/2/3/5/6/0） | `SHOOT_AUTO_AIM_MODE` | ~20 | 原行为不变，`Shoot_IntervalTime=25`（50ms 一发） |

> **设计要点**：
> - 新模式只是弹频档位不同，**保留全部辅瞄安全机制**（FireCode 沿检测、`AA_Shootable` 判定、PC 掉线安全停 `Shoot_AutoaimFrictionStandby_Cal`）——不会因为高速射击就跳过瞄准判定浪费子弹。
> - `Shoot_Cal()` 头部的辅瞄上下文清零条件已扩展为 `!= SHOOT_AUTO_AIM_MODE && != SHOOT_UNSTOPPABLE_AUTO_AIM_MODE`，保证两模式之间切换不丢开火沿。
> - 拨盘位置环 `ToggleAddGrid(&set_pos, 1)` 在 `shoot_pos_delay_num > Shoot_IntervalTime` 时步进一格；`Shoot_IntervalTime=10` 对应 20ms，约 50Hz，接近 M2006 输出轴空载上限（500rpm=3000°/s，36°/发≈83Hz）的 60%。
> - 掉线恢复首周期会被 `Shoot_Autoaim_Cal()` 内部 `pc_shoot_was_online==0` 分支拦截（`Shoot_AutoaimFrictionStandby_Cal()`），即使 `current_posture==4` 也不会立即开火。

---

## 四、重要的全局变量 / 结构体

### 4.1 `gimbal_controller`（最核心）

```c
extern GimbalController gimbal_controller;   // application/inc/Gimbal.h
```

云台控制的**唯一核心枢纽**，几乎所有控制数据都流经它：

| 分组 | 关键字段 | 说明 |
|------|----------|------|
| **Pitch 控制** | Pitch 三环 PID（角度/速度/电流）、`DM_Pitch_Motor`（DM_MIT） | 俯仰控制器 |
| **Big Yaw 控制** | Big Yaw 三环 PID、`DM_Big_Yaw_Motor`（DM_MIT） | 偏航控制器 |
| **目标角度** | `target_pitch_angle`、`target_big_yaw_angle` | 由遥控/上位机写入（原 `target_small_yaw_angle` 已废弃） |
| **反馈角度** | `gyro_pitch_angle`、`gyro_yaw_angle` | IMU 姿态解算结果 |
| **微分器** | TD 跟踪微分器 | 目标角平滑 |
| **大 Yaw 模型前馈**（2026-07-18 新增） | `big_yaw_ff_ref_speed_dps` / `big_yaw_ff_ref_accel_dps2` / `big_yaw_ff_inertia` / `big_yaw_ff_viscous` / `big_yaw_ff_coulomb` / `big_yaw_ff_output` | 参考速度/加速度、J/B/C 三项贡献、最终前馈输出（mN·m 编码单位） |
| **测试** | `gimbal_test` | 测试模式子模块 |

> 注：SMALL_YAW 相关字段以注释形式保留，未实际使用。系统辨识状态保存在独立全局变量 `gimbal_sysid`（`application/inc/GimbalSystemID.h`），不进入 `gimbal_controller`。

### 4.2 `big_yaw_controller`

```c
extern BigYawController big_yaw_controller;   // application/inc/Gimbal.h
```

专门处理大 Yaw 陀螺（CAN2 0x166）的角度/零偏：

| 字段 | 说明 |
|------|------|
| `big_yaw_gyro_raw` / `big_yaw_speed` | 原始角度 / 角速度 |
| `big_yaw_gyro_bias` | 零偏 |
| `dealed_big_yaw_gyro` | 去零偏后的可用角度 |
| `gimbal_last_mode` / `big_yaw_mode` | 模式记录与切换 |

### 4.3 `remote_controller`

```c
RemoteController remote_controller;   // application/inc/remote_control.h
```

遥控与状态机总线：`control_type` / `robot_state` / `gimbal_action` / `shoot_action` / `control_mode_action` / `dji_remote(RC_Ctl_t)`。GimbalTask 的模式分支即依据 `gimbal_action`。

### 4.4 上位机通信变量

```c
extern float pc_pitch, pc_yaw;        // 自瞄目标角（下行 0x00，pc_serial.h）
extern Nav_Cmd_t NAV_cmd;             // 导航速度（下行 0x00 的 vel_x/vel_y / 50）
extern uint8_t current_posture;       // 当前姿态 1~6（下行 0x01 的 SentryCmd bit21-23）
extern PC_StateControl PC_statecontrol; // CapState / if_through_hole / RotateState / if_target_in_view（下行 0x00 的 fire_code 解码）
extern int16_t sentry_position_x_cm, sentry_position_y_cm; // 自身坐标（下行 0x04，转发底盘 CAN 0x151）
```

底盘 CAN 上行新变量（`application/inc/ChassisGet.h`，由 `ChassisGet.c` 写入）：

```c
extern SentryDuration_t sentry_duration;  // CAN 0x09F：6 种哨兵姿态剩余秒数
extern DamageDiff_t     damage_diff;      // CAN 0x0A0：伤害值差 int16（正值=己方领先）
extern SentryInfoRecv_t sentry_info_recv; // CAN 0x09C：含 bit15 = sentry_is_enhanced_posture
```

> `sentry_duration` / `damage_diff` 经 `PC_Send` 上行（TypeID 10 / TypeID 6）；`current_posture` / `PC_statecontrol` 由 `PCReceive` 解析下行帧写入。

### 4.5 其它

- `offline_detector`：各设备（IMU/遥控/Pitch/Yaw/摩擦轮/拨盘/PC）离线标志，由 `Offline_task` 维护
- `motor_communication[]`：电机通信配置数组，`Motor_Data_Pack()` 遍历打包
- `chassis_solver`：控制求解上下文，贯穿 `get_control_info` 链路

---

## 五、附录：根目录文档索引

### 5.1 云台根目录文档（`Sentry_Steer/Gimbal/`）

| 文档 | 内容 |
|------|------|
| `哨兵秘诀.md` | 达妙电机调试记录（CAN ID / Master ID / 使能） |
| `INTEGRATION.md` | 哨兵坐标下发（ROS → 串口帧，CRC8 poly=0x31；开发验证记录已并入，TypeID 现归 0x04） |
| `CAN_0x09E_ROBOT_CMD.md` | CAN 0x09E 小地图指令（裁判 0x0303 → 底盘 → 云台） |
| `SMALL_YAW_REMOVAL.md` | 小 Yaw 删除记录 + Big Yaw 数据流 + PID 参数 |
| `DEBUG_RECORD.md` | WBUS 映射、电机 CAN ID、Pitch 调试、CAN1 接线遗留问题 |
| `2026-07-11_上位机下发协议总览.md` | **下行 6 种帧（DownlinkTypeID 0x00~0x05）总表与字段** |
| `协议变更与迁移记录.md` | **2026-07-11/12 协议变更：草案 + 差异分析 + 迁移记录三合一（原始已归档 `docs_archive/`）** |
| `底盘CAN协议变更_云台适配说明_20260713.md` | **底盘 CAN 新增 0x09F/0x0A0、0x09C bit 更名、0x094/0x097 数据源变更** |
| `STM32上行通信适配说明_20260713.md` | **上行 TypeID 10/11 新增发送清单（TypeID 10 已实现，TypeID 11 实际并入 TypeID 6）** |
| `遥控器模式映射表.md` | **物理拨杆→LEFT_SW 合成、9 档位矩阵、FireCode 旋转分档、枚举速查** |
| `YAW_FEEDFORWARD_CALIBRATION.md` | **（2026-07-18 新增）大 Yaw 模型前馈标定流程：J/B/C 实车辨识步骤、数据记录清单、验收标准** |
| `map_path_fragment_reassembly.md` | **（2026-07-16 新增）下行 0x02 MapPathFrame 跨 USB FS 64B 单包的分段重组方案（云台板侧 USB 接收与重组）** |
| `底盘侧map_data转发实现说明.md` | **（2026-07-18 新增）云台→底盘 CAN 0x152 分段传输（105B 分 15 帧）+ 底盘重组 + 封装 0x0307 发送裁判系统的实现规范** |
| `application/GimbalSystemID_PORTING.md` | **（2026-07-18 新增）GimbalSystemID 模块移植说明：状态机、Pitch/Yaw 辨识配置、安全互锁、结果判定** |

### 5.2 仓库根目录文档（`2026_Embedded-Sentry/`）

| 文档 | 内容 |
|------|------|
| `downlink_control_frame.md` | **（2026-07-16 新增）上位机下发协议总览（繁体版）：6 种 DownlinkTypeID 帧结构与 SentryCmd 位语义** |
| `最高优先级问题.md` | **（2026-07-18 新增）云台+底盘代码审查报告：14 项 P0/P1 问题（电机数组越界、前馈不限幅、DM MIT 打包错误、PC/WBUS 看门狗等）** |
| `2026-07-11_protocol_v2_change_brief.md` | 协议 V2 变更简报 |
| `map_command_typeid9.md` | TypeID 9 小地图指令说明 |

### 5.3 归档与其它

> 归档说明：原 `2026-07-11_新一轮通讯协议变更草案.md`、`2026-07-12_协议差异分析.md`、`2026-07-12_下行协议迁移改动记录.md`、`SENTRY_POS.md`、`CHASSIS_COMM_UPDATE.md` 已移入 `docs_archive/`，其有效内容分别并入 `协议变更与迁移记录.md` 与 `INTEGRATION.md`，根目录不再保留重复文档。

> 已清理（2026-07-16）：独立 `Sentry_Steer/BigYaw/` 子项目目录与 `Sentry_Steer/power_cal/` 功率计算工具目录（含 `.idea/`、csv 数据、`power_cal.py`）已从仓库删除，本文档不再涉及。

---

*文档基于 `Task/`、`application/`、`config/` 源码及 `bsp/`、`components/`、`tools/`、`Gravity_Compensation/`、`Pitch_Limit_Oscillation/` 文件清单、根目录 markdown 整理而成。*
