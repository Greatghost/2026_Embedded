# 云台删除小Yaw改动记录

> **日期**: 2026-06-08
> **标记**: 所有改动均以 `// [SMALL_YAW_REMOVED]` 注释标记，可通过搜索该关键词快速定位
> **恢复**: 所有被删除代码通过 `//` 或 `/* */` 保留，取消注释即可恢复

---

## 改动概览

| # | 文件 | 改动类型 |
|---|------|---------|
| 1 | `components/motor/inc/Motor_Typdef.h` | 注释枚举值 |
| 2 | `config/can_config.h` | 注释CAN宏 + 释放滤波器槽位 |
| 3 | `config/gimbal_config.h` | 注释电机方向/限位宏 |
| 4 | `application/inc/Gimbal.h` | 注释结构体字段 + 函数声明 |
| 5 | `config/can_send_config.c` | 注释电机通信配置 |
| 6 | `bsp/boards/src/bsp_can.c` | 注释CAN接收分支 |
| 7 | `application/src/pc_serial.c` | 变量改名 |
| 8 | `application/src/Gimbal.c` | 注释PID/前馈/计算函数/测试模块 |
| 9 | `Task/src/GimbalTask.c` | 目标角度替换 + 控制/解码注释 |
| 10 | `application/src/ChassisSolver.c` | 目标角度替换 |

---

## 1. Motor_Typdef.h

**说明**: `SMALL_YAW_MOTOR` 枚举值注释保留占位，避免 `TOGGLE_MOTOR` 等后续值的索引偏移。

```diff
 typedef enum MOTOR_APP_TYPE
 {
     LEFT_FRICTION_WHEEL_MOTOR,
     RIGHT_FRICTION_WHEEL_MOTOR,
     PITCH_MOTOR,
     BIG_YAW_MOTOR,
-    SMALL_YAW_MOTOR,
+    // [SMALL_YAW_REMOVED] 删除小Yaw, 保留枚举值占位以避免后续索引错乱
+    // SMALL_YAW_MOTOR,
     TOGGLE_MOTOR,
     MOTOR_APP_NUMS
 } MOTOR_APP_TYPE;
```

---

## 2. can_config.h（GOBLIN + TIGER 两段）

**说明**: 小Yaw电机CAN ID宏注释，释放 CAN2 FIFO0 滤波器槽位。

```diff
-// 小Yaw轴电机接收
-#define SMALL_YAW_MOTOR_CAN_ID 0x205 // small_yaw轴电机,RM6020
-#define SMALL_YAW_MOTOR_CAN CAN2
+// [SMALL_YAW_REMOVED] 小Yaw轴电机已删除
+// #define SMALL_YAW_MOTOR_CAN_ID 0x205
+// #define SMALL_YAW_MOTOR_CAN CAN2
```

```diff
-#define CAN2_FIFO0_ID2 SMALL_YAW_MOTOR_CAN_ID
+#define CAN2_FIFO0_ID2 0x000  // [SMALL_YAW_REMOVED] 原为 SMALL_YAW_MOTOR_CAN_ID
```

---

## 3. gimbal_config.h（GOBLIN + TIGER 两段）

**说明**: 小Yaw电机方向、限位、零点等7个宏全部注释。

```diff
-#define GIMBAL_SMALL_YAW_MOTOR_SIGN 1.0f
-#define GIMBAL_SMALL_YAW_GYRO_SIGN 1.0f
-#define GIMBAL_SMALL_YAW_POS_FORWARD_COEF 0.6f
-#define GIMBAL_SMALL_YAW_SPEED_FORWARD_COEF 0.f
-#define GIMBAL_SMALL_YAW_LIMIT_LEFG ...     // GOBLIN:210; TIGER:165
-#define GIMBAL_SMALL_YAW_LIMIT_RIGHT ...    // GOBLIN:150; TIGER:75
-#define GIMBAL_SMALL_YAW_ZERO_POINT ...     // GOBLIN:180; TIGER:120
+// [SMALL_YAW_REMOVED] 小Yaw轴电机宏定义已删除
+// #define GIMBAL_SMALL_YAW_MOTOR_SIGN 1.0f
+// #define GIMBAL_SMALL_YAW_GYRO_SIGN 1.0f
+// #define GIMBAL_SMALL_YAW_POS_FORWARD_COEF 0.6f
+// #define GIMBAL_SMALL_YAW_SPEED_FORWARD_COEF 0.f
+// #define GIMBAL_SMALL_YAW_LIMIT_LEFG ...
+// #define GIMBAL_SMALL_YAW_LIMIT_RIGHT ...
+// #define GIMBAL_SMALL_YAW_ZERO_POINT ...
```

---

## 4. Gimbal.h

**说明**: 结构体中所有小Yaw相关字段和函数声明全部注释。陀螺仪Yaw数据(`gyro_yaw_angle`, `gyro_yaw_speed`)保留，关联到大Yaw。

### 4.1 GimbalTest 结构体

```diff
 typedef struct GimbalTest
 {
     SquareWave pitch_square;
-    SquareWave small_yaw_square;
+    // [SMALL_YAW_REMOVED] 小Yaw方波测试相关字段
+    // SquareWave small_yaw_square;

     CostFunction_t pitch_cost;
-    CostFunction_t yaw_cost;
+    // [SMALL_YAW_REMOVED]
+    // CostFunction_t yaw_cost;

     uint16_t last_pitch_cycle;
-    uint16_t last_yaw_cycle;
+    // [SMALL_YAW_REMOVED]
+    // uint16_t last_yaw_cycle;

     float last_pitch_ise / last_pitch_control / last_pitch_max_error;
-    float last_yaw_ise / last_yaw_control / last_yaw_max_error;
+    // [SMALL_YAW_REMOVED]
+    // float last_yaw_ise / last_yaw_control / last_yaw_max_error;
 } GimbalTest_t;
```

### 4.2 GimbalController 结构体

```diff
-  GM6020_Recv small_yaw_recv;
-  GM6020_Info small_yaw_info;
+  // [SMALL_YAW_REMOVED] 小Yaw电机数据接收已删除
+  // GM6020_Recv small_yaw_recv;
+  // GM6020_Info small_yaw_info;
```

```diff
-  // // SMALL_YAW
-  PID_t small_yaw_current_pid;
-  PID_t small_yaw_speed_pid;
-  PID_t small_yaw_angle_pid;
-  Feedforward_t small_yaw_speed_forward;
-  Feedforward_t small_yaw_angle_forward;
+  // [SMALL_YAW_REMOVED] 小Yaw PID/前馈已删除
+  // PID_t small_yaw_current_pid;
+  // PID_t small_yaw_speed_pid;
+  // PID_t small_yaw_angle_pid;
+  // Feedforward_t small_yaw_speed_forward;
+  // Feedforward_t small_yaw_angle_forward;
```

```diff
-  DM_MIT DM_Small_Yaw_Motor;
+  // [SMALL_YAW_REMOVED] 小Yaw DM电机已删除
+  // DM_MIT DM_Small_Yaw_Motor;
   DM_MIT DM_Big_Yaw_Motor;
```

```diff
-  float set_small_yaw_speed / set_small_yaw_current / set_small_yaw_angle / set_small_yaw_vol;
+  // [SMALL_YAW_REMOVED] 小Yaw控制量已删除
+  // float set_small_yaw_speed / ...
```

```diff
-  float target_small_yaw_angle;
-  TD_t pos_small_yaw_td;
-  TD_t speed_small_yaw_td;
+  // [SMALL_YAW_REMOVED] 小Yaw目标角度和TD已删除
+  // float target_small_yaw_angle;
+  // TD_t pos_small_yaw_td;
+  // TD_t speed_small_yaw_td;
```

### 4.3 函数声明

```diff
-float Gimbal_Small_Yaw_Calculate(float set_point);
+// [SMALL_YAW_REMOVED] 小Yaw计算函数已删除
+// float Gimbal_Small_Yaw_Calculate(float set_point);
```

---

## 5. can_send_config.c（GOBLIN + TIGER 两段）

**说明**: 小Yaw电机通信配置注释，不再向该电机发送CAN控制帧。

```diff
-    motor_communication[SMALL_YAW_MOTOR].can = CAN2;
-    motor_communication[SMALL_YAW_MOTOR].motor_id = 0x205;
-    motor_communication[SMALL_YAW_MOTOR].motor_id_type = DJI_0x1FE;
-    motor_communication[SMALL_YAW_MOTOR].motor_type = GM6020;
-    motor_communication[SMALL_YAW_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x1FE];
+    // [SMALL_YAW_REMOVED] 小Yaw电机通信配置已删除
+    // motor_communication[SMALL_YAW_MOTOR].can = CAN2;
+    // motor_communication[SMALL_YAW_MOTOR].motor_id = 0x205;
+    // motor_communication[SMALL_YAW_MOTOR].motor_id_type = DJI_0x1FE;
+    // motor_communication[SMALL_YAW_MOTOR].motor_type = GM6020;
+    // motor_communication[SMALL_YAW_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x1FE];
```

---

## 6. bsp_can.c

**说明**: 小Yaw电机CAN接收分支用 `/* */` 块注释保留。

```diff
+// [SMALL_YAW_REMOVED] 小Yaw电机CAN接收已删除
+/*
 	else if (hcan->Instance == SMALL_YAW_MOTOR_CAN && rx_header->StdId == SMALL_YAW_MOTOR_CAN_ID)
 	{
 		if (motor_communication[SMALL_YAW_MOTOR].motor_type == GM6020)
 		{ ... }
 		else if (motor_communication[SMALL_YAW_MOTOR].motor_type == DM_MOTOR)
 		{ ... }
 		LossUpdate(&global_debugger.gimbal_debugger[1], 0.0015f);
 	}
+*/
```

---

## 7. pc_serial.c

**说明**: 变量名从 `small_yaw_offset` 改为 `yaw_bias_offset`（数据来源不变，仍为 `big_yaw_gyro_bias`）。

```diff
-	int16_t small_yaw_offset_angle_10 = (int16_t)(big_yaw_controller.big_yaw_gyro_bias * 10.0f);
-	PCSendExtended.gimbal_vel_data1 = ((uint32_t)(small_yaw_offset_angle_10 & 0xFFFF)) | ...
+	// [SMALL_YAW_REMOVED] 变量改名: small_yaw_offset → yaw_bias_offset (数据来源不变)
+	int16_t yaw_bias_offset_angle_10 = (int16_t)(big_yaw_controller.big_yaw_gyro_bias * 10.0f);
+	PCSendExtended.gimbal_vel_data1 = ((uint32_t)(yaw_bias_offset_angle_10 & 0xFFFF)) | ...
```

---

## 8. Gimbal.c（核心修改）

### 8.1 GimbalPidInit() — 小Yaw PID/前馈/TD初始化全部注释

```diff
- // yaw GM6020 CURRENT LOOP
- PID_Init(&gimbal_controller.small_yaw_angle_pid, 100.0, 0, 0.05, ...);
- PID_Init(&gimbal_controller.small_yaw_speed_pid, GM6020_MAX_CURRENT, 5000, ...);
+ // [SMALL_YAW_REMOVED] 小Yaw PID初始化已删除
+ // PID_Init(&gimbal_controller.small_yaw_angle_pid, ...);
+ // PID_Init(&gimbal_controller.small_yaw_speed_pid, ...);
```

```diff
- TD_Init(&gimbal_controller.pos_small_yaw_td, 40000, 0.01);
- TD_Init(&gimbal_controller.speed_small_yaw_td, 90000, 0.01);
+ // [SMALL_YAW_REMOVED] 小Yaw TD初始化已删除
+ // TD_Init(&gimbal_controller.pos_small_yaw_td, 40000, 0.01);
+ // TD_Init(&gimbal_controller.speed_small_yaw_td, 90000, 0.01);
```

```diff
- float small_yaw_angle_ff_c[3] = {0.f, 0.4f, 0.0f};
- float small_yaw_speed_ff_c[3] = {0.4f, 0.f, 0.0f};
- Feedforward_Init(&gimbal_controller.small_yaw_angle_forward, ...);
- Feedforward_Init(&gimbal_controller.small_yaw_speed_forward, ...);
+ // [SMALL_YAW_REMOVED] 小Yaw前馈参数和初始化已删除
+ // float small_yaw_angle_ff_c[3] = {0.f, 0.4f, 0.0f};
+ // float small_yaw_speed_ff_c[3] = {0.4f, 0.f, 0.0f};
+ // Feedforward_Init(&gimbal_controller.small_yaw_angle_forward, ...);
+ // Feedforward_Init(&gimbal_controller.small_yaw_speed_forward, ...);
```

### 8.2 Gimbal_Small_Yaw_Calculate() — 整个函数用 `/* */` 注释

```diff
+// [SMALL_YAW_REMOVED] 小Yaw计算函数已删除，陀螺仪yaw数据改由大Yaw使用
+/*
 // 陀螺仪零漂问题解决，大小yaw可解耦控制
 float Gimbal_Small_Yaw_Calculate(float set_point)
 {
     gimbal_controller.set_small_yaw_angle = TD_Calculate(...);
     gimbal_controller.set_small_yaw_speed = PID_Calculate(...) + Feedforward_Calculate(...);
     gimbal_controller.set_small_yaw_current = GIMBAL_SMALL_YAW_MOTOR_SIGN * (...);
     if (gimbal_controller.small_yaw_info.angle > GIMBAL_SMALL_YAW_LIMIT_RIGHT && ...)
     { return gimbal_controller.set_small_yaw_current; }
     else
     { gimbal_controller.set_small_yaw_current = 0; return ...; }
 }
+*/
```

### 8.3 GimbalClear() — 小Yaw PID/前馈/TD/目标清零全部注释

```diff
-    PID_Clear(&gimbal_controller.small_yaw_angle_pid);
-    PID_Clear(&gimbal_controller.small_yaw_speed_pid);
+    // [SMALL_YAW_REMOVED] 小Yaw PID Clear已删除
+    // PID_Clear(&gimbal_controller.small_yaw_angle_pid);
+    // PID_Clear(&gimbal_controller.small_yaw_speed_pid);
```

```diff
-    Feedforward_Clear(&gimbal_controller.small_yaw_speed_forward);
-    Feedforward_Clear(&gimbal_controller.small_yaw_angle_forward);
+    // [SMALL_YAW_REMOVED] 小Yaw Feedforward Clear已删除
+    // ...
```

```diff
-    TD_Clear(&gimbal_controller.pos_small_yaw_td, ...);
-    TD_Clear(&gimbal_controller.speed_small_yaw_td, ...);
+    // [SMALL_YAW_REMOVED] 小Yaw TD Clear已删除
+    // ...
```

```diff
-    gimbal_controller.target_small_yaw_angle = gimbal_controller.gyro_yaw_angle;
-    gimbal_controller.set_small_yaw_angle = ...;
-    gimbal_controller.set_small_yaw_speed = 0;
-    gimbal_controller.set_small_yaw_current = 0;
+    // [SMALL_YAW_REMOVED] 小Yaw目标角度/控制量清零已删除
+    // ...
```

### 8.4 Big_Yaw_Bias_Cal() — small_yaw_info.angle 替换

**关键改动**: 小Yaw电机角度不再可用，bias 计算改为直接置 0。

```diff
-    if (fabsf(gimbal_controller.big_yaw_angle_pid.Err) < 1.0f
-        && fabsf(gimbal_controller.small_yaw_angle_pid.Err) < 1.0f
+    // [SMALL_YAW_REMOVED] 原条件中small_yaw_angle_pid.Err检查已移除，仅检查大Yaw稳定性
+    if (fabsf(gimbal_controller.big_yaw_angle_pid.Err) < 1.0f
         && big_yaw_controller.big_yaw_mode == 0
         && big_yaw_controller.gimbal_last_mode != 0)
     {
-        fix_motor_angle = gimbal_controller.small_yaw_info.angle - GIMBAL_SMALL_YAW_ZERO_POINT;
+        // [SMALL_YAW_REMOVED] fix_motor_angle无需计算(小Yaw已删除)
+        // fix_motor_angle = ...
     }
```

```diff
-    big_yaw_controller.big_yaw_gyro_bias = gimbal_controller.small_yaw_info.angle - GIMBAL_SMALL_YAW_ZERO_POINT;
+    // [SMALL_YAW_REMOVED] 小Yaw删除后bias置0，原计算: small_yaw_info.angle - SMALL_YAW_ZERO_POINT
+    big_yaw_controller.big_yaw_gyro_bias = 0.0f;
```

### 8.5 Gimbal_Big_Yaw_Calculate() — 移除小Yaw掉线检查

```diff
-    if (gimbal_controller.small_yaw_recv.angle != 0 && big_yaw_controller.big_yaw_gyro_raw != 0)
+    // [SMALL_YAW_REMOVED] 原条件检查small_yaw_recv.angle(防小Yaw掉线)已移除，仅检查大Yaw陀螺
+    if (big_yaw_controller.big_yaw_gyro_raw != 0)
         return gimbal_controller.set_big_yaw_current;
     else
         return 0.f;
```

### 8.6 updateGyro() — GYRO_SIGN 替换

```diff
-    gimbal_controller.gyro_yaw_angle = GIMBAL_SMALL_YAW_GYRO_SIGN * INS.YawTotalAngle;
+    // [SMALL_YAW_REMOVED] GIMBAL_SMALL_YAW_GYRO_SIGN原为1.0f，直接使用
+    gimbal_controller.gyro_yaw_angle = 1.0f * INS.YawTotalAngle;
```

> 陀螺仪 Yaw 数据保留，仍用于大Yaw PID 反馈

### 8.7 GimbalTestInit/ResetCost — 小Yaw测试字段注释

```diff
-    SquareWaveInit(&test->small_yaw_square, ...);
-    test->last_yaw_cycle = 0;
-    test->last_yaw_ise = 0;
-    test->last_yaw_control = 0;
-    test->last_yaw_max_error = 0;
+    // [SMALL_YAW_REMOVED] 小Yaw方波测试已删除
+    // ...
```

```diff
-    test->yaw_cost.ise = 0; ...  (6行)
+    // [SMALL_YAW_REMOVED] 小Yaw目标函数清零已删除
+    // ...
```

---

## 9. GimbalTask.c

### 9.1 Gimbal_Powerdown_Cal()

```diff
-    motor_communication[SMALL_YAW_MOTOR].control = 0;
+    // [SMALL_YAW_REMOVED] 小Yaw电机控制已删除
+    // motor_communication[SMALL_YAW_MOTOR].control = 0;
```

### 9.2 Gimbal_Autoaim_Cal() / Gimbal_Small_Buff_Cal()

**核心改动**: `target_small_yaw_angle` → `target_big_yaw_angle`，删除小Yaw控制赋值。

```diff
-    if (... && fabsf(gimbal_controller.target_small_yaw_angle - pc_yaw) < 70.0f)
+    // [SMALL_YAW_REMOVED] target_small_yaw_angle → target_big_yaw_angle
+    if (... && fabsf(gimbal_controller.target_big_yaw_angle - pc_yaw) < 70.0f)
     {
-        gimbal_controller.target_small_yaw_angle = pc_yaw;
+        gimbal_controller.target_big_yaw_angle = pc_yaw;
     }
```

```diff
-    if (fabsf(gimbal_controller.target_small_yaw_angle - 999.0f) < 1e-4)
+    // [SMALL_YAW_REMOVED] target_big_yaw_angle保护 (原target_small_yaw)
+    if (fabsf(gimbal_controller.target_big_yaw_angle - 999.0f) < 1e-4)
     {
-        gimbal_controller.target_small_yaw_angle = gimbal_controller.gyro_yaw_angle;
-        gimbal_controller.target_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
+        gimbal_controller.target_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
     }
```

```diff
-    motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(gimbal_controller.target_small_yaw_angle);
+    // [SMALL_YAW_REMOVED] 小Yaw控制已删除，仅大Yaw
+    // motor_communication[SMALL_YAW_MOTOR].control = ...
     motor_communication[BIG_YAW_MOTOR].control = Gimbal_Big_Yaw_Calculate(gimbal_controller.target_big_yaw_angle);
```

### 9.3 Gimbal_Act_Cal()

```diff
-    // === Yaw控制 ===
-    #if (GIMBAL_TEST_CONFIG == GIMBAL_CONFIG_SMALLYAW_SQUARE)
-    gimbal_controller.target_small_yaw_angle = SquareWaveRun(...);
-    #elif ...
-    #endif
-    motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(...);
+    // [SMALL_YAW_REMOVED] 小Yaw控制已全部删除，大Yaw独立承载Yaw控制
+    // (原小Yaw方波测试 + 控制赋值已注释)
```

### 9.4 updataSensors()

```diff
-    if (motor_communication[SMALL_YAW_MOTOR].motor_type == GM6020)
-    {
-        GM6020_Decode(&gimbal_controller.small_yaw_recv, &gimbal_controller.small_yaw_info);
-    }
+    // [SMALL_YAW_REMOVED] 小Yaw电机解码已删除
+    // if (motor_communication[SMALL_YAW_MOTOR].motor_type == GM6020)
+    // { ... }
```

### 9.5 YawSawTest()

```diff
-    motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(test_yaw_angle);
+    // [SMALL_YAW_REMOVED] 小Yaw控制已删除，锯齿波测试仅用大Yaw
+    // motor_communication[SMALL_YAW_MOTOR].control = ...
```

---

## 10. ChassisSolver.c

**说明**: 所有遥控器/上位机对 Yaw 目标角度的赋值，从 `target_small_yaw_angle` 改为 `target_big_yaw_angle`（共 8 处）。

```diff
-    gimbal_controller.target_small_yaw_angle += 180;
+    gimbal_controller.target_big_yaw_angle /* [SMALL_YAW_REMOVED] 原为target_small_yaw */ += 180;

-    gimbal_controller.target_small_yaw_angle -= remote_controller.dji_remote.mouse.x * 0.005f;
+    gimbal_controller.target_big_yaw_angle /* [SMALL_YAW_REMOVED] 原为target_small_yaw */ -= ...;

-    gimbal_controller.target_small_yaw_angle -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * ...;  (6处)
+    gimbal_controller.target_big_yaw_angle /* [SMALL_YAW_REMOVED] 原为target_small_yaw */ -= ...;  (6处)
```

---

## 架构变化总结

### 改动前

```
上位机/遥控器 → target_small_yaw_angle → SmallYaw PID → GM6020 (CAN2 0x205) ─┐
                                         target_big_yaw_angle → BigYaw PID → DM_Motor (CAN1) ─┤→ 云台Yaw
                                         大Yaw动态IIR跟随小Yaw                                │
                                         小Yaw掉线 → 大Yaw输出被杀死                           │
```

### 改动后

```
上位机/遥控器 → target_big_yaw_angle → BigYaw PID → DM_Motor (CAN1) → 云台Yaw
                大Yaw独立承载全部Yaw控制
                gyro_yaw_angle 作为PID反馈 (来源不变: IMU陀螺仪)
```

> **关键**: 大Yaw DM_Motor 直接响应目标角度，不再通过小Yaw间接联动。陀螺仪 Yaw 数据保留作为大Yaw 速度环反馈。

---

## 11. Big Yaw 完整数据流（2026-06-19 调试用）

### 11.1 反馈链路（读）

```
底盘IMU ──CAN2 0x166──▶ bsp_can.c RX中断
  ├─ memcpy → big_yaw_controller.big_yaw_gyro_raw    (float, 4字节)
  └─ memcpy → big_yaw_controller.big_yaw_gyro_speed  (float, 4字节)
        │
        ▼ Big_Yaw_Bias_Cal() @ GimbalTask 500Hz
  dealed_big_yaw_gyro = big_yaw_gyro_raw   // 2026/06/19: 删除3.03修正因子

DM_Big_Yaw_Motor ──CAN1 0x10──▶ bsp_can.c RX中断
  └─ DM_Motor_Receive() → P_Receive, V_Receive, ERR_State, Motor_Enable
```

### 11.2 控制链路（写）

```
DJIRemoteUpdate() @ 250Hz (ChassisSolver.c)
  │  摇杆LEFT_CH_LR + 鼠标mouse.x
  ▼
gimbal_controller.target_big_yaw_angle  ←── 上位机 pc_yaw (自瞄模式)
        │
        ▼ GimbalTask @ 500Hz, switch(gimbal_action)
        │
  ┌─────┴──────────────────────────────────────────┐
  │ GIMBAL_ACT_MODE:  Gimbal_Act_Cal()              │
  │   target → IIR动态跟随 → Gimbal_Big_Yaw_Calculate(target) → control │
  │ GIMBAL_AUTO_AIM:  Gimbal_Autoaim_Cal()          │
  │   pc_yaw → target → Gimbal_Big_Yaw_Calculate(target) → control │
  │ GIMBAL_POWERDOWN: Gimbal_Powerdown_Cal()        │
  │   control = 0                                    │
  └────────────────────────────────────────────────┘
        │
        ▼ Motor_Data_Pack() @ 500Hz
  DM_Big_Yaw_Motor.t_ff = motor_communication[BIG_YAW_MOTOR].control
  DM_Motor_Control(&DM_Big_Yaw_Motor, ..., DM_MIT_CONTROL) → CAN数据打包
  is_has_motor_data[0][DM_MOTOR_1] = TRUE
        │
        ▼ Motor_Data_Send_1() @ 500Hz (每3次中的2次)
  CanSend(&hcan1, motor_send_data[0][DM_MOTOR_1], MOTOR_STD_ID_LIST[5]=0x05)
        │
        ▼ CAN1 0x05 ──▶ DM_Big_Yaw_Motor
```

### 11.3 Gimbal_Big_Yaw_Calculate() 内部

```
set_point ──▶ TD(pos_big_yaw_td) ──▶ set_big_yaw_angle
  │                                     反馈: dealed_big_yaw_gyro
  ├─ Angle PID (Kp=15, MaxOut=120) ──▶ set_big_yaw_speed
  │   + Feedforward(angle_forward)
  │
  ├─ TD(speed_big_yaw_td) ──▶ 
  │   反馈: big_yaw_gyro_speed
  ├─ Speed PID (Kp=25, MaxOut=1500) ──▶ set_big_yaw_current
  │   + Feedforward(speed_forward)
  │
  └─ GIMBAL_BIG_YAW_MOTOR_SIGN × set_big_yaw_current → t_ff
     + 缓启动: 误差<5°时MaxOut从160逐步放大到120
     + 保护: big_yaw_gyro_raw==0 时返回0 (防掉线疯转)
```

### 11.4 关键 Debug 变量

| 变量 | 位置 | 正常值 |
|------|------|--------|
| `motor_communication[3].control` | 控制命令(中间) | = set_big_yaw_current |
| `DM_Big_Yaw_Motor.t_ff` | 最终发给电机 | 同 control |
| `is_has_motor_data[0][5]` | CAN1发送标志 | = 1 |
| `DM_Big_Yaw_Motor.Motor_Enable` | 电机使能 | = 1 |
| `DM_Big_Yaw_Motor.ERR_State` | 错误状态 | = 1 (正常) |
| `remote_controller.gimbal_action` | 当前模式 | 1=ACT, 2=AUTO_AIM |
| `set_big_yaw_current` | PID输出 | 摇杆推时非零 |

### 11.5 当前调试状态 (2026/06/19)

**现象**: `set_big_yaw_current=1435` 但 `motor_communication[3].control=0`，`DM_Big_Yaw_Motor.t_ff=0`

**排查中**: 
- 枚举值正确: BIG_YAW_MOTOR=3, MOTOR_APP_NUMS=5
- can_send_config 正确: motor_type=DM_MOTOR, can=CAN1, motor_id=0x05
- is_has_motor_data[0][5]=1 ✓ (CAN打包正常)
- 下一步: 确认 gimbal_action 实际值
