# 新增 CAN 0x09E 小地图下发指令接收 + PC转发

> **日期**: 2026-06-08
> **协议依据**: `Chassis/0x0303_CAN接口_云台对接.md`
> **数据来源**: 裁判系统 0x0303（小地图下发指令）→ 底盘 MCU → CAN2 0x09E → 云台 MCU
> **PC转发**: TypeID 9，0.5Hz，编译开关 `DEBUG_ROBOT_CMD_SEND` 控制

---

## 数据流

```
裁判系统 0x0303 (小地图操作员指令)
    │
    ▼
底盘 MCU (Referee层去重, 变化时发送)
    │
    ▼
CAN2 ID 0x09E (DLC=8, 最高10Hz)
    │
    ▼
云台 MCU ── bsp_can.c MotorReceive() → robot_command_recv
    │                                       │
    │                          ┌────────────┘
    │                          ▼
    │              pc_serial.c SendtoPCRobotCmd()
    │                          │
    ▼                          ▼
  导航/行为决策           上位机 TypeID 9 (0.5Hz, 条件编译)
```

---

## CAN 帧布局

```
┌──────────┬──────────┬──────────┬──────────┬──────────┐
│ Byte 0-1 │ Byte 2-3 │ Byte 4   │ Byte 5   │ Byte 6-7 │
├──────────┼──────────┼──────────┼──────────┼──────────┤
│ target_x │ target_y │ cmd_     │ target_  │ cmd_     │
│ _100     │ _100     │ keyboard │ robot_id │ source   │
├──────────┼──────────┼──────────┼──────────┼──────────┤
│ int16    │ int16    │ uint8    │ uint8    │ uint16   │
└──────────┴──────────┴──────────┴──────────┴──────────┘
```

---

## 改动概览

| # | 文件 | 改动类型 |
|---|------|---------|
| 1 | `config/can_config.h` | 新增 CAN ID 宏 + 占用滤波器槽位 |
| 2 | `application/inc/ChassisGet.h` | 新增接收结构体 + extern |
| 3 | `application/src/ChassisGet.c` | 新增全局变量 |
| 4 | `bsp/boards/src/bsp_can.c` | 新增 CAN 接收分支 |
| 5 | `application/inc/pc_serial.h` | 新增 TypeID 9 枚举 + 发送结构体 + extern |
| 6 | `application/src/pc_serial.c` | 新增发送函数 + SendtoPC 分支 |
| 7 | `config/gimbal_config.h` | 新增编译开关 |
| 8 | `Task/src/GimbalTask.c` | 新增 0.5Hz PC_Send 调用 |

---

## 1. can_config.h（GOBLIN + TIGER 两段）

**说明**: 新增 `ROBOT_COMMAND_CAN_ID 0x09E`，占用 CAN2 FIFO0 空闲槽位 `CAN2_FIFO0_ID3`（原为 `0x000`）。

```diff
 #define GET_BULLET_EXTENDED_CAN_ID 0x09D      // 接收弹量扩展字段(0x0208扩展)
+#define ROBOT_COMMAND_CAN_ID 0x09E              // 接收小地图下发指令(0x0303)
 // 新增: 发送SentryCmd给底盘
```

```diff
-#define CAN2_FIFO0_ID3 0x000
+#define CAN2_FIFO0_ID3 ROBOT_COMMAND_CAN_ID
```

> **滤波器布局**: `ROBOT_COMMAND_CAN_ID` 加入 CAN2 bank 15 (FIFO0)，与 `GET_FROM_BIG_YAW_CAN_ID`(0x166)、`PITCH_MOTOR_CAN_ID`(0x11)、`SMALL_YAW_MOTOR_CAN_ID`(0x205) 共享滤波器组。

---

## 2. ChassisGet.h

**说明**: 新增 `RobotCommand_ForSend_t` 接收结构体，8 字节匹配 CAN DLC。

```diff
+// 小地图下发指令接收结构 (0x0303, CAN ID 0x09E)
+typedef struct RobotCommand_ForSend
+{
+  int16_t  target_position_x_100;  // 目标X坐标 (float×100 → int16)，单位 0.01m
+  int16_t  target_position_y_100;  // 目标Y坐标 (float×100 → int16)，单位 0.01m
+  uint8_t  cmd_keyboard;           // 键盘按键命令
+  uint8_t  target_robot_id;        // 目标机器人ID
+  uint16_t cmd_source;             // 指令来源
+} RobotCommand_ForSend_t;  // sizeof == 8 字节，匹配 CAN DLC
```

```diff
+extern RobotCommand_ForSend_t robot_command_recv;  // 小地图下发指令接收
```

---

## 3. ChassisGet.c

**说明**: 全局变量定义，与 `shoot_data_recv`、`sentry_info_recv` 等同一位置。

```diff
 BulletExtendedRecv_t bullet_extended_recv;
+RobotCommand_ForSend_t robot_command_recv;  // 小地图下发指令接收 (0x09E)
```

---

## 4. bsp_can.c

**说明**: 在 `MotorReceive()` 中新增 CAN2 0x09E 接收分支。因为数据来自底盘但走 CAN2（与大Yaw陀螺数据 0x166 同总线），条件直接使用 `CAN2` 实例匹配。

```diff
 	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_BULLET_EXTENDED_CAN_ID)
 	{
 		// 弹量扩展字段接收 (0x0208扩展)
 		memcpy(&bullet_extended_recv, data, sizeof(BulletExtendedRecv_t));
 	}
+	else if (hcan->Instance == CAN2 && rx_header->StdId == ROBOT_COMMAND_CAN_ID)
+	{
+		// 小地图下发指令接收 (0x09E, 来自底盘裁判系统0x0303)
+		memcpy(&robot_command_recv, data, sizeof(RobotCommand_ForSend_t));
+	}
 }
```

> **注意**: 此处分支使用 `CAN2` 直接匹配而非 `CHASSIS_CAN_COMM_CANx`（后者为 CAN1），因为底盘专门通过 CAN2 发送 0x0303 指令给云台。

---

## 5. pc_serial.h

### 5.1 TypeID 枚举

```diff
     JUDGE_PC_DATA_SENTRY_DATA = 7,      // TypeID 7: 哨兵信息 (0x020D + 0x0207初速度)
-    JUDGE_PC_DATA_BULLET_DATA_AND_RFID2 = 8  // TypeID 8: 弹量数据+RFID扩展
+    JUDGE_PC_DATA_BULLET_DATA_AND_RFID2 = 8, // TypeID 8: 弹量数据+RFID扩展
+    JUDGE_PC_DATA_ROBOT_COMMAND = 9     // TypeID 9: 小地图下发指令 (0x0303)
 } PC_dataType_enum;
```

### 5.2 发送结构体

**说明**: 15 字节固定包（1 Head + 1 Type + 12 Data + 1 CRC8），8 字节 RobotCommand 原样转发 + 4 字节 reserved 填充。

```diff
+// TypeID 9: RobotCmd - 小地图下发指令 (0x0303 CAN 0x09E)
+typedef struct PCSendDataRobotCmd
+{
+    uint8_t start_flag;              // '!'
+    uint8_t data_pack_type;          // = JUDGE_PC_DATA_ROBOT_COMMAND = 9
+    RobotCommand_ForSend_t cmd;      // 8 bytes 小地图指令原样转发
+    uint32_t reserved;               // 填充至12字节data
+    uint8_t crc8;                    // CRC8校验
+} PCSendDataRobotCmd_t;  // sizeof == 15 bytes (1+1+12+1)
```

### 5.3 extern 声明

```diff
 extern PCRecvData_1 pc_recv_data_1;
+extern PCSendDataRobotCmd_t PCSendRobotCmd;  // TypeID 9
```

---

## 6. pc_serial.c

### 6.1 全局变量

```diff
 PCSendDataBulletAndRfid2_t PCSendBulletAndRfid2;
+PCSendDataRobotCmd_t PCSendRobotCmd;
```

### 6.2 发送函数

**说明**: 将 `robot_command_recv`（底盘CAN收到的原始小地图指令）原样打包到 TypeID 9 PC包中。

```diff
+// TypeID 9: 发送小地图下发指令
+void SendtoPCRobotCmd(unsigned char* buff)
+{
+	PCSendRobotCmd.start_flag = '!';
+	PCSendRobotCmd.data_pack_type = JUDGE_PC_DATA_ROBOT_COMMAND;
+	PCSendRobotCmd.cmd = robot_command_recv;  // 8 bytes 原样转发
+	PCSendRobotCmd.reserved = 0;
+	PCSendRobotCmd.crc8 = 0;
+	Append_CRC8_Check_Sum((unsigned char *)&PCSendRobotCmd, PC_SEND_BLOOD_SIZE);
+	memcpy(buff, (void *)&PCSendRobotCmd, PC_SEND_BLOOD_SIZE);
+}
```

### 6.3 SendtoPC 分支

```diff
 	else if(data_type == JUDGE_PC_DATA_BULLET_DATA_AND_RFID2)
 	{
 		SendtoPCBulletAndRfid2(SendToPC_Buff);
 	}
+	else if(data_type == JUDGE_PC_DATA_ROBOT_COMMAND)
+	{
+		SendtoPCRobotCmd(SendToPC_Buff);
+	}
 	CDC_Transmit_FS(SendToPC_Buff,PC_SENDBUF_SIZE);
```

---

## 7. gimbal_config.h

**说明**: 编译开关，注释掉即屏蔽 TypeID 9 的 PC 发送。

```diff
 #define GIMBAL_CONTROL_DISCONNECT     0
+
+#define DEBUG_ROBOT_CMD_SEND             // 开启: 0.5Hz向PC发送0x0303小地图指令(TypeID 9), 注释即屏蔽
```

---

## 8. GimbalTask.c

**说明**: 在 `PC_Send()` 函数中新增 TypeID 9 的 0.5Hz 定时发送。`index % 1000 == 7` 即每 1000 次迭代（500Hz → 2 秒一次）发送一次。

```diff
 	// 新增: TypeID 8 弹量数据+RFID扩展发送 (10Hz)
 	if (index % 50 == 25)
 	{
 		SendtoPC(JUDGE_PC_DATA_BULLET_DATA_AND_RFID2);
 	}
+#ifdef DEBUG_ROBOT_CMD_SEND
+    // 调试: TypeID 9 小地图下发指令发送 (0.5Hz)
+    if (index % 1000 == 7)
+    {
+        SendtoPC(JUDGE_PC_DATA_ROBOT_COMMAND);
+    }
+#endif

 	if (index % 2 == 0) // 250HZ
```

---

## PC 包格式汇总（TypeID 0~9）

| TypeID | 名称 | 频率 | 说明 |
|--------|------|------|------|
| 0 | `USUAL_PC_DATA` | 250Hz | 云台yaw/pitch/弹量/电容电压 |
| 1 | `JUDGE_PC_DATA` | 25Hz | 比赛状态/血量/前哨站 |
| 2 | `BLOOD_1` | 10Hz | 友方血量 |
| 3 | `BLOOD_2` | 10Hz | 敌方血量 |
| 4 | `RFID_BUFF` | 5Hz | Buff状态+RFID |
| 5 | `POS` | 10Hz | 机器人坐标(轮询) |
| 6 | `EXTENDED` | 20Hz | UWB/姿态/底盘速度 |
| 7 | `SENTRY_DATA` | 10Hz | 哨兵信息+弹丸初速度 |
| 8 | `BULLET_DATA_AND_RFID2` | 10Hz | 弹量扩展+RFID扩展 |
| **9** | **`ROBOT_COMMAND`** | **0.5Hz** | **小地图下发指令(0x0303)** |

---

## 字段说明

| 字段 | 类型 | 裁判系统来源 | 说明 |
|------|------|-------------|------|
| `target_position_x_100` | int16 | 0x0303 target_position_x | 目标 X 坐标 ×100，单位 0.01m |
| `target_position_y_100` | int16 | 0x0303 target_position_y | 目标 Y 坐标 ×100，单位 0.01m |
| `cmd_keyboard` | uint8 | 0x0303 cmd_keyboard | 键盘按键命令，透传 |
| `target_robot_id` | uint8 | 0x0303 target_robot_id | 操作手指定的目标机器人 ID |
| `cmd_source` | uint16 | 0x0303 cmd_source | 指令来源，透传 |

---

## 去重机制（底盘侧）

裁判系统可能重复下发相同指令。底盘 Referee 层收到 0x0303 时进行二进
制比较，只有内容变化时才更新数据并发送 CAN 帧（最高 10Hz）。

```
裁判 0x0303 到达
      │
      ▼
memcmp(新, 旧) == 0 ? ──是──> 丢弃
      │
     否
      │
      ▼
更新 Robot_Command + 置标志
      │
      ▼
JudgeDataCanSend() 检测标志
      │
      ▼
CAN2 0x09E 发送 → 云台
      │
      ▼
清零标志
```
