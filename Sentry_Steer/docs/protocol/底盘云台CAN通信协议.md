# 底盘 ↔ 云台 CAN 通信协议

> 本文档整合了底盘与云台控制板之间的全部板间 CAN 通信定义，涵盖帧格式、数据来源、协议变更历史与两端适配清单。
>
> 合并自：`0x0303_CAN接口_云台对接.md`、`CHASSIS_COMM_UPDATE.md`、`底盘云台通信协议适配指南_20260713.md`、`底盘CAN协议变更_云台适配说明_20260713.md`。
>
> 裁判系统串口协议（0xA5 帧、cmd_id 分发、UI 绘制、哨兵决策等）详见 `裁判系统通信架构解析.md`。

---

## 一、CAN 帧总表

### 底盘 → 云台（CAN2）

| CAN ID | 帧名 | 频率 | 说明 | 状态 |
|--------|------|------|------|------|
| `0x094` | `JudgeData_ForSend1` | 20Hz | 比赛/热量/弹量等 | 数据源变更（前哨站血量） |
| `0x096` | `JudgeData_ForSend2` | — | 比赛数据2 | 不变 |
| `0x097` | `JudgeBloodData_ForSend1` | 10Hz | 血量数据 | 数据源变更（敌方基地血量） |
| `0x098` | RFID / Buff 帧 | — | — | 不变 |
| `0x099` | 位置轮询 | ~9.5Hz | 10 帧轮询（5 友+5 敌，IDs 1/2/3/4/7） | 不变 |
| `0x09A` | 底盘速度 | 20Hz | 正运动学反解结果（5ms 周期 `%10==2`） | 不变 |
| `0x09B` | 射击数据 | 10Hz | 0x0207 | 不变 |
| `0x09C` | `SentryInfo_ForSend` | 10Hz | 哨兵信息 | bit 语义变更 |
| `0x09D` | 弹量扩展 | 10Hz | 0x0208+RFID2 | 不变 |
| `0x09E` | 小地图下发指令 | ≤10Hz | 0x0303 目标坐标（事件触发） | 不变 |
| `0x09F` | `SentryDuration_ForSend` | 10Hz | 哨兵姿态时长 | **V2.0.0 新增** |
| `0x0A0` | `DamageDiff_ForSend` | 10Hz | 伤害值差 | **V2.0.0 新增** |
| `0x0A1` | `MotorOffline_ForSend` | **未调度** | 电机掉线位图 | **已定义未调用**（见 §五） |
| `0x0A2` | `UwbSteer_ForSend` | **未调度** | UWB角度+舵角 | **已定义未调用**（见 §五） |
| `0x0A3` | `OutpostHP_ForSend` | **未调度** | 前哨站 HP（原始 uint16） | **已定义未调用**（见 §五） |
| `0x15A` | SentryCmd 转发 | 事件触发 | 哨兵命令字 4B | 不变 |
| `0x160` | `GimbalSendPack_1` | 20Hz | 射击权限/阵营/电容/弹速 | 不变 |

### 云台 → 底盘

| CAN ID | 帧名 | 频率 | 说明 | 状态 |
|--------|------|------|------|------|
| `0x150` | `GimbalReceivePack1` (pack1) | — | 云台姿态等 | 不变 |
| `0x151` | `GimbalReceivePack2` | 10Hz | 哨兵坐标 | 2026-06-19 新增 |

---

## 二、底盘 → 云台 帧详解

### 2.1 `0x09E` 小地图下发指令（0x0303）

| 属性 | 值 |
|------|-----|
| CAN ID | `0x09E` |
| DLC | 8 字节 |
| CAN 总线 | CAN2 |
| 发送频率 | 最高 10Hz（内容变化时发送，Referee 层去重） |
| 字节序 | 小端（STM32 原生） |

**数据结构**

```c
#pragma pack(push, 1)
typedef struct RobotCommand_ForSend
{
    int16_t  target_position_x_100;  // 目标X坐标 (float×100 → int16)，单位 0.01m
    int16_t  target_position_y_100;  // 目标Y坐标 (float×100 → int16)，单位 0.01m
    uint8_t  cmd_keyboard;           // 键盘按键命令
    uint8_t  target_robot_id;        // 目标机器人ID
    uint16_t cmd_source;             // 指令来源
} RobotCommand_ForSend_t;
#pragma pack(pop)
// sizeof = 8 字节
```

**帧布局**

```
┌────────┬────────┬────────┬────────┬────────┐
│ Byte 0-1│ Byte 2-3│ Byte 4  │ Byte 5  │ Byte 6-7 │
├────────┼────────┼────────┼────────┼────────┤
│ target │ target │ cmd_   │ target │ cmd_   │
│ pos_x  │ pos_y  │ keybrd │ robot  │ source │
│ _100   │ _100   │        │ _id    │        │
├────────┼────────┼────────┼────────┼────────┤
│ int16  │ int16  │ uint8  │ uint8  │ uint16 │
└────────┴────────┴────────┴────────┴────────┘
```

**字段说明**

| 字段 | 类型 | 说明 |
|------|------|------|
| `target_position_x_100` | int16_t | 目标位置 X 坐标，原始值 = int16 ÷ 100（单位 m） |
| `target_position_y_100` | int16_t | 目标位置 Y 坐标，原始值 = int16 ÷ 100（单位 m） |
| `cmd_keyboard` | uint8_t | 键盘按键指令，直接透传裁判系统下发值 |
| `target_robot_id` | uint8_t | 操作手指定的目标机器人 ID |
| `cmd_source` | uint16_t | 指令来源，直接透传裁判系统下发值 |

**数据来源**

| CAN 字段 | 裁判系统字段 |
|----------|-------------|
| `target_position_x_100` | `referee_data.Robot_Command.target_position_x` (float) × 100 |
| `target_position_y_100` | `referee_data.Robot_Command.target_position_y` (float) × 100 |
| `cmd_keyboard` | `referee_data.Robot_Command.cmd_keyboard` (uint8) |
| `target_robot_id` | `referee_data.Robot_Command.target_robot_id` (uint8) |
| `cmd_source` | `referee_data.Robot_Command.cmd_source` (uint16) |

**去重机制**：裁判系统可能重复下发相同指令。底盘 Referee 层收到 `0x0303` 时进行 `memcmp` 二进制比较，只有内容变化时才更新数据并置 `is_robot_command_update` 标志。`JudgeDataCanSend()` 检测标志后发送一帧，并清零标志。

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

### 2.2 `0x09F` 哨兵姿态时长（V2.0.0 新增）

| CAN ID | 方向 | 频率 | 帧名 |
|--------|------|------|------|
| **0x09F** | 底盘 → 云台 | 10Hz | `SentryDuration_ForSend` |

| 字节 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | `normal_attack_duration` | uint8 | 哨兵进攻姿态弱化前剩余秒数 |
| 1 | `normal_defend_duration` | uint8 | 哨兵防御姿态弱化前剩余秒数 |
| 2 | `normal_move_duration` | uint8 | 哨兵移动姿态弱化前剩余秒数 |
| 3 | `reserved_duration_1` | uint8 | 保留 |
| 4 | `enhanced_attack_duration` | uint8 | 哨兵强化进攻姿态剩余秒数 |
| 5 | `enhanced_defend_duration` | uint8 | 哨兵强化防御姿态剩余秒数 |
| 6 | `enhanced_move_duration` | uint8 | 哨兵强化移动姿态剩余秒数 |
| 7 | `reserved_duration_2` | uint8 | 保留 |

> 数据来源于裁判系统 0x020D 新增的 `sentry_posture_duration` 字段（8 字节），底盘原样转发。
> 设计原因：CAN 单帧 DLC=8，原 `SentryInfo_ForSend_t`（0x09C）已占满 8 字节，新增 8 字节时长数据无法塞入原帧，故新增专用 CAN ID 0x09F 承载。

```c
typedef struct SentryDuration_ForSend
{
    uint8_t normal_attack_duration;    // 哨兵进攻姿态弱化前剩余秒数
    uint8_t normal_defend_duration;    // 哨兵防御姿态弱化前剩余秒数
    uint8_t normal_move_duration;      // 哨兵移动姿态弱化前剩余秒数
    uint8_t reserved_duration_1;       // 保留
    uint8_t enhanced_attack_duration;  // 哨兵强化进攻姿态剩余秒数
    uint8_t enhanced_defend_duration;  // 哨兵强化防御姿态剩余秒数
    uint8_t enhanced_move_duration;    // 哨兵强化移动姿态剩余秒数
    uint8_t reserved_duration_2;       // 保留
} SentryDuration_ForSend_t;
// sizeof = 8，恰好填满一个 CAN 帧
```

### 2.3 `0x0A0` 伤害值差（V2.0.0 新增）

| CAN ID | 方向 | 频率 | 帧名 |
|--------|------|------|------|
| **0x0A0** | 底盘 → 云台 | 10Hz | `DamageDiff_ForSend` |

| 字节 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0-1 | `damage_difference` | int16 LE | 伤害值差（己方血量总和 − 敌方血量总和） |
| 2-7 | `reserve[6]` | uint8[6] | 保留 0x00 |

> 数据来源于裁判系统 0x0003 新增的 `damage_difference` 字段。正值表示己方血量领先，负值表示落后。云台可用于判断双方血量对比，辅助战术决策。

### 2.4 已有帧的数据变更（2026-07-13 / V2.0.0）

#### `0x09C` `SentryInfo_ForSend` — bit 命名修正

`sentry_info_2` 字段的 **bit 15** 含义更新（**帧格式不变，仅 bit 语义变化**）：

| 旧名 | 新名 | 说明 |
|------|------|------|
| `sentry_reserved2` | `sentry_is_enhanced_posture` | 当前姿态是否为强化姿态：0-否 / 1-是 |

#### `0x094` `JudgeData_ForSend1` — 敌方前哨站血量数据源变更

`Enemy_outpost` 字段（6bit，压缩值）的数据来源从**雷达**改为**裁判系统 0x0003**：

| 旧来源 | 新来源 |
|--------|--------|
| 雷达交互数据 `enemy_HP.enemy_outpost_HP`（不可靠，曾注释掉） | `Game_Robot_friend_HP.enemy_outpost_HP`（0x0003 直接下发） |

> 解压公式不变：`HP = value / 0.04 - 24`。此字段之前被注释（始终为 0），现在正式启用。

#### `0x097` `JudgeBloodData_ForSend1` — 敌方基地血量启用

`blood_type = 1`（敌方）帧的 `ID8` 字段（基地血量，6bit）现在从 0x0003 获取真实值：

| 旧值 | 新来源 |
|------|--------|
| 硬编码 `0` | `Game_Robot_friend_HP.enemy_base_HP / 100` |

> `ID8` 为 6bit（0~63），实际基地血量需 ×100 还原。

---

## 三、云台 → 底盘 帧详解

### 3.1 `0x151` 哨兵坐标（2026-06-19 新增）

| 字节 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0-1 | `sentry_x_cm` | int16 LE | 哨兵 X 坐标 (cm) |
| 2-3 | `sentry_y_cm` | int16 LE | 哨兵 Y 坐标 (cm) |
| 4-7 | `reserved` | uint8[4] | 预留 0x00 |

- **发送频率**：10Hz（每 50 个 GimbalTask 周期发一次）
- **坐标来源**：上位机 → USB CDC TypeID=0x01 → STM32 解析校验后原值转发
- **范围**：X: 0~2800 cm，Y: 0~1500 cm

> `0x150` (pack1) 帧格式无改动。

---

## 四、底盘端代码修改清单（V2.0.0 适配）

> 对应 2026-07-13 裁判系统协议更新（0x0003 / 0x020D 结构体扩展）。

### 4.1 `GimbalSend.h` — 数据结构扩展

- **`SentryInfo_ForSend_t`（0x09C，保持不变）** + 新增 `SentryDuration_ForSend_t`（见 §2.2），由新 CAN ID `0x09F` 承载。
- **`JudgeData_ForSend1_t`（0x094）**：`Enemy_outpost` 字段数据源改用 `Game_Robot_friend_HP.enemy_outpost_HP`（6bit 压缩不变）。
- **`JudgeBloodData_ForSend1_t`（0x097）**：`ID8`（基地血量）改用 `Game_Robot_friend_HP.enemy_base_HP / 100`。
- **`DamageDiff_ForSend_t`**：新增结构体（见 §2.3），由新 CAN ID `0x0A0` 承载。

### 4.2 `GimbalSend.c` — 打包逻辑修改

| 位置 | 修改内容 |
|------|----------|
| `SentryInfoPack()`（约 `:175`） | `sentry_reserved2` → `sentry_is_enhanced_posture`（bit 15） |
| `SentryInfoPack()` | 新增 `SentryDurationPack()`：拷贝 6 种姿态时长到 `sentry_duration_send` |
| `JudgeDataBloodPack()`（约 `:287-288`） | `ID8 = referee_data.Game_Robot_friend_HP.enemy_base_HP / 100`（原硬编码 0） |
| `JudgeDataPack()`（约 `:418`） | 取消注释，`Enemy_outpost = 0.04f * (Game_Robot_friend_HP.enemy_outpost_HP + 24)` |
| `JudgeDataCanSend()`（约 `:600-665`） | 新增 0x09F 时长帧、0x0A0 伤害差帧的发送调度 |

`SentryInfoPack()` 关键修改：

```c
// 旧：
sentry_info_2_raw |= ((uint16_t)referee_data.Sentry_info.sentry_reserved2 & 0x1) << 15;
// 新：
sentry_info_2_raw |= ((uint16_t)referee_data.Sentry_info.sentry_is_enhanced_posture & 0x1) << 15;
```

`SentryDurationPack()` 新增：

```c
void SentryDurationPack(void)
{
    sentry_duration_send.normal_attack_duration   = (uint8_t)referee_data.Sentry_info.normal_attack_duration;
    sentry_duration_send.normal_defend_duration   = (uint8_t)referee_data.Sentry_info.normal_defend_duration;
    sentry_duration_send.normal_move_duration     = (uint8_t)referee_data.Sentry_info.normal_move_duration;
    sentry_duration_send.reserved_duration_1      = (uint8_t)referee_data.Sentry_info.reserved_duration_1;
    sentry_duration_send.enhanced_attack_duration = (uint8_t)referee_data.Sentry_info.enhanced_attack_duration;
    sentry_duration_send.enhanced_defend_duration = (uint8_t)referee_data.Sentry_info.enhanced_defend_duration;
    sentry_duration_send.enhanced_move_duration   = (uint8_t)referee_data.Sentry_info.enhanced_move_duration;
    sentry_duration_send.reserved_duration_2      = (uint8_t)referee_data.Sentry_info.reserved_duration_2;
}
```

### 4.3 `can_config.h` — 新增 CAN ID

```c
#define SEND_TO_GIMBAL_SENTRY_DURATION_CAN_ID 0x09F  // 0x020D 哨兵姿态时长
#define SEND_TO_GIMBAL_DAMAGE_DIFF_CAN_ID     0x0A0  // 0x0003 伤害值差
```

---

## 五、云台端适配说明

### 5.1 新增 CAN 0x09F / 0x0A0 接收（必须）

- CAN 滤波器添加 `0x09F`、`0x0A0`
- 解析 `SentryDuration_ForSend`（0x09F）：6 种姿态时长，用于 UI 倒计时显示、姿态超时判断
- 解析 `DamageDiff_ForSend`（0x0A0）：`damage_difference` 为 int16，正值 = 己方血量领先

### 5.2 更新字段名（建议）

- `sentry_reserved2` → `sentry_is_enhanced_posture`（0x09C 帧 bit 15），配合姿态时长帧判断是否处于强化期

### 5.3 验证数据有效性（建议）

- 敌方前哨站血量（0x094 `Enemy_outpost`）现在有真实值，可确认解压结果合理（前哨站满血通常 1500~2000）
- 敌方基地血量（0x097 `ID8`）不再为 0

### 5.4 哨兵坐标帧（0x151）

- 解析 int16 LE 的 X/Y 坐标（cm），来自上位机原值转发

---

## 六、协议变更历史

| 日期 | 变更 |
|------|------|
| 2026-06-19 | 云台侧新增 `0x151` 哨兵坐标帧（云台 → 底盘） |
| 2026-07-13 | 裁判系统协议 V2.0.0：`0x0003` / `0x020D` 结构体扩展；底盘新增 `0x09F`（姿态时长）、`0x0A0`（伤害差）；`0x09C` bit 15 语义修正；`0x094` 前哨站血量、`0x097` 基地血量数据源改用裁判系统 0x0003 |
