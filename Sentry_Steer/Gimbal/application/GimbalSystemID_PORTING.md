# 云台系统辨识移植与使用说明

## Sentry_Steer 当前接入状态

- `GimbalSystemID.c`、`GimbalSystemIDConfig.c`、对应头文件和本文档均直接来自
  `crossing_hole-main`；算法主体没有重写。
- 目标工程仅做了接口适配：`ins.h` 改为 `ins_task.h`，并把源码中的
  `yaw_speed_pid`、`DM_Yaw_Motor` 分别映射到大 Yaw 的
  `big_yaw_speed_pid`、`DM_Big_Yaw_Motor`。
- `GimbalSystemID_Init()` 已在电机和 PID 初始化后调用；
  `GimbalSystemID_Run()` 已按源码方式放入云台周期任务。
- Yaw、Pitch 辨识时均使用源码中的专用速度环参数；辨识未启动或结束后，
  输出被清零，避免初始化阶段的参考值直接驱动电机。
- 辨识所选轴的 DM MIT `Kp`、`Kd` 和 `V_des` 均强制为 0，最终控制链路只有
  “辨识状态机速度参考 -> 专用速度 PID -> `t_ff`”。普通角度环、Pitch 多项式
  重力补偿、角度/速度前馈和 Yaw J/B/C 模型前馈均不会进入辨识输出。
- 当前安全默认值为 `GIMBAL_SYSID_DISABLED` 和
  `GIMBAL_SYSID_PORTING_CONFIRMED 0`。源码车辆的运动配置只作为初始模板，
  未经本车实测不能直接把确认宏改为 `1`。
- `torque_feedback_coef` 已按本工程电机正方向先设为 `+1.0f`，仍必须用小力矩
  实车验证“沿模型正方向发力时，模型力矩为正”；不符合时改该系数符号。
- 本工程云台任务为 500 Hz，而来源工程为 1 kHz。算法使用实际 `dt` 计算，
  但按采样点数定义的窗口在本工程上会持续约两倍时间，首次运行应预留足够行程。

## 为什么采用“少量宏 + 配置结构体”

- `GIMBAL_SYSID` 和 `GIMBAL_SYSID_STEP` 会改变条件编译，只能使用宏。
- 机构、速度环和运动范围相关参数放在一个带字段名及单位的配置结构体中，避免几十个初始化形参，也避免散落宏难以理解。
- 采样数量、滤波频率、拟合阈值保留为算法内部默认值。只有出现采样不足或拟合质量问题时才修改它们。

## 辨识状态与普通控制器相互独立

`GimbalController` 不需要包含任何辨识状态。模块在
`GimbalSystemID.h` 中公开独立上下文：

```c
extern GimbalSystemIDContext gimbal_sysid;
```

调试器直接查看：

```text
gimbal_sysid.yaw
gimbal_sysid.pitch
```

因此移植到一个不含 `Gimbal_SI` 成员的控制器结构体时，不需要修改对方的
结构体布局。`GimbalSystemID_Init()` 接收控制器指针，仅用于接入本项目的陀螺仪、
电机反馈和速度环；若目标项目的控制器命名不同，只需在系统辨识实现的输入/输出
接入位置做适配，不需要改辨识状态结构。

移植者主要接触三个位置：

1. `application/inc/GimbalSystemIDConfig.h`：选择辨识轴、步骤，完成移植确认。
2. `application/src/GimbalSystemIDConfig.c`：设置机构和测试运动参数。
3. `application/src/Gimbal.c`：设置辨识时使用的速度环 PID。

复制模块到新工程时，还必须把 `GimbalSystemID.c` 和
`GimbalSystemIDConfig.c` 同时加入构建系统；只复制头文件会在链接时找不到
`gimbal_sysid_user_config`。

## 首次移植必须确认

### 1. 坐标和力矩符号

先用很小的正速度参考确认：

- Yaw 正方向应与项目定义一致。
- Pitch 正方向必须是向上。
- `torque_feedback_coef * t_ff_Receive` 在电机沿模型正方向发力时应为正。

如果正速度下模型力矩为负，修改对应的 `torque_feedback_coef`，不要在辨识公式中临时改符号。

### 2. Pitch 硬安全角度

手动缓慢移动 Pitch，记录 IMU 实际可达角度，再设置：

- `safe_min_deg`：略高于机械下限。
- `safe_max_deg`：略低于机械上限。

必须满足：

```text
safe_min < safe_max，并且可用跨度不少于 20°
```

这两个值是安全边界，不是普通控制限位。其余 Pitch 测试角度由程序自动生成：

- 扫描换向点距离上下边界 1°。
- 有效采样区和 J 运动区距离上下边界 2.5°。
- 三个 J 加减速切换点按运动区间四等分。
- 两个 J 配对中心位于相邻切换点中间。

调试器可查看 `gimbal_sysid.pitch_angle_layout`，确认自动生成的全部角度。
设置错误仍可能导致撞击机械限位，因此首次运行应低速观察换向是否正常。

### 3. 辨识专用速度环

辨识只使用速度环输出。先关闭辨识，以手动速度测试确认：

- 正负方向均能启动。
- 实际速度能稳定，不持续振荡。
- 输出不长期饱和。
- Pitch 上升和下降可使用不同参考速度，以得到相近的实际速度。

不要通过系统辨识运动顺便调 PID；应先把速度环调到可用，再开始采集。

### 4. Pitch 重力扫描速度

设置 `gravity_up_ref_dps` 和 `gravity_down_ref_dps`。两者不要求数值相等，要求实测上升和下降速度幅值接近。

例如：上升参考 50 得到实际 20，下降参考 1 得到实际 -20，则应保留不对称参考值。

### 5. Pitch B/C 三档速度映射

分别填写：

- `bc_up_ref_dps[3]`
- `bc_down_ref_dps[3]`
- `bc_expected_actual_dps[3]`

每个下标代表同一档实际速度。目标是同档上升、下降实际速度幅值接近，并覆盖低、中、高三个稳定运动速度。

### 6. Pitch J 运动范围

J 的起止、切换和配对角度不再手填，统一由 `safe_min_deg`、`safe_max_deg`
生成。`j_actual_peak_speed_dps` 应明显高于最低可稳定运动速度，但不能让输出饱和。

#### `j_prepare_up_ref_dps` / `j_prepare_down_ref_dps` 如何标定

这两个参数只用于正式 J 采样前把 Pitch 移到 `j_start_deg`：低于起点时使用
`+j_prepare_up_ref_dps`，高于起点时使用 `-j_prepare_down_ref_dps`。它们不是
正式 J 辨识轨迹的上下速度；J 只在向上加速和向上减速时采样，下降阶段只是返回起点。

作者所说“上下实际速度都等于 `j_actual_peak_speed_dps`”，应按下面方法标定本车
在重力作用下不对称的“速度参考 -> 实际速度”关系：

1. 先标定并确认 `safe_min_deg`、`safe_max_deg`，架空云台并准备急停。
2. 选择实际目标峰值 `Vpk = j_actual_peak_speed_dps`。它必须大于 5 dps，且不能让
   速度环输出持续饱和；新车应从保守速度开始，而不是直接照搬模板中的 30 dps。
3. 借用全行程的 `GIMBAL_SYSID_STEP_GRAVITY` 扫描测稳态速度。把
   `gravity_up_ref_dps` 暂设为候选 `U`，把 `gravity_down_ref_dps` 暂设为候选 `D`。
4. 在远离换向点的稳定区观察 `gimbal_controller.gyro_pitch_speed`，推荐同时观察
   `gimbal_sysid.pitch.td_omega.x`。调 `U` 使上升实际速度接近 `+Vpk`；调 `D`
   使下降实际速度接近 `-Vpk`。配置中的 `D` 填正数，算法会自动加负号。
5. 至少重复三次。建议上、下实际速度各自与目标差不超过 3 dps，且两方向速度幅值
   之差不超过 3 dps；同时确认无持续饱和、振荡和机械碰撞。
6. 把最终 `U`、`D` 分别写入 `j_prepare_up_ref_dps`、
   `j_prepare_down_ref_dps`，并恢复重力扫描原本需要的配置。

正式 J 上升时，发送给速度 PID 的参考不是直接的 `j_actual_peak_speed_dps`，而是：

```text
j_velocity_ref = j_up_ref_offset_dps + desired_actual_speed
```

并受 `j_up_ref_min_dps`、`j_up_ref_max_dps` 限制；返回下降使用的是
`j_return_down_ref_dps`，不是 `j_prepare_down_ref_dps`。因此还必须在正式 J 运行时：

- 观察 `j_motion_phase` 从 1 切到 2 附近的实际峰值；若不等于 `Vpk`，调整
  `j_up_ref_offset_dps`，并保证
  `j_up_ref_max_dps >= j_up_ref_offset_dps + j_actual_peak_speed_dps`。
- 单独校验 `j_return_down_ref_dps` 的返回速度和安全性；若希望返回速度也接近
  `Vpk`，可先用上述 `D` 作为候选值。
- 重点监视 `gyro_pitch_angle`、`gyro_pitch_speed`、`pitch_speed_pid.Ref`、
  `pitch_speed_pid.Output`、`j_motion_phase`、`j_velocity_ref`、`j_pass_count`、
  `J`、`j_rmse`、`sysid_valid` 和 `sysid_error`。

`GIMBAL_SYSID_STEP_J` 不会先辨识重力和 B/C，会直接读取
`GIMBAL_PITCH_SIN/COS/B/C/J`。因此单独跑 J 前必须先把本车上一轮 Gravity、BC
结果写回这些常量。分步跨重启时，应按 `Gravity -> 写回 G_sin/G_cos -> BC ->
写回 B/C -> J` 的顺序。`GIMBAL_SYSID_STEP_ALL` 则会在同一次上电中自动执行
`Gravity -> BC -> J`，中间结果保存在 RAM；任一阶段失败都会停止。

### 7. 老/新云台配置档案

`GimbalSystemIDConfig.c` 中分别保存 `OLD` 和 `NEW` 两套 Pitch 参数，切换时
不会覆盖另一套实测数据。默认档案由 `gimbal_sysid_pitch_profile_id` 决定；如果
整车能读取硬件版本，应在 `GimbalSystemID_Init()` 前选择：

```c
GimbalSystemID_SelectPitchProfile(GIMBAL_SYSID_PITCH_PROFILE_OLD);
/* 或 GIMBAL_SYSID_PITCH_PROFILE_NEW */
```

这属于运行时数据选择，不改变条件编译，也不要求在算法文件中增加车型宏。

### 8. Yaw B/C 和 J 运动

- `bc_speed_dps[8]` 必须同时包含正、负速度，推荐正负交替。
- 每档速度都必须是速度环能够稳定跟踪的值。
- `j_pair_center_dps` 必须位于 `0` 与 `j_max_ref_dps` 之间，并避开起停低速区。
- Yaw J 单方向三角速度轨迹的近似转角为：

```text
单方向转角 ≈ j_max_ref_dps² / j_ref_accel_dps2
```

当前配置约为 `200²/80=500°`。必须确认没有线缆缠绕或机械限位风险。

## 启用步骤

完成以上检查后，在 `GimbalSystemIDConfig.h` 中设置：

```c
#define GIMBAL_SYSID_PORTING_CONFIRMED 1
#define GIMBAL_SYSID GIMBAL_PITCH_SYSID   /* 或 GIMBAL_YAW_SYSID */
#define GIMBAL_SYSID_STEP GIMBAL_SYSID_STEP_ALL
```

编译、下载后，程序仍保持安全互锁。确认周围安全后，在调试器中手动设置：

```c
gimbal_sysid.pitch.sysid_done = 0;
/* 或 */
gimbal_sysid.yaw.sysid_done = 0;
```

也就是说，`GIMBAL_SYSID` 只选择编译哪个轴，`GIMBAL_SYSID_STEP` 只选择步骤；
仅打开宏不会自动运动。与 `crossing_hole-main` 完全相同，真正开始辨识仍需把所选轴的
`sysid_done` 手动改为 `0`。Pitch 的 `STEP_ALL` 会得到重力项、B、C、J；Yaw 的
`STEP_ALL` 会得到 B、C、J。

开始前还要让云台处于会调用 Pitch/Yaw 计算函数的使能模式（本工程通常为
`GIMBAL_ACT_MODE`），并确认 IMU、Pitch 电机和 Yaw 电机在线。若处于
`GIMBAL_POWERDOWN` 或掉线保护生效，状态机仍可能推进，但电机输出会被安全闸门清零。

DJI 遥控器建议使用“左拨杆向上、右拨杆向下”：该组合进入
`GIMBAL_ACT_MODE`、关闭射击，并且不会主动开启小陀螺。保持所有摇杆回中后，再在
调试器中把所选轴的 `sysid_done` 改为 0。

辨识结束后必须恢复：

```c
#define GIMBAL_SYSID_PORTING_CONFIRMED 0
#define GIMBAL_SYSID GIMBAL_SYSID_DISABLED
```

## 结果判定

成功结束应满足：

```text
sysid_done  = 1
sysid_valid = 1
sysid_error = 0
```

常见错误：

| `sysid_error` | 含义 | 优先检查 |
|---:|---|---|
| 1 | 激励或有效平均点不足 | 实际速度、采样窗口、是否卡住 |
| 2 | Pitch触发安全角度 | 安全范围、角度符号、机械限位 |
| 3 | B/C/J出现非物理结果 | 力矩符号、重力模型、运动方向 |
| 4 | 不同配对点差异过大 | 加速度是否稳定、是否饱和、窗口是否合理 |
| 5 | 拟合残差过大 | 速度环振荡、结构松动、采样噪声 |
| 6 | 用户配置关系无效 | 配置数组、角度顺序、正负速度、确认项 |

## 推荐的移植顺序

1. 只验证方向、角度和力矩单位，不运动辨识。
2. 调好辨识专用速度环。
3. Pitch 按 `GRAVITY -> BC -> J` 分步运行并保存中间结果。
4. Yaw 按 `BC -> J` 分步运行。
5. 分步结果稳定后再使用 `ALL`。
6. 将最终参数写入控制模型，关闭辨识宏，重新验证普通控制。

结果只保存在 RAM，不会自动写入 Flash。Yaw 成功后，把
`gimbal_sysid.yaw.J/B/C` 写回 `GIMBAL_BIG_YAW_MODEL_FF_J/B/C`。Pitch 会输出
`G_sin/G_cos/B/C/J`；本工程普通 Pitch 控制器当前使用多项式重力模型，不能把
这组正弦重力参数直接覆盖到多项式系数中。若要让普通 Pitch 控制也使用这组辨识模型，
需要另行统一前馈模型形式。
