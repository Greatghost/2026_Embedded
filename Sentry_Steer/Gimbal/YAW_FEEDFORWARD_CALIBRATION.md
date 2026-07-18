# 大 Yaw 模型前馈与实车标定

## 1. 已接入的模型

`crossing_hole-main` 的 Yaw 前馈已按目标车单大 Yaw 架构接入：

```text
u_ff = gain * (J * alpha_ref
               + B * omega_ref
               + C * sat(omega_ref / friction_blend_dps))
```

- `omega_ref`、`alpha_ref` 来自 `pos_big_yaw_td.dx/ddx`，不是实测速度。
- 摩擦方向取参考速度；在 `±friction_blend_dps` 内线性淡入，零速时为 0。
- `u_ff` 与速度 PID 输出相加，最后统一乘 `GIMBAL_BIG_YAW_MOTOR_SIGN`。
- 模型输出有独立的 `MAXOUT`，原有掉线清零、恢复首帧同步和大 Yaw 缓启动仍然有效。

参数入口在 `config/gimbal_config.h`。当前 TIGER 初值为：

```text
J=0.5, B=5.0, C=0.0, blend=6 deg/s, gain=1.0, maxout=500 mN*m
```

`J/B` 只是把本工程旧的 `{速度5.0, 加速度0.5}` 经验前馈迁移到新模型时使用的保守初值，`C=0`。仓库中没有本车 Yaw 标定数据；这些值不是实车辨识结果。

目标 DM 驱动在打包前把 `t_ff` 除以 1000，因此代码中的工程单位为：

| 参数 | 工程单位 |
|---|---|
| `J` | mN·m/(deg/s²) |
| `B` | mN·m/(deg/s) |
| `C`、`u_ff` | mN·m |

如需换算为 SI：

```text
J_phys [kg*m^2]     = J_code * 0.0572958
B_phys [N*m*s/rad]  = B_code * 0.0572958
C_phys [N*m]        = C_code / 1000
```

## 2. 标定前必须固定的条件

1. 取弹并关闭摩擦轮、拨盘，底盘刹停；准备可立即断电的急停。
2. 确认 Yaw 是滑环连续旋转还是存在线缆/机械限位。测试轨迹必须按单方向最大转角核算，不能只看最终净转角。
3. 使用比赛状态的枪管、相机、供弹链和配重；固定常用 Pitch 角。
4. 电机、轴承预热到正常工作温度，电池电压保持在正常范围。
5. 标定和最终运行必须保持相同的 DM `Kp/Kd/V_des/T_Max`。当前大 Yaw 为 `Kp=0`、`Kd=5`、`V_des=0`、`T_Max=20`；改变其中任一项都会改变等效 `B`。
6. 标定采集期间将 `GIMBAL_BIG_YAW_MODEL_FF_GAIN` 设为 `0.0f`，重新编译、烧录。不要一边采集一边改速度 PID。

当前 `Gimbal_SI_Cal()` 是空函数，SI 模式没有安全的自动辨识实现。不要直接进入 SI 模式做实车测试：该分支可能保留上一控制周期的电机输出。应使用受监督的 PC 角度轨迹，或另做每周期显式写输出、异常即清零的专用标定固件。

## 3. 先验证方向和量程

先用很小的正、负目标速度或连续角度斜坡测试：

1. 物理逆时针运动应使 `gyro_yaw_angle`、`gyro_yaw_speed` 同号增加。
2. 正模型方向的控制量应产生正模型方向加速度。
3. 记录发送 `t_ff` 与 `DM_Big_Yaw_Motor.t_ff_Receive`，检查比例和正负号。

虽然当前配置的 `GIMBAL_BIG_YAW_MOTOR_SIGN` 和 `GIMBAL_BIG_YAW_GYRO_SIGN` 都是 `+1`，反馈代码实际直接使用 `INS.YawTotalAngle`；必须以实车运动确认，不能只看宏。

用于拟合的模型坐标力矩建议定义为：

```text
yaw_torque_model_mnm =
    GIMBAL_BIG_YAW_MOTOR_SIGN * gimbal_controller.set_big_yaw_current
```

这会把最终电机命令重新变换回“逆时针为正”的模型坐标。另行记录 `t_ff_Receive` 作量程与饱和交叉检查。

## 4. 建议记录的数据

以 200–500 Hz 同步记录；至少保证每行来自同一控制周期：

```text
timestamp_s
gimbal_controller.delta_t
gimbal_controller.gyro_yaw_angle
gimbal_controller.gyro_yaw_speed
INS.Gyro[2]                         # 原始陀螺角速度，确认单位后转成deg/s
gimbal_controller.pos_big_yaw_td.dx
gimbal_controller.pos_big_yaw_td.ddx
gimbal_controller.big_yaw_speed_pid.Output
gimbal_controller.set_big_yaw_current
gimbal_controller.DM_Big_Yaw_Motor.t_ff_Receive
gimbal_controller.big_yaw_ff_inertia
gimbal_controller.big_yaw_ff_viscous
gimbal_controller.big_yaw_ff_coulomb
gimbal_controller.big_yaw_ff_output
DM Kp/Kd/V_des/T_Max
电机温度、电池电压、Pitch角、离线状态
```

目标工程的 `gyro_yaw_speed` 是 `YawTotalAngle` 差分后再 IIR 的结果；J 对微分噪声很敏感，建议同时记录原始陀螺角速度，在离线处理中低通后求实际加速度。不要直接对含毛刺的单点速度做差分。

供拟合脚本使用的 CSV 至少包含：

```csv
yaw_speed_dps,yaw_accel_dps2,yaw_torque_model_mnm
50.1,0.4,560.2
```

## 5. 标定 B 和 C

在模型前馈关闭、速度环参数固定的条件下，做正负恒速段。可先从本车容易稳定跟踪的四档开始：

```text
+30, -30, +70, -70, +120, -120, +160, -160 deg/s
```

如果某档实际速度无法稳定跟踪，就降低该档。每档：

1. 先等待 0.4–1.0 s；
2. 再至少采集 180° 的连续运动；
3. 剔除起停、换向、掉线和命令饱和段；
4. 对该档计算平均实际速度和平均模型坐标力矩，使每档成为一个等权点；
5. 在不同起始绝对 Yaw 角重复三组，检查滑环/线缆造成的角度相关阻力。

拟合本实现使用的精确模型：

```text
torque = B * omega + C * sat(omega / blend) + bias
```

运行仓库内脚本：

```powershell
python .\tools\fit_yaw_feedforward.py .\yaw_bc.csv --mode bc --blend-dps 6
```

要求 `B>=0`、`C>=0`。若 `bias` 明显大于拟合 RMSE，优先检查力矩符号、线缆预紧和正反向不对称，不要把偏置硬塞进 `C`。

`C` 是运动库仑摩擦，不是静摩擦。不能用“刚好推得动”的堵转力矩标定；零速剩余静摩擦由闭环处理。

## 6. 标定 J

先得到可信的 `B/C`，再做同方向加速/减速配对。建议首轮使用比源车更短的轨迹：

```text
参考加速度：80 deg/s²
峰值速度：120 deg/s
配对窗口中心：40、80 deg/s，各取约 ±10 deg/s
正方向完成 0 -> 120 -> 0 后，再反方向重复
```

三角速度轨迹单方向近似转角为：

```text
delta_angle ~= Vmax^2 / accel
```

上述首轮约为 `120²/80=180°`。源工程的 `200²/80=500°` 会先单向转约 1.39 圈，只有确认滑环和机械范围后才能使用。

在相同方向和相近速度窗口内，把加速、减速样本配对：

```text
delta_torque = torque_acc - torque_dec
residual     = delta_torque - B * delta_speed
J            = residual / delta_accel
```

同方向配对会消掉 `C`。两个速度窗口、两个方向至少得到四个独立 J；其最大/最小建议不超过 1.5，绝不能出现负值。

若 CSV 已包含经过合理低通的实际加速度，可运行：

```powershell
python .\tools\fit_yaw_feedforward.py .\yaw_j.csv --mode j --b <B值> --c <C值> --blend-dps 6
```

脚本的 J 模式是样本级最小二乘；正式结果仍应与上述加减速窗口配对结果交叉验证。

## 7. 标定低速摩擦淡入点

`6 deg/s` 是源工程经验值，不是 J/B/C 辨识结果。分别测试：

```text
±2, ±4, ±6, ±8, ±10 deg/s
```

寻找能连续运动、无明显爬行，并且所需力矩接近库仑摩擦平台的最低速度区域。将 `GIMBAL_BIG_YAW_MODEL_FF_FRICTION_BLEND_DPS` 放在该区域；零目标时 `big_yaw_ff_coulomb` 必须回到 0。

## 8. 写回与逐级启用

本工程没有 Yaw 参数的 Flash/EEPROM 持久化，调参流程是：

```text
修改 config/gimbal_config.h -> Keil全量编译 -> 烧录
```

写入实测 `J/B/C` 后，按以下顺序上车：

```text
gain = 0.25 -> 0.5 -> 1.0
```

每一级检查：

- `big_yaw_ff_output` 命中 `MAXOUT` 的占比；
- 总 `t_ff` 是否饱和；
- 速度误差、速度 PID 输出 RMS、超调、反向振荡；
- 电机温升和零速漂移。

目标 TD 当前为 `r=20000, h0=0.01`，源工程为 `r=2000, h0=0.005`。`r` 允许的瞬时参考加速度很大，标定后的真实 J 不应为了避免饱和而随意缩小；应调整 `GAIN/MAXOUT`、目标轨迹加速度或增加明确的参考加速度限幅。

## 9. 建议验收标准

- 三次独立标定：`B/C` 变异系数不超过 10%，`J` 不超过 15%；
- `B/C/J` 均为非负，正反方向结果无明显系统偏差；
- 前馈独立限幅命中占比低于 0.5%；
- 启用前馈后，典型跟踪轨迹的速度误差 RMS 和速度 PID 输出 RMS 至少下降 30%；
- 5°、15°、30°阶跃以及自动瞄准轨迹中，超调不高于关闭前馈的基线；
- 零速无新增漂移、抖动或周期换向；
- 在不同 Pitch 角、冷/热状态和底盘运动状态下复验。若 J 或摩擦项变化超过 15%，按负载/Pitch 分档，而不是使用一个折中的过大前馈。
