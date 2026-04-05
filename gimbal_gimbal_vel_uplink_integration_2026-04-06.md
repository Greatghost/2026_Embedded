# Gimbal Angular Velocity Uplink Integration (2026-04-06)

## 1. Purpose

Add lower-to-upper gimbal angular velocity feedback without changing existing frame length.

- Use uplink `TypeID=6` `ExtendData.Reserve_32_1`
- Publish ROS2 topic: `/ly/gimbal/gimbal_vel`
- Feed behavior_tree blackboard for strategy use

## 2. Uplink Frame Contract

`ExtendData` stays:

```cpp
struct ExtendData {
    static constexpr auto TypeID = 6;
    std::uint16_t UWBAngleYaw; // 2 bytes
    std::uint16_t Reserve_16;  // high 8 bits used by posture feedback
    std::uint32_t Reserve_32_1;
    std::uint32_t Reserve_32_2;
};
```

No frame size change.

## 3. Reserve_32_1 Bit/Byte Mapping

`Reserve_32_1` is decoded in little-endian byte order:

- byte0 + byte1 => `yaw_raw` (`int16`)
- byte2 + byte3 => `pitch_raw` (`int16`)

Scale:

- physical unit: `deg/s`
- encoded unit: `0.01 deg/s`
- conversion: `physical = raw * 0.01`

Range:

- `int16` => `[-32768, 32767]`
- physical => `[-327.68, 327.67] deg/s`

Direction:

- sign carries direction (`+` / `-`)
- exact positive direction for yaw/pitch must be aligned with lower firmware convention

## 4. ROS Interface Added

New msg:

- `gimbal_driver/msg/GimbalVel.msg`
  - `int16 yaw`
  - `int16 pitch`
  - unit: `0.01 deg/s`

New topic:

- `/ly/gimbal/gimbal_vel` (`gimbal_driver/msg/GimbalVel`)

## 5. Upper Computer Changes

### 5.1 gimbal_driver

- Add `GimbalVel.msg` to `rosidl_generate_interfaces`
- Parse `ExtendData.Reserve_32_1` in `PubExtendData()`
- Publish `/ly/gimbal/gimbal_vel`

### 5.2 behavior_tree

- Add topic definition for `/ly/gimbal/gimbal_vel`
- Subscribe and store:
  - `gimbalYawVelRaw`
  - `gimbalPitchVelRaw`
  - `gimbalYawVelDegPerSec`
  - `gimbalPitchVelDegPerSec`
- Write to global blackboard:
  - `GimbalYawVelRaw`
  - `GimbalPitchVelRaw`
  - `GimbalYawVelDegPerSec`
  - `GimbalPitchVelDegPerSec`

## 6. Lower Firmware Required Alignment

Lower firmware should keep `TypeID=6` frame format unchanged and fill `Reserve_32_1`:

1. Compute scaled signed values:
   - `yaw_raw = round(yaw_deg_per_s * 100)`
   - `pitch_raw = round(pitch_deg_per_s * 100)`
2. Pack in little-endian:
   - `Reserve_32_1[7:0]   = yaw_raw low8`
   - `Reserve_32_1[15:8]  = yaw_raw high8`
   - `Reserve_32_1[23:16] = pitch_raw low8`
   - `Reserve_32_1[31:24] = pitch_raw high8`
3. Keep `Reserve_16` high 8 bits for posture feedback as before.

## 7. Quick Runtime Check

After rebuild and launch:

```bash
ros2 topic echo /ly/gimbal/gimbal_vel
```

Expected:

- `yaw`/`pitch` update with signed `int16`
- sign changes when direction reverses
