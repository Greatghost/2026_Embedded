#include "Offline_Task.h"
#include "remote_control.h"
#include "gimbal_config.h"
#include "main.h"
#include "iwdgTask.h"

#if USE_WBUS_PROTOCOL
#include "wbus_decoder.h"
#endif

OfflineDetector offline_detector;

/**
 * @brief 根据接收计数维护带超时的在线状态
 *
 * 计数只负责表示“收到过新数据”，时间戳负责判定当前是否仍然新鲜。
 * 这样既不依赖数据内容，也不怕uint16_t计数回绕。
 */
static enum ROBOT_SENSORS_DETECT Offline_UpdateState(
    uint16_t current_count,
    uint16_t *count_snapshot,
    uint32_t *last_update_time,
    uint8_t *has_received,
    uint8_t *recovery_updates,
    uint32_t now,
    uint32_t timeout_ms,
    enum ROBOT_SENSORS_DETECT on_state,
    enum ROBOT_SENSORS_DETECT off_state)
{
    if (current_count != *count_snapshot)
    {
        *count_snapshot = current_count;
        *last_update_time = now;
        *has_received = 1;
        if (*recovery_updates < OFFLINE_RECOVERY_UPDATES)
        {
            (*recovery_updates)++;
        }
    }

    if (!*has_received || (uint32_t)(now - *last_update_time) > timeout_ms)
    {
        *recovery_updates = 0;
        return off_state;
    }

    return (*recovery_updates >= OFFLINE_RECOVERY_UPDATES) ? on_state : off_state;
}

static void Offline_ApplyRemoteSafeState(void)
{
    for (int i = 0; i < 4; i++)
    {
        remote_controller.dji_remote.rc.ch[i] = CH_MIDDLE;
    }
    remote_controller.dji_remote.rc.s[LEFT_SW] = Down;
    remote_controller.dji_remote.rc.s[RIGHT_SW] = Down;
    remote_controller.dji_remote.rc.Previous_rc_Right_SW = Down;
    remote_controller.dji_remote.mouse.press_l = 0;
    remote_controller.dji_remote.mouse.press_r = 0;
    remote_controller.dji_remote.keyValue = 0;

    setRobotState(OFFLINE_MODE);
    setGimbalAction(GIMBAL_POWERDOWN);
    setShootAction(SHOOT_POWERDOWN_MODE);
}

void Offline_task(void *pvParameters)
{
    (void)pvParameters;

    offline_detector.remote_state = REMOTE_OFF;
    offline_detector.pitch_motor_state = PITCH_MOTOR_OFF;
    offline_detector.yaw_motor_state = YAW_MOTOR_OFF;
    offline_detector.pc_state = PC_OFF;
    for (int i = 0; i < 2; i++)
    {
        offline_detector.imu_state[i] = IMU_OFF;
        offline_detector.friction_motor_state[i] = FRICTION_WHEELS_MOTOR_OFF;
    }
    offline_detector.toggle_motor_state = TOGGLE_MOTOR_OFF;

    while (1)
    {
        const uint32_t now = HAL_GetTick();

        /* BMI088下标0/1分别代表加速度计和陀螺仪芯片ID心跳。 */
        for (int i = 0; i < 2; i++)
        {
            offline_detector.imu_state[i] = Offline_UpdateState(
                global_debugger.imu_debugger[i].recv_msgs_num,
                &offline_detector.imu_receive_num[i],
                &offline_detector.imu_last_update_time[i],
                &offline_detector.imu_received[i],
                &offline_detector.imu_recovery_updates[i],
                now,
                IMU_FEEDBACK_TIMEOUT_MS,
                IMU_ON,
                IMU_OFF);
        }

#if USE_WBUS_PROTOCOL
        WBUS_CheckTimeout();
        offline_detector.remote_receive_num = global_debugger.remote_debugger.recv_msgs_num;
        offline_detector.remote_state = WBUS_IsConnected() ? REMOTE_ON : REMOTE_OFF;
#else
        offline_detector.remote_state = Offline_UpdateState(
            global_debugger.remote_debugger.recv_msgs_num,
            &offline_detector.remote_receive_num,
            &offline_detector.remote_last_update_time,
            &offline_detector.remote_received,
            &offline_detector.remote_recovery_updates,
            now,
            REMOTE_FEEDBACK_TIMEOUT_MS,
            REMOTE_ON,
            REMOTE_OFF);
#endif
        if (offline_detector.remote_state == REMOTE_OFF)
        {
            Offline_ApplyRemoteSafeState();
        }

        /* Pitch反馈固定记录在gimbal_debugger[0]。 */
        offline_detector.pitch_motor_state = Offline_UpdateState(
            global_debugger.gimbal_debugger[0].recv_msgs_num,
            &offline_detector.pitch_motor_receive_num,
            &offline_detector.pitch_motor_last_update_time,
            &offline_detector.pitch_motor_received,
            &offline_detector.pitch_motor_recovery_updates,
            now,
            GIMBAL_FEEDBACK_TIMEOUT_MS,
            PITCH_MOTOR_ON,
            PITCH_MOTOR_OFF);

        /* 小Yaw已删除，大Yaw反馈固定记录在gimbal_debugger[2]。 */
        offline_detector.yaw_motor_state = Offline_UpdateState(
            global_debugger.gimbal_debugger[2].recv_msgs_num,
            &offline_detector.yaw_motor_receive_num,
            &offline_detector.yaw_motor_last_update_time,
            &offline_detector.yaw_motor_received,
            &offline_detector.yaw_motor_recovery_updates,
            now,
            GIMBAL_FEEDBACK_TIMEOUT_MS,
            YAW_MOTOR_ON,
            YAW_MOTOR_OFF);

        for (int i = 0; i < 2; i++)
        {
            offline_detector.friction_motor_state[i] = Offline_UpdateState(
                global_debugger.friction_debugger[i].recv_msgs_num,
                &offline_detector.friction_motor_receive_num[i],
                &offline_detector.friction_motor_last_update_time[i],
                &offline_detector.friction_motor_received[i],
                &offline_detector.friction_motor_recovery_updates[i],
                now,
                SHOOT_FEEDBACK_TIMEOUT_MS,
                FRICTION_WHEEL_MOTOR_ON,
                FRICTION_WHEELS_MOTOR_OFF);
        }

        offline_detector.toggle_motor_state = Offline_UpdateState(
            global_debugger.toggle_debugger.recv_msgs_num,
            &offline_detector.toggle_motor_receive_num,
            &offline_detector.toggle_motor_last_update_time,
            &offline_detector.toggle_motor_received,
            &offline_detector.toggle_motor_recovery_updates,
            now,
            SHOOT_FEEDBACK_TIMEOUT_MS,
            TOGGLE_MOTOR_ON,
            TOGGLE_MOTOR_OFF);

        offline_detector.pc_state = Offline_UpdateState(
            global_debugger.pc_receive_debugger.recv_msgs_num,
            &offline_detector.pc_receive_num,
            &offline_detector.pc_last_update_time,
            &offline_detector.pc_received,
            &offline_detector.pc_recovery_updates,
            now,
            PC_FEEDBACK_TIMEOUT_MS,
            PC_ON,
            PC_OFF);

        Iwdg_ReportAlive(IWDG_HEARTBEAT_OFFLINE);
        vTaskDelay(pdMS_TO_TICKS(OFFLINE_MONITOR_PERIOD_MS));
    }
}
