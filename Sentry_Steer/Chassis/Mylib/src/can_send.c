#include "can_send.h"

int8_t CanSend(CAN_TypeDef *CANx, int8_t *data, uint32_t std_id, uint8_t data_length)
{
  CanTxMsg tx_message;
  uint8_t mailbox;
  uint8_t retry;

  if (CANx == NULL || data == NULL || data_length > 8U)
  {
    return FALSE;
  }

  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = data_length;
  tx_message.StdId = std_id;

  memcpy(tx_message.Data, data, data_length);
  for (retry = 0U; retry < 4U; retry++)
  {
    /* CAN_Transmit selects a mailbox and writes its registers.  Keep that
     * sequence atomic across the chassis tasks, then yield while mailboxes
     * are busy instead of silently dropping the frame. */
    taskENTER_CRITICAL();
    mailbox = CAN_Transmit(CANx, &tx_message);
    taskEXIT_CRITICAL();
    if (mailbox != CAN_TxStatus_NoMailBox)
    {
      return TRUE;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
      vTaskDelay(pdMS_TO_TICKS(1U));
    }
  }

  return FALSE;
}

// 上位机可视化测试用
// extern INS_t INS;
// uint64_t count_can = 0;

// /* 陀螺仪数据CAN发送 */
// void SendIMUByCan(void)
// {
//   IMU_Out imu_out;

//   imu_out.pitch = INS.Pitch;
//   imu_out.roll = INS.Roll;

//   count_can++;

//   if (count_can % 2 == 0)
//   {
//     CanSend(CAN2, (int8_t *)(&imu_out), 0x110, sizeof(imu_out));
//   }
// }
