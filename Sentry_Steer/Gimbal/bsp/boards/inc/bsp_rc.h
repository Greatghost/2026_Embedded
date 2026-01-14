#ifndef BSP_RC_H
#define BSP_RC_H

#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

#include "remote_control.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

void remote_control_init(void);
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void RC_unable(void);
void RC_restart(uint16_t dma_buf_num);

#endif
