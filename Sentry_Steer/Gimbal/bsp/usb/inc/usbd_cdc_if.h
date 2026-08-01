/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_CDC_IF USBD_CDC_IF
  * @brief Usb VCP device module
  * @{
  */

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048
/* USER CODE BEGIN EXPORTED_DEFINES */

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Macros USBD_CDC_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USB CDC 发送诊断计数器（供调试观察用） */
extern volatile uint32_t usb_cdc_busy_count;      /* CDC_Transmit_FS 返回 USBD_BUSY 的累计次数 */
extern volatile uint32_t usb_cdc_tx_reset_count;   /* 完整 USB Device 恢复的累计次数 */
extern volatile uint32_t usb_cdc_not_configured_count; /* USB 未枚举时发送被拒的累计次数 */
extern volatile uint8_t  usb_cdc_last_dev_state;        /* 最近一次观察到的 USB 设备状态 */
extern volatile uint8_t  usb_cdc_host_port_open;        /* NUC CDC 串口 DTR：1=已有进程打开 */
extern volatile uint32_t usb_cdc_host_closed_drop_count; /* 主机未打开串口时跳过上报次数 */

typedef enum
{
  USB_CDC_RESET_REASON_NONE = 0U,
  USB_CDC_RESET_REASON_TX_STUCK = 1U,
  USB_CDC_RESET_REASON_HOST_BUS = 2U
} USB_CDC_ResetReason_t;

typedef struct
{
  volatile uint32_t total_count;       /* 本次 MCU 上电期间的复位事件总数 */
  volatile uint32_t tx_stuck_count;    /* TxState 卡死导致的主动恢复次数 */
  volatile uint32_t host_bus_count;    /* NUC/Hub 发出的 USB 总线 RESET 次数 */
  volatile uint32_t last_tick_ms;      /* 最近复位发生时的 HAL tick */
  volatile uint32_t last_stuck_ms;     /* 最近一次 Tx 卡死持续时间 */
  volatile uint32_t last_timeout_ms;   /* 当时采用的看门狗阈值 */
  volatile uint8_t last_reason;        /* USB_CDC_ResetReason_t */
  volatile uint8_t last_dev_state;     /* 复位前 USBD dev_state */
  volatile uint8_t last_tx_state;      /* 复位前 CDC TxState（低 8 位） */
  volatile uint8_t restart_pending;    /* 等待重新枚举时为 1 */
} USB_CDC_ResetDebug_t;

extern volatile USB_CDC_ResetDebug_t usb_cdc_reset_debug;

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

/**
 * @brief USB CDC TxState recovery supervisor.
 * @note  A transfer that remains busy past timeout_ms causes a controlled USB
 *        disconnect/de-initialization.  The complete device is reinitialized
 *        after 250 ms; endpoint state is never modified concurrently with ISR.
 * @param timeout_ms TxState 卡死超时阈值（单位 ms）
 */
void CDC_Transmit_FS_Watchdog(uint32_t timeout_ms);
void USB_CDC_RecordHostBusReset(void);

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
