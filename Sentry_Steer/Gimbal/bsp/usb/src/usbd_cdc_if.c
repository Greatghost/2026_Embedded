/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_cdc_if.c
 * @version        : v1.0_Cube
 * @brief          : Usb device for Virtual Com Port.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include "pc_serial.h"

/* USER CODE BEGIN INCLUDE */
#include "stm32f4xx_hal_pcd.h"   /* PCD_HandleTypeDef, HAL_PCD_EP_Flush, USBx_INEP */
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @brief Usb device library.
 * @{
 */

/** @addtogroup USBD_CDC_IF
 * @{
 */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
 * @brief Private types.
 * @{
 */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
 * @brief Private defines.
 * @{
 */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
 * @brief Private macros.
 * @{
 */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
 * @brief Private variables.
 * @{
 */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
 * @brief Public variables.
 * @{
 */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USB CDC 发送诊断计数器 */
volatile uint32_t usb_cdc_busy_count = 0U;      /* CDC_Transmit_FS 返回 USBD_BUSY 的累计次数 */
volatile uint32_t usb_cdc_tx_reset_count = 0U;  /* 看门狗强制清零 TxState 的累计次数 */
volatile uint32_t usb_cdc_not_configured_count = 0U; /* USB 未枚举时发送被拒的累计次数 */
volatile uint8_t  usb_cdc_last_dev_state = 0U;       /* 最近一次观察到的 USB 设备状态 */

/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
 * @brief Private functions declaration.
 * @{
 */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
 * @}
 */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
    {
        CDC_Init_FS,
        CDC_DeInit_FS,
        CDC_Control_FS,
        CDC_Receive_FS,
        CDC_TransmitCplt_FS};

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initializes the CDC media low layer over the FS USB IP
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
 * @brief  DeInitializes the CDC media low layer
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
 * @brief  Manage the CDC class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

    /*******************************************************************************/
    /* Line Coding Structure                                                       */
    /*-----------------------------------------------------------------------------*/
    /* Offset | Field       | Size | Value  | Description                          */
    /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
    /* 4      | bCharFormat |   1  | Number | Stop bits                            */
    /*                                        0 - 1 Stop bit                       */
    /*                                        1 - 1.5 Stop bits                    */
    /*                                        2 - 2 Stop bits                      */
    /* 5      | bParityType |  1   | Number | Parity                               */
    /*                                        0 - None                             */
    /*                                        1 - Odd                              */
    /*                                        2 - Even                             */
    /*                                        3 - Mark                             */
    /*                                        4 - Space                            */
    /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
    /*******************************************************************************/
  case CDC_SET_LINE_CODING:

    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
 * @brief  Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will issue a NAK packet on any OUT packet received on
 *         USB endpoint until exiting this function. If you exit this function
 *         before transfer is complete on CDC interface (ie. using DMA controller)
 *         it will result in receiving more data while previous ones are still
 *         not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  /* IRQ context only copies bytes. Parsing and CAN forwarding run in
   * ChassisTask, outside the USB interrupt. */
  (void)PCStreamEnqueueFromISR(Buf, *Len);

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);

  /* USER CODE END 6 */
}

/**
 * @brief  CDC_Transmit_FS
 *         Data to send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *         @note
 *
 *
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  /* 记录最近一次 USB 设备状态（供调试观察） */
  usb_cdc_last_dev_state = hUsbDeviceFS.dev_state;

  /* USB 未配置（未枚举/已断开）时直接返回，避免设置 TxState=1 后
   * DMA 完成中断永不触发，导致 TxState 永久卡死 */
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    usb_cdc_not_configured_count++;
    return USBD_FAIL;
  }

  if (hcdc->TxState != 0)
  {
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN 8 */

/**
 * @brief USB CDC TxState 看门狗
 *
 * 当上位机停止读取 USB 数据时，USB 主机不发 IN 令牌，DMA 传输完成中断
 * 不触发，TxState 永久保持为 1，导致后续所有 CDC_Transmit_FS 返回 USBD_BUSY。
 *
 * 检测到 TxState 卡死超过 timeout_ms 后，必须做四件事才能彻底恢复:
 *   1) 清 TxState 软件标志        —— 让 CDC_Transmit_FS 不再返回 BUSY
 *   2) HAL_PCD_EP_Flush 清空 TX FIFO —— 清掉残留的旧数据(否则新数据会拼到旧数据后面,
 *      上位机重新读取时先收到碎片,帧同步丢失表现为"收不到数据")
 *   3) 清 DIEPINT 挂起的中断标志  —— 否则旧 XFRC 标志可能立即触发中断干扰新传输
 *   4) 清 IN_ep 软件状态(xfer_len/buff/count) —— 否则 HAL_PCD_EP_Transmit 行为异常
 *
 * 只清 TxState 不够:TX FIFO 残留数据会导致上位机重新读取时帧错位。
 * 在 GimbalTask 主循环中以 2ms 周期调用, timeout_ms 取 100。
 */
void CDC_Transmit_FS_Watchdog(uint32_t timeout_ms)
{
  static uint32_t stuck_start_tick = 0U;
  USBD_CDC_HandleTypeDef *hcdc;
  PCD_HandleTypeDef *hpcd;
  uint32_t epnum;
  uint32_t USBx_BASE;   /* USBx_INEP() 宏依赖此局部变量,HAL 内部同样如此 */

  if (hUsbDeviceFS.pClassData == NULL)
  {
    stuck_start_tick = 0U;
    return;
  }

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    stuck_start_tick = 0U;
    return;
  }

  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  if (hcdc->TxState != 0U)
  {
    if (stuck_start_tick == 0U)
    {
      stuck_start_tick = HAL_GetTick();
    }
    else if ((HAL_GetTick() - stuck_start_tick) > timeout_ms)
    {
      /* 1. 清 TxState 软件标志 */
      hcdc->TxState = 0U;

      /* 2/3/4. 刷新 USB IN 端点硬件状态 */
      hpcd = (PCD_HandleTypeDef *)hUsbDeviceFS.pData;
      if (hpcd != NULL)
      {
        epnum = CDC_IN_EP & 0x7FU;
        USBx_BASE = (uint32_t)hpcd->Instance;   /* 等同于 USBx */

        /* 2. 清空 TX FIFO (HAL_PCD_EP_Flush 内部只调 USB_FlushTxFifo,
         *    不清端点使能状态和中断标志,所以下面还要手动清) */
        HAL_PCD_EP_Flush(hpcd, CDC_IN_EP);

        /* 3. 清 DIEPINT 挂起的中断标志 (写1清零) */
        USBx_INEP(epnum)->DIEPINT = 0xFFFFFFFFU;

        /* 4. 清 HAL 软件层 IN_ep 状态,防止 HAL_PCD_EP_Transmit 误判 */
        if (hpcd->IN_ep[epnum].xfer_len > 0U)
        {
          hpcd->IN_ep[epnum].xfer_len = 0U;
          hpcd->IN_ep[epnum].xfer_buff = NULL;
          hpcd->IN_ep[epnum].xfer_count = 0U;
        }
      }

      usb_cdc_tx_reset_count++;
      stuck_start_tick = 0U;
    }
  }
  else
  {
    stuck_start_tick = 0U;
  }
}

/* USER CODE END 8 */

/**
 * @brief  CDC_TransmitCplt_FS
 *         Data transmitted callback
 *
 *         @note
 *         This function is IN transfer complete callback used to inform user that
 *         the submitted Data is successfully sent over USB.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @}
 */

/**
 * @}
 */
