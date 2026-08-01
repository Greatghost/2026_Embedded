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
#include "usb_device.h"
#include "usbd_core.h"
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
volatile uint32_t usb_cdc_tx_reset_count = 0U;  /* 完整 USB Device 恢复的累计次数 */
volatile uint32_t usb_cdc_not_configured_count = 0U; /* USB 未枚举时发送被拒的累计次数 */
volatile uint8_t  usb_cdc_last_dev_state = 0U;       /* 最近一次观察到的 USB 设备状态 */
volatile uint8_t  usb_cdc_host_port_open = 0U;
volatile uint32_t usb_cdc_host_closed_drop_count = 0U;
volatile USB_CDC_ResetDebug_t usb_cdc_reset_debug = {0};

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
  usb_cdc_host_port_open = 0U;
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
  usb_cdc_host_port_open = 0U;
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
    /* With wLength=0 ST passes the complete setup request in pbuf.  Linux
     * sets DTR (wValue bit 0) while /dev/ttyACM* is open. */
    if (pbuf != NULL)
    {
      const USBD_SetupReqTypedef *req = (const USBD_SetupReqTypedef *)pbuf;
      usb_cdc_host_port_open = ((req->wValue & 0x0001U) != 0U) ? 1U : 0U;
    }
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

  /* USB 未配置、已断开或类数据未就绪时拒绝发送。 */
  if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) || (hcdc == NULL))
  {
    usb_cdc_not_configured_count++;
    return USBD_FAIL;
  }

  /* An enumerated CDC device is not necessarily open on the NUC.  Do not
   * submit an IN transfer until the host asserts DTR, otherwise TxState can
   * legitimately remain busy forever and must not trigger re-enumeration. */
  if (usb_cdc_host_port_open == 0U)
  {
    usb_cdc_host_closed_drop_count++;
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
 * @brief USB CDC TxState recovery supervisor.
 *
 * Never alter TxState, endpoint registers or HAL endpoint bookkeeping while
 * the USB ISR can be using them.  If an IN transfer stays busy, disconnect and
 * de-initialize the complete USB device with OTG_FS IRQ masked.  Reinitialize
 * it after a short disconnect interval so the host performs a clean
 * enumeration and both the USB core and HAL state are rebuilt together.
 */
void CDC_Transmit_FS_Watchdog(uint32_t timeout_ms)
{
  static uint32_t stuck_start_tick = 0U;
  static uint32_t restart_tick = 0U;
  static uint8_t restart_pending = 0U;
  USBD_CDC_HandleTypeDef *hcdc;
  uint32_t now = HAL_GetTick();

  if (restart_pending != 0U)
  {
    if ((now - restart_tick) >= 250U)
    {
      MX_USB_DEVICE_Init();
      restart_pending = 0U;
      usb_cdc_reset_debug.restart_pending = 0U;
    }
    return;
  }

  /* Do not diagnose TxState while no process owns the NUC serial port. */
  if (usb_cdc_host_port_open == 0U)
  {
    stuck_start_tick = 0U;
    return;
  }

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
      stuck_start_tick = now;
    }
    else if ((now - stuck_start_tick) > timeout_ms)
    {
      /* USBD_DeInit 会清状态，所以先保存复位现场供调试器查看。 */
      usb_cdc_reset_debug.total_count++;
      usb_cdc_reset_debug.tx_stuck_count++;
      usb_cdc_reset_debug.last_tick_ms = now;
      usb_cdc_reset_debug.last_stuck_ms = now - stuck_start_tick;
      usb_cdc_reset_debug.last_timeout_ms = timeout_ms;
      usb_cdc_reset_debug.last_reason = USB_CDC_RESET_REASON_TX_STUCK;
      usb_cdc_reset_debug.last_dev_state = hUsbDeviceFS.dev_state;
      usb_cdc_reset_debug.last_tx_state = (uint8_t)hcdc->TxState;
      usb_cdc_reset_debug.restart_pending = 1U;

      HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
      __DSB();
      __ISB();
      (void)USBD_DeInit(&hUsbDeviceFS);
      HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);

      restart_tick = now;
      restart_pending = 1U;
      usb_cdc_tx_reset_count++;
      stuck_start_tick = 0U;
    }
  }
  else
  {
    stuck_start_tick = 0U;
  }
}

void USB_CDC_RecordHostBusReset(void)
{
  usb_cdc_reset_debug.total_count++;
  usb_cdc_reset_debug.host_bus_count++;
  usb_cdc_reset_debug.last_tick_ms = HAL_GetTick();
  usb_cdc_reset_debug.last_stuck_ms = 0U;
  usb_cdc_reset_debug.last_timeout_ms = 0U;
  usb_cdc_reset_debug.last_reason = USB_CDC_RESET_REASON_HOST_BUS;
  usb_cdc_reset_debug.last_dev_state = hUsbDeviceFS.dev_state;
  usb_cdc_reset_debug.last_tx_state = 0U;
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
