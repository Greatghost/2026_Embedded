#ifndef __ATK_MS53L0M_UART_H
#define __ATK_MS53L0M_UART_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h" 

#define     ATK_USART_BAUD_RATE                       115200
#define     ATK_USARTx                                USART2
// usart clock
#define     ATK_USART_APBxClock_FUN                   RCC_APB1PeriphClockCmd
#define     ATK_USART_CLK                             RCC_APB1Periph_USART2
// GPIO clock
#define     ATK_USART_GPIO_APBxClock_FUN              RCC_AHB1PeriphClockCmd
#define     ATK_USART_GPIO_CLK                        RCC_AHB1Periph_GPIOA

//TX IO
#define     ATK_USART_TX_PORT                         GPIOA   
#define     ATK_USART_TX_PIN                          GPIO_Pin_2
#define     ATK_USART_TX_AF                           GPIO_AF_USART2
#define     ATK_USART_TX_SOURCE                       GPIO_PinSource2 

//RX IO
#define     ATK_USART_RX_PORT                         GPIOA
#define     ATK_USART_RX_PIN                          GPIO_Pin_3
#define     ATK_USART_RX_AF                           GPIO_AF_USART2
#define     ATK_USART_RX_SOURCE                       GPIO_PinSource3

//TX_DMA
#define     ATK_DMA_TX_AHBxClock_FUN                 RCC_AHB1PeriphClockCmd
#define     ATK_DMA_TX_CLK                           RCC_AHB1Periph_DMA1

//TX_DMA Channel
#define	    ATK_DMA_TX_STREAM                         DMA1_Stream6
#define	    ATK_DMA_TX_CHANNEL                        DMA_Channel_4
#define     ATK_DMA_TX_IRQ                            DMA1_Stream6_IRQn
#define     ATK_DMA_TX_INT_FUN                        DMA1_Stream6_IRQHandler
#define     ATK_DMA_TX_FLAG_TCIF                      DMA_FLAG_TCIF6
#define     ATK_DMA_TX_IT_STATUS                      DMA_IT_TCIF6

//RX_DMA Channel
#define	    ATK_DMA_RX_STREAM                         DMA1_Stream5
#define	    ATK_DMA_RX_CHANNEL                        DMA_Channel_4
#define     ATK_DMA_RX_IRQ                            DMA1_Stream5_IRQn
#define     ATK_DMA_RX_INT_FUN                        DMA1_Stream5_IRQHandler
#define     ATK_DMA_RX_FLAG_TCIF                      DMA_FLAG_TCIF5  // transmision completed
#define     ATK_DMA_RX_IT_STATUS                      DMA_IT_TCIF5

/* PC UART */
#define PC_UART_GPIOx_CLOCK_CMD RCC_AHB1PeriphClockCmd
#define PC_UART_RCC_AxBxPeriph_GPIOx RCC_AHB1Periph_GPIOA
#define PC_UART_UARTx_CLOCK_CMD RCC_APB1PeriphClockCmd
#define PC_UART_RCC_AxBxPeriph_UARTx RCC_APB1Periph_USART2
#define PC_UART_GPIOx GPIOA
#define PC_UART_UARTx USART2
#define PC_UART_GPIO_AF_USARTx GPIO_AF_USART2
#define PC_UART_GPIO_PinSourcex1 GPIO_PinSource2
#define PC_UART_GPIO_PinSourcex2 GPIO_PinSource3
#define PC_UART_GPIO_Pin_x1 GPIO_Pin_2
#define PC_UART_GPIO_Pin_x2 GPIO_Pin_3
#define PC_UART_BaudRate 115200

#define PC_UART_RECV_DMAx_Streamx_IRQn DMA1_Stream5_IRQn
#define PC_UART_RECV_DMAx_Streamx DMA1_Stream5
#define PC_UART_RECV_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define PC_UART_RECV_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA1
#define PC_UART_RECV_DMA_Channel_x DMA_Channel_4
#define PC_UART_RECV_DMAx_Streamx_IRQHandler DMA1_Stream5_IRQHandler
#define PC_UART_RECV_DMA_FLAG_TCIFx DMA_FLAG_TCIF5
#define PC_UART_RECV_DMA_IT_TCIFx DMA_IT_TCIF5

#define PC_UART_SEND_DMAx_Streamx_IRQn DMA1_Stream6_IRQn
#define PC_UART_SEND_DMAx_Streamx DMA1_Stream6
#define PC_UART_SEND_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define PC_UART_SEND_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA1
#define PC_UART_SEND_DMA_Channel_x DMA_Channel_4
#define PC_UART_SEND_DMAx_Streamx_IRQHandler DMA1_Stream6_IRQHandler
#define PC_UART_SEND_DMA_FLAG_TCIFx DMA_FLAG_TCIF6
#define PC_UART_SEND_DMA_IT_TCIFx DMA_IT_TCIF6

#define PC_UART_RECV_USARTx_IRQn USART2_IRQn
#define PC_UART_RECV_USARTx_IRQHandler USART2_IRQHandler

	
/* UART�շ������С */
#define ATK_MS53L0M_UART_RX_BUF_SIZE            128
#define ATK_MS53L0M_UART_TX_BUF_SIZE            128

typedef struct __AtkRxFrame_t {
    uint8_t buf[ATK_MS53L0M_UART_RX_BUF_SIZE];              /* Ö¡½ÓÊÕ»º³å */
    struct
    {
        uint16_t len    : 15;                               /* Ö¡½ÓÊÕ³¤¶È£¬sta[14:0] */
        uint16_t finsh  : 1;                                /* Ö¡½ÓÊÕÍê³É±êÖ¾£¬sta[15] */
    } sta;                                                  /* Ö¡×´Ì¬ÐÅÏ¢ */
} AtkRxFrame_t;

/* �������� */
void ATK_Configuration(void);
void atk_ms53l0m_uart_send(uint8_t *dat, uint8_t len);  /* ATK-MS53L0M UART�������� */
void atk_ms53l0m_uart_rx_restart(void);                 /* ATK-MS53L0M UART���¿�ʼ�������� */
uint8_t *atk_ms53l0m_uart_rx_get_frame(void);           /* ��ȡATK-MS53L0M UART���յ���һ֡���� */
uint16_t atk_ms53l0m_uart_rx_get_frame_len(void);       /* ��ȡATK-MS53L0 UART���յ���һ֡���ݵĳ��� */
void atk_ms53l0m_uart_init(uint32_t baudrate);          /* ATK-MS53L0M UART��ʼ�� */

#endif
