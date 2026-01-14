#ifndef _REFEREE_H
#define _REFEREE_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Referee.h"
#include "robot_config.h"

//#if ROBOT == CHEN_JING_YUAN	//使用B板，更改can1和裁判系统uart串口配置
//#define REFEREE_GPIOx_CLOCK_CMD RCC_AHB1PeriphClockCmd
//#define REFEREE_RCC_AxBxPeriph_GPIOx RCC_AHB1Periph_GPIOA
//#define REFEREE_UARTx_CLOCK_CMD RCC_APB2PeriphClockCmd
//#define REFEREE_RCC_AxBxPeriph_UARTx RCC_APB2Periph_USART1
//#define REFEREE_GPIOx GPIOA
//#define REFEREE_UARTx USART1
//#define REFEREE_GPIO_AF_USARTx GPIO_AF_USART1
//#define REFEREE_GPIO_PinSourcex1 GPIO_PinSource9
//#define REFEREE_GPIO_PinSourcex2 GPIO_PinSource10
//#define REFEREE_GPIO_Pin_x1 GPIO_Pin_9
//#define REFEREE_GPIO_Pin_x2 GPIO_Pin_10
//#define REFEREE_BaudRate 115200
//	
//#define REFEREE_RECV_USARTx_IRQHandler USART1_IRQHandler
//#define REFEREE_RECV_USARTx_IRQn USART1_IRQn

//#define REFEREE_RECV_DMAx_Streamx_IRQn DMA2_Stream2_IRQn
//#define REFEREE_RECV_DMAx_Streamx DMA2_Stream2
//#define REFEREE_RECV_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
//#define REFEREE_RECV_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA2
//#define REFEREE_RECV_DMA_Channel_x DMA_Channel_4
//#define REFEREE_RECV_DMAx_Streamx_IRQHandler DMA2_Stream2_IRQHandler
//#define REFEREE_RECV_DMA_FLAG_TCIFx DMA_FLAG_TCIF2
//#define REFEREE_RECV_DMA_IT_TCIFx DMA_IT_TCIF2

//#define REFEREE_SEND_DMAx_Streamx_IRQn DMA2_Stream7_IRQn
//#define REFEREE_SEND_DMAx_Streamx DMA2_Stream7
//#define REFEREE_SEND_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
//#define REFEREE_SEND_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA2
//#define REFEREE_SEND_DMA_Channel_x DMA_Channel_4
//#define REFEREE_SEND_DMAx_Streamx_IRQHandler DMA2_Stream7_IRQHandler
//#define REFEREE_SEND_DMA_FLAG_TCIFx DMA_FLAG_TCIF4
//#define REFEREE_SEND_DMA_IT_TCIFx DMA_IT_TCIF4

//#define REFEREE_RECVBUF_SIZE 4096
//#define REFEREE_SENDBUF_SIZE 128

//#else//全向轮和舵轮使用上赛季老底盘板
#define REFEREE_GPIOx_CLOCK_CMD RCC_AHB1PeriphClockCmd
#define REFEREE_RCC_AxBxPeriph_GPIOx RCC_AHB1Periph_GPIOC
#define REFEREE_UARTx_CLOCK_CMD RCC_APB1PeriphClockCmd
#define REFEREE_RCC_AxBxPeriph_UARTx RCC_APB1Periph_UART4
#define REFEREE_GPIOx GPIOC
#define REFEREE_UARTx UART4
#define REFEREE_GPIO_AF_USARTx GPIO_AF_UART4
#define REFEREE_GPIO_PinSourcex1 GPIO_PinSource10
#define REFEREE_GPIO_PinSourcex2 GPIO_PinSource11
#define REFEREE_GPIO_Pin_x1 GPIO_Pin_10
#define REFEREE_GPIO_Pin_x2 GPIO_Pin_11
#define REFEREE_BaudRate 115200

#define REFEREE_RECV_USARTx_IRQHandler UART4_IRQHandler
#define REFEREE_RECV_USARTx_IRQn UART4_IRQn

#define REFEREE_RECV_DMAx_Streamx_IRQn DMA1_Stream2_IRQn
#define REFEREE_RECV_DMAx_Streamx DMA1_Stream2
#define REFEREE_RECV_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define REFEREE_RECV_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA1
#define REFEREE_RECV_DMA_Channel_x DMA_Channel_4
#define REFEREE_RECV_DMAx_Streamx_IRQHandler DMA1_Stream2_IRQHandler
#define REFEREE_RECV_DMA_FLAG_TCIFx DMA_FLAG_TCIF2
#define REFEREE_RECV_DMA_IT_TCIFx DMA_IT_TCIF2

#define REFEREE_SEND_DMAx_Streamx_IRQn DMA1_Stream4_IRQn
#define REFEREE_SEND_DMAx_Streamx DMA1_Stream4
#define REFEREE_SEND_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define REFEREE_SEND_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA1
#define REFEREE_SEND_DMA_Channel_x DMA_Channel_4
#define REFEREE_SEND_DMAx_Streamx_IRQHandler DMA1_Stream4_IRQHandler
#define REFEREE_SEND_DMA_FLAG_TCIFx DMA_FLAG_TCIF4
#define REFEREE_SEND_DMA_IT_TCIFx DMA_IT_TCIF4

#define REFEREE_RECVBUF_SIZE 4096
#define REFEREE_SENDBUF_SIZE 128

//#endif

/*  数据定义  */
extern uint8_t Refereebuffer[REFEREE_RECVBUF_SIZE];
extern uint8_t SendToReferee_Buff[REFEREE_SENDBUF_SIZE];

void REFEREE_Configuration(void);
void REFEREE_SendBytes(uint8_t *data, uint8_t len);

#endif // !_REFEREE_H
