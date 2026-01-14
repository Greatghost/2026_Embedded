#include "atk_ms53l0m_uart.h"

AtkRxFrame_t g_uart_rx_frame = {0};                              /* ATK-MS53L0M UART接收帧缓冲信息结构体 */
uint8_t ATK_send_Data[ATK_MS53L0M_UART_TX_BUF_SIZE];        /* 发送数据缓冲 */

void ATK_Configuration(void)
{
//	USART_InitTypeDef usart;
//	GPIO_InitTypeDef  gpio;
//	NVIC_InitTypeDef  nvic;
//	DMA_InitTypeDef   dma;
//	
//	nvic.NVIC_IRQChannel = ATK_DMA_TX_IRQ;
//	nvic.NVIC_IRQChannelPreemptionPriority = 0;
//	nvic.NVIC_IRQChannelSubPriority = 0;
//	nvic.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&nvic);
//	nvic.NVIC_IRQChannel = ATK_DMA_RX_IRQ;
//	nvic.NVIC_IRQChannelPreemptionPriority = 0;
//	nvic.NVIC_IRQChannelSubPriority = 1;
//	nvic.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&nvic);

//  // USART	
//	ATK_USART_GPIO_APBxClock_FUN(ATK_USART_GPIO_CLK,ENABLE);
//	ATK_USART_APBxClock_FUN(ATK_USART_CLK,ENABLE);

//	GPIO_PinAFConfig(ATK_USART_TX_PORT,ATK_USART_TX_SOURCE,ATK_USART_TX_AF);
//	GPIO_PinAFConfig(ATK_USART_RX_PORT,ATK_USART_RX_SOURCE,ATK_USART_RX_AF); 

//	gpio.GPIO_Pin = ATK_USART_TX_PIN;
//	gpio.GPIO_Mode = GPIO_Mode_AF;
//	gpio.GPIO_OType = GPIO_OType_PP;
//	gpio.GPIO_Speed = GPIO_Speed_100MHz;
//	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(ATK_USART_TX_PORT,&gpio);
//	
//	gpio.GPIO_Pin = ATK_USART_RX_PIN;
//	gpio.GPIO_Mode = GPIO_Mode_AF;
//	gpio.GPIO_OType = GPIO_OType_PP;
//	gpio.GPIO_Speed = GPIO_Speed_100MHz;
//	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(ATK_USART_RX_PORT,&gpio);

//	usart.USART_BaudRate = ATK_USART_BAUD_RATE;
//	usart.USART_WordLength = USART_WordLength_8b;
//	usart.USART_StopBits = USART_StopBits_1;
//	usart.USART_Parity = USART_Parity_No;
//	usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
//	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_Init(ATK_USARTx,&usart);
//	USART_ITConfig(ATK_USARTx, USART_IT_IDLE, ENABLE); // 空闲中断
//	USART_Cmd(ATK_USARTx,ENABLE);
//	USART_DMACmd(ATK_USARTx,USART_DMAReq_Tx,ENABLE);	
//	USART_DMACmd(ATK_USARTx,USART_DMAReq_Rx,ENABLE);
//	USART_ClearFlag(ATK_USARTx, USART_FLAG_TC);
//	// USART_ClearFlag(ATK_USARTx, USART_FLAG_IDLE);			 // 
//	(void)ATK_USARTx->SR;
//	(void)ATK_USARTx->DR;
//		
//	{// DMA
//		//TX
//		ATK_DMA_TX_AHBxClock_FUN(ATK_DMA_TX_CLK,ENABLE);
//		DMA_DeInit(ATK_DMA_TX_STREAM);
//		dma.DMA_Channel= ATK_DMA_TX_CHANNEL;
//		dma.DMA_PeripheralBaseAddr = (uint32_t)&(ATK_USARTx->DR);
//		dma.DMA_Memory0BaseAddr = (uint32_t)ATK_send_Data;
//		dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//		dma.DMA_BufferSize = (ATK_MS53L0M_UART_TX_BUF_SIZE);
//		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//		dma.DMA_Mode = DMA_Mode_Normal;
//		dma.DMA_Priority = DMA_Priority_VeryHigh;
//		dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//		dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//		dma.DMA_MemoryBurst = DMA_Mode_Normal;
//		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//		DMA_Init(ATK_DMA_TX_STREAM,&dma);
//		DMA_ClearFlag(ATK_DMA_TX_STREAM, ATK_DMA_TX_FLAG_TCIF);  // clear all DMA flags
//		DMA_ITConfig(ATK_DMA_TX_STREAM,DMA_IT_TC,ENABLE);  		   //open DMA send inttrupt
//		DMA_Cmd(ATK_DMA_TX_STREAM, DISABLE);										 // Disable
//		//RX
//		DMA_DeInit(ATK_DMA_RX_STREAM);
//		dma.DMA_Channel= ATK_DMA_RX_CHANNEL;
//		dma.DMA_PeripheralBaseAddr = (uint32_t)&(ATK_USARTx->DR);
//		dma.DMA_Memory0BaseAddr = (uint32_t)g_uart_rx_frame.buf;
//		dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
//		dma.DMA_BufferSize = (ATK_MS53L0M_UART_RX_BUF_SIZE);
//		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//		dma.DMA_Mode = DMA_Mode_Circular;
//		dma.DMA_Priority = DMA_Priority_Medium;
//		dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//		dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//		dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//		DMA_Init(ATK_DMA_RX_STREAM,&dma);
//		//DMA_ClearFlag(ATK_DMA_RX_STREAM, ATK_DMA_RX_FLAG_TCIF);  // clear all DMA flags
//		//DMA_SetCurrDataCounter(ATK_DMA_RX_STREAM, ATK_MS53L0M_UART_RX_BUF_SIZE);
//		DMA_ITConfig(ATK_DMA_RX_STREAM,DMA_IT_TC,ENABLE);  
//		DMA_Cmd(ATK_DMA_RX_STREAM, ENABLE);											 // Enable
//	}	

    USART_InitTypeDef usart;
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;

    PC_UART_GPIOx_CLOCK_CMD(PC_UART_RCC_AxBxPeriph_GPIOx, ENABLE);
    PC_UART_UARTx_CLOCK_CMD(PC_UART_RCC_AxBxPeriph_UARTx, ENABLE);

    GPIO_PinAFConfig(PC_UART_GPIOx, PC_UART_GPIO_PinSourcex1, PC_UART_GPIO_AF_USARTx);
    GPIO_PinAFConfig(PC_UART_GPIOx, PC_UART_GPIO_PinSourcex2, PC_UART_GPIO_AF_USARTx);

    gpio.GPIO_Pin = PC_UART_GPIO_Pin_x1 | PC_UART_GPIO_Pin_x2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(PC_UART_GPIOx, &gpio);

    USART_DeInit(PC_UART_UARTx);
    usart.USART_BaudRate = PC_UART_BaudRate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(PC_UART_UARTx, &usart);
    USART_Cmd(PC_UART_UARTx, ENABLE);

    USART_ITConfig(PC_UART_UARTx, USART_IT_IDLE, ENABLE);
    USART_DMACmd(PC_UART_UARTx, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(PC_UART_UARTx, USART_DMAReq_Tx, ENABLE);

    nvic.NVIC_IRQChannel = PC_UART_RECV_USARTx_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = PC_UART_SEND_DMAx_Streamx_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    { // RX
        DMA_InitTypeDef dma;
        PC_UART_RECV_RCC_AxBxPeriphClockCmd(PC_UART_RECV_RCC_AxBxPeriph_DMAx, ENABLE);
        DMA_DeInit(PC_UART_RECV_DMAx_Streamx);
        while (DMA_GetCmdStatus(PC_UART_RECV_DMAx_Streamx) != DISABLE)
        {
        };
        dma.DMA_Channel = PC_UART_RECV_DMA_Channel_x;
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (PC_UART_UARTx->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)g_uart_rx_frame.buf;
        dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
        dma.DMA_BufferSize = ATK_MS53L0M_UART_RX_BUF_SIZE;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma.DMA_Mode = DMA_Mode_Circular;
        dma.DMA_Priority = DMA_Priority_VeryHigh;
        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(PC_UART_RECV_DMAx_Streamx, &dma);
        DMA_ITConfig(PC_UART_RECV_DMAx_Streamx, DMA_IT_TC, ENABLE);
        DMA_Cmd(PC_UART_RECV_DMAx_Streamx, ENABLE);
    }

    { //  TX
        DMA_InitTypeDef dma;
        PC_UART_SEND_RCC_AxBxPeriphClockCmd(PC_UART_SEND_RCC_AxBxPeriph_DMAx, ENABLE);
        DMA_DeInit(PC_UART_SEND_DMAx_Streamx);
        while (DMA_GetCmdStatus(PC_UART_SEND_DMAx_Streamx) != DISABLE)
        {
        };
        dma.DMA_Channel = PC_UART_SEND_DMA_Channel_x;
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (PC_UART_UARTx->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)ATK_send_Data;
        dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        dma.DMA_BufferSize = ATK_MS53L0M_UART_TX_BUF_SIZE;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma.DMA_Mode = DMA_Mode_Normal;
        dma.DMA_Priority = DMA_Priority_VeryHigh;
        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(PC_UART_SEND_DMAx_Streamx, &dma);
        DMA_Cmd(PC_UART_SEND_DMAx_Streamx, DISABLE);
        DMA_ITConfig(PC_UART_SEND_DMAx_Streamx, DMA_IT_TC, ENABLE);
    }

}

/**
 * @brief       ATK-MS53L0M UART发送数据
 * @param       dat: 待发送的数据
 *              len: 待发送数据的长度
 * @retval      无
 */
void atk_ms53l0m_uart_send(uint8_t *dat, uint8_t len)
{
    memcpy(ATK_send_Data,dat,len);
	DMA_SetCurrDataCounter(ATK_DMA_TX_STREAM, len);
	DMA_Cmd(ATK_DMA_TX_STREAM, ENABLE);
}

/**
 * @brief       ATK-MS53L0M UART重新开始接收数据
 * @param       无
 * @retval      无
 */
void atk_ms53l0m_uart_rx_restart(void)
{
    g_uart_rx_frame.sta.len     = 0;
    g_uart_rx_frame.sta.finsh   = 0;
}

/**
 * @brief       获取ATK-MS53L0M UART接收到的一帧数据
 * @param       无
 * @retval      NULL: 未接收到一帧数据
 *              其他: 接收到的一帧数据
 */
uint8_t *atk_ms53l0m_uart_rx_get_frame(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = '\0';
        return g_uart_rx_frame.buf;
    }
    else
    {
        return NULL;
    }
}

/**
 * @brief       获取ATK-MS53L0 UART接收到的一帧数据的长度
 * @param       无
 * @retval      0   : 未接收到一帧数据
 *              其他: 接收到的一帧数据的长度
 */
uint16_t atk_ms53l0m_uart_rx_get_frame_len(void)
{
    if (g_uart_rx_frame.sta.finsh == 1) {
        return g_uart_rx_frame.sta.len;
    }
    else {
        return 0;
    }
}

float dt_tof = 0.0f; //检测中断频率
uint16_t recv_len;
TickType_t lastWakeTime, tempWakeTime;
// void ATK_DMA_RX_INT_FUN(void) {
// 	if(USART_GetITStatus(ATK_USARTx,USART_IT_IDLE) != RESET) {
// 		tempWakeTime = xTaskGetTickCount();
// 		dt_tof = (float)(tempWakeTime - lastWakeTime) /1000.0f;
// 		lastWakeTime = tempWakeTime;

// 		(void)ATK_USARTx->SR;
// 		(void)ATK_USARTx->DR;
// 		DMA_Cmd(ATK_DMA_RX_STREAM, DISABLE);
// 		DMA_ClearFlag(ATK_DMA_RX_STREAM, ATK_DMA_RX_FLAG_TCIF);
// 		DMA_ClearITPendingBit(ATK_DMA_RX_STREAM, ATK_DMA_RX_IT_STATUS);

// 		g_uart_rx_frame.sta.finsh = 1;      

// 		DMA_SetCurrDataCounter(ATK_DMA_RX_STREAM, ATK_MS53L0M_UART_RX_BUF_SIZE);
// 		DMA_Cmd(ATK_DMA_RX_STREAM, ENABLE);
// 	}

// 	if(DMA_GetITStatus(ATK_DMA_RX_STREAM,ATK_DMA_RX_IT_STATUS)) {
// 		DMA_Cmd(ATK_DMA_RX_STREAM, DISABLE);
// 		DMA_ClearITPendingBit(ATK_DMA_RX_STREAM, ATK_DMA_RX_IT_STATUS);
// 		/* process msg */
// 		DMA_SetCurrDataCounter(ATK_DMA_RX_STREAM, ATK_MS53L0M_UART_RX_BUF_SIZE);
// 		DMA_Cmd(ATK_DMA_RX_STREAM, ENABLE);
// 	}
// 	return;
// }

// /**
//  * @brief       ATK-MS53L0M UART发送DMA中断处理函数，清除DMA发送中断标志位
//  * @param       无
// */
// void ATK_DMA_TX_INT_FUN(void)
// {
// 	if(DMA_GetITStatus(ATK_DMA_TX_STREAM,ATK_DMA_TX_IT_STATUS  ))
// 	{
// 		DMA_ClearITPendingBit(ATK_DMA_TX_STREAM,ATK_DMA_TX_IT_STATUS);
// 		DMA_Cmd(ATK_DMA_TX_STREAM, DISABLE); 
// 	}
// }

/**
 * @brief 串口接收中断
 * @param[in] void
 */
void PC_UART_RECV_USARTx_IRQHandler(void)
{
    if (USART_GetITStatus(PC_UART_UARTx, USART_IT_IDLE) != RESET)
    {
        (void)PC_UART_UARTx->SR;
        (void)PC_UART_UARTx->DR;
        DMA_Cmd(PC_UART_RECV_DMAx_Streamx, DISABLE);
        DMA_ClearFlag(PC_UART_RECV_DMAx_Streamx, PC_UART_RECV_DMA_FLAG_TCIFx);
        DMA_ClearITPendingBit(PC_UART_RECV_DMAx_Streamx, PC_UART_RECV_DMA_IT_TCIFx);

        tempWakeTime = xTaskGetTickCount();
        dt_tof = (float)(tempWakeTime - lastWakeTime) /1000.0f;
        lastWakeTime = tempWakeTime;
        g_uart_rx_frame.sta.finsh = 1;    
				g_uart_rx_frame.sta.len =  ATK_MS53L0M_UART_RX_BUF_SIZE - DMA_GetCurrDataCounter(PC_UART_RECV_DMAx_Streamx);

        DMA_SetCurrDataCounter(PC_UART_RECV_DMAx_Streamx,  ATK_MS53L0M_UART_RX_BUF_SIZE);
        DMA_Cmd(PC_UART_RECV_DMAx_Streamx, ENABLE);
    }
}

/**
 * @brief 与PC通信的发送中断
 * @param[in] void
 */
void PC_UART_SEND_DMAx_Streamx_IRQHandler(void)
{
    if (DMA_GetITStatus(PC_UART_SEND_DMAx_Streamx, PC_UART_SEND_DMA_IT_TCIFx))
    {
        while (USART_GetFlagStatus(PC_UART_UARTx, USART_FLAG_TC) == RESET)
            ;
        DMA_ClearFlag(PC_UART_SEND_DMAx_Streamx, PC_UART_SEND_DMA_FLAG_TCIFx);
        DMA_ClearITPendingBit(PC_UART_SEND_DMAx_Streamx, PC_UART_SEND_DMA_IT_TCIFx);
        USART_ClearFlag(PC_UART_UARTx, USART_FLAG_TC);
    }
}
