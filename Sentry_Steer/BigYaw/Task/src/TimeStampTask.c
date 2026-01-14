#include "TimeStampTask.h"

//#pragma import(__use_no_semihosting)             
////标准库需要的支持函数                 
//struct __FILE 
//{ 
//	int handle; 

//}; 

//FILE __stdout;       
////定义_sys_exit()以避免使用半主机模式    
//_sys_exit(int x) 
//{ 
//	x = x; 
//} 
////重定义fputc函数 
//int fputc(int ch, FILE *f)
//{      
//	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//    USART1->DR = (uint8_t) ch;      
//	return ch;
//}

uint16_t varl=0;
uint16_t var_Exp=0;
uint16_t global_time;
char snum[7];
uint16_t shorttt=0;

char gprmcStr[7]="$GPRMC,";
int chckNum=0;
char chckNumChar[2];

int ss=0;
int mm=0;
int hh=0;

unsigned char result;
int i;
int checkNum(const char *gprmcContext)
{
    if (gprmcContext == NULL) 
    {
        return -1;
    }

    result = gprmcContext[1];

    for (i = 2; gprmcContext[i] != '*' && gprmcContext[i] && i<100; i++)
    {
        result ^= gprmcContext[i];
    }

    if (gprmcContext[i] != '*') 
    {
        //printf("No '*' found in the string.\n");
        return -1;
    }
		return result;
	}

char value_1[100]="";
char value_2[100]="";
char value_time[10]="";

char test[100]="$GPRMC,004015,A,2812.0498,N,11313.1361,E,0.0,180.0,150122,3.9,W,A*";

int testtime = 0;
void TimeStampTask(void const *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1000; // 1HZ

    while (1)
    {
			xLastWakeTime = xTaskGetTickCount();
			// 更新时间（直接在任务中计算，无需中断）
			if (ss < 59) ss++;
			else {
					ss = 0;
					if (mm < 59) mm++;
					else {
							mm = 0;
							if (hh < 23) hh++;
							else hh = 0;
					}
				}
			testtime++;

			// 格式化数据
			snprintf(value_2, sizeof(value_2), 
							"%s%02d%02d%02d.00,A,2237.496474,N,11356.089515,E,0.0,225.5,230520,2.3,W,A*",
							gprmcStr, hh, mm, ss);

			// 计算校验和
			int chckNum = checkNum(value_2);
			sprintf(chckNumChar, "%02X", chckNum);

			// 阻塞式UART发送（确保数据完整）
			HAL_UART_Transmit(&huart1, (uint8_t*)value_2, strlen(value_2), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t*)chckNumChar, strlen(chckNumChar), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);  // 换行
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
   }
   
}