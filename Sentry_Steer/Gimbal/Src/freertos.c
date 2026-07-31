/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led_flow_task.h"
#include "ins_task.h"
#include "ActionTask.h"
#include "BlueToothTask.h"
#include "ChassisTask.h"
#include "CPU_Task.h"
#include "GimbalTask.h"
#include "iwdgTask.h"
#include "Offline_Task.h"
// #include "SDCardTask.h"
#include "Test_Task.h"
#include "ShootTask.h"
#include "BuzzerAlarmTask.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId INSTaskHandle;
osThreadId ActionTaskHandle;
osThreadId OfflineTaskHandle;
osThreadId GimbalTaskHandle;
osThreadId BlueToothTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId CPUTaskHandle;
osThreadId iwdgTaskHandle;
osThreadId TestTaskHandle;
osThreadId BuzzerAlarmTaskHandle;
//osThreadId ShootTaskHandle;
/* USER CODE END Variables */
osThreadId testHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of test */
//  osThreadDef(test, test_task, osPriorityNormal, 0, 128);
//  testHandle = osThreadCreate(osThread(test), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	// INS Task
  osThreadDef(INSTask_, INS_task, osPriorityRealtime, 0, 512);
  INSTaskHandle = osThreadCreate(osThread(INSTask_), NULL);

  // Action Task
  osThreadDef(ActionTask_, ActionTask, osPriorityRealtime, 0, 256);
  ActionTaskHandle = osThreadCreate(osThread(ActionTask_), NULL);

  // Gimbal Task
  osThreadDef(GimbalTask_, GimbalTask, osPriorityRealtime, 0, 512);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask_), NULL);

  // Offline Task
  osThreadDef(Offline_task_, Offline_task, osPriorityRealtime, 0, 128);
  OfflineTaskHandle = osThreadCreate(osThread(Offline_task_), NULL);

  // Chassis Task
  osThreadDef(ChassisTask_, ChassisTask, osPriorityRealtime, 0, 512);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask_), NULL);

  // Buzzer Alarm Task (电机掉线蜂鸣器告警, 2026-07-19新增)
  osThreadDef(BuzzerAlarmTask_, BuzzerAlarmTask, osPriorityNormal, 0, 256);
  BuzzerAlarmTaskHandle = osThreadCreate(osThread(BuzzerAlarmTask_), NULL);

#if IWDG_TASK_ON
  // Independent watchdog supervisor. It feeds only after all critical tasks report.
  osThreadDef(IwdgTask_, Iwdg_task, osPriorityHigh, 0, 128);
  iwdgTaskHandle = osThreadCreate(osThread(IwdgTask_), NULL);
#endif
  // Shoot Task
//  osThreadDef(ShootTask_, ShootTask, osPriorityRealtime, 0, 256);
//  ShootTaskHandle = osThreadCreate(osThread(ShootTask_), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_test_task */
void test_task(void const * argument)
{
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
