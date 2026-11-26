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

/* USER CODE END Variables */
osThreadId crg_test_taskHandle;
osThreadId imu_taskHandle;
osThreadId gimbal_taskHandle;
osThreadId shoot_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId usb_taskHandle;
osThreadId chassis_data_taskHandle;
void Usb_Task(void const * argument);
void Chassis_Data_Task(void const * argument);
/* USER CODE END FunctionPrototypes */

void Crg_test_Task(void const * argument);
void Imu_Task(void const * argument);
void Gimbal_Task(void const * argument);
void Shoot_Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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
  /* definition and creation of crg_test_task */
  osThreadDef(crg_test_task, Crg_test_Task, osPriorityNormal, 0, 256);
  crg_test_taskHandle = osThreadCreate(osThread(crg_test_task), NULL);

  /* definition and creation of imu_task */
  osThreadDef(imu_task, Imu_Task, osPriorityHigh, 0, 1024);
  imu_taskHandle = osThreadCreate(osThread(imu_task), NULL);

  /* definition and creation of gimbal_task */
  osThreadDef(gimbal_task, Gimbal_Task, osPriorityNormal, 0, 1024);
  gimbal_taskHandle = osThreadCreate(osThread(gimbal_task), NULL);

  /* definition and creation of shoot_task */
  osThreadDef(shoot_task, Shoot_Task, osPriorityLow, 0, 256);
  shoot_taskHandle = osThreadCreate(osThread(shoot_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  
  osThreadDef(usb_task,Usb_Task,osPriorityNormal,0,256);
  usb_taskHandle = osThreadCreate(osThread(usb_task), NULL);

  osThreadDef(chassis_data_task,Chassis_Data_Task,osPriorityAboveNormal,0,512);
  chassis_data_taskHandle = osThreadCreate(osThread(chassis_data_task), NULL);
	
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Crg_test_Task */
/**
  * @brief  Function implementing the crg_test_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Crg_test_Task */
__weak void Crg_test_Task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Crg_test_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Crg_test_Task */
}

/* USER CODE BEGIN Header_Imu_Task */
/**
* @brief Function implementing the imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Imu_Task */
__weak void Imu_Task(void const * argument)
{
  /* USER CODE BEGIN Imu_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Imu_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the gimbal_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
__weak void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the shoot_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
__weak void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
