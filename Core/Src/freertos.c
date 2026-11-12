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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* 栈高水位（单位 words，剩余量，越小越危险）
 * [0] crg [1] imu [2] gimbal [3] shoot [4] chassis [5] referee [6] ui [7] debug */
volatile uint16_t chassis_task_stack_high_water_word[8];
volatile uint8_t chassis_task_stack_overflow_flag = 0U;
volatile char chassis_task_stack_overflow_name[configMAX_TASK_NAME_LEN];
/* USER CODE END Variables */
/* Definitions for crg_test_task */
osThreadId_t crg_test_taskHandle;
uint32_t crg_test_taskBuffer[ 512 ];
osStaticThreadDef_t crg_test_taskControlBlock;
const osThreadAttr_t crg_test_task_attributes = {
  .name = "crg_test_task",
  .cb_mem = &crg_test_taskControlBlock,
  .cb_size = sizeof(crg_test_taskControlBlock),
  .stack_mem = &crg_test_taskBuffer[0],
  .stack_size = sizeof(crg_test_taskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for imu_task */
osThreadId_t imu_taskHandle;
uint32_t imu_taskBuffer[ 1024 ];
osStaticThreadDef_t imu_taskControlBlock;
const osThreadAttr_t imu_task_attributes = {
  .name = "imu_task",
  .cb_mem = &imu_taskControlBlock,
  .cb_size = sizeof(imu_taskControlBlock),
  .stack_mem = &imu_taskBuffer[0],
  .stack_size = sizeof(imu_taskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for gimbal_task */
osThreadId_t gimbal_taskHandle;
uint32_t gimbal_taskBuffer[ 1024 ];
osStaticThreadDef_t gimbal_taskControlBlock;
const osThreadAttr_t gimbal_task_attributes = {
  .name = "gimbal_task",
  .cb_mem = &gimbal_taskControlBlock,
  .cb_size = sizeof(gimbal_taskControlBlock),
  .stack_mem = &gimbal_taskBuffer[0],
  .stack_size = sizeof(gimbal_taskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for shoot_task */
osThreadId_t shoot_taskHandle;
uint32_t shoot_taskBuffer[ 512 ];
osStaticThreadDef_t shoot_taskControlBlock;
const osThreadAttr_t shoot_task_attributes = {
  .name = "shoot_task",
  .cb_mem = &shoot_taskControlBlock,
  .cb_size = sizeof(shoot_taskControlBlock),
  .stack_mem = &shoot_taskBuffer[0],
  .stack_size = sizeof(shoot_taskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassis_task */
osThreadId_t chassis_taskHandle;
uint32_t chassis_taskBuffer[ 1024 ];
osStaticThreadDef_t chassis_taskControlBlock;
const osThreadAttr_t chassis_task_attributes = {
  .name = "chassis_task",
  .cb_mem = &chassis_taskControlBlock,
  .cb_size = sizeof(chassis_taskControlBlock),
  .stack_mem = &chassis_taskBuffer[0],
  .stack_size = sizeof(chassis_taskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for referee_usart_t */
osThreadId_t referee_usart_tHandle;
uint32_t referee_usart_tBuffer[ 512 ];
osStaticThreadDef_t referee_usart_tControlBlock;
const osThreadAttr_t referee_usart_t_attributes = {
  .name = "referee_usart_t",
  .cb_mem = &referee_usart_tControlBlock,
  .cb_size = sizeof(referee_usart_tControlBlock),
  .stack_mem = &referee_usart_tBuffer[0],
  .stack_size = sizeof(referee_usart_tBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ui_task */
osThreadId_t ui_taskHandle;
uint32_t ui_taskBuffer[ 512 ];
osStaticThreadDef_t ui_taskControlBlock;
const osThreadAttr_t ui_task_attributes = {
  .name = "ui_task",
  .cb_mem = &ui_taskControlBlock,
  .cb_size = sizeof(ui_taskControlBlock),
  .stack_mem = &ui_taskBuffer[0],
  .stack_size = sizeof(ui_taskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for debug_task */
osThreadId_t debug_taskHandle;
uint32_t debug_taskBuffer[ 512 ];
osStaticThreadDef_t debug_taskControlBlock;
const osThreadAttr_t debug_task_attributes = {
  .name = "debug_task",
  .cb_mem = &debug_taskControlBlock,
  .cb_size = sizeof(debug_taskControlBlock),
  .stack_mem = &debug_taskBuffer[0],
  .stack_size = sizeof(debug_taskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Chassis_Task_Report_Update(void);
/* USER CODE END FunctionPrototypes */

void Crg_test_Task(void *argument);
void Imu_Task(void *argument);
void Gimbal_Task(void *argument);
void Shoot_Task(void *argument);
void Chassis_Task(void *argument);
void Referee_Usart_Task(void *argument);
void Ui_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of crg_test_task */
  crg_test_taskHandle = osThreadNew(Crg_test_Task, NULL, &crg_test_task_attributes);

  /* creation of imu_task */
  imu_taskHandle = osThreadNew(Imu_Task, NULL, &imu_task_attributes);

  /* creation of gimbal_task */
  gimbal_taskHandle = osThreadNew(Gimbal_Task, NULL, &gimbal_task_attributes);

  /* creation of shoot_task */
  shoot_taskHandle = osThreadNew(Shoot_Task, NULL, &shoot_task_attributes);

  /* creation of chassis_task */
  chassis_taskHandle = osThreadNew(Chassis_Task, NULL, &chassis_task_attributes);

  /* creation of referee_usart_t */
  referee_usart_tHandle = osThreadNew(Referee_Usart_Task, NULL, &referee_usart_t_attributes);

  /* creation of ui_task */
  ui_taskHandle = osThreadNew(Ui_Task, NULL, &ui_task_attributes);

  /* creation of debug_task */
  debug_taskHandle = osThreadNew(Crg_test_Task, NULL, &debug_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Crg_test_Task */
/**
  * @brief  Function implementing the crg_test_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Crg_test_Task */
__weak void Crg_test_Task(void *argument)
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
  * @brief  Function implementing the imu_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Imu_Task */
__weak void Imu_Task(void *argument)
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
__weak void Gimbal_Task(void *argument)
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
__weak void Shoot_Task(void *argument)
{
  /* USER CODE BEGIN Shoot_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the chassis_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Referee_Usart_Task */
/**
* @brief Function implementing the referee_usart_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Usart_Task */
__weak void Referee_Usart_Task(void *argument)
{
  /* USER CODE BEGIN Referee_Usart_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Usart_Task */
}

/* USER CODE BEGIN Header_Ui_Task */
/**
* @brief Function implementing the ui_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ui_Task */
__weak void Ui_Task(void *argument)
{
  /* USER CODE BEGIN Ui_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ui_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Chassis_Task_Report_Update(void)
{
  chassis_task_stack_high_water_word[0] = (uint16_t)uxTaskGetStackHighWaterMark(crg_test_taskHandle);
  chassis_task_stack_high_water_word[1] = (uint16_t)uxTaskGetStackHighWaterMark(imu_taskHandle);
  chassis_task_stack_high_water_word[2] = (uint16_t)uxTaskGetStackHighWaterMark(gimbal_taskHandle);
  chassis_task_stack_high_water_word[3] = (uint16_t)uxTaskGetStackHighWaterMark(shoot_taskHandle);
  chassis_task_stack_high_water_word[4] = (uint16_t)uxTaskGetStackHighWaterMark(chassis_taskHandle);
  chassis_task_stack_high_water_word[5] = (uint16_t)uxTaskGetStackHighWaterMark(referee_usart_tHandle);
  chassis_task_stack_high_water_word[6] = (uint16_t)uxTaskGetStackHighWaterMark(ui_taskHandle);
  chassis_task_stack_high_water_word[7] = (uint16_t)uxTaskGetStackHighWaterMark(debug_taskHandle);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  chassis_task_stack_overflow_flag = 1U;
  (void)strncpy((char *)chassis_task_stack_overflow_name, pcTaskName, configMAX_TASK_NAME_LEN - 1U);
  chassis_task_stack_overflow_name[configMAX_TASK_NAME_LEN - 1U] = '\0';
  NVIC_SystemReset();
}
/* USER CODE END Application */

