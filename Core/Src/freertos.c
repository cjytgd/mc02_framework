/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"semphr.h"
#include"../../User/Inc/bsp_uart.h"
#include"stdio.h"
#include"../../User/Inc/bsp_led.h"
#include"../../User/Inc/bsp_buzzer.h"
#include"../../User/Inc/bsp_imu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern volatile uint8_t flag;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 526 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for uartQueue */
osMessageQueueId_t uartQueueHandle;
const osMessageQueueAttr_t uartQueue_attributes = {
  .name = "uartQueue"
};
/* Definitions for xUARTMutex */
osMutexId_t xUARTMutexHandle;
const osMutexAttr_t xUARTMutex_attributes = {
  .name = "xUARTMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void UART1Task(void *argument);
void UART2Task(void *argument);
void LEDTask(void *argument);
void BMI088(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of xUARTMutex */
  xUARTMutexHandle = osMutexNew(&xUARTMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uartQueue */
  uartQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &uartQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(UART1Task, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(UART2Task, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(LEDTask, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(BMI088, NULL, &myTask05_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_UART1Task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART1Task */
void UART1Task(void *argument)
{
  /* USER CODE BEGIN UART1Task */
  /* Infinite loop */
  for(;;)
  {xSemaphoreTake(xUARTMutexHandle,portMAX_DELAY);
    uart_start("task1\r\n");
    xSemaphoreGive(xUARTMutexHandle);
    osDelay(200);
  }
  /* USER CODE END UART1Task */
}

/* USER CODE BEGIN Header_UART2Task */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART2Task */
void UART2Task(void *argument)
{
  /* USER CODE BEGIN UART2Task */
  /* Infinite loop */
  for(;;)
  {xSemaphoreTake(xUARTMutexHandle, portMAX_DELAY);
    uart_start("task2\r\n");
    xSemaphoreGive(xUARTMutexHandle);
    osDelay(200);
  }
  /* USER CODE END UART2Task */
}

/* USER CODE BEGIN Header_LEDTask */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTask */
void LEDTask(void *argument)
{
  /* USER CODE BEGIN LEDTask */
  /* Infinite loop */
  uint8_t buf = 0;;
  for(;;)
  {xQueueReceive(uartQueueHandle,(uint8_t*)&buf,0);
    // if (buf != buf ) {
    //     buf = buf;
         if (buf == 1) {bsp_led_set(init,0,0,0);}
         else if (buf == 2) {
           bsp_led_set(normal,250,0,0);
           xSemaphoreTake(xUARTMutexHandle, portMAX_DELAY);
           uart_start("working normally\n");
           xSemaphoreGive(xUARTMutexHandle);
         }
         else if (buf == 3) {
           bsp_led_set(error,250,0,0);
           xSemaphoreTake(xUARTMutexHandle, portMAX_DELAY);
           uart_start("working error\n");
           xSemaphoreGive(xUARTMutexHandle);

       }
    if (flag == 1) {
           flag = 0;
       buzzer_start(1000,1000);
         }
         uint32_t now = HAL_GetTick();
         bsp_led_task(2000);
         buzzer_task();

      //
  }
  /* USER CODE END LEDTask */
}

/* USER CODE BEGIN Header_BMI088 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMI088 */
void BMI088(void *argument)
{
  /* USER CODE BEGIN BMI088 */
  /* Infinite loop */
  IMU_Data_t imu;
  for(;;)
  {IMU_GetData(&imu);
    xSemaphoreTake(xUARTMutexHandle, portMAX_DELAY);
    uart_start("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%f\n",imu.accel_x,imu.accel_y,imu.accel_z,imu.gyro_x,
    imu.gyro_y,imu.gyro_z,10.4);
    xSemaphoreGive(xUARTMutexHandle);
    osDelay(10);
  }
  /* USER CODE END BMI088 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

