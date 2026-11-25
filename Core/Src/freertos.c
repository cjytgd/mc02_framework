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
#include"../../User/Inc/bsp_motor.h"
#include"../../User/Inc/bsp_uart.h"
#include"../../User/Inc/bsp_led.h"
#include"../../User/Inc/bsp_pid.h"
#include"../../User/Inc/bsp_can.h"
#include"../../User/Inc/bsp_contrl.h"
#include"../../User/Inc/bsp_imu.h"
#include"../../User/Inc/bsp_attitude.h"
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
volatile float target_speed = 0.0f;
volatile uint8_t offline = 0;
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 526 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void motorTask(void *argument);
void UartTask(void *argument);
void watchDog(void *argument);
void attitideTask(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(motorTask, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(UartTask, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(watchDog, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(attitideTask, NULL, &myTask05_attributes);

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

/* USER CODE BEGIN Header_motorTask */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motorTask */
void motorTask(void *argument)
{
  /* USER CODE BEGIN motorTask */
  /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  pid_set_angle(1.1f,0.00f,0.0f);
  pid_set_speed(3.0f,0.6f,0.0f);
  //s1为2不使能；s1为3使能
  //s2为1进行电机速度PID调控，s2为3进行小陀螺模式
  for(;;) {
    //uint32_t now1 = xTaskGetTickCount();
    //   float recycle = (now1 %  2000) * ( 2 * 3.14f) / 2000;
    //   float duty =  (sin(recycle) + 1.0f) / 2.0f;
    //float angle = rx_data.me_angle ;
     if (contrl_data.s1 == 2) {
       contrl_data.channel_0 = 0;
     }
     else if (contrl_data.s1 == 3) {
       if (contrl_data.s2 == 3) {
         //进行小陀螺
       }
      else if (contrl_data.s2 ==1) {
        float n_speed = rx_data.speed ;

        //pid_speed.target_speed = pid_math_angle(3000.0f,angle ,0.001f,0.9f);
        // if (pid_speed.target_speed > 500.0f){pid_speed.target_speed = 500.0f;}
        //if (pid_speed.target_speed < -500.0f){pid_speed.target_speed = -500.0f;}
        int16_t ch = contrl_data.channel_0;
        if (offline == 1) {
          ch  = 0;
        }
        target_speed =  ch * 1.5f;
        float output = pid_math_speed(target_speed,n_speed ,0.001f,0.9f);
        //motor_set_I(M3508,id2,300);
        motor_set_I(M3508,id2, (int16_t)output);
        xTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(1));
      }
    }
  }
  /* USER CODE END motorTask */
}

/* USER CODE BEGIN Header_UartTask */
  /**
  * @brief Function implementing the myTask03 thread.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_UartTask */
void UartTask(void *argument)
{
  /* USER CODE BEGIN UartTask */
    /* Infinite loop */
    for(;;)
    {
      uint16_t current_me_angle = rx_data.me_angle;
      int16_t current_speed = rx_data.speed;
      int16_t current_temperature = rx_data.temperature;
      int16_t current_I = rx_data.current;
      int16_t target = pid_angle.target;
      int16_t error =pid_angle.error;
      int8_t d = contrl_data.s2;
      int16_t contrl_speed = contrl_data.channel_0*1.5f;
      uart_start("%d,%d,%d\r\n",(int16_t)attitude_data.roll,(int16_t)attitude_data.pitch,(int16_t)attitude_data.yaw);
      osDelay(1);
    }
  /* USER CODE END UartTask */
}

/* USER CODE BEGIN Header_watchDog */
  /**
  * @brief Function implementing the myTask04 thread.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_watchDog */
void watchDog(void *argument)
{
  /* USER CODE BEGIN watchDog */
    /* Infinite loop */

    for(;;)
    {uint32_t now = xTaskGetTickCount();
      // if (now - rx_data.time > pdMS_TO_TICKS(1000)) {
      //   bsp_led_task(2000);
      //   bsp_led_set(normal,250,0,0);
      // }
      // else {
      //   bsp_led_task(2000);
      //   bsp_led_set(normal,0,250,0);
      // }

      if (now - contrl_data.time > pdMS_TO_TICKS(1000)) {
        bsp_led_task(2000);
        //bsp_led_set(error,250,0,0);
        offline = 1;

      }
      else {
        bsp_led_task(2000);
        //bsp_led_set(normal,0,250,0);
        offline = 0;
      }

      osDelay(10);
    }
  /* USER CODE END watchDog */
}

/* USER CODE BEGIN Header_attitideTask */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_attitideTask */
void attitideTask(void *argument)
{
  /* USER CODE BEGIN attitideTask */
  /* Infinite loop */
  IMU_Data_t imu;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;)
  { IMU_GetData(&imu);
    Attitude_Update(&imu);
    xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));

  }
  /* USER CODE END attitideTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

