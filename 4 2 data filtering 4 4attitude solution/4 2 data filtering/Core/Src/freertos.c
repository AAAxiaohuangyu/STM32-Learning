/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <MPU6050.h>
#include <data.h>
#include <kalman_filter.h>
#include <attitude.h>
#include <string.h>
#include <usart.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void TASK_MPU_F(void *ptr);
void TASK_TRANS_F(void *ptr);
void package_Init(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t TASK_MPU_HANDLE,TASK_TRANS_HANDLE;
#define TASK_MPU_STACK_SIZE 1024
#define TASK_MPU_PRIORITY osPriorityNormal
#define TASK_TRANS_STACK_SIZE 512
#define TASK_TRANS_PRIORITY osPriorityNormal
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  MPU6050_Init();
  KalmanFilter_Init();
  AttitudeSolver_Init();
  package_Init();
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

  /* USER CODE BEGIN RTOS_THREADS */
  TASK_MPU_HANDLE = osThreadNew(TASK_MPU_F,NULL,&(osThreadAttr_t){
    .name = "TASK_MPU",
    .stack_size = TASK_MPU_STACK_SIZE,
    .priority = TASK_MPU_PRIORITY,

  });

  TASK_TRANS_HANDLE = osThreadNew(TASK_TRANS_F,NULL,&(osThreadAttr_t){
    .name = "TASK_TRANS",
    .stack_size = TASK_TRANS_STACK_SIZE,
    .priority = TASK_TRANS_PRIORITY,
  });

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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void TASK_MPU_F(void *ptr){
  while (1)
  {
    MPU6050_Read();
    KalmanFilter_Update(Ax,Ay,Az,Gx,Gy,Gz,&ax_filt,&ay_filt,&az_filt,&gx_filt,&gy_filt,&gz_filt);
    AttitudeSolver_Update(ax_filt,ay_filt,az_filt,gx_filt,gy_filt,gz_filt,&roll,&pitch,&yaw);
    memcpy(package+4*0,&Ax,4);
    memcpy(package+4*1,&Ay,4);
    memcpy(package+4*2,&Az,4);
    memcpy(package+4*3,&Gx,4);
    memcpy(package+4*4,&Gy,4);
    memcpy(package+4*5,&Gz,4);
    memcpy(package+4*6,&ax_filt,4);
    memcpy(package+4*7,&ay_filt,4);
    memcpy(package+4*8,&az_filt,4);
    memcpy(package+4*9,&gx_filt,4);
    memcpy(package+4*10,&gy_filt,4);
    memcpy(package+4*11,&gz_filt,4);
    memcpy(package+4*12,&roll,4);
    memcpy(package+4*13,&pitch,4);
    memcpy(package+4*14,&yaw,4);
    
    osDelay(10);
  }
}

void TASK_TRANS_F(void *ptr){
  while(1){
    if(DMA_ready){
    DMA_ready = 0;
    
    HAL_UART_Transmit_DMA(&huart1,(const uint8_t *)package,channel*4+4);
    }
    osDelay(10);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if(huart ->Instance == USART1){
    DMA_ready = 1;
  }
}

void package_Init(void){
  package[channel*4+0]=0x00;
  package[channel*4+1]=0x00;
  package[channel*4+2]=0x80;
  package[channel*4+3]=0x7f;
}
/* USER CODE END Application */

