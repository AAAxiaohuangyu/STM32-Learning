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
#include <usart.h>
#include <data.h>
#include <string.h>
#include <encoder.h>
#include <stdio.h>
#include <screen.h>
#include <PID.h>
#include <motor.h>
#include <tim.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void command_process(uint8_t* data,uint16_t length);
void OLED(void *ptr);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t OLEDHandle;
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
  OLED_Init();
  OLED_ON();
  OLED_CLS();
  HAL_UARTEx_ReceiveToIdle_DMA (&huart1,buff,sizeof(buff));
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim1);
  package[channel*4+0]=0x00;
  package[channel*4+1]=0x00;
  package[channel*4+2]=0x80;
  package[channel*4+3]=0x7f;
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
  OLEDHandle = osThreadNew(OLED,NULL,&(osThreadAttr_t){
    .name = "OLED",
    .stack_size = 1024,
    .priority = osPriorityNormal,
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
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart ->Instance == USART1 ){ 
    command_process(buff,Size);
    HAL_UARTEx_ReceiveToIdle_DMA (&huart1,buff,sizeof(buff));  //命令处理
  }
}

void command_process(uint8_t* data,uint16_t length){
  float value = 0;
  char command[20];
  data[length]='\0';
  sscanf((char*)data,"%s %f",command,&value);
  if(strcmp(command,"Spead")==0){
    Spead_Tar = value; 
  }
  else if(strcmp(command,"Location")==0){
    Location_Tar = value;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }

  if(htim ->Instance == TIM1){
    Spead = (float)Get_Cur_Spead()*Spead_trans; //单位转换,速度单位为弧度每秒,通过减速比换算将磁铁的旋转转换为圆盘的旋转,将40ms的平均速度认为是瞬时速度
    Location +=Spead*0.04f; //累加为位置,单位为弧度



    nor_Spead_Tar = normalize_motor(Spead_Tar);
    nor_Spead = normalize_motor(Spead);
    nor_output = PID(nor_Spead_Tar,nor_Spead);
    output = (int8_t)denormalize_PWM(nor_output);
    PWM_SET(output);
  }
}

void OLED(void *ptr){
  char charbuff[32];
  while (1)
  {
    sprintf(charbuff,"Spead:%+06.1f",Spead);
    OLED_ShowString(0,0,charbuff);
    sprintf(charbuff,"Location:%+06.1f",Location);
    OLED_ShowString(0,2,charbuff);
    sprintf(charbuff,"output:%+04d",output);
    OLED_ShowString(0,4,charbuff);

    //发送数据给上位机
    memcpy(package+4*0,&Spead_Tar,4);
    memcpy(package+4*1,&Spead,4);
    HAL_UART_Transmit_DMA(&huart1,(const uint8_t *)package,channel*4+4);
    osDelay(10);
  }
  
}
/* USER CODE END Application */

