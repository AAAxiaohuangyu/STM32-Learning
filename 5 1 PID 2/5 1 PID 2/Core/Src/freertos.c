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
#include <adc.h>
#include <attitude.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void command_process(uint8_t* data,uint16_t length);
void OLED(void *ptr);
void Attitude_ud(void *ptr);
void Position_ud(void *ptr);
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
osThreadId_t Attitude_udHandle;
osThreadId_t Position_udHandle;

PID_t Spead = {
  .Kp = 0,
  .Ki = 0,
  .Kd = 0,
};
PID_t Location = {
  .Kp = 0.0,
  .Ki = 0,
  .Kd = 0,
  .target = 0.0,
};
PID_t Attitude = {
  .Kp = 2.569,
  .Ki = 0.00088,
  .Kd = 5.075,
};
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
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 1);
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

  Attitude_udHandle = osThreadNew(Attitude_ud,NULL,&(osThreadAttr_t){
    .name = "Attitude_ud",
    .stack_size = 1024,
    .priority = osPriorityNormal,
  });

  Position_udHandle = osThreadNew(Position_ud,NULL,&(osThreadAttr_t){
    .name = "Position_ud",
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
  if(strcmp(command,"Location")==0){
    Location.target = value;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }

  if(htim ->Instance == TIM1){
    static int count1 = 0;
    static int count2 = 0;
    count1++;
    count2++;
    if(count1 >= 5){
    Spead.actual = (float)Get_Cur_Spead()*Spead_trans; //单位转换,速度单位为弧度每秒,通过减速比换算将磁铁的旋转转换为圆盘的旋转,将40ms的平均速度认为是瞬时速度
    Location.actual +=Spead.actual*0.04f; //累加为位置,单位为弧度
    Location.nor_actual = normalize_Position(Location.actual);
    Location.nor_target = normalize_Position(Location.target);
    count1 = 0;
    }

    if(count2 >= 5){
    Attitude.target = Location.output + center_attitude;
    Attitude.nor_target = normalize_Attitude(Attitude.target);
    Attitude.actual = (Get_Attitude() * 6.28318f) / 4095.0f;
    Attitude.nor_actual = normalize_Attitude(Attitude.actual);
    count2 = 0;
    }
    //当摆杆超过可调整范围的时候放弃PID调控
    if(Attitude.actual < 2.1 || Attitude.actual > 4.1){
    flag_Attitude = 0;  // 停止PID控制
    Attitude.output = 0;
    PWM_SET(0);
    } 
    else {
      flag_Attitude = 1;  // 允许PID控制
    }
  }
}

void OLED(void *ptr){
  char charbuff[32];
  while (1)
  {
    sprintf(charbuff,"Spead:%+06.1f",Spead.actual);
    OLED_ShowString(0,0,charbuff);
    sprintf(charbuff,"Location:%+06.1f",Location.actual);
    OLED_ShowString(0,2,charbuff);
    sprintf(charbuff,"output:%+06.1f",Attitude.output);
    OLED_ShowString(0,4,charbuff);
    sprintf(charbuff,"attitude:%+06.1f",Attitude.actual);
    OLED_ShowString(0,6,charbuff);

    //发送数据给上位机
    memcpy(package+4*0,&Attitude.actual,4);
    memcpy(package+4*1,&Spead.actual,4);
    HAL_UART_Transmit_DMA(&huart1,package,channel*4+4);
    osDelay(10);
  }
  
}

void Attitude_ud(void *ptr){
  while (1)
  {
    if(flag_Attitude){
    PID(&Attitude);
    Attitude.output = denormalize_PWM(Attitude.nor_output);
    PWM_SET((int8_t)Attitude.output);
    }
  }
}

void Position_ud(void *ptr){
  while(1){
    PID(&Location);
    Location.output = denormalize_Attitude(Location.nor_output);
    if(Location.output > 0.17453f){
      Location.output = 0.17453f;
    }
    else if(Location.output < -0.17453f){
      Location.output = -0.17453f;
    }
  }
}
/* USER CODE END Application */

