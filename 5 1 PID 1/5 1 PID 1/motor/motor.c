#include <motor.h>
#include <data.h>
#include <tim.h>
#include <gpio.h>



void PWM_SET(int8_t PWM){
    if(PWM >= PWM_MAX){
      flag=0;
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,PWM_MAX);
      direct();
    }
    else if(PWM <= -PWM_MAX){
      flag=1;
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,PWM_MAX);
      direct();
    }
    else if(PWM >= 0){
      flag=0;
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,PWM);
      direct();
    }
    else{
      flag=1;
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-PWM);    //设定速度
      direct();
    }
}

void direct(void){
  if(flag == 0){
    HAL_GPIO_WritePin(GPIOB,AIN1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,AIN2_Pin,GPIO_PIN_SET);
  }
  else if(flag == 1){
    HAL_GPIO_WritePin(GPIOB,AIN1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,AIN2_Pin,GPIO_PIN_RESET);
  }
}

