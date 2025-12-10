#include <FreeRTOS_F.h>

TaskFunction_t task_breathing_led_ptr = task_breathing_led;  //呼吸灯任务参数
TaskFunction_t task_serial_communications_ptr = task_serial_communications;//上位机指令任务参数

#define task_breathing_led_stack_size 128  //呼吸灯任务栈大小 128*4=512字节
#define task_breathing_led_priority 4      //呼吸灯任务优先级
TaskHandle_t task_breathing_led_handle_ptr;  //呼吸灯任务句柄

uint8_t Rxbuff[32]={0};  //命令缓冲区
uint8_t command =0; //0代表可以接收命令,1代表不能接收命令
#define task_serial_communications_stack_size 128
#define task_serial_communications_priority 4
TaskHandle_t task_serial_communications_handle_ptr;

void FreeRTOS_Start(void){





    xTaskCreate(task_breathing_led_ptr,"task_breathing_led",task_breathing_led_stack_size,NULL,task_breathing_led_priority,&task_breathing_led_handle_ptr);  //创建任务
    xTaskCreate(task_serial_communications_ptr,"task_serial_communications",task_serial_communications_stack_size,NULL,task_serial_communications_priority,&task_serial_communications_handle_ptr);

    vTaskStartScheduler();  //启动调度器
}

void task_breathing_led(void *pointer){
    uint16_t pwm_val=0; //定义ccr的值
	int8_t cha=10; //定义每次ccr的改变量

    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2); //启动计时器PWM模式

    while (1)
  {
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,pwm_val);//配置相应的ccr的初值
	  pwm_val +=cha; //改变ccr的值
	  if(pwm_val>=1000){
		cha=-cha; //占空比为100%即灯最亮时,使每次ccr改变量为负值
	  }
	  if(pwm_val<=0){
		cha=-cha; //占空比为0%即灯最暗时,使每次ccr改变量为正值
	  }
		HAL_Delay(10); //每10ms多一点点改变一次ccr的值
  }
}

void task_serial_communications(void *pointer){

	HAL_UARTEx_ReceiveToIdle_DMA (&huart1,Rxbuff,10);

	while(1){
		if(command){
			if(strncmp((char *)Rxbuff,"ON",2) == 0){
      	HAL_UART_Transmit_DMA (&huart1,(uint8_t *)"LED ON!",strlen("LED ON!"));
      	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		}
		else if(strncmp((char *)Rxbuff,"OFF",3) == 0){
      	HAL_UART_Transmit_DMA (&huart1,(uint8_t *)"LED OFF!",strlen("LED OFF!"));
      	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
    	}
		HAL_UARTEx_ReceiveToIdle_DMA (&huart1,Rxbuff,10);
		command=0;
		}
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	command=1;
}
