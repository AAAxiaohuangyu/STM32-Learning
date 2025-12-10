/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
	uint16_t press=0;
	uint16_t press_mode=0;      //0:初始,1:单击,2:双击,3:长按
	uint32_t last_time = 0;
	uint32_t current_time = 0;
	uint32_t press_time = 0;
	uint32_t press_time_sec = 0;
	uint32_t release_time = 0;
	uint32_t waiting_time = 0;
	uint32_t end_time = 0;
	typedef enum{
	key_vacant=0,
	key_true_press,
	key_single,
	key_release,
	key_press_shake,
	key_press_shake_sec,
	key_release_shake,
	key_record_time,
	key_end,
	}key_state_t;
	volatile key_state_t key_state = key_vacant;
	typedef enum{
	servo_free=0,
	servo_busy,
	servo_define_time,
	}servo_state_t;
	volatile servo_state_t servo_state = servo_free;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
	void key_sm(void);
	void servo_sm(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	current_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  key_sm();
	  servo_sm();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_Pin)
  {
    press = 1;
  }
}

void key_sm(void){
	current_time = HAL_GetTick();
	if (current_time - last_time < 5){
		return;
	}
    last_time = current_time;    //每5ms扫描一次
	
	switch(key_state){
	case (key_vacant):{
		if(press == 1){
			press_time = current_time;
			key_state=key_press_shake;
			press=0;
			}
		break;
		}                      //记录按键按下时间并跳转
	case (key_press_shake):{
		if(current_time - press_time >=20){
				if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)== GPIO_PIN_RESET){
					key_state=key_true_press;
				}
				else{
					key_state=key_vacant;
				}
			}
		break;
	}                            //消抖并判断是否真正按下
	case (key_true_press):{
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)== GPIO_PIN_SET){	
			release_time=current_time;
			key_state=key_release_shake;
		}
	break;
	}                           //记录按键松开时间并跳转
	case (key_release_shake):{
		if(current_time - release_time >=20){									
				if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)== GPIO_PIN_SET){
					key_state=key_release;
				}
				else{
					key_state=key_true_press;
				}
			}
		break;
	}                         //消抖并判断是否真正松开
	case (key_release):{
		if(release_time - press_time >=700){
			press_mode=3; 
			key_state=key_record_time;
		}
		else{
			key_state=key_single;
		}
		break;    
		}                        //判断是否为长按
	case(key_single):{
		if(current_time - release_time <= 350){
			if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)== GPIO_PIN_RESET){
			press_time_sec=current_time;
			key_state=key_press_shake_sec;
			} 
		}
		else {
			press_mode=1;
			key_state=key_record_time;
		}
		break;
		}                               //判断是否在双击时间内按下了第二次按键,否则为单击
	case (key_press_shake_sec):{
		if(current_time - press_time_sec >=20){
				if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)== GPIO_PIN_RESET){
					press_mode=2;                
					key_state=key_record_time;
					}                               //消抖并判断第二次是否真的按下,若按下则为双击
				else{
					key_state=key_single;
				}
			}
		break;
	}
	case (key_record_time):{
		end_time=HAL_GetTick();
		key_state = key_end;
		break;
	}
	case (key_end):{
		current_time = HAL_GetTick();
		if(current_time - end_time >=500){
			key_state = key_vacant;
		}
		break;
	}                  //提供500ms的时间供所有设置复原
	}
}
void servo_sm(void){
	switch (servo_state){
		case (servo_free):{
			if(press_mode == 1){
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,500);
		press_mode=0;
		servo_state=servo_define_time;
		}	
		if(press_mode == 2){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1500);
			press_mode=0;
			servo_state=servo_define_time;
			}
		if(press_mode == 3){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,2500);
			press_mode=0;
			servo_state=servo_define_time;
			}
	break;
		}                         //单击转到-90°,双击转到0°,长按转到90°
		case (servo_define_time):{
			waiting_time=current_time;
			servo_state=servo_busy;
			break;
		}
		case (servo_busy):{
			current_time = HAL_GetTick();
			if(current_time - waiting_time >=200){
				servo_state=servo_free;
			}
			break;                      //使舵机完成转向
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
