#include <FreeRTOS.h>
#include <task.h>
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>

void FreeRTOS_Start(void);
void task_breathing_led(void *pointer);
void task_serial_communications(void *pointer);
