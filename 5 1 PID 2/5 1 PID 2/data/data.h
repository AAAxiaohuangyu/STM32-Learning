#include <stdint.h>
#ifndef DATA_H
#define DATA_H

extern uint8_t buff[20];
extern uint16_t adc_buffer[1];
extern uint8_t flag;
extern uint8_t flag_Attitude;
extern uint8_t package[];



#define Spead_MAX 69.0  //电机最大转速为69弧度每秒
#define Position_MAX 62.83185 //设置位置转动最大为10圈
#define PWM_MAX 100  //PWM最大值
#define Spead_trans 0.3848356 //速度单位转换所用常数
#define channel 2
#define center_attitude 3.148

#endif
