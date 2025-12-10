#include <stdint.h>
#ifndef DATA_H
#define DATA_H

extern uint8_t buff[20];
extern float Spead;
extern float Location;
extern uint8_t flag;
extern float Spead_Tar;
extern float Location_Tar;
extern float nor_Spead_Tar;
extern float nor_Spead;
extern float nor_output;
extern int8_t output;
extern uint8_t package[];



#define motor_MAX 69.0  //电机最大转速为69弧度每秒
#define PWM_MAX 100  //PWM最大值
#define Spead_trans 0.3848356 //速度单位转换所用常数
#define channel 2

#endif
