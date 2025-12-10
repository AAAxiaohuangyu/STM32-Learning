#include <stdint.h>
float normalize_motor(float Spead);
float denormalize_PWM(float nor_output);
float PID(float nor_Spead_Tar,float nor_Spead);

#define Kp 1.67
#define Ki 0.31
#define Kd 0
