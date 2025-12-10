#include <stdint.h>

typedef struct{
    float target;
    float actual;
    float output;
    float nor_target;
    float nor_actual;
    float nor_output;

    float Kp;
    float Ki;
    float Kd;

    float error0;
    float error1;
    float errorint;
}PID_t;


float normalize_Spead(float Spead);
float normalize_Position(float Position);
float denormalize_PWM(float nor_output);
float denormalize_Attitude(float nor_output);
void PID(PID_t *ptr);
