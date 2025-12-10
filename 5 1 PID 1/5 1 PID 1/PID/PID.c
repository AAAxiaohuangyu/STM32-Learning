#include <PID.h>
#include <data.h>

//归一化
float normalize_motor(float Spead){
    return Spead/motor_MAX;
}

float denormalize_PWM(float nor_output){
    return nor_output*PWM_MAX;
}

float PID(float nor_Spead_Tar,float nor_Spead){
    static float error0 = 0;
    static float error1 = 0;
    static float error2 = 0;
    static float nor_output = 0;

    error2 = error1;
    error1 = error0;
    error0 = nor_Spead_Tar - nor_Spead;
    nor_output += Kp*(error0 - error1)+Ki*error0+Kd*(error0 - 2*error1 + error2);
    return nor_output;

}
