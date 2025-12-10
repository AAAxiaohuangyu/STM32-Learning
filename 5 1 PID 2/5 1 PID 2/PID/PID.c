#include <PID.h>
#include <data.h>

//归一化
float normalize_Spead(float Spead){
    return Spead/Spead_MAX;
}

float denormalize_PWM(float nor_output){
    return nor_output*PWM_MAX;
}

float normalize_Position(float Position){
    return Position/Position_MAX;
}

float denormalize_Attitude(float nor_output){
    return nor_output*4095.0f;
}

void PID(PID_t *ptr){

    float ip = 1;

    ptr -> error1 = ptr -> error0;
    ptr -> error0 = ptr -> nor_target - ptr -> nor_actual;
    ptr -> errorint += ptr -> error0;
    //积分限幅
    if(ptr -> Ki*ptr -> errorint > 0.2){
        ptr -> errorint = 0.2/ptr -> Ki;
    }
    if(ptr -> Ki*ptr -> errorint < -0.2){
        ptr -> errorint = -0.2/ptr -> Ki;
    }
    //死区
    if(ptr -> nor_actual - ptr -> nor_target <=0.004 && ptr -> nor_actual - ptr -> nor_target >=-0.004){
        ip = 0;
    }
    else{
        ip = 1;
    }
    ptr ->nor_output = ptr -> Kp*ptr -> error0+ip*ptr -> Ki*ptr -> errorint+ptr -> Kd*(ptr -> error0 - ptr -> error1);

    if(ptr -> nor_output > 1){
        ptr -> nor_output = 1;
    }
    if(ptr -> nor_output < -1){
        ptr -> nor_output = -1;
    }
}
