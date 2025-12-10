#include <encoder.h>
#include <data.h>
#include <tim.h>
int16_t Get_Cur_Spead(void){
    int16_t temp;
    temp = __HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3,0);   //记录此时CNT距上一次的增量
    return temp;
}
