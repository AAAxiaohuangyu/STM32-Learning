#include <attitude.h>
#include <main.h>

float normalize_Attitude(float Attitude){
    return Attitude/6.28318f;
}

float Get_Attitude(void){
    static uint8_t A_flag = 0;
    static uint32_t blind_zone_time = 0;
    float current_adc = (float)adc_buffer[0];
    
    switch(A_flag) {
        case 0: // 正常读取模式
            if(current_adc <= 60.0f || current_adc >= 3950.0f) {
                blind_zone_time = HAL_GetTick();
                A_flag = 1;
                return 0.0f;
            } else {
                return current_adc;
            }     
        case 1: // 盲区处理模式
            if(HAL_GetTick() - blind_zone_time < 1000) {
                return 0.0f;
            } else {
                A_flag = 0;
                return 0.0f;
            }
        default:
            A_flag = 0;
            return current_adc;
    }
}
