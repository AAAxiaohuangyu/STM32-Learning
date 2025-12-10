#include <data.h>

uint8_t buff[20];
uint16_t adc_buffer[1] = {0};
uint8_t flag;
uint8_t flag_Attitude = 0;  //1代表可以执行,0代表不可执行
uint8_t package[channel*4+4];
