#include <data.h>

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
uint8_t DMA_ready = 1;
uint8_t count = 0;

float Ax,Ay,Az,Gx,Gy,Gz;
float ax_filt, ay_filt, az_filt, gx_filt, gy_filt, gz_filt;  // 滤波后数据
float roll, pitch, yaw;  // 姿态角

uint8_t package[channel*4+4];
