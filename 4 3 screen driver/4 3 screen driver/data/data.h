#include <stdint.h>
#ifndef DATA_H
#define DATA_H

#define MPU6050_ADDR    (0x68 << 1)  // 若AD0接地，7位地址0x68，左移1位得到8位地址0xD0
#define WHO_AM_I_REG    0x75        // WHO_AM_I寄存器地址，默认值0x68
#define PWR_MGMT_1_REG  0x6B        // 电源管理寄存器1
#define SMPLRT_DIV_REG  0x19        // 采样率分频寄存器
#define CONFIG_REG      0x1A        // 配置寄存器（含DLPF设置）
#define GYRO_CONFIG_REG 0x1B        // 陀螺仪配置寄存器
#define ACCEL_CONFIG_REG 0x1C       // 加速度计配置寄存器

#define channel 15


extern int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
extern int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
extern uint8_t DMA_ready;
extern uint8_t count;

extern float Ax,Ay,Az,Gx,Gy,Gz;
extern float ax_filt, ay_filt, az_filt, gx_filt, gy_filt, gz_filt;  // 滤波后数据
extern float roll, pitch, yaw;  // 姿态角

extern uint8_t package[channel*4+4];

#endif
