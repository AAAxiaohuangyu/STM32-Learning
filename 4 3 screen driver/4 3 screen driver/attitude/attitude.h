#ifndef ATTITUDE_H
#define ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 姿态解算初始化 (只需要调用一次)
void AttitudeSolver_Init(void);

// 姿态解算更新函数 (在循环中调用)
// 输入: 传感器数据 (加速度g, 角速度°/s)
// 输出: 欧拉角 (度)
void AttitudeSolver_Update(float ax, float ay, float az, 
                          float gx, float gy, float gz,
                          float *roll, float *pitch, float *yaw);

// 获取校准状态
uint8_t AttitudeSolver_IsCalibrated(void);

// 获取当前零偏估计
void AttitudeSolver_GetBias(float *bias_x, float *bias_y, float *bias_z);


#ifdef __cplusplus
}
#endif

#endif
