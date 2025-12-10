#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

// 卡尔曼滤波初始化 (只需要调用一次)
void KalmanFilter_Init(void);

// 卡尔曼滤波更新函数 (在循环中调用)
// 输入: 原始传感器数据 (加速度g, 角速度°/s)
// 输出: 滤波后的传感器数据
void KalmanFilter_Update(float ax, float ay, float az, 
                        float gx, float gy, float gz,
                        float *ax_filt, float *ay_filt, float *az_filt,
                        float *gx_filt, float *gy_filt, float *gz_filt);

#ifdef __cplusplus
}
#endif

#endif
