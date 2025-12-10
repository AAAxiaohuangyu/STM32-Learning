#include "attitude.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

#define PI 3.14159265358979323846f
#define RAD_TO_DEG 57.2957795130823208768f
#define DEG_TO_RAD 0.01745329251994329577f

// 四元数结构体
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

// 姿态解算器结构体
typedef struct {
    Quaternion quat;            // 当前四元数
    float beta;                 // 梯度下降法步长
    float sample_freq;          // 采样频率 (Hz)
    float dt;                   // 采样周期 (秒)
    float gyro_bias[3];         // 陀螺仪零偏 [gx, gy, gz]
    float bias_alpha;           // 零偏估计系数
    float bias_threshold;       // 零偏更新阈值
    uint32_t calibration_count; // 校准计数
    uint8_t calibrated;         // 校准完成标志
    uint32_t update_count;      // 更新计数器
} AttitudeSolver;

// 静态变量，内部使用
static AttitudeSolver attitude;
static uint8_t initialized = 0;

// 四元数转欧拉角
static void Quaternion_ToEuler(const Quaternion *q, float *roll, float *pitch, float *yaw) {
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;
    
    // 横滚角 (x轴旋转)
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
    
    // 俯仰角 (y轴旋转)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(90.0f, sinp);
    } else {
        *pitch = asinf(sinp) * RAD_TO_DEG;
    }
    
    // 偏航角 (z轴旋转)
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}

// 改进的零偏校准函数
static void Attitude_CalibrateGyro(float gx, float gy, float gz) {
    static uint32_t stable_count = 0;
    static float last_gx = 0, last_gy = 0, last_gz = 0;
    
    if (attitude.calibration_count < 2000) {  // 增加到2000个样本
        // 检查数据稳定性
        if (fabsf(gx - last_gx) < 0.1f && fabsf(gy - last_gy) < 0.1f && fabsf(gz - last_gz) < 0.1f) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        
        // 只有稳定时才采集数据
        if (stable_count > 100) {
            attitude.gyro_bias[0] += gx;
            attitude.gyro_bias[1] += gy;
            attitude.gyro_bias[2] += gz;
            attitude.calibration_count++;
        }
        
        last_gx = gx;
        last_gy = gy;
        last_gz = gz;
        
        if (attitude.calibration_count == 2000) {
            // 计算平均值
            attitude.gyro_bias[0] /= 2000.0f;
            attitude.gyro_bias[1] /= 2000.0f;
            attitude.gyro_bias[2] /= 2000.0f;
            attitude.calibrated = 1;
            
            
        }
    }
}

// 姿态解算初始化 (只需要调用一次)
void AttitudeSolver_Init(void) {
    if (initialized) return;
    
    // 初始化四元数
    attitude.quat.q0 = 1.0f;
    attitude.quat.q1 = 0.0f;
    attitude.quat.q2 = 0.0f;
    attitude.quat.q3 = 0.0f;
    
    // 初始化参数
    attitude.beta = 1.0f;               // 梯度下降法步长
    attitude.sample_freq = 100.0f;      // 100Hz采样频率
    attitude.dt = 1.0f / attitude.sample_freq;
    
    // 零偏参数
    attitude.gyro_bias[0] = 0.0f;
    attitude.gyro_bias[1] = 0.0f;
    attitude.gyro_bias[2] = 0.0f;
    attitude.bias_alpha = 0.01f;        // 零偏估计系数
    attitude.bias_threshold = 0.5f;     // 零偏更新阈值(°/s)
    
    attitude.calibration_count = 0;
    attitude.calibrated = 0;
    attitude.update_count = 0;
    
    initialized = 1;
}

// 姿态解算更新函数 (在循环中调用)
void AttitudeSolver_Update(float ax, float ay, float az, 
                          float gx, float gy, float gz,
                          float *roll, float *pitch, float *yaw) {
    
    // 确保已经初始化
    if (!initialized) {
        AttitudeSolver_Init();
    }
    
    // 零偏校准（前2000次采样）
    if (!attitude.calibrated) {
        Attitude_CalibrateGyro(gx, gy, gz);
        *roll = *pitch = *yaw = 0.0f;
        return;  // 校准期间不更新姿态
    }
    
    // 应用初始零偏校正并转换为弧度/秒
    gx = (gx - attitude.gyro_bias[0]) * DEG_TO_RAD;
    gy = (gy - attitude.gyro_bias[1]) * DEG_TO_RAD;
    gz = (gz - attitude.gyro_bias[2]) * DEG_TO_RAD;
    
    Quaternion *q = &attitude.quat;
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;
    float halfT = attitude.dt * 0.5f;
    
    // 归一化加速度计数据
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // 估计重力和向量的方向
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    // 计算加速度计测量的误差 (叉积)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    
    // 积分误差比例增益
    ex *= attitude.beta;
    ey *= attitude.beta;
    ez *= attitude.beta;
    
    // 动态零偏估计 - 只在静止或低速时更新
    if (fabsf(gx * RAD_TO_DEG) < attitude.bias_threshold &&
        fabsf(gy * RAD_TO_DEG) < attitude.bias_threshold &&
        fabsf(gz * RAD_TO_DEG) < attitude.bias_threshold) {
        
        // 缓慢更新零偏估计
        attitude.gyro_bias[0] += attitude.bias_alpha * ex * RAD_TO_DEG;
        attitude.gyro_bias[1] += attitude.bias_alpha * ey * RAD_TO_DEG;
        attitude.gyro_bias[2] += attitude.bias_alpha * ez * RAD_TO_DEG;
        
        // 限制零偏范围，防止过度补偿
        for (int i = 0; i < 3; i++) {
            if (attitude.gyro_bias[i] > 5.0f) attitude.gyro_bias[i] = 5.0f;
            if (attitude.gyro_bias[i] < -5.0f) attitude.gyro_bias[i] = -5.0f;
        }
    }
    
    // 调整陀螺仪测量值
    gx += ex;
    gy += ey;
    gz += ez;
    
    // 四元数微分方程
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += (q0 * gz + q1 * gy - q2 * gx) * halfT;
    
    // 归一化四元数
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q->q0 = q0 * recipNorm;
    q->q1 = q1 * recipNorm;
    q->q2 = q2 * recipNorm;
    q->q3 = q3 * recipNorm;
    
    // 更新欧拉角
    Quaternion_ToEuler(q, roll, pitch, yaw);
    
    attitude.update_count++;
}

// 获取校准状态
uint8_t AttitudeSolver_IsCalibrated(void) {
    return attitude.calibrated;
}

// 获取当前零偏估计
void AttitudeSolver_GetBias(float *bias_x, float *bias_y, float *bias_z) {
    if (bias_x) *bias_x = attitude.gyro_bias[0];
    if (bias_y) *bias_y = attitude.gyro_bias[1];
    if (bias_z) *bias_z = attitude.gyro_bias[2];
}
