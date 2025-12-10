#include "kalman_filter.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

// 超强卡尔曼滤波器结构体
typedef struct {
    float Q;
    float R;
    float x;
    float P;
    float K;
    
    // 超强滤波专用参数
    float last_measurement;
    float last_5_measurements[5];
    uint8_t history_index;
    uint32_t outlier_count;
    uint32_t consecutive_outliers;
    float adaptive_R_multiplier;
    uint32_t stable_count;
} SuperKalmanFilter;

// 静态变量
static SuperKalmanFilter kf_ax, kf_ay, kf_az;
static SuperKalmanFilter kf_gx, kf_gy, kf_gz;
static uint8_t initialized = 0;

// 初始化
static void Kalman_Init(SuperKalmanFilter *kf, float Q, float R, float initial_value) {
    kf->Q = Q;
    kf->R = R;
    kf->x = initial_value;
    kf->P = 1.0f;
    kf->K = 0.0f;
    kf->last_measurement = initial_value;
    kf->history_index = 0;
    kf->outlier_count = 0;
    kf->consecutive_outliers = 0;
    kf->adaptive_R_multiplier = 1.0f;
    kf->stable_count = 0;
    
    for (int i = 0; i < 5; i++) {
        kf->last_5_measurements[i] = initial_value;
    }
}

// 计算标准差
static float Calculate_StdDev(SuperKalmanFilter *kf) {
    float mean = 0.0f;
    for (int i = 0; i < 5; i++) {
        mean += kf->last_5_measurements[i];
    }
    mean /= 5.0f;
    
    float variance = 0.0f;
    for (int i = 0; i < 5; i++) {
        float diff = kf->last_5_measurements[i] - mean;
        variance += diff * diff;
    }
    variance /= 5.0f;
    
    return sqrtf(variance);
}

// 计算中位数
static float Calculate_Median(SuperKalmanFilter *kf) {
    float temp[5];
    memcpy(temp, kf->last_5_measurements, sizeof(temp));
    
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 5; j++) {
            if (temp[i] > temp[j]) {
                float swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }
    return temp[2];
}

// 超强卡尔曼滤波更新
static float Kalman_Update(SuperKalmanFilter *kf, float measurement, int is_gyro) {
    // 更新历史数据
    kf->last_5_measurements[kf->history_index] = measurement;
    kf->history_index = (kf->history_index + 1) % 5;
    
    float diff = fabsf(measurement - kf->last_measurement);
    
    // 针对陀螺仪的超级强力滤波
    if (is_gyro) {
        float std_dev = Calculate_StdDev(kf);
        float median = Calculate_Median(kf);
        float mean_diff_from_median = fabsf(measurement - median);
        
        // 多重噪声检测阈值
        float outlier_threshold = 8.0f;     // 降低到8°/s
        float extreme_threshold = 20.0f;    // 极端噪声20°/s
        float statistical_threshold = std_dev * 5.0f;
        
        // 多重条件检测
        int is_outlier = 0;
        int is_extreme = 0;
        
        if (diff > extreme_threshold) {
            is_extreme = 1;
            is_outlier = 1;
        } else if (diff > outlier_threshold || mean_diff_from_median > statistical_threshold) {
            is_outlier = 1;
        }
        
        // 连续异常检测
        if (is_outlier) {
            kf->consecutive_outliers++;
            kf->stable_count = 0;
        } else {
            kf->consecutive_outliers = 0;
            kf->stable_count++;
        }
        
        // 超强噪声处理
        if (is_extreme) {
            // 极端噪声：完全忽略，使用中位数，只预测不更新
            kf->adaptive_R_multiplier = 50000.0f;
            kf->outlier_count++;
            kf->P = kf->P + kf->Q;
            kf->last_measurement = median;
            return kf->x;
        }
        else if (is_outlier || kf->consecutive_outliers > 0) {
            // 噪声：大幅增加噪声协方差，使用中位数平滑
            kf->adaptive_R_multiplier = 2000.0f;
            kf->outlier_count++;
            measurement = 0.8f * median + 0.2f * measurement;
        }
        else {
            // 正常数据
            if (kf->stable_count > 20) {
                kf->adaptive_R_multiplier = kf->adaptive_R_multiplier * 0.7f + 0.3f;
            }
            if (kf->adaptive_R_multiplier < 1.0f) kf->adaptive_R_multiplier = 1.0f;
        }
    } else {
        // 加速度计温和滤波
        float accel_threshold = 0.2f;
        if (diff > accel_threshold) {
            kf->adaptive_R_multiplier = 5.0f;
        } else {
            kf->adaptive_R_multiplier = kf->adaptive_R_multiplier * 0.9f + 0.1f;
            if (kf->adaptive_R_multiplier < 1.0f) kf->adaptive_R_multiplier = 1.0f;
        }
    }
    
    kf->last_measurement = measurement;
    
    // 限制范围
    if (kf->adaptive_R_multiplier > 50000.0f) kf->adaptive_R_multiplier = 50000.0f;
    if (kf->adaptive_R_multiplier < 1.0f) kf->adaptive_R_multiplier = 1.0f;
    
    // 卡尔曼滤波核心
    kf->P = kf->P + kf->Q;
    float current_R = kf->R * kf->adaptive_R_multiplier;
    kf->K = kf->P / (kf->P + current_R);
    kf->x = kf->x + kf->K * (measurement - kf->x);
    kf->P = (1 - kf->K) * kf->P;
    
    return kf->x;
}

// 三重移动平均滤波
static float Triple_MovingAverage(float new_value, int sensor_id) {
    static float history[6][15] = {{0}};
    static uint8_t indices[6] = {0};
    static uint8_t fill_count[6] = {0};
    
    history[sensor_id][indices[sensor_id]] = new_value;
    indices[sensor_id] = (indices[sensor_id] + 1) % 15;
    if (fill_count[sensor_id] < 15) fill_count[sensor_id]++;
    
    // 三重平均：短期、中期、长期
    float short_sum = 0.0f, mid_sum = 0.0f, long_sum = 0.0f;
    int short_count = 0, mid_count = 0, long_count = 0;
    
    for (int i = 0; i < fill_count[sensor_id]; i++) {
        int idx = (indices[sensor_id] - 1 - i + 15) % 15;
        float val = history[sensor_id][idx];
        
        if (i < 5) {
            short_sum += val;
            short_count++;
        }
        if (i < 10) {
            mid_sum += val;
            mid_count++;
        }
        long_sum += val;
        long_count++;
    }
    
    float short_avg = short_sum / short_count;
    float mid_avg = mid_sum / mid_count;
    float long_avg = long_sum / long_count;
    
    // 加权组合
    return 0.6f * short_avg + 0.3f * mid_avg + 0.1f * long_avg;
}

// 初始化函数 - 保持原名
void KalmanFilter_Init(void) {
    if (initialized) return;
    
    // 加速度计参数
    Kalman_Init(&kf_ax, 0.01f, 0.1f, 0.0f);
    Kalman_Init(&kf_ay, 0.01f, 0.1f, 0.0f);
    Kalman_Init(&kf_az, 0.01f, 0.1f, 1.0f);
    
    // 陀螺仪超强参数
    Kalman_Init(&kf_gx, 0.0001f, 0.005f, 0.0f);  // 极低的Q和R
    Kalman_Init(&kf_gy, 0.0001f, 0.005f, 0.0f);
    Kalman_Init(&kf_gz, 0.0001f, 0.005f, 0.0f);
    
    initialized = 1;
}

// 更新函数 - 保持原名
void KalmanFilter_Update(float ax, float ay, float az, 
                        float gx, float gy, float gz,
                        float *ax_filt, float *ay_filt, float *az_filt,
                        float *gx_filt, float *gy_filt, float *gz_filt) {
    
    if (!initialized) {
        KalmanFilter_Init();
    }
    
    // 加速度计滤波
    *ax_filt = Kalman_Update(&kf_ax, ax, 0);
    *ay_filt = Kalman_Update(&kf_ay, ay, 0);
    *az_filt = Kalman_Update(&kf_az, az, 0);
    
    // 陀螺仪超强滤波：卡尔曼 + 三重移动平均
    float gx_kalman = Kalman_Update(&kf_gx, gx, 1);
    float gy_kalman = Kalman_Update(&kf_gy, gy, 1);
    float gz_kalman = Kalman_Update(&kf_gz, gz, 1);
    
    *gx_filt = Triple_MovingAverage(gx_kalman, 0);
    *gy_filt = Triple_MovingAverage(gy_kalman, 1);
    *gz_filt = Triple_MovingAverage(gz_kalman, 2);
    
    // 输出限制
    #define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
    
    *ax_filt = CLAMP(*ax_filt, -2.0f, 2.0f);
    *ay_filt = CLAMP(*ay_filt, -2.0f, 2.0f);
    *az_filt = CLAMP(*az_filt, -2.0f, 2.0f);
    *gx_filt = CLAMP(*gx_filt, -250.0f, 250.0f);
    *gy_filt = CLAMP(*gy_filt, -250.0f, 250.0f);
    *gz_filt = CLAMP(*gz_filt, -250.0f, 250.0f);
}
