#include <MPU6050.h>

HAL_StatusTypeDef MPU6050_Init(void) {
    uint8_t check, data;
    HAL_StatusTypeDef res;
    // 1. 读取 WHO_AM_I 寄存器，检查设备ID是否正确 (0x68)
    res = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);
    if(res != HAL_OK || check != 0x68) {
        return HAL_ERROR;  // 通信失败或ID不符
    }
    // 2. 解除休眠，将 PWR_MGMT_1 寄存器写0
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);
    HAL_Delay(10);  // 小延迟，等待芯片唤醒稳定
    // 3. 设置采样率分频器 SMPLRT_DIV (比如设置成7获得1kHz采样率)
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);
    // 4. 配置DLPF，在CONFIG寄存器中设置数字低通滤波器 (例如0x03,Accel带宽44Hz)
    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, 100);
    // 5. 配置陀螺仪满量程范围 ±250°/s (0x00) 和加速度计满量程范围 ±2g (0x00)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100);
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100);
    return HAL_OK;
}

void MPU6050_Read(void) {
    uint8_t buf[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buf, 6, 100);
    // 拼接高低字节为 16 位有符号值
    Accel_X_RAW = (int16_t)(buf[0] << 8 | buf[1]);
    Accel_Y_RAW = (int16_t)(buf[2] << 8 | buf[3]);
    Accel_Z_RAW = (int16_t)(buf[4] << 8 | buf[5]);
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, buf, 6, 100);
    Gyro_X_RAW = (int16_t)(buf[0] << 8 | buf[1]);
    Gyro_Y_RAW = (int16_t)(buf[2] << 8 | buf[3]);
    Gyro_Z_RAW = (int16_t)(buf[4] << 8 | buf[5]);
}

