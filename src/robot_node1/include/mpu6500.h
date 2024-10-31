// sensor_imu/src/mpu6500.h
#ifndef SENSOR_IMU_MPU6500_H
#define SENSOR_IMU_MPU6500_H

#include <stdint.h>
#include <cstddef> // For size_t

// 定義加速度靈敏度
typedef enum {
    ACCEL_SENS_2G = 0x00,
    ACCEL_SENS_4G = 0x08,
    ACCEL_SENS_8G = 0x10,
    ACCEL_SENS_16G = 0x18
} accel_sensitivity_t;

// 定義陀螺儀靈敏度
typedef enum {
    GYRO_SENS_250DPS = 0x00,
    GYRO_SENS_500DPS = 0x08,
    GYRO_SENS_1000DPS = 0x10,
    GYRO_SENS_2000DPS = 0x18
} gyro_sensitivity_t;

class MPU6500 {
public:
    // 建構子，指定 I2C 檔案描述符與靈敏度參數
    explicit MPU6500(int i2c_fd, accel_sensitivity_t accel_sens = ACCEL_SENS_2G, gyro_sensitivity_t gyro_sens = GYRO_SENS_250DPS);

    // 解構子
    ~MPU6500();

    // 初始化傳感器
    bool initialize();
    
    // 讀取 IMU 數據
    bool read_imu_data(float &accel_x, float &accel_y, float &accel_z, float &gyro_x, float &gyro_y, float &gyro_z);

private:
    int i2c_fd; // I2C 檔案描述符
    accel_sensitivity_t accel_sens; // 加速度靈敏度
    gyro_sensitivity_t gyro_sens;   // 陀螺儀靈敏度

    // 設置寄存器
    bool write_register(uint8_t reg, uint8_t value);
    
    // 讀取寄存器
    bool read_register(uint8_t reg, uint8_t *buffer, int len);
};

#endif // MPU6500_H