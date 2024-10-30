// sensor_imu/src/mpu6500.h
#ifndef SENSOR_IMU_MPU6500_H
#define SENSOR_IMU_MPU6500_H

#include <stdint.h>
#include <cstddef> // For size_t

#define MPU6500_ADDR         0x68
#define ACCEL_XOUT_H         0x3B
#define GYRO_XOUT_H          0x43
#define PWR_MGMT_1           0x6B
#define ACCEL_CONFIG         0x1C
#define GYRO_CONFIG          0x1B

typedef enum {
    ACCEL_SENS_2G = 0x00,
    ACCEL_SENS_4G = 0x08,
    ACCEL_SENS_8G = 0x10,
    ACCEL_SENS_16G = 0x18
} accel_sensitivity_t;

typedef enum {
    GYRO_SENS_250DPS = 0x00,
    GYRO_SENS_500DPS = 0x08,
    GYRO_SENS_1000DPS = 0x10,
    GYRO_SENS_2000DPS = 0x18
} gyro_sensitivity_t;

class MPU6500 {
public:
    MPU6500(int i2c_fd);
    int init(accel_sensitivity_t accel_sens, gyro_sensitivity_t gyro_sens);
    int read_accel(int16_t &accel_x, int16_t &accel_y, int16_t &accel_z);
    int read_gyro(int16_t &gyro_x, int16_t &gyro_y, int16_t &gyro_z);

private:
    int i2c_fd;
    int write_register(uint8_t reg, uint8_t value);
    int read_register(uint8_t reg, uint8_t *buffer, size_t len);
};

#endif
