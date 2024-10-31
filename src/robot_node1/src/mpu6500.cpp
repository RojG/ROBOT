// sensor_imu/src/mpu6500.cpp
#include <iostream>
#include "mpu6500.h"
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>   // For open, close

#define MPU6500_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

MPU6500::MPU6500(int i2c_fd, accel_sensitivity_t accel_sens, gyro_sensitivity_t gyro_sens)
    : i2c_fd(i2c_fd), accel_sens(accel_sens), gyro_sens(gyro_sens) {}

MPU6500::~MPU6500() {}

bool MPU6500::initialize() {
    if (ioctl(i2c_fd, I2C_SLAVE, MPU6500_ADDR) < 0) {
        std::cerr << "Failed to set I2C address" << std::endl;
        return false;
    }
    // 退出休眠模式
    if (!write_register(PWR_MGMT_1, 0x00)) {
        return false;
    }
    // 設置加速度靈敏度
    if (!write_register(ACCEL_CONFIG, accel_sens)) {
        return false;
    }
    // 設置陀螺儀靈敏度
    if (!write_register(GYRO_CONFIG, gyro_sens)) {
        return false;
    }
    return true;
}

bool MPU6500::read_imu_data(float &accel_x, float &accel_y, float &accel_z, float &gyro_x, float &gyro_y, float &gyro_z) {
    uint8_t buffer[14];

    if (!read_register(ACCEL_XOUT_H, buffer, 14)) {
        return false;
    }

    // 解析加速度數據
    float accel_scale = (accel_sens == ACCEL_SENS_2G) ? 16384.0 :
                        (accel_sens == ACCEL_SENS_4G) ? 8192.0 :
                        (accel_sens == ACCEL_SENS_8G) ? 4096.0 : 2048.0;
    accel_x = (int16_t)((buffer[0] << 8) | buffer[1]) / accel_scale;
    accel_y = (int16_t)((buffer[2] << 8) | buffer[3]) / accel_scale;
    accel_z = (int16_t)((buffer[4] << 8) | buffer[5]) / accel_scale;

    // 解析陀螺儀數據
    float gyro_scale = (gyro_sens == GYRO_SENS_250DPS) ? 131.0 :
                       (gyro_sens == GYRO_SENS_500DPS) ? 65.5 :
                       (gyro_sens == GYRO_SENS_1000DPS) ? 32.8 : 16.4;
    gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]) / gyro_scale;
    gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]) / gyro_scale;
    gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]) / gyro_scale;

    return true;
}

bool MPU6500::write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    return write(i2c_fd, buffer, 2) == 2;
}

bool MPU6500::read_register(uint8_t reg, uint8_t *buffer, int len) {
    if (write(i2c_fd, &reg, 1) != 1) {
        return false;
    }
    return read(i2c_fd, buffer, len) == len;
}