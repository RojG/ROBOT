// sensor_imu/src/mpu6500.cpp
#include "mpu6500.h"
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>   // For open, close

MPU6500::MPU6500(int i2c_fd) : i2c_fd(i2c_fd) {}

int MPU6500::write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    return write(i2c_fd, buffer, 2);
}

int MPU6500::read_register(uint8_t reg, uint8_t *buffer, size_t len) {
    if (write(i2c_fd, &reg, 1) != 1) return -1;
    if (read(i2c_fd, buffer, len) != (int)len) return -1;
    return 0;
}

int MPU6500::init(accel_sensitivity_t accel_sens, gyro_sensitivity_t gyro_sens) {
    if (ioctl(i2c_fd, I2C_SLAVE, MPU6500_ADDR) < 0) return -1;
    if (write_register(PWR_MGMT_1, 0x00) < 0) return -1;
    if (write_register(ACCEL_CONFIG, accel_sens) < 0) return -1;
    if (write_register(GYRO_CONFIG, gyro_sens) < 0) return -1;
    return 0;
}

int MPU6500::read_accel(int16_t &accel_x, int16_t &accel_y, int16_t &accel_z) {
    uint8_t buffer[6];
    if (read_register(ACCEL_XOUT_H, buffer, 6) < 0) return -1;
    accel_x = (buffer[0] << 8) | buffer[1];
    accel_y = (buffer[2] << 8) | buffer[3];
    accel_z = (buffer[4] << 8) | buffer[5];
    return 0;
}

int MPU6500::read_gyro(int16_t &gyro_x, int16_t &gyro_y, int16_t &gyro_z) {
    uint8_t buffer[6];
    if (read_register(GYRO_XOUT_H, buffer, 6) < 0) return -1;
    gyro_x = (buffer[0] << 8) | buffer[1];
    gyro_y = (buffer[2] << 8) | buffer[3];
    gyro_z = (buffer[4] << 8) | buffer[5];
    return 0;
}
