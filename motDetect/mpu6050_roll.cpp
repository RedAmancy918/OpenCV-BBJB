/*
调整时间步长dt和卡尔曼滤波器的其他参数。
根据测试调整这两个参数。
Editor Geo
*/
#include <iostream>
#include <pigpio.h>
#include <cmath>
#include <unistd.h>
#include "KalmanFilter.h"  

// MPU6050 I2C地址和寄存器定义
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

// 卡尔曼滤波器对象
KalmanFilter kalmanX, kalmanY;

void mpu6050_reset(int pi, int handle) {
    if (i2c_write_byte_data(pi, handle, PWR_MGMT_1, 0x80) != 0) {
        std::cerr << "Failed to reset MPU6050" << std::endl;
        return;
    }
    sleep(1); 
}

void mpu6050_init(int pi, int handle) {
    mpu6050_reset(pi, handle);
    i2c_write_byte_data(pi, handle, PWR_MGMT_1, 0x00); // 唤醒传感器
    i2c_write_byte_data(pi, handle, CONFIG, 0x01);     // 设置DLPF
    i2c_write_byte_data(pi, handle, GYRO_CONFIG, 0x00); // 设置陀螺仪灵敏度（±250度/秒）
    i2c_write_byte_data(pi, handle, ACCEL_CONFIG, 0x00); // 设置加速度计灵敏度（±2g）
}

void read_mpu6050_data(int pi, int handle, double &roll, double &pitch, double &yaw, double accel[3]) {
    char buffer[14];
    if (i2c_read_i2c_block_data(pi, handle, ACCEL_XOUT_H, buffer, 14) != 14) {
        std::cerr << "Failed to read block data from MPU6050" << std::endl;
        return;
    }

    int16_t ax = (buffer[0] << 8) | buffer[1];
    int16_t ay = (buffer[2] << 8) | buffer[3];
    int16_t az = (buffer[4] << 8) | buffer[5];
    int16_t gx = (buffer[8] << 8) | buffer[9];
    int16_t gy = (buffer[10] << 8) | buffer[11];
    int16_t gz = (buffer[12] << 8) | buffer[13];

    double dt = 0.01; // 时间步长，假设有定时调用
    pitch = kalmanX.updateEstimate(atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI, gy, dt);
    roll = kalmanY.updateEstimate(atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI, gx, dt);
    yaw = atan2(sqrt(ax * ax + ay * ay), az) * 180.0 / M_PI; // 近似值

    accel[0] = ax / 16384.0 * 9.80665;
    accel[1] = ay / 16384.0 * 9.80665;
    accel[2] = az / 16384.0 * 9.80665;
}

int main() {
    int pi = pigpio_start(NULL, NULL); // 连接到pigpio
    if (pi < 0) {
        std::cerr << "Failed to connect to pigpio daemon" << std::endl;
        return 1;
    }

    int handle = i2c_open(pi, 1, MPU6050_ADDR, 0); // 打开I2C通信
    if (handle < 0) {
        std::cerr << "Failed to open I2C handle" << std::endl;
        pigpio_stop(pi);
        return 1;
    }

    mpu6050_init(pi, handle); // 初始化MPU6050

    double roll, pitch, yaw;
    double accel[3];

    while (true) {
        read_mpu6050_data(pi, handle, roll, pitch, yaw, accel);
        std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
        std::cout << "Acceleration - X: " << accel[0] << ", Y: " << accel[1] << ", Z: " << accel[2] << std::endl;
        sleep(1);
    }

    i2c_close(pi, handle);
    pigpio_stop(pi);
    return 0;
}
