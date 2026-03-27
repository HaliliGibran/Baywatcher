#include "IMU.h"
#include <stdio.h>

BayWatcher_IMU::BayWatcher_IMU() 
    : raw_ax(0), raw_ay(0), raw_az(0), 
      raw_gx(0), raw_gy(0), raw_gz(0), 
      temperature(0.0f), is_initialized(false), 
      mpu6050_dev(nullptr) 
{
}

BayWatcher_IMU::~BayWatcher_IMU() {
    if (mpu6050_dev != nullptr) {
        delete mpu6050_dev;
        mpu6050_dev = nullptr;
    }
}

bool BayWatcher_IMU::init(const std::string& dev_path) {
    if (is_initialized) return true;

    // 动态实例化底层驱动对象，不修改底层代码
    mpu6050_dev = new lq_i2c_mpu6050(dev_path);

    // 尝试读取一次 ID，验证是否通信成功 (MPU6050 默认 ID 一般为 0x68)
    uint8_t id = mpu6050_dev->get_mpu6050_id();
    if (id == 0) {
        printf("[IMU] 初始化失败！无法读取设备 ID，请检查 %s 节点或接线。\n", dev_path.c_str());
        return false;
    }

    printf("[IMU] 初始化成功！读取到设备 ID: 0x%02X\n", id);
    is_initialized = true;
    return true;
}

void BayWatcher_IMU::update() {
    if (!is_initialized || mpu6050_dev == nullptr) return;

    // // 获取 6 轴数据
    // bool success = mpu6050_dev->get_mpu6050_gyro(
    //     &raw_ax, &raw_ay, &raw_az, 
    //     &raw_gx, &raw_gy, &raw_gz
    // );

    bool success = mpu6050_dev->get_mpu6050_ang(&raw_gx, &raw_gy, &raw_gz);//only gyrp
    // bool success = mpu6050_dev->get_mpu6050_acc(&raw_ax, &raw_ay, &raw_az);//only acc

    if (success) {
        // 获取当前温度
        temperature = mpu6050_dev->get_mpu6050_tem();
    } else {
        printf("[NEUQ_IMU] 数据读取失败，请检查 I2C 总线状态。\n");
    }
}