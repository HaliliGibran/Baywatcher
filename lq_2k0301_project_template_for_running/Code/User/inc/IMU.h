#ifndef __BAYWATCHER_IMU_H
#define __BAYWATCHER_IMU_H

#include <stdint.h>
#include <string>
#include "lq_app_inc.hpp" 

class BayWatcher_IMU {
public:
    BayWatcher_IMU();
    ~BayWatcher_IMU();

    // 初始化 IMU，默认挂载在 /dev/i2c-0 节点（可根据实际情况修改）
    bool init(const std::string& dev_path = "/dev/i2c-0");
    // 周期性更新数据
    void update();
    // 原始数据存储
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;
    float   temperature;
    // 标志位
    bool is_initialized;
private:
    // 指向底层驱动类的指针
    lq_i2c_mpu6050* mpu6050_dev;
};
extern BayWatcher_IMU imu_sys;

#endif // !__BAYWATCHER_IMU_H