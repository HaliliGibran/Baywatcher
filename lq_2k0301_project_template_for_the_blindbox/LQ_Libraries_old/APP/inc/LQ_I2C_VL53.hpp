#pragma once

#include "LQ_I2C_DEV.hpp"

// VL53L0X 相关幻数号
#define I2C_VL53L0X_MAGIC 'v'                         // 自定义幻数号，用于区分不同的设备驱动
#define I2C_GET_VL53L0X_DIS _IO(I2C_VL53L0X_MAGIC, 1) // 获取 VL53L0X 距离值

// VL53L0X 相关类
class I2C_VL53L0X : public LS_I2C_DEVS
{
public:
    /*!
     * @brief   构造函数
     * @param   i2c_path : I2C 驱动生成的设备文件路径
     * @return  无
     * @date    2025/11/20
     */
    explicit I2C_VL53L0X(const std::string &i2c_path);

    /*!
     * @brief   获取距离值
     * @param   无
     * @return  返回距离值，单位为毫米（mm）
     * @date    2025/11/20
     */
    uint16_t I2C_VL53L0X_Get_Dis(void);
};
