#pragma once

#include "LQ_I2C_DEV.hpp"

// ICM42605 相关幻数号
#define I2C_ICM42605_MAGIC    'i'                        // 自定义幻数号，用于区分不同的设备驱动
#define I2C_GET_ICM42605_ID   _IO(I2C_ICM42605_MAGIC, 1)  // 获取 MPU6050 ID
#define I2C_GET_ICM42605_TEM  _IO(I2C_ICM42605_MAGIC, 2)  // 获取 MPU6050 温度
#define I2C_GET_ICM42605_ANG  _IO(I2C_ICM42605_MAGIC, 3)  // 获取 MPU6050 角度值
#define I2C_GET_ICM42605_ACC  _IO(I2C_ICM42605_MAGIC, 4)  // 获取 MPU6050 加速度
#define I2C_GET_ICM42605_GYRO _IO(I2C_ICM42605_MAGIC, 5)  // 获取 MPU6050 角度和加速度值

// ICM42605 相关类
class I2C_ICM42605 : public LS_I2C_DEVS
{
public:
    /*!
     * @brief   构造函数
     * @param   i2c_path : I2C 驱动生成的设备文件路径
     * @return  无
     * @date    2025/3/20
     */
    explicit I2C_ICM42605(const std::string &i2c_path);

    /*!
     * @brief   获取陀螺仪 ID
     * @param   无
     * @return  返回陀螺仪 ID
     * @date    2025/11/20
     */
    uint8_t I2C_ICM42605_Get_ID(void);

    /*!
     * @brief   获取温度值
     * @param   无
     * @return  温度值
     * @date    2025/11/20
     */
    float I2C_ICM42605_Get_Tem(void);

    /*!
     * @brief    获取角速度值
     * @param    gx,gy,gz : 陀螺仪 x,y,z 轴的角速度值原始读数(带符号)
     * @return   成功返回 0，失败返回 -1
     * @date     2025/11/20
     */
    int8_t I2C_ICM42605_Get_Ang(int16_t *gx, int16_t *gy, int16_t *gz);

    /*!
     * @brief    获取加速度值
     * @param    ax,ay,az : 陀螺仪 x,y,z 轴的加速度值原始读数(带符号)
     * @return   成功返回 0，失败返回 -1
     * @date     2025/11/20
     */
    int8_t I2C_ICM42605_Get_Acc(int16_t *ax, int16_t *ay, int16_t *az);

    /*!
     * @brief    获取陀螺仪加速度值、角速度值
     * @param    ax,ay,az : 陀螺仪 x,y,z 轴的加速度值原始读数(带符号)
     * @param    gx,gy,gz : 陀螺仪 x,y,z 轴的角速度值原始读数(带符号)
     * @return   成功返回 0
     * @date     2025/11/20
     */
    int8_t I2C_ICM42605_Get_RawData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
};
