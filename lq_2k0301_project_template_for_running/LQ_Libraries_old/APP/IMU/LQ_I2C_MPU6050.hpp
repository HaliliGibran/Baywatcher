// /*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
// @编   写：龙邱科技
// @邮   箱：chiusir@163.com
// @编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
// @使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
// @相关信息参考下列地址
//     网      站：http://www.lqist.cn
//     淘 宝 店 铺：http://longqiu.taobao.com
//     程序配套视频：https://space.bilibili.com/95313236
// @软件版本：V1.0 版权所有，单位使用请先联系授权
// @参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

// @修改日期：2025-03-20
// @修改内容：
// @注意事项：
// QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
// #pragma once

// #include "LQ_I2C_DEV.hpp"

// // MPU6050 相关幻数号
// #define I2C_MPU6050_MAGIC 'm'                          // 自定义幻数号，用于区分不同的设备驱动
// #define I2C_GET_MPU6050_ID _IO(I2C_MPU6050_MAGIC, 1)   // 获取 MPU6050 ID
// #define I2C_GET_MPU6050_TEM _IO(I2C_MPU6050_MAGIC, 2)  // 获取 MPU6050 温度
// #define I2C_GET_MPU6050_ANG _IO(I2C_MPU6050_MAGIC, 3)  // 获取 MPU6050 角度值
// #define I2C_GET_MPU6050_ACC _IO(I2C_MPU6050_MAGIC, 4)  // 获取 MPU6050 加速度
// #define I2C_GET_MPU6050_GYRO _IO(I2C_MPU6050_MAGIC, 5) // 获取 MPU6050 角度和加速度值

// // MPU6050 相关类
// class I2C_MPU6050 : public LS_I2C_DEVS
// {
// public:
//     /*!
//      * @brief   构造函数
//      * @param   i2c_path : I2C 驱动生成的设备文件路径
//      * @return  无
//      * @date    2025/3/20
//      */
//     explicit I2C_MPU6050(const std::string &i2c_path);

//     /*!
//      * @brief   获取陀螺仪 ID
//      * @param   无
//      * @return  返回陀螺仪 ID
//      * @date    2025/11/20
//      */
//     uint8_t I2C_MPU6050_Get_ID(void);

//     /*!
//      * @brief   获取温度值
//      * @param   无
//      * @return  温度值
//      * @date    2025/11/20
//      */
//     float I2C_MPU6050_Get_Tem(void);

//     /*!
//      * @brief    获取角速度值
//      * @param    gx,gy,gz : 陀螺仪 x,y,z 轴的角速度值原始读数(带符号)
//      * @return   成功返回 0，失败返回 -1
//      * @date     2025/11/20
//      */
//     int8_t I2C_MPU6050_Get_Ang(int16_t *gx, int16_t *gy, int16_t *gz);

//     /*!
//      * @brief    获取加速度值
//      * @param    ax,ay,az : 陀螺仪 x,y,z 轴的加速度值原始读数(带符号)
//      * @return   成功返回 0，失败返回 -1
//      * @date     2025/11/20
//      */
//     int8_t I2C_MPU6050_Get_Acc(int16_t *ax, int16_t *ay, int16_t *az);

//     /*!
//      * @brief    获取陀螺仪加速度值、角速度值
//      * @param    ax,ay,az : 陀螺仪 x,y,z 轴的加速度值原始读数(带符号)
//      * @param    gx,gy,gz : 陀螺仪 x,y,z 轴的角速度值原始读数(带符号)
//      * @return   成功返回 0
//      * @date     2025/11/20
//      */
//     int8_t I2C_MPU6050_Get_RawData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
// };
