#include "LQ_I2C_ICM42605.hpp"

/*!
 * @brief   构造函数
 * @param   i2c_path : I2C 驱动生成的设备文件路径
 * @return  无
 * @date    2025/11/24
 */
I2C_ICM42605::I2C_ICM42605(const std::string &i2c_path)
{
    LS_I2C_DEVS &i2c_dev = GetInstance(i2c_path);
}

/*!
 * @brief   获取陀螺仪 ID
 * @param   无
 * @return  返回陀螺仪 ID
 * @date    2025/11/24
 */
uint8_t I2C_ICM42605::I2C_ICM42605_Get_ID(void)
{
    uint8_t id = 0;
    ioctl(this->I2C_fd, I2C_GET_ICM42605_ID, &id);
    return id;
}

/*!
 * @brief   获取温度值
 * @param   无
 * @return  温度值
 * @date    2025/11/24
 */
float I2C_ICM42605::I2C_ICM42605_Get_Tem(void)
{
    int16_t tem = 0;
    ioctl(this->I2C_fd, I2C_GET_ICM42605_TEM, &tem);
    return (float)(tem / 100.0);
}

/*!
 * @brief    获取角速度值
 * @param    gx,gy,gz : 陀螺仪 x,y,z 轴的角速度值原始读数(带符号)
 * @return   成功返回 0，失败返回 -1
 * @date     2025/11/20
 */
int8_t I2C_ICM42605::I2C_ICM42605_Get_Ang(int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t data[3] = {0};
    if (ioctl(this->I2C_fd, I2C_GET_ICM42605_ANG, data) != 0)
        return -1;
    *gx = data[0];
    *gy = data[1];
    *gz = data[2];
    return 0;
}

/*!
 * @brief    获取加速度值
 * @param    ax,ay,az : 陀螺仪 x,y,z 轴的加速度值原始读数(带符号)
 * @return   成功返回 0，失败返回 -1
 * @date     2025/11/20
 */
int8_t I2C_ICM42605::I2C_ICM42605_Get_Acc(int16_t *ax, int16_t *ay, int16_t *az)
{
    int16_t data[3] = {0};
    if (ioctl(this->I2C_fd, I2C_GET_ICM42605_ACC, data) != 0)
        return -1;
    *ax = data[0];
    *ay = data[1];
    *az = data[2];
    return 0;
}

/*!
 * @brief    获取陀螺仪加速度值、角速度值
 * @param    ax,ay,az : 陀螺仪 x,y,z 轴的加速度值原始读数(带符号)
 * @param    gx,gy,gz : 陀螺仪 x,y,z 轴的角速度值原始读数(带符号)
 * @return   成功返回 0
 * @date     2025/11/20
 */
int8_t I2C_ICM42605::I2C_ICM42605_Get_RawData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t data[6] = {0};
    if (ioctl(this->I2C_fd, I2C_GET_ICM42605_GYRO, data) != 0)
        return -1;
    *ax = data[0];
    *ay = data[1];
    *az = data[2];
    *gx = data[3];
    *gy = data[4];
    *gz = data[5];
    return 0;
}
