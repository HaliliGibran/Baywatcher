#include "LQ_I2C_VL53.hpp"

/*!
 * @brief   1.如果想要使用该文件中的MPU6050相关函数，需要先将对应模块加载到内核中
 * @brief   2.需要加载的模块：lq_i2c_dev.ko 和 lq_i2c_drv.ko
 * @brief   3.安装模块：insmod lq_i2c_dev.ko
 * @brief   4.卸载模块：rmmod lq_i2c_dev.ko
 * @brief   5.查看当前模块：lsmod
 */

 /*!
 * @brief   构造函数
 * @param   i2c_path : I2C 驱动生成的设备文件路径
 * @return  无
 * @date    2025/11/20
 */
I2C_VL53L0X::I2C_VL53L0X(const std::string &i2c_path)
{
    LS_I2C_DEVS &i2c_dev = GetInstance(i2c_path);
}

/*!
 * @brief   获取距离值
 * @param   无
 * @return  返回距离值，单位为毫米（mm）
 * @date    2025/11/20
 */
uint16_t I2C_VL53L0X::I2C_VL53L0X_Get_Dis(void)
{
    uint16_t dis = 0;
    ioctl(this->I2C_fd, I2C_GET_VL53L0X_DIS, &dis);
    return dis;
}