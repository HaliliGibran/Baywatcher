#include "lq_i2c_icm42605.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @brief   ICM42605 设备驱动类有参构造函数.
 * @param   _dev_path: ICM42605 设备路径.
 * @return  none.
 * @example lq_i2c_icm42605 MyICM42605("/dev/i2c-0");
 * @note    none.
 ********************************************************************************/
lq_i2c_icm42605::lq_i2c_icm42605(const std::string _dev_path)
{
    this->i2c_dev_open(_dev_path);
}

/********************************************************************************
 * @brief   ICM42605 设备驱动类析构函数.
 * @param   none.
 * @return  none.
 * @example none.
 * @note    变量生命周期结束后自动调用.
 ********************************************************************************/
lq_i2c_icm42605::~lq_i2c_icm42605()
{
}

/********************************************************************************
 * @brief   获取 ICM42605 设备 ID.
 * @param   none.
 * @return  MPU6050 设备 ID.
 * @example uint8_t id = MyICM42605.get_icm42605_id();
 * @note    none.
 ********************************************************************************/
uint8_t lq_i2c_icm42605::get_icm42605_id()
{
    uint8_t id = 0;
    ioctl(this->fd_, I2C_GET_ICM42605_ID, &id);
    return id;
}

/********************************************************************************
 * @brief   获取 ICM42605 温度.
 * @param   none.
 * @return  MPU6050 温度.
 * @example float tem = MyICM42605.get_icm42605_tem();
 * @note    none.
 ********************************************************************************/
float lq_i2c_icm42605::get_icm42605_tem()
{
    float tem = 0.0f;
    ioctl(this->fd_, I2C_GET_ICM42605_TEM, &tem);
    return (float)(tem / 100.0); 
}

/********************************************************************************
 * @brief   获取 ICM42605 角速度值.
 * @param   gx : 角速度值指针.
 * @param   gy : 角速度值指针.
 * @param   gz : 角速度值指针.
 * @return  true: 成功, false: 失败.
 * @example int16_t gx, gy, gz;
 *          MyICM42605.get_icm42605_ang(&gx, &gy, &gz);
 * @note    none.
 ********************************************************************************/
bool lq_i2c_icm42605::get_icm42605_ang(int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t ang[3] = {0};
    if (ioctl(this->fd_, I2C_GET_ICM42605_ANG, ang) != 0)
    {
        lq_log_error("get_icm42605_ang failed");
        return false;
    }
    *gx = ang[0];
    *gy = ang[1];
    *gz = ang[2];
    return true;
}

/********************************************************************************
 * @brief   获取 ICM42605 加速度值.
 * @param   ax : 加速度值指针.
 * @param   ay : 加速度值指针.
 * @param   az : 加速度值指针.
 * @return  true: 成功, false: 失败.
 * @example uint16_t ax, ay, az;
 *          MyICM42605.get_icm42605_acc(&ax, &ay, &az);
 * @note    none.
 ********************************************************************************/
bool lq_i2c_icm42605::get_icm42605_acc(int16_t *ax, int16_t *ay, int16_t *az)
{
    int16_t acc[3] = {0};
    if (ioctl(this->fd_, I2C_GET_ICM42605_ACC, acc) != 0)
    {
        lq_log_error("get_icm42605_acc failed");
        return false;
    }
    *ax = acc[0];
    *ay = acc[1];
    *az = acc[2];
    return true;
}

/********************************************************************************
 * @brief   获取 ICM42605 角速度和加速度值.
 * @param   ax : 角速度值指针.
 * @param   ay : 角速度值指针.
 * @param   az : 角速度值指针.
 * @param   gx : 角速度值指针.
 * @param   gy : 角速度值指针.
 * @param   gz : 角速度值指针.
 * @return  true: 成功, false: 失败.
 * @example int16_t ax, ay, az, gx, gy, gz;
 *          MyICM42605.get_icm42605_gyro(&ax, &ay, &az, &gx, &gy, &gz);
 * @note    none.
 ********************************************************************************/
bool lq_i2c_icm42605::get_icm42605_gyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t data[6] = {0};
    if (ioctl(this->fd_, I2C_GET_ICM42605_GYRO, data) != 0)
    {
        lq_log_error("get_icm42605_gyro failed");
        return false;
    }
    *ax = data[0];
    *ay = data[1];
    *az = data[2];
    *gx = data[3];
    *gy = data[4];
    *gz = data[5];
    return true;
}
