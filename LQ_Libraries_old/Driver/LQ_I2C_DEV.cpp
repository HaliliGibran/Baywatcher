#include "LQ_I2C_DEV.hpp"

// 初始化
int LS_I2C_DEVS::I2C_fd = -1;       // 初始化为-1，表示未初始化
std::mutex LS_I2C_DEVS::i2c_mutex;  // 互斥锁初始化(默认构造)

/*!
 * @brief   获取全局唯一I2C实例
 * @param   path : I2C 驱动生成的设备文件路径
 * @return  返回全局唯一I2C实例
 * @date    2025/11/20
 */
LS_I2C_DEVS& LS_I2C_DEVS::GetInstance(const std::string &path)
{
    // 局部静态变量
    static LS_I2C_DEVS instance;
    // 首次调用时初始化I2C总线
    if (!path.empty() && I2C_fd < 0)
    {
        instance.I2C_Init(path);
    }
    return instance;
}

/*!
 * @brief   初始化I2C总线(打开I2C设备文件)
 * @param   path : I2C 驱动生成的设备文件路径
 * @return  成功返回 0，失败返回 -1
 * @date    2025/11/20
 */
int8_t LS_I2C_DEVS::I2C_Init(const std::string &path)
{
    std::lock_guard<std::mutex> lock(i2c_mutex); // 自动加锁/解锁，防止死锁

    // 避免重复初始化
    if (I2C_fd >= 0)
    {
        printf("I2C already initialized\n");
        return 0;
    }

    // 打开I2C设备文件
    I2C_fd = open(path.c_str(), O_RDWR);
    if (I2C_fd < 0)
    {
        perror("open I2C device failed");
        return -1;
    }
    return 0;
}

/*!
 * @brief   析构函数，关闭I2C设备文件描述符
 * @param   无
 * @return  无
 * @date    2025/11/20
 */
LS_I2C_DEVS::~LS_I2C_DEVS()
{
    std::lock_guard<std::mutex> lock(i2c_mutex);
    if (I2C_fd >= 0)
    {
        close(I2C_fd);
        I2C_fd = -1;
        printf("I2C device closed\n");
    }
}
