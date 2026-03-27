// /*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
// @��   д������Ƽ�
// @��   �䣺chiusir@163.com
// @����IDE��Linux ������VSCode_1.93 �����ϰ汾��Cmake_3.16 �����ϰ汾
// @ʹ��ƽ̨����о2K0300�þ��ɺͱ����������ܿƼ���о�þ�����չ��
// @�����Ϣ�ο����е�ַ
//     ��      վ��http://www.lqist.cn
//     �� �� �� �̣�http://longqiu.taobao.com
//     ����������Ƶ��https://space.bilibili.com/95313236
// @�����汾��V1.0 ��Ȩ���У���λʹ��������ϵ��Ȩ
// @�ο���Ŀ���ӣ�https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

// @�޸����ڣ�2025-03-25
// @�޸����ݣ�
// @ע�����
// QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
// #include "LQ_I2C_MPU6050.hpp"

// /*!
//  * @brief   1.�����Ҫʹ�ø��ļ��е�MPU6050��غ�������Ҫ�Ƚ���Ӧģ����ص��ں���
//  * @brief   2.��Ҫ���ص�ģ�飺lq_i2c_dev.ko �� lq_i2c_drv.ko
//  * @brief   3.��װģ�飺insmod lq_i2c_dev.ko
//  * @brief   4.ж��ģ�飺rmmod lq_i2c_dev.ko
//  * @brief   5.�鿴��ǰģ�飺lsmod
//  */

// /*!
//  * @brief   ���캯��
//  * @param   i2c_path : I2C �������ɵ��豸�ļ�·��
//  * @return  ��
//  * @date    2025/3/20
//  */
// I2C_MPU6050::I2C_MPU6050(const std::string &i2c_path)
// {
//     LS_I2C_DEVS &i2c_dev = GetInstance(i2c_path);
// }

// /*!
//  * @brief   ��ȡ������ ID
//  * @param   ��
//  * @return  ���������� ID
//  * @date    2025/11/20
//  */
// uint8_t I2C_MPU6050::I2C_MPU6050_Get_ID(void)
// {
//     uint8_t id = 0;
//     ioctl(this->I2C_fd, I2C_GET_MPU6050_ID, &id);
//     return id;
// }

// /*!
//  * @brief   ��ȡ�¶�ֵ
//  * @param   ��
//  * @return  �¶�ֵ
//  * @date    2025/11/20
//  */
// float I2C_MPU6050::I2C_MPU6050_Get_Tem(void)
// {
//     int16_t tem = 0;
//     ioctl(this->I2C_fd, I2C_GET_MPU6050_TEM, &tem);
//     return (float)(tem / 100.0);
// }

// /*!
//  * @brief    ��ȡ���ٶ�ֵ
//  * @param    gx,gy,gz : ������ x,y,z ��Ľ��ٶ�ֵԭʼ����(������)
//  * @return   �ɹ����� 0��ʧ�ܷ��� -1
//  * @date     2025/11/20
//  */
// int8_t I2C_MPU6050::I2C_MPU6050_Get_Ang(int16_t *gx, int16_t *gy, int16_t *gz)
// {
//     int16_t data[3] = {0};
//     if (ioctl(this->I2C_fd, I2C_GET_MPU6050_ANG, data) != 0)
//         return -1;
//     *gx = data[0];
//     *gy = data[1];
//     *gz = data[2];
//     return 0;
// }

// /*!
//  * @brief    ��ȡ���ٶ�ֵ
//  * @param    ax,ay,az : ������ x,y,z ��ļ��ٶ�ֵԭʼ����(������)
//  * @return   �ɹ����� 0��ʧ�ܷ��� -1
//  * @date     2025/11/20
//  */
// int8_t I2C_MPU6050::I2C_MPU6050_Get_Acc(int16_t *ax, int16_t *ay, int16_t *az)
// {
//     int16_t data[3] = {0};
//     if (ioctl(this->I2C_fd, I2C_GET_MPU6050_ACC, data) != 0)
//         return -1;
//     *ax = data[0];
//     *ay = data[1];
//     *az = data[2];
//     return 0;
// }

// /*!
//  * @brief    ��ȡ�����Ǽ��ٶ�ֵ�����ٶ�ֵ
//  * @param    ax,ay,az : ������ x,y,z ��ļ��ٶ�ֵԭʼ����(������)
//  * @param    gx,gy,gz : ������ x,y,z ��Ľ��ٶ�ֵԭʼ����(������)
//  * @return   �ɹ����� 0
//  * @date     2025/11/20
//  */
// int8_t I2C_MPU6050::I2C_MPU6050_Get_RawData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
// {
//     int16_t data[6] = {0};
//     if (ioctl(this->I2C_fd, I2C_GET_MPU6050_GYRO, data) != 0)
//         return -1;
//     *ax = data[0];
//     *ay = data[1];
//     *az = data[2];
//     *gx = data[3];
//     *gy = data[4];
//     *gz = data[5];
//     return 0;
// }
