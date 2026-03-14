/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-03-16
@修改内容：
@注意事项：注意查看路径的修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#pragma once

//////////////// C++标准库 //////////////////////////////
#include <iostream>
#include <string>

//////////////// C标准库 ////////////////////////////////
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>    // 共享内存
#include <sys/sem.h>    // 信号量集

//////////////// OpenCV头文件 ///////////////////////////
#include <opencv2/opencv.hpp>

//////////////// 龙邱库文件（旧） /////////////////////////////
#include "LQ_GTIM_PWM.hpp"
#include "LQ_PWM_ENCODER.hpp"
#include "LQ_TFT18.hpp"
#include "LQ_SOFT_I2C.hpp"
#include "LQ_HW_GPIO.hpp"
#include "LQ_HW_PWM.hpp"
#include "LQ_HW_SPI.hpp"
#include "LQ_SOFT_I2C_Gyro.hpp"
#include "LQ_Uart.hpp"
#include "LQ_I2C_MPU6050.hpp"
#include "LQ_I2C_VL53.hpp"
#include "LQ_I2C_ICM42605.hpp"
#include "LQ_TCP_Client.hpp"
#include "LQ_UDP_Client.hpp"
#include "LQ_TFT18_dri.hpp"
#include "LQ_HW_ADC.hpp"
#include "LQ_ATIM_PWM.hpp"
#include "LQ_module_loader.hpp"
#include "LQ_YOLO.hpp"
#include "LQ_AB_ENCODER.hpp"

// [Legacy Model Chain Disabled] 旧 NCNN 模型链路已停用。
// 当前工程统一使用 model/cls.onnx + model/class_names.json 的 OpenCV DNN 识别链路。
// #include "lq_ncnn.hpp"

////////////// 龙邱库文件（新）//////////////////////////
#include "lq_app_inc.hpp"
#include "lq_module_load.hpp"
//////////////// WUWU库文件 /////////////////////////////
#include "WW_CAMERA.h"
#include "WW_transmission.h"
// #include "WW_TimerThread.h"

///////////////////用户文件/////////////////////////////////
#include "TimerThread.h"
#include "Encoder.h"
#include "Menu.h"
#include "ADC.h"
#include "Motor.h"
#include "Fuzzy.h"
#include "PID.h"
#include "Key.h"
#include "VOFA.h"
#include "TOF.h"
#include "TargetHandler.h"
#include "Logger.h"
#include "ESC.h"
#include "IMU.h"
#include "StateMachine.h"

//////////////////全局变量声明//////////////////////
extern BayWatcher_Encoder  encoder_sys;
extern BayWatcher_Menu&    menu;
// extern BayWatcher_ADC      ADC_sys;
extern BayWatcher_Motor    motor_sys;
extern BayWatcher_Key      key_sys;
extern BayWatcher_VOFA     VOFA;
extern Bay_PID_t           PID; 
extern Bay_IncPID_t        PID_Speed_L;
extern Bay_IncPID_t        PID_Speed_R;
extern Bay_PosPID_t        PID_Speed_F_L;   // 前进环左轮
extern Bay_PosPID_t        PID_Speed_F_R;   // 前进环右轮
extern Bay_PosPID_t        PID_Angle;    
extern Bay_PosPID_t        PID_YawSpeed;
// extern BayWatcher_TargetHandler   handler_sys;
extern Camera              camera;
extern TransmissionStreamServer server;
extern cv::Mat             img;
extern cv::Mat             binimg;
extern cv::Mat             view;
extern void BayWatcher_Start_Car(void);
extern void BayWatcher_Stop_Car(void);

/////////////////图像文件/////////////////
#include "image_process.h"
#include "image_data.h"
#include "element.h"
