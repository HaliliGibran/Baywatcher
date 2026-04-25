#pragma once

#include <memory>

//////////////// OpenCV头文件 ///////////////////////////
#include <opencv2/opencv.hpp>

//////////////// 新库相机 / 图传文件 /////////////////////////////
#include "lq_camera_ex.hpp"
#include "WW_transmission.h"

///////////////////识别板用户文件/////////////////////////////////
#include "Communication.h"

//////////////////识别板全局对象声明//////////////////////
extern std::unique_ptr<lq_camera_ex> camera;
extern TransmissionStreamServer server;
extern cv::Mat                  img;
extern cv::Mat                  view;
