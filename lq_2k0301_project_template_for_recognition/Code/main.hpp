#pragma once

//////////////// OpenCV头文件 ///////////////////////////
#include <opencv2/opencv.hpp>

//////////////// WUWU库文件 /////////////////////////////
#include "WW_CAMERA.h"
#include "WW_transmission.h"

///////////////////识别板用户文件/////////////////////////////////
#include "Communication.h"

//////////////////识别板全局对象声明//////////////////////
extern Camera                   camera;
extern TransmissionStreamServer server;
extern cv::Mat                  img;
extern cv::Mat                  view;
