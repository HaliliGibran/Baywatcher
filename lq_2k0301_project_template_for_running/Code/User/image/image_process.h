#ifndef SMARTCAR_IMAGE_PROCESS_H
#define SMARTCAR_IMAGE_PROCESS_H

#include <cstdint>
#include "common.h"

namespace cv
{
class Mat;
}

// 功能: 图像处理总控入口（输入二值图）
// 类型: 图像处理总控函数（image_process）
// 关键参数: img-二值图像(0/255)，尺寸 IMAGE_H x IMAGE_W
void img_processing(const uint8_t (&img)[IMAGE_H][IMAGE_W]);

// 功能: 图像处理总控入口（OpenCV Mat 版本）
// 类型: 图像处理总控函数（image_process）
// 关键参数: binary-CV_8UC1 且连续的二值图
void img_processing(const cv::Mat& binary);

#endif
