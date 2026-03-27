#ifndef SMARTCAR_IMAGE_PROCESS_H
#define SMARTCAR_IMAGE_PROCESS_H

#include <cstdint>
#include "common.h"

namespace cv
{
class Mat;
}

// 说明：
// - 本头文件只暴露运行板灰度巡线主处理入口。
// - 远端识别事件不再以“强制左右选边”形式进入图像主链，
//   绕行动作和 vehicle 保持航向都在 vision_runtime 层接管 pure_angle。

// 功能: 图像处理总控入口（输入二值图）
// 类型: 图像处理总控函数（image_process）
// 关键参数: img-二值图像(0/255)，尺寸 IMAGE_H x IMAGE_W
void img_processing(const uint8_t (&img)[IMAGE_H][IMAGE_W]);

// 功能: 图像处理总控入口（OpenCV Mat 版本）
// 类型: 图像处理总控函数（image_process）
// 关键参数: binary-CV_8UC1 且连续的二值图
void img_processing(const cv::Mat& binary);

#endif
