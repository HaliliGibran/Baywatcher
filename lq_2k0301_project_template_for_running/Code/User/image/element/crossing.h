#ifndef SMARTCAR_ELEMENT_CROSSING_H
#define SMARTCAR_ELEMENT_CROSSING_H

#include <cstdint>
#include "common.h"

// 功能: 十字路口状态机更新
// 类型: 图像处理函数
// 关键参数: 无（使用全局 pts_left/pts_right 等）
void crossing_update();

// 功能: 十字状态机复位（调试用）
// 类型: 图像处理函数
// 关键参数: 无
void crossing_reset();

// 功能: 十字远端线检测（成功时设置 if_find_far_line 并填充远端中线）
// 类型: 图像处理函数
// 关键参数: img-二值图
void crossing_far_line_check(const uint8_t (&img)[IMAGE_H][IMAGE_W]);

#endif
