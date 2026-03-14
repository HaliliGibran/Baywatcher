#ifndef SMARTCAR_ELEMENT_H
#define SMARTCAR_ELEMENT_H

#include "image_data.h"

// 功能: 元素判定（根据左右边线/角点/曲率更新 element_type）
// 类型: 图像处理函数
// 关键参数: 无（使用全局 pts_left/pts_right 等）
void element_detect();

// 功能: 强制复位当前元素状态量（调试用）
// 类型: 全局功能函数
// 关键参数: 无
void track_force_reset();

#endif
