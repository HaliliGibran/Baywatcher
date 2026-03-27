#ifndef SMARTCAR_ELEMENT_ZEBRA_H
#define SMARTCAR_ELEMENT_ZEBRA_H

#include <cstdint>
#include "common.h"

// 说明：
// - 斑马线调参宏统一集中在 common.h。
// - 本头文件只保留算法接口，避免业务阈值继续散落在元素实现文件附近。

// 功能: 斑马线检测（黑白交替 + 色块长度一致性）
// 类型: 图像处理函数
// 关键参数: img-二值图
bool zebra_detection(const uint8_t (&img)[IMAGE_H][IMAGE_W]);


#endif // SMARTCAR_ELEMENT_ZEBRA_H
