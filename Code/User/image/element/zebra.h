#ifndef SMARTCAR_ELEMENT_ZEBRA_H
#define SMARTCAR_ELEMENT_ZEBRA_H

#include "image_headfile.h"
    
// 斑马线检测参数（全局宏）
#define ZEBRA_STEP 50            // 横向搜索步长（像素）
#define ZEBRA_CONSECUTIVE 3      // 连续黑点阈值（保留）
#define NEAR_DETECT_MAX       (30) // 近距离检测长度（点数）

// 视觉侧只负责置位 zebra_stop（“停车请求”）。真正停车由控制层处理。
// 斑马线触发后会一直保持停车，直到手动 Start 解锁。
// 为避免“手动 Start 时仍在斑马线区域，立刻又触发锁停”，这里提供毫秒级 cooldown。
// cooldown 的计时由 image_process 的 system tick（steady_clock）实现。
#ifndef ZEBRA_COOLDOWN_MS
#define ZEBRA_COOLDOWN_MS 1200
#endif

// 检测到斑马线后延迟停车的时间（毫秒）。
#ifndef ZEBRA_STOP_DELAY_MS
#define ZEBRA_STOP_DELAY_MS 3000
#endif

// 历史遗留：曾用 pure_angle 的特殊值表达“停车”。现在不再使用该机制，保留宏避免外部引用编译失败。
#ifndef ZEBRA_STOP_ANGLE
#define ZEBRA_STOP_ANGLE 111.11f
#endif


// 功能: 斑马线检测（黑白交替计数）
// 类型: 图像处理函数
// 关键参数: img-二值图
bool zebra_detection(const uint8_t (&img)[IMAGE_H][IMAGE_W]);


#endif // SMARTCAR_ELEMENT_ZEBRA_H
