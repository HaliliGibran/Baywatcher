#ifndef SMARTCAR_ELEMENT_CIRCLE_H
#define SMARTCAR_ELEMENT_CIRCLE_H

// 功能: 环岛状态机更新（更新 follow_mode/circle_state）
// 类型: 图像处理函数
// 关键参数: 无（使用全局 circle_state/circle_direction/边线信息）
void roundabout_update();

// 功能: 环岛状态机复位（调试用）
// 类型: 图像处理函数
// 关键参数: 无
void roundabout_reset();

#endif
