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

// 功能: 清空元素运行态并回到普通赛道
// 类型: 全局功能函数
// 关键参数: reset_follow_mode-是否一并把跟线模式重置为 MIXED
// 说明：
// - 该函数只处理“当前公开的元素运行态”，不清内部投票计数。
// - 供识别覆盖、主链短路等需要临时屏蔽元素状态机的场景复用。
void track_reset_element_runtime_state(bool reset_follow_mode);

#endif
