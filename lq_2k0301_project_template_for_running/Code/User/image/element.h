#ifndef SMARTCAR_ELEMENT_H
#define SMARTCAR_ELEMENT_H

#include "image_data.h"

// 功能: 元素判定（根据左右边线/角点/曲率更新 element_type）
// 类型: 图像处理函数
// 关键参数: 无（使用全局 pts_left/pts_right 等）
void element_detect();

// 功能: 查询当前帧环岛单角点候选及方向，不推进投票或状态机
// 类型: 图像处理查询函数
// 关键参数: out_direction-命中时输出左/右环岛方向
// 说明：与 element_detect 复用完全相同的候选判定，供识别门控抢在远端接管前生效。
bool track_get_circle_candidate_direction(CircleDirection* out_direction);

// 功能: 查询 h 状态切入的环岛/十字首状态是否仍在二次确认
// 说明：确认期间 CIRCLE_BEGIN/CROSSING_IN 只保持首状态，不推进内部迁移计数。
bool track_sign_loss_element_entry_confirmation_pending();

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
