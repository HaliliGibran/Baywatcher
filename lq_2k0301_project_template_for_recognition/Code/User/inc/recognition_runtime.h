#pragma once

#include "common.h"

// ==================== 识别板视觉运行时接口 ====================
namespace recognition_runtime {

// 识别板固定彩色采集分辨率。
// 说明：触发与识别都直接围绕同一份 640x480 原图展开。
constexpr int kRecognitionFrameWidth = 640;
constexpr int kRecognitionFrameHeight = 480;
constexpr int kRecognitionFrameFps = BW_RECOG_CAMERA_FPS;

} // namespace recognition_runtime

// 功能: 识别板独立运行时入口
// 类型: 图像运行时主循环
// 关键参数:
// - stream_enabled-是否启用图传
// - recognition_enabled_by_switch-是否允许进入模型识别链
// 说明：
// - 固定执行“彩色采集 -> 红色触发 -> 分类投票 -> 串口发包”。
// - 该入口是当前识别板唯一主流程。
void RunRecognitionBoard(bool stream_enabled, bool recognition_enabled_by_switch);
