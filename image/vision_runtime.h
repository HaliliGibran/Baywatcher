#pragma once

namespace vision_runtime {

constexpr int kLineFrameWidth = 160;
constexpr int kLineFrameHeight = 120;
constexpr int kLineFrameFps = 60;
constexpr int kRecognitionFrameWidth = 640;
constexpr int kRecognitionFrameHeight = 480;
constexpr int kRecognitionFrameFps = 30;

} // namespace vision_runtime

// [Vision Runtime] 图像侧顶层调度入口。
// 作用：统一管理采集、识别链、图传链和巡线主循环。
void Vision_System_Run(bool stream_enabled, bool recognition_enabled_by_switch);
