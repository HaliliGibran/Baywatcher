#pragma once

namespace vision_runtime {

constexpr int kLineFrameWidth = 160;
constexpr int kLineFrameHeight = 120;
// constexpr int kLineFrameFps = 100;
constexpr int kLineFrameFps = 110;

} // namespace vision_runtime

// [Vision Runtime] 图像侧顶层调度入口。
// 作用：统一管理运行板固定灰度采集、图传链和巡线主循环。
void Vision_System_Run(bool stream_enabled);
