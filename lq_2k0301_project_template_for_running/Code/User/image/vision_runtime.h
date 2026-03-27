#pragma once

// ==================== 运行板视觉运行时接口 ====================
namespace vision_runtime {

// 运行板固定灰度巡线分辨率。
// 说明：双板方案下运行板不再切换本地识别分辨率，避免主链职责再次发散。
constexpr int kLineFrameWidth = 160;
constexpr int kLineFrameHeight = 120;
constexpr int kLineFrameFps = 110;

} // namespace vision_runtime

// 功能: 运行板图像侧顶层调度入口
// 类型: 图像运行时主循环
// 关键参数: stream_enabled-是否启用图传发布
// 说明：
// - 固定执行“灰度采集 -> 二值巡线 -> 图传发布”的运行板主流程。
// - 远端识别事件只在这里接成“绕行动作 pure_angle 接管 / vehicle 保持航向”。
void Vision_System_Run(bool stream_enabled);
