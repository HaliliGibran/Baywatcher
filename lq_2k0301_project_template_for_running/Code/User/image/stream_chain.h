#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
#include "main.hpp"

// 功能: 图传链轻量封装
// 类型: 图像侧辅助类
// 说明：
// - 对 main/vision_runtime 暴露统一的“初始化 + 发布帧”接口。
// - 运行板当前只保留图传，不再把图传链与本地识别链绑在一起调度。
class StreamChain
{
public:
    // [Stream Chain Interface] 返回图传链默认开关状态。
    // 作用：让 main 侧统一通过接口读取编译期默认值。
    static bool DefaultEnabled();
    // [Stream Chain Interface] 解析图传链运行时开关。
    // 作用：统一处理 --stream / --no-stream / --stream=on|off。
    static bool ParseSwitch(int argc, char** argv, bool default_value);

    explicit StreamChain(TransmissionStreamServer* server);

    // [Stream Chain Step 1] 按开关决定是否启动图传服务器。
    // 作用：对外隐藏 start_server 与启动日志。
    bool Initialize(bool enabled_by_switch);
    // [Stream Chain Step 2] 发布当前显示帧。
    // 作用：内部做降频，避免每帧都推送图传。
    void PublishFrame(const cv::Mat& frame);

private:
    TransmissionStreamServer* server_; // 外部注入的图传服务实例，owner 仍在 main
    bool enabled_;                     // 当前运行时是否允许图传
    bool started_;                     // 图传服务是否已成功启动
    uint32_t frame_seq_;               // 当前发布帧序号，用于简单降频
};
