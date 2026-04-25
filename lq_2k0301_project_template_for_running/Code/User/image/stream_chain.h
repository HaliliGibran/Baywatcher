#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
#include "main.hpp"

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
    TransmissionStreamServer* server_;
    bool enabled_;
    bool started_;
    uint32_t frame_seq_;
};
