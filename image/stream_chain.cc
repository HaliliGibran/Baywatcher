#include "stream_chain.h"

#include "common.h"
#include "image_switch_utils.h"
#include <iostream>
#include <string>

bool StreamChain::DefaultEnabled()
{
    // 通过编译期宏决定默认开关，但对外暴露为统一接口。
    return (BW_ENABLE_STREAM != 0);
}

bool StreamChain::ParseSwitch(int argc, char** argv, bool default_value)
{
    bool stream_enabled = default_value;

    // 按顺序扫描命令行参数，后出现的参数可以覆盖前面的结果。
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--stream")
        {
            stream_enabled = true;
            continue;
        }
        if (arg == "--no-stream")
        {
            stream_enabled = false;
            continue;
        }

        const std::string key = "--stream=";
        if (arg.compare(0, key.size(), key) == 0)
        {
            bool parsed_value = stream_enabled;
            if (ParseImageBoolSwitchText(arg.substr(key.size()), &parsed_value))
            {
                stream_enabled = parsed_value;
            }
            continue;
        }

        if (arg == "--stream-mode" && i + 1 < argc)
        {
            bool parsed_value = stream_enabled;
            if (ParseImageBoolSwitchText(argv[i + 1], &parsed_value))
            {
                stream_enabled = parsed_value;
            }
            ++i;
            continue;
        }
    }

    return stream_enabled;
}

StreamChain::StreamChain(TransmissionStreamServer* server)
    : server_(server),
      enabled_(false),
      started_(false),
      frame_seq_(0)
{
    // 构造时不做实际启动，只保存依赖对象并清空内部状态。
}

bool StreamChain::Initialize(bool enabled_by_switch)
{
    enabled_ = enabled_by_switch;
    started_ = false;
    frame_seq_ = 0;

    if (!enabled_)
    {
        // 图传关闭时直接返回，不占用额外线程和网络资源。
        std::cout << "[BOOT] Stream disabled by switch" << std::endl;
        return false;
    }

    if (server_ == nullptr)
    {
        // 防御式保护：如果外部没有传入 server 实例，则不给图传链继续启动。
        std::cout << "[BOOT] Stream server missing." << std::endl;
        return false;
    }

    // [Stream Chain Step 1] 启动图传服务器。
    // 作用：把 HTTP/MJPEG 图传启动逻辑从 main 调度里分离出去。
    std::cout << "[BOOT] Starting TransmissionStreamServer..." << std::endl;
    started_ = (server_->start_server() == 0);
    if (started_)
    {
        std::cout << "[BOOT] Stream Started." << std::endl;
    }
    else
    {
        std::cout << "[BOOT] Stream start failed." << std::endl;
    }
    return started_;
}

void StreamChain::PublishFrame(const cv::Mat& frame)
{
    if (!started_ || server_ == nullptr || frame.empty())
    {
        // 只有“图传已启动 + server 有效 + 当前帧有效”三者同时成立才发布。
        return;
    }

    // [Stream Chain Step 2] 按 1/2 帧率推送图传。
    // 作用：降低图传占用，避免对巡线和识别链造成额外压力。
    if (((frame_seq_++) & 1u) != 0u)
    {
        return;
    }

    server_->update_frame_mat(frame);
}
