#include "recognition_runtime.h"

#include "recognition_chain.h"
#include "stream_chain.h"
#include "main.hpp"
#include <chrono>
#include <iostream>
#include <sstream>
#include <unistd.h>

using namespace cv;

namespace {

// ==================== 识别板运行时辅助函数 ====================
// 功能: 获取识别板统一毫秒时间戳
// 类型: 局部功能函数
// 关键参数: 无
uint64_t recognition_now_ms()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

// 功能: 处理手动复位
// 类型: 局部功能函数
// 关键参数: recognition-当前识别链实例
// 说明：识别板当前只保留识别链自身状态，手动复位只需要清识别链。
void HandleManualRecognitionReset(RecognitionChain* recognition)
{
    if (recognition == nullptr)
    {
        return;
    }

    char ch = 0;
    if (read(STDIN_FILENO, &ch, 1) != 1 || (ch != 'c' && ch != 'C'))
    {
        return;
    }

    recognition->Reset();
}

// 功能: 绘制识别板空闲态画面
// 类型: 局部功能函数
// 关键参数:
// - frame_bgr-当前彩色原图
// - view-输出显示画面
// 说明：纯事件流模式下空闲态不再持续发 idle 包，只提示当前处于待触发状态。
void RenderRecognitionIdleView(const cv::Mat& frame_bgr, cv::Mat& view)
{
    view = frame_bgr.clone();

    cv::putText(view, "Recognition Board Idle", cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.65, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::putText(view, "Waiting red trigger / event edge", cv::Point(10, 52), cv::FONT_HERSHEY_SIMPLEX,
                0.58, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
}

} // namespace

// 功能: 识别板独立运行时主循环
// 类型: 图像运行时主循环
// 关键参数:
// - stream_enabled-是否启用图传
// - recognition_enabled_by_switch-是否允许启用模型识别
void RunRecognitionBoard(bool stream_enabled, bool recognition_enabled_by_switch)
{
    StreamChain stream(&server);
    RecognitionChain recognition;

    stream.Initialize(stream_enabled);
    recognition.Initialize(recognition_enabled_by_switch);

    std::cout << "[RECOG BOARD] fixed capture "
              << recognition_runtime::kRecognitionFrameWidth << "x"
              << recognition_runtime::kRecognitionFrameHeight << "@"
              << recognition_runtime::kRecognitionFrameFps
              << " color, UART1@115200" << std::endl;

    while (1)
    {
        // 1. 优先响应手动复位
        HandleManualRecognitionReset(&recognition);

        // 2. 识别板固定采彩色原图，不再承担巡线灰度处理职责
        if (!camera.capture_frame(img, true, cv::IMREAD_COLOR, false))
        {
            usleep(5 * 1000);
            continue;
        }

        const uint64_t t_ms = recognition_now_ms();
        if (recognition.IsInRecognitionMode())
        {
            // 3. 识别态直接消费 640x480 原图做 ROI 分类
            recognition.ProcessRecognitionFrame(img, t_ms, view);
        }
        else if (!recognition.TryEnterRecognition(img, t_ms, view))
        {
            // 4. 普通态只做触发判定；未命中时显示空闲画面
            RenderRecognitionIdleView(img, view);
        }

        // 5. 纯事件流模式下，只在识别链里有待发送事件时发包；
        //    同一个新事件会重复发送固定帧数，便于运行板稳定收到。
        BoardActionEvent action = BoardActionEvent::NONE;
        uint8_t seq = 0;
        if (recognition.TryGetNextTxEvent(&action, &seq))
        {
            comm.send_event(action, seq);
        }

        // 6. 发布图传画面
        stream.PublishFrame(view);
    }
}
