#include "recognition_runtime.h"

#include "recognition_chain.h"
#include "stream_chain.h"
#include "main.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unistd.h>

using namespace cv;

namespace {

const char* VisionCodeText(BoardVisionCode code)
{
    switch (code)
    {
    case BoardVisionCode::VEHICLE: return "v";
    case BoardVisionCode::WEAPON: return "w";
    case BoardVisionCode::SUPPLY: return "s";
    case BoardVisionCode::BRICK: return "b";
    case BoardVisionCode::NO_RESULT: return "u";
    case BoardVisionCode::UNKNOWN: return "n";
    default: return "-";
    }
}

cv::Mat BuildPublishView(const cv::Mat& source_view)
{
    if (source_view.empty())
    {
        return source_view;
    }

    const int crop_max_y = BW_STREAM_CROP_MAX_Y;
    if (crop_max_y <= 0 || crop_max_y >= source_view.rows)
    {
        return source_view;
    }

    return source_view.rowRange(0, crop_max_y);
}

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

// 功能: 手动启动一次红块检测与识别链
// 类型: 局部功能函数
// 关键参数:
// - recognition-当前识别链实例
// - manual_started-当前是否已经进入测试启动态
// 说明：
// - 测试模式下，按一次 c 才开始真正跑“红块检测 -> ROI -> 分类 -> 发事件”。
// - 每次按 c 都会先复位识别链，再进入新一轮测试。
void HandleManualRecognitionStart(RecognitionChain* recognition, bool* manual_started)
{
    if (recognition == nullptr || manual_started == nullptr)
    {
        return;
    }

    char ch = 0;
    if (read(STDIN_FILENO, &ch, 1) != 1 || (ch != 'c' && ch != 'C'))
    {
        return;
    }

    recognition->Reset();
    *manual_started = true;
    std::cout << "[RECOG TEST] armed by key 'c', waiting red trigger..." << std::endl;
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

// 功能: 绘制“等待按 c 启动测试”的提示画面
// 类型: 局部功能函数
// 关键参数:
// - frame_bgr-当前彩色原图
// - view-输出显示画面
void RenderManualArmView(const cv::Mat& frame_bgr, cv::Mat& view)
{
    view = frame_bgr.clone();

    cv::putText(view, "Recognition Test Idle", cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.65, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::putText(view, "Press c to start red-trigger detection", cv::Point(10, 52), cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
}

// 功能: 绘制识别链关闭提示
// 类型: 局部功能函数
// 关键参数:
// - frame_bgr-当前彩色原图
// - view-输出显示画面
void RenderRecognitionDisabledView(const cv::Mat& frame_bgr, cv::Mat& view)
{
    view = frame_bgr.clone();

    cv::putText(view, "Recognition Disabled", cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.65, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    cv::putText(view, "Use --recognition or set BW_ENABLE_RECOGNITION=1", cv::Point(10, 52),
                cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
}

void RenderVisionStateOverlay(cv::Mat& view, BoardVisionCode code, double blob_area)
{
    if (view.empty())
    {
        return;
    }

    int overlay_bottom = view.rows;
    if (BW_STREAM_CROP_MAX_Y > 0 && BW_STREAM_CROP_MAX_Y < overlay_bottom)
    {
        overlay_bottom = BW_STREAM_CROP_MAX_Y;
    }
    const int state_text_y = std::max(28, overlay_bottom - 40);
    const int area_text_y = std::max(56, overlay_bottom - 14);

    std::ostringstream oss;
    oss << "state: " << VisionCodeText(code);
    cv::putText(view, oss.str(), cv::Point(10, state_text_y), cv::FONT_HERSHEY_SIMPLEX,
                0.62, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    std::ostringstream area_info;
    area_info << "red_area: " << std::fixed << std::setprecision(1) << blob_area;
    cv::putText(view, area_info.str(), cv::Point(10, area_text_y), cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
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
    bool manual_test_started = (BW_RECOG_REQUIRE_MANUAL_START == 0);
    bool prev_in_recognition = false;
    bool manual_cycle_finished = false;
    uint8_t tx_seq = 0;
    const bool render_debug = stream_enabled;

    stream.Initialize(stream_enabled);
    recognition.Initialize(recognition_enabled_by_switch);

    std::cout << "[RECOG BOARD] fixed capture "
              << recognition_runtime::kRecognitionFrameWidth << "x"
              << recognition_runtime::kRecognitionFrameHeight << "@"
              << recognition_runtime::kRecognitionFrameFps
              << " color, UART1@115200" << std::endl;

    while (1)
    {
        // 1. 测试模式下，按 c 手动启动一次检测与识别链
        HandleManualRecognitionStart(&recognition, &manual_test_started);

        // 2. 识别板固定采彩色原图，不再承担巡线灰度处理职责
        if (!camera)
        {
            usleep(5 * 1000);
            continue;
        }

        img = camera->get_frame_raw();
        if (img.empty())
        {
            usleep(5 * 1000);
            continue;
        }

        const uint64_t t_ms = recognition_now_ms();
        if (!recognition.IsEnabled())
        {
            if (render_debug)
            {
                RenderRecognitionDisabledView(img, view);
            }
            else
            {
                view.release();
            }
        }
        else if (recognition.IsInRecognitionMode())
        {
            // 3. 识别态直接消费 640x480 原图做 ROI 分类
            recognition.ProcessRecognitionFrame(img, t_ms, view, render_debug);
        }
        else if (BW_RECOG_REQUIRE_MANUAL_START != 0 && !manual_test_started)
        {
            // 4. 测试门控开启且尚未按 c 时，只维持待机画面，不开始红块检测。
            if (render_debug)
            {
                RenderManualArmView(img, view);
            }
            else
            {
                view.release();
            }
        }
        else if (!recognition.TryEnterRecognition(
                     img,
                     t_ms,
                     view,
                     render_debug))
        {
            // 5. 普通态触发未命中时，识别链会在 view 上保留搜索框、状态和 ROI 预览。
        }

        const bool in_recognition_now = recognition.IsInRecognitionMode();
        manual_cycle_finished = false;
        if (BW_RECOG_REQUIRE_MANUAL_START != 0 &&
            manual_test_started &&
            prev_in_recognition &&
            !in_recognition_now)
        {
            manual_cycle_finished = true;
        }
        prev_in_recognition = in_recognition_now;

        RenderVisionStateOverlay(view,
                                 recognition.GetCurrentVisionCode(),
                                 recognition.GetCurrentBlobArea());

        // 6. 状态流模式下，每帧发送当前状态码；
        //    运行板只对 w/s/v 的状态上升沿触发一次原动作。
        comm.send_state(recognition.GetCurrentVisionCode(), tx_seq++);

        // 7. 发布图传画面
        stream.PublishFrame(BuildPublishView(view));

        if (manual_cycle_finished)
        {
            recognition.Reset();
            manual_test_started = false;
            std::cout << "[RECOG TEST] one-shot cycle finished, press c to arm again." << std::endl;
        }

        // 8. 轻微让步 CPU：测试待机态休眠更久，活动态保持较短睡眠。
        const int sleep_ms =
            (!recognition.IsEnabled() ||
             (BW_RECOG_REQUIRE_MANUAL_START != 0 && !manual_test_started && !in_recognition_now))
                ? BW_RECOG_IDLE_SLEEP_MS
                : BW_RECOG_ACTIVE_SLEEP_MS;
        if (sleep_ms > 0)
        {
            usleep(static_cast<useconds_t>(sleep_ms * 1000));
        }
    }
}
