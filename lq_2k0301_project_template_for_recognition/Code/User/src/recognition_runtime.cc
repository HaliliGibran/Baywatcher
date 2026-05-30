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

using steady_clock_t = std::chrono::steady_clock;
using steady_time_point_t = std::chrono::time_point<steady_clock_t>;
constexpr bool kRecognitionTextLog = (BW_RECOG_TEXT_LOG_ENABLE != 0);

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

struct PerfStageStats
{
    double total_ms = 0.0;
    double max_ms = 0.0;
    uint64_t count = 0;

    void Add(double ms, bool called = true)
    {
        if (!called)
        {
            return;
        }
        total_ms += ms;
        if (ms > max_ms)
        {
            max_ms = ms;
        }
        ++count;
    }

    double AverageMs() const
    {
        return (count > 0) ? (total_ms / static_cast<double>(count)) : 0.0;
    }

    std::string Format() const
    {
        std::ostringstream oss;
        const double avg_ms = AverageMs();
        oss << std::fixed << std::setprecision(2)
            << avg_ms << "/" << max_ms << "(" << count << ")";
        return oss.str();
    }
};

struct PerfWindowStats
{
    PerfStageStats capture;
    PerfStageStats ultra_precheck;
    PerfStageStats hsv_precheck;
    PerfStageStats extract_roi;
    PerfStageStats onnx_infer;
    PerfStageStats classify_total;
    PerfStageStats try_total;
    PerfStageStats process_recog_total;
    PerfStageStats overlay;
    PerfStageStats send_state;
    PerfStageStats publish;
    PerfStageStats loop;
    uint64_t loop_count = 0;

    void Reset()
    {
        *this = PerfWindowStats();
    }
};

static double elapsed_ms_between(const steady_time_point_t& begin, const steady_time_point_t& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

static int select_active_loop_target_fps(const RecognitionChain& recognition, uint64_t t_ms)
{
    int target_fps = BW_RECOG_LOOP_TARGET_FPS;

    if (recognition.IsInRecognitionMode())
    {
        target_fps = BW_RECOG_LOOP_FPS_RECOGNITION;
    }
    else if (recognition.IsLatchedHoldingResult())
    {
        target_fps = BW_RECOG_LOOP_FPS_LATCHED;
    }
    else if (recognition.GetCurrentVisionCode() == BoardVisionCode::NO_RESULT ||
             recognition.HasRecentRedCandidate(t_ms))
    {
        target_fps = BW_RECOG_LOOP_FPS_CANDIDATE;
    }
    else
    {
        target_fps = BW_RECOG_LOOP_FPS_NORMAL;
    }

    if (target_fps <= 0)
    {
        target_fps = BW_RECOG_LOOP_TARGET_FPS;
    }
    return target_fps;
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
    if (kRecognitionTextLog)
    {
        std::cout << "[RECOG TEST] armed by key 'c', waiting red trigger..." << std::endl;
    }
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
    BoardVisionCode last_sent_code = BoardVisionCode::INVALID;
    uint64_t last_send_ms = 0;
    const bool render_debug = stream_enabled;
    PerfWindowStats perf_window;
    steady_time_point_t perf_window_begin = steady_clock_t::now();

    stream.Initialize(stream_enabled);
    recognition.Initialize(recognition_enabled_by_switch);

    if (kRecognitionTextLog)
    {
        std::cout << "[RECOG BOARD] fixed capture "
                  << recognition_runtime::kRecognitionFrameWidth << "x"
                  << recognition_runtime::kRecognitionFrameHeight << "@"
                  << recognition_runtime::kRecognitionFrameFps
                  << " color, UART1@115200" << std::endl;
        std::cout << "[RECOG BOARD] loop_fps normal/candidate/recognition/latched="
                  << BW_RECOG_LOOP_FPS_NORMAL << "/"
                  << BW_RECOG_LOOP_FPS_CANDIDATE << "/"
                  << BW_RECOG_LOOP_FPS_RECOGNITION << "/"
                  << BW_RECOG_LOOP_FPS_LATCHED
                  << ", fallback=" << BW_RECOG_LOOP_TARGET_FPS
                  << std::endl;
    }

    while (1)
    {
        const steady_time_point_t loop_begin = steady_clock_t::now();
        double capture_ms = 0.0;
        double overlay_ms = 0.0;
        double send_state_ms = 0.0;
        double publish_ms = 0.0;
        bool send_state_called = false;
        bool publish_called = false;

        // 1. 测试模式下，按 c 手动启动一次检测与识别链
        HandleManualRecognitionStart(&recognition, &manual_test_started);

        // 2. 识别板固定采彩色原图，不再承担巡线灰度处理职责
        if (!camera)
        {
            usleep(5 * 1000);
            continue;
        }

        const steady_time_point_t capture_begin = steady_clock_t::now();
        img = camera->get_frame_raw();
        const steady_time_point_t capture_end = steady_clock_t::now();
        capture_ms = elapsed_ms_between(capture_begin, capture_end);
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
        else
        {
            if (!recognition.TryEnterRecognition(
                     img,
                     t_ms,
                     view,
                     render_debug))
            {
                // 5. 普通态触发未命中时，识别链会在 view 上保留搜索框、状态和 ROI 预览。
            }
            else
            {
                // 5. 普通态触发命中时，识别链已切入 RECOGNITION。
            }
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

        const steady_time_point_t overlay_begin = steady_clock_t::now();
        RenderVisionStateOverlay(view,
                                 recognition.GetCurrentVisionCode(),
                                 recognition.GetCurrentBlobArea());
        const steady_time_point_t overlay_end = steady_clock_t::now();
        overlay_ms = elapsed_ms_between(overlay_begin, overlay_end);

        // 6. 状态流模式下，状态变化立即发包；未变化时按心跳周期补发。
        const BoardVisionCode code = recognition.GetCurrentVisionCode();
        bool should_send_state = false;
        if (code != last_sent_code)
        {
            if (last_sent_code != BoardVisionCode::INVALID)
            {
                ++tx_seq;
            }
            last_sent_code = code;
            should_send_state = true;
        }
        else if (last_send_ms == 0 ||
                 BW_RECOG_STATE_HEARTBEAT_INTERVAL_MS <= 0 ||
                 t_ms >= last_send_ms + static_cast<uint64_t>(BW_RECOG_STATE_HEARTBEAT_INTERVAL_MS))
        {
            should_send_state = true;
        }
        if (should_send_state)
        {
            const steady_time_point_t send_begin = steady_clock_t::now();
            comm.send_state(code, tx_seq);
            const steady_time_point_t send_end = steady_clock_t::now();
            send_state_ms = elapsed_ms_between(send_begin, send_end);
            send_state_called = true;
            last_send_ms = t_ms;
        }

        // 7. 发布图传画面
        const steady_time_point_t publish_begin = steady_clock_t::now();
        stream.PublishFrame(BuildPublishView(view));
        const steady_time_point_t publish_end = steady_clock_t::now();
        publish_ms = elapsed_ms_between(publish_begin, publish_end);
        publish_called = true;

        if (manual_cycle_finished)
        {
            recognition.Reset();
            manual_test_started = false;
            if (kRecognitionTextLog)
            {
                std::cout << "[RECOG TEST] one-shot cycle finished, press c to arm again." << std::endl;
            }
        }

        // 8. 固定限频仅作用于活动态；idle 模式保持原 idle sleep 口径。
        const bool idle_mode =
            (!recognition.IsEnabled() ||
             (BW_RECOG_REQUIRE_MANUAL_START != 0 && !manual_test_started && !in_recognition_now));
        if (idle_mode)
        {
            if (BW_RECOG_IDLE_SLEEP_MS > 0)
            {
                usleep(static_cast<useconds_t>(BW_RECOG_IDLE_SLEEP_MS * 1000));
            }
        }
        else
        {
            const int target_fps = select_active_loop_target_fps(recognition, t_ms);
            if (target_fps > 0)
            {
                const double target_loop_ms = 1000.0 / static_cast<double>(target_fps);
                const double elapsed_before_sleep_ms =
                    elapsed_ms_between(loop_begin, steady_clock_t::now());
                if (elapsed_before_sleep_ms < target_loop_ms)
                {
                    const double remain_ms = target_loop_ms - elapsed_before_sleep_ms;
                    usleep(static_cast<useconds_t>(remain_ms * 1000.0));
                }
            }

            if (BW_RECOG_ACTIVE_SLEEP_MS > 0)
            {
                usleep(static_cast<useconds_t>(BW_RECOG_ACTIVE_SLEEP_MS * 1000));
            }
        }

        const steady_time_point_t loop_end = steady_clock_t::now();
        const double loop_ms = elapsed_ms_between(loop_begin, loop_end);

#if BW_RECOG_ENABLE_PERF_LOG
        const RecognitionChain::PerfSample& perf_sample = recognition.GetLastPerfSample();
        perf_window.capture.Add(capture_ms);
        perf_window.ultra_precheck.Add(perf_sample.ultra_precheck_ms, perf_sample.ultra_precheck_called);
        perf_window.hsv_precheck.Add(perf_sample.hsv_precheck_ms, perf_sample.hsv_precheck_called);
        perf_window.extract_roi.Add(perf_sample.extract_roi_ms, perf_sample.extract_roi_called);
        perf_window.onnx_infer.Add(perf_sample.onnx_infer_ms, perf_sample.onnx_infer_called);
        perf_window.classify_total.Add(perf_sample.classify_total_ms, perf_sample.classify_total_called);
        perf_window.try_total.Add(perf_sample.try_total_ms, perf_sample.try_total_called);
        perf_window.process_recog_total.Add(perf_sample.process_recog_total_ms, perf_sample.process_recog_total_called);
        perf_window.overlay.Add(overlay_ms);
        perf_window.send_state.Add(send_state_ms, send_state_called);
        perf_window.publish.Add(publish_ms, publish_called);
        perf_window.loop.Add(loop_ms);
        ++perf_window.loop_count;

        const double perf_window_ms = elapsed_ms_between(perf_window_begin, loop_end);
        if (perf_window_ms >= 1000.0)
        {
            if (perf_window.loop.count > 0)
            {
                const double effective_fps =
                    (perf_window_ms > 0.0)
                        ? (perf_window.loop_count * 1000.0 / perf_window_ms)
                        : 0.0;
                const double measured_capture_fps =
                    (perf_window.capture.AverageMs() > 0.0)
                        ? (1000.0 / perf_window.capture.AverageMs())
                        : 0.0;
                std::cout << "[PERF] state=" << VisionCodeText(code)
                          << " fps=" << std::fixed << std::setprecision(2) << effective_fps
                          << " capture_fps=" << std::fixed << std::setprecision(2) << measured_capture_fps
                          << " capture=" << perf_window.capture.Format()
                          << " ultra=" << perf_window.ultra_precheck.Format()
                          << " hsv=" << perf_window.hsv_precheck.Format()
                          << " roi=" << perf_window.extract_roi.Format()
                          << " onnx=" << perf_window.onnx_infer.Format()
                          << " cls=" << perf_window.classify_total.Format()
                          << " try=" << perf_window.try_total.Format()
                          << " recog=" << perf_window.process_recog_total.Format()
                          << " overlay=" << perf_window.overlay.Format()
                          << " send=" << perf_window.send_state.Format()
                          << " publish=" << perf_window.publish.Format()
                          << " loop=" << perf_window.loop.Format()
                          << std::endl;
            }
            perf_window.Reset();
            perf_window_begin = loop_end;
        }
#endif
    }
}
