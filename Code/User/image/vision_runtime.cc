#include "vision_runtime.h"

#include "recognition_chain.h"
#include "stream_chain.h"
#include "main.hpp"
#include <chrono>
#include <cmath>

using namespace cv;

namespace {

enum class CameraWorkMode : uint8_t
{
    LINE = 0,
    RECOGNITION,
};

// [Vision Runtime] 在二值图/远端线视图上绘制整型边线点。
// 作用：统一把搜索得到的边线点叠加到调试图层，方便图传观察。
static inline void DrawEdgePoints(cv::Mat& view_bgr, const int32_t (&pts)[PT_MAXLEN][2], int32_t count, const cv::Scalar& color)
{
    if (view_bgr.empty() || view_bgr.type() != CV_8UC3) return;
    if (count < 0) count = 0;
    if (count > PT_MAXLEN) count = PT_MAXLEN;

    const cv::Vec3b c((uint8_t)color[0], (uint8_t)color[1], (uint8_t)color[2]);
    const int radius = 1;

    for (int i = 0; i < count; ++i)
    {
        const int y = pts[i][0];
        const int x = pts[i][1];
        if ((unsigned)x >= (unsigned)view_bgr.cols || (unsigned)y >= (unsigned)view_bgr.rows) continue;

        const int y0 = (y - radius > 0) ? (y - radius) : 0;
        const int y1 = (y + radius < view_bgr.rows - 1) ? (y + radius) : (view_bgr.rows - 1);
        const int x0 = (x - radius > 0) ? (x - radius) : 0;
        const int x1 = (x + radius < view_bgr.cols - 1) ? (x + radius) : (view_bgr.cols - 1);

        for (int yy = y0; yy <= y1; ++yy)
        {
            cv::Vec3b* row = view_bgr.ptr<cv::Vec3b>(yy);
            for (int xx = x0; xx <= x1; ++xx) row[xx] = c;
        }
    }
}

// [Vision Runtime] 在二值图/远端线视图上绘制浮点中线或路径点。
// 作用：把 midline/path 等浮点轨迹转换成屏幕上的可视化标记。
static inline void DrawMidPoints(cv::Mat& view_bgr, const float (&pts)[PT_MAXLEN][2], int32_t count, const cv::Scalar& color)
{
    if (view_bgr.empty() || view_bgr.type() != CV_8UC3) return;
    if (count < 0) count = 0;
    if (count > PT_MAXLEN) count = PT_MAXLEN;

    const cv::Vec3b c((uint8_t)color[0], (uint8_t)color[1], (uint8_t)color[2]);
    const int radius = 1;

    for (int i = 0; i < count; ++i)
    {
        const int y = (int)std::lroundf(pts[i][0]);
        const int x = (int)std::lroundf(pts[i][1]);
        if ((unsigned)x >= (unsigned)view_bgr.cols || (unsigned)y >= (unsigned)view_bgr.rows) continue;

        const int y0 = (y - radius > 0) ? (y - radius) : 0;
        const int y1 = (y + radius < view_bgr.rows - 1) ? (y + radius) : (view_bgr.rows - 1);
        const int x0 = (x - radius > 0) ? (x - radius) : 0;
        const int x1 = (x + radius < view_bgr.cols - 1) ? (x + radius) : (view_bgr.cols - 1);

        for (int yy = y0; yy <= y1; ++yy)
        {
            cv::Vec3b* row = view_bgr.ptr<cv::Vec3b>(yy);
            for (int xx = x0; xx <= x1; ++xx) row[xx] = c;
        }
    }
}

// [Vision Runtime] 普通巡线渲染分支。
// 作用：把 640x480 原图缩放为巡线分支使用的 160x120，再完成二值化、形态学和路径绘制。
static void RenderLineTrackingView(const cv::Mat& frame_bgr, const cv::Mat& kernel)
{
    Mat line_img;
    // 巡线分支始终在低分辨率图上运行，减少图像处理负担；
    // 若当前相机本来就是 160x120，则直接复用当前帧，避免重复 resize。
    if (frame_bgr.cols == vision_runtime::kLineFrameWidth &&
        frame_bgr.rows == vision_runtime::kLineFrameHeight)
    {
        line_img = frame_bgr;
    }
    else
    {
        resize(frame_bgr, line_img,
               Size(vision_runtime::kLineFrameWidth, vision_runtime::kLineFrameHeight),
               0, 0, INTER_AREA);
    }
    cvtColor(line_img, binimg, COLOR_BGR2GRAY);
    threshold(binimg, binimg, 0, 255, THRESH_BINARY | THRESH_OTSU);

    morphologyEx(binimg, binimg, MORPH_OPEN, kernel);
    morphologyEx(binimg, binimg, MORPH_CLOSE, kernel);

    binimg.row(0).setTo(0);
    binimg.row(IMAGE_H - 1).setTo(0);
    binimg.col(0).setTo(0);
    binimg.col(IMAGE_W - 1).setTo(0);

    img_processing(binimg);

    // 普通巡线分支的图传底图可在“二值图”和“彩图”之间切换：
    // - 二值图更利于观察阈值、寻线和远端补线结果
    // - 彩图更利于观察现场原始画面与轨迹叠加关系
    if (BW_STREAM_LINE_USE_BINARY_VIEW != 0)
    {
        cvtColor(binimg, view, COLOR_GRAY2BGR);
    }
    else
    {
        view = line_img.clone();
    }

    if (if_find_far_line)
    {
        // 十字远端线模式下，优先显示远端补线结果和远端中线。
        DrawEdgePoints(view, pts_far_left.pts, pts_far_left.pts_count, Scalar(0, 255, 0));
        DrawEdgePoints(view, pts_far_right.pts, pts_far_right.pts_count, Scalar(255, 0, 0));
        DrawMidPoints(view, pts_far_left.mid, pts_far_left.mid_count, Scalar(0, 255, 255));
        DrawMidPoints(view, pts_far_right.mid, pts_far_right.mid_count, Scalar(255, 0, 255));
    }
    else
    {
        DrawEdgePoints(view, pts_left.pts, pts_left.pts_count, Scalar(0, 255, 0));
        DrawEdgePoints(view, pts_right.pts, pts_right.pts_count, Scalar(255, 0, 0));
        DrawMidPoints(view, midline.path, midline.path_count, Scalar(0, 0, 255));
        DrawMidPoints(view, pts_left.mid, pts_left.mid_count, Scalar(0, 255, 255));
        DrawMidPoints(view, pts_right.mid, pts_right.mid_count, Scalar(255, 0, 255));
    }
}

// [Vision Runtime] 切换相机工作分辨率。
// 作用：普通巡线用低分辨率，识别态切高分辨率；切换失败时保持当前模式继续运行。
static bool SwitchCameraMode(CameraWorkMode target_mode, CameraWorkMode* active_mode)
{
    if (active_mode == nullptr || *active_mode == target_mode)
    {
        return true;
    }

    int width = vision_runtime::kLineFrameWidth;
    int height = vision_runtime::kLineFrameHeight;
    int fps = vision_runtime::kLineFrameFps;
    if (target_mode == CameraWorkMode::RECOGNITION)
    {
        width = vision_runtime::kRecognitionFrameWidth;
        height = vision_runtime::kRecognitionFrameHeight;
        fps = vision_runtime::kRecognitionFrameFps;
    }

    if (camera.init(width, height, fps) == 0)
    {
        *active_mode = target_mode;
        std::cout << "[CAM] switch success: " << width << "x" << height << "@" << fps << std::endl;
        usleep(60 * 1000);
        return true;
    }

    std::cout << "[CAM] switch failed, keep current mode" << std::endl;
    return false;
}

// [Vision Runtime] 根据“当前相机模式 + 当前业务需求”抓帧。
// 作用：普通态优先抓低分辨率图；若摄像头仍停留在高分辨率，则用 JPEG 降采样解码减轻 CPU。
static bool CaptureFrameForMode(CameraWorkMode active_mode, bool want_recognition_frame, cv::Mat& frame)
{
    int decode_flags = cv::IMREAD_COLOR;
    if (want_recognition_frame)
    {
        decode_flags = cv::IMREAD_COLOR;
    }
    else if (active_mode == CameraWorkMode::RECOGNITION &&
             vision_runtime::kRecognitionFrameWidth / vision_runtime::kLineFrameWidth == 4 &&
             vision_runtime::kRecognitionFrameHeight / vision_runtime::kLineFrameHeight == 4)
    {
        decode_flags = cv::IMREAD_REDUCED_COLOR_4;
    }

    return camera.capture_frame(frame, true, decode_flags, false);
}

// [Vision Runtime] 处理手动复位。
// 作用：统一清图像状态、识别链状态和相机模式，避免这组动作散落在主循环里。
static void HandleManualVisionReset(RecognitionChain* recognition, CameraWorkMode* active_mode)
{
    if (recognition == nullptr || active_mode == nullptr)
    {
        return;
    }

    char ch = 0;
    if (read(STDIN_FILENO, &ch, 1) != 1 || (ch != 'c' && ch != 'C'))
    {
        return;
    }

    track_force_reset();
    recognition->Reset();
    recognition_follow_override = RecognitionFollowOverride::NONE;
    SwitchCameraMode(CameraWorkMode::LINE, active_mode);
}

// [Vision Runtime] 绕行执行中的显示分支。
// 作用：绕行期间完全旁路巡线与识别，只输出原图和提示文本。
static bool RenderBypassBranchIfNeeded()
{
    if (!handler_sys.Is_Executing())
    {
        return false;
    }

    view = img.clone();
    putText(view, "Bypass Running...", Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
    recognition_follow_override = RecognitionFollowOverride::NONE;
    return true;
}

// [Vision Runtime] 识别态处理分支。
// 作用：识别态完全交给 RecognitionChain 管理，并在退出时切回巡线分辨率。
static bool ProcessRecognitionBranchIfNeeded(RecognitionChain* recognition, uint64_t t_ms, CameraWorkMode* active_mode)
{
    if (recognition == nullptr || active_mode == nullptr || !recognition->IsInRecognitionMode())
    {
        return false;
    }

    recognition_follow_override = RecognitionFollowOverride::NONE;
    const bool was_in_recognition = recognition->IsInRecognitionMode();
    recognition->ProcessRecognitionFrame(img, t_ms, pure_angle, view);
    if (was_in_recognition && !recognition->IsInRecognitionMode())
    {
        SwitchCameraMode(CameraWorkMode::LINE, active_mode);
    }
    return true;
}

// [Vision Runtime] 检测是否应从普通态切入识别态。
// 作用：命中红色触发器后只显示触发框，本帧不再继续普通巡线。
static bool TryEnterRecognitionBranch(RecognitionChain* recognition, uint64_t t_ms, CameraWorkMode* active_mode)
{
    if (recognition == nullptr || active_mode == nullptr)
    {
        return false;
    }

    if (!recognition->TryEnterRecognition(img, t_ms, view))
    {
        return false;
    }

    recognition_follow_override = RecognitionFollowOverride::NONE;
    SwitchCameraMode(CameraWorkMode::RECOGNITION, active_mode);
    return true;
}

// [Vision Runtime] 普通态的“保持航向 / 正常巡线”分支。
// 作用：把识别链产生的短时 hold 和普通巡线链都收敛到同一个出口。
static void ProcessTrackingBranch(RecognitionChain* recognition, uint64_t t_ms, const cv::Mat& kernel)
{
    if (recognition == nullptr)
    {
        return;
    }

    float hold_yaw = 0.0f;
    if (recognition->TryRenderVehicleHold(img, t_ms, view, &hold_yaw))
    {
        track_force_reset();
        recognition_follow_override = RecognitionFollowOverride::NONE;
        pure_angle = hold_yaw;
        return;
    }

    recognition_follow_override = recognition->GetFollowOverride();
    RenderLineTrackingView(img, kernel);
}

} // namespace

void Vision_System_Run(bool stream_enabled, bool recognition_enabled_by_switch)
{
    // [Vision Runtime] 顶层调度只负责组织三条链路：
    // 1. 采集链  2. 识别链  3. 图传链。
    StreamChain stream(&server);
    RecognitionChain recognition;
    const Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    CameraWorkMode active_camera_mode = CameraWorkMode::LINE;
    recognition_follow_override = RecognitionFollowOverride::NONE;

    auto now_ms = []() -> uint64_t {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::steady_clock::now().time_since_epoch())
                                         .count());
    };

    // [Stream Chain Step 1] 启动图传链。
    stream.Initialize(stream_enabled);
    // [Recognition Chain Step 1] 启动识别链。
    recognition.Initialize(recognition_enabled_by_switch);

    std::cout << "[CAM] line mode " << vision_runtime::kLineFrameWidth << "x"
              << vision_runtime::kLineFrameHeight << "@" << vision_runtime::kLineFrameFps
              << ", recognition mode " << vision_runtime::kRecognitionFrameWidth << "x"
              << vision_runtime::kRecognitionFrameHeight << "@"
              << vision_runtime::kRecognitionFrameFps << std::endl;

    // 主循环顺序固定：
    // 1. 处理手动复位
    // 2. 采集相机帧
    // 3. 根据顶层状态决定走“绕行显示 / 识别态 / 进入识别态 / 普通巡线态”
    // 4. 发布图传帧
    while (1)
    {
        HandleManualVisionReset(&recognition, &active_camera_mode);

        const bool want_recognition_frame = recognition.IsInRecognitionMode();
        if (!CaptureFrameForMode(active_camera_mode, want_recognition_frame, img))
        {
            usleep(10 * 1000);
            continue;
        }

        const uint64_t t_ms = now_ms();

        if (RenderBypassBranchIfNeeded())
        {
        }
        else if (ProcessRecognitionBranchIfNeeded(&recognition, t_ms, &active_camera_mode))
        {
        }
        else if (TryEnterRecognitionBranch(&recognition, t_ms, &active_camera_mode))
        {
        }
        else
        {
            ProcessTrackingBranch(&recognition, t_ms, kernel);
        }

        // [Stream Chain Step 2] 把当前 view 交给图传链。
        stream.PublishFrame(view);
    }
}
