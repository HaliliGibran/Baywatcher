#include "vision_runtime.h"

#include "stream_chain.h"
#include "main.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace cv;

namespace {

static void ConvertGrayToBgrView(const cv::Mat& gray_frame, cv::Mat& view_bgr)
{
    if (gray_frame.channels() == 1)
    {
        cv::cvtColor(gray_frame, view_bgr, cv::COLOR_GRAY2BGR);
    }
    else
    {
        view_bgr = gray_frame.clone();
    }
}

// [Vision Runtime] 在二值图/远端线视图上绘制整型边线点。
// 作用：统一把搜索得到的边线点叠加到调试图层，方便图传观察。
static inline void DrawEdgePoints(cv::Mat& view_bgr,
                                  const int32_t (&pts)[PT_MAXLEN][2],
                                  int32_t count,
                                  const cv::Scalar& color)
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
static inline void DrawMidPoints(cv::Mat& view_bgr,
                                 const float (&pts)[PT_MAXLEN][2],
                                 int32_t count,
                                 const cv::Scalar& color)
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

static void RenderLineTrackingView(const cv::Mat& frame_img,
                                   const cv::Mat& kernel,
                                   bool render_debug_view)
{
    cv::Mat line_img;
    if (frame_img.cols == vision_runtime::kLineFrameWidth &&
        frame_img.rows == vision_runtime::kLineFrameHeight)
    {
        line_img = frame_img;
    }
    else
    {
        cv::resize(frame_img,
                   line_img,
                   cv::Size(vision_runtime::kLineFrameWidth, vision_runtime::kLineFrameHeight),
                   0,
                   0,
                   cv::INTER_AREA);
    }

    cv::Mat working_gray;
    if (line_img.channels() == 1)
    {
        working_gray = line_img;
    }
    else
    {
        cv::cvtColor(line_img, working_gray, cv::COLOR_BGR2GRAY);
    }

    cv::threshold(working_gray, binimg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::morphologyEx(binimg, binimg, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(binimg, binimg, cv::MORPH_CLOSE, kernel);

    binimg.row(0).setTo(0);
    binimg.row(IMAGE_H - 1).setTo(0);
    binimg.col(0).setTo(0);
    binimg.col(IMAGE_W - 1).setTo(0);

    img_processing(binimg);

    if (!render_debug_view)
    {
        return;
    }

    if (BW_STREAM_LINE_USE_BINARY_VIEW != 0)
    {
        cv::cvtColor(binimg, view, cv::COLOR_GRAY2BGR);
    }
    else
    {
        ConvertGrayToBgrView(line_img, view);
    }

    if (if_find_far_line)
    {
        DrawEdgePoints(view, pts_far_left.pts, pts_far_left.pts_count, cv::Scalar(0, 255, 0));
        DrawEdgePoints(view, pts_far_right.pts, pts_far_right.pts_count, cv::Scalar(255, 0, 0));
    }
    else
    {
        DrawEdgePoints(view, pts_left.pts, pts_left.pts_count, cv::Scalar(0, 255, 0));
        DrawEdgePoints(view, pts_right.pts, pts_right.pts_count, cv::Scalar(255, 0, 0));
    }

    DrawMidPoints(view, midline.path, midline.path_count, cv::Scalar(0, 0, 255));
}

static void HandleManualVisionReset()
{
    char ch = 0;
    if (read(STDIN_FILENO, &ch, 1) != 1 || (ch != 'c' && ch != 'C'))
    {
        return;
    }

    track_force_reset();
    handler_sys.Stop_Action();
    image_remote_recognition_reset();
}

static bool RenderBypassBranchIfNeeded(const cv::Mat& gray_frame, bool render_debug_view)
{
    float bypass_angle = 0.0f;
    if (!handler_sys.TryGetPureAngleOverride(&bypass_angle))
    {
        return false;
    }

    track_force_reset();
    pure_angle = bypass_angle;

    if (!render_debug_view)
    {
        return true;
    }

    ConvertGrayToBgrView(gray_frame, view);
    cv::putText(view, "REMOTE BYPASS: PURE_ANGLE", cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.58, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

    std::ostringstream oss;
    oss << "pure_angle=" << std::fixed << std::setprecision(2) << bypass_angle;
    cv::putText(view, oss.str(), cv::Point(8, 52), cv::FONT_HERSHEY_SIMPLEX,
                0.58, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    return true;
}

static void ProcessTrackingBranch(const cv::Mat& gray_frame,
                                  const cv::Mat& kernel,
                                  bool render_debug_view)
{
    RenderLineTrackingView(gray_frame, kernel, render_debug_view);
}

} // namespace

void Vision_System_Run(bool stream_enabled)
{
    StreamChain stream(&server);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    const bool render_debug_view = stream_enabled;

    stream.Initialize(stream_enabled);

    std::cout << "[CAM] running board fixed capture "
              << vision_runtime::kLineFrameWidth << "x"
              << vision_runtime::kLineFrameHeight << "@"
              << vision_runtime::kLineFrameFps
              << " gray" << std::endl;

    while (1)
    {
        HandleManualVisionReset();

        if (!camera.capture_frame(img, true, cv::IMREAD_GRAYSCALE, false))
        {
            usleep(10 * 1000);
            continue;
        }

        cv::Mat gray_frame;
        if (img.channels() == 1)
        {
            gray_frame = img;
        }
        else
        {
            cv::cvtColor(img, gray_frame, cv::COLOR_BGR2GRAY);
            img = gray_frame;
        }

        handler_sys.Update_Tick();

        if (RenderBypassBranchIfNeeded(gray_frame, render_debug_view))
        {
        }
        else
        {
            ProcessTrackingBranch(gray_frame, kernel, render_debug_view);
        }

        stream.PublishFrame(view);
    }
}
