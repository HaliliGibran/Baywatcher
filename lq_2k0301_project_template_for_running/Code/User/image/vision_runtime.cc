#include "vision_runtime.h"

#include "stream_chain.h"
#include "main.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace cv;

namespace {

// 功能: 把灰度巡线底图统一转成 BGR 显示图
// 类型: 局部功能函数
// 关键参数:
// - gray_frame: 当前巡线输入帧（允许单通道或三通道）
// - view_bgr: 输出显示底图
// 说明：图传和提示文字统一叠在 BGR 图上，避免各分支重复判断通道数。
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

// 功能: 获取当前稳态时钟毫秒值
// 类型: 局部功能函数
// 关键参数: 无
static uint64_t now_ms()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
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

// 功能: 普通巡线分支的图像处理与图传底图构建
// 类型: 局部功能函数
// 关键参数:
// - frame_img: 当前巡线输入帧
// - kernel: 开闭运算核
// - render_debug_view: 当前是否需要构建图传显示画面
// 说明：
// - 普通巡线分支固定在 160x120 图上完成二值化、形态学与路径绘制。
// - 图传底图可切换为二值图或原始低分辨率底图，并叠加边线/路径点用于调试。
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

    // 图传关闭时，普通巡线分支不再构建显示图层，减少额外的色彩转换与画点开销。
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
        // 十字远端线模式下，优先显示远端补线结果。
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

// 功能: 处理键盘手动复位
// 类型: 局部功能函数
// 关键参数: 无
// 说明：手动复位不仅要清赛道状态，还要同步清远端动作缓存，避免复位后继续吃旧动作。
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

// 功能: 绕行动作执行期间的显示分支
// 类型: 局部功能函数
// 关键参数:
// - gray_frame: 当前巡线灰度帧
// - render_debug_view: 当前是否需要构建图传显示画面
// 说明：绕行执行态完全旁路巡线覆盖逻辑，只保留状态提示，避免图像链与动作链互相抢 owner。
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

// 功能: 处理远端 vehicle 对应的“保持航向”分支
// 类型: 局部功能函数
// 关键参数:
// - gray_frame: 当前巡线灰度帧
// - t_ms: 当前时间戳（毫秒）
// - render_debug_view: 当前是否需要构建图传显示画面
// 说明：进入该分支时，本帧不再跑普通巡线，让 pure_angle 直接被锁存航向角接管。
static bool ProcessRemoteVehicleHoldBranchIfNeeded(const cv::Mat& gray_frame,
                                                   uint64_t t_ms,
                                                   bool render_debug_view)
{
    float hold_yaw = 0.0f;
    if (!image_remote_recognition_try_get_hold_yaw(t_ms, &hold_yaw))
    {
        return false;
    }

    track_force_reset();
    pure_angle = hold_yaw;

    if (!render_debug_view)
    {
        return true;
    }

    ConvertGrayToBgrView(gray_frame, view);
    cv::putText(view, "REMOTE VEHICLE: HOLD YAW", cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.60, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    std::ostringstream oss;
    oss << "yaw=" << std::fixed << std::setprecision(2) << hold_yaw;
    cv::putText(view, oss.str(), cv::Point(10, 52), cv::FONT_HERSHEY_SIMPLEX,
                0.60, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    return true;
}

// 功能: 普通巡线分支
// 类型: 局部功能函数
// 关键参数:
// - gray_frame: 当前巡线灰度帧
// - kernel: 开闭运算核
// - render_debug_view: 当前是否需要构建图传显示画面
// 说明：普通巡线分支不再关心远端识别事件，图像链恢复成纯巡线 owner。
static void ProcessTrackingBranch(const cv::Mat& gray_frame,
                                  const cv::Mat& kernel,
                                  bool render_debug_view)
{
    RenderLineTrackingView(gray_frame, kernel, render_debug_view);
}

} // namespace

// 功能: 运行板图像侧顶层调度入口
// 类型: 图像运行时主循环
// 关键参数:
// - stream_enabled: 图传开关
// 主循环顺序：
// 1. 处理手动复位
// 2. 固定灰度抓帧
// 3. 根据当前状态决定走“绕行显示 / vehicle 保持航向 / 普通巡线”
// 4. 发布图传帧
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

        // 运行板固定采 160x120 灰度，不再承担本地彩色识别与分辨率切换职责。
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

        const uint64_t t_ms = now_ms();
        handler_sys.Update_Tick();
        // 顶层分支优先级固定：
        // 1. 绕行动作最高，完全旁路巡线
        // 2. vehicle 保持航向次之，直接接管 pure_angle
        // 3. 其余情况走普通巡线
        if (RenderBypassBranchIfNeeded(gray_frame, render_debug_view))
        {
        }
        else if (ProcessRemoteVehicleHoldBranchIfNeeded(gray_frame, t_ms, render_debug_view))
        {
        }
        else
        {
            ProcessTrackingBranch(gray_frame, kernel, render_debug_view);
        }

        stream.PublishFrame(view);
    }
}
