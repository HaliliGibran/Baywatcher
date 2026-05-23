#include "lq_camera_ex.hpp"

#include <cmath>

namespace
{

uint16_t round_u16(double value, uint16_t fallback)
{
    if (!(value > 0.0))
    {
        return fallback;
    }

    if (value >= 65535.0)
    {
        return 65535u;
    }

    return static_cast<uint16_t>(std::lround(value));
}

}  // namespace

struct lq_camera_ex::lq_camera_ex_Impl
{
    mutable std::mutex mutex;
    cv::VideoCapture capture;
    uint16_t width = 0;
    uint16_t height = 0;
    uint16_t fps = 0;
    lq_camera_format_t format = LQ_CAMERA_HIGH_MJPG;
    std::string path = LQ_CAMERA_PATH;
    bool opened = false;

    void apply_capture_settings()
    {
        capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        capture.set(cv::CAP_PROP_FPS, fps);
        capture.set(cv::CAP_PROP_BUFFERSIZE, 1);

        const double actual_width = capture.get(cv::CAP_PROP_FRAME_WIDTH);
        const double actual_height = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
        const double actual_fps = capture.get(cv::CAP_PROP_FPS);

        width = round_u16(actual_width, width);
        height = round_u16(actual_height, height);
        fps = round_u16(actual_fps, fps);
    }

    bool open_capture()
    {
        capture.release();
        opened = false;

        if (!capture.open(path))
        {
            return false;
        }

        apply_capture_settings();
        opened = true;
        return true;
    }

    bool read_frame(cv::Mat& frame)
    {
        if (!opened || !capture.isOpened())
        {
            return false;
        }

        if (!capture.read(frame) || frame.empty())
        {
            frame.release();
            return false;
        }

        return true;
    }
};

lq_camera_ex::lq_camera_ex(uint16_t _width,
                           uint16_t _height,
                           uint16_t _fps,
                           lq_camera_format_t _fmt,
                           const std::string _path)
    : pImpl(new lq_camera_ex_Impl())
{
    init(_width, _height, _fps, _fmt, _path);
}

lq_camera_ex::~lq_camera_ex()
{
    stop_collect();
}

int lq_camera_ex::init(uint16_t _width,
                       uint16_t _height,
                       uint16_t _fps,
                       lq_camera_format_t _format,
                       const std::string _path)
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    pImpl->width = _width;
    pImpl->height = _height;
    pImpl->fps = _fps;
    pImpl->format = _format;
    pImpl->path = _path.empty() ? std::string(LQ_CAMERA_PATH) : _path;
    return pImpl->open_capture() ? 0 : -1;
}

int lq_camera_ex::start_collect()
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    if (pImpl->opened && pImpl->capture.isOpened())
    {
        return 0;
    }

    return pImpl->open_capture() ? 0 : -1;
}

int lq_camera_ex::stop_collect()
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    if (pImpl->capture.isOpened())
    {
        pImpl->capture.release();
    }
    pImpl->opened = false;
    return 0;
}

cv::Mat lq_camera_ex::get_frame_raw()
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    cv::Mat frame;
    if (!pImpl->read_frame(frame))
    {
        return cv::Mat();
    }
    return frame.clone();
}

cv::Mat lq_camera_ex::get_frame_gray()
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    cv::Mat frame;
    if (!pImpl->read_frame(frame))
    {
        return cv::Mat();
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

bool lq_camera_ex::get_frame_raw_gray(cv::Mat& raw, cv::Mat& gray)
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    cv::Mat frame;
    if (!pImpl->read_frame(frame))
    {
        raw.release();
        gray.release();
        return false;
    }

    raw = frame.clone();
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    return !raw.empty() && !gray.empty();
}

bool lq_camera_ex::set_exposure_manual(int16_t expo)
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    if (!pImpl->opened || !pImpl->capture.isOpened())
    {
        return false;
    }

    bool ok = false;
    ok = pImpl->capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 1.0) || ok;
    ok = pImpl->capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25) || ok;
    ok = pImpl->capture.set(cv::CAP_PROP_EXPOSURE, static_cast<double>(expo)) || ok;
    return ok;
}

bool lq_camera_ex::save_image_picture(const cv::Mat& frame, const std::string& filename)
{
    if (frame.empty() || filename.empty())
    {
        return false;
    }

    try
    {
        return cv::imwrite(filename, frame);
    }
    catch (const cv::Exception&)
    {
        return false;
    }
}

uint16_t lq_camera_ex::get_camera_width() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->width;
}

uint16_t lq_camera_ex::get_camera_height() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->height;
}

uint16_t lq_camera_ex::get_camera_fps() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->fps;
}

bool lq_camera_ex::is_cam_opened() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->opened && pImpl->capture.isOpened();
}
