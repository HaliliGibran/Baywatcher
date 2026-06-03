#include "latest_frame_grabber.h"

#include <chrono>
#include <utility>

#include <unistd.h>

LatestFrameGrabber::~LatestFrameGrabber()
{
    Stop();
}

bool LatestFrameGrabber::Start(lq_camera_ex* camera)
{
    Stop();

    if (camera == nullptr)
    {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        camera_ = camera;
        latest_frame_.release();
        latest_seq_ = 0;
        latest_capture_ms_ = 0;
        perf_window_ = CapturePerfStats();
        running_ = true;
    }

    worker_ = std::thread(&LatestFrameGrabber::ThreadMain, this);
    return true;
}

void LatestFrameGrabber::Stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
    }
    frame_ready_cv_.notify_all();

    if (worker_.joinable())
    {
        worker_.join();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    camera_ = nullptr;
}

bool LatestFrameGrabber::WaitForFirstFrame(int timeout_ms)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (latest_seq_ > 0)
    {
        return true;
    }

    if (timeout_ms < 0)
    {
        timeout_ms = 0;
    }

    return frame_ready_cv_.wait_for(
        lock,
        std::chrono::milliseconds(timeout_ms),
        [this]() { return latest_seq_ > 0 || !running_; });
}

bool LatestFrameGrabber::GetLatestFrameSnapshot(cv::Mat* out_frame,
                                                uint64_t* out_seq,
                                                uint64_t* out_capture_ms) const
{
    if (out_frame == nullptr)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_seq_ == 0 || latest_frame_.empty())
    {
        out_frame->release();
        if (out_seq != nullptr)
        {
            *out_seq = 0;
        }
        if (out_capture_ms != nullptr)
        {
            *out_capture_ms = 0;
        }
        return false;
    }

    *out_frame = latest_frame_;
    if (out_seq != nullptr)
    {
        *out_seq = latest_seq_;
    }
    if (out_capture_ms != nullptr)
    {
        *out_capture_ms = latest_capture_ms_;
    }
    return true;
}

LatestFrameGrabber::CapturePerfStats LatestFrameGrabber::ConsumeCapturePerfWindow()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const CapturePerfStats snapshot = perf_window_;
    perf_window_ = CapturePerfStats();
    return snapshot;
}

void LatestFrameGrabber::ThreadMain()
{
    while (true)
    {
        lq_camera_ex* camera = nullptr;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_)
            {
                break;
            }
            camera = camera_;
        }

        if (camera == nullptr || !camera->is_cam_opened())
        {
            usleep(1000);
            continue;
        }

        const auto capture_begin = std::chrono::steady_clock::now();
        cv::Mat frame = camera->get_frame_raw();
        const auto capture_end = std::chrono::steady_clock::now();
        const double capture_ms =
            std::chrono::duration<double, std::milli>(capture_end - capture_begin).count();

        if (frame.empty())
        {
            usleep(1000);
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_)
            {
                break;
            }

            latest_frame_ = std::move(frame);
            latest_capture_ms_ = NowMs();
            ++latest_seq_;

            perf_window_.total_ms += capture_ms;
            if (capture_ms > perf_window_.max_ms)
            {
                perf_window_.max_ms = capture_ms;
            }
            ++perf_window_.count;
        }

        frame_ready_cv_.notify_all();
    }
}

uint64_t LatestFrameGrabber::NowMs()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}
