#pragma once

#include "lq_camera_ex.hpp"

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include <opencv2/opencv.hpp>

// 识别板 latest-frame 采集层。
// 后台线程持续抓相机帧，只保留最近一帧；
// 前台识别链只消费最新快照，被模型阻塞期间的旧帧会直接被覆盖丢弃。
class LatestFrameGrabber
{
public:
    struct CapturePerfStats
    {
        double total_ms = 0.0;
        double max_ms = 0.0;
        uint64_t count = 0;

        double AverageMs() const
        {
            return (count > 0) ? (total_ms / static_cast<double>(count)) : 0.0;
        }
    };

    LatestFrameGrabber() = default;
    LatestFrameGrabber(const LatestFrameGrabber&) = delete;
    LatestFrameGrabber& operator=(const LatestFrameGrabber&) = delete;
    ~LatestFrameGrabber();

    bool Start(lq_camera_ex* camera);
    void Stop();

    bool WaitForFirstFrame(int timeout_ms);
    bool GetLatestFrameSnapshot(cv::Mat* out_frame,
                                uint64_t* out_seq = nullptr,
                                uint64_t* out_capture_ms = nullptr) const;

    CapturePerfStats ConsumeCapturePerfWindow();

private:
    void ThreadMain();
    static uint64_t NowMs();

private:
    lq_camera_ex* camera_ = nullptr;
    mutable std::mutex mutex_;
    std::condition_variable frame_ready_cv_;
    std::thread worker_;
    cv::Mat latest_frame_;
    uint64_t latest_seq_ = 0;
    uint64_t latest_capture_ms_ = 0;
    CapturePerfStats perf_window_;
    bool running_ = false;
};
