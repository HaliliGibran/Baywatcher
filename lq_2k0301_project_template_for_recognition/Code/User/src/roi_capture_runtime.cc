#include "roi_capture_runtime.h"

#include "common.h"
#include "image_switch_utils.h"
#include "latest_frame_grabber.h"
#include "main.hpp"
#include "recognition_runtime.h"
#include "recognition_white_reference.h"
#include "roi_runtime_geometry.h"
#include "stream_chain.h"

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <netdb.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

namespace {

using steady_clock_t = std::chrono::steady_clock;
using steady_time_point_t = std::chrono::time_point<steady_clock_t>;
constexpr bool kRecognitionTextLog = (BW_RECOG_TEXT_LOG_ENABLE != 0);

enum class RoiCaptureModeState : uint8_t
{
    IDLE = 0,
    PREVIEW,
};

struct RoiSendFeedback
{
    std::string text;
    cv::Scalar color = cv::Scalar(0, 255, 255);
    uint64_t valid_until_ms = 0;
};

struct RoiBurstSendState
{
    bool active = false;
    int success_count = 0;
    int target_count = BW_RECOG_ROI_CAPTURE_BURST_COUNT;
};

struct FullFrameBurstSendState
{
    bool active = false;
    int success_count = 0;
    int target_count = BW_RECOG_FULLFRAME_CAPTURE_BURST_COUNT;
    uint64_t start_after_frame_index = 0;
};

static bool send_named_bgr_png_to_pc(const RoiCaptureTransferConfig& config,
                                     const cv::Mat& image_bgr,
                                     const char* name_prefix,
                                     uint64_t t_ms,
                                     std::string* out_message);

static bool send_all_bytes(int fd, const void* data, size_t size)
{
    const unsigned char* ptr = static_cast<const unsigned char*>(data);
    size_t sent = 0;
    while (sent < size)
    {
        const ssize_t n = send(fd, ptr + sent, size - sent, 0);
        if (n <= 0)
        {
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

static bool recv_ack_line(int fd, std::string* out_line)
{
    if (out_line == nullptr)
    {
        return false;
    }

    out_line->clear();
    char ch = 0;
    while (true)
    {
        const ssize_t n = recv(fd, &ch, 1, 0);
        if (n <= 0)
        {
            return false;
        }
        if (ch == '\n')
        {
            return true;
        }
        if (ch != '\r')
        {
            out_line->push_back(ch);
        }
    }
}

static bool connect_tcp_target(const RoiCaptureTransferConfig& config,
                               int* out_fd,
                               std::string* out_error)
{
    if (out_fd == nullptr)
    {
        return false;
    }
    *out_fd = -1;

    if (config.host.empty())
    {
        if (out_error != nullptr)
        {
            *out_error = "host empty, set --roi-host=...";
        }
        return false;
    }
    if (config.port <= 0)
    {
        if (out_error != nullptr)
        {
            *out_error = "invalid port";
        }
        return false;
    }

    struct addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* result = nullptr;
    std::ostringstream port_text;
    port_text << config.port;
    const int gai_rc = getaddrinfo(config.host.c_str(), port_text.str().c_str(), &hints, &result);
    if (gai_rc != 0 || result == nullptr)
    {
        if (out_error != nullptr)
        {
            *out_error = std::string("getaddrinfo failed: ") + gai_strerror(gai_rc);
        }
        return false;
    }

    const int timeout_ms = std::max(100, BW_RECOG_ROI_CAPTURE_SOCKET_TIMEOUT_MS);
    const struct timeval tv = {
        timeout_ms / 1000,
        static_cast<suseconds_t>((timeout_ms % 1000) * 1000)
    };

    bool connected = false;
    for (struct addrinfo* rp = result; rp != nullptr; rp = rp->ai_next)
    {
        const int fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0)
        {
            continue;
        }

        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0)
        {
            *out_fd = fd;
            connected = true;
            break;
        }
        close(fd);
    }
    freeaddrinfo(result);

    if (!connected)
    {
        if (out_error != nullptr)
        {
            *out_error = "connect failed";
        }
        return false;
    }
    return true;
}

static bool send_roi_to_pc(const RoiCaptureTransferConfig& config,
                           const cv::Mat& roi_bgr,
                           uint64_t t_ms,
                           std::string* out_message)
{
    return send_named_bgr_png_to_pc(
        config,
        roi_bgr,
        "roi",
        t_ms,
        out_message);
}

static bool send_named_bgr_png_to_pc(const RoiCaptureTransferConfig& config,
                                     const cv::Mat& image_bgr,
                                     const char* name_prefix,
                                     uint64_t t_ms,
                                     std::string* out_message)
{
    if (out_message != nullptr)
    {
        out_message->clear();
    }

    if (image_bgr.empty())
    {
        if (out_message != nullptr)
        {
            *out_message = "empty image";
        }
        return false;
    }

    std::vector<unsigned char> encoded_bytes;
    std::vector<int> encode_params;
    encode_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    encode_params.push_back(BW_RECOG_CAPTURE_PNG_COMPRESSION);
    if (!cv::imencode(".png", image_bgr, encoded_bytes, encode_params) || encoded_bytes.empty())
    {
        if (out_message != nullptr)
        {
            *out_message = "png encode failed";
        }
        return false;
    }

    int fd = -1;
    std::string connect_error;
    if (!connect_tcp_target(config, &fd, &connect_error))
    {
        if (out_message != nullptr)
        {
            *out_message = connect_error;
        }
        return false;
    }

    const char* prefix = (name_prefix != nullptr && name_prefix[0] != '\0') ? name_prefix : "capture";
    std::ostringstream name;
    name << prefix << "_" << t_ms << ".png";
    std::ostringstream header;
    header << "BWROI1\n";
    header << "name " << name.str() << "\n";
    header << "size " << encoded_bytes.size() << "\n";
    header << "\n";

    bool ok = send_all_bytes(fd, header.str().data(), header.str().size()) &&
              send_all_bytes(fd, encoded_bytes.data(), encoded_bytes.size());

    std::string ack_line;
    if (ok)
    {
        ok = recv_ack_line(fd, &ack_line);
    }
    close(fd);

    if (!ok)
    {
        if (out_message != nullptr)
        {
            *out_message = "send/ack failed";
        }
        return false;
    }

    if (ack_line.compare(0, 2, "OK") != 0)
    {
        if (out_message != nullptr)
        {
            *out_message = ack_line.empty() ? "receiver rejected" : ack_line;
        }
        return false;
    }

    if (out_message != nullptr)
    {
        *out_message = ack_line;
    }
    return true;
}

static cv::Mat build_fullframe_capture_crop(const cv::Mat& frame_bgr)
{
    if (frame_bgr.empty())
    {
        return cv::Mat();
    }

    const int y0 = std::max(0, std::min(BW_RECOG_PROCESS_KEEP_Y_MIN, frame_bgr.rows));
    const int y1 = std::max(y0, std::min(BW_RECOG_PROCESS_KEEP_Y_MAX, frame_bgr.rows));
    if (y1 <= y0)
    {
        return cv::Mat();
    }

    cv::Mat crop = frame_bgr.rowRange(y0, y1).clone();
    recognition_white_reference::ApplyGainsToMat(&crop);
    return crop;
}

static RoiExtractionResult extract_model_input_roi(const cv::Mat& frame_bgr,
                                                   bool render_debug,
                                                   bool* out_hold_white_reference)
{
    if (out_hold_white_reference != nullptr)
    {
        *out_hold_white_reference = false;
    }

    if (BW_RECOG_LIGHTWEIGHT_RED_PREFILTER_ENABLE == 0)
    {
        RoiExtractionResult result = ExtractRotatedRoi(
            frame_bgr,
            BW_RECOG_ROI_CAPTURE_OUTPUT_SIZE,
            DefaultRoiMethod(),
            render_debug);
        if (out_hold_white_reference != nullptr)
        {
            *out_hold_white_reference =
                result.status == "rotated_roi" && result.target_type == "marker";
        }
        return result;
    }

    RoiTrackRedPrefilterResult track_prefilter;
    const int early_slowdown_y_max =
        std::min(BW_RECOG_SLOWDOWN_TRIGGER_SEARCH_Y_MAX, BW_RECOG_TRIGGER_SEARCH_Y_MIN);
    const bool prefilter_ok = DetectTrackAwareRedPrefilter(
        frame_bgr,
        BW_RECOG_SLOWDOWN_TRIGGER_SEARCH_Y_MIN,
        early_slowdown_y_max,
        BW_RECOG_TRIGGER_SEARCH_Y_MIN,
        BW_RECOG_TRIGGER_SEARCH_Y_MAX,
        render_debug,
        &track_prefilter);

    RoiExtractionResult result;
    if (!prefilter_ok)
    {
        result.status = "prefilter_failed";
        return result;
    }

    const bool has_early_marker =
        BW_RECOG_EARLY_SLOWDOWN_ENABLE != 0 && track_prefilter.has_early_marker_red;
    if (out_hold_white_reference != nullptr)
    {
        *out_hold_white_reference =
            has_early_marker || track_prefilter.has_recognition_marker_red;
    }

    const bool has_recognition_red =
        track_prefilter.has_recognition_marker_red ||
        track_prefilter.has_recognition_brick_red;
    if (!has_recognition_red)
    {
        if (render_debug)
        {
            result.track_left_boundary = track_prefilter.track_left_boundary;
            result.track_right_boundary = track_prefilter.track_right_boundary;
            result.track_region_polygon = track_prefilter.track_region_polygon;
            result.has_track_left_boundary = !result.track_left_boundary.empty();
            result.has_track_right_boundary = !result.track_right_boundary.empty();
            result.has_track_region_polygon = result.track_region_polygon.size() >= 4;
        }
        result.status = "prefilter_no_trigger";
        return result;
    }

    return ExtractRotatedRoi(
        frame_bgr,
        BW_RECOG_ROI_CAPTURE_OUTPUT_SIZE,
        DefaultRoiMethod(),
        render_debug,
        track_prefilter.has_track_boundaries ? &track_prefilter : nullptr);
}

static void draw_roi_preview_panel(cv::Mat& view, const cv::Mat& roi_bgr)
{
    if (view.empty() || roi_bgr.empty())
    {
        return;
    }

    const int preview_size = std::max(96, BW_RECOG_ROI_PREVIEW_SIZE);
    const int margin = 16;
    if (view.cols < preview_size + margin * 2 || view.rows < preview_size + margin * 2)
    {
        return;
    }

    cv::Mat roi_preview;
    cv::resize(roi_bgr, roi_preview, cv::Size(preview_size, preview_size), 0, 0, cv::INTER_AREA);
    const cv::Rect preview_rect(
        view.cols - preview_size - margin,
        margin,
        preview_size,
        preview_size);
    roi_preview.copyTo(view(preview_rect));
    cv::rectangle(view, preview_rect, cv::Scalar(0, 255, 255), 2);
    cv::putText(view, "ROI_CAPTURE", cv::Point(preview_rect.x, preview_rect.y + preview_rect.height + 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
}

static void draw_roi_capture_idle_view(const cv::Mat& frame_bgr,
                                       cv::Mat& view,
                                       const RoiCaptureTransferConfig& config)
{
    view = frame_bgr.clone();
    cv::putText(view, "ROI Capture Idle", cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.70, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::putText(view, "1: preview | 2: send 10 lossless ROI | 3: send 10 lossless frames | 0: idle",
                cv::Point(10, 52), cv::FONT_HERSHEY_SIMPLEX,
                0.50, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    std::ostringstream host_info;
    host_info << "target=" << (config.host.empty() ? "<unset>" : config.host)
              << ":" << config.port;
    cv::putText(view, host_info.str(), cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX,
                0.50, cv::Scalar(255, 220, 0), 2, cv::LINE_AA);
}

static void draw_roi_capture_overlay(cv::Mat& view,
                                     const RoiExtractionResult& roi_result,
                                     const RoiCaptureTransferConfig& config,
                                     const RoiSendFeedback& feedback,
                                     const RoiBurstSendState& roi_burst_state,
                                     const FullFrameBurstSendState& fullframe_burst_state,
                                     uint64_t t_ms)
{
    if (view.empty())
    {
        return;
    }

    DrawRoiDebugOverlay(view, roi_result);
    draw_roi_preview_panel(view, roi_result.roi_bgr);

    cv::putText(view, "ROI Capture Preview", cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.70, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::putText(view, "1: preview  2: send 10 lossless ROI  3: send 10 lossless frames  0: idle",
                cv::Point(10, 52),
                cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    std::ostringstream roi_info;
    roi_info << "status=" << roi_result.status
             << " target=" << roi_result.target_type;
    cv::putText(view, roi_info.str(), cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX,
                0.50, cv::Scalar(255, 220, 0), 2, cv::LINE_AA);

    std::ostringstream host_info;
    host_info << "target=" << (config.host.empty() ? "<unset>" : config.host)
              << ":" << config.port
              << " roi=" << BW_RECOG_ROI_CAPTURE_OUTPUT_SIZE << "x" << BW_RECOG_ROI_CAPTURE_OUTPUT_SIZE
              << " frame_y=[" << BW_RECOG_PROCESS_KEEP_Y_MIN << "," << (BW_RECOG_PROCESS_KEEP_Y_MAX - 1) << "]";
    cv::putText(view, host_info.str(), cv::Point(10, 108), cv::FONT_HERSHEY_SIMPLEX,
                0.48, cv::Scalar(255, 220, 0), 2, cv::LINE_AA);

    std::ostringstream burst_info;
    burst_info << "burst_roi="
               << (roi_burst_state.active ? "RUN " : "IDLE ")
               << roi_burst_state.success_count << "/" << std::max(0, roi_burst_state.target_count)
               << "  burst_frame="
               << (fullframe_burst_state.active ? "RUN " : "IDLE ")
               << fullframe_burst_state.success_count << "/"
               << std::max(0, fullframe_burst_state.target_count);
    cv::putText(view, burst_info.str(), cv::Point(10, 136), cv::FONT_HERSHEY_SIMPLEX,
                0.50,
                (roi_burst_state.active || fullframe_burst_state.active)
                    ? cv::Scalar(0, 255, 0)
                    : cv::Scalar(255, 220, 0),
                2,
                cv::LINE_AA);

    if (feedback.valid_until_ms > t_ms && !feedback.text.empty())
    {
        cv::putText(view, feedback.text, cv::Point(10, 164), cv::FONT_HERSHEY_SIMPLEX,
                    0.55, feedback.color, 2, cv::LINE_AA);
    }
}

static bool read_nonblocking_key(char* out_key)
{
    if (out_key == nullptr)
    {
        return false;
    }
    char ch = 0;
    if (read(STDIN_FILENO, &ch, 1) != 1)
    {
        return false;
    }
    if (ch == '\n' || ch == '\r')
    {
        return false;
    }
    *out_key = ch;
    return true;
}

static double elapsed_ms_between(const steady_time_point_t& begin, const steady_time_point_t& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

} // namespace

bool RoiCaptureModeDefaultEnabled()
{
    return (BW_ENABLE_ROI_CAPTURE_MODE != 0);
}

bool ParseRoiCaptureModeSwitch(int argc, char** argv, bool default_value)
{
    bool enabled = default_value;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--roi-capture")
        {
            enabled = true;
            continue;
        }
        if (arg == "--no-roi-capture")
        {
            enabled = false;
            continue;
        }
        const std::string key = "--roi-capture=";
        if (arg.compare(0, key.size(), key) == 0)
        {
            bool parsed = enabled;
            if (ParseImageBoolSwitchText(arg.substr(key.size()), &parsed))
            {
                enabled = parsed;
            }
            continue;
        }
        if (arg == "--mode" && i + 1 < argc)
        {
            const std::string mode = argv[++i];
            if (mode == "roi" || mode == "roi-capture")
            {
                enabled = true;
            }
            else if (mode == "recognition")
            {
                enabled = false;
            }
            continue;
        }
        const std::string mode_key = "--mode=";
        if (arg.compare(0, mode_key.size(), mode_key) == 0)
        {
            const std::string mode = arg.substr(mode_key.size());
            if (mode == "roi" || mode == "roi-capture")
            {
                enabled = true;
            }
            else if (mode == "recognition")
            {
                enabled = false;
            }
        }
    }
    return enabled;
}

RoiCaptureTransferConfig ParseRoiCaptureTransferConfig(int argc, char** argv)
{
    RoiCaptureTransferConfig config;
    config.host = BW_RECOG_ROI_CAPTURE_SEND_HOST;
    config.port = BW_RECOG_ROI_CAPTURE_SEND_PORT;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--roi-host" && i + 1 < argc)
        {
            config.host = argv[++i];
            continue;
        }
        if (arg == "--roi-port" && i + 1 < argc)
        {
            config.port = std::max(0, std::atoi(argv[++i]));
            continue;
        }
        const std::string host_key = "--roi-host=";
        if (arg.compare(0, host_key.size(), host_key) == 0)
        {
            config.host = arg.substr(host_key.size());
            continue;
        }
        const std::string port_key = "--roi-port=";
        if (arg.compare(0, port_key.size(), port_key) == 0)
        {
            config.port = std::max(0, std::atoi(arg.substr(port_key.size()).c_str()));
            continue;
        }
    }
    return config;
}

void RunRoiCaptureBoard(bool stream_enabled, const RoiCaptureTransferConfig& transfer_config)
{
    StreamChain stream(&server);
    LatestFrameGrabber latest_frame_source;
    const bool latest_frame_enabled = (BW_RECOG_LATEST_FRAME_ENABLE != 0);
    const bool render_debug = stream_enabled;
    bool latest_frame_running = false;
    uint64_t last_consumed_frame_seq = 0;
    uint64_t consumed_frame_index = 0;
    uint64_t white_reference_hold_until_ms = 0;
    RoiCaptureModeState mode = RoiCaptureModeState::IDLE;
    RoiSendFeedback feedback;
    RoiBurstSendState roi_burst_state;
    FullFrameBurstSendState fullframe_burst_state;

    stream.Initialize(stream_enabled);

    if (latest_frame_enabled && camera && camera->is_cam_opened())
    {
        latest_frame_running = latest_frame_source.Start(camera.get());
        if (latest_frame_running)
        {
            latest_frame_source.WaitForFirstFrame(BW_RECOG_LATEST_FRAME_WAIT_FIRST_FRAME_MS);
        }
    }

    if (kRecognitionTextLog)
    {
        std::cout << "[ROI CAPTURE] mode=roi-capture target="
                  << (transfer_config.host.empty() ? "<unset>" : transfer_config.host)
                  << ":" << transfer_config.port
                  << " roi=" << BW_RECOG_ROI_CAPTURE_OUTPUT_SIZE << "x" << BW_RECOG_ROI_CAPTURE_OUTPUT_SIZE
                  << " roi_burst=" << BW_RECOG_ROI_CAPTURE_BURST_COUNT
                  << " roi_format=png"
                  << " fullframe_burst=" << BW_RECOG_FULLFRAME_CAPTURE_BURST_COUNT
                  << " fullframe_format=png"
                  << " png_compression=" << BW_RECOG_CAPTURE_PNG_COMPRESSION
                  << std::endl;
    }

    while (1)
    {
        const steady_time_point_t loop_begin = steady_clock_t::now();
        uint64_t current_frame_seq = 0;

        if (!camera)
        {
            usleep(5 * 1000);
            continue;
        }

        if (latest_frame_running)
        {
            if (!latest_frame_source.GetLatestFrameSnapshot(&img, &current_frame_seq, nullptr))
            {
                usleep(1000);
                continue;
            }
        }
        else
        {
            img = camera->get_frame_raw();
        }
        if (img.empty())
        {
            usleep(5 * 1000);
            continue;
        }
        if (latest_frame_running)
        {
            if (current_frame_seq == 0 || current_frame_seq == last_consumed_frame_seq)
            {
                usleep(1000);
                continue;
            }
            last_consumed_frame_seq = current_frame_seq;
        }
        ++consumed_frame_index;

        const uint64_t t_ms = recognition_runtime::now_ms();
        const bool hold_white_reference =
            mode == RoiCaptureModeState::PREVIEW &&
            t_ms < white_reference_hold_until_ms;
        recognition_runtime::prepare_frame_for_processing(&img, !hold_white_reference);

        RoiExtractionResult roi_result;
        bool hold_white_reference_from_frame = false;
        if (mode == RoiCaptureModeState::PREVIEW)
        {
            roi_result = extract_model_input_roi(
                img,
                render_debug,
                &hold_white_reference_from_frame);
            if (hold_white_reference_from_frame)
            {
                white_reference_hold_until_ms =
                    t_ms + static_cast<uint64_t>(std::max(0, BW_RECOG_U_LOSS_HOLD_MS));
            }
            else if (roi_result.status == "prefilter_failed")
            {
                white_reference_hold_until_ms = 0;
            }
        }

        char key = 0;
        while (read_nonblocking_key(&key))
        {
            if (key == '1')
            {
                mode = RoiCaptureModeState::PREVIEW;
                roi_burst_state.active = false;
                roi_burst_state.success_count = 0;
                fullframe_burst_state.active = false;
                fullframe_burst_state.success_count = 0;
                feedback.text = "preview on";
                feedback.color = cv::Scalar(0, 255, 255);
                feedback.valid_until_ms = t_ms + 1200;
            }
            else if (key == '2')
            {
                if (mode != RoiCaptureModeState::PREVIEW)
                {
                    feedback.text = "press 1 first";
                    feedback.color = cv::Scalar(0, 165, 255);
                    feedback.valid_until_ms = t_ms + 1500;
                }
                else
                {
                    fullframe_burst_state.active = false;
                    fullframe_burst_state.success_count = 0;
                    roi_burst_state.active = true;
                    roi_burst_state.success_count = 0;
                    roi_burst_state.target_count = std::max(1, BW_RECOG_ROI_CAPTURE_BURST_COUNT);
                    feedback.text = "burst send start";
                    feedback.color = cv::Scalar(0, 255, 255);
                    feedback.valid_until_ms = t_ms + 1500;
                }
            }
            else if (key == '3')
            {
                roi_burst_state.active = false;
                roi_burst_state.success_count = 0;
                fullframe_burst_state.active = true;
                fullframe_burst_state.success_count = 0;
                fullframe_burst_state.target_count =
                    std::max(1, BW_RECOG_FULLFRAME_CAPTURE_BURST_COUNT);
                fullframe_burst_state.start_after_frame_index = consumed_frame_index;
                feedback.text = "lossless frame burst start";
                feedback.color = cv::Scalar(0, 255, 255);
                feedback.valid_until_ms = t_ms + 1500;
            }
            else if (key == '0')
            {
                mode = RoiCaptureModeState::IDLE;
                white_reference_hold_until_ms = 0;
                roi_burst_state.active = false;
                roi_burst_state.success_count = 0;
                fullframe_burst_state.active = false;
                fullframe_burst_state.success_count = 0;
                feedback.text = "preview off";
                feedback.color = cv::Scalar(0, 255, 255);
                feedback.valid_until_ms = t_ms + 1200;
            }
        }

        if (mode == RoiCaptureModeState::PREVIEW && roi_burst_state.active)
        {
            if (roi_result.status == "rotated_roi" && !roi_result.roi_bgr.empty())
            {
                std::string send_message;
                if (send_roi_to_pc(transfer_config, roi_result.roi_bgr, t_ms, &send_message))
                {
                    ++roi_burst_state.success_count;
                    std::ostringstream ok_text;
                    ok_text << "burst send ok " << roi_burst_state.success_count
                            << "/" << roi_burst_state.target_count << ": " << send_message;
                    feedback.text = ok_text.str();
                    feedback.color = cv::Scalar(0, 255, 0);
                    feedback.valid_until_ms = t_ms + 1200;

                    if (roi_burst_state.success_count >= roi_burst_state.target_count)
                    {
                        roi_burst_state.active = false;
                        feedback.text = "burst done";
                        feedback.color = cv::Scalar(0, 255, 0);
                        feedback.valid_until_ms = t_ms + 2000;
                    }
                }
                else
                {
                    std::ostringstream fail_text;
                    fail_text << "burst send fail " << roi_burst_state.success_count
                              << "/" << roi_burst_state.target_count << ": " << send_message;
                    feedback.text = fail_text.str();
                    feedback.color = cv::Scalar(0, 0, 255);
                    feedback.valid_until_ms = t_ms + 1200;
                }
            }
            else
            {
                std::ostringstream wait_text;
                wait_text << "waiting valid roi " << roi_burst_state.success_count
                          << "/" << roi_burst_state.target_count;
                feedback.text = wait_text.str();
                feedback.color = cv::Scalar(0, 165, 255);
                feedback.valid_until_ms = t_ms + 500;
            }
        }

        if (fullframe_burst_state.active &&
            consumed_frame_index > fullframe_burst_state.start_after_frame_index)
        {
            const cv::Mat fullframe_crop = build_fullframe_capture_crop(img);
            if (fullframe_crop.empty())
            {
                feedback.text = "no valid frame crop";
                feedback.color = cv::Scalar(0, 0, 255);
                feedback.valid_until_ms = t_ms + 1200;
            }
            else
            {
                std::ostringstream prefix;
                prefix << "frame_y" << BW_RECOG_PROCESS_KEEP_Y_MIN
                       << "_" << (BW_RECOG_PROCESS_KEEP_Y_MAX - 1);
                std::string send_message;
                if (send_named_bgr_png_to_pc(
                        transfer_config,
                        fullframe_crop,
                        prefix.str().c_str(),
                        t_ms,
                        &send_message))
                {
                    ++fullframe_burst_state.success_count;
                    std::ostringstream ok_text;
                    ok_text << "frame send ok " << fullframe_burst_state.success_count
                            << "/" << fullframe_burst_state.target_count << ": " << send_message;
                    feedback.text = ok_text.str();
                    feedback.color = cv::Scalar(0, 255, 0);
                    feedback.valid_until_ms = t_ms + 1200;

                    if (fullframe_burst_state.success_count >= fullframe_burst_state.target_count)
                    {
                        fullframe_burst_state.active = false;
                        feedback.text = "lossless frame burst done";
                        feedback.color = cv::Scalar(0, 255, 0);
                        feedback.valid_until_ms = t_ms + 2000;
                    }
                }
                else
                {
                    std::ostringstream fail_text;
                    fail_text << "frame send fail " << fullframe_burst_state.success_count
                              << "/" << fullframe_burst_state.target_count << ": " << send_message;
                    feedback.text = fail_text.str();
                    feedback.color = cv::Scalar(0, 0, 255);
                    feedback.valid_until_ms = t_ms + 1200;
                }
            }
        }

        if (render_debug)
        {
            if (mode == RoiCaptureModeState::PREVIEW)
            {
                view = img.clone();
                draw_roi_capture_overlay(
                    view,
                    roi_result,
                    transfer_config,
                    feedback,
                    roi_burst_state,
                    fullframe_burst_state,
                    t_ms);
            }
            else
            {
                draw_roi_capture_idle_view(img, view, transfer_config);
                if (feedback.valid_until_ms > t_ms && !feedback.text.empty())
                {
                    cv::putText(view, feedback.text, cv::Point(10, 108), cv::FONT_HERSHEY_SIMPLEX,
                                0.55, feedback.color, 2, cv::LINE_AA);
                }
            }
        }
        else
        {
            view.release();
        }

        if (stream_enabled)
        {
            stream.PublishFrame(recognition_runtime::build_publish_view(view));
        }

        const double target_loop_ms =
            (BW_RECOG_LOOP_TARGET_FPS > 0)
                ? (1000.0 / static_cast<double>(BW_RECOG_LOOP_TARGET_FPS))
                : 0.0;
        if (target_loop_ms > 0.0)
        {
            const double elapsed_ms = elapsed_ms_between(loop_begin, steady_clock_t::now());
            if (elapsed_ms < target_loop_ms)
            {
                usleep(static_cast<useconds_t>((target_loop_ms - elapsed_ms) * 1000.0));
            }
        }
    }
}
