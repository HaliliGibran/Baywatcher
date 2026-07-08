#include "recognition_runtime.h"

#include "latest_frame_grabber.h"
#include "recognition_chain.h"
#include "stream_chain.h"
#include "main.hpp"
#include <algorithm>
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
constexpr bool kRecognitionUToResultTimingLog = (BW_RECOG_U_TO_RESULT_TIMING_LOG_ENABLE != 0);

struct RuntimeWhiteReferenceStats
{
    bool valid = false;
    int sample_count = 0;
    float mean_b = 0.0f;
    float mean_g = 0.0f;
    float mean_r = 0.0f;
    float mean_luma = 0.0f;
};

struct RuntimeWhiteReferenceNormalizeState
{
    bool initialized = false;
    float gain_b = 1.0f;
    float gain_g = 1.0f;
    float gain_r = 1.0f;
};

static float clamp_float(float value, float min_value, float max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

static bool IsRuntimeWhiteReferenceSeed(const cv::Vec3b& bgr, const cv::Vec3b& hsv)
{
    const int max_rgb = std::max(std::max(static_cast<int>(bgr[0]), static_cast<int>(bgr[1])),
                                 static_cast<int>(bgr[2]));
    const int min_rgb = std::min(std::min(static_cast<int>(bgr[0]), static_cast<int>(bgr[1])),
                                 static_cast<int>(bgr[2]));
    return hsv[1] <= static_cast<unsigned char>(BW_RECOG_WHITE_REF_SEED_MAX_SATURATION) &&
           hsv[2] >= static_cast<unsigned char>(BW_RECOG_WHITE_REF_SEED_MIN_VALUE) &&
           min_rgb >= BW_RECOG_WHITE_REF_SEED_MIN_RGB &&
           (max_rgb - min_rgb) <= BW_RECOG_WHITE_REF_SEED_MAX_CHANNEL_DIFF;
}

static bool ComputeRuntimeWhiteReferenceStats(const cv::Mat& frame_bgr,
                                              RuntimeWhiteReferenceStats* out_stats)
{
    if (out_stats == nullptr)
    {
        return false;
    }
    *out_stats = RuntimeWhiteReferenceStats();
    if (frame_bgr.empty())
    {
        return false;
    }

    const int row_y = std::max(0, std::min(BW_RECOG_WHITE_REFERENCE_ROW_Y, frame_bgr.rows - 1));
    const int half_height = std::max(0, BW_RECOG_WHITE_REF_STATS_HALF_HEIGHT);
    const int y0 = std::max(0, row_y - half_height);
    const int y1 = std::min(frame_bgr.rows - 1, row_y + half_height);
    if (y1 < y0)
    {
        return false;
    }

    const cv::Mat roi_bgr = frame_bgr.rowRange(y0, y1 + 1);
    cv::Mat roi_hsv;
    cv::cvtColor(roi_bgr, roi_hsv, cv::COLOR_BGR2HSV);

    double sum_b = 0.0;
    double sum_g = 0.0;
    double sum_r = 0.0;
    double sum_luma = 0.0;
    int sample_count = 0;

    for (int y = 0; y < roi_bgr.rows; ++y)
    {
        const cv::Vec3b* bgr_row = roi_bgr.ptr<cv::Vec3b>(y);
        const cv::Vec3b* hsv_row = roi_hsv.ptr<cv::Vec3b>(y);
        for (int x = 0; x < roi_bgr.cols; ++x)
        {
            if (!IsRuntimeWhiteReferenceSeed(bgr_row[x], hsv_row[x]))
            {
                continue;
            }

            const float b = static_cast<float>(bgr_row[x][0]);
            const float g = static_cast<float>(bgr_row[x][1]);
            const float r = static_cast<float>(bgr_row[x][2]);
            sum_b += b;
            sum_g += g;
            sum_r += r;
            sum_luma += 0.114 * b + 0.587 * g + 0.299 * r;
            ++sample_count;
        }
    }

    if (sample_count <= 0)
    {
        return false;
    }

    out_stats->valid = true;
    out_stats->sample_count = sample_count;
    out_stats->mean_b = static_cast<float>(sum_b / sample_count);
    out_stats->mean_g = static_cast<float>(sum_g / sample_count);
    out_stats->mean_r = static_cast<float>(sum_r / sample_count);
    out_stats->mean_luma = static_cast<float>(sum_luma / sample_count);
    return true;
}

static RuntimeWhiteReferenceNormalizeState& GetRuntimeWhiteReferenceNormalizeState()
{
    static RuntimeWhiteReferenceNormalizeState state;
    return state;
}

static void ApplyRuntimeWhiteReferenceGains(cv::Mat* frame_bgr,
                                            const RuntimeWhiteReferenceNormalizeState& state)
{
    if (frame_bgr == nullptr || frame_bgr->empty() || !state.initialized)
    {
        return;
    }

    for (int y = 0; y < frame_bgr->rows; ++y)
    {
        cv::Vec3b* row = frame_bgr->ptr<cv::Vec3b>(y);
        for (int x = 0; x < frame_bgr->cols; ++x)
        {
            row[x][0] = cv::saturate_cast<unsigned char>(row[x][0] * state.gain_b);
            row[x][1] = cv::saturate_cast<unsigned char>(row[x][1] * state.gain_g);
            row[x][2] = cv::saturate_cast<unsigned char>(row[x][2] * state.gain_r);
        }
    }
}

static void ApplyRecognitionWhiteReferenceNormalization(cv::Mat* frame_bgr, bool allow_adapt)
{
#if BW_RECOG_WHITE_REF_NORMALIZE_ENABLE == 0
    (void)frame_bgr;
    (void)allow_adapt;
#else
    if (frame_bgr == nullptr || frame_bgr->empty())
    {
        return;
    }

    RuntimeWhiteReferenceNormalizeState& state = GetRuntimeWhiteReferenceNormalizeState();
    RuntimeWhiteReferenceStats stats;
    const bool has_stats = ComputeRuntimeWhiteReferenceStats(*frame_bgr, &stats);

    if (allow_adapt && has_stats && stats.valid)
    {
        const float safe_luma = std::max(stats.mean_luma, 1.0f);
        const float safe_b = std::max(stats.mean_b, 1.0f);
        const float safe_g = std::max(stats.mean_g, 1.0f);
        const float safe_r = std::max(stats.mean_r, 1.0f);
        const float target_luma = std::max(1.0f, BW_RECOG_WHITE_REF_NORMALIZE_TARGET_LUMA);
        const float avg_channel = (stats.mean_b + stats.mean_g + stats.mean_r) / 3.0f;
        const float alpha = clamp_float(BW_RECOG_WHITE_REF_NORMALIZE_ALPHA, 0.0f, 1.0f);

        const float luma_gain = clamp_float(
            target_luma / safe_luma,
            BW_RECOG_WHITE_REF_NORMALIZE_LUMA_GAIN_MIN,
            BW_RECOG_WHITE_REF_NORMALIZE_LUMA_GAIN_MAX);
        const float wb_gain_b = clamp_float(
            avg_channel / safe_b,
            BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MIN,
            BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MAX);
        const float wb_gain_g = clamp_float(
            avg_channel / safe_g,
            BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MIN,
            BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MAX);
        const float wb_gain_r = clamp_float(
            avg_channel / safe_r,
            BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MIN,
            BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MAX);

        const float target_gain_b = luma_gain * wb_gain_b;
        const float target_gain_g = luma_gain * wb_gain_g;
        const float target_gain_r = luma_gain * wb_gain_r;

        if (!state.initialized)
        {
            state.initialized = true;
            state.gain_b = target_gain_b;
            state.gain_g = target_gain_g;
            state.gain_r = target_gain_r;
        }
        else
        {
            state.gain_b += (target_gain_b - state.gain_b) * alpha;
            state.gain_g += (target_gain_g - state.gain_g) * alpha;
            state.gain_r += (target_gain_r - state.gain_r) * alpha;
        }
    }

    ApplyRuntimeWhiteReferenceGains(frame_bgr, state);
#endif
}

void ApplyRecognitionProcessingMask(cv::Mat* frame_bgr)
{
    if (frame_bgr == nullptr || frame_bgr->empty())
    {
        return;
    }

    const int rows = frame_bgr->rows;
    const int keep_y_min = std::max(0, std::min(BW_RECOG_PROCESS_KEEP_Y_MIN, rows));
    const int keep_y_max =
        std::max(keep_y_min, std::min(BW_RECOG_PROCESS_KEEP_Y_MAX, rows));

    if (keep_y_min > 0)
    {
        frame_bgr->rowRange(0, keep_y_min).setTo(cv::Scalar::all(0));
    }
    if (keep_y_max < rows)
    {
        frame_bgr->rowRange(keep_y_max, rows).setTo(cv::Scalar::all(0));
    }

    const int cols = frame_bgr->cols;
    if (cols > 0)
    {
        frame_bgr->colRange(0, 1).setTo(cv::Scalar::all(0));
        frame_bgr->colRange(cols - 1, cols).setTo(cv::Scalar::all(0));
    }
}

const char* VisionCodeText(BoardVisionCode code)
{
    switch (code)
    {
    case BoardVisionCode::VEHICLE: return "v";
    case BoardVisionCode::WEAPON: return "w";
    case BoardVisionCode::SUPPLY: return "s";
    case BoardVisionCode::BRICK: return "b";
    case BoardVisionCode::BRICK_LEFT: return "bl";
    case BoardVisionCode::BRICK_RIGHT: return "br";
    case BoardVisionCode::NO_RESULT: return "u";
    case BoardVisionCode::CLOTH_STOP: return "c";
    case BoardVisionCode::UNKNOWN: return "n";
    default: return "-";
    }
}

static bool IsRecognitionSuccessCode(BoardVisionCode code)
{
    return code == BoardVisionCode::VEHICLE ||
           code == BoardVisionCode::WEAPON ||
           code == BoardVisionCode::SUPPLY;
}

struct ClothStartDetectionResult
{
    bool active = false;
    cv::Rect search_rect;
    int green_pixels = 0;
    float green_ratio = 0.0f;
};

static bool IsBlindBoxGreenClothStartEnabled()
{
    return BW_SOFTWARE_BLIND_BOX_TASK == BW_SOFTWARE_BLIND_BOX_TASK_COLOR_CLOTH_START &&
           BW_SOFTWARE_BLIND_BOX_CLOTH_COLOR == BW_CLOTH_COLOR_GREEN;
}

static cv::Rect BuildClothCenterSearchRect(const cv::Mat& frame_bgr)
{
    if (frame_bgr.empty())
    {
        return cv::Rect();
    }

    const int cols = frame_bgr.cols;
    const int rows = frame_bgr.rows;
    const int keep_y_min = std::max(0, std::min(BW_RECOG_PROCESS_KEEP_Y_MIN, rows));
    const int keep_y_max = std::max(keep_y_min + 1, std::min(BW_RECOG_PROCESS_KEEP_Y_MAX, rows));
    const int center_x = cols / 2;
    const int center_y = (keep_y_min + keep_y_max - 1) / 2;
    const int half_width = std::max(1, BW_RECOG_CLOTH_CENTER_HALF_WIDTH);
    const int half_height = std::max(1, BW_RECOG_CLOTH_CENTER_HALF_HEIGHT);

    const int x0 = std::max(0, center_x - half_width);
    const int x1 = std::min(cols, center_x + half_width + 1);
    const int y0 = std::max(keep_y_min, center_y - half_height);
    const int y1 = std::min(keep_y_max, center_y + half_height + 1);
    if (x1 <= x0 || y1 <= y0)
    {
        return cv::Rect();
    }
    return cv::Rect(x0, y0, x1 - x0, y1 - y0);
}

static bool IsGreenClothPixel(const cv::Vec3b& bgr, const cv::Vec3b& hsv)
{
    const int h = static_cast<int>(hsv[0]);
    const int s = static_cast<int>(hsv[1]);
    const int v = static_cast<int>(hsv[2]);
    const int b = static_cast<int>(bgr[0]);
    const int g = static_cast<int>(bgr[1]);
    const int r = static_cast<int>(bgr[2]);
    const bool hue_match = (BW_RECOG_CLOTH_GREEN_H_MIN <= BW_RECOG_CLOTH_GREEN_H_MAX)
        ? (h >= BW_RECOG_CLOTH_GREEN_H_MIN && h <= BW_RECOG_CLOTH_GREEN_H_MAX)
        : (h >= BW_RECOG_CLOTH_GREEN_H_MIN || h <= BW_RECOG_CLOTH_GREEN_H_MAX);

    return hue_match &&
           s >= BW_RECOG_CLOTH_GREEN_S_MIN &&
           v >= BW_RECOG_CLOTH_GREEN_V_MIN &&
           (g - std::max(r, b)) >= BW_RECOG_CLOTH_GREEN_DOM_MIN;
}

static bool DetectBlindBoxGreenClothStart(const cv::Mat& frame_bgr,
                                          ClothStartDetectionResult* out_result)
{
    if (out_result != nullptr)
    {
        *out_result = ClothStartDetectionResult();
    }
    if (!IsBlindBoxGreenClothStartEnabled() || frame_bgr.empty())
    {
        return false;
    }

    const cv::Rect search_rect = BuildClothCenterSearchRect(frame_bgr);
    if (search_rect.width <= 0 || search_rect.height <= 0)
    {
        return false;
    }

    const cv::Mat roi_bgr = frame_bgr(search_rect);
    cv::Mat roi_hsv;
    cv::cvtColor(roi_bgr, roi_hsv, cv::COLOR_BGR2HSV);

    int green_pixels = 0;
    for (int y = 0; y < roi_bgr.rows; ++y)
    {
        const cv::Vec3b* bgr_row = roi_bgr.ptr<cv::Vec3b>(y);
        const cv::Vec3b* hsv_row = roi_hsv.ptr<cv::Vec3b>(y);
        for (int x = 0; x < roi_bgr.cols; ++x)
        {
            if (IsGreenClothPixel(bgr_row[x], hsv_row[x]))
            {
                ++green_pixels;
            }
        }
    }

    const int total_pixels = std::max(1, search_rect.area());
    const float green_ratio = static_cast<float>(green_pixels) / static_cast<float>(total_pixels);
    const bool active =
        green_pixels >= BW_RECOG_CLOTH_GREEN_MIN_PIXELS &&
        green_ratio >= BW_RECOG_CLOTH_GREEN_MIN_RATIO;

    if (out_result != nullptr)
    {
        out_result->active = active;
        out_result->search_rect = search_rect;
        out_result->green_pixels = green_pixels;
        out_result->green_ratio = green_ratio;
    }
    return active;
}

static void RenderClothStopView(const cv::Mat& frame_bgr,
                                const ClothStartDetectionResult& detection,
                                cv::Mat& view)
{
    view = frame_bgr.clone();
    if (view.empty())
    {
        return;
    }

    if (detection.search_rect.width > 0 && detection.search_rect.height > 0)
    {
        cv::rectangle(view, detection.search_rect, cv::Scalar(0, 255, 0), 2);
    }
    cv::putText(view, "BLIND BOX GREEN CLOTH -> c", cv::Point(10, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    std::ostringstream oss;
    oss << "green_pixels=" << detection.green_pixels
        << ", ratio=" << std::fixed << std::setprecision(3) << detection.green_ratio
        << ", speed_cap=0.01";
    cv::putText(view, oss.str(), cv::Point(10, 56),
                cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

struct UToResultTimingState
{
    bool active = false;
    steady_time_point_t trigger_read_begin;
    uint64_t trigger_frame_seq = 0;
    uint32_t frame_count = 0;
    uint32_t u_packet_count = 0;
    bool first_u_sent = false;
    uint8_t first_u_tx_seq = 0;
    double first_u_send_offset_ms = 0.0;
};

enum class RuntimeFrameStage
{
    DISABLED = 0,
    MANUAL_IDLE,
    CLOTH_STOP,
    TRY_ENTER,
    PROCESS_RECOG,
};

struct RuntimeFrameTimingSample
{
    bool valid = false;
    uint64_t frame_seq = 0;
    uint64_t t_ms = 0;
    RuntimeFrameStage stage = RuntimeFrameStage::TRY_ENTER;
    BoardVisionCode code_before = BoardVisionCode::INVALID;
    BoardVisionCode code_after = BoardVisionCode::INVALID;
    bool in_recognition_before = false;
    bool in_recognition_after = false;
    bool entered_recognition = false;
    bool exited_recognition = false;
    steady_time_point_t read_begin;
    steady_time_point_t chain_end;
    steady_time_point_t send_end;
    double capture_ms = 0.0;
    double prepare_ms = 0.0;
    double chain_ms = 0.0;
    double overlay_ms = 0.0;
    double send_state_ms = 0.0;
    double publish_ms = 0.0;
    double read_to_prepare_done_ms = 0.0;
    double read_to_chain_done_ms = 0.0;
    double read_to_overlay_done_ms = 0.0;
    double read_to_send_done_ms = 0.0;
    bool send_attempted = false;
    bool send_ok = false;
    uint8_t tx_seq = 0;
    RecognitionChain::PerfSample perf;
};

static const char* RuntimeFrameStageText(RuntimeFrameStage stage)
{
    switch (stage)
    {
    case RuntimeFrameStage::DISABLED: return "识别关闭";
    case RuntimeFrameStage::MANUAL_IDLE: return "手动待机";
    case RuntimeFrameStage::CLOTH_STOP: return "色布停车";
    case RuntimeFrameStage::TRY_ENTER: return "普通态触发";
    case RuntimeFrameStage::PROCESS_RECOG: return "识别态推理";
    default: return "未知阶段";
    }
}

static double elapsed_ms_since(const steady_time_point_t& begin,
                               const steady_time_point_t& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

static steady_time_point_t timing_frame_trace_end(const RuntimeFrameTimingSample& sample)
{
    return sample.send_attempted ? sample.send_end : sample.chain_end;
}

static void PrintTimingFrameLine(const char* label,
                                 const RuntimeFrameTimingSample& sample,
                                 const steady_time_point_t& trace_begin)
{
    if (!sample.valid)
    {
        std::cout << "[识别耗时] " << label << ": 无" << std::endl;
        return;
    }

    std::cout << "[识别耗时] " << label
              << ": 帧号=" << sample.frame_seq
              << ", 相对起点_ms=" << std::fixed << std::setprecision(2)
              << elapsed_ms_since(trace_begin, sample.read_begin)
              << ", 阶段=" << RuntimeFrameStageText(sample.stage)
              << ", 状态=" << VisionCodeText(sample.code_before)
              << "->" << VisionCodeText(sample.code_after)
              << ", 识别态=" << (sample.in_recognition_before ? "1" : "0")
              << "->" << (sample.in_recognition_after ? "1" : "0")
              << ", 读帧_ms=" << sample.capture_ms
              << ", 预处理_ms=" << sample.prepare_ms
              << ", 链路处理_ms=" << sample.chain_ms
              << ", 叠字_ms=" << sample.overlay_ms
              << ", 发包_ms=" << (sample.send_attempted ? sample.send_state_ms : 0.0)
              << ", 图传_ms=" << sample.publish_ms
              << ", 读帧到预处理完成_ms=" << sample.read_to_prepare_done_ms
              << ", 读帧到链路完成_ms=" << sample.read_to_chain_done_ms
              << ", 读帧到叠字完成_ms=" << sample.read_to_overlay_done_ms
              << ", 读帧到发包完成_ms="
              << (sample.send_attempted ? sample.read_to_send_done_ms : 0.0)
              << ", 发包成功=" << (sample.send_ok ? "是" : "否")
              << ", 包序号=" << static_cast<int>(sample.tx_seq)
              << std::endl;

    std::cout << "[识别耗时] " << label
              << ".细分: 普通态总耗时_ms=" << std::fixed << std::setprecision(2)
              << sample.perf.try_total_ms
              << ", 识别态总耗时_ms=" << sample.perf.process_recog_total_ms
              << ", ROI总耗时_ms=" << sample.perf.extract_roi_ms
              << ", 搜索框_ms=" << sample.perf.roi_search_rect_ms
              << ", 边线爬线_ms=" << sample.perf.roi_track_boundary_ms
              << ", 红色掩码_ms=" << sample.perf.roi_red_mask_ms
              << ", 红块选择_ms=" << sample.perf.roi_red_band_ms
              << ", 赛道区域判定_ms=" << sample.perf.roi_track_classify_ms
              << ", ROI构建透视_ms=" << sample.perf.roi_build_warp_ms
              << ", 模型总耗时_ms=" << sample.perf.onnx_infer_ms
              << ", 模型预处理_ms=" << sample.perf.onnx_preprocess_ms
              << ", setInput_ms=" << sample.perf.onnx_set_input_ms
              << ", forward_ms=" << sample.perf.onnx_forward_ms
              << ", 模型后处理_ms=" << sample.perf.onnx_postprocess_ms
              << ", 分类总耗时_ms=" << sample.perf.classify_total_ms
              << std::endl;
}

static void PrintUToResultTimingTrace(const UToResultTimingState& state,
                                      const RuntimeFrameTimingSample& trigger_frame,
                                      const RuntimeFrameTimingSample& enter_frame,
                                      const RuntimeFrameTimingSample& result_frame)
{
    if (!result_frame.valid)
    {
        return;
    }

    const steady_time_point_t result_trace_end = timing_frame_trace_end(result_frame);
    const double total_ms = elapsed_ms_since(state.trigger_read_begin, result_trace_end);
    const double trigger_to_first_u_send_ms =
        state.first_u_sent ? state.first_u_send_offset_ms : -1.0;
    const double trigger_to_enter_read_ms =
        enter_frame.valid ? elapsed_ms_since(state.trigger_read_begin, enter_frame.read_begin) : -1.0;
    const double trigger_to_enter_done_ms =
        enter_frame.valid ? elapsed_ms_since(state.trigger_read_begin, enter_frame.chain_end) : -1.0;
    const double trigger_to_result_read_ms =
        elapsed_ms_since(state.trigger_read_begin, result_frame.read_begin);

    std::cout << "[识别耗时] 起始帧=" << state.trigger_frame_seq
              << ", 结果=" << VisionCodeText(result_frame.code_after)
              << ", 总耗时_读帧到结果发包完成_ms=" << std::fixed << std::setprecision(2)
              << total_ms
              << ", 触发到首次u发包完成_ms=" << trigger_to_first_u_send_ms
              << ", 触发到进入识别帧读取_ms=" << trigger_to_enter_read_ms
              << ", 触发到进入识别帧处理完成_ms=" << trigger_to_enter_done_ms
              << ", 触发到结果帧读取_ms=" << trigger_to_result_read_ms
              << ", 经历帧数=" << state.frame_count
              << ", u包数量=" << state.u_packet_count
              << ", 首次u包序号="
              << (state.first_u_sent ? static_cast<int>(state.first_u_tx_seq) : -1)
              << ", 结果帧发包=" << (result_frame.send_attempted ? "是" : "否")
              << ", 结果帧发包成功=" << (result_frame.send_ok ? "是" : "否")
              << ", 结果包序号=" << static_cast<int>(result_frame.tx_seq)
              << std::endl;

    PrintTimingFrameLine("触发u帧", trigger_frame, state.trigger_read_begin);
    PrintTimingFrameLine("进入识别态帧", enter_frame, state.trigger_read_begin);
    PrintTimingFrameLine("结果帧", result_frame, state.trigger_read_begin);
}

static void UpdateUToResultTimingAfterFrame(UToResultTimingState* state,
                                            const RuntimeFrameTimingSample& sample)
{
    if (!kRecognitionUToResultTimingLog || state == nullptr || !sample.valid)
    {
        return;
    }

    static RuntimeFrameTimingSample trigger_frame;
    static RuntimeFrameTimingSample enter_frame;

    const bool triggers_u =
        sample.code_before != BoardVisionCode::NO_RESULT &&
        sample.code_after == BoardVisionCode::NO_RESULT;

    if (!state->active)
    {
        if (!triggers_u)
        {
            return;
        }

        *state = UToResultTimingState();
        state->active = true;
        state->trigger_read_begin = sample.read_begin;
        state->trigger_frame_seq = sample.frame_seq;
        trigger_frame = sample;
        enter_frame = RuntimeFrameTimingSample();

        std::cout << "[识别耗时] 统计开始: 起始帧=" << state->trigger_frame_seq
                  << ", 触发状态=" << VisionCodeText(sample.code_before)
                  << "->" << VisionCodeText(sample.code_after)
                  << ", 触发帧发包=" << (sample.send_attempted ? "是" : "否")
                  << ", 触发帧发包成功=" << (sample.send_ok ? "是" : "否")
                  << ", 包序号=" << static_cast<int>(sample.tx_seq)
                  << std::endl;
        PrintTimingFrameLine("触发u帧", trigger_frame, state->trigger_read_begin);
    }

    ++state->frame_count;

    if (sample.entered_recognition && !enter_frame.valid)
    {
        enter_frame = sample;
    }

    if (sample.send_ok && sample.code_after == BoardVisionCode::NO_RESULT)
    {
        if (!state->first_u_sent)
        {
            state->first_u_sent = true;
            state->first_u_tx_seq = sample.tx_seq;
            state->first_u_send_offset_ms =
                elapsed_ms_since(state->trigger_read_begin, sample.send_end);
        }
        ++state->u_packet_count;
        return;
    }

    if (IsRecognitionSuccessCode(sample.code_after))
    {
        PrintUToResultTimingTrace(*state, trigger_frame, enter_frame, sample);
        *state = UToResultTimingState();
        trigger_frame = RuntimeFrameTimingSample();
        enter_frame = RuntimeFrameTimingSample();
        return;
    }

    if (sample.code_after != BoardVisionCode::NO_RESULT &&
        sample.code_after != BoardVisionCode::INVALID)
    {
        const double abort_ms = sample.send_attempted
            ? elapsed_ms_since(state->trigger_read_begin, sample.send_end)
            : elapsed_ms_since(state->trigger_read_begin, sample.chain_end);
        std::cout << "[识别耗时] 统计中止_ms=" << std::fixed << std::setprecision(2)
                  << abort_ms
                  << ", 中止状态=" << VisionCodeText(sample.code_after)
                  << ", 经历帧数=" << state->frame_count
                  << ", u包数量=" << state->u_packet_count
                  << std::endl;
        PrintTimingFrameLine("触发u帧", trigger_frame, state->trigger_read_begin);
        PrintTimingFrameLine("中止帧", sample, state->trigger_read_begin);
        *state = UToResultTimingState();
        trigger_frame = RuntimeFrameTimingSample();
        enter_frame = RuntimeFrameTimingSample();
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

namespace recognition_runtime {

uint64_t now_ms()
{
    return recognition_now_ms();
}

void prepare_frame_for_processing(cv::Mat* frame_bgr, bool allow_adapt)
{
    ApplyRecognitionProcessingMask(frame_bgr);
    ApplyRecognitionWhiteReferenceNormalization(frame_bgr, allow_adapt);
}

cv::Mat build_publish_view(const cv::Mat& source_view)
{
    return BuildPublishView(source_view);
}

} // namespace recognition_runtime

// 功能: 识别板独立运行时主循环
// 类型: 图像运行时主循环
// 关键参数:
// - stream_enabled-是否启用图传
// - recognition_enabled_by_switch-是否允许启用模型识别
void RunRecognitionBoard(bool stream_enabled, bool recognition_enabled_by_switch)
{
    StreamChain stream(&server);
    RecognitionChain recognition;
    LatestFrameGrabber latest_frame_source;
    bool manual_test_started = (BW_RECOG_REQUIRE_MANUAL_START == 0);
    bool prev_in_recognition = false;
    bool manual_cycle_finished = false;
    uint8_t tx_seq = 0;
    BoardVisionCode last_sent_code = BoardVisionCode::INVALID;
    uint64_t last_send_ms = 0;
    uint64_t last_consumed_frame_seq = 0;
    uint64_t runtime_frame_seq = 0;
    UToResultTimingState u_to_result_timing;
    const bool render_debug = stream_enabled;
    const bool latest_frame_enabled = (BW_RECOG_LATEST_FRAME_ENABLE != 0);
    bool latest_frame_running = false;
    PerfWindowStats perf_window;
    steady_time_point_t perf_window_begin = steady_clock_t::now();

    stream.Initialize(stream_enabled);
    recognition.Initialize(recognition_enabled_by_switch);

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
        double prepare_ms = 0.0;
        double chain_ms = 0.0;
        double overlay_ms = 0.0;
        double send_state_ms = 0.0;
        double publish_ms = 0.0;
        uint64_t current_frame_seq = 0;
        bool send_state_called = false;
        bool send_state_ok = false;
        bool publish_called = false;
        steady_time_point_t send_end_time;

        // 1. 测试模式下，按 c 手动启动一次检测与识别链
        HandleManualRecognitionStart(&recognition, &manual_test_started);

        // 2. 识别板固定采彩色原图，不再承担巡线灰度处理职责
        if (!camera)
        {
            usleep(5 * 1000);
            continue;
        }

        const steady_time_point_t capture_begin = steady_clock_t::now();
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
        const steady_time_point_t capture_end = steady_clock_t::now();
        capture_ms = elapsed_ms_between(capture_begin, capture_end);
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
        if (current_frame_seq == 0)
        {
            current_frame_seq = ++runtime_frame_seq;
        }
        else
        {
            runtime_frame_seq = current_frame_seq;
        }

        const steady_time_point_t frame_read_begin = capture_begin;
        const BoardVisionCode code_before_frame = recognition.GetCurrentVisionCode();
        const bool in_recognition_before = recognition.IsInRecognitionMode();
        const uint64_t t_ms = recognition_runtime::now_ms();
        const steady_time_point_t prepare_begin = steady_clock_t::now();
        recognition_runtime::prepare_frame_for_processing(
            &img,
            !in_recognition_before && !recognition.HasRecentRedCandidate(t_ms));
        const steady_time_point_t prepare_end = steady_clock_t::now();
        prepare_ms = elapsed_ms_between(prepare_begin, prepare_end);

        ClothStartDetectionResult cloth_detection;
        const bool cloth_stop_active = DetectBlindBoxGreenClothStart(img, &cloth_detection);

        RuntimeFrameStage frame_stage = RuntimeFrameStage::TRY_ENTER;
        const steady_time_point_t chain_begin = steady_clock_t::now();
        if (cloth_stop_active)
        {
            frame_stage = RuntimeFrameStage::CLOTH_STOP;
            if (render_debug)
            {
                RenderClothStopView(img, cloth_detection, view);
            }
            else
            {
                view.release();
            }
        }
        else if (!recognition.IsEnabled())
        {
            frame_stage = RuntimeFrameStage::DISABLED;
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
            frame_stage = RuntimeFrameStage::PROCESS_RECOG;
            // 3. 识别态直接消费当前 320x240 原图做 ROI 分类
            recognition.ProcessRecognitionFrame(img, t_ms, view, render_debug);
        }
        else if (BW_RECOG_REQUIRE_MANUAL_START != 0 && !manual_test_started)
        {
            frame_stage = RuntimeFrameStage::MANUAL_IDLE;
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
            frame_stage = RuntimeFrameStage::TRY_ENTER;
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
        const steady_time_point_t chain_end = steady_clock_t::now();
        chain_ms = elapsed_ms_between(chain_begin, chain_end);

        const bool in_recognition_now = recognition.IsInRecognitionMode();
        const BoardVisionCode code_after_chain =
            cloth_stop_active ? BoardVisionCode::CLOTH_STOP : recognition.GetCurrentVisionCode();
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
                                 code_after_chain,
                                 cloth_stop_active ? 0.0 : recognition.GetCurrentBlobArea());
        const steady_time_point_t overlay_end = steady_clock_t::now();
        overlay_ms = elapsed_ms_between(overlay_begin, overlay_end);

        // 6. 状态流模式下，状态变化立即发包；未变化时按心跳周期补发。
        const BoardVisionCode code = code_after_chain;
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
            send_state_ok = comm.send_state(code, tx_seq);
            const steady_time_point_t send_end = steady_clock_t::now();
            send_end_time = send_end;
            send_state_ms = elapsed_ms_between(send_begin, send_end);
            send_state_called = true;
            last_send_ms = t_ms;
        }

        // 7. 发布图传画面
        const steady_time_point_t publish_begin = steady_clock_t::now();
        stream.PublishFrame(recognition_runtime::build_publish_view(view));
        const steady_time_point_t publish_end = steady_clock_t::now();
        publish_ms = elapsed_ms_between(publish_begin, publish_end);
        publish_called = true;

        RuntimeFrameTimingSample timing_sample;
        timing_sample.valid = true;
        timing_sample.frame_seq = current_frame_seq;
        timing_sample.t_ms = t_ms;
        timing_sample.stage = frame_stage;
        timing_sample.code_before = code_before_frame;
        timing_sample.code_after = code_after_chain;
        timing_sample.in_recognition_before = in_recognition_before;
        timing_sample.in_recognition_after = in_recognition_now;
        timing_sample.entered_recognition = !in_recognition_before && in_recognition_now;
        timing_sample.exited_recognition = in_recognition_before && !in_recognition_now;
        timing_sample.read_begin = frame_read_begin;
        timing_sample.chain_end = chain_end;
        timing_sample.send_end = send_end_time;
        timing_sample.capture_ms = capture_ms;
        timing_sample.prepare_ms = prepare_ms;
        timing_sample.chain_ms = chain_ms;
        timing_sample.overlay_ms = overlay_ms;
        timing_sample.send_state_ms = send_state_ms;
        timing_sample.publish_ms = publish_ms;
        timing_sample.read_to_prepare_done_ms = elapsed_ms_between(frame_read_begin, prepare_end);
        timing_sample.read_to_chain_done_ms = elapsed_ms_between(frame_read_begin, chain_end);
        timing_sample.read_to_overlay_done_ms = elapsed_ms_between(frame_read_begin, overlay_end);
        timing_sample.read_to_send_done_ms = send_state_called
            ? elapsed_ms_between(frame_read_begin, send_end_time)
            : 0.0;
        timing_sample.send_attempted = send_state_called;
        timing_sample.send_ok = send_state_ok;
        timing_sample.tx_seq = tx_seq;
        timing_sample.perf = recognition.GetLastPerfSample();
        UpdateUToResultTimingAfterFrame(&u_to_result_timing, timing_sample);

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
        if (!latest_frame_running)
        {
            perf_window.capture.Add(capture_ms);
        }
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
                PerfStageStats capture_stats = perf_window.capture;
                if (latest_frame_running)
                {
                    const LatestFrameGrabber::CapturePerfStats capture_perf =
                        latest_frame_source.ConsumeCapturePerfWindow();
                    capture_stats.total_ms = capture_perf.total_ms;
                    capture_stats.max_ms = capture_perf.max_ms;
                    capture_stats.count = capture_perf.count;
                }
                const double effective_fps =
                    (perf_window_ms > 0.0)
                        ? (perf_window.loop_count * 1000.0 / perf_window_ms)
                        : 0.0;
                const double measured_capture_fps =
                    (capture_stats.AverageMs() > 0.0)
                        ? (1000.0 / capture_stats.AverageMs())
                        : 0.0;
                std::cout << "[PERF] state=" << VisionCodeText(code)
                          << " fps=" << std::fixed << std::setprecision(2) << effective_fps
                          << " capture_fps=" << std::fixed << std::setprecision(2) << measured_capture_fps
                          << " capture=" << capture_stats.Format()
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
