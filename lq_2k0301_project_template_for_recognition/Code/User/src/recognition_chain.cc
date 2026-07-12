#include "recognition_chain.h"

#include "common.h"
#include "image_switch_utils.h"
#include "recognition_mlp_weights.h"
#include "recognition_white_reference.h"
#include "roi_runtime_geometry.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace {

constexpr uint64_t kRecognitionTriggerRejectLogIntervalMs = 300;
constexpr int kRecognitionSlowdownMinSearchYInclusive = BW_RECOG_SLOWDOWN_TRIGGER_SEARCH_Y_MIN;
constexpr int kRecognitionSlowdownMaxSearchYExclusive = BW_RECOG_SLOWDOWN_TRIGGER_SEARCH_Y_MAX;
constexpr int kRecognitionMinSearchYInclusive = BW_RECOG_TRIGGER_SEARCH_Y_MIN;
constexpr int kRecognitionMaxSearchYExclusive = BW_RECOG_TRIGGER_SEARCH_Y_MAX;
constexpr uint64_t kRecognitionRecentCandidateHoldMs = BW_RECOG_U_LOSS_HOLD_MS;
constexpr bool kRecognitionTextLog = (BW_RECOG_TEXT_LOG_ENABLE != 0);
constexpr bool kRecognitionResultLog = (BW_RECOG_RESULT_LOG_ENABLE != 0);
constexpr bool kRecognitionVerboseLog = kRecognitionTextLog && (BW_RECOG_VERBOSE_LOG != 0);
constexpr float kRecognitionDecisionTop1Threshold = BW_RECOG_DECISION_TOP1_THRESHOLD;
constexpr float kRecognitionDecisionMarginThreshold = BW_RECOG_DECISION_MARGIN_THRESHOLD;
constexpr bool kRecognitionAdaptiveTwoFrameEnable = (BW_RECOG_ADAPTIVE_TWO_FRAME_ENABLE != 0);
constexpr float kRecognitionSingleFrameHighConfTop1Threshold =
    BW_RECOG_SINGLE_FRAME_HIGH_CONF_TOP1_THRESHOLD;
constexpr float kRecognitionSingleFrameHighConfMarginThreshold =
    BW_RECOG_SINGLE_FRAME_HIGH_CONF_MARGIN_THRESHOLD;
constexpr int kRecognitionAdaptiveTwoFrameMaxBadFrames =
    BW_RECOG_ADAPTIVE_TWO_FRAME_MAX_BAD_FRAMES;
constexpr int kRecognitionOnnxWarmupRuns = BW_RECOG_ONNX_WARMUP_RUNS;
constexpr bool kRecognitionManualMlpCompareOnnx =
    (BW_RECOG_MANUAL_MLP_COMPARE_ONNX != 0);
constexpr bool kRecognitionLightweightRedPrefilterEnable =
    (BW_RECOG_LIGHTWEIGHT_RED_PREFILTER_ENABLE != 0);
constexpr bool kRecognitionEarlySlowdownEnable =
    (BW_RECOG_EARLY_SLOWDOWN_ENABLE != 0);
constexpr bool kRecognitionTriggerFrameInferEnable =
    (BW_RECOG_TRIGGER_FRAME_INFER_ENABLE != 0);
constexpr bool kRecognitionCircleMarkerQualityGateEnable =
    (BW_RECOG_CIRCLE_MARKER_QUALITY_GATE_ENABLE != 0);
constexpr int kRecognitionModelInputSize = 32;
constexpr const char* kRecognitionModelRootDir =
    "./model_boardroi_transfer_mlp_rgb_128_s32_rank1";
constexpr const char* kRecognitionModelName = "rgb32_boardroi8_manual_mlp_128";
constexpr int kRecognitionModelClassToGrouped[8] = {1, 1, 2, 0, 1, 0, 0, 2};
constexpr size_t kRecognitionMaxClasses = RecognitionChain::kMaxModelClasses;

struct CircleMarkerQualityObservation
{
    bool geometry_available = false;
    bool position_passed = false;
    bool immediate = false;
    int bottom_y = -1;
    float center_x = -1.0f;
    float center_x_ratio = -1.0f;
    float min_x_ratio = BW_RECOG_CIRCLE_X_GATE_FAR_MIN_RATIO;
    float max_x_ratio = BW_RECOG_CIRCLE_X_GATE_FAR_MAX_RATIO;
    int width = 0;
    int height = 0;
    double area = 0.0;
    const char* reason = "missing_geometry";
};

static CircleMarkerQualityObservation evaluate_circle_marker_quality(
    const RoiExtractionResult& roi_result,
    int frame_width)
{
    CircleMarkerQualityObservation observation;
    if (roi_result.has_candidate_area)
    {
        observation.area = roi_result.candidate_area;
    }
    else if (roi_result.has_blob_area)
    {
        observation.area = roi_result.blob_area;
    }
    if (frame_width <= 0 ||
        !roi_result.has_candidate_center ||
        (!roi_result.has_blob_box && !roi_result.has_candidate_size))
    {
        return observation;
    }

    observation.geometry_available = true;
    observation.center_x = roi_result.candidate_center_x;
    observation.center_x_ratio =
        observation.center_x / static_cast<float>(frame_width);
    if (roi_result.has_blob_box)
    {
        observation.bottom_y =
            roi_result.blob_box.y + roi_result.blob_box.height - 1;
        observation.width = roi_result.blob_box.width;
        observation.height = roi_result.blob_box.height;
    }
    else
    {
        observation.bottom_y = static_cast<int>(
            std::lround(roi_result.candidate_center_y +
                        roi_result.candidate_height * 0.5f - 1.0f));
        observation.width = roi_result.candidate_width;
        observation.height = roi_result.candidate_height;
    }

    const float y_span = static_cast<float>(
        std::max(1, BW_RECOG_CIRCLE_X_GATE_NEAR_Y - BW_RECOG_CIRCLE_X_GATE_FAR_Y));
    const float y_alpha = std::max(
        0.0f,
        std::min(
            1.0f,
            (static_cast<float>(observation.bottom_y) -
             static_cast<float>(BW_RECOG_CIRCLE_X_GATE_FAR_Y)) /
                y_span));
    observation.min_x_ratio =
        BW_RECOG_CIRCLE_X_GATE_FAR_MIN_RATIO +
        y_alpha * (BW_RECOG_CIRCLE_X_GATE_NEAR_MIN_RATIO -
                   BW_RECOG_CIRCLE_X_GATE_FAR_MIN_RATIO);
    observation.max_x_ratio =
        BW_RECOG_CIRCLE_X_GATE_FAR_MAX_RATIO +
        y_alpha * (BW_RECOG_CIRCLE_X_GATE_NEAR_MAX_RATIO -
                   BW_RECOG_CIRCLE_X_GATE_FAR_MAX_RATIO);

    if (observation.bottom_y < BW_RECOG_CIRCLE_TRIGGER_BOTTOM_Y_MIN)
    {
        observation.reason = "bottom_y_too_far";
        return observation;
    }
    if (observation.center_x_ratio < observation.min_x_ratio ||
        observation.center_x_ratio > observation.max_x_ratio)
    {
        observation.reason = "center_x_outside_gate";
        return observation;
    }

    observation.position_passed = true;
    observation.immediate =
        observation.bottom_y >= BW_RECOG_CIRCLE_TRIGGER_IMMEDIATE_BOTTOM_Y;
    observation.reason = observation.immediate
        ? "immediate_distance"
        : "stable_frame_required";
    return observation;
}

static_assert(recognition_mlp_weights::kInputSize == 32,
              "recognition_mlp_weights input size mismatch");
static_assert(recognition_mlp_weights::kChannels == 3,
              "recognition_mlp_weights channel count mismatch");
static_assert(recognition_mlp_weights::kInputElements == 3 * 32 * 32,
              "recognition_mlp_weights input element count mismatch");
static_assert(recognition_mlp_weights::kHiddenUnits == 128,
              "recognition_mlp_weights hidden size mismatch");
static_assert(recognition_mlp_weights::kClassCount == 8,
              "recognition_mlp_weights class count mismatch");

struct RoiClassificationResult
{
    int predicted_index = -1;
    std::array<float, kRecognitionMaxClasses> probabilities = {};
};

struct RoiClassificationTiming
{
    double preprocess_ms = 0.0;
    double set_input_ms = 0.0;
    double forward_ms = 0.0;
    double postprocess_ms = 0.0;
};

struct DeployCalibration
{
    float temperature = 1.0f;
    std::array<float, kRecognitionMaxClasses> logit_bias = {};
    float decision_top1_threshold = kRecognitionDecisionTop1Threshold;
    float decision_margin_threshold = kRecognitionDecisionMarginThreshold;
    bool loaded = false;
};

struct ProbabilityDecisionSummary
{
    int top1_index = -1;
    int top2_index = -1;
    float top1_prob = 0.0f;
    float top2_prob = 0.0f;
    float margin = 0.0f;
};

static RoiClassificationResult finalize_logits_to_result(const float* logits,
                                                         int logits_count,
                                                         float calibration_temperature,
                                                         const std::array<float, kRecognitionMaxClasses>& logit_bias)
{
    RoiClassificationResult result;
    if (logits == nullptr || logits_count <= 0)
    {
        return result;
    }

    const int count = std::min(logits_count, static_cast<int>(kRecognitionMaxClasses));
    float max_logit = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < count; ++i)
    {
        const float value = logits[i] / std::max(calibration_temperature, 1e-4f)
            + logit_bias[static_cast<size_t>(i)];
        if (value > max_logit)
        {
            max_logit = value;
        }
    }
    if (!std::isfinite(max_logit))
    {
        return result;
    }

    float exp_sum = 0.0f;
    for (int i = 0; i < count; ++i)
    {
        const float adjusted = logits[i] / std::max(calibration_temperature, 1e-4f)
            + logit_bias[static_cast<size_t>(i)];
        const float exp_value = std::exp(adjusted - max_logit);
        result.probabilities[static_cast<size_t>(i)] = exp_value;
        exp_sum += exp_value;
    }
    if (exp_sum <= 0.0f)
    {
        return result;
    }

    int best_index = 0;
    float best_prob = -1.0f;
    for (int i = 0; i < count; ++i)
    {
        result.probabilities[static_cast<size_t>(i)] /= exp_sum;
        if (result.probabilities[static_cast<size_t>(i)] > best_prob)
        {
            best_prob = result.probabilities[static_cast<size_t>(i)];
            best_index = i;
        }
    }
    result.predicted_index = best_index;
    return result;
}

static RoiClassificationResult finalize_logits_to_result(const cv::Mat& logits_f32,
                                                         float calibration_temperature,
                                                         const std::array<float, kRecognitionMaxClasses>& logit_bias)
{
    if (logits_f32.empty())
    {
        return RoiClassificationResult();
    }
    const cv::Mat logits_row = logits_f32.reshape(1, 1);
    return finalize_logits_to_result(
        logits_row.ptr<float>(0),
        static_cast<int>(logits_row.total()),
        calibration_temperature,
        logit_bias);
}

static void draw_trigger_search_info(cv::Mat& view)
{
    std::ostringstream oss;
    oss << "slowdown_y=[" << kRecognitionSlowdownMinSearchYInclusive
        << "," << kRecognitionSlowdownMaxSearchYExclusive << ")"
        << " recog_y=[" << kRecognitionMinSearchYInclusive
        << "," << kRecognitionMaxSearchYExclusive << ")";
    cv::putText(view, oss.str(), cv::Point(16, 84), cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(255, 220, 0), 2, cv::LINE_AA);
}

static void draw_roi_preview_inset(cv::Mat& view, const cv::Mat& roi_bgr)
{
    if (view.empty() || roi_bgr.empty() || BW_RECOG_ROI_PREVIEW_SIZE <= 0)
    {
        return;
    }

    const int preview_size = BW_RECOG_ROI_PREVIEW_SIZE;
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
    cv::Mat preview_roi = view(preview_rect);
    roi_preview.copyTo(preview_roi);
    cv::rectangle(view, preview_rect, cv::Scalar(0, 255, 255), 2);
    cv::putText(view, "ROI", cv::Point(preview_rect.x, preview_rect.y + preview_rect.height + 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
}

static BoardVisionCode vision_code_from_target_code(uint8_t target_code)
{
    switch (target_code)
    {
    case 1: return BoardVisionCode::WEAPON;
    case 2: return BoardVisionCode::SUPPLY;
    case 3: return BoardVisionCode::VEHICLE;
    default: return BoardVisionCode::INVALID;
    }
}

static const char* vision_code_text(BoardVisionCode code)
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

static bool is_brick_vision_code(BoardVisionCode code)
{
    return code == BoardVisionCode::BRICK ||
           code == BoardVisionCode::BRICK_LEFT ||
           code == BoardVisionCode::BRICK_RIGHT;
}

static bool is_success_symbol_code(BoardVisionCode code)
{
    return code == BoardVisionCode::VEHICLE ||
           code == BoardVisionCode::WEAPON ||
           code == BoardVisionCode::SUPPLY;
}


static double roi_observed_red_area(const RoiExtractionResult& roi_result)
{
    if (roi_result.has_candidate_area)
    {
        return roi_result.candidate_area;
    }
    if (roi_result.has_loose_blob_area)
    {
        return roi_result.loose_blob_area;
    }
    if (roi_result.has_max_red_contour_area)
    {
        return roi_result.max_red_contour_area;
    }
    if (roi_result.has_blob_area)
    {
        return roi_result.blob_area;
    }
    return 0.0;
}



static bool roi_has_non_noise_red(const RoiExtractionResult& roi_result)
{
    return roi_observed_red_area(roi_result) >= 80.0;
}


static const char* roi_blob_side_lower(const RoiExtractionResult& roi_result, int frame_width)
{
    if (frame_width <= 0)
    {
        return "unknown";
    }

    const cv::Rect* box =
        roi_result.has_loose_blob_box ? &roi_result.loose_blob_box :
        (roi_result.has_max_red_contour_box ? &roi_result.max_red_contour_box :
         (roi_result.has_blob_box ? &roi_result.blob_box : nullptr));
    if (box == nullptr)
    {
        return "unknown";
    }

    const int center_x = box->x + box->width / 2;
    int split_x = frame_width / 2;
    if (roi_result.has_track_classify_bounds)
    {
        split_x = (roi_result.track_classify_left_x + roi_result.track_classify_right_x) / 2;
    }
    return (center_x < split_x) ? "left" : "right";
}

static BoardVisionCode brick_code_from_roi_result(const RoiExtractionResult& roi_result, int frame_width)
{
    const cv::Rect* box =
        roi_result.has_loose_blob_box ? &roi_result.loose_blob_box :
        (roi_result.has_max_red_contour_box ? &roi_result.max_red_contour_box :
         (roi_result.has_blob_box ? &roi_result.blob_box : nullptr));
    if (box == nullptr)
    {
        return BoardVisionCode::BRICK;
    }

    const int center_x = box->x + box->width / 2;
    int split_x = (frame_width > 0) ? (frame_width / 2) : center_x;
    if (roi_result.has_track_classify_bounds)
    {
        split_x = (roi_result.track_classify_left_x + roi_result.track_classify_right_x) / 2;
    }

    return (center_x < split_x) ? BoardVisionCode::BRICK_LEFT : BoardVisionCode::BRICK_RIGHT;
}

static const char* roi_blob_side_upper(const RoiExtractionResult& roi_result, int frame_width)
{
    if (frame_width <= 0)
    {
        return "UNKNOWN";
    }

    const cv::Rect* box =
        roi_result.has_loose_blob_box ? &roi_result.loose_blob_box :
        (roi_result.has_max_red_contour_box ? &roi_result.max_red_contour_box :
         (roi_result.has_blob_box ? &roi_result.blob_box : nullptr));
    if (box == nullptr)
    {
        return "UNKNOWN";
    }

    const int center_x = box->x + box->width / 2;
    int split_x = frame_width / 2;
    if (roi_result.has_track_classify_bounds)
    {
        split_x = (roi_result.track_classify_left_x + roi_result.track_classify_right_x) / 2;
    }
    return (center_x < split_x) ? "LEFT" : "RIGHT";
}

static std::string roi_reject_reason_text(const RoiExtractionResult& roi_result)
{
    std::ostringstream oss;
    oss << "roi_status=" << roi_result.status;
    oss << ", loose_red_area=" << std::fixed << std::setprecision(1) << roi_observed_red_area(roi_result);
    oss << ", white_crop=" << (roi_result.white_crop_clipped ? "clipped" : "no");
    oss << ", reject_stage=" << roi_result.max_red_reject_stage;
    oss << ", touches_top=" << (roi_result.touches_search_top ? "yes" : "no");
    oss << ", touch_top_expand=" << (roi_result.touch_top_expand ? "yes" : "no");
    if (roi_result.has_expanded_blob_area)
    {
        oss << ", expanded_blob_area=" << std::fixed << std::setprecision(1)
            << roi_result.expanded_blob_area;
    }
    oss << ", touch_top_skip_refine=" << (roi_result.touch_top_skip_refine ? "yes" : "no");
    oss << ", touch_top_brick_override=" << (roi_result.touch_top_brick_override ? "yes" : "no");
    oss << ", loose_ipm_valid=" << (roi_result.loose_ipm_valid ? "yes" : "no");
    if (!roi_result.loose_ipm_reason.empty())
    {
        oss << ", loose_ipm_reason=" << roi_result.loose_ipm_reason;
    }
    if (roi_result.merged_reference_span_count > 0)
    {
        oss << ", reference_spans=" << roi_result.merged_reference_span_count;
    }
    if (roi_result.has_track_forward_direction)
    {
        oss << ", track_forward="
            << (roi_result.roi_used_track_forward_direction ? "used" : "fallback");
    }
    if (!roi_result.ipm_reason.empty())
    {
        oss << ", ipm_reason=" << roi_result.ipm_reason;
    }
    return oss.str();
}

static std::string red_observation_text(const RoiExtractionResult& roi_result, int frame_width)
{
    if (!roi_has_non_noise_red(roi_result))
    {
        return "NO RED";
    }

    if (roi_result.target_type == "roadblock" || roi_result.status == "roadblock")
    {
        std::ostringstream oss;
        oss << "BRICK " << roi_blob_side_upper(roi_result, frame_width);
        return oss.str();
    }

    if (roi_result.target_type == "marker" && roi_result.status != "rotated_roi")
    {
        return "SIGN-LIKE RED, ROI NOT READY";
    }

    if (roi_result.target_type == "marker")
    {
        return "MARKER";
    }

    return "RED PRESENT";
}



static bool roi_is_visible_sign_candidate(const RoiExtractionResult& roi_result)
{
    return roi_result.target_type == "marker";
}



static bool roi_should_hold_success_latch(const RoiExtractionResult& roi_result)
{
    return roi_result.target_type == "marker";
}

static bool roi_is_valid_marker_red_observation(const RoiExtractionResult& roi_result)
{
    return roi_result.target_type == "marker";
}


static BoardVisionCode fallback_code_from_roi_result(const RoiExtractionResult& roi_result, int frame_width)
{
    if (roi_result.target_type == "roadblock" || roi_result.status == "roadblock")
    {
        return brick_code_from_roi_result(roi_result, frame_width);
    }
    if (roi_result.target_type == "marker")
    {
        return BoardVisionCode::NO_RESULT;
    }

    const double red_area = roi_observed_red_area(roi_result);
    if (red_area < 80.0)
    {
        return BoardVisionCode::UNKNOWN;
    }

    return BoardVisionCode::UNKNOWN;
}


// [Recognition Chain] 文件存在性检查。
// 作用：在模型加载前快速判断运行时路径是否有效。
static bool file_exists(const std::string& path)
{
    return (!path.empty()) && (access(path.c_str(), F_OK) == 0);
}

static bool is_absolute_path(const std::string& path)
{
    if (path.empty())
    {
        return false;
    }
    if (path[0] == '/' || path[0] == '\\')
    {
        return true;
    }
    return (path.size() > 1 && std::isalpha(static_cast<unsigned char>(path[0])) && path[1] == ':');
}

static std::string get_dirname(const std::string& path)
{
    const std::string::size_type pos = path.find_last_of("/\\");
    if (pos == std::string::npos)
    {
        return ".";
    }
    if (pos == 0)
    {
        return path.substr(0, 1);
    }
    return path.substr(0, pos);
}

static std::string join_path(const std::string& base, const std::string& rel)
{
    if (base.empty())
    {
        return rel;
    }
    if (rel.empty())
    {
        return base;
    }
    const char tail = base[base.size() - 1];
    if (tail == '/' || tail == '\\')
    {
        return base + rel;
    }
    return base + "/" + rel;
}

static std::string strip_leading_dot_slash(const std::string& path)
{
    if (path.size() >= 2 && path[0] == '.' && (path[1] == '/' || path[1] == '\\'))
    {
        return path.substr(2);
    }
    return path;
}

static std::string get_current_working_directory()
{
    char buffer[PATH_MAX] = {0};
    if (getcwd(buffer, sizeof(buffer)) == nullptr)
    {
        return "";
    }
    return std::string(buffer);
}

static std::string get_executable_directory()
{
    char buffer[PATH_MAX] = {0};
    const ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (len <= 0)
    {
        return "";
    }
    buffer[len] = '\0';
    return get_dirname(std::string(buffer));
}

// [Recognition Chain] 运行时路径解析。
// 作用：把相对路径依次尝试为“当前工作目录”和“可执行文件目录”下的实际路径。
static std::string resolve_runtime_path(const std::string& configured_path)
{
    if (configured_path.empty())
    {
        return configured_path;
    }
    if (is_absolute_path(configured_path) && file_exists(configured_path))
    {
        return configured_path;
    }
    if (file_exists(configured_path))
    {
        return configured_path;
    }

    const std::string suffix = strip_leading_dot_slash(configured_path);
    const std::string exe_dir = get_executable_directory();
    if (!exe_dir.empty())
    {
        const std::string exe_relative = join_path(exe_dir, suffix);
        if (file_exists(exe_relative))
        {
            return exe_relative;
        }
    }

    const std::string cwd = get_current_working_directory();
    if (!cwd.empty())
    {
        const std::string cwd_relative = join_path(cwd, suffix);
        if (file_exists(cwd_relative))
        {
            return cwd_relative;
        }
    }

    return configured_path;
}

// [Recognition Chain] 解析类别文件中的转义字符串。
// 作用：兼容 class_names.json 里的基础 JSON 转义字符。
static std::string unescape_json(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    bool escaping = false;
    for (char ch : s)
    {
        if (!escaping)
        {
            if (ch == '\\')
            {
                escaping = true;
            }
            else
            {
                out.push_back(ch);
            }
            continue;
        }

        switch (ch)
        {
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        default: out.push_back(ch); break;
        }
        escaping = false;
    }
    return out;
}

// [Recognition Chain] 加载类别名列表。
// 作用：把 class_names.json 转成推理后的类别索引 -> 类别名映射。
static float parse_json_number_or_default(const std::string& content,
                                          const std::string& key,
                                          float default_value)
{
    const std::regex re("\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
    std::smatch match;
    if (!std::regex_search(content, match, re) || match.size() < 2)
    {
        return default_value;
    }
    try
    {
        return std::stof(match[1].str());
    }
    catch (...)
    {
        return default_value;
    }
}

static std::array<float, kRecognitionMaxClasses> parse_json_float_array(const std::string& content,
                                                                        const std::string& key,
                                                                        const std::array<float, kRecognitionMaxClasses>& default_value)
{
    const std::regex re("\"" + key + "\"\\s*:\\s*\\[([^\\]]*)\\]");
    std::smatch match;
    if (!std::regex_search(content, match, re) || match.size() < 2)
    {
        return default_value;
    }
    std::array<float, kRecognitionMaxClasses> out = default_value;
    std::stringstream ss(match[1].str());
    std::string item;
    int idx = 0;
    while (std::getline(ss, item, ',') && idx < static_cast<int>(kRecognitionMaxClasses))
    {
        try
        {
            out[static_cast<size_t>(idx)] = std::stof(item);
        }
        catch (...)
        {
        }
        ++idx;
    }
    return out;
}

static DeployCalibration load_deploy_calibration_json(const std::string& path)
{
    DeployCalibration calibration;
    if (!file_exists(path))
    {
        return calibration;
    }

    std::ifstream fin(path);
    if (!fin.is_open())
    {
        return calibration;
    }
    std::ostringstream ss;
    ss << fin.rdbuf();
    const std::string content = ss.str();

    calibration.temperature = std::max(1e-4f, parse_json_number_or_default(content, "temperature", 1.0f));
    calibration.logit_bias = parse_json_float_array(content, "logit_bias", calibration.logit_bias);
    calibration.decision_top1_threshold = parse_json_number_or_default(content, "decision_top1_threshold", kRecognitionDecisionTop1Threshold);
    calibration.decision_margin_threshold = parse_json_number_or_default(content, "decision_margin_threshold", kRecognitionDecisionMarginThreshold);
    calibration.loaded = true;
    return calibration;
}

static std::vector<std::string> load_class_names_from_json(const std::string& path)
{
    std::ifstream fin(path);
    if (!fin.is_open())
    {
        return {};
    }

    std::ostringstream ss;
    ss << fin.rdbuf();
    const std::string content = ss.str();
    std::vector<std::string> out;

    if (content.find('[') != std::string::npos)
    {
        const std::regex re("\"((?:\\\\.|[^\"\\\\])*)\"");
        for (std::sregex_iterator it(content.begin(), content.end(), re), end; it != end; ++it)
        {
            out.push_back(unescape_json((*it)[1].str()));
        }
    }
    else if (content.find('{') != std::string::npos)
    {
        const std::regex re("\"((?:\\\\.|[^\"\\\\])*)\"\\s*:\\s*\"((?:\\\\.|[^\"\\\\])*)\"");
        std::vector<std::pair<int, std::string>> kv;
        for (std::sregex_iterator it(content.begin(), content.end(), re), end; it != end; ++it)
        {
            const std::string k = unescape_json((*it)[1].str());
            const std::string v = unescape_json((*it)[2].str());
            int key = 0;
            try { key = std::stoi(k); } catch (...) { key = static_cast<int>(kv.size()); }
            kv.push_back(std::make_pair(key, v));
        }
        std::sort(kv.begin(), kv.end(),
                  [](const std::pair<int, std::string>& a, const std::pair<int, std::string>& b) {
                      return a.first < b.first;
                  });
        for (size_t i = 0; i < kv.size(); ++i)
        {
            out.push_back(kv[i].second);
        }
    }

    return out;
}

static std::vector<std::string> expected_class_names()
{
    return {"急救包", "急救包（空白）", "急救车", "手枪", "望远镜", "步枪", "炸药包", "装甲车"};
}

static std::string describe_class_name_mismatch(const std::vector<std::string>& actual,
                                                const std::vector<std::string>& expected)
{
    std::ostringstream message;
    if (actual.empty())
    {
        message << "class_names.json missing, unreadable, or empty";
        return message.str();
    }
    if (actual.size() != expected.size())
    {
        message << "class count mismatch: expected=" << expected.size()
                << ", actual=" << actual.size();
        return message.str();
    }
    for (size_t i = 0; i < expected.size(); ++i)
    {
        if (actual[i] != expected[i])
        {
            message << "class order mismatch at index " << i
                    << ": expected=" << expected[i]
                    << ", actual=" << actual[i];
            return message.str();
        }
    }
    return {};
}

static size_t recognition_accum_class_count(size_t model_class_count)
{
    (void)model_class_count;
    return 3;
}

static const char* grouped_class_name(int grouped_index)
{
    switch (grouped_index)
    {
    case 0: return "weapon";
    case 1: return "supply";
    case 2: return "vehicle";
    default: return "unknown";
    }
}

static uint8_t grouped_target_code(int grouped_index)
{
    switch (grouped_index)
    {
    case 0: return 1;
    case 1: return 2;
    case 2: return 3;
    default: return 0;
    }
}

static void accumulate_probabilities_for_runtime_decision(
    const RoiClassificationResult& cls,
    const std::vector<std::string>& class_names,
    std::array<float, kRecognitionMaxClasses>& prob_sum)
{
    const size_t active_class_count = std::min(
        class_names.size(),
        static_cast<size_t>(recognition_mlp_weights::kClassCount));
    for (size_t i = 0; i < active_class_count; ++i)
    {
        const int grouped_index = kRecognitionModelClassToGrouped[i];
        prob_sum[static_cast<size_t>(grouped_index)] += cls.probabilities[i];
    }
}

static std::string runtime_decision_label(int decision_index,
                                          const std::vector<std::string>& class_names)
{
    (void)class_names;
    return grouped_class_name(decision_index);
}

static uint8_t runtime_decision_target_code(int decision_index,
                                            const std::vector<std::string>& class_names)
{
    (void)class_names;
    return grouped_target_code(decision_index);
}

static ProbabilityDecisionSummary summarize_classification_result(const RoiClassificationResult& cls,
                                                                  int class_count)
{
    ProbabilityDecisionSummary summary;
    const int count = std::min(class_count, static_cast<int>(kRecognitionMaxClasses));
    for (int i = 0; i < count; ++i)
    {
        const float prob = cls.probabilities[static_cast<size_t>(i)];
        if (summary.top1_index < 0 || prob > summary.top1_prob)
        {
            summary.top2_index = summary.top1_index;
            summary.top2_prob = summary.top1_prob;
            summary.top1_index = i;
            summary.top1_prob = prob;
        }
        else if (summary.top2_index < 0 || prob > summary.top2_prob)
        {
            summary.top2_index = i;
            summary.top2_prob = prob;
        }
    }
    summary.margin = summary.top1_prob - summary.top2_prob;
    return summary;
}

static void build_rgb32_subclass_input(const cv::Mat& roi_bgr,
                                       std::array<float, recognition_mlp_weights::kInputElements>& input)
{
    input.fill(0.0f);
    if (roi_bgr.empty() || roi_bgr.channels() != 3)
    {
        return;
    }

    cv::Mat resized;
    const cv::Mat* src = &roi_bgr;
    if (roi_bgr.cols != recognition_mlp_weights::kInputSize ||
        roi_bgr.rows != recognition_mlp_weights::kInputSize)
    {
        cv::resize(roi_bgr, resized,
                   cv::Size(recognition_mlp_weights::kInputSize,
                            recognition_mlp_weights::kInputSize),
                   0, 0, cv::INTER_AREA);
        src = &resized;
    }

    constexpr int plane = recognition_mlp_weights::kInputSize * recognition_mlp_weights::kInputSize;
    for (int y = 0; y < recognition_mlp_weights::kInputSize; ++y)
    {
        const cv::Vec3b* row = src->ptr<cv::Vec3b>(y);
        for (int x = 0; x < recognition_mlp_weights::kInputSize; ++x)
        {
            const int idx = y * recognition_mlp_weights::kInputSize + x;
            const float b = static_cast<float>(row[x][0]) * (1.0f / 255.0f);
            const float g = static_cast<float>(row[x][1]) * (1.0f / 255.0f);
            const float r = static_cast<float>(row[x][2]) * (1.0f / 255.0f);
            input[idx] = (r - recognition_mlp_weights::kMean[0]) / recognition_mlp_weights::kStd[0];
            input[plane + idx] = (g - recognition_mlp_weights::kMean[1]) / recognition_mlp_weights::kStd[1];
            input[2 * plane + idx] = (b - recognition_mlp_weights::kMean[2]) / recognition_mlp_weights::kStd[2];
        }
    }
}

static void run_manual_rgb32_mlp(const float* input,
                                 std::array<float, recognition_mlp_weights::kClassCount>& logits)
{
    std::array<float, recognition_mlp_weights::kHiddenUnits> hidden = {};
    for (int o = 0; o < recognition_mlp_weights::kHiddenUnits; ++o)
    {
        const float* weight = recognition_mlp_weights::kFc1Weight +
            o * recognition_mlp_weights::kInputElements;
        float sum = recognition_mlp_weights::kFc1Bias[o];
        for (int i = 0; i < recognition_mlp_weights::kInputElements; ++i)
        {
            sum += input[i] * weight[i];
        }
        hidden[static_cast<size_t>(o)] = std::max(sum, 0.0f);
    }

    for (int o = 0; o < recognition_mlp_weights::kClassCount; ++o)
    {
        const float* weight = recognition_mlp_weights::kFc2Weight +
            o * recognition_mlp_weights::kHiddenUnits;
        float sum = recognition_mlp_weights::kFc2Bias[o];
        for (int i = 0; i < recognition_mlp_weights::kHiddenUnits; ++i)
        {
            sum += hidden[static_cast<size_t>(i)] * weight[i];
        }
        logits[static_cast<size_t>(o)] = sum;
    }
}

static RoiClassificationResult classify_roi_index_manual_rgb32(
    const cv::Mat& roi_bgr,
    float calibration_temperature,
    const std::array<float, kRecognitionMaxClasses>& logit_bias,
    RoiClassificationTiming* timing,
    std::array<float, kRecognitionMaxClasses>* out_logits)
{
    const auto preprocess_begin = std::chrono::steady_clock::now();
    std::array<float, recognition_mlp_weights::kInputElements> input = {};
    build_rgb32_subclass_input(roi_bgr, input);
    const auto preprocess_end = std::chrono::steady_clock::now();

    const auto forward_begin = preprocess_end;
    std::array<float, recognition_mlp_weights::kClassCount> logits = {};
    run_manual_rgb32_mlp(input.data(), logits);
    const auto forward_end = std::chrono::steady_clock::now();

    if (out_logits != nullptr)
    {
        out_logits->fill(0.0f);
        for (int i = 0; i < recognition_mlp_weights::kClassCount; ++i)
        {
            (*out_logits)[static_cast<size_t>(i)] = logits[static_cast<size_t>(i)];
        }
    }

    const auto postprocess_begin = forward_end;
    RoiClassificationResult result = finalize_logits_to_result(
        logits.data(),
        recognition_mlp_weights::kClassCount,
        calibration_temperature,
        logit_bias);
    const auto postprocess_end = std::chrono::steady_clock::now();

    if (timing != nullptr)
    {
        timing->preprocess_ms =
            std::chrono::duration<double, std::milli>(preprocess_end - preprocess_begin).count();
        timing->set_input_ms = 0.0;
        timing->forward_ms =
            std::chrono::duration<double, std::milli>(forward_end - forward_begin).count();
        timing->postprocess_ms =
            std::chrono::duration<double, std::milli>(postprocess_end - postprocess_begin).count();
    }
    return result;
}

// [Recognition Chain] 当前 RGB32 MLP 的 ONNX 对拍推理。
static RoiClassificationResult classify_roi_index_onnx(cv::dnn::Net& net,
                                                       const cv::Mat& roi_bgr,
                                                       float calibration_temperature,
                                                       const std::array<float, kRecognitionMaxClasses>& logit_bias,
                                                       RoiClassificationTiming* timing = nullptr,
                                                       std::array<float, kRecognitionMaxClasses>* out_logits = nullptr)
{
    const auto preprocess_begin = std::chrono::steady_clock::now();
    std::array<float, recognition_mlp_weights::kInputElements> input = {};
    build_rgb32_subclass_input(roi_bgr, input);
    const int sizes[4] = {
        1,
        recognition_mlp_weights::kChannels,
        recognition_mlp_weights::kInputSize,
        recognition_mlp_weights::kInputSize};
    cv::Mat blob(4, sizes, CV_32F, input.data());

    const auto preprocess_end = std::chrono::steady_clock::now();
    const auto set_input_begin = preprocess_end;
    net.setInput(blob);
    const auto set_input_end = std::chrono::steady_clock::now();
    const auto forward_begin = set_input_end;
    cv::Mat out = net.forward().reshape(1, 1);
    const auto forward_end = std::chrono::steady_clock::now();
    const auto postprocess_begin = forward_end;
    cv::Mat out_f;
    out.convertTo(out_f, CV_32F);
    if (out_logits != nullptr)
    {
        out_logits->fill(0.0f);
        const cv::Mat logits_row = out_f.reshape(1, 1);
        const int count = std::min(static_cast<int>(logits_row.total()),
                                   static_cast<int>(kRecognitionMaxClasses));
        const float* logits_ptr = logits_row.ptr<float>(0);
        for (int i = 0; i < count; ++i)
        {
            (*out_logits)[static_cast<size_t>(i)] = logits_ptr[i];
        }
    }
    RoiClassificationResult result =
        finalize_logits_to_result(out_f, calibration_temperature, logit_bias);
    const auto postprocess_end = std::chrono::steady_clock::now();

    if (timing != nullptr)
    {
        timing->preprocess_ms =
            std::chrono::duration<double, std::milli>(preprocess_end - preprocess_begin).count();
        timing->set_input_ms =
            std::chrono::duration<double, std::milli>(set_input_end - set_input_begin).count();
        timing->forward_ms =
            std::chrono::duration<double, std::milli>(forward_end - forward_begin).count();
        timing->postprocess_ms =
            std::chrono::duration<double, std::milli>(postprocess_end - postprocess_begin).count();
    }

    return result;
}

static void maybe_log_manual_mlp_compare(const RoiClassificationResult& manual_result,
                                         const RoiClassificationResult& onnx_result,
                                         const std::array<float, kRecognitionMaxClasses>& manual_logits,
                                         const std::array<float, kRecognitionMaxClasses>& onnx_logits)
{
    if (!kRecognitionResultLog)
    {
        return;
    }

    constexpr int kClassCount = recognition_mlp_weights::kClassCount;
    float max_logit_abs_diff = 0.0f;
    float max_prob_abs_diff = 0.0f;
    for (int i = 0; i < kClassCount; ++i)
    {
        max_logit_abs_diff = std::max(
            max_logit_abs_diff,
            std::fabs(manual_logits[static_cast<size_t>(i)] - onnx_logits[static_cast<size_t>(i)]));
        max_prob_abs_diff = std::max(
            max_prob_abs_diff,
            std::fabs(manual_result.probabilities[static_cast<size_t>(i)] -
                      onnx_result.probabilities[static_cast<size_t>(i)]));
    }

    const ProbabilityDecisionSummary manual_summary =
        summarize_classification_result(manual_result, kClassCount);
    const ProbabilityDecisionSummary onnx_summary =
        summarize_classification_result(onnx_result, kClassCount);
    const bool top1_same = (manual_summary.top1_index == onnx_summary.top1_index);
    const bool warn =
        !top1_same ||
        max_logit_abs_diff > BW_RECOG_MANUAL_MLP_COMPARE_LOGIT_DIFF_WARN ||
        max_prob_abs_diff > BW_RECOG_MANUAL_MLP_COMPARE_PROB_DIFF_WARN;

    static auto last_log_time = std::chrono::steady_clock::time_point();
    const auto now = std::chrono::steady_clock::now();
    const bool interval_elapsed =
        last_log_time.time_since_epoch().count() == 0 ||
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >=
            BW_RECOG_MANUAL_MLP_COMPARE_LOG_INTERVAL_MS;
    if (!warn && !interval_elapsed)
    {
        return;
    }
    last_log_time = now;

    std::cout << "[MLP_COMPARE]"
              << " status=" << (warn ? "warn" : "ok")
              << ", max_logit_abs_diff=" << std::fixed << std::setprecision(6)
              << max_logit_abs_diff
              << ", max_prob_abs_diff=" << max_prob_abs_diff
              << ", manual_top1=" << manual_summary.top1_index
              << ", manual_top2=" << manual_summary.top2_index
              << ", manual_margin=" << manual_summary.margin
              << ", onnx_top1=" << onnx_summary.top1_index
              << ", onnx_top2=" << onnx_summary.top2_index
              << ", onnx_margin=" << onnx_summary.margin
              << ", top1_same=" << (top1_same ? "yes" : "no")
              << std::endl;
}

static RoiClassificationResult classify_roi_index(cv::dnn::Net& net,
                                                  const cv::Mat& roi_bgr,
                                                  float calibration_temperature,
                                                  const std::array<float, kRecognitionMaxClasses>& logit_bias,
                                                  RoiClassificationTiming* timing = nullptr)
{
    RoiClassificationTiming manual_timing;
    std::array<float, kRecognitionMaxClasses> manual_logits = {};
    const RoiClassificationResult manual_result = classify_roi_index_manual_rgb32(
        roi_bgr,
        calibration_temperature,
        logit_bias,
        &manual_timing,
        &manual_logits);

    if (!kRecognitionManualMlpCompareOnnx)
    {
        if (timing != nullptr)
        {
            *timing = manual_timing;
        }
        return manual_result;
    }

    std::array<float, kRecognitionMaxClasses> onnx_logits = {};
    const RoiClassificationResult onnx_result = classify_roi_index_onnx(
        net,
        roi_bgr,
        calibration_temperature,
        logit_bias,
        nullptr,
        &onnx_logits);
    maybe_log_manual_mlp_compare(manual_result, onnx_result, manual_logits, onnx_logits);

    if (timing != nullptr)
    {
        *timing = manual_timing;
    }
    return manual_result;
}

static void warmup_recognition_net(cv::dnn::Net& net,
                                   float calibration_temperature,
                                   const std::array<float, kRecognitionMaxClasses>& logit_bias)
{
    if (net.empty() || kRecognitionOnnxWarmupRuns <= 0)
    {
        return;
    }

    cv::Mat dummy_roi(kRecognitionModelInputSize, kRecognitionModelInputSize, CV_8UC3,
                      cv::Scalar(128, 128, 128));
    for (int i = 0; i < kRecognitionOnnxWarmupRuns; ++i)
    {
        const auto begin = std::chrono::steady_clock::now();
        (void)classify_roi_index_onnx(net, dummy_roi, calibration_temperature, logit_bias);
        const auto end = std::chrono::steady_clock::now();
        if (kRecognitionResultLog)
        {
            const double warmup_ms = std::chrono::duration<double, std::milli>(end - begin).count();
            std::cout << "[ONNX] warmup " << (i + 1) << "/" << kRecognitionOnnxWarmupRuns
                      << ", infer_ms=" << std::fixed << std::setprecision(2) << warmup_ms
                      << std::endl;
        }
    }
}

struct RedCandidateBounds
{
    int min_x = std::numeric_limits<int>::max();
    int max_x = -1;
    int min_y = std::numeric_limits<int>::max();
    int max_y = -1;
    int count = 0;

    void Add(int x, int y)
    {
        ++count;
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }

    bool ToRect(int min_pixel_count, int min_width, int min_height, cv::Rect* out_rect) const
    {
        if (count < min_pixel_count || max_x < min_x || max_y < min_y)
        {
            return false;
        }
        const int width = max_x - min_x + 1;
        const int height = max_y - min_y + 1;
        if (width < min_width || height < min_height)
        {
            return false;
        }
        if (out_rect != nullptr)
        {
            *out_rect = cv::Rect(min_x, min_y, width, height);
        }
        return true;
    }
};

static bool red_pixel_passes_thresholds(int b,
                                        int g,
                                        int r,
                                        int red_score_threshold,
                                        int min_r_threshold,
                                        int red_dom_threshold)
{
    const int red_score = 2 * r - g - b;
    const int dom = r - std::max(g, b);
    return red_score >= red_score_threshold &&
           r >= min_r_threshold &&
           dom >= red_dom_threshold;
}

static bool detect_red_candidate_in_y_range(const cv::Mat& frame_bgr,
                                            const cv::Rect& limit_rect,
                                            int y_min_inclusive,
                                            int y_max_exclusive,
                                            cv::Rect* best_rect,
                                            int red_score_threshold,
                                            int min_r_threshold,
                                            int red_dom_threshold,
                                            int min_pixel_count,
                                            int min_width,
                                            int min_height)
{
    if (frame_bgr.empty() || limit_rect.width <= 0 || limit_rect.height <= 0)
    {
        return false;
    }

    const int y0 = std::max(0, std::min(y_min_inclusive, frame_bgr.rows - 1));
    const int y1 = std::max(y0 + 1, std::min(y_max_exclusive, frame_bgr.rows));
    const int scan_x0 = std::max(0, limit_rect.x);
    const int scan_x1 = std::min(frame_bgr.cols, limit_rect.x + limit_rect.width);
    const int scan_y0 = std::max(y0, limit_rect.y);
    const int scan_y1 = std::min(y1, limit_rect.y + limit_rect.height);
    if (scan_y1 <= scan_y0 || scan_x1 <= scan_x0)
    {
        return false;
    }

    RedCandidateBounds bounds;

    for (int y = scan_y0; y < scan_y1; ++y)
    {
        const cv::Vec3b* row_ptr = frame_bgr.ptr<cv::Vec3b>(y);
        for (int x = scan_x0; x < scan_x1; ++x)
        {
            const cv::Vec3b adjusted_bgr =
                recognition_white_reference::ApplyGainsToPixel(row_ptr[x]);
            const int b = static_cast<int>(adjusted_bgr[0]);
            const int g = static_cast<int>(adjusted_bgr[1]);
            const int r = static_cast<int>(adjusted_bgr[2]);
            if (!red_pixel_passes_thresholds(
                    b, g, r, red_score_threshold, min_r_threshold, red_dom_threshold))
            {
                continue;
            }

            bounds.Add(x, y);
        }
    }

    return bounds.ToRect(min_pixel_count, min_width, min_height, best_rect);
}

static bool detect_red_candidate_for_early_slowdown(const cv::Mat& frame_bgr,
                                                    const cv::Rect& limit_rect,
                                                    cv::Rect* best_rect)
{
    return detect_red_candidate_in_y_range(
        frame_bgr,
        limit_rect,
        kRecognitionSlowdownMinSearchYInclusive,
        kRecognitionSlowdownMaxSearchYExclusive,
        best_rect,
        130,
        70,
        60,
        24,
        4,
        4);
}

static void detect_lightweight_red_prefilter(const cv::Mat& frame_bgr,
                                             const cv::Rect& limit_rect,
                                             bool collect_debug_geometry,
                                             bool* out_slowdown_red,
                                             cv::Rect* slowdown_rect,
                                             bool* out_recognition_red,
                                             cv::Rect* recognition_rect,
                                             RoiTrackRedPrefilterResult* out_track_prefilter)
{
    if (out_track_prefilter != nullptr)
    {
        *out_track_prefilter = RoiTrackRedPrefilterResult();
    }
    if (out_slowdown_red != nullptr)
    {
        *out_slowdown_red = false;
    }
    if (out_recognition_red != nullptr)
    {
        *out_recognition_red = false;
    }
    if (frame_bgr.empty() || limit_rect.width <= 0 || limit_rect.height <= 0)
    {
        return;
    }

    const int early_slowdown_y_max =
        std::min(kRecognitionSlowdownMaxSearchYExclusive, kRecognitionMinSearchYInclusive);
    RoiTrackRedPrefilterResult track_prefilter;
    if (DetectTrackAwareRedPrefilter(
            frame_bgr,
            kRecognitionSlowdownMinSearchYInclusive,
            early_slowdown_y_max,
            kRecognitionMinSearchYInclusive,
            kRecognitionMaxSearchYExclusive,
            collect_debug_geometry,
            &track_prefilter))
    {
        if (out_track_prefilter != nullptr)
        {
            *out_track_prefilter = track_prefilter;
        }
        const bool has_early_slowdown_red =
            kRecognitionEarlySlowdownEnable && track_prefilter.has_early_marker_red;
        if (out_slowdown_red != nullptr)
        {
            *out_slowdown_red = has_early_slowdown_red;
        }
        if (slowdown_rect != nullptr && has_early_slowdown_red)
        {
            *slowdown_rect = track_prefilter.early_marker_rect;
        }

        const bool has_recognition_red =
            track_prefilter.has_recognition_marker_red ||
            track_prefilter.has_recognition_brick_red;
        if (out_recognition_red != nullptr)
        {
            *out_recognition_red = has_recognition_red;
        }
        if (recognition_rect != nullptr && has_recognition_red)
        {
            *recognition_rect = track_prefilter.has_recognition_marker_red
                ? track_prefilter.recognition_marker_rect
                : track_prefilter.recognition_brick_rect;
        }
        return;
    }

    // 双边线不可用时不退回全帧红扫，避免边界外红色触发和候选框突然放大。
}

static ProbabilityDecisionSummary summarize_probabilities(const std::array<float, kRecognitionMaxClasses>& prob_sum,
                                                          size_t active_class_count,
                                                          int valid_frames)
{
    ProbabilityDecisionSummary summary;
    if (valid_frames <= 0)
    {
        return summary;
    }

    active_class_count = std::min(active_class_count, kRecognitionMaxClasses);
    if (active_class_count == 0)
    {
        return summary;
    }

    std::array<float, kRecognitionMaxClasses> avg_probs = {};
    for (size_t i = 0; i < active_class_count; ++i)
    {
        avg_probs[i] = prob_sum[i] / static_cast<float>(valid_frames);
    }

    int best_index = -1;
    int second_index = -1;
    float best_value = -1.0f;
    float second_value = -1.0f;
    for (int i = 0; i < static_cast<int>(active_class_count); ++i)
    {
        const float value = avg_probs[static_cast<size_t>(i)];
        if (value > best_value)
        {
            second_value = best_value;
            second_index = best_index;
            best_value = value;
            best_index = i;
        }
        else if (value > second_value)
        {
            second_value = value;
            second_index = i;
        }
    }

    summary.top1_index = best_index;
    summary.top2_index = second_index;
    summary.top1_prob = std::max(best_value, 0.0f);
    summary.top2_prob = std::max(second_value, 0.0f);
    summary.margin = std::max(summary.top1_prob - summary.top2_prob, 0.0f);
    return summary;
}

static void copy_roi_timing_to_perf(const RoiExtractionResult& roi_result,
                                    RecognitionChain::PerfSample* perf_sample)
{
    if (perf_sample == nullptr)
    {
        return;
    }

    perf_sample->roi_search_rect_ms = roi_result.timing_search_rect_ms;
    perf_sample->roi_track_boundary_ms = roi_result.timing_track_boundary_ms;
    perf_sample->roi_red_mask_ms = roi_result.timing_red_mask_ms;
    perf_sample->roi_red_band_ms = roi_result.timing_red_band_ms;
    perf_sample->roi_track_classify_ms = roi_result.timing_track_classify_ms;
    perf_sample->roi_build_warp_ms = roi_result.timing_roi_build_warp_ms;
}

} // namespace

bool RecognitionChain::DefaultEnabled()
{
    return (BW_ENABLE_RECOGNITION != 0);
}

// [Recognition Chain Interface] 模型识别运行时接口开关。
// 作用：允许启动参数直接控制是否启用整条模型识别链。
bool RecognitionChain::ParseSwitch(int argc, char** argv, bool default_value)
{
    bool recognition_enabled = default_value;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--recognition")
        {
            recognition_enabled = true;
            continue;
        }
        if (arg == "--no-recognition")
        {
            recognition_enabled = false;
            continue;
        }

        const std::string key = "--recognition=";
        if (arg.compare(0, key.size(), key) == 0)
        {
            bool parsed_value = recognition_enabled;
            if (ParseImageBoolSwitchText(arg.substr(key.size()), &parsed_value))
            {
                recognition_enabled = parsed_value;
            }
            continue;
        }

        if (arg == "--recognition-mode" && i + 1 < argc)
        {
            bool parsed_value = recognition_enabled;
            if (ParseImageBoolSwitchText(argv[i + 1], &parsed_value))
            {
                recognition_enabled = parsed_value;
            }
            ++i;
            continue;
        }
    }

    return recognition_enabled;
}

RecognitionChain::RecognitionChain()
    : enabled_(false),
      logit_bias_(),
      calibration_temperature_(1.0f),
      decision_top1_threshold_(kRecognitionDecisionTop1Threshold),
      decision_margin_threshold_(kRecognitionDecisionMarginThreshold),
      mode_(Mode::NORMAL),
      current_vision_code_(BoardVisionCode::UNKNOWN),
      latched_symbol_code_(BoardVisionCode::INVALID),
      latched_release_deadline_ms_(0),
      latched_release_pending_(false),
      current_blob_area_(0.0),
      recent_red_candidate_until_ms_(0),
      adaptive_decision_pending_(false),
      adaptive_prob_sum_(),
      adaptive_valid_frame_count_(0),
      adaptive_bad_frame_count_(0),
      circle_running_mode_(false),
      circle_marker_quality_pass_frames_(0),
      circle_marker_quality_last_log_ms_(0),
      pending_trigger_roi_valid_(false),
      pending_trigger_roi_(),
      pending_trigger_perf_(),
      last_perf_sample_()
{
}

bool RecognitionChain::Initialize(bool enabled_by_switch)
{
    const std::string configured_model_path = std::string(kRecognitionModelRootDir) + "/cls.onnx";
    const std::string configured_class_path = std::string(kRecognitionModelRootDir) + "/class_names.json";
    const std::string configured_calibration_path = std::string(kRecognitionModelRootDir) + "/deploy_calibration.json";
    const std::string model_path = resolve_runtime_path(configured_model_path);
    const std::string class_path = resolve_runtime_path(configured_class_path);
    const std::string calibration_path = resolve_runtime_path(configured_calibration_path);

    if (!enabled_by_switch)
    {
        enabled_ = false;
        if (kRecognitionTextLog)
        {
            std::cout << "[RECOG] disabled by switch" << std::endl;
        }
        return false;
    }

    if (!file_exists(class_path) || !file_exists(calibration_path) ||
        (kRecognitionManualMlpCompareOnnx && !file_exists(model_path)))
    {
        enabled_ = false;
        std::cerr << "[MLP] deployment files missing: classes=" << class_path
                  << ", calibration=" << calibration_path;
        if (kRecognitionManualMlpCompareOnnx)
        {
            std::cerr << ", compare_model=" << model_path;
        }
        std::cerr << std::endl;
        std::cerr << "[MLP] cwd=" << get_current_working_directory()
                  << ", exe_dir=" << get_executable_directory() << std::endl;
        return false;
    }

    try
    {
        class_names_ = load_class_names_from_json(class_path);
        const std::vector<std::string> expected_names = expected_class_names();
        const std::string class_mismatch =
            describe_class_name_mismatch(class_names_, expected_names);
        if (!class_mismatch.empty())
        {
            throw std::runtime_error(
                std::string("invalid class_names.json: ") + class_mismatch +
                ", path=" + class_path);
        }
        const DeployCalibration calibration = load_deploy_calibration_json(calibration_path);
        if (!calibration.loaded)
        {
            throw std::runtime_error("invalid deploy_calibration.json: " + calibration_path);
        }
        calibration_temperature_ = calibration.temperature;
        logit_bias_ = calibration.logit_bias;
        decision_top1_threshold_ = calibration.decision_top1_threshold;
        decision_margin_threshold_ = calibration.decision_margin_threshold;

        if (kRecognitionManualMlpCompareOnnx)
        {
            net_ = cv::dnn::readNetFromONNX(model_path);
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            if (net_.empty())
            {
                throw std::runtime_error("failed to load ONNX compare model: " + model_path);
            }
            warmup_recognition_net(net_, calibration_temperature_, logit_bias_);
        }
        enabled_ = true;
    }
    catch (const std::exception& e)
    {
        enabled_ = false;
        std::cerr << "[MLP] disabled: " << e.what() << std::endl;
    }

    if (enabled_)
    {
        if (kRecognitionTextLog)
        {
            std::cout << "[MLP] model=" << kRecognitionModelName
                       << ", input=" << kRecognitionModelInputSize << "x" << kRecognitionModelInputSize
                       << ", channels=3(rgb)"
                       << std::endl;
            std::cout << "[MLP] manual_infer=on"
                       << ", compare_onnx=" << (kRecognitionManualMlpCompareOnnx ? "on" : "off");
            if (kRecognitionManualMlpCompareOnnx)
            {
                std::cout << ", compare_model=" << model_path;
            }
            std::cout << std::endl;
            std::cout << "[MLP] classes=" << class_path << std::endl;
            std::cout << "[RECOG] roi_method=" << RoiMethodName(DefaultRoiMethod()) << std::endl;
            std::cout << "[RECOG] decision=adaptive_1_or_2_frame"
                      << ", top1_threshold=" << decision_top1_threshold_
                      << ", margin_threshold=" << decision_margin_threshold_
                      << ", single_high_top1=" << kRecognitionSingleFrameHighConfTop1Threshold
                      << ", single_high_margin=" << kRecognitionSingleFrameHighConfMarginThreshold
                      << std::endl;
            std::cout << "[RECOG] calibration=" << calibration_path
                      << ", temperature=" << calibration_temperature_ << std::endl;
            std::cout << "[RECOG] state map:"
                      << " weapon->w, supply->s, vehicle->v, brick->b, no_result->u, unknown->n" << std::endl;
        }
    }
    else
    {
        if (kRecognitionTextLog)
        {
            std::cout << "[MLP] disabled, waiting for valid deployment files." << std::endl;
        }
    }
    return enabled_;
}

void RecognitionChain::Reset()
{
    mode_ = Mode::NORMAL;
    current_vision_code_ = BoardVisionCode::UNKNOWN;
    latched_symbol_code_ = BoardVisionCode::INVALID;
    latched_release_deadline_ms_ = 0;
    latched_release_pending_ = false;
    current_blob_area_ = 0.0;
    recent_red_candidate_until_ms_ = 0;
    circle_marker_quality_pass_frames_ = 0;
    circle_marker_quality_last_log_ms_ = 0;
    pending_trigger_roi_valid_ = false;
    pending_trigger_roi_ = RoiExtractionResult();
    pending_trigger_perf_ = PerfSample();
    ClearAdaptiveDecision();
    last_perf_sample_ = PerfSample();
}

void RecognitionChain::SetCircleRunningMode(bool active)
{
    if (circle_running_mode_ == active)
    {
        return;
    }
    circle_running_mode_ = active;
    circle_marker_quality_pass_frames_ = 0;
    circle_marker_quality_last_log_ms_ = 0;
}

void RecognitionChain::ClearAdaptiveDecision()
{
    adaptive_decision_pending_ = false;
    adaptive_prob_sum_ = {};
    adaptive_valid_frame_count_ = 0;
    adaptive_bad_frame_count_ = 0;
}

bool RecognitionChain::IsEnabled() const
{
    return enabled_;
}

bool RecognitionChain::IsInRecognitionMode() const
{
    return mode_ == Mode::RECOGNITION;
}

bool RecognitionChain::IsIdleNoTargetState() const
{
    return mode_ == Mode::NORMAL &&
           current_vision_code_ == BoardVisionCode::UNKNOWN &&
           current_blob_area_ < static_cast<double>(BW_RECOG_LOOSE_RED_MIN_AREA);
}

bool RecognitionChain::HasRecentRedCandidate(uint64_t t_ms) const
{
    return t_ms < recent_red_candidate_until_ms_ ||
           current_vision_code_ == BoardVisionCode::NO_RESULT;
}

bool RecognitionChain::IsLatchedHoldingResult() const
{
    return mode_ == Mode::NORMAL && is_success_symbol_code(latched_symbol_code_);
}

BoardVisionCode RecognitionChain::GetCurrentVisionCode() const
{
    return current_vision_code_;
}

double RecognitionChain::GetCurrentBlobArea() const
{
    return current_blob_area_;
}

const RecognitionChain::PerfSample& RecognitionChain::GetLastPerfSample() const
{
    return last_perf_sample_;
}

bool RecognitionChain::HasPendingTriggerRoiForImmediateInference() const
{
    return kRecognitionTriggerFrameInferEnable &&
           pending_trigger_roi_valid_ &&
           mode_ == Mode::RECOGNITION;
}

void RecognitionChain::ProcessPendingTriggerRoi(const cv::Mat& frame_bgr,
                                                uint64_t t_ms,
                                                cv::Mat& view,
                                                bool render_debug)
{
    ProcessRecognitionFrame(frame_bgr, t_ms, view, render_debug);
}

bool RecognitionChain::TryEnterRecognition(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view, bool render_debug)
{
    using steady_clock_t = std::chrono::steady_clock;
    const auto try_begin = steady_clock_t::now();
    last_perf_sample_ = PerfSample();
    last_perf_sample_.try_total_called = true;

    if (!enabled_ || mode_ != Mode::NORMAL)
    {
        last_perf_sample_.try_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - try_begin).count();
        return false;
    }

    // [Recognition Chain Step 2] 运行时 ROI 触发器。
    // 作用：严格按《红带分类与ROI提取流程.md》执行一次完整红带提取与 ROI 构造。
    if (render_debug)
    {
        view = frame_bgr.clone();
        draw_trigger_search_info(view);
    }
    else
    {
        view.release();
    }

    RoiTrackRedPrefilterResult lightweight_track_prefilter;
    if (kRecognitionLightweightRedPrefilterEnable &&
        !is_success_symbol_code(latched_symbol_code_))
    {
        const auto prefilter_begin = steady_clock_t::now();
        const cv::Rect full_frame_rect(0, 0, frame_bgr.cols, frame_bgr.rows);
        cv::Rect lightweight_slowdown_rect;
        cv::Rect lightweight_recognition_rect;
        bool has_lightweight_slowdown_red = false;
        bool has_lightweight_recognition_red = false;
        detect_lightweight_red_prefilter(
            frame_bgr,
            full_frame_rect,
            render_debug,
            &has_lightweight_slowdown_red,
            &lightweight_slowdown_rect,
            &has_lightweight_recognition_red,
            &lightweight_recognition_rect,
            &lightweight_track_prefilter);
        last_perf_sample_.ultra_precheck_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - prefilter_begin).count();
        last_perf_sample_.ultra_precheck_called = true;

        if (has_lightweight_slowdown_red ||
            lightweight_track_prefilter.has_recognition_marker_red)
        {
            recent_red_candidate_until_ms_ = t_ms + kRecognitionRecentCandidateHoldMs;
        }
        if (!lightweight_track_prefilter.has_track_boundaries)
        {
            recent_red_candidate_until_ms_ = 0;
        }

        if (!has_lightweight_recognition_red)
        {
            circle_marker_quality_pass_frames_ = 0;
            const bool hold_recent_u =
                lightweight_track_prefilter.has_track_boundaries &&
                t_ms < recent_red_candidate_until_ms_;
            const bool output_u = has_lightweight_slowdown_red || hold_recent_u;
            current_blob_area_ = has_lightweight_slowdown_red
                ? static_cast<double>(lightweight_slowdown_rect.area())
                : 0.0;
            current_vision_code_ = output_u
                ? BoardVisionCode::NO_RESULT
                : BoardVisionCode::UNKNOWN;
            latched_symbol_code_ = BoardVisionCode::INVALID;
            latched_release_pending_ = false;
            latched_release_deadline_ms_ = 0;
            pending_trigger_roi_valid_ = false;
            pending_trigger_roi_ = RoiExtractionResult();
            pending_trigger_perf_ = PerfSample();
            ClearAdaptiveDecision();
            if (render_debug)
            {
                if (!lightweight_track_prefilter.track_left_boundary.empty())
                {
                    cv::polylines(
                        view,
                        std::vector<std::vector<cv::Point>>(1, lightweight_track_prefilter.track_left_boundary),
                        false,
                        cv::Scalar(255, 0, 0),
                        2,
                        cv::LINE_AA);
                }
                if (!lightweight_track_prefilter.track_right_boundary.empty())
                {
                    cv::polylines(
                        view,
                        std::vector<std::vector<cv::Point>>(1, lightweight_track_prefilter.track_right_boundary),
                        false,
                        cv::Scalar(0, 255, 255),
                        2,
                        cv::LINE_AA);
                }
                if (lightweight_track_prefilter.track_region_polygon.size() >= 4)
                {
                    cv::polylines(
                        view,
                        std::vector<std::vector<cv::Point>>(1, lightweight_track_prefilter.track_region_polygon),
                        true,
                        cv::Scalar(255, 0, 255),
                        1,
                        cv::LINE_AA);
                }

                if (has_lightweight_slowdown_red)
                {
                    cv::rectangle(view, lightweight_slowdown_rect, cv::Scalar(0, 255, 255), 2);
                    cv::putText(view, "EARLY RED ONLY -> u", cv::Point(16, 112),
                                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
                }
                else if (hold_recent_u)
                {
                    cv::putText(view, "RED LOST HOLD -> u", cv::Point(16, 112),
                                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
                }
            }
            last_perf_sample_.try_total_ms =
                std::chrono::duration<double, std::milli>(steady_clock_t::now() - try_begin).count();
            return false;
        }
    }

    const RoiMethod roi_method = DefaultRoiMethod();
    const auto extract_begin = steady_clock_t::now();
    RoiExtractionResult trigger_roi =
        ExtractRotatedRoi(
            frame_bgr,
            kRecognitionModelInputSize,
            roi_method,
            render_debug,
            lightweight_track_prefilter.has_track_boundaries
                ? &lightweight_track_prefilter
                : nullptr);
    const auto extract_end = steady_clock_t::now();
    last_perf_sample_.extract_roi_ms =
        std::chrono::duration<double, std::milli>(extract_end - extract_begin).count();
    last_perf_sample_.extract_roi_called = true;
    copy_roi_timing_to_perf(trigger_roi, &last_perf_sample_);
    if (render_debug)
    {
        DrawRoiDebugOverlay(view, trigger_roi);
        draw_roi_preview_inset(view, trigger_roi.roi_bgr);
    }
    current_blob_area_ = roi_observed_red_area(trigger_roi);
    const bool valid_marker_red_observation = roi_is_valid_marker_red_observation(trigger_roi);
    if (valid_marker_red_observation && roi_has_non_noise_red(trigger_roi))
    {
        recent_red_candidate_until_ms_ = t_ms + kRecognitionRecentCandidateHoldMs;
    }
    const bool symbol_candidate_visible = roi_is_visible_sign_candidate(trigger_roi);
    const bool holdable_sign_red_visible = roi_should_hold_success_latch(trigger_roi);
    cv::Rect slowdown_red_rect;
    bool has_slowdown_red_candidate = false;
    if (kRecognitionEarlySlowdownEnable && symbol_candidate_visible && trigger_roi.has_search_rect)
    {
        has_slowdown_red_candidate =
            detect_red_candidate_for_early_slowdown(frame_bgr, trigger_roi.search_rect, &slowdown_red_rect);
        if (has_slowdown_red_candidate)
        {
            recent_red_candidate_until_ms_ = t_ms + kRecognitionRecentCandidateHoldMs;
            current_vision_code_ = BoardVisionCode::NO_RESULT;
            if (render_debug)
            {
                cv::rectangle(view, slowdown_red_rect, cv::Scalar(0, 255, 255), 2);
                cv::putText(view, "EARLY RED -> u", cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                            0.60, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
        }
    }
    if (trigger_roi.target_type == "roadblock" || trigger_roi.status == "roadblock")
    {
        circle_marker_quality_pass_frames_ = 0;
        current_vision_code_ = brick_code_from_roi_result(trigger_roi, frame_bgr.cols);
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;
        pending_trigger_roi_valid_ = false;
        pending_trigger_roi_ = RoiExtractionResult();
        pending_trigger_perf_ = PerfSample();
        ClearAdaptiveDecision();
        if (render_debug)
        {
            const std::string brick_text = red_observation_text(trigger_roi, frame_bgr.cols);
            cv::putText(view, brick_text, cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                        0.65, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
        last_perf_sample_.try_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - try_begin).count();
        return false;
    }
    if (is_success_symbol_code(latched_symbol_code_))
    {
        circle_marker_quality_pass_frames_ = 0;
        current_vision_code_ = latched_symbol_code_;
        if (holdable_sign_red_visible)
        {
            if (latched_release_pending_)
            {
                if (kRecognitionTextLog)
                {
                    std::cout << "[RECOG] sign visible again, keep latched result="
                              << vision_code_text(latched_symbol_code_) << std::endl;
                }
            }
            latched_release_pending_ = false;
            latched_release_deadline_ms_ = 0;
            if (render_debug)
            {
                cv::putText(view,
                            !symbol_candidate_visible
                                ? "STABLE RESULT HOLD (RED)"
                                : "STABLE RESULT HOLD",
                            cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                            0.65, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            }
            last_perf_sample_.try_total_ms =
                std::chrono::duration<double, std::milli>(steady_clock_t::now() - try_begin).count();
            return false;
        }

        if (!latched_release_pending_)
        {
            latched_release_pending_ = true;
            latched_release_deadline_ms_ = t_ms + BW_RECOG_SIGN_LOSS_HOLD_MS;
            if (kRecognitionTextLog)
            {
                std::cout << "[RECOG] sign lost, hold latched result="
                          << vision_code_text(latched_symbol_code_)
                          << " for " << BW_RECOG_SIGN_LOSS_HOLD_MS << " ms" << std::endl;
            }
        }

        if (t_ms < latched_release_deadline_ms_)
        {
            const uint64_t remaining_ms = latched_release_deadline_ms_ - t_ms;
            std::ostringstream hold_text;
            hold_text << "SIGN LOST HOLD "
                      << vision_code_text(latched_symbol_code_)
                      << " " << remaining_ms << " ms";
            if (render_debug)
            {
                cv::putText(view, hold_text.str(), cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                            0.62, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
            last_perf_sample_.try_total_ms =
                std::chrono::duration<double, std::milli>(steady_clock_t::now() - try_begin).count();
            return false;
        }

        if (kRecognitionTextLog)
        {
            std::cout << "[RECOG] sign loss timeout, release latched result="
                      << vision_code_text(latched_symbol_code_)
                      << " -> n" << std::endl;
        }
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;
        current_vision_code_ = BoardVisionCode::UNKNOWN;
        if (render_debug)
        {
            cv::putText(view, "SIGN LOST -> n", cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                        0.65, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
        }
        last_perf_sample_.try_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - try_begin).count();
        return false;
    }

    if (trigger_roi.status == "miss")
    {
        circle_marker_quality_pass_frames_ = 0;
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;
        current_vision_code_ = fallback_code_from_roi_result(trigger_roi, frame_bgr.cols);
        if (current_vision_code_ == BoardVisionCode::UNKNOWN &&
            (has_slowdown_red_candidate || t_ms < recent_red_candidate_until_ms_))
        {
            current_vision_code_ = BoardVisionCode::NO_RESULT;
        }
        const std::string red_text = red_observation_text(trigger_roi, frame_bgr.cols);
        const std::string reject_text = roi_reject_reason_text(trigger_roi);
        if (kRecognitionVerboseLog &&
            (roi_has_non_noise_red(trigger_roi) ||
             trigger_roi.has_prewhite_max_red_contour_area ||
             trigger_roi.white_crop_clipped))
        {
            if (current_vision_code_ == BoardVisionCode::NO_RESULT)
            {
                std::cout << "[RECOG] IPM PASS -> ROI NOT READY: area="
                          << std::fixed << std::setprecision(1)
                          << current_blob_area_ << std::endl;
            }
            else if (is_brick_vision_code(current_vision_code_))
            {
                std::cout << "[RECOG] brick detected: side="
                          << roi_blob_side_lower(trigger_roi, frame_bgr.cols)
                          << ", area=" << std::fixed << std::setprecision(1)
                          << current_blob_area_ << std::endl;
            }
            else
            {
                std::cout << "[RECOG] red present but not brick: area="
                          << std::fixed << std::setprecision(1)
                          << current_blob_area_ << std::endl;
            }
            std::cout << "[RECOG] miss debug: " << reject_text << std::endl;
        }
        if (render_debug)
        {
            cv::putText(view, red_text, cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                        0.65, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            cv::putText(view, reject_text, cv::Point(16, 140), cv::FONT_HERSHEY_SIMPLEX,
                        0.48, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            if (current_vision_code_ == BoardVisionCode::NO_RESULT && has_slowdown_red_candidate)
            {
                cv::putText(view, "EARLY RED -> u", cv::Point(16, 168), cv::FONT_HERSHEY_SIMPLEX,
                            0.55, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
            else if (current_vision_code_ == BoardVisionCode::NO_RESULT)
            {
                cv::putText(view, "RED HOLD -> u", cv::Point(16, 168), cv::FONT_HERSHEY_SIMPLEX,
                            0.55, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
        }
        last_perf_sample_.try_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - try_begin).count();
        return false;
    }

    if (trigger_roi.status != "rotated_roi")
    {
        circle_marker_quality_pass_frames_ = 0;
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;
        current_vision_code_ = fallback_code_from_roi_result(trigger_roi, frame_bgr.cols);
        if (current_vision_code_ == BoardVisionCode::UNKNOWN &&
            (has_slowdown_red_candidate || t_ms < recent_red_candidate_until_ms_))
        {
            current_vision_code_ = BoardVisionCode::NO_RESULT;
        }
        const std::string red_text = red_observation_text(trigger_roi, frame_bgr.cols);
        const std::string reject_text = roi_reject_reason_text(trigger_roi);
        static uint64_t last_reject_log_ms = 0;
        if (kRecognitionVerboseLog &&
            t_ms >= last_reject_log_ms + kRecognitionTriggerRejectLogIntervalMs)
        {
            last_reject_log_ms = t_ms;
            if (roi_has_non_noise_red(trigger_roi))
            {
                if (current_vision_code_ == BoardVisionCode::NO_RESULT)
                {
                    std::cout << "[RECOG] IPM PASS -> ROI NOT READY: area="
                              << std::fixed << std::setprecision(1)
                              << current_blob_area_ << std::endl;
                }
                else if (is_brick_vision_code(current_vision_code_))
                {
                    std::cout << "[RECOG] brick detected: side="
                              << roi_blob_side_lower(trigger_roi, frame_bgr.cols)
                              << ", area=" << std::fixed << std::setprecision(1)
                              << current_blob_area_ << std::endl;
                }
                else
                {
                    std::cout << "[RECOG] red present but not brick: area="
                              << std::fixed << std::setprecision(1)
                              << current_blob_area_ << std::endl;
                }
            }
            std::cout << "[RECOG] red roi rejected: " << reject_text;
            if (trigger_roi.has_quad_red_fill && trigger_roi.has_quad_core_fill)
            {
                std::cout << ", quad_fill=(" << std::fixed << std::setprecision(4)
                          << trigger_roi.quad_red_fill << ", " << trigger_roi.quad_core_fill << ")";
            }
            std::cout << std::endl;
        }
        if (render_debug)
        {
            cv::putText(view, red_text, cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                        0.65, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            cv::putText(view, reject_text, cv::Point(16, 140), cv::FONT_HERSHEY_SIMPLEX,
                        0.50, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            if (current_vision_code_ == BoardVisionCode::NO_RESULT && has_slowdown_red_candidate)
            {
                cv::putText(view, "EARLY RED -> u", cv::Point(16, 168), cv::FONT_HERSHEY_SIMPLEX,
                            0.55, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            }
            else if (current_vision_code_ == BoardVisionCode::NO_RESULT)
            {
                cv::putText(view, "RED HOLD -> u", cv::Point(16, 168), cv::FONT_HERSHEY_SIMPLEX,
                            0.55, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            }
        }
        last_perf_sample_.try_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - try_begin).count();
        return false;
    }

    if (kRecognitionCircleMarkerQualityGateEnable && circle_running_mode_)
    {
        const CircleMarkerQualityObservation circle_quality =
            evaluate_circle_marker_quality(trigger_roi, frame_bgr.cols);
        const int stable_frames_required =
            std::max(1, BW_RECOG_CIRCLE_TRIGGER_STABLE_FRAMES);
        bool quality_passed = false;
        if (!circle_quality.position_passed)
        {
            circle_marker_quality_pass_frames_ = 0;
        }
        else if (circle_quality.immediate)
        {
            quality_passed = true;
        }
        else
        {
            ++circle_marker_quality_pass_frames_;
            quality_passed =
                circle_marker_quality_pass_frames_ >= stable_frames_required;
        }

        if (!quality_passed)
        {
            current_vision_code_ = BoardVisionCode::NO_RESULT;
            latched_symbol_code_ = BoardVisionCode::INVALID;
            latched_release_pending_ = false;
            latched_release_deadline_ms_ = 0;
            pending_trigger_roi_valid_ = false;
            pending_trigger_roi_ = RoiExtractionResult();
            pending_trigger_perf_ = PerfSample();
            ClearAdaptiveDecision();

            const bool should_log =
                kRecognitionResultLog &&
                (circle_marker_quality_last_log_ms_ == 0 ||
                 t_ms >= circle_marker_quality_last_log_ms_ +
                             kRecognitionTriggerRejectLogIntervalMs);
            if (should_log)
            {
                circle_marker_quality_last_log_ms_ = t_ms;
                std::cout << "[环岛识别质量] 保持u"
                          << ", 原因=" << circle_quality.reason
                          << ", bottom_y=" << circle_quality.bottom_y
                          << ", center_x=" << std::fixed << std::setprecision(1)
                          << circle_quality.center_x
                          << ", x_ratio=" << std::setprecision(3)
                          << circle_quality.center_x_ratio
                          << ", 允许范围=[" << circle_quality.min_x_ratio
                          << "," << circle_quality.max_x_ratio << "]"
                          << ", area=" << std::setprecision(1)
                          << circle_quality.area
                          << ", box=" << circle_quality.width
                          << "x" << circle_quality.height
                          << ", 连续帧=" << circle_marker_quality_pass_frames_
                          << "/" << stable_frames_required
                          << std::endl;
            }
            if (render_debug)
            {
                cv::putText(view, "CIRCLE QUALITY HOLD -> u", cv::Point(16, 84),
                            cv::FONT_HERSHEY_SIMPLEX, 0.62,
                            cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
                std::ostringstream quality_text;
                quality_text << "bottom=" << circle_quality.bottom_y
                             << " x=" << std::fixed << std::setprecision(2)
                             << circle_quality.center_x_ratio
                             << " gate=[" << circle_quality.min_x_ratio
                             << "," << circle_quality.max_x_ratio << "]"
                             << " stable=" << circle_marker_quality_pass_frames_
                             << "/" << stable_frames_required;
                cv::putText(view, quality_text.str(), cv::Point(16, 168),
                            cv::FONT_HERSHEY_SIMPLEX, 0.48,
                            cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
            }
            last_perf_sample_.try_total_ms =
                std::chrono::duration<double, std::milli>(
                    steady_clock_t::now() - try_begin).count();
            return false;
        }

        if (kRecognitionResultLog)
        {
            std::cout << "[环岛识别质量] 通过"
                      << ", 模式=" << (circle_quality.immediate ? "近端立即" : "连续稳定")
                      << ", bottom_y=" << circle_quality.bottom_y
                      << ", center_x=" << std::fixed << std::setprecision(1)
                      << circle_quality.center_x
                      << ", x_ratio=" << std::setprecision(3)
                      << circle_quality.center_x_ratio
                      << ", 允许范围=[" << circle_quality.min_x_ratio
                      << "," << circle_quality.max_x_ratio << "]"
                      << ", area=" << std::setprecision(1)
                      << circle_quality.area
                      << ", box=" << circle_quality.width
                      << "x" << circle_quality.height
                      << std::endl;
        }
        circle_marker_quality_pass_frames_ = 0;
        circle_marker_quality_last_log_ms_ = 0;
    }
    else
    {
        circle_marker_quality_pass_frames_ = 0;
    }

    // [Recognition Chain Step 3] 进入识别态。
    // 作用：marker ROI 一旦构造成功，就直接进入分类阶段。
    mode_ = Mode::RECOGNITION;
    current_vision_code_ = BoardVisionCode::NO_RESULT;
    latched_release_pending_ = false;
    latched_release_deadline_ms_ = 0;
    ClearAdaptiveDecision();
    pending_trigger_roi_valid_ = kRecognitionTriggerFrameInferEnable;
    if (pending_trigger_roi_valid_)
    {
        pending_trigger_roi_ = trigger_roi;
        pending_trigger_perf_ = last_perf_sample_;
    }
    else
    {
        pending_trigger_roi_ = RoiExtractionResult();
        pending_trigger_perf_ = PerfSample();
    }
    if (kRecognitionTextLog)
    {
        std::cout << "[RECOG] sign board accepted: entering recognition"
                  << ", side=" << roi_blob_side_lower(trigger_roi, frame_bgr.cols)
                  << ", area=" << std::fixed << std::setprecision(1) << current_blob_area_
                  << std::endl;
    }

    if (render_debug)
    {
        cv::putText(view, "TRIGGER -> RECOGNITION", cv::Point(16, 84), cv::FONT_HERSHEY_SIMPLEX,
                    0.65, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }
    last_perf_sample_.try_total_ms =
        std::chrono::duration<double, std::milli>(steady_clock_t::now() - try_begin).count();
    if (pending_trigger_roi_valid_)
    {
        pending_trigger_perf_ = last_perf_sample_;
    }
    return true;
}

void RecognitionChain::ProcessRecognitionFrame(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view, bool render_debug)
{
    using steady_clock_t = std::chrono::steady_clock;
    const auto process_begin = steady_clock_t::now();
    const bool use_pending_trigger_roi = HasPendingTriggerRoiForImmediateInference();
    RoiExtractionResult pending_trigger_roi;
    PerfSample pending_trigger_perf;
    if (use_pending_trigger_roi)
    {
        pending_trigger_roi = pending_trigger_roi_;
        pending_trigger_perf = pending_trigger_perf_;
        pending_trigger_roi_valid_ = false;
        pending_trigger_roi_ = RoiExtractionResult();
        pending_trigger_perf_ = PerfSample();
    }

    last_perf_sample_ = use_pending_trigger_roi ? pending_trigger_perf : PerfSample();
    last_perf_sample_.process_recog_total_called = true;

    if (!enabled_ || mode_ != Mode::RECOGNITION)
    {
        ClearAdaptiveDecision();
        if (render_debug)
        {
            view = frame_bgr.clone();
        }
        else
        {
            view.release();
        }
        last_perf_sample_.process_recog_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
        return;
    }

    // [Recognition Chain Step 4] 识别态自适应 1/2 帧推理。
    // 作用：达到门槛时单帧输出，否则保留首帧概率等待第二个有效推理帧。
    if (render_debug)
    {
        view = frame_bgr.clone();
    }
    else
    {
        view.release();
    }
    RoiMethod roi_method = DefaultRoiMethod();
    RoiExtractionResult roi_result;
    if (use_pending_trigger_roi)
    {
        roi_result = pending_trigger_roi;
        roi_method = roi_result.roi_method;
    }
    else
    {
        const auto extract_begin = steady_clock_t::now();
        roi_result = ExtractRotatedRoi(frame_bgr, kRecognitionModelInputSize, roi_method, render_debug);
        const auto extract_end = steady_clock_t::now();
        last_perf_sample_.extract_roi_ms =
            std::chrono::duration<double, std::milli>(extract_end - extract_begin).count();
        last_perf_sample_.extract_roi_called = true;
        copy_roi_timing_to_perf(roi_result, &last_perf_sample_);
    }
    if (render_debug)
    {
        DrawRoiDebugOverlay(view, roi_result);
        draw_roi_preview_inset(view, roi_result.roi_bgr);
        draw_trigger_search_info(view);
    }
    current_blob_area_ = roi_observed_red_area(roi_result);
    if (roi_is_valid_marker_red_observation(roi_result) && roi_has_non_noise_red(roi_result))
    {
        recent_red_candidate_until_ms_ = t_ms + kRecognitionRecentCandidateHoldMs;
    }

    const size_t active_class_count = recognition_accum_class_count(class_names_.size());
    const auto force_cached_first_frame_decision =
        [&](const char* reason) -> bool {
            const ProbabilityDecisionSummary cached_summary =
                summarize_probabilities(
                    adaptive_prob_sum_,
                    active_class_count,
                    adaptive_valid_frame_count_);
            const bool has_valid_top1 =
                adaptive_valid_frame_count_ > 0 &&
                cached_summary.top1_index >= 0 &&
                cached_summary.top1_index < static_cast<int>(active_class_count);
            if (!has_valid_top1)
            {
                return false;
            }

            const TargetClass target = static_cast<TargetClass>(
                runtime_decision_target_code(cached_summary.top1_index, class_names_));
            const BoardVisionCode final_code =
                vision_code_from_target_code(static_cast<uint8_t>(target));
            if (final_code == BoardVisionCode::INVALID)
            {
                return false;
            }

            current_vision_code_ = final_code;
            latched_symbol_code_ = is_success_symbol_code(final_code)
                ? final_code
                : BoardVisionCode::INVALID;
            latched_release_pending_ = false;
            latched_release_deadline_ms_ = 0;

            if (kRecognitionResultLog)
            {
                std::cout << "[RECOG] result="
                          << runtime_decision_label(cached_summary.top1_index, class_names_)
                          << ", valid_frames=" << adaptive_valid_frame_count_
                          << ", mode=cached_first_frame_fallback"
                          << ", top1_prob=" << std::fixed << std::setprecision(4)
                          << cached_summary.top1_prob
                          << ", margin=" << cached_summary.margin
                          << ", reason=" << reason
                          << std::endl;
                std::cout << "[RECOG] state_out="
                          << vision_code_text(current_vision_code_)
                          << ", blob_area=" << std::fixed << std::setprecision(1)
                          << current_blob_area_
                          << std::endl;
            }

            ClearAdaptiveDecision();
            mode_ = Mode::NORMAL;
            last_perf_sample_.process_recog_total_ms =
                std::chrono::duration<double, std::milli>(
                    steady_clock_t::now() - process_begin).count();
            return true;
        };

    const auto keep_adaptive_wait_on_bad_roi =
        [&](const char* roi_status, const std::string& reject_text) -> bool {
            if (!adaptive_decision_pending_)
            {
                return false;
            }

            ++adaptive_bad_frame_count_;
            if (adaptive_bad_frame_count_ >= kRecognitionAdaptiveTwoFrameMaxBadFrames)
            {
                const std::string fallback_reason =
                    std::string("second_frame_roi_timeout:") +
                    roi_status + ":" + reject_text;
                return force_cached_first_frame_decision(fallback_reason.c_str());
            }

            current_vision_code_ = BoardVisionCode::NO_RESULT;
            latched_symbol_code_ = BoardVisionCode::INVALID;
            latched_release_pending_ = false;
            latched_release_deadline_ms_ = 0;
            if (render_debug)
            {
                cv::putText(view, "RECOG keep waiting 2nd frame",
                            cv::Point(16, 84), cv::FONT_HERSHEY_SIMPLEX,
                            0.65, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
            if (kRecognitionResultLog)
            {
                std::cout << "[RECOG] result=wait_second_frame_roi_not_ready"
                          << ", bad_frames=" << adaptive_bad_frame_count_
                          << "/" << kRecognitionAdaptiveTwoFrameMaxBadFrames
                          << ", roi_status=" << roi_status
                          << ", reason=" << reject_text
                          << std::endl;
                std::cout << "[RECOG] state_out=" << vision_code_text(current_vision_code_)
                          << ", blob_area=" << std::fixed << std::setprecision(1) << current_blob_area_
                          << std::endl;
            }
            last_perf_sample_.process_recog_total_ms =
                std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
            return true;
        };

    if (roi_result.status == "miss")
    {
        current_vision_code_ = fallback_code_from_roi_result(roi_result, frame_bgr.cols);
        if (current_vision_code_ == BoardVisionCode::UNKNOWN &&
            t_ms < recent_red_candidate_until_ms_)
        {
            current_vision_code_ = BoardVisionCode::NO_RESULT;
        }
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;
        const std::string red_text = red_observation_text(roi_result, frame_bgr.cols);
        const std::string reject_text = roi_reject_reason_text(roi_result);
        if (render_debug)
        {
            cv::putText(view, red_text, cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                        0.65, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            cv::putText(view, reject_text, cv::Point(16, 140), cv::FONT_HERSHEY_SIMPLEX,
                        0.48, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            if (current_vision_code_ == BoardVisionCode::NO_RESULT)
            {
                cv::putText(view, "RED LOST HOLD -> u", cv::Point(16, 168),
                            cv::FONT_HERSHEY_SIMPLEX, 0.55,
                            cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
        }
        if (kRecognitionVerboseLog)
        {
            std::cout << "[RECOG] recognition exit: miss, fallback_state="
                      << vision_code_text(current_vision_code_)
                      << ", " << reject_text << std::endl;
        }
        if (keep_adaptive_wait_on_bad_roi("miss", reject_text))
        {
            return;
        }
        ClearAdaptiveDecision();
        mode_ = Mode::NORMAL;
        last_perf_sample_.process_recog_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
        return;
    }

    if (roi_result.status != "rotated_roi")
    {
        current_vision_code_ = fallback_code_from_roi_result(roi_result, frame_bgr.cols);
        if (current_vision_code_ == BoardVisionCode::UNKNOWN &&
            t_ms < recent_red_candidate_until_ms_)
        {
            current_vision_code_ = BoardVisionCode::NO_RESULT;
        }
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;
        const std::string red_text = red_observation_text(roi_result, frame_bgr.cols);
        const std::string reject_text = roi_reject_reason_text(roi_result);
        if (render_debug)
        {
            cv::putText(view, red_text, cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                        0.65, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            cv::putText(view, reject_text, cv::Point(16, 140), cv::FONT_HERSHEY_SIMPLEX,
                        0.50, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            if (current_vision_code_ == BoardVisionCode::NO_RESULT)
            {
                cv::putText(view, "RED LOST HOLD -> u", cv::Point(16, 168),
                            cv::FONT_HERSHEY_SIMPLEX, 0.55,
                            cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
        }
        if (kRecognitionVerboseLog)
        {
            std::cout << "[RECOG] recognition exit: " << reject_text
                      << ", fallback_state=" << vision_code_text(current_vision_code_);
            if (roi_has_non_noise_red(roi_result))
            {
                std::cout << ", area=" << std::fixed << std::setprecision(1) << current_blob_area_;
                if (is_brick_vision_code(current_vision_code_))
                {
                    std::cout << ", side=" << roi_blob_side_lower(roi_result, frame_bgr.cols);
                }
            }
            std::cout << std::endl;
        }
        if (keep_adaptive_wait_on_bad_roi(roi_result.status.c_str(), reject_text))
        {
            return;
        }
        ClearAdaptiveDecision();
        mode_ = Mode::NORMAL;
        last_perf_sample_.process_recog_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
        return;
    }

    current_vision_code_ = BoardVisionCode::NO_RESULT;

    const auto classify_begin = steady_clock_t::now();
    last_perf_sample_.classify_total_called = true;

    const auto infer_begin = steady_clock_t::now();
    RoiClassificationTiming cls_timing;
    const RoiClassificationResult cls =
        classify_roi_index(net_, roi_result.roi_bgr, calibration_temperature_, logit_bias_, &cls_timing);
    const auto infer_end = steady_clock_t::now();
    const double infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_begin).count();
    last_perf_sample_.onnx_infer_ms = infer_ms;
    last_perf_sample_.onnx_preprocess_ms = cls_timing.preprocess_ms;
    last_perf_sample_.onnx_set_input_ms = cls_timing.set_input_ms;
    last_perf_sample_.onnx_forward_ms = cls_timing.forward_ms;
    last_perf_sample_.onnx_postprocess_ms = cls_timing.postprocess_ms;
    last_perf_sample_.onnx_infer_called = true;

    std::array<float, kRecognitionMaxClasses> frame_prob_sum = {};
    int frame_valid_count = 0;
    std::string frame_pred_name = "invalid";
    if (cls.predicted_index >= 0 && cls.predicted_index < static_cast<int>(class_names_.size()))
    {
        accumulate_probabilities_for_runtime_decision(cls, class_names_, frame_prob_sum);
        frame_valid_count = 1;
        frame_pred_name = class_names_[cls.predicted_index];
        if (render_debug)
        {
            cv::putText(view, std::string("pred: ") + frame_pred_name, cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                        0.65, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }
        std::ostringstream infer_info;
        infer_info << "infer: " << std::fixed << std::setprecision(2) << infer_ms << " ms";
        if (render_debug)
        {
            cv::putText(view, infer_info.str(), cv::Point(16, 168), cv::FONT_HERSHEY_SIMPLEX,
                        0.55, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
        }
        std::ostringstream prob_info;
        prob_info << "frame_probs:";
        for (size_t i = 0; i < active_class_count; ++i)
        {
            prob_info << (i == 0 ? " " : "/")
                      << std::fixed << std::setprecision(2)
                      << frame_prob_sum[i];
        }
        if (render_debug)
        {
            cv::putText(view, prob_info.str(), cv::Point(16, 196), cv::FONT_HERSHEY_SIMPLEX,
                        0.52, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
        }
        const ProbabilityDecisionSummary frame_summary =
            summarize_probabilities(frame_prob_sum, active_class_count, frame_valid_count);
        std::ostringstream top_info;
        top_info << "frame_top1=" << runtime_decision_label(frame_summary.top1_index, class_names_)
                 << " p=" << std::fixed << std::setprecision(2) << frame_summary.top1_prob
                 << " m=" << frame_summary.margin;
        if (render_debug)
        {
            cv::putText(view, top_info.str(), cv::Point(16, 224), cv::FONT_HERSHEY_SIMPLEX,
                        0.52, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
        }
        if (kRecognitionVerboseLog)
        {
            std::cout << "[RECOG] frame_pred=" << frame_pred_name
                      << ", infer_ms=" << std::fixed << std::setprecision(2) << infer_ms
                      << ", roi_method=" << RoiMethodName(roi_method) << std::endl;
        }
    }
    last_perf_sample_.classify_total_ms =
        std::chrono::duration<double, std::milli>(steady_clock_t::now() - classify_begin).count();

    const ProbabilityDecisionSummary frame_summary =
        summarize_probabilities(frame_prob_sum, active_class_count, frame_valid_count);
    const bool was_waiting_second_frame = adaptive_decision_pending_;
    if (frame_valid_count > 0)
    {
        adaptive_bad_frame_count_ = 0;
    }
    if (was_waiting_second_frame && frame_valid_count <= 0)
    {
        ++adaptive_bad_frame_count_;
        if (adaptive_bad_frame_count_ < kRecognitionAdaptiveTwoFrameMaxBadFrames)
        {
            current_vision_code_ = BoardVisionCode::NO_RESULT;
            latched_symbol_code_ = BoardVisionCode::INVALID;
            latched_release_pending_ = false;
            latched_release_deadline_ms_ = 0;
            if (kRecognitionResultLog)
            {
                std::cout << "[RECOG] result=wait_second_frame_invalid_model_output"
                          << ", bad_frames=" << adaptive_bad_frame_count_
                          << "/" << kRecognitionAdaptiveTwoFrameMaxBadFrames
                          << ", reason=invalid_model_output"
                          << std::endl;
                std::cout << "[RECOG] state_out=" << vision_code_text(current_vision_code_)
                          << ", blob_area=" << std::fixed << std::setprecision(1) << current_blob_area_
                          << std::endl;
            }
            last_perf_sample_.process_recog_total_ms =
                std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
            return;
        }
        if (force_cached_first_frame_decision("second_frame_invalid_model_output_timeout"))
        {
            return;
        }
    }
    const bool high_conf_single_frame =
        frame_valid_count > 0 &&
        frame_summary.top1_prob >= kRecognitionSingleFrameHighConfTop1Threshold &&
        frame_summary.margin >= kRecognitionSingleFrameHighConfMarginThreshold;
    const bool should_wait_second_frame =
        kRecognitionAdaptiveTwoFrameEnable &&
        !was_waiting_second_frame &&
        frame_valid_count > 0 &&
        !high_conf_single_frame;

    if (should_wait_second_frame)
    {
        adaptive_decision_pending_ = true;
        adaptive_prob_sum_ = frame_prob_sum;
        adaptive_valid_frame_count_ = frame_valid_count;
        adaptive_bad_frame_count_ = 0;
        current_vision_code_ = BoardVisionCode::NO_RESULT;
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;

        if (render_debug)
        {
            cv::putText(view, "RECOG wait 2nd frame", cv::Point(16, 84), cv::FONT_HERSHEY_SIMPLEX,
                        0.75, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            cv::putText(view, "result: wait_second_frame", cv::Point(16, 252), cv::FONT_HERSHEY_SIMPLEX,
                        0.70, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        }

        if (kRecognitionResultLog)
        {
            std::cout << "[RECOG] result=wait_second_frame"
                      << ", valid_frames=" << frame_valid_count
                      << ", infer_ms=" << std::fixed << std::setprecision(2) << infer_ms
                      << ", forward_ms=" << cls_timing.forward_ms
                      << ", cls_ms=" << last_perf_sample_.classify_total_ms
                      << ", top1_prob=" << std::fixed << std::setprecision(4) << frame_summary.top1_prob
                      << ", margin=" << frame_summary.margin
                      << ", reason=low_conf_wait_second_frame"
                      << std::endl;
            std::cout << "[RECOG] state_out=" << vision_code_text(current_vision_code_)
                      << ", blob_area=" << std::fixed << std::setprecision(1) << current_blob_area_
                      << std::endl;
        }

        last_perf_sample_.process_recog_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
        return;
    }

    std::array<float, kRecognitionMaxClasses> decision_prob_sum = frame_prob_sum;
    int decision_valid_count = frame_valid_count;
    if (was_waiting_second_frame)
    {
        if (frame_valid_count > 0)
        {
            for (size_t i = 0; i < kRecognitionMaxClasses; ++i)
            {
                decision_prob_sum[i] += adaptive_prob_sum_[i];
            }
            decision_valid_count += adaptive_valid_frame_count_;
        }
        else
        {
            decision_prob_sum = {};
            decision_valid_count = 0;
        }
    }

    std::ostringstream vote_info;
    vote_info << (was_waiting_second_frame ? "RECOG two-frame avg" : "RECOG high-conf single");
    if (render_debug)
    {
        cv::putText(view, vote_info.str(), cv::Point(16, 84), cv::FONT_HERSHEY_SIMPLEX,
                    0.75, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }

    const ProbabilityDecisionSummary prob_summary =
        summarize_probabilities(decision_prob_sum, active_class_count, decision_valid_count);

    // [Recognition Chain Step 5] 自适应 1/2 帧判定。
    // 作用：达到门槛时单帧输出；否则第二个有效推理帧按两帧平均 top1 强制输出。
    std::string label = "no_decision";
    TargetClass target = TargetClass::UNKNOWN;
    bool has_final_decision = false;
    bool forced_two_frame_top1 = false;
    BoardVisionCode final_code = BoardVisionCode::NO_RESULT;
    std::string failure_reason =
        (decision_valid_count > 0) ? "low_confidence_or_margin_reject" :
        (was_waiting_second_frame ? "second_frame_invalid_model_output" : "invalid_model_output");
    const bool has_valid_top1 =
        decision_valid_count > 0 &&
        prob_summary.top1_index >= 0 &&
        prob_summary.top1_index < static_cast<int>(active_class_count);
    const bool passes_normal_threshold =
        has_valid_top1 &&
        prob_summary.top1_prob >= decision_top1_threshold_ &&
        prob_summary.margin >= decision_margin_threshold_;
    forced_two_frame_top1 = was_waiting_second_frame && has_valid_top1;
    if (passes_normal_threshold || forced_two_frame_top1)
    {
        label = runtime_decision_label(prob_summary.top1_index, class_names_);
        target = static_cast<TargetClass>(runtime_decision_target_code(prob_summary.top1_index, class_names_));
        final_code = vision_code_from_target_code(static_cast<uint8_t>(target));
        has_final_decision = (final_code != BoardVisionCode::INVALID);
        if (forced_two_frame_top1)
        {
            failure_reason = has_final_decision
                ? "two_frame_force_top1"
                : "two_frame_invalid_target_code";
        }
    }

    if (render_debug)
    {
        cv::putText(view, std::string("result: ") + label, cv::Point(16, 252), cv::FONT_HERSHEY_SIMPLEX,
                    0.70, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        if (!has_final_decision)
        {
            cv::putText(view, failure_reason, cv::Point(16, 280), cv::FONT_HERSHEY_SIMPLEX,
                        0.58, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
        }
    }

    if (kRecognitionResultLog)
    {
        std::cout << "[RECOG] result=" << label
                  << ", valid_frames=" << decision_valid_count
                  << ", mode=" << (forced_two_frame_top1 ? "two_frame_forced_avg" : "single_high_conf")
                  << ", infer_ms=" << std::fixed << std::setprecision(2) << infer_ms
                  << ", forward_ms=" << cls_timing.forward_ms
                  << ", cls_ms=" << last_perf_sample_.classify_total_ms
                  << ", top1_prob=" << std::fixed << std::setprecision(4) << prob_summary.top1_prob
                  << ", margin=" << prob_summary.margin;
        if (forced_two_frame_top1 || !has_final_decision)
        {
            std::cout << ", reason=" << failure_reason;
        }
        std::cout << std::endl;
    }

    current_vision_code_ = has_final_decision ? final_code : BoardVisionCode::NO_RESULT;
    latched_symbol_code_ = is_success_symbol_code(current_vision_code_)
        ? current_vision_code_
        : BoardVisionCode::INVALID;
    latched_release_pending_ = false;
    latched_release_deadline_ms_ = 0;

    if (kRecognitionResultLog)
    {
        std::cout << "[RECOG] state_out=" << vision_code_text(current_vision_code_)
                  << ", blob_area=" << std::fixed << std::setprecision(1) << current_blob_area_
                  << std::endl;
    }

    ClearAdaptiveDecision();
    mode_ = Mode::NORMAL;
    last_perf_sample_.process_recog_total_ms =
        std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
}
