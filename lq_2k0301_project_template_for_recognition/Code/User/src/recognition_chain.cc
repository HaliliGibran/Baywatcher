#include "recognition_chain.h"

#include "common.h"
#include "image_switch_utils.h"
#include "roi_runtime_geometry.h"
#include "transform_table.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <limits>
#include <regex>
#include <sstream>
#include <unistd.h>

namespace {

constexpr uint64_t kRecognitionTriggerRejectLogIntervalMs = 300;
constexpr uint64_t kRecognitionProbabilityTimeoutMs = 1200;
constexpr int kRecognitionTriggerFrameWidth = 640;
constexpr int kRecognitionTriggerFrameHeight = 480;
constexpr int kRecognitionMinSearchYInclusive = BW_RECOG_TRIGGER_SEARCH_Y_MIN;
constexpr int kRecognitionMaxSearchYExclusive = BW_RECOG_TRIGGER_SEARCH_Y_MAX;
constexpr int kPrecheckWhiteMaxSaturation = 60;
constexpr int kPrecheckWhiteMinValue = 150;
constexpr int kPrecheckWhiteMinSpanWidth = 120;
const cv::Scalar kPrecheckLowRed1(0, 50, 50);
const cv::Scalar kPrecheckHighRed1(10, 255, 255);
const cv::Scalar kPrecheckLowRed2(160, 50, 50);
const cv::Scalar kPrecheckHighRed2(180, 255, 255);
constexpr int kRecognitionMinValidFrames = 3;
constexpr int kRecognitionMaxValidFrames = 3;
constexpr float kRecognitionDecisionTop1AvgThreshold = 0.80f;
constexpr float kRecognitionDecisionMarginThreshold = 0.15f;

struct RoiClassificationResult
{
    int predicted_index = -1;
    std::array<float, 3> probabilities = {0.0f, 0.0f, 0.0f};
};

struct DeployCalibration
{
    float temperature = 1.0f;
    std::array<float, 3> logit_bias = {0.0f, 0.0f, 0.0f};
    float decision_top1_threshold = kRecognitionDecisionTop1AvgThreshold;
    float decision_margin_threshold = kRecognitionDecisionMarginThreshold;
    bool loaded = false;
};

struct ProbabilityDecisionSummary
{
    int top1_index = -1;
    int top2_index = -1;
    float top1_avg = 0.0f;
    float top2_avg = 0.0f;
    float margin = 0.0f;
};

static int clamp_int_range(int value, int low, int high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

static cv::Rect build_top_band_search_rect(const cv::Mat& frame_bgr)
{
    const int width = std::max(frame_bgr.cols, 1);
    const int y0 = clamp_int_range(kRecognitionMinSearchYInclusive, 0, std::max(frame_bgr.rows - 1, 0));
    const int y1 = clamp_int_range(kRecognitionMaxSearchYExclusive, y0 + 1, std::max(frame_bgr.rows, y0 + 1));
    return cv::Rect(0, y0, width, y1 - y0);
}

static bool compute_white_x_range_on_reference_row(const cv::Mat& frame_bgr, int* out_x_min, int* out_x_max)
{
    if (out_x_min == nullptr || out_x_max == nullptr || frame_bgr.empty())
    {
        return false;
    }

    const int image_height = frame_bgr.rows;
    const int image_width = frame_bgr.cols;
    if (image_height <= 0 || image_width <= 0)
    {
        return false;
    }

    const int row_y = std::max(0, std::min(BW_RECOG_WHITE_REFERENCE_ROW_Y, image_height - 1));
    const cv::Mat row_bgr = frame_bgr.row(row_y).clone();
    cv::Mat row_hsv;
    cv::cvtColor(row_bgr, row_hsv, cv::COLOR_BGR2HSV);

    int merged_start = -1;
    int merged_end = -1;
    int merged_count = 0;
    int start = -1;
    for (int x = 0; x < image_width; ++x)
    {
        const cv::Vec3b hsv = row_hsv.at<cv::Vec3b>(0, x);
        const bool is_white =
            hsv[1] <= static_cast<unsigned char>(kPrecheckWhiteMaxSaturation) &&
            hsv[2] >= static_cast<unsigned char>(kPrecheckWhiteMinValue);
        if (is_white && start < 0)
        {
            start = x;
        }
        else if (!is_white && start >= 0)
        {
            const int end = x - 1;
            if (end - start + 1 >= kPrecheckWhiteMinSpanWidth)
            {
                if (merged_count == 0)
                {
                    merged_start = start;
                    merged_end = end;
                }
                else
                {
                    merged_start = std::min(merged_start, start);
                    merged_end = std::max(merged_end, end);
                }
                ++merged_count;
            }
            start = -1;
        }
    }
    if (start >= 0)
    {
        const int end = image_width - 1;
        if (end - start + 1 >= kPrecheckWhiteMinSpanWidth)
        {
            if (merged_count == 0)
            {
                merged_start = start;
                merged_end = end;
            }
            else
            {
                merged_start = std::min(merged_start, start);
                merged_end = std::max(merged_end, end);
            }
            ++merged_count;
        }
    }

    if (merged_start < 0 || merged_end < merged_start || merged_count <= 0)
    {
        return false;
    }

    *out_x_min = merged_start;
    *out_x_max = merged_end;
    return true;
}

static bool cheap_red_precheck_hsv(const cv::Mat& frame_bgr, int* red_pixels_out = nullptr)
{
    if (red_pixels_out != nullptr)
    {
        *red_pixels_out = 0;
    }
    if (frame_bgr.empty() || frame_bgr.cols <= 0 || frame_bgr.rows <= 0)
    {
        return false;
    }

    const cv::Rect full_search_rect = build_top_band_search_rect(frame_bgr);
    cv::Rect search_rect = full_search_rect;
    int white_x_min = 0;
    int white_x_max = 0;
    if (compute_white_x_range_on_reference_row(frame_bgr, &white_x_min, &white_x_max))
    {
        const int x1 = std::max(full_search_rect.x, white_x_min);
        const int x2 = std::min(full_search_rect.x + full_search_rect.width, white_x_max + 1);
        if (x2 > x1)
        {
            search_rect.x = x1;
            search_rect.width = x2 - x1;
        }
    }

    if (search_rect.width <= 0 || search_rect.height <= 0)
    {
        return false;
    }

    cv::Mat hsv;
    cv::cvtColor(frame_bgr(search_rect), hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask_low;
    cv::Mat mask_high;
    cv::inRange(hsv, kPrecheckLowRed1, kPrecheckHighRed1, mask_low);
    cv::inRange(hsv, kPrecheckLowRed2, kPrecheckHighRed2, mask_high);
    cv::Mat red_mask;
    cv::bitwise_or(mask_low, mask_high, red_mask);

    const int red_pixels = cv::countNonZero(red_mask);
    if (red_pixels_out != nullptr)
    {
        *red_pixels_out = red_pixels;
    }
    return red_pixels >= BW_RECOG_NORMAL_PRECHECK_MIN_PIXELS;
}

static void draw_trigger_search_info(cv::Mat& view)
{
    std::ostringstream oss;
    oss << "search_y=[" << kRecognitionMinSearchYInclusive
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

static int clamp_recognition_trigger_coord(int v, int hi)
{
    if (v < 0)
    {
        return 0;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

static void map_trigger_rect_point_to_ipm(int x, int y, float* out_ipm_x, float* out_ipm_y)
{
    const int px = clamp_recognition_trigger_coord(x, kRecognitionTriggerFrameWidth - 1);
    const int py = clamp_recognition_trigger_coord(y, kRecognitionTriggerFrameHeight - 1);
    if (out_ipm_x != nullptr)
    {
        *out_ipm_x = UndistInverseMapW[py][px];
    }
    if (out_ipm_y != nullptr)
    {
        *out_ipm_y = UndistInverseMapH[py][px];
    }
}

static float ipm_segment_length(float x0, float y0, float x1, float y1)
{
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    return std::sqrt(dx * dx + dy * dy);
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
    case BoardVisionCode::NO_RESULT: return "u";
    case BoardVisionCode::UNKNOWN: return "n";
    default: return "-";
    }
}

static bool is_success_symbol_code(BoardVisionCode code)
{
    return code == BoardVisionCode::VEHICLE ||
           code == BoardVisionCode::WEAPON ||
           code == BoardVisionCode::SUPPLY;
}

static double roi_observed_red_area(const RoiExtractionResult& roi_result)
{
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
    return roi_observed_red_area(roi_result) >= static_cast<double>(BW_RECOG_LOOSE_RED_MIN_AREA);
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
    return (center_x < frame_width / 2) ? "left" : "right";
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
    return (center_x < frame_width / 2) ? "LEFT" : "RIGHT";
}

static std::string roi_reject_reason_text(const RoiExtractionResult& roi_result)
{
    std::ostringstream oss;
    oss << "roi_status=" << roi_result.status;
    oss << ", loose_red_area=" << std::fixed << std::setprecision(1) << roi_observed_red_area(roi_result);
    oss << ", white_crop=" << (roi_result.white_crop_clipped ? "clipped" : "no");
    oss << ", reject_stage=" << roi_result.max_red_reject_stage;
    oss << ", touches_top=" << (roi_result.touches_search_top ? "yes" : "no");
    oss << ", loose_ipm_valid=" << (roi_result.loose_ipm_valid ? "yes" : "no");
    if (!roi_result.loose_ipm_reason.empty())
    {
        oss << ", loose_ipm_reason=" << roi_result.loose_ipm_reason;
    }
    if (roi_result.merged_white_span_count > 0)
    {
        oss << ", white_spans=" << roi_result.merged_white_span_count;
    }
    if (!roi_result.ipm_reason.empty())
    {
        oss << ", ipm_reason=" << roi_result.ipm_reason;
    }
    return oss.str();
}

static bool roi_red_touches_search_top(const RoiExtractionResult& roi_result);

static std::string red_observation_text(const RoiExtractionResult& roi_result, int frame_width)
{
    if (!roi_has_non_noise_red(roi_result))
    {
        return "NO RED";
    }

    if (roi_red_touches_search_top(roi_result))
    {
        return "RED TOUCHES TOP";
    }

    if (roi_result.loose_ipm_valid &&
        roi_result.status != "rotated_roi")
    {
        return "SIGN-LIKE RED, ROI NOT READY";
    }

    if (roi_observed_red_area(roi_result) >= static_cast<double>(BW_RECOG_BRICK_MIN_AREA))
    {
        std::ostringstream oss;
        oss << "BRICK " << roi_blob_side_upper(roi_result, frame_width);
        return oss.str();
    }

    return "RED PRESENT, NOT BRICK";
}

static bool roi_red_touches_search_top(const RoiExtractionResult& roi_result)
{
    return roi_result.touches_search_top;
}

static bool roi_is_visible_sign_candidate(const RoiExtractionResult& roi_result)
{
    return roi_result.status == "rotated_roi" && !roi_red_touches_search_top(roi_result);
}

static bool roi_should_hold_success_latch(const RoiExtractionResult& roi_result)
{
    return roi_has_non_noise_red(roi_result) &&
           roi_result.loose_ipm_valid &&
           !roi_red_touches_search_top(roi_result);
}

static BoardVisionCode fallback_code_from_roi_result(const RoiExtractionResult& roi_result)
{
    const double red_area = roi_observed_red_area(roi_result);
    if (red_area < static_cast<double>(BW_RECOG_LOOSE_RED_MIN_AREA))
    {
        return BoardVisionCode::UNKNOWN;
    }

    if (roi_result.loose_ipm_valid && !roi_red_touches_search_top(roi_result))
    {
        return BoardVisionCode::NO_RESULT;
    }

    return red_area >= static_cast<double>(BW_RECOG_BRICK_MIN_AREA)
        ? BoardVisionCode::BRICK
        : BoardVisionCode::UNKNOWN;
}

static bool red_rect_is_horizontal_ipm_rect(const cv::Rect& rect,
                                            float* top_width_out = nullptr,
                                            float* bottom_width_out = nullptr,
                                            float* left_height_out = nullptr,
                                            float* right_height_out = nullptr)
{
    if (BW_RECOG_TRIGGER_IPM_RECT_ENABLE == 0)
    {
        return true;
    }

    if (rect.width <= 1 || rect.height <= 1)
    {
        return false;
    }

    float tl_x = 0.0f, tl_y = 0.0f;
    float tr_x = 0.0f, tr_y = 0.0f;
    float br_x = 0.0f, br_y = 0.0f;
    float bl_x = 0.0f, bl_y = 0.0f;
    map_trigger_rect_point_to_ipm(rect.x, rect.y, &tl_x, &tl_y);
    map_trigger_rect_point_to_ipm(rect.x + rect.width - 1, rect.y, &tr_x, &tr_y);
    map_trigger_rect_point_to_ipm(rect.x + rect.width - 1, rect.y + rect.height - 1, &br_x, &br_y);
    map_trigger_rect_point_to_ipm(rect.x, rect.y + rect.height - 1, &bl_x, &bl_y);

    const float top_width = ipm_segment_length(tl_x, tl_y, tr_x, tr_y);
    const float bottom_width = ipm_segment_length(bl_x, bl_y, br_x, br_y);
    const float left_height = ipm_segment_length(tl_x, tl_y, bl_x, bl_y);
    const float right_height = ipm_segment_length(tr_x, tr_y, br_x, br_y);

    if (top_width_out != nullptr)
    {
        *top_width_out = top_width;
    }
    if (bottom_width_out != nullptr)
    {
        *bottom_width_out = bottom_width;
    }
    if (left_height_out != nullptr)
    {
        *left_height_out = left_height;
    }
    if (right_height_out != nullptr)
    {
        *right_height_out = right_height;
    }

    const float ratio = BW_RECOG_TRIGGER_IPM_MIN_WIDTH_HEIGHT_RATIO;
    return top_width >= left_height * ratio &&
           top_width >= right_height * ratio &&
           bottom_width >= left_height * ratio &&
           bottom_width >= right_height * ratio;
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

static std::array<float, 3> parse_json_float_array3(const std::string& content,
                                                    const std::string& key,
                                                    const std::array<float, 3>& default_value)
{
    const std::regex re("\"" + key + "\"\\s*:\\s*\\[([^\\]]*)\\]");
    std::smatch match;
    if (!std::regex_search(content, match, re) || match.size() < 2)
    {
        return default_value;
    }
    std::array<float, 3> out = default_value;
    std::stringstream ss(match[1].str());
    std::string item;
    int idx = 0;
    while (std::getline(ss, item, ',') && idx < 3)
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
    calibration.logit_bias = parse_json_float_array3(content, "logit_bias", calibration.logit_bias);
    calibration.decision_top1_threshold = parse_json_number_or_default(content, "decision_top1_threshold", kRecognitionDecisionTop1AvgThreshold);
    calibration.decision_margin_threshold = parse_json_number_or_default(content, "decision_margin_threshold", kRecognitionDecisionMarginThreshold);
    calibration.loaded = true;
    return calibration;
}

static std::vector<std::string> load_class_names_from_json(const std::string& path)
{
    std::ifstream fin(path);
    if (!fin.is_open())
    {
        return {"supply", "vehicle", "weapon"};
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

    if (out.empty())
    {
        out = {"supply", "vehicle", "weapon"};
    }
    return out;
}

// [Recognition Chain] 模型标签到车体策略标签的映射。
// 作用：把模型输出的文本类别统一映射到工程内的控制枚举。
static uint8_t parse_target_class_code(const std::string& name)
{
    std::string s;
    s.resize(name.size());
    std::transform(name.begin(), name.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (s.find("weapon") != std::string::npos)
    {
        return 1;
    }
    if (s.find("supply") != std::string::npos)
    {
        return 2;
    }
    if (s.find("vehicle") != std::string::npos)
    {
        return 3;
    }
    return 0;
}

// [Recognition Chain] 红色触发物检测。
// 作用：在 640x480 原始图上找红色近矩形目标，作为进入识别态的前置触发器。
static bool detect_red_rect_like(const cv::Mat& frame_bgr, cv::Rect* best_rect, cv::Mat* out_mask = nullptr)
{
    if (best_rect == nullptr || frame_bgr.empty())
    {
        return false;
    }

    cv::Mat hsv;
    cv::cvtColor(frame_bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat m1, m2, red;
    cv::inRange(hsv, cv::Scalar(0, 60, 50), cv::Scalar(10, 255, 255), m1);
    cv::inRange(hsv, cv::Scalar(160, 60, 50), cv::Scalar(180, 255, 255), m2);
    cv::bitwise_or(m1, m2, red);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(red, red, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(red, red, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_score = -1.0;
    cv::Rect chosen;
    for (size_t i = 0; i < contours.size(); ++i)
    {
        const double area = cv::contourArea(contours[i]);
        if (area < 300.0)
        {
            continue;
        }

        const cv::Rect r = cv::boundingRect(contours[i]);
        if (r.width < 12 || r.height < 12)
        {
            continue;
        }

        const double fill = area / (double)(r.width * r.height + 1e-6);
        if (fill < 0.55)
        {
            continue;
        }

        const double ratio = (double)r.width / (double)r.height;
        const double ratio_penalty = std::min(std::abs(ratio - 1.0), 1.2);
        const double score = area * (1.0 - 0.35 * ratio_penalty) * fill;

        if (score > best_score)
        {
            best_score = score;
            chosen = r;
        }
    }

    if (best_score < 0.0)
    {
        return false;
    }

    *best_rect = chosen;
    if (out_mask != nullptr)
    {
        *out_mask = red;
    }
    return true;
}

// [Recognition Chain] 单个 ROI 的 Top-1 分类推理。
// 作用：把 ROI 统一缩放到 64x64，送入 ONNX，输出当前帧的类别索引。
static RoiClassificationResult classify_roi_index(cv::dnn::Net& net, const cv::Mat& roi_bgr, float calibration_temperature, const std::array<float, 3>& logit_bias)
{
    static const float kInputMean[3] = {0.485f, 0.456f, 0.406f};
    static const float kInputStd[3] = {0.229f, 0.224f, 0.225f};
    RoiClassificationResult result;
    cv::Mat resized;
    cv::resize(roi_bgr, resized, cv::Size(64, 64), 0, 0, cv::INTER_AREA);
    cv::Mat blob = cv::dnn::blobFromImage(
        resized,
        1.0 / 255.0,
        cv::Size(64, 64),
        cv::Scalar(),
        true,
        false
    );
    const int plane = 64 * 64;
    for (int c = 0; c < 3; ++c)
    {
        float* ptr = blob.ptr<float>(0, c);
        if (ptr == nullptr)
        {
            continue;
        }
        for (int i = 0; i < plane; ++i)
        {
            ptr[i] = (ptr[i] - kInputMean[c]) / kInputStd[c];
        }
    }
    net.setInput(blob);
    cv::Mat out = net.forward().reshape(1, 1);
    cv::Mat out_f;
    out.convertTo(out_f, CV_32F);
    const int count = std::min(static_cast<int>(out_f.total()), 3);
    float max_logit = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < count; ++i)
    {
        const float value = out_f.at<float>(0, i) / std::max(calibration_temperature, 1e-4f)
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
        const float adjusted = out_f.at<float>(0, i) / std::max(calibration_temperature, 1e-4f)
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

static ProbabilityDecisionSummary summarize_probabilities(const std::array<float, 3>& prob_sum, int valid_frames)
{
    ProbabilityDecisionSummary summary;
    if (valid_frames <= 0)
    {
        return summary;
    }

    std::array<float, 3> avg_probs = {0.0f, 0.0f, 0.0f};
    for (size_t i = 0; i < avg_probs.size(); ++i)
    {
        avg_probs[i] = prob_sum[i] / static_cast<float>(valid_frames);
    }

    int best_index = -1;
    int second_index = -1;
    float best_value = -1.0f;
    float second_value = -1.0f;
    for (int i = 0; i < static_cast<int>(avg_probs.size()); ++i)
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
    summary.top1_avg = std::max(best_value, 0.0f);
    summary.top2_avg = std::max(second_value, 0.0f);
    summary.margin = std::max(summary.top1_avg - summary.top2_avg, 0.0f);
    return summary;
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
      prob_sum_({0.0f, 0.0f, 0.0f}),
      logit_bias_({0.0f, 0.0f, 0.0f}),
      valid_frame_count_(0),
      min_valid_frames_(kRecognitionMinValidFrames),
      max_valid_frames_(kRecognitionMaxValidFrames),
      calibration_temperature_(1.0f),
      decision_top1_threshold_(kRecognitionDecisionTop1AvgThreshold),
        decision_margin_threshold_(kRecognitionDecisionMarginThreshold),
        mode_(Mode::NORMAL),
        recognition_timeout_ms_(0),
        current_vision_code_(BoardVisionCode::UNKNOWN),
        latched_symbol_code_(BoardVisionCode::INVALID),
        latched_release_deadline_ms_(0),
        latched_release_pending_(false),
        normal_precheck_next_ms_(0),
        normal_precheck_positive_(false),
        current_blob_area_(0.0)
{
}

bool RecognitionChain::Initialize(bool enabled_by_switch)
{
    const std::string configured_model_path = "./model/cls.onnx";
    const std::string configured_class_path = "./model/class_names.json";
    const std::string configured_calibration_path = "./model/deploy_calibration.json";
    const std::string model_path = resolve_runtime_path(configured_model_path);
    const std::string class_path = resolve_runtime_path(configured_class_path);
    const std::string calibration_path = resolve_runtime_path(configured_calibration_path);

    if (!enabled_by_switch)
    {
        enabled_ = false;
        std::cout << "[RECOG] disabled by switch" << std::endl;
        return false;
    }

    if (!file_exists(model_path))
    {
        enabled_ = false;
        std::cout << "[ONNX] model missing: configured=" << configured_model_path
                  << ", resolved=" << model_path << std::endl;
        std::cout << "[ONNX] cwd=" << get_current_working_directory()
                  << ", exe_dir=" << get_executable_directory() << std::endl;
        std::cout << "[ONNX] disabled, waiting for valid model/class files." << std::endl;
        return false;
    }

    try
    {
        // [Recognition Chain Step 1] 加载 ONNX 模型与类别表。
        // 作用：完成后 enabled_ 才允许进入红色触发和识别态。
        net_ = cv::dnn::readNetFromONNX(model_path);
        class_names_ = load_class_names_from_json(class_path);
        const DeployCalibration calibration = load_deploy_calibration_json(calibration_path);
        calibration_temperature_ = calibration.temperature;
        logit_bias_ = calibration.logit_bias;
        decision_top1_threshold_ = calibration.decision_top1_threshold;
        decision_margin_threshold_ = calibration.decision_margin_threshold;
        enabled_ = !net_.empty();
    }
    catch (const std::exception& e)
    {
        enabled_ = false;
        std::cout << "[ONNX] disabled: " << e.what() << std::endl;
    }

    if (enabled_)
    {
        std::cout << "[ONNX] enabled, model=" << model_path << std::endl;
        std::cout << "[ONNX] classes=" << class_path << std::endl;
        std::cout << "[RECOG] roi_method=" << RoiMethodName(DefaultRoiMethod()) << std::endl;
        std::cout << "[RECOG] decision=fixed3_prob_accum"
                  << ", top1_threshold=" << decision_top1_threshold_
                  << ", margin_threshold=" << decision_margin_threshold_ << std::endl;
        std::cout << "[RECOG] calibration=" << calibration_path
                  << ", temperature=" << calibration_temperature_ << std::endl;
            std::cout << "[RECOG] state map:"
                    << " weapon->w, supply->s, vehicle->v, brick->b, no_result->u, unknown->n" << std::endl;
    }
    else
    {
        std::cout << "[ONNX] disabled, waiting for valid model/class files." << std::endl;
    }
    return enabled_;
}

void RecognitionChain::Reset()
{
    prob_sum_ = {0.0f, 0.0f, 0.0f};
    valid_frame_count_ = 0;
    mode_ = Mode::NORMAL;
    recognition_timeout_ms_ = 0;
    current_vision_code_ = BoardVisionCode::UNKNOWN;
    latched_symbol_code_ = BoardVisionCode::INVALID;
    latched_release_deadline_ms_ = 0;
    latched_release_pending_ = false;
    normal_precheck_next_ms_ = 0;
    normal_precheck_positive_ = false;
    current_blob_area_ = 0.0;
}

bool RecognitionChain::IsEnabled() const
{
    return enabled_;
}

bool RecognitionChain::IsInRecognitionMode() const
{
    return mode_ == Mode::RECOGNITION;
}

BoardVisionCode RecognitionChain::GetCurrentVisionCode() const
{
    return current_vision_code_;
}

double RecognitionChain::GetCurrentBlobArea() const
{
    return current_blob_area_;
}

bool RecognitionChain::TryEnterRecognition(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view, bool render_debug)
{
    if (!enabled_ || mode_ != Mode::NORMAL)
    {
        return false;
    }

    // [Recognition Chain Step 2] 运行时 ROI 触发器。
    // 作用：识别板直接复用 yolo 侧的 ROI 提取与质量过滤逻辑，只要 ROI 合法才进入识别态。
    if (render_debug)
    {
        view = frame_bgr.clone();
        draw_trigger_search_info(view);
    }
    else
    {
        view.release();
    }

    if (!is_success_symbol_code(latched_symbol_code_))
    {
        if (t_ms >= normal_precheck_next_ms_)
        {
            int red_pixels = 0;
            normal_precheck_positive_ = cheap_red_precheck_hsv(frame_bgr, &red_pixels);
            normal_precheck_next_ms_ = t_ms + BW_RECOG_NORMAL_PRECHECK_INTERVAL_MS;
            if (!normal_precheck_positive_)
            {
                current_vision_code_ = BoardVisionCode::UNKNOWN;
                current_blob_area_ = 0.0;
                if (render_debug)
                {
                    std::ostringstream oss;
                    oss << "WAIT RED ROI (precheck=" << red_pixels << ")";
                    cv::putText(view, oss.str(), cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                                0.62, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
                }
                return false;
            }
        }

        if (!normal_precheck_positive_)
        {
            current_vision_code_ = BoardVisionCode::UNKNOWN;
            current_blob_area_ = 0.0;
            if (render_debug)
            {
                cv::putText(view, "WAIT RED ROI", cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                            0.62, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
            return false;
        }
    }

    const RoiMethod roi_method = DefaultRoiMethod();
    const cv::Rect search_rect = build_top_band_search_rect(frame_bgr);
    const RoiExtractionResult trigger_roi =
        ExtractRotatedRoi(frame_bgr, 64, roi_method, &search_rect);
    if (render_debug)
    {
        DrawRoiDebugOverlay(view, trigger_roi);
        draw_roi_preview_inset(view, trigger_roi.roi_bgr);
    }
    current_blob_area_ = roi_observed_red_area(trigger_roi);
    const bool symbol_candidate_visible = roi_is_visible_sign_candidate(trigger_roi);
    const bool holdable_sign_red_visible = roi_should_hold_success_latch(trigger_roi);
    if (is_success_symbol_code(latched_symbol_code_))
    {
        current_vision_code_ = latched_symbol_code_;
        if (holdable_sign_red_visible)
        {
            if (latched_release_pending_)
            {
                std::cout << "[RECOG] sign visible again, keep latched result="
                          << vision_code_text(latched_symbol_code_) << std::endl;
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
            return false;
        }

        if (!latched_release_pending_)
        {
            latched_release_pending_ = true;
            latched_release_deadline_ms_ = t_ms + BW_RECOG_SIGN_LOSS_HOLD_MS;
            std::cout << "[RECOG] sign lost, hold latched result="
                      << vision_code_text(latched_symbol_code_)
                      << " for " << BW_RECOG_SIGN_LOSS_HOLD_MS << " ms" << std::endl;
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
            return false;
        }

        std::cout << "[RECOG] sign loss timeout, release latched result="
                  << vision_code_text(latched_symbol_code_)
                  << " -> n" << std::endl;
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;
        current_vision_code_ = BoardVisionCode::UNKNOWN;
        if (render_debug)
        {
            cv::putText(view, "SIGN LOST -> n", cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                        0.65, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
        }
        return false;
    }

    if (trigger_roi.status == "miss")
    {
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;
        current_vision_code_ = fallback_code_from_roi_result(trigger_roi);
        const std::string red_text = red_observation_text(trigger_roi, frame_bgr.cols);
        const std::string reject_text = roi_reject_reason_text(trigger_roi);
        if (roi_has_non_noise_red(trigger_roi) ||
            trigger_roi.has_prewhite_max_red_contour_area ||
            trigger_roi.white_crop_clipped)
        {
            if (current_vision_code_ == BoardVisionCode::NO_RESULT)
            {
                std::cout << "[RECOG] IPM PASS -> ROI NOT READY: area="
                          << std::fixed << std::setprecision(1)
                          << current_blob_area_ << std::endl;
            }
            else if (current_vision_code_ == BoardVisionCode::BRICK)
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
        }
        return false;
    }

    if (roi_has_non_noise_red(trigger_roi) && roi_red_touches_search_top(trigger_roi))
    {
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;
        current_vision_code_ = fallback_code_from_roi_result(trigger_roi);
        const std::string red_text = red_observation_text(trigger_roi, frame_bgr.cols);
        std::ostringstream reject_text;
        reject_text << "touches_search_top_y=" << BW_RECOG_TRIGGER_SEARCH_Y_MIN
                    << ", skip_sign_path";
        std::cout << "[RECOG] red touches search top, skip sign-board path"
                  << ", side=" << roi_blob_side_lower(trigger_roi, frame_bgr.cols)
                  << ", area=" << std::fixed << std::setprecision(1) << current_blob_area_
                  << ", state=" << vision_code_text(current_vision_code_)
                  << std::endl;
        if (render_debug)
        {
            cv::putText(view, red_text, cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                        0.65, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            cv::putText(view, reject_text.str(), cv::Point(16, 140), cv::FONT_HERSHEY_SIMPLEX,
                        0.50, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
        }
        return false;
    }

    if (trigger_roi.status != "rotated_roi")
    {
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;
        current_vision_code_ = fallback_code_from_roi_result(trigger_roi);
        const std::string red_text = red_observation_text(trigger_roi, frame_bgr.cols);
        const std::string reject_text = roi_reject_reason_text(trigger_roi);
        static uint64_t last_reject_log_ms = 0;
        if (t_ms >= last_reject_log_ms + kRecognitionTriggerRejectLogIntervalMs)
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
                else if (current_vision_code_ == BoardVisionCode::BRICK)
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
        }
        return false;
    }

    const RoiQualityMetrics quality =
        ComputeLowInformationRoiMetrics(trigger_roi.roi_bgr, roi_method, trigger_roi);
    if (!quality.valid)
    {
        current_vision_code_ = BoardVisionCode::NO_RESULT;
        const std::string side_text = roi_blob_side_lower(trigger_roi, frame_bgr.cols);
        static uint64_t last_quality_reject_log_ms = 0;
        if (t_ms >= last_quality_reject_log_ms + kRecognitionTriggerRejectLogIntervalMs)
        {
            last_quality_reject_log_ms = t_ms;
            std::cout << "[RECOG] sign-like red rejected before recognition: quality=" << quality.reason
                      << ", side=" << side_text
                      << ", area=" << std::fixed << std::setprecision(1) << current_blob_area_
                      << ", gray_std=" << std::fixed << std::setprecision(4) << quality.gray_std_top
                      << ", canny=" << quality.canny_density_top
                      << ", lap_var=" << quality.lap_var_top;
            if (trigger_roi.has_ipm_backproject_height_ratio)
            {
                std::cout << ", raw_height_ratio=" << trigger_roi.ipm_backproject_height_ratio;
            }
            std::cout << std::endl;
        }
        if (render_debug)
        {
            cv::putText(view, "SIGN-LIKE RED REJECT", cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                        0.65, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            cv::putText(view, quality.reason, cv::Point(16, 140), cv::FONT_HERSHEY_SIMPLEX,
                        0.55, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
        }
        return false;
    }

    // [Recognition Chain Step 3] 进入识别态。
    // 作用：开始累计 3~5 帧概率。
    mode_ = Mode::RECOGNITION;
    prob_sum_ = {0.0f, 0.0f, 0.0f};
    valid_frame_count_ = 0;
    recognition_timeout_ms_ = t_ms + kRecognitionProbabilityTimeoutMs;
    current_vision_code_ = BoardVisionCode::NO_RESULT;
    latched_release_pending_ = false;
    latched_release_deadline_ms_ = 0;
    std::cout << "[RECOG] sign board accepted: entering recognition"
              << ", side=" << roi_blob_side_lower(trigger_roi, frame_bgr.cols)
              << ", area=" << std::fixed << std::setprecision(1) << current_blob_area_
              << std::endl;

    if (render_debug)
    {
        cv::putText(view, "TRIGGER -> RECOGNITION", cv::Point(16, 84), cv::FONT_HERSHEY_SIMPLEX,
                    0.65, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }
    return true;
}

void RecognitionChain::ProcessRecognitionFrame(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view, bool render_debug)
{
    if (!enabled_ || mode_ != Mode::RECOGNITION)
    {
        if (render_debug)
        {
            view = frame_bgr.clone();
        }
        else
        {
            view.release();
        }
        return;
    }

    // [Recognition Chain Step 4] 识别态逐帧推理。
    // 作用：持续按 yolo 侧 ROI 几何提取 64x64 小图，只对质量合格的 ROI 累计概率。
    if (render_debug)
    {
        view = frame_bgr.clone();
    }
    else
    {
        view.release();
    }
    const RoiMethod roi_method = DefaultRoiMethod();
    const cv::Rect search_rect = build_top_band_search_rect(frame_bgr);
    const RoiExtractionResult roi_result =
        ExtractRotatedRoi(frame_bgr, 64, roi_method, &search_rect);
    if (render_debug)
    {
        DrawRoiDebugOverlay(view, roi_result);
        draw_roi_preview_inset(view, roi_result.roi_bgr);
        draw_trigger_search_info(view);
    }
    current_blob_area_ = roi_observed_red_area(roi_result);

    if (roi_result.status == "miss")
    {
        prob_sum_ = {0.0f, 0.0f, 0.0f};
        valid_frame_count_ = 0;
        current_vision_code_ = fallback_code_from_roi_result(roi_result);
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
        }
        std::cout << "[RECOG] recognition exit: miss, fallback_state="
                  << vision_code_text(current_vision_code_)
                  << ", " << reject_text << std::endl;
        mode_ = Mode::NORMAL;
        return;
    }

    if (roi_result.status != "rotated_roi")
    {
        prob_sum_ = {0.0f, 0.0f, 0.0f};
        valid_frame_count_ = 0;
        current_vision_code_ = fallback_code_from_roi_result(roi_result);
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
        }
        std::cout << "[RECOG] recognition exit: " << reject_text
                  << ", fallback_state=" << vision_code_text(current_vision_code_);
        if (roi_has_non_noise_red(roi_result))
        {
            std::cout << ", area=" << std::fixed << std::setprecision(1) << current_blob_area_;
            if (current_vision_code_ == BoardVisionCode::BRICK)
            {
                std::cout << ", side=" << roi_blob_side_lower(roi_result, frame_bgr.cols);
            }
        }
        std::cout << std::endl;
        mode_ = Mode::NORMAL;
        return;
    }

    current_vision_code_ = BoardVisionCode::NO_RESULT;

    const RoiQualityMetrics quality =
        ComputeLowInformationRoiMetrics(roi_result.roi_bgr, roi_method, roi_result);
    std::ostringstream quality_info;
    quality_info << "quality: " << quality.reason;
    if (render_debug)
    {
        cv::putText(view, quality_info.str(), cv::Point(16, 140), cv::FONT_HERSHEY_SIMPLEX,
                    0.55, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
    }

    if (quality.valid)
    {
        const auto infer_begin = std::chrono::steady_clock::now();
        const RoiClassificationResult cls = classify_roi_index(net_, roi_result.roi_bgr, calibration_temperature_, logit_bias_);
        const auto infer_end = std::chrono::steady_clock::now();
        const double infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_begin).count();
        if (cls.predicted_index >= 0 && cls.predicted_index < static_cast<int>(class_names_.size()))
        {
            for (size_t i = 0; i < prob_sum_.size(); ++i)
            {
                prob_sum_[i] += cls.probabilities[i];
            }
            valid_frame_count_++;
            const std::string& name = class_names_[cls.predicted_index];
            if (render_debug)
            {
                cv::putText(view, std::string("pred: ") + name, cv::Point(16, 112), cv::FONT_HERSHEY_SIMPLEX,
                            0.65, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            }
            std::ostringstream infer_info;
            infer_info << "infer: " << std::fixed << std::setprecision(2) << infer_ms << " ms";
            if (render_debug)
            {
                cv::putText(view, infer_info.str(), cv::Point(16, 168), cv::FONT_HERSHEY_SIMPLEX,
                            0.55, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
            }
            const ProbabilityDecisionSummary prob_summary =
                summarize_probabilities(prob_sum_, valid_frame_count_);
            std::ostringstream prob_info;
            prob_info << "avg_probs: "
                      << std::fixed << std::setprecision(2)
                      << prob_sum_[0] / static_cast<float>(valid_frame_count_) << "/"
                      << prob_sum_[1] / static_cast<float>(valid_frame_count_) << "/"
                      << prob_sum_[2] / static_cast<float>(valid_frame_count_);
            if (render_debug)
            {
                cv::putText(view, prob_info.str(), cv::Point(16, 196), cv::FONT_HERSHEY_SIMPLEX,
                            0.52, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
            }
            std::ostringstream top_info;
            top_info << "top1="
                     << (prob_summary.top1_index >= 0 && prob_summary.top1_index < static_cast<int>(class_names_.size())
                             ? class_names_[prob_summary.top1_index]
                             : std::string("unknown"))
                     << " p=" << std::fixed << std::setprecision(2) << prob_summary.top1_avg
                     << " m=" << prob_summary.margin;
            if (render_debug)
            {
                cv::putText(view, top_info.str(), cv::Point(16, 224), cv::FONT_HERSHEY_SIMPLEX,
                            0.52, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
            }
            std::cout << "[RECOG] frame_pred=" << name
                      << ", infer_ms=" << std::fixed << std::setprecision(2) << infer_ms
                      << ", valid_frames=" << valid_frame_count_ << "/" << max_valid_frames_
                      << ", roi_method=" << RoiMethodName(roi_method) << std::endl;
        }
    }

    std::ostringstream vote_info;
    vote_info << "RECOG valid: " << valid_frame_count_ << "/" << max_valid_frames_;
    if (render_debug)
    {
        cv::putText(view, vote_info.str(), cv::Point(16, 84), cv::FONT_HERSHEY_SIMPLEX,
                    0.75, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }

    const ProbabilityDecisionSummary prob_summary =
        summarize_probabilities(prob_sum_, valid_frame_count_);
    const bool early_ready =
        valid_frame_count_ >= min_valid_frames_ &&
        prob_summary.top1_avg >= decision_top1_threshold_ &&
        prob_summary.margin >= decision_margin_threshold_;
    const bool max_frames_reached = valid_frame_count_ >= max_valid_frames_;
    const bool timeout = t_ms >= recognition_timeout_ms_;
    if (!early_ready && !max_frames_reached && !timeout)
    {
        return;
    }

    // [Recognition Chain Step 5] 概率累积收敛。
    // 作用：从 3~5 帧概率里选最终类别，再映射到后续车体策略。
    std::string label = "no_decision";
    TargetClass target = TargetClass::UNKNOWN;
    bool has_final_decision = false;
    BoardVisionCode final_code = BoardVisionCode::NO_RESULT;
    std::string failure_reason = quality.valid ? "low_confidence_or_margin_reject"
                                               : std::string("quality=") + quality.reason;
    if (!quality.valid && timeout)
    {
        failure_reason = std::string("quality=") + quality.reason + ", timeout";
    }
    else if (valid_frame_count_ < min_valid_frames_ && timeout)
    {
        failure_reason = "timeout_before_min_valid_frames";
    }
    if (valid_frame_count_ >= min_valid_frames_ &&
        prob_summary.top1_index >= 0 &&
        prob_summary.top1_index < static_cast<int>(class_names_.size()) &&
        prob_summary.top1_avg >= decision_top1_threshold_ &&
        prob_summary.margin >= decision_margin_threshold_)
    {
        label = class_names_[prob_summary.top1_index];
        target = static_cast<TargetClass>(parse_target_class_code(label));
        final_code = vision_code_from_target_code(static_cast<uint8_t>(target));
        has_final_decision = (final_code != BoardVisionCode::INVALID);
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

    std::cout << "[RECOG] result=" << label
              << ", valid_frames=" << valid_frame_count_
              << ", top1_avg=" << std::fixed << std::setprecision(4) << prob_summary.top1_avg
              << ", margin=" << prob_summary.margin;
    if (!has_final_decision)
    {
        std::cout << ", reason=" << failure_reason;
    }
    std::cout << std::endl;

    current_vision_code_ = has_final_decision ? final_code : BoardVisionCode::NO_RESULT;
    latched_symbol_code_ = is_success_symbol_code(current_vision_code_)
        ? current_vision_code_
        : BoardVisionCode::INVALID;
    latched_release_pending_ = false;
    latched_release_deadline_ms_ = 0;

    std::cout << "[RECOG] state_out=" << vision_code_text(current_vision_code_)
              << ", blob_area=" << std::fixed << std::setprecision(1) << current_blob_area_
              << std::endl;

    mode_ = Mode::NORMAL;
    prob_sum_ = {0.0f, 0.0f, 0.0f};
    valid_frame_count_ = 0;
}
