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
constexpr int kRecognitionTriggerFrameWidth = BW_RECOG_CAMERA_FRAME_WIDTH;
constexpr int kRecognitionTriggerFrameHeight = BW_RECOG_CAMERA_FRAME_HEIGHT;
constexpr int kRecognitionMinSearchYInclusive = BW_RECOG_TRIGGER_SEARCH_Y_MIN;
constexpr int kRecognitionMaxSearchYExclusive = BW_RECOG_TRIGGER_SEARCH_Y_MAX;
constexpr uint64_t kRecognitionRecentCandidateHoldMs = 200;
constexpr bool kRecognitionTextLog = (BW_RECOG_TEXT_LOG_ENABLE != 0);
constexpr bool kRecognitionVerboseLog = kRecognitionTextLog && (BW_RECOG_VERBOSE_LOG != 0);
constexpr int kRecognitionMinValidFrames = BW_RECOG_MIN_VALID_FRAMES;
constexpr int kRecognitionMaxValidFrames = BW_RECOG_MAX_VALID_FRAMES;
constexpr float kRecognitionDecisionTop1AvgThreshold = BW_RECOG_DECISION_TOP1_AVG_THRESHOLD;
constexpr float kRecognitionDecisionMarginThreshold = BW_RECOG_DECISION_MARGIN_THRESHOLD;
constexpr int kRecognitionModelVariant = BW_RECOG_MODEL_VARIANT;
constexpr bool kRecognitionUseGrayRed32Model =
    (kRecognitionModelVariant == BW_RECOG_MODEL_VARIANT_GRAYRED32);
constexpr bool kRecognitionUseGray32SubclassModel =
    (kRecognitionModelVariant == BW_RECOG_MODEL_VARIANT_GRAY32_SUBCLASS);
constexpr int kRecognitionModelInputSize =
    (kRecognitionUseGrayRed32Model || kRecognitionUseGray32SubclassModel) ? 32 : 64;
constexpr const char* kRecognitionModelRootDir =
    kRecognitionUseGray32SubclassModel
        ? "./model_subclass320_mlp_gray_256_rank1"
        :
    kRecognitionUseGrayRed32Model
        ? "./model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1"
        : "./model";
constexpr const char* kRecognitionModelVariantName =
    kRecognitionUseGray32SubclassModel ? "gray32_subclass_mlp_256" :
    (kRecognitionUseGrayRed32Model ? "grayred32_mlp_wider" : "rgb64_classic");
constexpr size_t kRecognitionMaxClasses = RecognitionChain::kMaxModelClasses;

struct RoiClassificationResult
{
    int predicted_index = -1;
    std::array<float, kRecognitionMaxClasses> probabilities = {};
};

struct DeployCalibration
{
    float temperature = 1.0f;
    std::array<float, kRecognitionMaxClasses> logit_bias = {};
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

static RoiClassificationResult finalize_logits_to_result(const cv::Mat& logits_f32,
                                                         float calibration_temperature,
                                                         const std::array<float, kRecognitionMaxClasses>& logit_bias)
{
    RoiClassificationResult result;
    const int count = std::min(static_cast<int>(logits_f32.total()),
                               static_cast<int>(kRecognitionMaxClasses));
    float max_logit = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < count; ++i)
    {
        const float value = logits_f32.at<float>(0, i) / std::max(calibration_temperature, 1e-4f)
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
        const float adjusted = logits_f32.at<float>(0, i) / std::max(calibration_temperature, 1e-4f)
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



static BoardVisionCode fallback_code_from_roi_result(const RoiExtractionResult& roi_result)
{
    if (roi_result.target_type == "roadblock" || roi_result.status == "roadblock")
    {
        return BoardVisionCode::BRICK;
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
    calibration.decision_top1_threshold = parse_json_number_or_default(content, "decision_top1_threshold", kRecognitionDecisionTop1AvgThreshold);
    calibration.decision_margin_threshold = parse_json_number_or_default(content, "decision_margin_threshold", kRecognitionDecisionMarginThreshold);
    calibration.loaded = true;
    return calibration;
}

static std::vector<std::string> load_class_names_from_json(const std::string& path)
{
    const auto default_class_names = []() -> std::vector<std::string> {
        if (kRecognitionUseGray32SubclassModel)
        {
            return {"急救包", "望远镜", "救护车", "装甲车", "枪支", "炸药包"};
        }
        return {"weapon", "supply", "vehicle"};
    };

    std::ifstream fin(path);
    if (!fin.is_open())
    {
        return default_class_names();
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
        out = default_class_names();
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
    if (name.find("枪支") != std::string::npos || name.find("炸药包") != std::string::npos)
    {
        return 1;
    }
    if (name.find("急救包") != std::string::npos || name.find("望远镜") != std::string::npos)
    {
        return 2;
    }
    if (name.find("救护车") != std::string::npos || name.find("装甲车") != std::string::npos)
    {
        return 3;
    }
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
// 作用：在当前 320x240 原始图上找红色近矩形目标，作为进入识别态的前置触发器。
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
// 作用：按当前模型模式把 ROI 预处理成对应 blob，再送入 ONNX，输出当前帧的类别索引。
static RoiClassificationResult classify_roi_index(cv::dnn::Net& net,
                                                  const cv::Mat& roi_bgr,
                                                  float calibration_temperature,
                                                  const std::array<float, kRecognitionMaxClasses>& logit_bias)
{
    cv::Mat blob;

    if (kRecognitionUseGray32SubclassModel)
    {
        cv::Mat resized;
        cv::resize(roi_bgr, resized, cv::Size(32, 32), 0, 0, cv::INTER_AREA);

        cv::Mat gray_u8;
        cv::cvtColor(resized, gray_u8, cv::COLOR_BGR2GRAY);
        cv::Mat gray_f32;
        gray_u8.convertTo(gray_f32, CV_32F, 1.0 / 255.0);

        const int sizes[4] = {1, 1, 32, 32};
        blob = cv::Mat(4, sizes, CV_32F, cv::Scalar(0));
        float* gray_channel = blob.ptr<float>(0, 0);

        for (int y = 0; y < 32; ++y)
        {
            for (int x = 0; x < 32; ++x)
            {
                const int idx = y * 32 + x;
                const float gray = gray_f32.at<float>(y, x);
                gray_channel[idx] = (gray - 0.449f) / 0.226f;
            }
        }
    }
    else if (kRecognitionUseGrayRed32Model)
    {
        cv::Mat resized;
        cv::resize(roi_bgr, resized, cv::Size(32, 32), 0, 0, cv::INTER_AREA);

        cv::Mat resized_f32;
        resized.convertTo(resized_f32, CV_32FC3, 1.0 / 255.0);

        const int sizes[4] = {1, 2, 32, 32};
        blob = cv::Mat(4, sizes, CV_32F, cv::Scalar(0));
        float* gray_channel = blob.ptr<float>(0, 0);
        float* red_dom_channel = blob.ptr<float>(0, 1);

        for (int y = 0; y < 32; ++y)
        {
            for (int x = 0; x < 32; ++x)
            {
                const cv::Vec3f bgr = resized_f32.at<cv::Vec3f>(y, x);
                const float b = bgr[0];
                const float g = bgr[1];
                const float r = bgr[2];
                const float gray = 0.299f * r + 0.587f * g + 0.114f * b;
                const float red_dom = std::max(r - std::max(g, b), 0.0f);
                const int idx = y * 32 + x;
                gray_channel[idx] = (gray - 0.449f) / 0.226f;
                red_dom_channel[idx] = (red_dom - 0.0f) / 1.0f;
            }
        }
    }
    else
    {
        static const float kInputMean[3] = {0.485f, 0.456f, 0.406f};
        static const float kInputStd[3] = {0.229f, 0.224f, 0.225f};
        cv::Mat resized;
        cv::resize(roi_bgr, resized, cv::Size(64, 64), 0, 0, cv::INTER_AREA);
        blob = cv::dnn::blobFromImage(
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
    }

    net.setInput(blob);
    cv::Mat out = net.forward().reshape(1, 1);
    cv::Mat out_f;
    out.convertTo(out_f, CV_32F);
    return finalize_logits_to_result(out_f, calibration_temperature, logit_bias);
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
      prob_sum_(),
      logit_bias_(),
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
        current_blob_area_(0.0),
        recent_red_candidate_until_ms_(0),
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

    if (!file_exists(model_path))
    {
        enabled_ = false;
        std::cerr << "[ONNX] model missing: configured=" << configured_model_path
                  << ", resolved=" << model_path << std::endl;
        std::cerr << "[ONNX] cwd=" << get_current_working_directory()
                  << ", exe_dir=" << get_executable_directory() << std::endl;
        std::cerr << "[ONNX] disabled, waiting for valid model/class files." << std::endl;
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
        decision_top1_threshold_ = kRecognitionDecisionTop1AvgThreshold;
        decision_margin_threshold_ = kRecognitionDecisionMarginThreshold;
        enabled_ = !net_.empty();
    }
    catch (const std::exception& e)
    {
        enabled_ = false;
        std::cerr << "[ONNX] disabled: " << e.what() << std::endl;
    }

    if (enabled_)
    {
        if (kRecognitionTextLog)
        {
            std::cout << "[ONNX] variant=" << kRecognitionModelVariantName
                      << ", input=" << kRecognitionModelInputSize << "x" << kRecognitionModelInputSize
                      << (kRecognitionUseGray32SubclassModel
                              ? ", channels=1(gray)"
                              : (kRecognitionUseGrayRed32Model ? ", channels=2(gray+red_dom)" : ", channels=3(rgb)"))
                      << std::endl;
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
    }
    else
    {
        if (kRecognitionTextLog)
        {
            std::cout << "[ONNX] disabled, waiting for valid model/class files." << std::endl;
        }
    }
    return enabled_;
}

void RecognitionChain::Reset()
{
    prob_sum_.fill(0.0f);
    valid_frame_count_ = 0;
    mode_ = Mode::NORMAL;
    recognition_timeout_ms_ = 0;
    current_vision_code_ = BoardVisionCode::UNKNOWN;
    latched_symbol_code_ = BoardVisionCode::INVALID;
    latched_release_deadline_ms_ = 0;
    latched_release_pending_ = false;
    current_blob_area_ = 0.0;
    recent_red_candidate_until_ms_ = 0;
    last_perf_sample_ = PerfSample();
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

    const RoiMethod roi_method = DefaultRoiMethod();
    const auto extract_begin = steady_clock_t::now();
    RoiExtractionResult trigger_roi =
        ExtractRotatedRoi(frame_bgr, kRecognitionModelInputSize, roi_method);
    const auto extract_end = steady_clock_t::now();
    last_perf_sample_.extract_roi_ms =
        std::chrono::duration<double, std::milli>(extract_end - extract_begin).count();
    last_perf_sample_.extract_roi_called = true;
    if (render_debug)
    {
        DrawRoiDebugOverlay(view, trigger_roi);
        draw_roi_preview_inset(view, trigger_roi.roi_bgr);
    }
    current_blob_area_ = roi_observed_red_area(trigger_roi);
    if (roi_has_non_noise_red(trigger_roi))
    {
        recent_red_candidate_until_ms_ = t_ms + kRecognitionRecentCandidateHoldMs;
    }
    const bool symbol_candidate_visible = roi_is_visible_sign_candidate(trigger_roi);
    const bool holdable_sign_red_visible = roi_should_hold_success_latch(trigger_roi);
    if (is_success_symbol_code(latched_symbol_code_))
    {
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
        latched_symbol_code_ = BoardVisionCode::INVALID;
        latched_release_pending_ = false;
        latched_release_deadline_ms_ = 0;
        current_vision_code_ = fallback_code_from_roi_result(trigger_roi);
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
        last_perf_sample_.try_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - try_begin).count();
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
        last_perf_sample_.try_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - try_begin).count();
        return false;
    }

    // [Recognition Chain Step 3] 进入识别态。
    // 作用：marker ROI 一旦构造成功，就直接进入分类阶段。
    mode_ = Mode::RECOGNITION;
    prob_sum_.fill(0.0f);
    valid_frame_count_ = 0;
    recognition_timeout_ms_ = t_ms + kRecognitionProbabilityTimeoutMs;
    current_vision_code_ = BoardVisionCode::NO_RESULT;
    latched_release_pending_ = false;
    latched_release_deadline_ms_ = 0;
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
    return true;
}

void RecognitionChain::ProcessRecognitionFrame(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view, bool render_debug)
{
    using steady_clock_t = std::chrono::steady_clock;
    const auto process_begin = steady_clock_t::now();
    last_perf_sample_ = PerfSample();
    last_perf_sample_.process_recog_total_called = true;

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
        last_perf_sample_.process_recog_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
        return;
    }

    // [Recognition Chain Step 4] 识别态逐帧推理。
    // 作用：每帧都按文档流程重新提取 marker ROI，再做分类概率累计。
    if (render_debug)
    {
        view = frame_bgr.clone();
    }
    else
    {
        view.release();
    }
    const RoiMethod roi_method = DefaultRoiMethod();
    const auto extract_begin = steady_clock_t::now();
    RoiExtractionResult roi_result =
        ExtractRotatedRoi(frame_bgr, kRecognitionModelInputSize, roi_method);
    const auto extract_end = steady_clock_t::now();
    last_perf_sample_.extract_roi_ms =
        std::chrono::duration<double, std::milli>(extract_end - extract_begin).count();
    last_perf_sample_.extract_roi_called = true;
    if (render_debug)
    {
        DrawRoiDebugOverlay(view, roi_result);
        draw_roi_preview_inset(view, roi_result.roi_bgr);
        draw_trigger_search_info(view);
    }
    current_blob_area_ = roi_observed_red_area(roi_result);
    if (roi_has_non_noise_red(roi_result))
    {
        recent_red_candidate_until_ms_ = t_ms + kRecognitionRecentCandidateHoldMs;
    }

    if (roi_result.status == "miss")
    {
        prob_sum_.fill(0.0f);
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
        if (kRecognitionVerboseLog)
        {
            std::cout << "[RECOG] recognition exit: miss, fallback_state="
                      << vision_code_text(current_vision_code_)
                      << ", " << reject_text << std::endl;
        }
        mode_ = Mode::NORMAL;
        last_perf_sample_.process_recog_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
        return;
    }

    if (roi_result.status != "rotated_roi")
    {
        prob_sum_.fill(0.0f);
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
        if (kRecognitionVerboseLog)
        {
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
        }
        mode_ = Mode::NORMAL;
        last_perf_sample_.process_recog_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
        return;
    }

    current_vision_code_ = BoardVisionCode::NO_RESULT;

    const auto classify_begin = steady_clock_t::now();
    last_perf_sample_.classify_total_called = true;
    const size_t active_class_count = std::min(class_names_.size(), kRecognitionMaxClasses);
    const RoiQualityMetrics quality =
        ComputeLowInformationRoiMetrics(roi_result.roi_bgr, roi_method, roi_result);
    std::ostringstream quality_info;
    quality_info << "quality: " << quality.reason;
    if (render_debug)
    {
        cv::putText(view, quality_info.str(), cv::Point(16, 140), cv::FONT_HERSHEY_SIMPLEX,
                    0.55, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
    }

    const auto infer_begin = steady_clock_t::now();
    const RoiClassificationResult cls =
        classify_roi_index(net_, roi_result.roi_bgr, calibration_temperature_, logit_bias_);
    const auto infer_end = steady_clock_t::now();
    const double infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_begin).count();
    last_perf_sample_.onnx_infer_ms = infer_ms;
    last_perf_sample_.onnx_infer_called = true;
    if (cls.predicted_index >= 0 && cls.predicted_index < static_cast<int>(class_names_.size()))
    {
        for (size_t i = 0; i < active_class_count; ++i)
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
            summarize_probabilities(prob_sum_, active_class_count, valid_frame_count_);
        std::ostringstream prob_info;
        prob_info << "avg_probs:";
        for (size_t i = 0; i < active_class_count; ++i)
        {
            prob_info << (i == 0 ? " " : "/")
                      << std::fixed << std::setprecision(2)
                      << prob_sum_[i] / static_cast<float>(valid_frame_count_);
        }
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
        if (kRecognitionVerboseLog)
        {
            std::cout << "[RECOG] frame_pred=" << name
                      << ", infer_ms=" << std::fixed << std::setprecision(2) << infer_ms
                      << ", valid_frames=" << valid_frame_count_ << "/" << max_valid_frames_
                      << ", roi_method=" << RoiMethodName(roi_method) << std::endl;
        }
    }
    last_perf_sample_.classify_total_ms =
        std::chrono::duration<double, std::milli>(steady_clock_t::now() - classify_begin).count();

    std::ostringstream vote_info;
    vote_info << "RECOG valid: " << valid_frame_count_ << "/" << max_valid_frames_;
    if (render_debug)
    {
        cv::putText(view, vote_info.str(), cv::Point(16, 84), cv::FONT_HERSHEY_SIMPLEX,
                    0.75, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }

    const ProbabilityDecisionSummary prob_summary =
        summarize_probabilities(prob_sum_, active_class_count, valid_frame_count_);
    const bool early_ready =
        valid_frame_count_ >= min_valid_frames_ &&
        prob_summary.top1_avg >= decision_top1_threshold_ &&
        prob_summary.margin >= decision_margin_threshold_;
    const bool max_frames_reached = valid_frame_count_ >= max_valid_frames_;
    const bool timeout = t_ms >= recognition_timeout_ms_;
    if (!early_ready && !max_frames_reached && !timeout)
    {
        last_perf_sample_.process_recog_total_ms =
            std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
        return;
    }

    // [Recognition Chain Step 5] 概率累积收敛。
    // 作用：从当前累计的有效推理帧里选最终类别，再映射到后续车体策略。
    std::string label = "no_decision";
    TargetClass target = TargetClass::UNKNOWN;
    bool has_final_decision = false;
    BoardVisionCode final_code = BoardVisionCode::NO_RESULT;
    std::string failure_reason = "low_confidence_or_margin_reject";
    if (valid_frame_count_ < min_valid_frames_ && timeout)
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

    if (kRecognitionTextLog)
    {
        std::cout << "[RECOG] result=" << label
                  << ", valid_frames=" << valid_frame_count_
                  << ", top1_avg=" << std::fixed << std::setprecision(4) << prob_summary.top1_avg
                  << ", margin=" << prob_summary.margin;
        if (!has_final_decision)
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

    if (kRecognitionTextLog)
    {
        std::cout << "[RECOG] state_out=" << vision_code_text(current_vision_code_)
                  << ", blob_area=" << std::fixed << std::setprecision(1) << current_blob_area_
                  << std::endl;
    }

    mode_ = Mode::NORMAL;
    prob_sum_.fill(0.0f);
    valid_frame_count_ = 0;
    last_perf_sample_.process_recog_total_ms =
        std::chrono::duration<double, std::milli>(steady_clock_t::now() - process_begin).count();
}
