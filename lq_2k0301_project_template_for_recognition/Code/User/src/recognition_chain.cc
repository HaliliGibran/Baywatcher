#include "recognition_chain.h"

#include "common.h"
#include "image_switch_utils.h"
#include "transform_table.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <regex>
#include <sstream>
#include <unistd.h>

namespace {

constexpr uint64_t kRecognitionCooldownMs = 3000;
constexpr uint64_t kRecognitionTriggerRejectLogIntervalMs = 300;
constexpr int kRecognitionTriggerFrameWidth = 640;
constexpr int kRecognitionTriggerFrameHeight = 480;

static bool red_rect_center_y_in_trigger_band(const cv::Rect& rect, int* center_y_out = nullptr)
{
    const int center_y = rect.y + rect.height / 2;
    if (center_y_out != nullptr)
    {
        *center_y_out = center_y;
    }

    const int min_y = BW_RECOG_TRIGGER_CENTER_Y - BW_RECOG_TRIGGER_CENTER_Y_TOL;
    const int max_y = BW_RECOG_TRIGGER_CENTER_Y + BW_RECOG_TRIGGER_CENTER_Y_TOL;
    return center_y >= min_y && center_y <= max_y;
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

static BoardActionEvent action_event_from_target_code(uint8_t target_code)
{
    switch (target_code)
    {
    case 1: return BoardActionEvent::WEAPON;
    case 2: return BoardActionEvent::SUPPLY;
    case 3: return BoardActionEvent::VEHICLE;
    default: return BoardActionEvent::NONE;
    }
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
static int classify_roi_index(cv::dnn::Net& net, const cv::Mat& roi_bgr)
{
    cv::Mat resized;
    cv::resize(roi_bgr, resized, cv::Size(64, 64), 0, 0, cv::INTER_AREA);
    cv::Mat blob = cv::dnn::blobFromImage(resized, 1.0, cv::Size(64, 64), cv::Scalar(), true, false);
    net.setInput(blob);
    cv::Mat out = net.forward().reshape(1, 1);
    cv::Mat out_f;
    out.convertTo(out_f, CV_32F);
    cv::Point class_id;
    double score = 0.0;
    cv::minMaxLoc(out_f, nullptr, &score, nullptr, &class_id);
    (void)score;
    return class_id.x;
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
      required_votes_(8),
      mode_(Mode::NORMAL),
      recognition_timeout_ms_(0),
      trigger_cooldown_until_ms_(0),
      event_armed_(true),
      next_event_seq_(0),
      pending_tx_action_(BoardActionEvent::NONE),
      pending_tx_seq_(0),
      pending_tx_repeat_remain_(0)
{
}

bool RecognitionChain::Initialize(bool enabled_by_switch)
{
    const std::string configured_model_path = "./model/cls.onnx";
    const std::string configured_class_path = "./model/class_names.json";
    const std::string model_path = resolve_runtime_path(configured_model_path);
    const std::string class_path = resolve_runtime_path(configured_class_path);

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
        std::cout << "[RECOG] event map:"
                  << " weapon->WEAPON, supply->SUPPLY, vehicle->VEHICLE" << std::endl;
    }
    else
    {
        std::cout << "[ONNX] disabled, waiting for valid model/class files." << std::endl;
    }
    return enabled_;
}

void RecognitionChain::Reset()
{
    votes_.clear();
    mode_ = Mode::NORMAL;
    recognition_timeout_ms_ = 0;
    trigger_cooldown_until_ms_ = 0;
    event_armed_ = true;
    next_event_seq_ = 0;
    pending_tx_action_ = BoardActionEvent::NONE;
    pending_tx_seq_ = 0;
    pending_tx_repeat_remain_ = 0;
}

bool RecognitionChain::IsEnabled() const
{
    return enabled_;
}

bool RecognitionChain::IsInRecognitionMode() const
{
    return mode_ == Mode::RECOGNITION;
}

bool RecognitionChain::TryEnterRecognition(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view)
{
    if (!enabled_ || mode_ != Mode::NORMAL || t_ms < trigger_cooldown_until_ms_)
    {
        return false;
    }

    // [Recognition Chain Step 2] 红色触发器。
    // 作用：只有“红色矩形出现 + 模型可用 + 冷却结束”才进入识别态。
    cv::Rect trigger_rect;
    cv::Mat trigger_mask;
    if (!detect_red_rect_like(frame_bgr, &trigger_rect, &trigger_mask))
    {
        // 纯事件流模式下，只有目标离场回到 NONE 后，才重新允许下一次同类事件触发。
        event_armed_ = true;
        return false;
    }

    if (!event_armed_)
    {
        return false;
    }

    int center_y = 0;
    if (!red_rect_center_y_in_trigger_band(trigger_rect, &center_y))
    {
        static uint64_t last_reject_log_ms = 0;
        if (t_ms >= last_reject_log_ms + kRecognitionTriggerRejectLogIntervalMs)
        {
            last_reject_log_ms = t_ms;
            std::cout << "[RECOG] trigger rejected: center_y=" << center_y
                      << ", expect=" << BW_RECOG_TRIGGER_CENTER_Y
                      << "+/-" << BW_RECOG_TRIGGER_CENTER_Y_TOL << std::endl;
        }
        return false;
    }

    float top_width = 0.0f;
    float bottom_width = 0.0f;
    float left_height = 0.0f;
    float right_height = 0.0f;
    if (!red_rect_is_horizontal_ipm_rect(trigger_rect,
                                         &top_width,
                                         &bottom_width,
                                         &left_height,
                                         &right_height))
    {
        static uint64_t last_ipm_reject_log_ms = 0;
        if (t_ms >= last_ipm_reject_log_ms + kRecognitionTriggerRejectLogIntervalMs)
        {
            last_ipm_reject_log_ms = t_ms;
            std::cout << "[RECOG] trigger rejected: ipm_rect top=" << std::fixed << std::setprecision(2)
                      << top_width << ", bottom=" << bottom_width
                      << ", left=" << left_height << ", right=" << right_height
                      << ", min_ratio=" << BW_RECOG_TRIGGER_IPM_MIN_WIDTH_HEIGHT_RATIO << std::endl;
        }
        return false;
    }

    // [Recognition Chain Step 3] 进入识别态。
    // 作用：开始累计识别投票。
    mode_ = Mode::RECOGNITION;
    votes_.clear();
    recognition_timeout_ms_ = t_ms + 2500;

    view = frame_bgr.clone();
    cv::rectangle(view, trigger_rect, cv::Scalar(0, 255, 255), 2);
    cv::putText(view, "TRIGGER -> RECOGNITION", cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.65, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    return true;
}

void RecognitionChain::ProcessRecognitionFrame(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view)
{
    if (!enabled_ || mode_ != Mode::RECOGNITION)
    {
        view = frame_bgr.clone();
        return;
    }

    // [Recognition Chain Step 4] 识别态逐帧推理。
    // 作用：持续在原始 640x480 图上截红色 ROI，执行 ONNX 分类并累计投票。
    view = frame_bgr.clone();
    cv::Rect red_rect;
    cv::Mat red_mask;
    const bool found = detect_red_rect_like(frame_bgr, &red_rect, &red_mask);

    if (found)
    {
        cv::rectangle(view, red_rect, cv::Scalar(0, 255, 255), 2);
        cv::Mat roi = frame_bgr(red_rect).clone();
        const auto infer_begin = std::chrono::steady_clock::now();
        const int pred = classify_roi_index(net_, roi);
        const auto infer_end = std::chrono::steady_clock::now();
        const double infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_begin).count();
        if (pred >= 0 && pred < static_cast<int>(class_names_.size()))
        {
            votes_.push_back(pred);
            const std::string& name = class_names_[pred];
            cv::putText(view, std::string("pred: ") + name, cv::Point(10, 54), cv::FONT_HERSHEY_SIMPLEX,
                        0.75, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            std::ostringstream infer_info;
            infer_info << "infer: " << std::fixed << std::setprecision(2) << infer_ms << " ms";
            cv::putText(view, infer_info.str(), cv::Point(10, 82), cv::FONT_HERSHEY_SIMPLEX,
                        0.65, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
            std::cout << "[RECOG] frame_pred=" << name
                      << ", infer_ms=" << std::fixed << std::setprecision(2) << infer_ms
                      << ", votes=" << votes_.size() << "/" << required_votes_ << std::endl;
        }
    }

    std::ostringstream vote_info;
    vote_info << "RECOG votes: " << votes_.size() << "/" << required_votes_;
    cv::putText(view, vote_info.str(), cv::Point(10, 26), cv::FONT_HERSHEY_SIMPLEX,
                0.75, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    const bool vote_done = votes_.size() >= static_cast<size_t>(required_votes_);
    const bool timeout = t_ms >= recognition_timeout_ms_;
    if (!vote_done && !timeout)
    {
        return;
    }

    // [Recognition Chain Step 5] 投票收敛。
    // 作用：从多帧结果里选最终类别，再映射到后续车体策略。
    int best_idx = -1;
    int best_count = -1;
    std::vector<int> counts(class_names_.size(), 0);
    for (size_t i = 0; i < votes_.size(); ++i)
    {
        const int v = votes_[i];
        if (v >= 0 && v < static_cast<int>(counts.size()))
        {
            counts[v]++;
        }
    }
    for (size_t i = 0; i < counts.size(); ++i)
    {
        if (counts[i] > best_count)
        {
            best_count = counts[i];
            best_idx = static_cast<int>(i);
        }
    }

    std::string label = "unknown";
    TargetClass target = TargetClass::UNKNOWN;
    if (best_idx >= 0 && best_idx < static_cast<int>(class_names_.size()))
    {
        label = class_names_[best_idx];
        target = static_cast<TargetClass>(parse_target_class_code(label));
    }

    const BoardActionEvent action_event =
        action_event_from_target_code(static_cast<uint8_t>(target));
    if (action_event != BoardActionEvent::NONE)
    {
        pending_tx_action_ = action_event;
        pending_tx_seq_ = next_event_seq_++;
        pending_tx_repeat_remain_ = static_cast<uint8_t>(BW_BOARD_EVENT_REPEAT_FRAMES);
        event_armed_ = false;
    }

    cv::putText(view, std::string("result: ") + label, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX,
                0.70, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    std::cout << "[RECOG] result=" << label
              << ", votes=" << best_count << "/" << votes_.size();
    if (action_event != BoardActionEvent::NONE)
    {
        std::cout << " -> event(action=" << static_cast<int>(action_event)
                  << ", seq=" << static_cast<int>(pending_tx_seq_)
                  << ", repeat=" << static_cast<int>(pending_tx_repeat_remain_) << ")";
    }
    std::cout << std::endl;

    mode_ = Mode::NORMAL;
    trigger_cooldown_until_ms_ = t_ms + kRecognitionCooldownMs;
}

bool RecognitionChain::TryGetNextTxEvent(BoardActionEvent* action, uint8_t* seq)
{
    if (action == nullptr || seq == nullptr)
    {
        return false;
    }

    if (pending_tx_action_ == BoardActionEvent::NONE || pending_tx_repeat_remain_ == 0)
    {
        return false;
    }

    *action = pending_tx_action_;
    *seq = pending_tx_seq_;

    pending_tx_repeat_remain_--;
    if (pending_tx_repeat_remain_ == 0)
    {
        pending_tx_action_ = BoardActionEvent::NONE;
    }

    return true;
}
