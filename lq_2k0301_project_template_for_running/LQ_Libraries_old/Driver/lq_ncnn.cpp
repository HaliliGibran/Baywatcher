#include "lq_ncnn.hpp"
#include <limits>
#include <stdexcept>
#include <cmath> 
#include <opencv2/imgproc.hpp>

LQ_NCNN::LQ_NCNN()
    : m_initialized(false), m_input_width(96), m_input_height(96)
    , m_input_name("in0"), m_output_name("out0")
{
    m_mean_vals[0] = 123.675f; m_mean_vals[1] = 116.28f; m_mean_vals[2] = 103.53f;
    m_norm_vals[0] = 0.01712475f; m_norm_vals[1] = 0.017507f; m_norm_vals[2] = 0.01742919f;
}

bool LQ_NCNN::Init()
{
    m_net.opt.use_vulkan_compute = false;
    m_net.opt.num_threads = 1;

    if (m_param_path.empty() || m_net.load_param(m_param_path.c_str()) != 0) return false;
    if (m_bin_path.empty() || m_net.load_model(m_bin_path.c_str()) != 0) return false;

    m_initialized = true;
    return true;
}

std::string LQ_NCNN::Infer(const cv::Mat& bgr_image, float& out_confidence)
{
    if (!m_initialized) throw std::runtime_error("NCNN not initialized.");
    if (bgr_image.empty()) throw std::invalid_argument("Input image empty.");

    cv::Mat resized, rgb;
    cv::resize(bgr_image, resized, cv::Size(m_input_width, m_input_height));
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    ncnn::Mat input = ncnn::Mat::from_pixels(rgb.data, ncnn::Mat::PIXEL_RGB, m_input_width, m_input_height);
    input.substract_mean_normalize(m_mean_vals, m_norm_vals);

    ncnn::Extractor ex = m_net.create_extractor();
    ex.input(m_input_name.c_str(), input);

    ncnn::Mat logits;
    ex.extract(m_output_name.c_str(), logits);

    // 调用原版 Argmax 找到最大值的索引
    int class_id = Argmax(logits);
    if (class_id < 0) throw std::runtime_error("Failed to get class id.");

    // =================================================================
    // 【核心修复】：手动执行 Softmax 计算，将 Logits 转换为真实的置信度概率
    // 公式: P(i) = exp(x_i - max_x) / sum(exp(x_j - max_x))
    // =================================================================
    float max_logit = logits[class_id]; 
    float sum_exp = 0.0f;
    for (int i = 0; i < logits.w; ++i) {
        sum_exp += std::exp(logits[i] - max_logit);
    }
    // 最终的真实置信度 (必定在 0.0 到 1.0 之间)
    out_confidence = std::exp(logits[class_id] - max_logit) / sum_exp;
    // =================================================================

    if (class_id >= 0 && class_id < static_cast<int>(m_labels.size())) {
        return m_labels[class_id];
    }
    return std::to_string(class_id);
}

void LQ_NCNN::SetModelPath(const std::string& param_path, const std::string& bin_path) {
    m_param_path = param_path; m_bin_path = bin_path;
}

void LQ_NCNN::SetInputSize(int width, int height) {
    m_input_width = width; m_input_height = height;
}

void LQ_NCNN::SetLabels(const std::vector<std::string>& labels) {
    m_labels = labels;
}

void LQ_NCNN::SetNormalize(const float mean_vals[3], const float norm_vals[3]) {
    for(int i=0; i<3; i++) { m_mean_vals[i] = mean_vals[i]; m_norm_vals[i] = norm_vals[i]; }
}

// 彻底恢复成原版的优雅实现，只负责找最大值
int LQ_NCNN::Argmax(const ncnn::Mat& logits)
{
    if (logits.w <= 0) return -1;

    int best_index = 0;
    float best_value = -std::numeric_limits<float>::infinity();

    for (int i = 0; i < logits.w; ++i) {
        if (logits[i] > best_value) {
            best_value = logits[i];
            best_index = i;
        }
    }
    return best_index;
}

LQ_NCNN::~LQ_NCNN() {}

// #include "lq_ncnn.hpp"
// #include <limits>
// #include <stdexcept>
// #include <cmath> 
// #include <opencv2/imgproc.hpp>

// LQ_NCNN::LQ_NCNN()
//     : m_initialized(false), m_input_width(96), m_input_height(96)
//     , m_input_name("in0"), m_output_name("out0")
// {
//     m_mean_vals[0] = 123.675f; m_mean_vals[1] = 116.28f; m_mean_vals[2] = 103.53f;
//     m_norm_vals[0] = 0.01712475f; m_norm_vals[1] = 0.017507f; m_norm_vals[2] = 0.01742919f;
// }

// bool LQ_NCNN::Init()
// {
//     m_net.opt.use_vulkan_compute = false;
//     m_net.opt.num_threads = 2; // 双核满载
//     m_net.opt.use_packing_layout = true; // 内存加速
//     m_net.opt.use_sgemm_convolution = true; // 矩阵加速

//     if (m_param_path.empty() || m_net.load_param(m_param_path.c_str()) != 0) return false;
//     if (m_bin_path.empty() || m_net.load_model(m_bin_path.c_str()) != 0) return false;

//     m_initialized = true;
//     return true;
// }

// std::string LQ_NCNN::Infer(const cv::Mat& bgr_image, float& out_confidence)
// {
//     if (!m_initialized) throw std::runtime_error("NCNN not initialized.");
//     if (bgr_image.empty()) throw std::invalid_argument("Input image empty.");

//     // 彻底干掉 OpenCV 前处理，直接用底层极速转换
//     ncnn::Mat input = ncnn::Mat::from_pixels_resize(
//         bgr_image.data,
//         ncnn::Mat::PIXEL_BGR2RGB,
//         bgr_image.cols, bgr_image.rows,
//         m_input_width, m_input_height
//     );

//     input.substract_mean_normalize(m_mean_vals, m_norm_vals);

//     ncnn::Extractor ex = m_net.create_extractor();
//     ex.input(m_input_name.c_str(), input);

//     ncnn::Mat logits;
//     ex.extract(m_output_name.c_str(), logits);

//     int class_id = Argmax(logits);
//     if (class_id < 0) throw std::runtime_error("Failed to get class id.");

//     // Softmax 计算置信度
//     float max_logit = logits[class_id]; 
//     float sum_exp = 0.0f;
//     for (int i = 0; i < logits.w; ++i) {
//         sum_exp += std::exp(logits[i] - max_logit);
//     }
//     out_confidence = std::exp(logits[class_id] - max_logit) / sum_exp;

//     if (class_id >= 0 && class_id < static_cast<int>(m_labels.size())) {
//         return m_labels[class_id];
//     }
//     return std::to_string(class_id);
// }

// void LQ_NCNN::SetModelPath(const std::string& param_path, const std::string& bin_path) { m_param_path = param_path; m_bin_path = bin_path; }
// void LQ_NCNN::SetInputSize(int width, int height) { m_input_width = width; m_input_height = height; }
// void LQ_NCNN::SetLabels(const std::vector<std::string>& labels) { m_labels = labels; }
// void LQ_NCNN::SetNormalize(const float mean_vals[3], const float norm_vals[3]) {
//     for(int i=0; i<3; i++) { m_mean_vals[i] = mean_vals[i]; m_norm_vals[i] = norm_vals[i]; }
// }
// int LQ_NCNN::Argmax(const ncnn::Mat& logits) {
//     if (logits.w <= 0) return -1;
//     int best_index = 0;
//     float best_value = -std::numeric_limits<float>::infinity();
//     for (int i = 0; i < logits.w; ++i) {
//         if (logits[i] > best_value) { best_value = logits[i]; best_index = i; }
//     }
//     return best_index;
// }
// LQ_NCNN::~LQ_NCNN() {}