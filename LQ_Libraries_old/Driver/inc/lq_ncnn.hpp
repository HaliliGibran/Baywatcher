#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <ncnn/net.h>

class LQ_NCNN
{
public:
    LQ_NCNN();
    bool Init();

    // 仅保留这个引用参数，用来带出计算好的标准置信度 (0.0~1.0)
    std::string Infer(const cv::Mat& bgr_image, float& out_confidence);

    void SetModelPath(const std::string& param_path, const std::string& bin_path);
    void SetInputSize(int width, int height);
    void SetLabels(const std::vector<std::string>& labels);
    void SetNormalize(const float mean_vals[3], const float norm_vals[3]);
    ~LQ_NCNN();

private:
    // 恢复原版：只负责极其纯粹地找最大值索引
    int Argmax(const ncnn::Mat& logits);

private:
    ncnn::Net                 m_net;              
    std::vector<std::string>  m_labels;           
    bool                      m_initialized;      

    std::string               m_param_path;
    std::string               m_bin_path;
    int                       m_input_width;
    int                       m_input_height;
    float                     m_mean_vals[3];
    float                     m_norm_vals[3];
    std::string               m_input_name;
    std::string               m_output_name;
};

// #pragma once

// #include <string>
// #include <vector>
// #include <opencv2/core.hpp>
// #include <ncnn/net.h>

// class LQ_NCNN
// {
// public:
//     LQ_NCNN();
//     bool Init();

//     // 推理并输出真实的 Softmax 置信度 (0.0~1.0)
//     std::string Infer(const cv::Mat& bgr_image, float& out_confidence);

//     void SetModelPath(const std::string& param_path, const std::string& bin_path);
//     void SetInputSize(int width, int height);
//     void SetLabels(const std::vector<std::string>& labels);
//     void SetNormalize(const float mean_vals[3], const float norm_vals[3]);
//     ~LQ_NCNN();

// private:
//     int Argmax(const ncnn::Mat& logits);

// private:
//     ncnn::Net                 m_net;              
//     std::vector<std::string>  m_labels;           
//     bool                      m_initialized;      

//     std::string               m_param_path;
//     std::string               m_bin_path;
//     int                       m_input_width;
//     int                       m_input_height;
//     float                     m_mean_vals[3];
//     float                     m_norm_vals[3];
//     std::string               m_input_name;
//     std::string               m_output_name;
// };