#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include "Communication.h"

class RecognitionChain
{
public:
    // [Recognition Chain Interface] 返回识别链默认开关状态。
    // 作用：让 main 侧无需关心编译期宏，只读取统一接口。
    static bool DefaultEnabled();
    // [Recognition Chain Interface] 解析识别链运行时开关。
    // 作用：统一处理 --recognition / --no-recognition 等命令行参数。
    static bool ParseSwitch(int argc, char** argv, bool default_value);

    RecognitionChain();

    // [Recognition Chain Step 1] 初始化模型识别链。
    // 作用：加载 ONNX 模型、类别表，并决定后续是否允许进入识别态。
    bool Initialize(bool enabled_by_switch);
    // [Recognition Chain Interface] 清空整条识别链内部状态。
    // 作用：用于手动复位或外部强制回到干净初始态。
    void Reset();

    bool IsEnabled() const;
    bool IsInRecognitionMode() const;
    BoardVisionCode GetCurrentVisionCode() const;
    double GetCurrentBlobArea() const;
    // [Recognition Chain Step 2-3] 在普通态里检测红色触发器并切入识别态。
    // 作用：识别链自己管理 NORMAL -> RECOGNITION 的切换，并开始概率累积。
    bool TryEnterRecognition(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view, bool render_debug);
    // [Recognition Chain Step 4-5A] 识别态逐帧推理并在概率累积收敛后给出结果。
    // 作用：处理 ROI 分类、概率累积、类别映射和退出识别态。
    void ProcessRecognitionFrame(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view, bool render_debug);

private:
    enum class TargetClass : uint8_t {
        UNKNOWN = 0,
        WEAPON,
        SUPPLY,
        VEHICLE,
    };

    enum class Mode : uint8_t {
        NORMAL = 0,
        RECOGNITION,
    };

private:
    bool enabled_;
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    std::array<float, 3> prob_sum_;
    std::array<float, 3> logit_bias_;
    int valid_frame_count_;
    int min_valid_frames_;
    int max_valid_frames_;
    float calibration_temperature_;
    float decision_top1_threshold_;
    float decision_margin_threshold_;
    Mode mode_;
    uint64_t recognition_timeout_ms_;
    BoardVisionCode current_vision_code_;
    BoardVisionCode latched_symbol_code_;
    uint64_t latched_release_deadline_ms_;
    bool latched_release_pending_;
    uint64_t normal_precheck_next_ms_;
    bool normal_precheck_positive_;
    double current_blob_area_;
};
