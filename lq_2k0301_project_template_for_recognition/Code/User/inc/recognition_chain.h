#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include "Communication.h"
#include "roi_runtime_geometry.h"

class RecognitionChain
{
public:
    static constexpr size_t kMaxModelClasses = 8;

    struct PerfSample
    {
        double ultra_precheck_ms = 0.0;
        double hsv_precheck_ms = 0.0;
        double extract_roi_ms = 0.0;
        double roi_search_rect_ms = 0.0;
        double roi_track_boundary_ms = 0.0;
        double roi_red_mask_ms = 0.0;
        double roi_red_band_ms = 0.0;
        double roi_track_classify_ms = 0.0;
        double roi_build_warp_ms = 0.0;
        double onnx_infer_ms = 0.0;
        double onnx_preprocess_ms = 0.0;
        double onnx_set_input_ms = 0.0;
        double onnx_forward_ms = 0.0;
        double onnx_postprocess_ms = 0.0;
        double classify_total_ms = 0.0;
        double try_total_ms = 0.0;
        double process_recog_total_ms = 0.0;
        bool ultra_precheck_called = false;
        bool hsv_precheck_called = false;
        bool extract_roi_called = false;
        bool onnx_infer_called = false;
        bool classify_total_called = false;
        bool try_total_called = false;
        bool process_recog_total_called = false;
    };

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
    // 运行板处于 CIRCLE_RUNNING 时启用环岛专用 marker 质量门控。
    void SetCircleRunningMode(bool active);

    bool IsEnabled() const;
    bool IsInRecognitionMode() const;
    bool IsIdleNoTargetState() const;
    bool HasRecentRedCandidate(uint64_t t_ms) const;
    bool IsLatchedHoldingResult() const;
    BoardVisionCode GetCurrentVisionCode() const;
    double GetCurrentBlobArea() const;
    const PerfSample& GetLastPerfSample() const;
    // [Recognition Chain Step 2-3] 在普通态里检测红色触发器并切入识别态。
    // 作用：识别链自己管理 NORMAL -> RECOGNITION 的切换，并进入自适应判定。
    bool TryEnterRecognition(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view, bool render_debug);
    bool HasPendingTriggerRoiForImmediateInference() const;
    void ProcessPendingTriggerRoi(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view, bool render_debug);
    // [Recognition Chain Step 4-5A] 识别态自适应 1/2 帧推理并给出结果。
    // 作用：达到门槛时单帧输出，否则第二个有效推理帧按两帧平均 top1 强制输出。
    void ProcessRecognitionFrame(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view, bool render_debug);

private:
    void ClearAdaptiveDecision();

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
    std::array<float, kMaxModelClasses> logit_bias_;
    float calibration_temperature_;
    float decision_top1_threshold_;
    float decision_margin_threshold_;
    Mode mode_;
    BoardVisionCode current_vision_code_;
    BoardVisionCode latched_symbol_code_;
    uint64_t latched_release_deadline_ms_;
    bool latched_release_pending_;
    double current_blob_area_;
    uint64_t recent_red_candidate_until_ms_;
    bool adaptive_decision_pending_;
    std::array<float, kMaxModelClasses> adaptive_prob_sum_;
    int adaptive_valid_frame_count_;
    int adaptive_bad_frame_count_;
    bool circle_running_mode_;
    int circle_marker_quality_pass_frames_;
    uint64_t circle_marker_quality_last_log_ms_;
    bool pending_trigger_roi_valid_;
    RoiExtractionResult pending_trigger_roi_;
    PerfSample pending_trigger_perf_;
    PerfSample last_perf_sample_;
};
