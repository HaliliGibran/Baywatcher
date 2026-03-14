#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include "image_data.h"

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
    // [Recognition Chain Step 6] 输出当前识别结果对应的巡线覆盖指令。
    // 作用：把识别标签转换为 FORCE_LEFT / FORCE_RIGHT / NONE。
    // 若识别后策略开关关闭，则始终返回 NONE。
    RecognitionFollowOverride GetFollowOverride() const;
    // [Recognition Chain Step 7] 输出 vehicle 类别对应的航向保持画面和目标角。
    // 作用：在短时保持窗口内直接接管 pure_angle。
    // 若识别后策略开关关闭，则不会进入该分支。
    bool TryRenderVehicleHold(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view, float* hold_yaw) const;
    // [Recognition Chain Step 2-3] 在普通态里检测红色触发器并切入识别态。
    // 作用：识别链自己管理 NORMAL -> RECOGNITION 的切换。
    bool TryEnterRecognition(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view);
    // [Recognition Chain Step 4-5A] 识别态逐帧推理并在投票收敛后给出结果。
    // 作用：处理 ROI 分类、投票、类别映射和退出识别态。
    void ProcessRecognitionFrame(const cv::Mat& frame_bgr, uint64_t t_ms, float current_pure_angle, cv::Mat& view);

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
    std::deque<int> votes_;
    TargetClass last_result_;
    std::string last_label_;
    int required_votes_;
    Mode mode_;
    uint64_t recognition_timeout_ms_;
    uint64_t trigger_cooldown_until_ms_;
    uint64_t vehicle_hold_until_ms_;
    float vehicle_hold_yaw_;
};
