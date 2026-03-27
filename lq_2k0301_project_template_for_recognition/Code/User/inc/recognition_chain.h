#pragma once

#include <cstdint>
#include <deque>
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
    // [Recognition Chain Step 2-3] 在普通态里检测红色触发器并切入识别态。
    // 作用：识别链自己管理 NORMAL -> RECOGNITION 的切换。
    bool TryEnterRecognition(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view);
    // [Recognition Chain Step 4-5A] 识别态逐帧推理并在投票收敛后给出结果。
    // 作用：处理 ROI 分类、投票、类别映射和退出识别态。
    void ProcessRecognitionFrame(const cv::Mat& frame_bgr, uint64_t t_ms, cv::Mat& view);
    // [Recognition Chain UART] 取出一帧待发送事件。
    // 作用：稳定识别结果只在边沿产生一次事件，然后重复发送固定帧数。
    bool TryGetNextTxEvent(BoardActionEvent* action, uint8_t* seq);

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
    int required_votes_;
    Mode mode_;
    uint64_t recognition_timeout_ms_;
    uint64_t trigger_cooldown_until_ms_;
    bool event_armed_;
    uint8_t next_event_seq_;
    BoardActionEvent pending_tx_action_;
    uint8_t pending_tx_seq_;
    uint8_t pending_tx_repeat_remain_;
};
