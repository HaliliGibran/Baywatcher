#pragma once

#include <string>

struct RoiCaptureTransferConfig
{
    std::string host;
    int port = 0;
};

bool RoiCaptureModeDefaultEnabled();
bool ParseRoiCaptureModeSwitch(int argc, char** argv, bool default_value);
RoiCaptureTransferConfig ParseRoiCaptureTransferConfig(int argc, char** argv);

// 功能: ROI 拍摄专用运行时入口
// 类型: 图像运行时主循环
// 关键参数:
// - stream_enabled-是否启用图传
// - transfer_config-电脑端接收主机和端口
// 说明：
// - 该入口不进入模型识别链，只持续做 ROI 几何提取与人工确认发送。
void RunRoiCaptureBoard(bool stream_enabled, const RoiCaptureTransferConfig& transfer_config);
