#include "main.hpp"
#include "recognition_chain.h"
#include "recognition_runtime.h"
#include "stream_chain.h"

#include <fcntl.h>
#include <iostream>
#include <unistd.h>

using namespace cv;

// ==================== 全局对象与外部声明 ====================
Camera                    camera;
TransmissionStreamServer  server;
Mat                       img;
Mat                       view;

namespace {

// ==================== 系统初始化辅助函数 ====================
// 功能: 系统初始化
// 类型: 局部功能函数
// 关键参数: 无
// 说明：识别板不再承担巡线/控制职责，上电后固定进入“彩色采集 + 识别链 + UART 发包”。
void system_init()
{
    printf("Initializing recognition board...\n");
    camera.init(recognition_runtime::kRecognitionFrameWidth,
                recognition_runtime::kRecognitionFrameHeight,
                recognition_runtime::kRecognitionFrameFps);
    if (!comm.init(UART1, B115200))
    {
        printf("[BoardComm] 识别板串口发送未就绪，先继续跑识别链。\n");
    }
    printf("Recognition board init done.\n");
}

} // namespace

// ==================== 主函数 ====================
int main(int argc, char** argv)
{
    // 系统初始化
    system_init();

    // 设置终端为非阻塞（用于按键 'c' 快速复位识别链状态）
    {
        const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags != -1)
        {
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        }
    }

    std::cout << "输入 c: 手动复位识别链状态" << std::endl;

    // 识别板保留图传开关和识别开关两个入口
    const bool stream_enabled =
        StreamChain::ParseSwitch(argc, argv, StreamChain::DefaultEnabled());
    const bool recognition_enabled =
        RecognitionChain::ParseSwitch(argc, argv, RecognitionChain::DefaultEnabled());

    std::cout << "[BOOT] stream switch=" << (stream_enabled ? "on" : "off")
              << " (args: --stream / --no-stream / --stream=on|off)" << std::endl;
    std::cout << "[BOOT] recognition switch=" << (recognition_enabled ? "on" : "off")
              << " (args: --recognition / --no-recognition / --recognition=on|off)" << std::endl;
    std::cout << "[BOOT] recognition board camera="
              << recognition_runtime::kRecognitionFrameWidth << "x"
              << recognition_runtime::kRecognitionFrameHeight << "@"
              << recognition_runtime::kRecognitionFrameFps
              << " color, UART1@115200" << std::endl;

    // 启动识别板独立运行时（阻塞运行）
    RunRecognitionBoard(stream_enabled, recognition_enabled);
    return 0;
}
