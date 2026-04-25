#include "main.hpp"
#include "recognition_chain.h"
#include "recognition_runtime.h"
#include "stream_chain.h"

#include <fcntl.h>
#include <iostream>
#include <unistd.h>

using namespace cv;

// ==================== 全局对象与外部声明 ====================
std::unique_ptr<lq_camera_ex> camera;
TransmissionStreamServer  server;
Mat                       img;
Mat                       view;

namespace {

// ==================== 系统初始化辅助函数 ====================
// 功能: 初始化识别板串口，并在首次失败后按固定节奏有限次重试
// 类型: 局部功能函数
// 关键参数: 无
// 说明：上电早期串口设备节点可能还没完全就绪；这里做有限重试，但不会无限阻塞识别板启动。
static bool init_board_comm_with_retry()
{
    if (comm.init(UART1, B115200))
    {
        return true;
    }

    for (int retry = 1; retry <= BW_BOARD_COMM_INIT_RETRY_TIMES; ++retry)
    {
        printf("[BoardComm] reconnecting... attempt %d/%d in %d ms\n",
               retry,
               BW_BOARD_COMM_INIT_RETRY_TIMES,
               BW_BOARD_COMM_INIT_RETRY_INTERVAL_MS);
        usleep((useconds_t)BW_BOARD_COMM_INIT_RETRY_INTERVAL_MS * 1000u);

        if (comm.init(UART1, B115200))
        {
            printf("[BoardComm] reconnect success on attempt %d/%d\n",
                   retry,
                   BW_BOARD_COMM_INIT_RETRY_TIMES);
            return true;
        }
    }

    printf("[BoardComm] reconnect stopped after %d failed attempts\n",
           BW_BOARD_COMM_INIT_RETRY_TIMES);
    return false;
}

// 功能: 系统初始化
// 类型: 局部功能函数
// 关键参数: 无
// 说明：识别板不再承担巡线/控制职责，上电后固定进入“彩色采集 + 识别链 + UART 发包”。
void system_init()
{
    printf("Initializing recognition board...\n");
    auto create_camera = [](lq_camera_format_t format) -> std::unique_ptr<lq_camera_ex> {
        return std::unique_ptr<lq_camera_ex>(new lq_camera_ex(
            recognition_runtime::kRecognitionFrameWidth,
            recognition_runtime::kRecognitionFrameHeight,
            recognition_runtime::kRecognitionFrameFps,
            format));
    };

#if BW_RECOG_CAMERA_TRY_0CPU_MJPG
    camera = create_camera(LQ_CAMERA_0CPU_MJPG);
    if (camera && camera->is_cam_opened())
    {
        printf("[Camera] using low-cpu MJPG mode.\n");
    }
    else
    {
        camera.reset();
        printf("[Camera] low-cpu MJPG unavailable, fallback to high MJPG.\n");
    }
#endif

    if (!camera)
    {
        camera = create_camera(LQ_CAMERA_HIGH_MJPG);
        if (camera && camera->is_cam_opened())
        {
            printf("[Camera] using high MJPG mode.\n");
        }
    }
    if (!camera || !camera->is_cam_opened())
    {
        printf("[Camera] 识别板相机打开失败。\n");
    }
#if BW_RECOG_CAMERA_USE_MANUAL_EXPOSURE
    else if (!camera->set_exposure_manual(BW_RECOG_CAMERA_MANUAL_EXPOSURE))
    {
        printf("[Camera] 手动曝光设置失败，当前仍沿用摄像头默认曝光模式。\n");
    }
    else
    {
        printf("[Camera] 手动曝光已设置为 %d。\n", BW_RECOG_CAMERA_MANUAL_EXPOSURE);
    }
#else
    else
    {
        printf("[Camera] 当前沿用摄像头默认自动曝光。\n");
    }
#endif
    if (!init_board_comm_with_retry())
    {
        printf("[BoardComm] tx uart unavailable, recognition will continue without board link\n");
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

    std::cout << "输入 c: 启动一次红块检测与识别链" << std::endl;

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
