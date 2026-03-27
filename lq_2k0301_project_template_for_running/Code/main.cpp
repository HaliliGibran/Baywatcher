/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-02-26
@修改内容：
@注意事项：注意查看路径的修改
@注意事项：TFT程序优先推荐使用硬件SPI部分, 也就是LQ_TFT18_dri部分, 使用前需加载对应驱动模块,
         双龙mini派中已提前添加, 未添加也可自行编译.
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "main.hpp"
#include "stream_chain.h"
#include "vision_runtime.h"
#include <fcntl.h>
#include <chrono>
#include <vector>

const float SAMPLE_PERIOD = 0.005f;

using namespace cv;

// ==================== 全局对象与外部声明 ====================
BayWatcher_Encoder        encoder_sys;
BayWatcher_Menu&          menu = BayWatcher_Menu::getInstance();
BayWatcher_ADC            ADC_sys;
BayWatcher_Motor          motor_sys;
BayWatcher_Key            key_sys;
BayWatcher_Speed_Ramper   speed_ramper;

Bay_PID_t                 PID;
Bay_ESC_t                 ESC;
Bay_IncPID_t              PID_Speed_L;
Bay_IncPID_t              PID_Speed_R;
Bay_PosPID_t              PID_Speed_F_L;
Bay_PosPID_t              PID_Speed_F_R;
Bay_PosPID_t              PID_Angle;
Bay_PosPID_t              PID_YawSpeed;
Bay_CubePID_t             PID_Cube;

BayWatcher_VOFA           VOFA;
BayWatcher_ESC            esc_sys;
BayWatcher_IMU            imu_sys;

BayWatcher_TOF            tof_left;
BayWatcher_TOF            tof_right;
TOF_StateMachine          tof_manager(tof_left, tof_right);

Camera                    camera;
TransmissionStreamServer  server;
Mat                       img;
Mat                       binimg;
Mat                       view;

BayWatcher_TargetHandler  handler_sys;
BayWatcher_Logger         Logger;

namespace {

// ==================== 系统初始化辅助函数 ====================
// 功能: 获取双板通信所用的当前时间戳（毫秒）
// 类型: 局部功能函数
// 关键参数: 无
uint64_t board_comm_now_ms()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

// 功能: 系统初始化
// 类型: 局部功能函数
// 关键参数: 无
// 说明：运行板上电后固定进入“灰度巡线 + 串口收包”职责，不再初始化本地识别链。
void system_init()
{
    printf("Innitializing...\n");
    encoder_sys.init();
    encoder_sys.setMethod(SpeedMethod::M_Method);
    key_sys.init();
    ADC_sys.init();
    esc_sys.init();
    menu.init();
    BayWatcher_Control_Init();
    camera.init(vision_runtime::kLineFrameWidth,
                vision_runtime::kLineFrameHeight,
                vision_runtime::kLineFrameFps);
    handler_sys.init();
    imu_sys.init(("/dev/lq_i2c_mpu6050"));
    if (!comm.init(UART1, B115200))
    {
        printf("[BoardComm] 运行板串口接收未就绪，先继续跑巡线控制。\n");
    }
    image_remote_recognition_reset();
    printf("Initialization Success.\n");
}

} // namespace

// ====================== 线程任务  ===================
// 功能: 传感器轮询任务
// 类型: 线程任务函数
// 关键参数: arg-线程入口保留参数（未使用）
void task_sensor_read(void* arg)
{
    key_sys.Tick();
    if (PID.is_running)
    {
    }
}

void task_display_status(void* arg) {} // ADC/TFT 预留

// 功能: VOFA 通讯与调参任务
// 类型: 线程任务函数
// 关键参数: arg-线程入口保留参数（未使用）
void task_vofa_comm(void* arg)
{
    // 如果电机已经启动且通讯尚未建立，则开始尝试连接
    if (PID.is_running && !VOFA.is_initialized)
    {
        VOFA.init_UDP("172.20.10.2", 8888);
        if (VOFA.is_initialized)
        {
            printf("Motor Started: Communication Successfully Initialized.\n");
        }
    }

    // 通讯建立后，处理收发逻辑
    if (VOFA.is_initialized)
    {
        VOFA.Process_Incoming();
        if (PID.is_running)
        {
            VOFA.Send_Data(PID.target_speed_L, vL, PID.pwm_out_L,
                           PID.target_speed_R, vR, PID.pwm_out_R,
                           pure_angle, PID.yaw_control_base_speed,
                           average_curvature, preview_img_y);
        }
    }
}

// 双板通信接收线程：运行板周期读取识别板动作包
// 功能: 双板动作包接收任务
// 类型: 线程任务函数
// 关键参数: arg-线程入口保留参数（未使用）
// 说明：这里只负责把串口包喂给远端动作状态层，不直接改控制线程逻辑。
void task_board_comm_rx(void* arg)
{
    BoardActionEvent action = BoardActionEvent::NONE;
    uint8_t seq = 0;
    if (!comm.try_receive_event(&action, &seq))
    {
        return;
    }

    // 串口线程只做远端事件收口：
    // - vehicle 在这里锁存保持航向
    // - weapon/supply 在这里触发本地 TargetHandler
    // 真正的 pure_angle 接管仍由 vision_runtime / TargetHandler 统一消费。
    image_remote_recognition_apply_event(action, seq, pure_angle, board_comm_now_ms());
}

// ==================== 主函数 ====================
int main(int argc, char** argv)
{
    // 确认设备并挂载驱动
    std::vector<std::string> required_modules = {
        "lq_i2c_all_dev",
        "TFT18_dev",
        "TFT18_dri"
    };

    lq_module_load loader;
    printf("--- Loading Kernel Modules ---\n");
    bool all_loaded = true;
    for (const auto& mod : required_modules) {
        if (!loader.set_load_module(mod)) {
            printf("[ERROR] Failed to load module: %s\n", mod.c_str());
            all_loaded = false;
        }
    }

    if (!all_loaded) {
        printf("[ERROR] 部分硬件模块加载失败！请检查 .ko 文件是否在 /home/ 目录下。\n");
        return -1;
    }
    printf("--- Modules Loaded Successfully ---\n");

    // 系统初始化
    system_init();

    // 开启线程
    TimerThread pid_thread(BayWatcher_Control_Loop, NULL, 2);      // 底层速度控制
    TimerThread turn_thread(BayWatcher_Cube_Loop, NULL, 10);       // 外环 pure_angle -> speed_adjust
    TimerThread sensor_thread(task_sensor_read, NULL, 20);         // 按键/传感器轮询
    TimerThread vofa_thread(task_vofa_comm, NULL, 20);             // VOFA+日志观测
    TimerThread board_comm_thread(task_board_comm_rx, NULL, 5);    // 双板串口收包

    sensor_thread.start();
    vofa_thread.start();
    turn_thread.start();
    board_comm_thread.start();

    // 只允许启动一次控制线程，避免重复启动同一线程造成伪快但顿挫的错误状态
    if (pid_thread.start())
    {
        printf("速度控制线程已启动，采样周期: 2ms\n");
    }
    else
    {
        printf("线程启动失败！请检查是否链接了 pthread 库。\n");
        return -1;
    }

    // 设置终端为非阻塞（用于按键 'c' 快速复位赛道状态）
    {
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags != -1) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
    std::cout << "输入 c: 手动复位当前状态量" << std::endl;

    // 运行板只保留图传开关，识别由识别板负责
    const bool stream_enabled =
        StreamChain::ParseSwitch(argc, argv, StreamChain::DefaultEnabled());
    std::cout << "[BOOT] stream switch=" << (stream_enabled ? "on" : "off")
              << " (args: --stream / --no-stream / --stream=on|off)" << std::endl;
    std::cout << "[BOOT] running board camera="
              << vision_runtime::kLineFrameWidth << "x"
              << vision_runtime::kLineFrameHeight << "@"
              << vision_runtime::kLineFrameFps
              << " gray, UART1@115200" << std::endl;

    // 启动视觉系统（阻塞运行）
    Vision_System_Run(stream_enabled);
    return 0;
}
