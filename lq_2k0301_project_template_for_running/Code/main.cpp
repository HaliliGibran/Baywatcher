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
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <vector>
const float SAMPLE_PERIOD = 0.005f;

using namespace cv;

// ==================== 全局对象与外部声明 ====================
lq_module_load            loader;

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
Bay_PosPID_t              PID_Speed_F_L;   // 前进环左轮
Bay_PosPID_t              PID_Speed_F_R;   // 前进环右轮
Bay_PosPID_t              PID_Angle;       // 最外环
Bay_PosPID_t              PID_YawSpeed;    // 中间环
Bay_CubePID_t             PID_Cube;        // 三次拟位置环

BayWatcher_VOFA           VOFA;
BayWatcher_ESC            esc_sys;
BayWatcher_IMU            imu_sys;

BayWatcher_TOF            tof_left;
BayWatcher_TOF            tof_right;
TOF_StateMachine          tof_manager(tof_left, tof_right);

BayWatcher_Buzzer         buzzer_sys;

Camera                    camera;          // 创建摄像头对象
TransmissionStreamServer  server;
Mat                       img;             // 创建OpenCV Mat对象(原始图像)
Mat                       binimg;          // 用于输出的图像
Mat                       view;            // 轨迹画板

BayWatcher_TargetHandler  handler_sys;

BayWatcher_Logger         Logger;

// ==================== 系统初始化 ====================
static struct termios g_stdin_termios;
static int g_stdin_flags = -1;
static bool g_stdin_termios_saved = false;

static void setup_nonblocking_keyboard_input()
{
    g_stdin_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (g_stdin_flags != -1) {
        fcntl(STDIN_FILENO, F_SETFL, g_stdin_flags | O_NONBLOCK);
    }

    if (tcgetattr(STDIN_FILENO, &g_stdin_termios) == 0) {
        struct termios raw = g_stdin_termios;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
            g_stdin_termios_saved = true;
        }
    }
}

static void restore_keyboard_input()
{
    if (g_stdin_flags != -1) {
        fcntl(STDIN_FILENO, F_SETFL, g_stdin_flags);
    }

    if (g_stdin_termios_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_stdin_termios);
    }
}

// 功能: 系统初始化
void system_init(){
    printf("Innitializing...\n");
    encoder_sys.init();
    encoder_sys.setMethod(SpeedMethod::M_Method);
    // encoder_sys.setMethod(SpeedMethod::T_Method);
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
    // tof_left.init(("/dev/lq_i2c_vl53l0x"));
    // handler_sys.setMethod(JudgeMethod::Fixed_Method);
    if (!comm.init(UART1, B115200))
    {
        printf("[BoardComm] rx uart unavailable, line tracking will continue without board link\n");
    }
    image_remote_recognition_reset();
    printf("Initialization Success.\n");
}

uint64_t board_comm_now_ms()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

static const char* board_comm_code_text(BoardVisionCode code)
{
    switch (code)
    {
    case BoardVisionCode::VEHICLE: return "v";
    case BoardVisionCode::WEAPON: return "w";
    case BoardVisionCode::SUPPLY: return "s";
    case BoardVisionCode::BRICK: return "b";
    case BoardVisionCode::BRICK_LEFT: return "bl";
    case BoardVisionCode::BRICK_RIGHT: return "br";
    case BoardVisionCode::NO_RESULT: return "u";
    case BoardVisionCode::CLOTH_STOP: return "c";
    case BoardVisionCode::UNKNOWN: return "n";
    default: return "-";
    }
}

// ====================== 线程任务  ===================
void task_sensor_read(void* arg) {
    key_sys.Tick();
    // printf("%f\n",vL);
    // printf("%f\n",vR);
    // printf("%d\n",imu_sys.raw_gz);
    if (PID.is_running){
        // tof_left.Update();
        // xyz = tof_left.Get_Filtered_Distance();
        // printf("%d\n",xyz);
        // // tof_right.Update();
        // tof_manager.Update_State();
        // TrackElement current_elem = tof_manager.Get_Current_Element();
        // if (current_elem == TrackElement::OBSTACLE_DOUBLE) {
        //     // 应对正前方横杆/墙
        // } else if (current_elem == TrackElement::OBSTACLE_SINGLE_L) {
        //     // 发现左侧路障，向右打方向绕行
        // }
    }
}// 传感器：IMU/TOF/TFT
void task_display_status(void* arg) {
    /*ADC phase*/
    float bat_v = ADC_sys.getBatteryVoltage();
    printf("%.2f\n",bat_v);
    // if (bat_v < 6.8f && bat_v > 0) {
    //     printf("[WARNING] 电池电压过低: %.2fV, 请及时充电！\n", bat_v);
    // }
}// ADC/TFT
void task_vofa_comm(void* arg) {
    // 如果电机已经启动 (is_running == 1) 且通讯尚未建立，则开始尝试连接
    // if (PID.is_running && !VOFA.is_initialized) {
    if (!VOFA.is_initialized) {
        
            // 你可以通过解开下面任意一行的注释，无缝切换三种通信方式：
        // VOFA.init_UDP("172.20.10.2", 8888);      // 方式1：连接 UDP 上位机，VOFA中设置连接板卡IP 172.20.10.9
        // VOFA.init_UDP("192.168.0.199", 8888);      // 方式1：连接 UDP 上位机
        // VOFA.init_UDP("192.168.0.200", 8888);      // 方式1：连接 UDP 上位机
        VOFA.init_UDP("192.168.1.11", 8888);      // 方式1：连接 UDP 上位机
        // VOFA.init_TCP("192.168.1.100", 9999);   // 方式2：连接 TCP 上位机
        // VOFA.init_UART(UART1, B115200);         // 方式3：通过串口接蓝牙透传/线缆通讯

        if (VOFA.is_initialized) {
            printf("Motor Started: Communication Successfully Initialized.\n");
        }
    }

    // 通讯建立后，处理收发逻辑
    if (VOFA.is_initialized) {
        VOFA.Process_Incoming(); // 随时接收调参数据
        
        if (PID.is_running) {
            // VOFA.Send_Data(PID.base_target_speed, vL, PID.pwm_out_L,
            //                vR, PID.pwm_out_R ,v_avg,
            //                PID.current_angle,  PID.current_yaw_speed);
            // VOFA.Send_Data(PID.target_speed_L, vL, PID.pwm_out_L,
            //                PID.target_speed_R, vR, PID.pwm_out_R,
            //                pure_angle,  PID.base_target_speed);
            VOFA.Send_Data(PID.target_speed_L, vL, PID.pwm_out_L,
                           PID.target_speed_R, vR, PID.pwm_out_R,
                           pure_angle,  PID.base_target_speed,
                           preview_curve_angle_deg, preview_img_y,big_langd_add);
            // Logger.Log_Data(PID.target_speed_L, vL, PID.pwm_out_L,
            //                PID.target_speed_R, vR, PID.pwm_out_R,
            //                PID.current_angle,  PID.base_target_speed);
        }
    }
}
void task_target_handler(void* arg){
    if (PID.is_running == 1 && handler_sys.is_initialized) {
        handler_sys.Update_Tick();
    }
}

static const char* board_recognition_gate_text(BoardRecognitionGate gate)
{
    switch (gate)
    {
    case BoardRecognitionGate::ALLOW: return "ALLOW";
    case BoardRecognitionGate::ALLOW_CIRCLE_RUNNING: return "ALLOW_CIRCLE";
    case BoardRecognitionGate::BLOCK: return "BLOCK";
    default: return "INVALID";
    }
}

void task_board_comm_rx(void* arg)
{
    static bool g_board_comm_link_logged = false;
    static BoardRecognitionGate last_gate_sent = BoardRecognitionGate::INVALID;
    static uint8_t gate_tx_seq = 0;
    static uint64_t last_gate_send_ms = 0;
    static uint64_t last_gate_attempt_ms = 0;
    static uint64_t last_gate_failure_log_ms = 0;

    const uint64_t now_ms = board_comm_now_ms();
    const BoardRecognitionGate current_gate =
        image_circle_recognition_gate_is_blocked()
            ? BoardRecognitionGate::BLOCK
            : (BW_CIRCLE_RECOGNITION_GATE_ENABLE != 0 &&
                       circle_state == CircleState::CIRCLE_RUNNING
                   ? BoardRecognitionGate::ALLOW_CIRCLE_RUNNING
                   : BoardRecognitionGate::ALLOW);
    const bool gate_changed = current_gate != last_gate_sent;
    const bool gate_heartbeat_due =
        last_gate_send_ms == 0 ||
        BW_CIRCLE_RECOGNITION_GATE_HEARTBEAT_MS <= 0 ||
        now_ms >= last_gate_send_ms +
                      static_cast<uint64_t>(BW_CIRCLE_RECOGNITION_GATE_HEARTBEAT_MS);
    const bool gate_retry_due =
        last_gate_attempt_ms == 0 ||
        BW_CIRCLE_RECOGNITION_GATE_HEARTBEAT_MS <= 0 ||
        now_ms >= last_gate_attempt_ms +
                      static_cast<uint64_t>(BW_CIRCLE_RECOGNITION_GATE_HEARTBEAT_MS);
    if ((gate_changed || gate_heartbeat_due) && gate_retry_due)
    {
        last_gate_attempt_ms = now_ms;
        const uint8_t send_seq =
            (gate_changed && last_gate_sent != BoardRecognitionGate::INVALID)
                ? static_cast<uint8_t>(gate_tx_seq + 1u)
                : gate_tx_seq;
        const bool send_ok = comm.send_recognition_gate(current_gate, send_seq);
        if (send_ok)
        {
            gate_tx_seq = send_seq;
            last_gate_sent = current_gate;
            last_gate_send_ms = now_ms;
        }
        if (gate_changed && send_ok)
        {
            printf("[BoardComm] recognition gate tx=%s, seq=%u, ok=%s\n",
                   board_recognition_gate_text(current_gate),
                   static_cast<unsigned int>(send_seq),
                   "yes");
        }
        else if (!send_ok &&
                 (last_gate_failure_log_ms == 0 ||
                  now_ms >= last_gate_failure_log_ms + 1000u))
        {
            last_gate_failure_log_ms = now_ms;
            printf("[BoardComm] recognition gate tx=%s failed\n",
                   board_recognition_gate_text(current_gate));
        }
    }

    BoardVisionCode code = BoardVisionCode::INVALID;
    uint8_t seq = 0;
    if (!comm.try_receive_state(&code, &seq))
    {
        return;
    }

    if (!g_board_comm_link_logged)
    {
        g_board_comm_link_logged = true;
        printf("\n====================================================\n");
        printf("[BoardComm] link online: first valid packet received\n");
        printf("[BoardComm] UART1 @ 115200, state=%s, seq=%u\n",
               board_comm_code_text(code),
               static_cast<unsigned int>(seq));
        printf("====================================================\n\n");
    }

    if (image_circle_recognition_gate_is_blocked())
    {
        return;
    }
    image_remote_recognition_apply_state(code, seq, pure_angle, now_ms);
}

// ==================== 主函数 ====================
int main(int argc, char** argv)
{   
    // 初始化紧急抱死
    // 确保按下 Ctrl+C 的瞬间，电机被强制断电停转
    lq_signal_set_exit_cb([]() {
        printf("\n[System] Received Ctrl+C,stop motor...\n");
        BayWatcher_Stop_Car(); 
    });

    //确认设备并挂载驱动
    std::vector<std::string> required_modules = {
        "lq_i2c_all_dev",         // I2C 设备总线
        "lq_i2c_mpu6050_drv",     // MPU6050 陀螺仪驱动
        // "lq_i2c_vl53l0x_drv",     // VL53L0X 测距驱动
        "TFT18_dev",              // TFT 屏幕设备
        "TFT18_dri"               // TFT 屏幕驱动
        // "lq_i2c_all_dev",         // I2C 设备总线
        // "lq_i2c_mpu6050_drv",     // MPU6050 陀螺仪驱动
    };

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

    //系统初始化
    system_init();

    //开启线程
    TimerThread pid_thread(BayWatcher_Control_Loop, NULL, 5);    
    // TimerThread pid_thread(BayWatcher_Inner_Loop, NULL, 5);     //Inner PID
    // TimerThread turn_thread(BayWatcher_Outter_Loop, NULL, 10);    //Outter PID
    // TimerThread turn_thread(BayWatcher_Cube_Loop, NULL, 10);    //Outter PID
    TimerThread turn_thread(BayWatcher_Cube_Loop, NULL, 10);    //Outter PID
    TimerThread sensor_thread(task_sensor_read, NULL, 20);
    TimerThread vofa_thread(task_vofa_comm, NULL, 20);            //VOFA+通信(UDP/TCP/UART),Log日志
    TimerThread board_comm_thread(task_board_comm_rx, NULL, 5);    // 双板串口收包
    TimerThread debug_thread(task_display_status, NULL, 2000);    //ADC
    // TimerThread handler_thread(task_target_handler, NULL, 20);    //targethandler
    // handler_thread.start();
    pid_thread.start();
    turn_thread.start();
    sensor_thread.start();
    vofa_thread.start();
    board_comm_thread.start();
    debug_thread.start();
    // if (pid_thread.start()) 
    // {
    //     printf("速度采样线程已启动，采样周期: 5ms\n");
    // } else {
    //     printf("线程启动失败！请检查是否链接了 pthread 库。\n");
    //     return -1;
    // }
    
    // 设置终端为非阻塞、非规范模式，用于即时读取 c / Ctrl+E。
    setup_nonblocking_keyboard_input();
    std::cout << "输入 c: 手动复位当前状态量；Ctrl+E: 关闭负压和电机" << std::endl;
    const bool stream_enabled =
        StreamChain::ParseSwitch(argc, argv, StreamChain::DefaultEnabled());
    std::cout << "[BOOT] stream switch=" << (stream_enabled ? "on" : "off")
              << " (args: --stream / --no-stream / --stream=on|off)" << std::endl;

    // 启动视觉系统 (阻塞运行)
    Vision_System_Run(stream_enabled);

    // 清理并退回终端

    // // 当程序运行到这里，说明 ls_system_running 已经变为 false
    // // 恢复终端状态：将 STDIN 改回正常的阻塞模式，防止退出后终端“假死”
    // int terminal_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    // if (terminal_flags != -1) {
    //     fcntl(STDIN_FILENO, F_SETFL, terminal_flags & ~O_NONBLOCK);
    // }

    // printf("\n[System] All tasks stopped. Preparing for exit...\n");
    // return 0;    // 注意：无需显式调用 exit(0)，执行 return 0 后，全局变量（包括 loader）会按正确顺序析构。

    restore_keyboard_input();

    system("stty sane");
    printf("\n\n[System] All tasks stopped. Preparing for exit...\n");
    printf("\n\n[System] Awakening terminal...\n");
    // return 0;
    _exit(0);//没招了
}
