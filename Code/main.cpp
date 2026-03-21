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
#include "recognition_chain.h"
#include "stream_chain.h"
#include "vision_runtime.h"
#include <vector>
const float SAMPLE_PERIOD = 0.005f;

using namespace cv;

// ==================== 全局对象与外部声明 ====================
BayWatcher_Encoder        encoder_sys;
BayWatcher_Menu&          menu = BayWatcher_Menu::getInstance();
BayWatcher_ADC            ADC_sys;
BayWatcher_Motor          motor_sys;
BayWatcher_Key            key_sys;

Bay_PID_t                 PID;
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

Camera                    camera;          // 创建摄像头对象
TransmissionStreamServer  server;
Mat                       img;             // 创建OpenCV Mat对象(原始图像)
Mat                       binimg;          // 用于输出的图像
Mat                       view;            // 轨迹画板

BayWatcher_TargetHandler  handler_sys;

// LQ_NCNN                ncnn_sys;

BayWatcher_Logger         Logger;
// BoardComm                 comm;

// ==================== 系统初始化 ====================
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
    // comm.init(UART1, B115200); 
    printf("Initialization Success.\n");
}

// ====================== 线程任务  ===================
void task_sensor_read(void* arg) {
    key_sys.Tick();
    if (PID.is_running){
        // tof_left.Update();
        // tof_right.Update();
        // tof_manager.Update_State();
        // TrackElement current_elem = tof_manager.Get_Current_Element();
        // if (current_elem == TrackElement::OBSTACLE_DOUBLE) {
        //     // 应对正前方横杆/墙
        // } else if (current_elem == TrackElement::OBSTACLE_SINGLE_L) {
        //     // 发现左侧路障，向右打方向绕行
        // }
    }
}// 传感器：IMU/TOF/TFT
void task_display_status(void* arg) {/*ADC phase*/}// ADC/TFT
void task_vofa_comm(void* arg) {
    // 如果电机已经启动 (is_running == 1) 且通讯尚未建立，则开始尝试连接
    if (PID.is_running && !VOFA.is_initialized) {
        
            // 你可以通过解开下面任意一行的注释，无缝切换三种通信方式：
        VOFA.init_UDP("172.20.10.2", 8888);      // 方式1：连接 UDP 上位机，VOFA中设置连接板卡IP 172.20.10.9
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
            // VOFA.Send_Data(PID.target_speed_L, vL, PID.pwm_out_L,
            //                PID.target_speed_R, vR, PID.pwm_out_R,
            //                pure_angle,  PID.yaw_control_base_speed,
            //                average_curvature, preview_img_y);
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
// ==================== 主函数 ====================
int main(int argc, char** argv)
{   
    //确认设备并挂载驱动
    std::vector<std::string> required_modules = {
        "lq_i2c_all_dev",         // I2C 设备总线
        "lq_i2c_mpu6050_drv",     // MPU6050 陀螺仪驱动
        // "lq_i2c_vl53l0x_drv",     // VL53L0X 测距驱动 (预留)
        "TFT18_dev",              // TFT 屏幕设备
        "TFT18_dri"               // TFT 屏幕驱动
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

    //系统初始化
    system_init();

    //开启线程
    TimerThread pid_thread(BayWatcher_Control_Loop, NULL, 2);    
    // TimerThread pid_thread(BayWatcher_Inner_Loop, NULL, 5);     //Inner PID
    // TimerThread turn_thread(BayWatcher_Outter_Loop, NULL, 10);    //Outter PID
    // TimerThread turn_thread(BayWatcher_Cube_Loop, NULL, 10);    // 视觉外环已并入 BayWatcher_Control_Loop
    TimerThread sensor_thread(task_sensor_read, NULL, 20);
    TimerThread vofa_thread(task_vofa_comm, NULL, 20);            //VOFA+通信(UDP/TCP/UART),Log日志
    // TimerThread debug_thread(task_display_status, NULL, 5000);    //ADC
    // TimerThread handler_thread(task_target_handler, NULL, 20);    //targethandler
    // handler_thread.start();
    if (!pid_thread.start())
    {
        printf("线程启动失败！请检查是否链接了 pthread 库。\n");
        return -1;
    }
    sensor_thread.start();
    vofa_thread.start();
    // debug_thread.start();
    printf("速度采样线程已启动，采样周期: 2ms\n");

    // while(1)
    // {
    //     if(!camera.capture_frame(img)){
    //         break;
    //     }
    //     cvtColor(img,binimg,COLOR_BGR2GRAY);
    //     threshold(binimg,binimg,0,255,THRESH_OTSU);
    //     server.update_frame_mat(binimg);
    //     // server.update_frame_jpeg(camera.jpeg_nowdata);
    // }
    
    // 设置终端为非阻塞 (用于按键 'c' 快速复位赛道状态)
    {
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags != -1) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
    std::cout << "输入 c: 手动复位当前状态量" << std::endl;
    const bool stream_enabled =
        StreamChain::ParseSwitch(argc, argv, StreamChain::DefaultEnabled());
    const bool recognition_enabled =
        RecognitionChain::ParseSwitch(argc, argv, RecognitionChain::DefaultEnabled());
    std::cout << "[BOOT] stream switch=" << (stream_enabled ? "on" : "off")
              << " (args: --stream / --no-stream / --stream=on|off)" << std::endl;
    std::cout << "[BOOT] recognition switch=" << (recognition_enabled ? "on" : "off")
              << " (args: --recognition / --no-recognition / --recognition=on|off)" << std::endl;

    // 启动视觉系统 (阻塞运行)
    Vision_System_Run(stream_enabled, recognition_enabled);
    return 0;
}


// // ==================== 系统初始化 ====================
// void system_init(){
//     printf("Innitializing...\n");
//     encoder_sys.init();
//     encoder_sys.setMethod(SpeedMethod::M_Method);
//     // encoder_sys.setMethod(SpeedMethod::T_Method);
//     key_sys.init();
//     ADC_sys.init();
//     menu.init();
//     BayWatcher_Control_Init();
//     camera.init(160,120,60);
//     // camera.init(640,480,60);
//     handler_sys.init();
//     imu_sys.init(("/dev/lq_i2c_mpu6050"));
//     // tof_left.init(("/dev/lq_i2c_vl53l0x"));
//     // handler_sys.setMethod(JudgeMethod::Fixed_Method);
//     ncnn_sys.SetModelPath("/home/root/tst/tiny_classifier_fp32.ncnn.param", "/home/root/tst/tiny_classifier_fp32.ncnn.bin");
//     ncnn_sys.SetInputSize(96, 96); // 必须跟训练时一样
//     ncnn_sys.SetLabels({"Weapon", "Supplies", "Vehicle"}); 
//     if(ncnn_sys.Init()) {
//         printf("NCNN Initialized!\n");
//     } else {
//         printf("NCNN Init Failed!\n");
//     }

//     printf("Initialization Success.\n");
// }

// // ====================== 线程任务  ===================
// void task_sensor_read(void* arg) {
//     key_sys.Tick();
//     if (PID.is_running){
//         // tof_left.Update();
//         // tof_right.Update();
//         // tof_manager.Update_State();
//         // TrackElement current_elem = tof_manager.Get_Current_Element();
//         // if (current_elem == TrackElement::OBSTACLE_DOUBLE) {
//         //     // 应对正前方横杆/墙
//         // } else if (current_elem == TrackElement::OBSTACLE_SINGLE_L) {
//         //     // 发现左侧路障，向右打方向绕行
//         // }
//     }
// }// 传感器：IMU/TOF/TFT
// void task_display_status(void* arg) {/*ADC phase*/}// ADC/TFT
// void task_vofa_comm(void* arg) {
//     // 如果电机已经启动 (is_running == 1) 且通讯尚未建立，则开始尝试连接
//     if (PID.is_running && !VOFA.is_initialized) {
        
//         // 你可以通过解开下面任意一行的注释，无缝切换三种通信方式：
//         VOFA.init_UDP("172.20.10.2", 8888);      // 方式1：连接 UDP 上位机，VOFA中设置连接板卡IP 172.20.10.9
//         // VOFA.init_TCP("192.168.1.100", 9999);   // 方式2：连接 TCP 上位机
//         // VOFA.init_UART(UART1, B115200);         // 方式3：通过串口接蓝牙透传/线缆通讯

//         if (VOFA.is_initialized) {
//             printf("Motor Started: Communication Successfully Initialized.\n");
//         }
//     }

//     // 通讯建立后，处理收发逻辑
//     if (VOFA.is_initialized) {
//         VOFA.Process_Incoming(); // 随时接收调参数据
        
//         if (PID.is_running) {
//             // VOFA.Send_Data(PID.base_target_speed, vL, PID.pwm_out_L,
//             //                vR, PID.pwm_out_R ,v_avg,
//             //                PID.current_angle,  PID.current_yaw_speed);
//             VOFA.Send_Data(PID.target_speed_L, vL, PID.pwm_out_L,
//                            PID.target_speed_R, vR, PID.pwm_out_R,
//                            PID.current_angle,  PID.base_target_speed);
//             // Logger.Log_Data(PID.target_speed_L, vL, PID.pwm_out_L,
//             //                PID.target_speed_R, vR, PID.pwm_out_R,
//             //                PID.current_angle,  PID.base_target_speed);
//         }
//     }
// }
// void task_target_handler(void* arg){
//     if (PID.is_running == 1 && handler_sys.is_initialized) {
//         handler_sys.Update_Tick();
//     }
// }
// // ==================== 图像画线辅助函数 ====================
// static inline void DrawEdgePoints(cv::Mat& view_bgr, const int32_t (&pts)[PT_MAXLEN][2], int32_t count, const cv::Scalar& color)
// {
//     if (view_bgr.empty() || view_bgr.type() != CV_8UC3) return;
//     if (count < 0) count = 0;
//     if (count > PT_MAXLEN) count = PT_MAXLEN;

//     const cv::Vec3b c((uint8_t)color[0], (uint8_t)color[1], (uint8_t)color[2]);
//     const int radius = 1; // 画小方块半径

//     for (int i = 0; i < count; ++i)
//     {
//         const int y = pts[i][0];
//         const int x = pts[i][1];
//         if ((unsigned)x >= (unsigned)view_bgr.cols || (unsigned)y >= (unsigned)view_bgr.rows) continue;

//         const int y0 = (y - radius > 0) ? (y - radius) : 0;
//         const int y1 = (y + radius < view_bgr.rows - 1) ? (y + radius) : (view_bgr.rows - 1);
//         const int x0 = (x - radius > 0) ? (x - radius) : 0;
//         const int x1 = (x + radius < view_bgr.cols - 1) ? (x + radius) : (view_bgr.cols - 1);

//         for (int yy = y0; yy <= y1; ++yy)
//         {
//             cv::Vec3b* row = view_bgr.ptr<cv::Vec3b>(yy);
//             for (int xx = x0; xx <= x1; ++xx) row[xx] = c;
//         }
//     }
// }

// static inline void DrawMidPoints(cv::Mat& view_bgr, const float (&pts)[PT_MAXLEN][2], int32_t count, const cv::Scalar& color)
// {
//     if (view_bgr.empty() || view_bgr.type() != CV_8UC3) return;
//     if (count < 0) count = 0;
//     if (count > PT_MAXLEN) count = PT_MAXLEN;

//     const cv::Vec3b c((uint8_t)color[0], (uint8_t)color[1], (uint8_t)color[2]);
//     const int radius = 1;

//     for (int i = 0; i < count; ++i)
//     {
//         const int y = (int)std::lroundf(pts[i][0]);
//         const int x = (int)std::lroundf(pts[i][1]);
//         if ((unsigned)x >= (unsigned)view_bgr.cols || (unsigned)y >= (unsigned)view_bgr.rows) continue;

//         const int y0 = (y - radius > 0) ? (y - radius) : 0;
//         const int y1 = (y + radius < view_bgr.rows - 1) ? (y + radius) : (view_bgr.rows - 1);
//         const int x0 = (x - radius > 0) ? (x - radius) : 0;
//         const int x1 = (x + radius < view_bgr.cols - 1) ? (x + radius) : (view_bgr.cols - 1);

//         for (int yy = y0; yy <= y1; ++yy)
//         {
//             cv::Vec3b* row = view_bgr.ptr<cv::Vec3b>(yy);
//             for (int xx = x0; xx <= x1; ++xx) row[xx] = c;
//         }
//     }
// }

// // ==================== 红色底座检测DEMO ====================
// // 检查画面下半部分是否有红色底座
// bool check_red_bottom(const cv::Mat& src_bgr) {
//     if (src_bgr.empty()) return false;

//     // 截取画面最下方 1/2 的区域
//     int roi_height = src_bgr.rows / 2;
//     cv::Rect roi(0, src_bgr.rows - roi_height, src_bgr.cols, roi_height);
//     cv::Mat bottom_roi = src_bgr(roi);

//     // 转HSV找红色
//     cv::Mat hsv, mask1, mask2, red_mask;
//     cv::cvtColor(bottom_roi, hsv, cv::COLOR_BGR2HSV);

//     // 红色在HSV的两个区间
//     cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
//     cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), mask2);
//     red_mask = mask1 | mask2;

//     // 统计红色像素点
//     int red_pixels = cv::countNonZero(red_mask);
//     int total_pixels = bottom_roi.rows * bottom_roi.cols;

//     // 如果红色像素占比超过 5%，判定为看到了底座 (这个阈值下赛道测一下)
//     return red_pixels > (total_pixels * 0.05);
// }

// // ================== 视觉与图传系统 =================
// void Vision_System_Run()
// {
//     // 图传初始化逻辑
// #ifndef BW_ENABLE_STREAM
// #define BW_ENABLE_STREAM 1
// #endif

//     bool server_ok = false;
// #if BW_ENABLE_STREAM
//     std::cout << "[BOOT] Starting TransmissionStreamServer..." << std::endl;
//     server.start_server(); 
//     server_ok = true; // 假定开启成功
//     std::cout << "[BOOT] Stream Started." << std::endl;
// #else
//     std::cout << "[BOOT] Stream disabled by BW_ENABLE_STREAM=0" << std::endl;
// #endif

//     // OpenCV 形态学算子与帧率控制变量
//     const Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
//     uint32_t stream_frame_seq = 0;

//     // 视觉主循环
//     while(1)
//     {
//         // 轮询键盘输入 (复位图像状态)
//         char ch = 0;
//         if (read(STDIN_FILENO, &ch, 1) == 1 && (ch == 'c' || ch == 'C')) {
//             track_force_reset(); // 确保 image_headfile 中包含了此函数
//         }

//         // 抓取图像
//         if(!camera.capture_frame(img)) {
//             usleep(10 * 1000); // 没抓到图时短暂休眠，防止 CPU 空转
//             continue;
//         }

//         // --- 核心视觉流控 ---

//         // 情况1：当前正在执行绕行，直接关闭巡线
//         if (handler_sys.Is_Executing()) {
//             view = img.clone();
//             putText(view, "Bypass Running...", Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
//         } 
//         // 情况2：正常模式，寻找目标或巡线
//         else {
//             // // 用下方的红色作信号
//             // if (check_red_bottom(img)) {
//             //     // 红色出现了！截取图片上半部分（目标图案），送进 NCNN
//             //     cv::Rect top_roi(0, 0, img.cols, img.rows / 2); // 截取上半部分
//             //     // cv::Rect top_roi(0, 0, img.cols, img.rows);
//             //     cv::Mat target_img = img(top_roi);

//             //     float confidence = 0.0f;
//             //     std::string obj_class = ncnn_sys.Infer(target_img, confidence); // 获取推理结果和置信度

//             //     // 打印物品类别和置信度
//             //     printf("识别目标: %s, 置信度: %.2f\n", obj_class.c_str(), confidence);

//             //     // 置信度大于阈值（如0.6），触发绕行控制
//             //     if (confidence > 0.8f) {
//             //         if (obj_class == "Weapon") {
//             //             handler_sys.Start_Action(TargetBoardType::WEAPON);
//             //         } else if (obj_class == "Supplies") {
//             //             handler_sys.Start_Action(TargetBoardType::SUPPLIES);
//             //         } else if (obj_class == "Vehicle") {
//             //             handler_sys.Start_Action(TargetBoardType::VEHICLE);
//             //         }
//             //     }

//             //     // 给图传打框标一下，方便你看
//             //     view = img.clone();
//             //     rectangle(view, top_roi, Scalar(0, 255, 255), 2);
//             //     putText(view, obj_class, Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
//             // } 
//             // // 迷宫法巡线
//             // else {
//                 cvtColor(img, binimg, COLOR_BGR2GRAY);
//                 threshold(binimg, binimg, 0, 255, THRESH_BINARY | THRESH_OTSU);
                
//                 morphologyEx(binimg, binimg, MORPH_OPEN, kernel);   
//                 morphologyEx(binimg, binimg, MORPH_CLOSE, kernel);   
                
//                 binimg.row(0).setTo(0); binimg.row(119).setTo(0); 
//                 binimg.col(0).setTo(0); binimg.col(159).setTo(0); 

//                 // 正常巡线
//                 img_processing(binimg);

//                 // 图传画线
//                 cvtColor(binimg, view, COLOR_GRAY2BGR);
//                 if(if_find_far_line) {
//                     DrawEdgePoints(view, pts_far_left.pts, pts_far_left.pts_count, Scalar(0, 255, 0));
//                     DrawEdgePoints(view, pts_far_right.pts, pts_far_right.pts_count, Scalar(255, 0, 0));
//                     DrawMidPoints(view, midline.path, midline.path_count, Scalar(0, 0, 255));
//                 } else {
//                     DrawEdgePoints(view, pts_left.pts, pts_left.pts_count, Scalar(0, 255, 0));
//                     DrawEdgePoints(view, pts_right.pts, pts_right.pts_count, Scalar(255, 0, 0));
//                     DrawMidPoints(view, midline.path, midline.path_count, Scalar(0, 0, 255));
//                 }
//             // }
//         }

// #if BW_ENABLE_STREAM
//         if (server_ok) {
//             if (((stream_frame_seq++) & 1u) == 0u) {
//                 server.update_frame_mat(view); 
//             }
//         }
// #endif
//     }
// }

// // ==================== 主函数 ====================
// int main()
// {   
//     //确认设备并挂载驱动
//     // std::unordered_map<std::string, std::string> required_modules = {
//     //     {"lq_i2c_all_dev", "lq_i2c_all_dev.ko"},
//     //     {"lq_i2c_mpu6050_drv",  "lq_i2c_mpu6050_drv.ko"},
//     //     {"TFT18_dri",  "TFT18_dri.ko"},
//     //     {"TFT18_dev",  "TFT18_dev.ko"}
//     // };
//     // LQModuleLoader loader(required_modules);
//     // printf("--- Loading Kernel Modules ---\n");
//     // if (!loader.loadAllModules()) {
//     //     printf("[ERROR] Failed to load some hardware modules. Check if .ko files exist.\n");
//     //     return -1; 
//     // }
//     // printf("--- Modules Loaded Successfully ---\n");

//     std::vector<std::string> required_modules = {
//         "lq_i2c_all_dev",         // I2C 设备总线
//         "lq_i2c_mpu6050_drv",     // MPU6050 陀螺仪驱动
//         // "lq_i2c_vl53l0x_drv",     // VL53L0X 测距驱动 (预留)
//         "TFT18_dev",              // TFT 屏幕设备
//         "TFT18_dri"               // TFT 屏幕驱动
//     };

//     lq_module_load loader;
//     printf("--- Loading Kernel Modules ---\n");
//     bool all_loaded = true;
//     for (const auto& mod : required_modules) {
//         if (!loader.set_load_module(mod)) {
//             printf("[ERROR] Failed to load module: %s\n", mod.c_str());
//             all_loaded = false;
//         }
//     }

//     if (!all_loaded) {
//         printf("[ERROR] 部分硬件模块加载失败！请检查 .ko 文件是否在 /home/ 目录下。\n");
//         return -1; 
//     }
//     printf("--- Modules Loaded Successfully ---\n");

//     //系统初始化
//     system_init();

//     //开启线程
//     TimerThread pid_thread(BayWatcher_Control_Loop, NULL, 2);    
//     // TimerThread pid_thread(BayWatcher_Inner_Loop, NULL, 5);     //Inner PID
//     TimerThread turn_thread(BayWatcher_Outter_Loop, NULL, 10);    //Outter PID
//     TimerThread sensor_thread(task_sensor_read, NULL, 20);
//     TimerThread vofa_thread(task_vofa_comm, NULL, 20);            //VOFA+通信(UDP/TCP/UART),Log日志
//     // TimerThread debug_thread(task_display_status, NULL, 5000);    //ADC
//     // TimerThread handler_thread(task_target_handler, NULL, 20);    //targethandler
//     // handler_thread.start();
//     pid_thread.start();
//     turn_thread.start();
//     sensor_thread.start();
//     vofa_thread.start();
//     // debug_thread.start();
//     if (pid_thread.start()) 
//     {
//         printf("速度采样线程已启动，采样周期: 5ms\n");
//     } else {
//         printf("线程启动失败！请检查是否链接了 pthread 库。\n");
//         return -1;
//     }

//     server.start_server();//启动图传

//     // while(1)
//     // {
//     //     if(!camera.capture_frame(img)){
//     //         break;
//     //     }
//     //     cvtColor(img,binimg,COLOR_BGR2GRAY);
//     //     threshold(binimg,binimg,0,255,THRESH_OTSU);
//     //     server.update_frame_mat(binimg);
//     //     // server.update_frame_jpeg(camera.jpeg_nowdata);
//     // }
    
//     // 设置终端为非阻塞 (用于按键 'c' 快速复位赛道状态)
//     {
//         int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
//         if (flags != -1) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
//     }
//     std::cout << "输入 c: 手动复位当前状态量" << std::endl;

//     // 启动视觉系统 (阻塞运行)
//     Vision_System_Run();
//     return 0;
// }
