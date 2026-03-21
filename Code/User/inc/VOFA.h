#ifndef _BAYWATCHER_VOFA_HPP_
#define _BAYWATCHER_VOFA_HPP_

#include <string>
#include "LQ_UDP_Client.hpp"
#include "LQ_TCP_Client.hpp"
#include "LQ_Uart.hpp"

// 通信模式枚举
enum class Vofa_Mode {
    NONE,
    UDP,
    TCP,
    UART
};

class BayWatcher_VOFA
{
public:
    BayWatcher_VOFA();
    ~BayWatcher_VOFA();

    // 初始化接口 
    void init_UDP(const std::string& target_ip, uint16_t target_port, uint16_t local_port = 8888);
    void init_TCP(const std::string& target_ip, uint16_t target_port);
    void init_UART(const std::string& dev = UART1, speed_t baudrate = B115200);
    
    // 数据收发接口 
    void Send_Data(float target_L, float speed_L, int pwm_L, 
                   float target_R, float speed_R, int pwm_R, 
                   float yaw, float base_speed,
                   float average_curvature, float preview_img_y);

    // void Send_Data(float base_speed, float speed_L, int pwm_L, 
    //                             float speed_R, int pwm_R, float speed_average,
    //                             float yaw, float current_yawspeed);
                   
    void Process_Incoming(void); // 放在主循环或通讯线程中非阻塞轮询
    bool is_initialized = false; // 通信是否成功建立标志位
    UDP_Client udp_cli;

private:
    Vofa_Mode  current_mode;
    
    // 龙邱通信库对象
    // UDP_Client udp_cli;
    TCP_Client tcp_cli;
    LS_UART* uart_cli; // 因为 UART 库没有独立 Init 函数且析构会关 fd，必须用指针动态分配
};

#endif
