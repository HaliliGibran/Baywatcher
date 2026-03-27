#ifndef _BAYWATCHER_LOGGER_H_
#define _BAYWATCHER_LOGGER_H_

#include "main.hpp"

class BayWatcher_Logger {
public:
    BayWatcher_Logger();

    // 记录连续的高频物理数据 (格式化为 CSV 行)
    // 传入参数: 左轮目标速度，左轮速度，左轮占空比，右轮目标速度，右轮速度，右轮占空比，偏航角，基本速度
    void Log_Data(float target_L, float speed_L, int pwm_L, 
                   float target_R, float speed_R, int pwm_R, 
                   float yaw, float base_speed);

    // 记录突发的低频重要事件 (格式化为文本语句)
    void Log_Event(const char* event_message);

private:
    char log_buffer[256]; // 用于安全拼接字符串的内存池
    // 内部实际发送函数
    void Send_Via_UDP(const char* data, int length);
};

// 声明全局实例
extern BayWatcher_Logger Logger;

#endif