#include "Logger.h"

BayWatcher_Logger::BayWatcher_Logger() {
    memset(log_buffer, 0, sizeof(log_buffer));
}

void BayWatcher_Logger::Send_Via_UDP(const char* data, int length) {
    if (length > 0) {
        // 直接调用底层 UDP 发送，极速非阻塞
        VOFA.udp_cli.UDP_Send((char*)data, length); 
    }
}

// ==================== 记录高频数据 ====================
void BayWatcher_Logger::Log_Data(float target_L, float speed_L, int pwm_L, 
                                 float target_R, float speed_R, int pwm_R, 
                                 float yaw, float base_speed) {
    // 格式化为：[DATA],左目标,左实际,左PWM,右目标,右实际,右PWM,偏航角,基础速度
    int len = snprintf(log_buffer, sizeof(log_buffer), 
                       "[DATA],%.2f,%.2f,%d,%.2f,%.2f,%d,%.2f,%.2f\n", 
                       target_L, speed_L, pwm_L, 
                       target_R, speed_R, pwm_R, 
                       yaw, base_speed);
    
    Send_Via_UDP(log_buffer, len);
}

// ==================== 记录低频事件 ====================
void BayWatcher_Logger::Log_Event(const char* event_message) {
    int len = snprintf(log_buffer, sizeof(log_buffer), 
                       "[EVENT],%s\n", 
                       event_message);
    
    Send_Via_UDP(log_buffer, len);
}