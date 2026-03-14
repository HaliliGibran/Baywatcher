#ifndef _BAYWATCHER_CASCADE_PID_H_
#define _BAYWATCHER_CASCADE_PID_H_

#include "main.hpp"

// 增量式 PID (内环：速度)
typedef struct {
    float Kp, Ki, Kd;
    float error, prev_error, last_error;
    float error_i;      // 积分相差值
    float output;       // 当前累计输出 (PWM)
    float output_limit; // PWM限幅 (10000)
    float integral_limit; // 积分限幅
} Bay_IncPID_t;

// 位置式 PID (外环：角度 / 中环：角速度)
typedef struct {
    float Kp, Ki, Kd;
    float error, integral, last_error;
    float output;       
    float output_limit; // 差速限幅 
    float integral_limit;
} Bay_PosPID_t;

// typedef struct {
//     float Kp, Ki, Kd;
//     float error, integral, last_error;
//     float output;       
//     float output_limit; // 差速限幅 
//     float integral_limit;
// } Bay_FforwardPID_t;

// 运动状态
typedef struct {
    uint8_t  is_running;        // 运行标志        
    
    // --- 设定值 (Setpoints) ---
    float    base_target_speed; // 基础直行速度
    float    target_angle;      // 目标角度 (最外环输入, 通常为0)
    
    // --- 传感器反馈 (Feedback) ---
    float    vision_yaw;        // 视觉偏差 (摄像头)
    float    current_angle;     // 当前角度 (来自IMU/视觉)
    float    current_yaw_speed; // 当前角速度 (来自陀螺仪 Z轴)

    // --- 中间计算值 (Cascade Outputs) ---
    float    target_yaw_speed;  // 角度环输出 -> 角速度环输入
    float    speed_adjust;      // 角速度环输出 -> 速度环差速输入
    
    float    target_speed_L;    // 左轮最终目标速度
    float    target_speed_R;    // 右轮最终目标速度
    
    // --- 执行器输出 ---
    int32_t  pwm_out_L;         
    int32_t  pwm_out_R;
} Bay_PID_t;

void BayWatcher_Control_Init(void);  
void BayWatcher_Control_Loop(void* arg);  

void BayWatcher_Inner_Loop(void* arg);
void BayWatcher_Middle_Loop(void* arg);
void BayWatcher_Outter_Loop(void* arg);

void BayWatcher_Set_BaseSpeed(float speed);
void BayWatcher_Set_TargetAngle(float angle);
void BayWatcher_Start_Car(void);
void BayWatcher_Stop_Car(void);

#endif