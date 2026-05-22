/*
 * 用户自定义思路版：连贯顺序序列解码器 (一倍频)
 * 文件名: LQ_AB_ENCODER.cpp
 */
#include "LQ_AB_ENCODER.hpp"

#ifndef NUM_ENCODER_LINE
#define NUM_ENCODER_LINE    1024.0f 
#endif
#define WHEEL_PERIMETER_CM  20.0f   

LQ_AB_Encoder::LQ_AB_Encoder(int pin_a, int pin_b) : 
    PhaseA(pin_a, GPIO_Mode_In), 
    PhaseB(pin_b, GPIO_Mode_In) 
{
    pulse_count = 0;
    last_ab_state = 0;
    forward_step = 0;
    reverse_step = 0;
}

void LQ_AB_Encoder::Init(void) {
    uint8_t a = PhaseA.GetGpioValue() ? 1 : 0;
    uint8_t b = PhaseB.GetGpioValue() ? 1 : 0;
    last_ab_state = (a << 1) | b;
    pulse_count = 0;
    
    forward_step = 0;
    reverse_step = 0;
}

void LQ_AB_Encoder::Poll(void) {
    uint8_t a = PhaseA.GetGpioValue() ? 1 : 0;
    uint8_t b = PhaseB.GetGpioValue() ? 1 : 0;
    uint8_t current_ab_state = (a << 1) | b;
    
    // 如果状态没变，直接退出，不干扰进度条
    if (current_ab_state == last_ab_state) {
        return; 
    }

    // 拼接旧状态和新状态，用于精确判断边缘变化方向
    uint8_t trans = (last_ab_state << 2) | current_ab_state;

    // ================== 正转序列检测 (你的第一种状态) ==================
    // 1. A上升 B低 (旧00 -> 新10，拼接后为 0x02)
    if (trans == 0x02) {
        forward_step = 1; // 触发第一步 (随时可以作为序列起点)
    }
    // 2. A高 B上升 (旧10 -> 新11，拼接后为 0x0B)
    else if (trans == 0x0B && forward_step == 1) {
        forward_step = 2; // 完成第二步
    }
    // 3. A下降 B高 (旧11 -> 新01，拼接后为 0x0D)
    else if (trans == 0x0D && forward_step == 2) {
        forward_step = 3; // 完成第三步
    }
    // 4. A低 B下降 (旧01 -> 新00，拼接后为 0x04)
    else if (trans == 0x04 && forward_step == 3) {
        pulse_count++;    // 连贯顺序整体出现一次，记一次增加！
        forward_step = 0; // 进度条清零，准备迎接下一轮
    }
    else {
        forward_step = 0; // 只要中间出现了任何不按套路出牌的动作，进度清零
    }

    // ================== 反转序列检测 (你的第二种状态) ==================
    // 1. A下降 B低 (旧10 -> 新00，拼接后为 0x08)
    if (trans == 0x08) {
        reverse_step = 1; // 触发反转第一步
    }
    // 2. A低 B上升 (旧00 -> 新01，拼接后为 0x01)
    else if (trans == 0x01 && reverse_step == 1) {
        reverse_step = 2; // 完成第二步
    }
    // 3. A上升 B高 (旧01 -> 新11，拼接后为 0x07)
    else if (trans == 0x07 && reverse_step == 2) {
        reverse_step = 3; // 完成第三步
    }
    // 4. A高 B下降 (旧11 -> 新10，拼接后为 0x0E)
    else if (trans == 0x0E && reverse_step == 3) {
        pulse_count--;    // 连贯顺序整体出现一次，记一次减少！
        reverse_step = 0; // 进度条清零
    }
    else {
        reverse_step = 0; // 顺序被打断，清零
    }

    // 更新最后状态
    last_ab_state = current_ab_state;
}

float LQ_AB_Encoder::Get_Speed(float sample_time_s) {
    int32_t temp_pulses = pulse_count;
    pulse_count = 0; 
    
    // 【极其关键的修改】：
    // 因为你的逻辑是“走完四个微观步骤才算1个宏观脉冲”（一倍频解码）
    // 所以这里的总脉冲数不再乘以4，而是直接等于物理线数 NUM_ENCODER_LINE
    float speed_cm_s = ((float)temp_pulses / NUM_ENCODER_LINE) / sample_time_s * WHEEL_PERIMETER_CM;
    
    return speed_cm_s;
}

void LQ_AB_Encoder::ResetCounter(void) {
    pulse_count = 0;
}