/*
 * 用户自定义思路版：连贯顺序序列解码器 (一倍频)
 * 文件名: LQ_AB_ENCODER.hpp
 */
#pragma once

#include "LQ_HW_GPIO.hpp"
#include <stdint.h>

class LQ_AB_Encoder {
public:
    LQ_AB_Encoder(int pin_a, int pin_b);
    void Init(void);
    void Poll(void);
    float Get_Speed(float sample_time_s = 0.005f);
    void ResetCounter(void);

private:
    HWGpio PhaseA;
    HWGpio PhaseB;
    
    uint8_t last_ab_state;
    volatile int32_t pulse_count;

    // 【新增】：为你思路设计的“序列进度条”
    uint8_t forward_step; // 正转连贯动作进度 (0~3)
    uint8_t reverse_step; // 反转连贯动作进度 (0~3)
};