#ifndef _BAYWATCHER_ESC_H_
#define _BAYWATCHER_ESC_H_

#include "main.hpp"

extern GtimPwm TIM2CH3;
extern GtimPwm TIM2CH4;

class BayWatcher_ESC{
public:
    BayWatcher_ESC();
    ~BayWatcher_ESC();
    void init();
    void start();
    void Esc1_Set(int16_t percentage);
    void Esc2_Set(int16_t percentage);
    void stop();
    bool is_running;
private:
    bool is_initialized = false;
    const uint32_t max_pwm = 1000;    // 电调满转PWM 值
    const uint32_t min_pwm = 500;     // 最低油门 PWM 值
    uint16_t duty;
    uint16_t hw_duty_1;
    uint16_t hw_duty_2;

};

extern BayWatcher_ESC esc_sys;
#endif // !_BAYWATCHER_ESC_H_