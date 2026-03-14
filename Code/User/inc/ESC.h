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
    void stop();
private:
    bool is_initialized = false;
    uint32_t max_pwm;     // 满油门 PWM 值
    uint32_t min_pwm;     // 最低油门 PWM 值
    uint16_t duty;
};

extern BayWatcher_ESC esc_sys;
#endif // !_BAYWATCHER_ESC_H_