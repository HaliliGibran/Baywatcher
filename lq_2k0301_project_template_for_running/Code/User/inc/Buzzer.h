#ifndef _BAYWATCHER_BUZZER_H_
#define _BAYWATCHER_BUZZER_H_

#include "main.hpp"
#include "LQ_HW_GPIO.hpp" 
#include <stdint.h>

#define BUZZER_PIN 12 

class BayWatcher_Buzzer {
public:
    BayWatcher_Buzzer();
    ~BayWatcher_Buzzer();

    // 初始化蜂鸣器 GPIO 引脚
    void init();

    // 手动常开/常关
    void on();
    void off();

    // 异步非阻塞鸣叫 (单位：毫秒)
    void beep(uint32_t duration_ms);

    // 状态机 Tick (需放在定时线程或主循环中轮询)
    void Tick();

private:
    bool is_beeping;
    uint64_t beep_end_time_ms;
    HWGpio* gpio_dev; // 使用你底层库的硬件对象指针

    uint64_t get_current_time_ms();
};

extern BayWatcher_Buzzer buzzer_sys;

#endif