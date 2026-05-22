#include "Buzzer.h"
#include <chrono>
#include <stdio.h>

BayWatcher_Buzzer::BayWatcher_Buzzer() : is_beeping(false), beep_end_time_ms(0), gpio_dev(nullptr) {}

BayWatcher_Buzzer::~BayWatcher_Buzzer() {
    off();
    if (gpio_dev != nullptr) {
        delete gpio_dev;
    }
}

void BayWatcher_Buzzer::init() {
    if (gpio_dev != nullptr) {
        delete gpio_dev;
    }
    
    // 实例化底层 GPIO 对象，使用你库里的 GPIO_Mode_Out 宏配置为输出
    gpio_dev = new HWGpio(BUZZER_PIN, GPIO_Mode_Out);
    
    is_beeping = false;
    off(); // 默认保持静音
    printf("[Buzzer] Initialized on GPIO %d using LQ_HW_GPIO (mmap)\n", BUZZER_PIN);
}

void BayWatcher_Buzzer::on() {
    if (gpio_dev != nullptr) {
        // 调用你的底层库函数，写入 1 导通三极管，蜂鸣器响
        gpio_dev->SetGpioValue(1); 
    }
}

void BayWatcher_Buzzer::off() {
    if (gpio_dev != nullptr) {
        // 调用你的底层库函数，写入 0 截止三极管，蜂鸣器停
        gpio_dev->SetGpioValue(0); 
    }
}

// 获取系统的毫秒级时间戳
uint64_t BayWatcher_Buzzer::get_current_time_ms() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

// 触发鸣叫任务（非阻塞）
void BayWatcher_Buzzer::beep(uint32_t duration_ms) {
    on();
    is_beeping = true;
    beep_end_time_ms = get_current_time_ms() + duration_ms;
}

// 检查是否到了该闭嘴的时间
void BayWatcher_Buzzer::Tick() {
    if (is_beeping) {
        if (get_current_time_ms() >= beep_end_time_ms) {
            off();
            is_beeping = false;
        }
    }
}