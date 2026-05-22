#include "main.hpp"

BayWatcher_Button::BayWatcher_Button(uint8_t pin, KeyOp op) 
    : gpio(pin, GPIO_Mode_In), operation(op) {
    last_debounce_time = std::chrono::steady_clock::now();
}

void BayWatcher_Button::Scan() {
    bool now_level = gpio.GetGpioValue(); // 获取当前引脚物理电平
    auto now = std::chrono::steady_clock::now();

    // 如果电平有跳变，重置消抖时间
    if (now_level != last_raw_level) {
        last_debounce_time = now;
    }

    // 维持 20ms 电平不变，才认为是稳定状态 (软件消抖)
    if ((now - last_debounce_time) > std::chrono::milliseconds(20)) {
        if (now_level != stable_level) {
            stable_level = now_level;
            
            // 边沿检测：从 1 (高电平) 变 0 (低电平) 表示按键按下瞬间
            if (stable_level == false) {
                // 直接通知全局 menu 对象执行对应的动作并刷新屏幕
                menu.refresh_menu(operation);
            }
        }
    }
    last_raw_level = now_level;
}

void BayWatcher_Key::init() {
    btns.emplace_back(44, KeyOp::Up);
    btns.emplace_back(45, KeyOp::Down);
    btns.emplace_back(80, KeyOp::OK);
    btns.emplace_back(20, KeyOp::Cancel);
    btns.emplace_back(17, KeyOp::Back);
}

void BayWatcher_Key::Tick() {
    for (auto& btn : btns) {
        btn.Scan();
    }
}