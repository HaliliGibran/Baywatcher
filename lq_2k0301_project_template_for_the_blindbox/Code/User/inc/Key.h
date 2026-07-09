#ifndef _BAYWATCHER_KEY_H_
#define _BAYWATCHER_KEY_H_

#include "main.hpp"
#include "LQ_HW_GPIO.hpp"
#include <vector>   
#include <chrono>   
#include <stdint.h>

class BayWatcher_Button 
{
public:
    BayWatcher_Button(uint8_t pin, KeyOp op);
    void Scan(); // 扫描逻辑

private:
    HWGpio gpio;       // 硬件 GPIO 对象
    KeyOp  operation;         // 对应的操作
    bool last_raw_level = true;
    bool stable_level = true;
    std::chrono::steady_clock::time_point last_debounce_time;
};

class BayWatcher_Key
{
public:
    BayWatcher_Key() = default;
    void init();
    void Tick(); // 放在主循环中持续调用
private:
    std::vector<BayWatcher_Button> btns;
};
#endif // _BAYWATCHER_KEY_H_