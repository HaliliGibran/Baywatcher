#include "ADC.h"

BayWatcher_ADC::BayWatcher_ADC() {
    bat_adc = nullptr;
}

BayWatcher_ADC::~BayWatcher_ADC() {
    if (bat_adc != nullptr) {
        delete bat_adc;
    }
}

bool BayWatcher_ADC::init() {
    // 实例化 AD6 通道，内部会自动触发单例硬件的 mmap 映射和寄存器初始化
    bat_adc = new ls_adc(ADC_CH_BATTERY);
    
    printf("[ADC] Initialized successfully using hardware registers.\n");
    return true;
}

// 读取并返回原始数据 (0-4095)
int BayWatcher_ADC::getRaw(ls_adc_channel_t channel) {
    if (channel == ADC_CH_BATTERY && bat_adc != nullptr) {
        return bat_adc->read_raw();
    }
    // 若后续有其他通道，在此扩展
    return -1;
}

// 读取引脚真实测量电压
float BayWatcher_ADC::getPinVoltage(ls_adc_channel_t channel) {
    if (channel == ADC_CH_BATTERY && bat_adc != nullptr) {
        // 新库的 read_voltage() 直接返回 V
        return bat_adc->read_voltage(); 
    }
    return -1.0f;
}

// 获取最终的电池电压
float BayWatcher_ADC::getBatteryVoltage() {
    float pin_voltage = getPinVoltage(ADC_CH_BATTERY);
    
    if (pin_voltage < 0) {
        printf("[ADC Error] Failed to read pin voltage.\n");
        return -1.0f; 
    }

    // 根据硬件原理图，分压比为 1:11
    float battery_voltage = pin_voltage * 11.0f;
    
    // 终端打印显示 (如果需要高频调用，建议注释掉这句打印，防止终端刷屏卡顿)
    // printf("CH: 6 | Pin = %.3f V | Battery = %.3f V\n", pin_voltage, battery_voltage);
    
    return battery_voltage;
}