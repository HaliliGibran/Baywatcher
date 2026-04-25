#ifndef _BAYWATCHER_ADC_H_
#define _BAYWATCHER_ADC_H_

#include "main.hpp"
#include "lq_reg_adc.hpp" // 直接引入新版 ADC 硬件库

// 硬件层通道映射 (根据原理图，电池电压连接在 AD6)
#define ADC_CH_BATTERY LS_ADC_CH6

class BayWatcher_ADC {
public:
    BayWatcher_ADC();
    ~BayWatcher_ADC();

    // 初始化ADC实例
    bool init();

    // 取ADC原始数据 (0-4095)
    int getRaw(ls_adc_channel_t channel);

    // 获取 ADC 引脚的测量电压 (V)
    float getPinVoltage(ls_adc_channel_t channel);

    // 结合原理图分压比，计算并返回真实的电池总电压 (V)
    float getBatteryVoltage();

private:
    // 声明底层的 ADC 对象指针
    ls_adc* bat_adc;
};

extern BayWatcher_ADC ADC_sys;

#endif