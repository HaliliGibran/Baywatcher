#ifndef _BAYWATCHER_ADC_H_
#define _BAYWATCHER_ADC_H_

#include "main.hpp"

// 硬件层
#define ADC_CH_BATTERY 0
#define ADC_CH_SENSOR1 1
#define ADC_CH_SENSOR2 2

class BayWatcher_ADC {
public:
    BayWatcher_ADC();
    ~BayWatcher_ADC();

    // 初始化ADC并获取Scale值
    bool init();

    // 读取ADC的scale值用于单位转换，并返回
    double getScale();

    // 取ADC原始数据，返回原始数据
    double getRaw(int channel);

    // 将原始数据转换为实际电压值，并终端显示，返回实际电压值
    double getVoltage(int channel);

private:
    double _scale;               // 内部缓存的Scale值，避免每次算电压都要重新读文件
    std::string _base_path;      // Linux IIO 设备的根目录
};

extern BayWatcher_ADC      ADC_sys;

#endif