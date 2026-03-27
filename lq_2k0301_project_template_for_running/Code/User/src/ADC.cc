#include "ADC.h"

BayWatcher_ADC::BayWatcher_ADC() {
    _scale = 0.0;
    _base_path = "/sys/bus/iio/devices/iio:device0/"; // Linux IIO 默认的 ADC 设备路径
}

BayWatcher_ADC::~BayWatcher_ADC() {}

bool BayWatcher_ADC::init() {
    // 初始化时读取一次 scale
    _scale = getScale();
    if (_scale <= 0.0) {
        printf("[ADC Error] Failed to initialize ADC. Scale is invalid.\n");
        return false;
    }
    printf("[ADC] Initialized successfully. Scale = %f\n", _scale);
    return true;
}

// 读取并返回 Scale
double BayWatcher_ADC::getScale() {
    std::string scale_path = _base_path + "in_voltage_scale";
    int fd_scale = open(scale_path.c_str(), O_RDONLY);
    
    if(fd_scale < 0) {
        perror("[ADC Error] open scale failed");
        return 0.0;
    }
    
    char buf[16] = {0};
    if (read(fd_scale, buf, sizeof(buf) - 1) <= 0) {
        perror("[ADC Error] read scale failed");
        close(fd_scale);
        return 0.0;
    }
    close(fd_scale);
    
    double scaleNum = strtod(buf, nullptr);//字符串转为浮点数
    _scale = scaleNum; // 更新内部缓存
    
    return scaleNum;
}

// 读取并返回原始数据
double BayWatcher_ADC::getRaw(int channel) {
    if (channel < 0 || channel > 7) {
        printf("[ADC Error] Invalid channel %d\n", channel);
        return 0.0;
    }
    
    // 动态拼接路径，eg: /sys/bus/iio/devices/iio:device0/in_voltage0_raw
    std::string raw_path = _base_path + "in_voltage" + std::to_string(channel) + "_raw";
    int fd_adc = open(raw_path.c_str(), O_RDONLY);
    
    if (fd_adc < 0) {
        perror("[ADC Error] open raw failed");
        return 0.0;
    }
    
    char buf[16] = {0};
    if (read(fd_adc, buf, sizeof(buf) - 1) <= 0) {
        close(fd_adc);
        return 0.0;
    }
    close(fd_adc);
    
    return strtod(buf, nullptr);
}

// 读取实际电压
double BayWatcher_ADC::getVoltage(int channel) {
    double raw = getRaw(channel);
    double voltage = (_scale * raw) / 1000.0;//mV转V
    // 终端打印显示
    printf("CH: %d | Raw = %4.0f | Voltage = %.3f V\n", channel, raw, voltage);
    return voltage;
}