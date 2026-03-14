#ifndef _BAYWATCHER_TOF_H_
#define _BAYWATCHER_TOF_H_

// #include "main.hpp"
#include "lq_i2c_vl53l0x.hpp" 

class BayWatcher_TOF {
public:
    BayWatcher_TOF(); 
    ~BayWatcher_TOF(); 

    // 初始化，允许传入不同的 I2C 节点 (如 /dev/i2c-0 和 /dev/i2c-1)
    bool init(const std::string& i2c_path); 

    void Update(); 

    // 纯粹的数据输出接口
    uint16_t Get_Filtered_Distance();
    int16_t  Get_Distance_Derivative();

    bool is_initialized;

private:
    lq_i2c_vl53l0x* vl53_dev; 
    
    uint16_t raw_distance;
    uint16_t filtered_distance;
    uint16_t last_filtered_distance;
    int16_t  distance_derivative; 

    static const int FILTER_WINDOW_SIZE = 5;
    uint16_t filter_buffer[FILTER_WINDOW_SIZE];
    int filter_index;

    uint16_t Read_Raw_Distance(); 
    int16_t Calculate_Derivative(); 
    uint16_t Moving_Average_Filter(uint16_t new_val);
};

extern BayWatcher_TOF tof_left;
extern BayWatcher_TOF tof_right;

#endif