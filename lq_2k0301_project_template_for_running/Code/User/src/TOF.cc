#include "TOF.h"
#include <stdio.h>

BayWatcher_TOF::BayWatcher_TOF() 
    : vl53_dev(nullptr), is_initialized(false) 
{
    raw_distance = 0;
    filtered_distance = 0;
    last_filtered_distance = 0;
    distance_derivative = 0;
    
    filter_index = 0;
    for(int i = 0; i < FILTER_WINDOW_SIZE; i++) filter_buffer[i] = 0;
}

BayWatcher_TOF::~BayWatcher_TOF() {
    if (vl53_dev != nullptr) {
        delete vl53_dev;
        vl53_dev = nullptr;
    }
}

bool BayWatcher_TOF::init(const std::string& i2c_path) {
    if (is_initialized) return true;

    vl53_dev = new lq_i2c_vl53l0x(i2c_path);
    uint16_t test_dis = vl53_dev->get_vl53l0x_dis();
    printf("[TOF] 挂载成功！节点: %s | 测试测距: %d mm\n", i2c_path.c_str(), test_dis);
    is_initialized = true;
    return true;
}

uint16_t BayWatcher_TOF::Read_Raw_Distance() {
    if (!is_initialized || vl53_dev == nullptr) return 8190; 
    return vl53_dev->get_vl53l0x_dis(); 
}

int16_t BayWatcher_TOF::Calculate_Derivative() {
    return (int16_t)filtered_distance - (int16_t)last_filtered_distance;
}

uint16_t BayWatcher_TOF::Moving_Average_Filter(uint16_t new_val) {
    if (new_val > 2000) new_val = 2000; 
    filter_buffer[filter_index] = new_val;
    filter_index = (filter_index + 1) % FILTER_WINDOW_SIZE;
    
    uint32_t sum = 0;
    for(int i = 0; i < FILTER_WINDOW_SIZE; i++) sum += filter_buffer[i];
    return (uint16_t)(sum / FILTER_WINDOW_SIZE);
}

void BayWatcher_TOF::Update() {
    if (!is_initialized) return; 

    raw_distance = Read_Raw_Distance();
    filtered_distance = Moving_Average_Filter(raw_distance);
    distance_derivative = Calculate_Derivative();
    last_filtered_distance = filtered_distance;
    // printf("%.2f\n",last_filtered_distance);
}

uint16_t BayWatcher_TOF::Get_Filtered_Distance() { return filtered_distance; }
int16_t BayWatcher_TOF::Get_Distance_Derivative() { return distance_derivative; }