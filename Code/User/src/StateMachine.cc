#include "StateMachine.h"

#define OBSTACLE_THRESHOLD_MM    350  // 识别路障的触发距离
#define RAMP_GRADIENT_THRESHOLD  -15  // 坡道突变导数阈值

TOF_StateMachine::TOF_StateMachine(BayWatcher_TOF& left, BayWatcher_TOF& right)
    : tof_L(left), tof_R(right), current_state(TrackElement::NORMAL_TRACK),
      count_double(0), count_single_L(0), count_single_R(0), count_ramp(0) 
{}

void TOF_StateMachine::Update_State() {
    uint16_t dist_L = tof_L.Get_Filtered_Distance();
    uint16_t dist_R = tof_R.Get_Filtered_Distance();
    int16_t deriv_L = tof_L.Get_Distance_Derivative();
    int16_t deriv_R = tof_R.Get_Distance_Derivative();

    // 1. 获取当前周期的瞬时判定
    bool see_obs_L = (dist_L < OBSTACLE_THRESHOLD_MM);
    bool see_obs_R = (dist_R < OBSTACLE_THRESHOLD_MM);

    // 2. 累加/清零确认计数器 (核心防抖逻辑)
    if (see_obs_L && see_obs_R) {
        count_double++;
        count_single_L = 0; count_single_R = 0;
    } else if (see_obs_L && !see_obs_R) {
        count_single_L++;
        count_double = 0; count_single_R = 0;
    } else if (!see_obs_L && see_obs_R) {
        count_single_R++;
        count_double = 0; count_single_L = 0;
    } else {
        // 前方空旷，路障计数器全部清零
        count_double = 0; count_single_L = 0; count_single_R = 0;
    }

    // 上坡检测：双侧必须同时检测到快速缩减的距离
    if (deriv_L < RAMP_GRADIENT_THRESHOLD && dist_L < 800 &&
        deriv_R < RAMP_GRADIENT_THRESHOLD && dist_R < 800) {
        count_ramp++;
    } else {
        count_ramp = 0;
    }

    // 3. 终局状态裁决 (谁先达到阈值，切换至谁)
    if (count_double >= CONFIRM_THRESHOLD) {
        current_state = TrackElement::OBSTACLE_DOUBLE;
    } else if (count_single_L >= CONFIRM_THRESHOLD) {
        current_state = TrackElement::OBSTACLE_SINGLE_L;
    } else if (count_single_R >= CONFIRM_THRESHOLD) {
        current_state = TrackElement::OBSTACLE_SINGLE_R;
    } else if (count_ramp >= CONFIRM_THRESHOLD) {
        current_state = TrackElement::RAMP_UP;
    } else {
        // 如果障碍物移开导致计数器清零，或者没达到阈值，保持正常赛道状态
        current_state = TrackElement::NORMAL_TRACK;
    }
}

TrackElement TOF_StateMachine::Get_Current_Element() {
    return current_state;
}