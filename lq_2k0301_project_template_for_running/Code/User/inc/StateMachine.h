#ifndef _TOF_STATE_MACHINE_H_
#define _TOF_STATE_MACHINE_H_

#include "TOF.h"

// 赛道元素状态枚举
enum class TrackElement {
    NORMAL_TRACK,      // 正常赛道
    OBSTACLE_DOUBLE,   // 左右都识别到 -> 正前方的墙/横杆
    OBSTACLE_SINGLE_L, // 只有左边识别到 -> 左侧路障 (需向右打方向)
    OBSTACLE_SINGLE_R, // 只有右边识别到 -> 右侧路障 (需向左打方向)
    RAMP_UP            // 上坡
};

class TOF_StateMachine {
public:
    // 构造函数传入左右两颗雷达的引用
    TOF_StateMachine(BayWatcher_TOF& left, BayWatcher_TOF& right);
    
    void Update_State(); // 每次读取数据后调用
    TrackElement Get_Current_Element();

private:
    BayWatcher_TOF& tof_L;
    BayWatcher_TOF& tof_R;

    TrackElement current_state;

    // 状态确认计数器 (连续识别达到阈值才切换状态)
    int count_double;
    int count_single_L;
    int count_single_R;
    int count_ramp;

    static const int CONFIRM_THRESHOLD = 4; // 连续 4 次满足条件才判定为真 (根据帧率微调)
};

extern TOF_StateMachine   tof_manager;

#endif