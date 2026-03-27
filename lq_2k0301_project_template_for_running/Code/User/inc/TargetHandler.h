#ifndef _BAYWATCHER_TARGETHANDLER_H_
#define _BAYWATCHER_TARGETHANDLER_H_

#include "main.hpp"

enum class TargetBoardType {
    NONE,           
    WEAPON,         // 武器 -> 左侧绕行
    SUPPLIES,       // 物资 -> 右侧绕行
    VEHICLE         // 交通工具 -> 直行压过
};

enum class ActionStage {
    IDLE,           
    TURN_OUT,       // 阶段1：打方向脱离赛道
    STRAIGHTEN_OUT, // 阶段2：反打方向，让车身回正平行于赛道
    DRIVE_PAST,     // 阶段3：取消差速，保持侧向直行通过
    TURN_IN,        // 阶段4：打方向切回赛道中心
    STRAIGHTEN_IN,  // 阶段5：反打方向，让车身在赛道中心回正
    DONE            
};

enum class JudgeMethod {
    Fixed_Method,   // 下位机写死定时器
    Vision_Method   // 上位机切线
};

class BayWatcher_TargetHandler {
public:
    BayWatcher_TargetHandler();
    ~BayWatcher_TargetHandler();
    void init();
    void setMethod(JudgeMethod method);             
    void Start_Action(TargetBoardType target);
    void Update_Tick();

    bool TryGetPureAngleOverride(float* out_angle);

    bool Is_Executing();
    void Stop_Action();
    bool is_initialized = false; 
private:
    JudgeMethod _method;
    TargetBoardType current_target;
    ActionStage     current_stage;
    uint32_t stage_timer;             
    float    pure_angle_override_value;
};

extern BayWatcher_TargetHandler   handler_sys;

#endif
