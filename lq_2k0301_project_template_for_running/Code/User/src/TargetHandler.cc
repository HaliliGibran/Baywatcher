#include "TargetHandler.h"

namespace {

// 功能: 查询旧绕行状态机各阶段持续帧数
// 类型: 局部功能函数
// 关键参数:
// - target: 当前目标板类型
// - stage: 当前阶段
// 说明：这里刻意保留旧阶段时序，不改变原有动作节奏，只把输出量改成 pure_angle 接管。
static uint32_t get_stage_duration_frames(TargetBoardType target, ActionStage stage)
{
    switch (target)
    {
    case TargetBoardType::WEAPON:
        switch (stage)
        {
        case ActionStage::TURN_OUT:       return 30;
        case ActionStage::STRAIGHTEN_OUT: return 15;
        case ActionStage::DRIVE_PAST:     return 20;
        case ActionStage::TURN_IN:        return 30;
        case ActionStage::STRAIGHTEN_IN:  return 15;
        default:                          return 0;
        }

    case TargetBoardType::SUPPLIES:
        switch (stage)
        {
        case ActionStage::TURN_OUT:       return 30;
        case ActionStage::STRAIGHTEN_OUT: return 20;
        case ActionStage::DRIVE_PAST:     return 100;
        case ActionStage::TURN_IN:        return 40;
        case ActionStage::STRAIGHTEN_IN:  return 20;
        default:                          return 0;
        }

    case TargetBoardType::VEHICLE:
        return (stage == ActionStage::DRIVE_PAST) ? 100 : 0;

    default:
        return 0;
    }
}

// 功能: 根据目标板和阶段返回当前 pure_angle 接管量
// 类型: 局部功能函数
// 关键参数:
// - target: 当前目标板类型
// - stage: 当前阶段
// 说明：左右方向只做镜像，具体角度统一收口在 common.h。
static float get_stage_pure_angle(TargetBoardType target, ActionStage stage)
{
    switch (target)
    {
    case TargetBoardType::WEAPON:
        switch (stage)
        {
        case ActionStage::TURN_OUT:       return BW_BYPASS_LEFT_TURN_OUT_PURE_ANGLE;
        case ActionStage::STRAIGHTEN_OUT: return BW_BYPASS_LEFT_STRAIGHTEN_OUT_PURE_ANGLE;
        case ActionStage::DRIVE_PAST:     return BW_BYPASS_LEFT_DRIVE_PAST_PURE_ANGLE;
        case ActionStage::TURN_IN:        return BW_BYPASS_LEFT_TURN_IN_PURE_ANGLE;
        case ActionStage::STRAIGHTEN_IN:  return BW_BYPASS_LEFT_STRAIGHTEN_IN_PURE_ANGLE;
        default:                          return 0.0f;
        }

    case TargetBoardType::SUPPLIES:
        switch (stage)
        {
        case ActionStage::TURN_OUT:       return BW_BYPASS_RIGHT_TURN_OUT_PURE_ANGLE;
        case ActionStage::STRAIGHTEN_OUT: return BW_BYPASS_RIGHT_STRAIGHTEN_OUT_PURE_ANGLE;
        case ActionStage::DRIVE_PAST:     return BW_BYPASS_RIGHT_DRIVE_PAST_PURE_ANGLE;
        case ActionStage::TURN_IN:        return BW_BYPASS_RIGHT_TURN_IN_PURE_ANGLE;
        case ActionStage::STRAIGHTEN_IN:  return BW_BYPASS_RIGHT_STRAIGHTEN_IN_PURE_ANGLE;
        default:                          return 0.0f;
        }

    case TargetBoardType::VEHICLE:
        return 0.0f;

    default:
        return 0.0f;
    }
}

} // namespace

BayWatcher_TargetHandler::BayWatcher_TargetHandler() {
}

BayWatcher_TargetHandler::~BayWatcher_TargetHandler()
{
}

void BayWatcher_TargetHandler::init()
{   
    current_target = TargetBoardType::NONE;
    current_stage = ActionStage::IDLE;
    stage_timer = 0;
    pure_angle_override_value = 0.0f;
    _method = JudgeMethod::Fixed_Method; 
    is_initialized = true;
}

void BayWatcher_TargetHandler::setMethod(JudgeMethod method) {
    _method = method;
}

void BayWatcher_TargetHandler::Stop_Action()
{
    // 这里非常关键：清空状态后，Is_Executing() 会变为 false
    // main.cpp 会因此自动切回正常巡线模式！
    current_target = TargetBoardType::NONE;
    current_stage = ActionStage::IDLE;
    pure_angle_override_value = 0.0f;
    stage_timer = 0;
}

bool BayWatcher_TargetHandler::Is_Executing() {
    return current_stage != ActionStage::IDLE;
}

bool BayWatcher_TargetHandler::TryGetPureAngleOverride(float* out_angle) {
    if (!Is_Executing() || out_angle == nullptr) {
        return false;
    }

    *out_angle = pure_angle_override_value;
    return true;
}

void BayWatcher_TargetHandler::Start_Action(TargetBoardType target) {
    if (Is_Executing() || target == TargetBoardType::NONE) return;

    current_target = target;
    
    if (target == TargetBoardType::VEHICLE) {
        current_stage = ActionStage::DRIVE_PAST;
        stage_timer = get_stage_duration_frames(target, current_stage);
    } else {
        current_stage = ActionStage::TURN_OUT;
        stage_timer = get_stage_duration_frames(target, current_stage);
    }

    pure_angle_override_value = get_stage_pure_angle(current_target, current_stage);
}

void BayWatcher_TargetHandler::Update_Tick() {
    if (_method == JudgeMethod::Fixed_Method) {
        if (!Is_Executing()) return;

        // 当前帧先输出“当前阶段”的 pure_angle，
        // 再在计时耗尽时把状态推进到下一阶段，保持旧状态机的拍点语义。
        pure_angle_override_value = get_stage_pure_angle(current_target, current_stage);

        if (stage_timer > 0) {
            stage_timer--;
        }

        if (stage_timer == 0) {
            switch (current_stage) {
            case ActionStage::TURN_OUT:
                current_stage = ActionStage::STRAIGHTEN_OUT;
                break;
            case ActionStage::STRAIGHTEN_OUT:
                current_stage = ActionStage::DRIVE_PAST;
                break;
            case ActionStage::DRIVE_PAST:
                current_stage = (current_target == TargetBoardType::VEHICLE)
                                    ? ActionStage::DONE
                                    : ActionStage::TURN_IN;
                break;
            case ActionStage::TURN_IN:
                current_stage = ActionStage::STRAIGHTEN_IN;
                break;
            case ActionStage::STRAIGHTEN_IN:
                current_stage = ActionStage::DONE;
                break;
            default:
                break;
            }

            if (current_stage == ActionStage::DONE) {
                Stop_Action();
                return;
            }

            stage_timer = get_stage_duration_frames(current_target, current_stage);
        }
    } else {
        // 如果以后有视觉引导绕行的逻辑，写在这里
    }
}
