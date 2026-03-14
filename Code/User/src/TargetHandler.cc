#include "TargetHandler.h"

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
    diff_override_value = 0.0f;
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
    diff_override_value = 0.0f;
    stage_timer = 0;
}

bool BayWatcher_TargetHandler::Is_Executing() {
    return current_stage != ActionStage::IDLE;
}

float BayWatcher_TargetHandler::Get_Differential_Override() {
    return diff_override_value;
}

void BayWatcher_TargetHandler::Start_Action(TargetBoardType target) {
    if (Is_Executing() || target == TargetBoardType::NONE) return;

    current_target = target;
    
    if (target == TargetBoardType::VEHICLE) {
        current_stage = ActionStage::DRIVE_PAST;
        stage_timer = 100; 
    } else {
        current_stage = ActionStage::TURN_OUT;
        // 第一阶段打方向的时间 (根据你的车速和底盘宽度微调)
        stage_timer = 30;  
    }
}

void BayWatcher_TargetHandler::Update_Tick() {
    if (_method == JudgeMethod::Fixed_Method) {
        if (!Is_Executing()) return;

        if (stage_timer > 0) stage_timer--;

        switch (current_target) {
            
            // ================= 武器：向左侧绕行 =================
            case TargetBoardType::WEAPON:
                if (current_stage == ActionStage::TURN_OUT) {
                    diff_override_value = -2.0f; // 1. 向左打方向出赛道
                    if (stage_timer == 0) {
                        current_stage = ActionStage::STRAIGHTEN_OUT;
                        stage_timer = 15; // 回正所需时间通常比打出赛道短一点
                    }
                } 
                else if (current_stage == ActionStage::STRAIGHTEN_OUT) {
                    diff_override_value = 2.0f;  // 2. 向右反打，回正车身平行赛道
                    if (stage_timer == 0) {
                        current_stage = ActionStage::DRIVE_PAST;
                        stage_timer = 20; // 直行通过目标板
                    }
                }
                else if (current_stage == ActionStage::DRIVE_PAST) {
                    diff_override_value = 0.0f;  // 3. 差速归零，笔直往前开
                    if (stage_timer == 0) {
                        current_stage = ActionStage::TURN_IN;
                        stage_timer = 30; 
                    }
                }
                else if (current_stage == ActionStage::TURN_IN) {
                    diff_override_value = 2.0f;  // 4. 向右打方向，切回赛道
                    if (stage_timer == 0) {
                        current_stage = ActionStage::STRAIGHTEN_IN; 
                        stage_timer = 15; 
                    }
                }
                else if (current_stage == ActionStage::STRAIGHTEN_IN) {
                    diff_override_value = -2.0f; // 5. 向左反打，车身在赛道上彻底摆正
                    if (stage_timer == 0) {
                        current_stage = ActionStage::DONE; // 绕行彻底结束
                    }
                }
                break;

            // ================= 物资：向右侧绕行 =================
            case TargetBoardType::SUPPLIES:
                if (current_stage == ActionStage::TURN_OUT) {
                    diff_override_value = 2.0f;  // 1. 向右打方向出赛道
                    if (stage_timer == 0) {
                        current_stage = ActionStage::STRAIGHTEN_OUT;
                        stage_timer = 20; 
                    }
                } 
                else if (current_stage == ActionStage::STRAIGHTEN_OUT) {
                    diff_override_value = -2.0f; // 2. 向左反打，回正车身
                    if (stage_timer == 0) {
                        current_stage = ActionStage::DRIVE_PAST;
                        stage_timer = 100; 
                    }
                }
                else if (current_stage == ActionStage::DRIVE_PAST) {
                    diff_override_value = 0.0f;  // 3. 差速归零，笔直往前开
                    if (stage_timer == 0) {
                        current_stage = ActionStage::TURN_IN;
                        stage_timer = 40; 
                    }
                }
                else if (current_stage == ActionStage::TURN_IN) {
                    diff_override_value = -2.0f; // 4. 向左打方向，切回赛道
                    if (stage_timer == 0) {
                        current_stage = ActionStage::STRAIGHTEN_IN;
                        stage_timer = 20;
                    }
                }
                else if (current_stage == ActionStage::STRAIGHTEN_IN) {
                    diff_override_value = 2.0f;  // 5. 向右反打，车身在赛道上彻底摆正
                    if (stage_timer == 0) {
                        current_stage = ActionStage::DONE; // 绕行彻底结束
                    }
                }
                break;

            // ================= 交通工具：直行压过 =================
            case TargetBoardType::VEHICLE:
                if (current_stage == ActionStage::DRIVE_PAST) {
                    diff_override_value = 0.0f; // 强行设为0，强行笔直开过去
                    if (stage_timer == 0) {
                        current_stage = ActionStage::DONE;
                    }
                }
                break;
                
            default: break;
        }
    } else {
        // 如果以后有视觉引导绕行的逻辑，写在这里
    }

    // 只要上面任何一个状态走到了 DONE，这里就会触发停止动作
    // 状态归位为 IDLE，剥夺底盘强制控制权，主程序无缝恢复巡线
    if (current_stage == ActionStage::DONE) {
        Stop_Action();
    }
}