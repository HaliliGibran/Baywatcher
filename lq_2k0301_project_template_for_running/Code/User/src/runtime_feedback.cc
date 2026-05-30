#include "runtime_feedback.h"

#include "main.hpp"

float BayWatcher_GetBaseTargetSpeed(void)
{
    return PID.base_target_speed;
}

float BayWatcher_GetLeftWheelSpeed(void)
{
    return vL;
}

float BayWatcher_GetRightWheelSpeed(void)
{
    return vR;
}
