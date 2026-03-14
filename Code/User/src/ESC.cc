#include "ESC.h"

// GtimPwm TIM2CH1(87, 1, LS_GTIM_INVERSED, 50, 1000);
// GtimPwm TIM2CH2(88, 2, LS_GTIM_INVERSED, 50, 1000);
GtimPwm TIM2CH3(89, 3, LS_GTIM_INVERSED, 50, 0);
GtimPwm TIM2CH4(77, 4, LS_GTIM_INVERSED, 50, 0, 0b01);

BayWatcher_ESC::BayWatcher_ESC(){}
BayWatcher_ESC::~BayWatcher_ESC(){}

void BayWatcher_ESC::init()
{   
    TIM2CH3.Enable();
    TIM2CH4.Enable();
    is_initialized = true ;
}

void BayWatcher_ESC::start()
{   
}

void BayWatcher_ESC::stop()
{
}
