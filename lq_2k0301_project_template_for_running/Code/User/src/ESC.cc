#include "ESC.h"

// GtimPwm TIM2CH1(87, 1, LS_GTIM_INVERSED, 50, 1000);
// GtimPwm TIM2CH2(88, 2, LS_GTIM_INVERSED, 50, 1000);
GtimPwm TIM2CH3(89, 3, LS_GTIM_INVERSED, 50, 500);         //右牙反转0%
GtimPwm TIM2CH4(77, 4, LS_GTIM_INVERSED, 50, 500, 0b01);   //左牙反转0%
// GtimPwm TIM2CH4(77, 4, LS_GTIM_INVERSED, 50, 500, 0b01);   //左牙反转0%

BayWatcher_ESC::BayWatcher_ESC(){}
BayWatcher_ESC::~BayWatcher_ESC(){}

void BayWatcher_ESC::init()
{   
    TIM2CH3.Enable();
    TIM2CH4.Enable();
    is_initialized = true ;
    esc_sys.is_running =0;
    printf("--- Brushless ESC Initialized ---\n");
}

void BayWatcher_ESC::start()
{   esc_sys.is_running =1;
    if(esc_sys.is_running ==1 && is_initialized ==true){
        // TIM2CH3.SetDutyCycle(500);//0%
        // TIM2CH4.SetDutyCycle(500);
        // TIM2CH3.SetDutyCycle(525);//5%
        // TIM2CH4.SetDutyCycle(525);
        // TIM2CH3.SetDutyCycle(550);//10%
        // TIM2CH4.SetDutyCycle(550);
        // TIM2CH3.SetDutyCycle(575);//15%
        // TIM2CH4.SetDutyCycle(575);
        TIM2CH3.SetDutyCycle(625);//25%
        TIM2CH4.SetDutyCycle(625);
        // TIM2CH3.SetDutyCycle(750);//50%
        // TIM2CH4.SetDutyCycle(750);
        // TIM2CH3.SetDutyCycle(1000);//100%
        // TIM2CH4.SetDutyCycle(1000);
    }
}

void BayWatcher_ESC::Esc1_Set(int16_t percentage) {
    duty = min_pwm*(1+percentage);
    if (duty > max_pwm) duty = max_pwm;
    if (duty < min_pwm) duty = min_pwm;
    TIM2CH4.SetDutyCycle(duty);//左负压 IO77
}

void BayWatcher_ESC::Esc2_Set(int16_t percentage) {
    duty = min_pwm*(1+percentage);
    if (duty > max_pwm) duty = max_pwm;
    if (duty < min_pwm) duty = min_pwm;
    TIM2CH3.SetDutyCycle(duty);//右负压 IO89
}

void BayWatcher_ESC::stop()
{
    esc_sys.is_running =0;
    TIM2CH3.SetDutyCycle(500);//0%
    TIM2CH4.SetDutyCycle(500);
}
