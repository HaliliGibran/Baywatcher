#include "main.hpp"

AtimPwm Motor1(81, 1, LS_ATIM_INVERSED, 17000, 0);//电机1：PWM引脚81，方向引脚21，反转
AtimPwm Motor2(82, 2, LS_ATIM_INVERSED, 17000, 0);//电机2：PWM引脚82，方向引脚22  反转
HWGpio Motor1_DIR(Motor1_N,GPIO_Mode_Out);
HWGpio Motor2_DIR(Motor2_N,GPIO_Mode_Out);

BayWatcher_Motor::BayWatcher_Motor() {}
BayWatcher_Motor::~BayWatcher_Motor() { Stop();}

void BayWatcher_Motor::init(void) {
    Motor1.Enable();
    Motor2.Enable();
    // 默认停止
    Stop();
}

// === 电机 1 单独控制 ===(右轮)
void BayWatcher_Motor::Motor1_Set(int16_t duty) {
    //  限幅 
    if (duty > MAX_LOGIC_DUTY) duty = MAX_LOGIC_DUTY;
    if (duty < -MAX_LOGIC_DUTY) duty = -MAX_LOGIC_DUTY;

    // 方向控制
    if (duty >= 0) {
        Motor1_DIR.SetGpioValue(0); // 假设1为正转
    } else {
        Motor1_DIR.SetGpioValue(1); // 0为反转
    }

    // 占空比映射 
    hw_duty_1 = (uint16_t)(std::abs(duty));
    Motor1.SetDutyCycle(hw_duty_1);
}

// === 电机 2 单独控制 ===(左轮)
void BayWatcher_Motor::Motor2_Set(int16_t duty) {
    //  限幅 
    if (duty > MAX_LOGIC_DUTY) duty = MAX_LOGIC_DUTY;
    if (duty < -MAX_LOGIC_DUTY) duty = -MAX_LOGIC_DUTY;

    // 方向控制
    if (duty >= 0) {
        Motor2_DIR.SetGpioValue(1); // 假设1为正转
    } else {
        Motor2_DIR.SetGpioValue(0); // 0为反转
    }

    // 占空比映射 
    hw_duty_2 = (uint16_t)(std::abs(duty));
    Motor2.SetDutyCycle(hw_duty_2);
}

// === 急停 ===
void BayWatcher_Motor::Stop(void) {
    Motor1.SetDutyCycle(0);
    Motor2.SetDutyCycle(0);
    PID.is_running = 0;
}