#include "ESC.h"

GtimPwm TIM2CH3(89, 3, LS_GTIM_INVERSED, 50, 500);       // 右负压 IO89
GtimPwm TIM2CH4(77, 4, LS_GTIM_INVERSED, 50, 500, 0b01); // 左负压 IO77

BayWatcher_ESC::BayWatcher_ESC()
{
    is_running = false;
    is_soft_starting = false;
    enable_esc_diff = false;
    esc_diff_ratio = 0.0f;
    esc_diff_limit = 0.0f;
    enable_dynamic_esc = false;
    dynamic_esc_base = 0.0f;
    dynamic_esc_max = 0.0f;
    dynamic_esc_threshold = 0.0f;
    dynamic_esc_factor_max = 0.0f;
    current_factor = 0.0f;
    lp_alpha = 0.0f;
    filtered_target_base = 0.0f;
    duty = 500;
}

BayWatcher_ESC::~BayWatcher_ESC()
{
    stop();
}

void BayWatcher_ESC::init()
{
    TIM2CH3.Enable();
    TIM2CH4.Enable();
    is_initialized = true;
    stop();
    printf("--- Brushless ESC Initialized ---\n");
}

void BayWatcher_ESC::start()
{
    is_running = true;
    Esc1_Set(5);
    Esc2_Set(5);
}

void BayWatcher_ESC::Esc1_Set(int16_t percentage)
{
    if (percentage < 0) percentage = 0;
    if (percentage > 80) percentage = 80; // blindbox 电调调参限幅
    duty = min_pwm * (1 + percentage * 0.01f);
    if (duty > max_pwm) duty = max_pwm;
    if (duty < min_pwm) duty = min_pwm;
    TIM2CH4.SetDutyCycle(duty);
}

void BayWatcher_ESC::Esc2_Set(int16_t percentage)
{
    if (percentage < 0) percentage = 0;
    if (percentage > 80) percentage = 80; // blindbox 电调调参限幅
    duty = min_pwm * (1 + percentage * 0.01f);
    if (duty > max_pwm) duty = max_pwm;
    if (duty < min_pwm) duty = min_pwm;
    TIM2CH3.SetDutyCycle(duty);
}

void BayWatcher_ESC::Start_Soft(float target_pct, float)
{
    is_running = true;
    Esc1_Set(static_cast<int16_t>(target_pct));
    Esc2_Set(static_cast<int16_t>(target_pct));
}

void BayWatcher_ESC::Update_Tick()
{
}

void BayWatcher_ESC::stop()
{
    is_running = false;
    is_soft_starting = false;
    TIM2CH4.SetDutyCycle(min_pwm);
    TIM2CH3.SetDutyCycle(min_pwm);
}
