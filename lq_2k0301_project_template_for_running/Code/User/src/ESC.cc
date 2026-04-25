#include "ESC.h"

// GtimPwm TIM2CH1(87, 1, LS_GTIM_INVERSED, 50, 1000);
// GtimPwm TIM2CH2(88, 2, LS_GTIM_INVERSED, 50, 1000);
GtimPwm TIM2CH3(89, 3, LS_GTIM_INVERSED, 50, 500);         //右牙反转0%
GtimPwm TIM2CH4(77, 4, LS_GTIM_INVERSED, 50, 500, 0b01);   //左牙反转0%
// GtimPwm TIM2CH4(77, 4, LS_GTIM_INVERSED, 50, 500, 0b01);   //左牙反转0%

BayWatcher_ESC::BayWatcher_ESC(){
    is_running = false;
    is_soft_starting = false;
    base_percentage = 0.0f;
    current_percentage = 0.0f;
    ramp_step = 0.0f;
    
    // 初始化对外开放配置
    enable_esc_diff = false;
    esc_diff_ratio = 1.0f;
    esc_diff_limit = 15.0f;
}
BayWatcher_ESC::~BayWatcher_ESC(){}

void BayWatcher_ESC::init()
{   
    TIM2CH3.Enable();
    TIM2CH4.Enable();
    is_initialized = true ;
    is_running = false;
    is_soft_starting = false;
    printf("--- Brushless ESC Initialized ---\n");
}

void BayWatcher_ESC::start()
{   
    is_running = true;
    is_soft_starting = false; // 强行拉启动时，禁用平滑软启动，直接给油
    if(is_running && is_initialized){
        hw_duty_1 = 625;
        hw_duty_2 = 625;
        TIM2CH4.SetDutyCycle(hw_duty_1);//25%
        TIM2CH3.SetDutyCycle(hw_duty_2);
    }
}

void BayWatcher_ESC::Esc1_Set(int16_t percentage) {
    duty = min_pwm*(1+percentage*0.01f);
    if (duty > max_pwm) duty = max_pwm;
    if (duty < min_pwm) duty = min_pwm;
    hw_duty_1 = duty;
    TIM2CH4.SetDutyCycle(hw_duty_1);//左负压 IO77
}

void BayWatcher_ESC::Esc2_Set(int16_t percentage) {
    duty = min_pwm*(1+percentage*0.01f);
    if (duty > max_pwm) duty = max_pwm;
    if (duty < min_pwm) duty = min_pwm;
    hw_duty_2 = duty;
    TIM2CH3.SetDutyCycle(hw_duty_2);//右负压 IO89
}

void BayWatcher_ESC::Start_Soft(float target_pct, float step) {
    base_percentage = target_pct;
    ramp_step = step;
    current_percentage = 0.0f; // 软启动永远从 0 的纯怠速开始往上爬
    is_soft_starting = true;   // 设置标志位，让后续能在 Update_Tick 持续接力
    is_running = true;         // 先置高使能态
}

void BayWatcher_ESC::stop()
{
    is_running = false;
    is_soft_starting = false;
    base_percentage = 0.0f;
    current_percentage = 0.0f;
    hw_duty_1 = min_pwm;
    hw_duty_2 = min_pwm;
    TIM2CH4.SetDutyCycle(hw_duty_1);//左负压 IO77
    TIM2CH3.SetDutyCycle(hw_duty_2);//右负压 IO89
}

void BayWatcher_ESC::Update_Tick() {
    // 当系统未准备好或被停止时，直接退出，不干扰电机
    if (!is_running || !is_initialized) return;

    static uint32_t esc_log_cnt = 0;
    esc_log_cnt++;

    if (is_soft_starting) {
        // --- 正在软起动，百分比持续爬坡阶段 ---
        current_percentage += ramp_step;
        
        // 软起动到达或超过目标后，状态锁定
        if (current_percentage >= base_percentage) {
            current_percentage = base_percentage;
            is_soft_starting = false; 
            printf("[ESC] 软起动完成, 目标负压: %.1f%%\n", base_percentage);
        } else {
            if (esc_log_cnt % 20 == 0) { // 每 100ms 打印一次 (假设 Tick 是 5ms)
                printf("[ESC] 软起动爬升中: %.1f%% / %.1f%%\n", current_percentage, base_percentage);
            }
        }
        
        Esc1_Set((int16_t)current_percentage);
        Esc2_Set((int16_t)current_percentage);
    } else {
        // --- 软起动已结束（或手工直接直开）：系统正常全功率工作 ---
        if (enable_esc_diff) {
            // 将底盘舵机差速的修正偏差（通常与转弯半径正相关），叠加给两侧负压电调
            // 使得内圈车轮能获得比外圈更大的压力，进一步增强向心力并减弱外侧滚动阻力
            float diff = PID.speed_adjust * esc_diff_ratio;
            
            #define ESC_LIMIT(x, min, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))
            // 差速叠加限幅
            diff = ESC_LIMIT(diff, -esc_diff_limit, esc_diff_limit);

            float esc1_target = base_percentage - diff; // 左轮目标负压
            float esc2_target = base_percentage + diff; // 右轮目标负压
            
            // 严苛的差速安全限幅 (0% ~ 100%)
            esc1_target = ESC_LIMIT(esc1_target, 0.0f, 100.0f);
            esc2_target = ESC_LIMIT(esc2_target, 0.0f, 100.0f);
            #undef ESC_LIMIT

            if (esc_log_cnt % 50 == 0) { // 每 250ms 打印一次
                printf("[ESC] 负压差速 | Base:%.1f Diff:%.1f | L:%.1f%% R:%.1f%%\n", base_percentage, diff, esc1_target, esc2_target);
            }

            Esc1_Set((int16_t)esc1_target);  // 应用左外侧/内侧限幅后负压
            Esc2_Set((int16_t)esc2_target);  // 应用右外侧/内侧限幅后负压
        } else {
            // 无发车差速状态下双发同步固定推力
            Esc1_Set((int16_t)base_percentage);
            Esc2_Set((int16_t)base_percentage);
        }
    }
}