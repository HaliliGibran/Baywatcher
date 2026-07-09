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
    esc_diff_ratio = 0.0f;
    esc_diff_limit = 0.0f;

    // 动态负压初始化
    enable_dynamic_esc = false;
    // enable_dynamic_esc = true;
    dynamic_esc_base = 30.0f;
    dynamic_esc_max = 45.0f;
    dynamic_esc_threshold = 0.45f;
    dynamic_esc_factor_max = 1.00f;
    current_factor = 0.0f;
    lp_alpha = 0.4f;               // 低通滤波系数：越小越平滑，0.1能极大地抑制高频噪声
    filtered_target_base = 30.0f;
    esc_log_cnt = 0;
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
        // hw_duty_1 = 650;
        // hw_duty_2 = 650;
        // TIM2CH4.SetDutyCycle(hw_duty_1);//25%
        // TIM2CH3.SetDutyCycle(hw_duty_2);
        // Esc1_Set(30);
        // Esc2_Set(30);
        Esc1_Set(5);
        Esc2_Set(5);
    }
}

void BayWatcher_ESC::Esc1_Set(int16_t percentage) {
    duty = min_pwm*(1+percentage*0.01f);
    if (duty > max_pwm) duty = max_pwm;
    if (duty < min_pwm) duty = min_pwm;
    TIM2CH4.SetDutyCycle(duty);//左负压 IO77
}

void BayWatcher_ESC::Esc2_Set(int16_t percentage) {
    duty = min_pwm*(1+percentage*0.01f);
    if (duty > max_pwm) duty = max_pwm;
    if (duty < min_pwm) duty = min_pwm;
    TIM2CH3.SetDutyCycle(duty);//右负压 IO89
}

void BayWatcher_ESC::Start_Soft(float target_pct, float step) {
    base_percentage = target_pct;
    ramp_step = step;
    current_percentage = 0;
    // current_percentage = 10.0f; // 软启动从10%的占空比起步，避免占空比太小导致启动困难
    is_soft_starting = true;   // 设置标志位，让后续能在 Update_Tick 持续接力
    is_running = true;         // 先置高使能态
}

void BayWatcher_ESC::stop()
{
    is_running = false;
    is_soft_starting = false;
    base_percentage = 0.0f;
    current_percentage = 0.0f;
    TIM2CH4.SetDutyCycle(min_pwm);//左负压 IO77
    TIM2CH3.SetDutyCycle(min_pwm);//右负压 IO89
}

void BayWatcher_ESC::Update_Tick() {
    // 当系统未准备好或被停止时，直接退出，不干扰电机
    if (!is_running || !is_initialized) return;

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
        
        // 基础负压推力（默认等于软起动达到的稳态值）
        float target_base = base_percentage;

        // ================= 1. 动态负压计算 (解耦叠加) =================
        // 条件：启用且不在发车序列的阶段 1-3 之中（发车完成状态4或怠速状态0）
        if (enable_dynamic_esc && (PID.startup_state >= 4 || PID.startup_state == 0)) {
            float abs_f = std::fabs(current_factor);
            if (abs_f < dynamic_esc_threshold) {
                // 死区内，维持设定好的基础推力
                target_base = dynamic_esc_base;
            } else {
                // 归一化 factor 到 0.0 ~ 1.0
                float t = (abs_f - dynamic_esc_threshold) / (dynamic_esc_factor_max - dynamic_esc_threshold);
                if (t > 1.0f) t = 1.0f;
                if (t < 0.0f) t = 0.0f;
                
                // 应用 Smoothstep S-Curve 平滑插值函数: 3t^2 - 2t^3
                float smooth_t = t * t * (3.0f - 2.0f * t);
                
                // 根据插值结果计算动态目标负压
                target_base = dynamic_esc_base + (dynamic_esc_max - dynamic_esc_base) * smooth_t;
            }
            
            // 加入一阶低通滤波，极大平滑高频噪声引起的抖动
            filtered_target_base = filtered_target_base * (1.0f - lp_alpha) + target_base * lp_alpha;
            target_base = filtered_target_base;
        } else {
            // 如果没开动态负压或者正在软启动发车，强制复位滤波器
            filtered_target_base = target_base;
        }

        // ================= 2. 负压差速计算 (建立在 target_base 之上) =================
        if (enable_esc_diff) {
            // 将底盘舵机差速的修正偏差叠加给两侧负压电调
            float diff = PID.speed_adjust * esc_diff_ratio;
            
            #define ESC_LIMIT(x, min, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))
            // 差速叠加限幅
            diff = ESC_LIMIT(diff, -esc_diff_limit, esc_diff_limit);

            float esc1_target = target_base - diff; // 左轮目标负压
            float esc2_target = target_base + diff; // 右轮目标负压
            
            // 严苛的差速安全限幅 (0% ~ 100%)
            esc1_target = ESC_LIMIT(esc1_target, 0.0f, 100.0f);
            esc2_target = ESC_LIMIT(esc2_target, 0.0f, 100.0f);
            #undef ESC_LIMIT

            if (esc_log_cnt % 50 == 0) { // 每 250ms 打印一次
                printf("[ESC] 动态负压|基准:%.1f 差速:%.1f | 左:%.1f%% 右:%.1f%%\n", target_base, diff, esc1_target, esc2_target);
            }

            Esc1_Set((int16_t)esc1_target);  // 应用左外侧/内侧限幅后负压
            Esc2_Set((int16_t)esc2_target);  // 应用右外侧/内侧限幅后负压
        } else {
            // 无差速状态下双发同步动态/静态推力
            if (target_base > 100.0f) target_base = 100.0f;
            if (target_base < 0.0f) target_base = 0.0f;
            
            Esc1_Set((int16_t)target_base);
            Esc2_Set((int16_t)target_base);
        }
    }
}