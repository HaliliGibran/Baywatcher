#ifndef _BAYWATCHER_ESC_H_
#define _BAYWATCHER_ESC_H_

#include "main.hpp"

extern GtimPwm TIM2CH3;
extern GtimPwm TIM2CH4;

class BayWatcher_ESC{
public:
    BayWatcher_ESC();
    ~BayWatcher_ESC();
    void init();
    void start();
    void Esc1_Set(int16_t percentage);
    void Esc2_Set(int16_t percentage);
    // 启动多状态软起动机制
    void Start_Soft(float target_pct, float step);
    
    // 系统主频节拍更新（需在定时器内循环调用）
    void Update_Tick();
    
    // 紧急停止电调及复位
    void stop();

    // ================== 系统核心状态与标志位 ==================
    // 听取您的建议，将这批状态判断量置于 public 区方便全局自由调用。
    // 为了防止 Menu.cpp 强制将 bool 转为 float* 导致的内存越权覆写，
    // 将原本的 bool enable_esc_diff 直接改为 float 类型（非零为真），从根本上保证安全。
    bool is_running;               // 系统运行标志位
    bool is_soft_starting;         // 标志位：1=正在软启动（持续爬升中），0=已爬升完毕或未开启软起
    bool enable_esc_diff;          // 是否开启负压差速 (UI配置项)
    float esc_diff_ratio;          // 负压差速补偿强度 (UI配置项)
    float esc_diff_limit;          // 负压差速限幅 (UI配置项)

private:
    bool is_initialized = false;


    const uint32_t max_pwm = 1000; // 电调满油门 PWM 最大值限制
    const uint32_t min_pwm = 500;  // 电调最低油门 PWM 怠速值

    uint16_t duty;                 // 临时计算的占空比
    uint16_t hw_duty_1;
    uint16_t hw_duty_2;

    float base_percentage;    // 软起动最终要达到/稳定保持的目标负压百分比 (0~100)
    float current_percentage; // 当前实时正在爬升的负压百分比
    float ramp_step;          // 爬升过程每步的斜率步长
};

extern BayWatcher_ESC esc_sys;
#endif // !_BAYWATCHER_ESC_H_