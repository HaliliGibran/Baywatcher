#ifndef _BAYWATCHER_MOTOR_H_
#define _BAYWATCHER_MOTOR_H_

#include <stdint.h>

#define Motor1_N 21
#define Motor2_N 22

extern AtimPwm Motor1;
extern AtimPwm Motor2;
extern HWGpio Motor1_DIR;
extern HWGpio Motor2_DIR;


// ================== 电机控制 ==================
class BayWatcher_Motor
{
public:
    BayWatcher_Motor();
    ~BayWatcher_Motor();
    void init(void);
    void Motor1_Set(int16_t duty);
    void Motor2_Set(int16_t duty);
    void Stop(void);
private:
    uint16_t hw_duty_1;
    uint16_t hw_duty_2;
    const uint16_t MAX_LOGIC_DUTY = 10000;
};

// ================= 发车管理状态机 =================
enum class StartMode {
    WITH_ESC,   // 模式1：带负压延时发车
    NO_ESC      // 模式2：无负压直接发车
};

enum class StartState {
    STOPPED,        // 停车状态
    WAITING_ESC,    // 负压建立中
    RUNNING         // 延时结束，允许发车
};
// ================= 阶梯加速 =================
class BayWatcher_Speed_Ramper {
public:
    BayWatcher_Speed_Ramper();
    
    // 每次重新发车前调用，清空累计的速度
    void Reset();

    // 核心函数：传入目标速度和步长，返回平滑后的当前速度
    float Calc_Ramped_Speed(float target_speed, float step);

private:
    float current_ramped_speed;
};

extern BayWatcher_Speed_Ramper speed_ramper;
#endif // _BAYWATCHER_MOTOR_H_