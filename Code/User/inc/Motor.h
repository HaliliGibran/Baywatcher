#ifndef _BAYWATCHER_MOTOR_H_
#define _BAYWATCHER_MOTOR_H_

#include <stdint.h>

#define Motor1_N 21
#define Motor2_N 22

extern AtimPwm Motor1;
extern AtimPwm Motor2;
extern HWGpio Motor1_DIR;
extern HWGpio Motor2_DIR;

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
    // 逻辑限幅值 
    const uint16_t MAX_LOGIC_DUTY = 10000;
};

#endif // _BAYWATCHER_MOTOR_H_