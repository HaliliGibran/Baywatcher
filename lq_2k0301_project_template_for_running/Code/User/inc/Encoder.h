#ifndef _BAYWATCHER_ENCODER_H_
#define _BAYWATCHER_ENCODER_H_

#include "main.hpp"

// 测速模式枚举
enum class SpeedMethod {
    M_Method,  // 频率法：适合高速，数据平滑
    T_Method   // 周期法：适合低速/平衡，响应极快
};

extern float vL;
extern float vR;
extern float v_avg;
extern LS_PwmEncoder Encoder1; //64//编码器1
extern LS_PwmEncoder Encoder2; //65//编码器2
// extern LS_PwmEncoder Encoder3; //66
// extern LS_PwmEncoder Encoder4; //67
// extern LQ_AB_Encoder Encoder_L;
// extern LQ_AB_Encoder Encoder_R;

//物理参数
#define WHEEL_DIAMETER_CM     6.50f        //直径(cm)
#define PI                    3.14159f
const float ENCODER_PHYSICAL_SCALE = (WHEEL_DIAMETER_CM * PI);
const float PULSES_PER_REV = 1024.0f;       // 编码器线数 (M法直接用线数)

class BayWatcher_Encoder
{
public:
    BayWatcher_Encoder();
    ~BayWatcher_Encoder();
    void init();                                     // 初始化硬件
    void setMethod(SpeedMethod method);             // 切换测速模式
    float getSpeedL();                               // 获取左轮物理速度 (cm/s)
    float getSpeedR();                               // 获取右轮物理速度 (cm/s)
    void clear();                                    // 状态清零

private:
    SpeedMethod _method;
    float _filter_l, _filter_r;                      // 内部滤波缓存
    const float _alpha = 0.6f;                       // M法模拟滤波系数

    float _read_raw_l();                             // 读取左轮原始RPS
    float _read_raw_r();                             // 读取右轮原始RPS
};
#endif