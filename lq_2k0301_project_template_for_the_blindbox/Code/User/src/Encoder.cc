#include "main.hpp"

float vL=0.0f;
float vR=0.0f;
float v_avg= 0;

LS_PwmEncoder Encoder1(0, 72); //64
LS_PwmEncoder Encoder2(1, 73); //65
// LS_PwmEncoder Encoder3(2, 74);//66
// LS_PwmEncoder Encoder4(3, 75); //67
// LQ_AB_Encoder Encoder_L(64, 72);
// LQ_AB_Encoder Encoder_R(65, 73);


BayWatcher_Encoder::BayWatcher_Encoder(){ clear(); }
BayWatcher_Encoder::~BayWatcher_Encoder() {}

void BayWatcher_Encoder::init(void) {
    Encoder1.Init();
    Encoder2.Init();
    // Encoder_L.Init();
    // Encoder_R.Init();
    clear();
    printf("--- Encoders Initialized and Cleared ---\n");
}

void BayWatcher_Encoder::clear() {
    _filter_l = 0.0f; 
    _filter_r = 0.0f;
}

void BayWatcher_Encoder::setMethod(SpeedMethod method) { _method = method;}

float BayWatcher_Encoder::getSpeedL() {
    float raw_rps = _read_raw_l();
    // float speed_now = raw_rps * ENCODER_PHYSICAL_SCALE;
    float speed_now = raw_rps;

    if (_method == SpeedMethod::T_Method) {
        return -speed_now; // T法直接返回瞬时值
    } else {
        // M法模拟：通过固定频率采样 + 低通滤波实现平滑
        _filter_l = (_alpha * speed_now) + ((1.0f - _alpha) * _filter_l);
        return -_filter_l;
    }
}

float BayWatcher_Encoder::getSpeedR() {
    float raw_rps = _read_raw_r();
    // float speed_now = raw_rps * ENCODER_PHYSICAL_SCALE;
    float speed_now = raw_rps;

    if (_method == SpeedMethod::T_Method) {
        return speed_now; // 右轮通常需要取反
    } else {
        _filter_r = (_alpha * speed_now) + ((1.0f - _alpha) * _filter_r);
        return _filter_r;
    }
}


float BayWatcher_Encoder::_read_raw_l() { return Encoder1.Update(); }
float BayWatcher_Encoder::_read_raw_r() { return Encoder2.Update(); }