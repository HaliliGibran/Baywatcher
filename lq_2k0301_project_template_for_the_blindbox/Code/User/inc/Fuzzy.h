#ifndef _BAYWATCHER_FUZZY_H_
#define _BAYWATCHER_FUZZY_H_

#include "main.hpp"

void calculateMembership (float value, float* liShu, float* liShuDu, const float* lunYu);
void get_degree_of_membership (float error);
void upPIDdata_pd(float* kp, float* kd);
void get_updataPD_pid (float error, float *kp, float *kd);

extern float KP_Base;     // 基础Kp (也就是你原本调好的静态Kp)
extern float KD_Base;     // 基础Kd
extern float KP_Fuzzy;    // 模糊增益系数 (模糊调节的力度)
extern float KD_Fuzzy;    // 模糊增益系数
extern float ERROR_MAX;   // 误差的最大范围 (超过这个值就认为是最大误差)

#endif