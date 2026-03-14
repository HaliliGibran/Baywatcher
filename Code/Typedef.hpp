// #ifndef _BAYWATCHER_TYPEDEF_H_
// #define _BAYWATCHER_TYPEDEF_H_

// #include "main.hpp"

// #pragma region KEY_TYPEDEF_REGION

// // 按键操作枚举
// enum class Bay_KeyOp_t { Up, Down, OK, Cancel, Back, None };

// #pragma endrefion


// #pragma region PID_TYPEDEF_REGION

// // 增量式 PID
// typedef struct {
//     float Kp, Ki, Kd;
//     float error, prev_error, last_error;
//     float error_i;      // 积分相差值
//     float output;       // 当前累计输出 (PWM)
//     float output_limit; // PWM限幅 (10000)
//     float integral_limit; // 积分限幅
// } Bay_IncPID_t;

// // 位置式 PID 
// typedef struct {
//     float Kp, Ki, Kd;
//     float error, integral, last_error;
//     float output;       
//     float output_limit; 
//     float integral_limit;
// } Bay_PosPID_t;

// // 三次曲线拟位置式PID
// typedef struct {
//     float Kpa, Kpb, Ki, Kd;
//     float error, integral, last_error;
//     float output;       
//     float output_limit;
//     float integral_limit;
// } Bay_CubPID_t;

// // 模糊自适应PID
// typedef struct {
//     float KP_Base, KD_Base, Ki, KP_Fuzzy, KD_Fuzzy;
//     float error, integral, last_error;
//     float output;       
//     float output_limit; // 差速限幅 
//     float integral_limit;
// } Bay_FuzzyPID_t;

// //*******************************Error论域********************************//

// #define ER_ZO0                              (0.0f)
// #define ER_PSS                              (3.5f)
// #define ER_PSB                              (4.0f)
// #define ER_PMS                              (4.5f)
// #define ER_PMB                              (5.0f)
// #define ER_PBS                              (5.5f)
// #define ER_PBB                              (6.0f)

// const float Er_lunYu[7] = {ER_ZO0, ER_PSS, ER_PSB, ER_PMS, ER_PMB, ER_PBS, ER_PBB};

// //*******************************debt论域*********************************//

// #define DT_NB                               (-3.0f)
// #define DT_NM                               (-2.0f)
// #define DT_NS                               (-1.0f)
// #define DT_Z0                               (0.0f)
// #define DT_PS                               (1.0f)
// #define DT_PM                               (2.0f)
// #define DT_PB                               (3.0f)

// const float Dedt_lunYu[7] = {DT_NB, DT_NM, DT_NS, DT_Z0, DT_PS, DT_PM, DT_PB};

// //*******************************隶属度*********************************//

// static float er_liShu[2];                      //er隶属
// static float dedt_liShu[2];                    //dedt隶属
// static float er_liShuDu[2];                    //er隶属度
// static float dedt_liShuDu[2];                  //dedt隶属度

// //*******************************KP Rule*********************************//

// #define KP_ZO0                              (0.0f)
// #define KP_PSS                              (1.0f)
// #define KP_PSB                              (2.0f)
// #define KP_PMS                              (3.0f)
// #define KP_PMB                              (4.0f)
// #define KP_PBS                              (5.0f)
// #define KP_PBB                              (6.0f)
// const float KP_Rule[7][7] = {
// /*   er/dedt         DT_NB   DT_NM   DT_NS   DT_Z0   DT_PS   DT_PM   DT_PB */
//     /*ER_ZO0*/      {KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0},
//     /*ER_PSS*/      {KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS},
//     /*ER_PSB*/      {KP_PSS, KP_PSS, KP_PSS, KP_PSB, KP_PMS, KP_PMS, KP_PMS},
//     /*ER_PMS*/      {KP_PSB, KP_PSB, KP_PMS, KP_PMS, KP_PMS, KP_PMB, KP_PMB},
//     /*ER_PMB*/      {KP_PMS, KP_PMS, KP_PMB, KP_PMB, KP_PMB, KP_PBS, KP_PBS},
//     /*ER_PBS*/      {KP_PMB, KP_PBS, KP_PBS, KP_PBS, KP_PBS, KP_PBS, KP_PBB},
//     /*ER_PBB*/      {KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB},
// };

// //*******************************KD Rule*********************************//

// #define KD_ZO0                              (0.0f)
// #define KD_PSS                              (1.0f)
// #define KD_PSB                              (2.0f)
// #define KD_PMS                              (3.0f)
// #define KD_PMB                              (4.0f)
// #define KD_PBS                              (5.0f)
// #define KD_PBB                              (6.0f)
// const float KD_Rule[7][7] = {
// /*  er/dedt          DT_NB   DT_NM   DT_NS   DT_Z0   DT_PS   DT_PM   DT_PB */
//     /*ER_ZO0*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMB, KD_PMB, KD_PMS, KD_PSB},
//     /*ER_PSS*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMB, KD_PMB, KD_PMS, KD_PSB},
//     /*ER_PSB*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMS, KD_PMS, KD_PSB, KD_PSB},
//     /*ER_PMS*/      {KD_PBS, KD_PMB, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS},
//     /*ER_PMB*/      {KD_PMB, KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSS, KD_ZO0},
//     /*ER_PBS*/      {KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS, KD_ZO0},
//     /*ER_PBB*/      {KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS, KD_ZO0},
// };
// #pragma endrefion

// #pragma region IMU_TYPEDEF_REGION
// #pragma endrefion


















// #pragma region COM_TYPEDEF_REGION

// // VOFA+通信模式枚举
// enum class VOFA_ComType_t{
//     FireWater,
//     JustFloat,
//     Rawdata
// };


// #pragma endrefion

// #endif