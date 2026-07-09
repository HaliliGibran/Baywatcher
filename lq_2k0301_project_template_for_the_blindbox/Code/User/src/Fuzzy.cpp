#include "Fuzzy.h"

// float ERROR_MAX = 40.0;                         //误差最大值
// float DEDT_MAX = 1.0;                           //误差变化率最大值(d(error)/dt)
// float KP_Fuzzy = 2.0;                           //kp模糊值
// float KD_Fuzzy = 10.0;                          //kd模糊值

float ERROR_MAX = 30.0;                         //误差最大值
float DEDT_MAX = 1.0;                           //误差变化率最大值(d(error)/dt)
float KP_Fuzzy = 0.0;                           //kp模糊值
float KD_Fuzzy = 0.0;                          //kd模糊值

float KP_Base = 0;                              //kp基础值
float KD_Base = 0;                              //kd基础值

//*******************************Error论域********************************//

#define ER_ZO0                              (0.0f)
#define ER_PSS                              (3.5f)
#define ER_PSB                              (4.0f)
#define ER_PMS                              (4.5f)
#define ER_PMB                              (5.0f)
#define ER_PBS                              (5.5f)
#define ER_PBB                              (6.0f)

const float Er_lunYu[7] = {ER_ZO0, ER_PSS, ER_PSB, ER_PMS, ER_PMB, ER_PBS, ER_PBB};

//*******************************debt论域*********************************//

#define DT_NB                               (-3.0f)
#define DT_NM                               (-2.0f)
#define DT_NS                               (-1.0f)
#define DT_Z0                               (0.0f)
#define DT_PS                               (1.0f)
#define DT_PM                               (2.0f)
#define DT_PB                               (3.0f)

const float Dedt_lunYu[7] = {DT_NB, DT_NM, DT_NS, DT_Z0, DT_PS, DT_PM, DT_PB};

//*******************************隶属度*********************************//

static float er_liShu[2];                      //er隶属
static float dedt_liShu[2];                    //dedt隶属
static float er_liShuDu[2];                    //er隶属度
static float dedt_liShuDu[2];                  //dedt隶属度

//*******************************KP Rule*********************************//

#define KP_ZO0                              (0.0f)
#define KP_PSS                              (1.0f)
#define KP_PSB                              (2.0f)
#define KP_PMS                              (3.0f)
#define KP_PMB                              (4.0f)
#define KP_PBS                              (5.0f)
#define KP_PBB                              (6.0f)
const float KP_Rule[7][7] = {
/*   er/dedt         DT_NB   DT_NM   DT_NS   DT_Z0   DT_PS   DT_PM   DT_PB */
    /*ER_ZO0*/      {KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0},
    /*ER_PSS*/      {KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS},
    /*ER_PSB*/      {KP_PSS, KP_PSS, KP_PSS, KP_PSB, KP_PMS, KP_PMS, KP_PMS},
    /*ER_PMS*/      {KP_PSB, KP_PSB, KP_PMS, KP_PMS, KP_PMS, KP_PMB, KP_PMB},
    /*ER_PMB*/      {KP_PMS, KP_PMS, KP_PMB, KP_PMB, KP_PMB, KP_PBS, KP_PBS},
    /*ER_PBS*/      {KP_PMB, KP_PBS, KP_PBS, KP_PBS, KP_PBS, KP_PBS, KP_PBB},
    /*ER_PBB*/      {KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB},
};

//*******************************KD Rule*********************************//

#define KD_ZO0                              (0.0f)
#define KD_PSS                              (1.0f)
#define KD_PSB                              (2.0f)
#define KD_PMS                              (3.0f)
#define KD_PMB                              (4.0f)
#define KD_PBS                              (5.0f)
#define KD_PBB                              (6.0f)
const float KD_Rule[7][7] = {
/*  er/dedt          DT_NB   DT_NM   DT_NS   DT_Z0   DT_PS   DT_PM   DT_PB */
    /*ER_ZO0*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMB, KD_PMB, KD_PMS, KD_PSB},
    /*ER_PSS*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMB, KD_PMB, KD_PMS, KD_PSB},
    /*ER_PSB*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMS, KD_PMS, KD_PSB, KD_PSB},
    /*ER_PMS*/      {KD_PBS, KD_PMB, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS},
    /*ER_PMB*/      {KD_PMB, KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSS, KD_ZO0},
    /*ER_PBS*/      {KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS, KD_ZO0},
    /*ER_PBB*/      {KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS, KD_ZO0},
};


/* 函数用处：   隶属度获取
 *
 * 传入参数：   float value         映射到论域上的值
 * 传入参数：   float* liShu        映射值的隶属
 * 传入参数：   float* liShuDu      映射值的隶属度
 * 传入参数：   const int* lunYu    论域
 */
void calculateMembership (float value, float* liShu, float* liShuDu, const float* lunYu)
{
    if (value > lunYu[0] && value < lunYu[6]) {
        for (int i = 0; i < 6; i++) {
            if (lunYu[i] < value && value <= lunYu[i + 1]) {             /* 判断在哪个区间内 */
                /* 三角隶属度 */
                liShu[0] = i;                                                                   /* 记录前隶属 */
                liShu[1] = i + 1;                                                               /* 记录后隶属 */
                liShuDu[0] = (value - lunYu[i]) / (lunYu[i + 1] - lunYu[i]);                    /* 记录前隶属度 */
                liShuDu[1] = (lunYu[i + 1] - value) / (lunYu[i + 1] - lunYu[i]);                /* 记录后隶属度 */
                break;
            }
        }
    } else if (value <= lunYu[0]) {
        /* 三角隶属度 */
        liShu[0] = 0;                   /* 记录前隶属 */
        liShu[1] = 1;                   /* 记录后隶属 */
        liShuDu[0] = 1.0;               /* 记录前隶属度(100%) */
        liShuDu[1] = 0;                 /* 记录后隶属度 */
    } else if (value >= lunYu[6]) {
        /* 三角隶属度 */
        liShu[0] = 5;                   /* 记录前隶属 */
        liShu[1] = 6;                   /* 记录后隶属 */
        liShuDu[0] = 0;                 /* 记录前隶属度 */
        liShuDu[1] = 1.0;               /* 记录后隶属度(100%) */
    }
}

/* 函数用处：   获取er与de/dt的隶属度
 * 
 * 传入参数：   float er        当前差值
 */
void get_degree_of_membership (float error)
{
    static float laste_error = 0;
    float dedt = error - laste_error;

    float error_Remap = error / ERROR_MAX * ER_PBB;        /* 对er映射 */
    float dedtRemap = dedt / DEDT_MAX * DT_PB;            /* 对dedt映射 */

    //误差为负时 取反
    if(error_Remap < 0) {
        error_Remap = -error_Remap;
        dedtRemap = -dedtRemap;
    }
    //误差在正负震荡时让KD最大
    if(laste_error * error < 0) {
        dedtRemap = -DT_NB;
    }

    calculateMembership(error_Remap, er_liShu, er_liShuDu, Er_lunYu);
    calculateMembership(dedtRemap, dedt_liShu, dedt_liShuDu, Dedt_lunYu);

    laste_error = error;
}

/* 函数用处：   更新pd参数
 *
 * 传入参数：   float *kp           传递pid->kp用于更新
 * 传入参数：   float *kd           传递pid->kd用于更新
 */
void upPIDdata_pd(float* kp, float* kd)
{
    float kptmp = er_liShuDu[0] * dedt_liShuDu[0] * KP_Rule[(int)er_liShu[0]][(int)dedt_liShu[0]]
                + er_liShuDu[0] * dedt_liShuDu[1] * KP_Rule[(int)er_liShu[0]][(int)dedt_liShu[1]]
                + er_liShuDu[1] * dedt_liShuDu[0] * KP_Rule[(int)er_liShu[1]][(int)dedt_liShu[0]]
                + er_liShuDu[1] * dedt_liShuDu[1] * KP_Rule[(int)er_liShu[1]][(int)dedt_liShu[1]];

    float kdtmp = er_liShuDu[0] * dedt_liShuDu[0] * KD_Rule[(int)er_liShu[0]][(int)dedt_liShu[0]]
                + er_liShuDu[0] * dedt_liShuDu[1] * KD_Rule[(int)er_liShu[0]][(int)dedt_liShu[1]]
                + er_liShuDu[1] * dedt_liShuDu[0] * KD_Rule[(int)er_liShu[1]][(int)dedt_liShu[0]]
                + er_liShuDu[1] * dedt_liShuDu[1] * KD_Rule[(int)er_liShu[1]][(int)dedt_liShu[1]];

    kptmp = (kptmp / KP_PBB) * KP_Fuzzy;
    kdtmp = (kdtmp / KD_PBB) * KD_Fuzzy;

    *kp = KP_Base + kptmp;
    *kd = KD_Base + kdtmp;
}

/* 函数用处：   模糊PD控制器
 *
 * 传入参数：   float er        差值
 * 传入参数：   float *kp       传递pid->kp用于更新
 * 传入参数：   float *kd       传递pid->kd用于更新
 */
void get_updataPD_pid (float error, float *kp, float *kd)
{   
    //实时更新
    get_degree_of_membership(error);
    upPIDdata_pd(kp, kd);

    //(2)次进行一次模糊更新更新
    // static int counter=0;
    // if(counter%2==0){
    //     upPIDdata_pd(kp, kd);
    // }
    // counter+=counter;
}