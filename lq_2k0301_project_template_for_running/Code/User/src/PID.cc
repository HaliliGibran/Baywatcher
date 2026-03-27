#include "main.hpp"
#include <cmath>

// ============================ 弯道减速配置 ============================
// 基于图像侧输出的 average_curvature，对基础巡线速度做连续减速。
// 设计目标：
// 1. 菜单/VOFA 设置的 base_target_speed 仍表示“手动设定速度”
// 2. 控制环内部再生成一个 yaw_control_base_speed 作为“实际生效速度”
// 3. 急弯时快速减速，出弯时更平缓地恢复，避免速度指令突然跳变
// 调参建议：
// - 觉得弯道减速介入太早：调大 CURV_LOW / CURV_HIGH
// - 觉得弯道减速不够狠：调小 MIN_SPEED_RATIO
// - 觉得出弯恢复太慢：调大 ALPHA_UP
// - 觉得进弯减速还不够快：调大 ALPHA_DOWN
// 总开关：1=启用 average_curvature 弯道减速，0=完全关闭，恢复为手动基准速度直通。
#ifndef BW_ENABLE_CURVE_SLOWDOWN
#define BW_ENABLE_CURVE_SLOWDOWN 1
#endif

// 环岛减速开关：
// 使用位置：PID.cc / curve_slowdown_roundabout_target_speed。
// 作用：控制“环岛阶段是否启用固定减速”。
// 1：进入环岛后不再使用 average_curvature，直接切到固定减速速度。
// 0：进入环岛后不减速，yaw_control_base_speed 立即回到手动基准速度。
// 推荐：如果环岛内曲率波动较大、连续减速会导致速度抖动，保持开启更稳。
#ifndef BW_ENABLE_ROUNDABOUT_CURVE_SLOWDOWN
#define BW_ENABLE_ROUNDABOUT_CURVE_SLOWDOWN 1
#endif

// 环岛固定减速比例：
// 使用位置：PID.cc / curve_slowdown_roundabout_target_speed。
// 作用：当环岛减速开关开启时，环岛内实际生效速度 = 手动基准速度 * 该比例。
// 调小：环岛更慢、更稳。
// 调大：环岛更快、更接近普通巡线速度。
// 默认值 0.92 约等于当前曲率减速链在 average_curvature≈0.015 附近的典型减速强度。
#ifndef BW_ROUNDABOUT_FIXED_SLOWDOWN_RATIO
#define BW_ROUNDABOUT_FIXED_SLOWDOWN_RATIO 0.76f
#endif

// 曲率减速下限：average_curvature 低于该值时，不触发减速。
// 调大：更晚开始减速；调小：更早开始减速。
#ifndef BW_CURVE_SLOWDOWN_CURV_LOW
#define BW_CURVE_SLOWDOWN_CURV_LOW 0.020f
#endif

// 曲率减速上限：average_curvature 高于该值时，认为进入“最大减速区”。
// 调大：只有更急的弯才会触发最大减速；调小：更容易打满减速。
#ifndef BW_CURVE_SLOWDOWN_CURV_HIGH
// #define BW_CURVE_SLOWDOWN_CURV_HIGH 0.120f
#define BW_CURVE_SLOWDOWN_CURV_HIGH 0.090f
#endif

// 最小速度比例：急弯时实际生效速度不会低于“手动设定速度 * 该比例”。
// 例如 0.65 表示最大只减到 65%。
// 调小：急弯更慢、更稳；调大：减速更弱。
#ifndef BW_CURVE_SLOWDOWN_MIN_SPEED_RATIO
#define BW_CURVE_SLOWDOWN_MIN_SPEED_RATIO 0.75f
#endif

// 最小绝对速度保护：当手动设定速度本身较高时，急弯减速也不会低于该绝对速度。
// 作用：防止曲率很大时被压得过慢，导致车体发呆或失去流畅性。
// 调小：允许更低速过弯；调大：保留更多底速。
#ifndef BW_CURVE_SLOWDOWN_MIN_SPEED_ABS
#define BW_CURVE_SLOWDOWN_MIN_SPEED_ABS 6.0f
#endif

// 进弯减速平滑系数：目标速度变小时使用。
// 调大：减速跟随更快、更果断；调小：减速更柔和，但可能来不及。
#ifndef BW_CURVE_SLOWDOWN_ALPHA_DOWN
// #define BW_CURVE_SLOWDOWN_ALPHA_DOWN 0.35f
#define BW_CURVE_SLOWDOWN_ALPHA_DOWN 0.25f
#endif

// 出弯恢复平滑系数：目标速度变大时使用。
// 调大：恢复速度更快；调小：恢复更慢，更不容易在弯后突然窜车。
#ifndef BW_CURVE_SLOWDOWN_ALPHA_UP
#define BW_CURVE_SLOWDOWN_ALPHA_UP 0.010f
#endif

namespace {

struct CurveSlowdownState {
    bool initialized = false;
};

static CurveSlowdownState g_curve_slowdown;

static inline float clampf_pid(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 十字阶段不做基于曲率的减速：
// 十字内的曲率会受到远端线、状态切换和中线拼接影响，直接拿来减速容易误触发。
static bool curve_slowdown_crossing_blocked()
{
    return (element_type == ElementType::CROSSING) ||
           (crossing_state != CrossingState::CROSSING_NONE);
}

// 当前是否处于环岛阶段：
// element_type 和 circle_state 双条件并存，兼容“元素刚进入/刚退出”时的短暂过渡帧。
static bool curve_slowdown_roundabout_active()
{
    return (element_type == ElementType::CIRCLE) ||
           (circle_state != CircleState::CIRCLE_NONE);
}

// 环岛阶段的固定减速目标：
// - 开关开启：直接切到固定比例速度
// - 开关关闭：不减速，回到手动基准速度
static float curve_slowdown_roundabout_target_speed(float manual_base_speed)
{
#if BW_ENABLE_ROUNDABOUT_CURVE_SLOWDOWN
    const float ratio = clampf_pid(BW_ROUNDABOUT_FIXED_SLOWDOWN_RATIO, 0.0f, 1.0f);
    return manual_base_speed * ratio;
#else
    return manual_base_speed;
#endif
}

static void reset_curve_slowdown_state(float base_speed)
{
    g_curve_slowdown.initialized = false;
    PID.yaw_control_base_speed = base_speed;
}

// 依据平均曲率把“手动基准速度”映射成“期望有效速度”。
static float curve_slowdown_target_speed(float manual_base_speed)
{
#if BW_ENABLE_CURVE_SLOWDOWN
    const float abs_base = std::fabs(manual_base_speed);
    if (abs_base <= 1e-4f)
    {
        return 0.0f;
    }

    if (curve_slowdown_crossing_blocked())
    {
        return manual_base_speed;
    }

    if (curve_slowdown_roundabout_active())
    {
        return curve_slowdown_roundabout_target_speed(manual_base_speed);
    }

    const float curve_low = BW_CURVE_SLOWDOWN_CURV_LOW;
    float curve_high = BW_CURVE_SLOWDOWN_CURV_HIGH;
    if (curve_high < curve_low)
    {
        curve_high = curve_low;
    }

    float t = 0.0f;
    if (curve_high > curve_low)
    {
        t = (average_curvature - curve_low) / (curve_high - curve_low);
        t = clampf_pid(t, 0.0f, 1.0f);
    }
    else if (average_curvature >= curve_high)
    {
        t = 1.0f;
    }

    const float min_ratio = clampf_pid(BW_CURVE_SLOWDOWN_MIN_SPEED_RATIO, 0.0f, 1.0f);
    float desired_abs = abs_base * (1.0f - t * (1.0f - min_ratio));

    if (abs_base > BW_CURVE_SLOWDOWN_MIN_SPEED_ABS && desired_abs < BW_CURVE_SLOWDOWN_MIN_SPEED_ABS)
    {
        desired_abs = BW_CURVE_SLOWDOWN_MIN_SPEED_ABS;
    }
    if (desired_abs > abs_base)
    {
        desired_abs = abs_base;
    }

    return (manual_base_speed >= 0.0f) ? desired_abs : -desired_abs;
#else
    return manual_base_speed;
#endif
}

// 对弯道减速目标做平滑：进弯减速快一些，出弯恢复慢一些。
static float update_curve_slowdown_base_speed(float manual_base_speed)
{
    if (curve_slowdown_crossing_blocked())
    {
        g_curve_slowdown.initialized = true;
        PID.yaw_control_base_speed = manual_base_speed;
        return PID.yaw_control_base_speed;
    }

    if (curve_slowdown_roundabout_active())
    {
        g_curve_slowdown.initialized = true;
        PID.yaw_control_base_speed = curve_slowdown_roundabout_target_speed(manual_base_speed);
        return PID.yaw_control_base_speed;
    }

    const float target_speed = curve_slowdown_target_speed(manual_base_speed);
    if (!g_curve_slowdown.initialized)
    {
        g_curve_slowdown.initialized = true;
        PID.yaw_control_base_speed = target_speed;
        return PID.yaw_control_base_speed;
    }

    const float alpha_down = clampf_pid(BW_CURVE_SLOWDOWN_ALPHA_DOWN, 0.0f, 1.0f);
    const float alpha_up = clampf_pid(BW_CURVE_SLOWDOWN_ALPHA_UP, 0.0f, 1.0f);
    const float alpha = (std::fabs(target_speed) < std::fabs(PID.yaw_control_base_speed)) ? alpha_down : alpha_up;

    PID.yaw_control_base_speed += alpha * (target_speed - PID.yaw_control_base_speed);
    return PID.yaw_control_base_speed;
}

} // namespace

//============================ PID算法分区 =============================

// 增量式 PID 算法
static float Calc_Inc_PID(Bay_IncPID_t *pid, float target, float measured) {
    pid->error = target - measured;
    float inc = (pid->Kp * (pid->error-pid->prev_error)) +
                (pid->Ki * (pid->error)) +
                (pid->Kd * (pid->error - 2 * pid->prev_error + pid->last_error));
    pid->output += inc;

    // //增量限幅
    // float max_inc = 500.0f; 
    // if (inc > max_inc) inc = max_inc;
    // if (inc < -max_inc) inc = -max_inc;

    // PWM 限幅
    if (pid->output > pid->output_limit) pid->output = pid->output_limit;
    if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;
    pid->last_error = pid->prev_error;
    pid->prev_error = pid->error;
    return pid->output;
}

// 位置式 PID 算法
static float Calc_Pos_PID(Bay_PosPID_t *pid, float target, float measured) {
    pid->error = target - measured;
    pid->integral += pid->error;
    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float output = (pid->Kp * pid->error) + 
                   (pid->Ki * pid->integral) + 
                   (pid->Kd * (pid->error - pid->last_error));
    pid->last_error = pid->error;
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    return output;
}

// 三次曲线拟位置式PID
static float Cube_Pos_PID(Bay_CubePID_t *pid, float target, float measured) {
    pid->error = target - measured;
    pid->integral += pid->error;
    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float output = (pid->Kp_a * pid->error) + 
                //    (pid->Kp_b * pid->error * pid->error * pid->error) +
                   (pid->Kp_b * pid->error * pid->error ) +
                   (pid->Ki * pid->integral) + 
                   (pid->Kd_a * (pid->error - pid->last_error));
                   (pid->Kd_b * pid->gyro );
    pid->last_error = pid->error;
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    return output;
}

// 前馈位置式PID
static float Feedforward_PID(Bay_FforwardPID_t *pid, float target, float measured)
{
    pid->error = target - measured;
    pid->integral += pid->error;
    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float feedforward = pid ->Kff *target ;
    float output = (pid->Kp * pid->error) + 
                   (pid->Ki * pid->integral) + 
                   (pid->Kd * (pid->error - pid->last_error)) +feedforward;
    pid->last_error = pid->error;
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    return output;
}

//============================ 控制算法分区 ==============================

void BayWatcher_Control_Init(void) {
    memset(&PID,           0, sizeof(Bay_PID_t));    // PID基础状态
    memset(&PID_Speed_L,   0, sizeof(Bay_IncPID_t)); // 左轮增量式
    memset(&PID_Speed_R,   0, sizeof(Bay_IncPID_t)); // 右轮增量式
    memset(&PID_Speed_F_L, 0, sizeof(Bay_PosPID_t)); // 前进环左轮位置式
    memset(&PID_Speed_F_R, 0, sizeof(Bay_PosPID_t)); // 前进环右轮位置式
    memset(&PID_Angle,     0, sizeof(Bay_PosPID_t)); // 角度环位置式
    memset(&PID_YawSpeed,  0, sizeof(Bay_PosPID_t)); // 角速度环位置式
    memset(&PID_Cube,      0 ,sizeof(Bay_CubePID_t)); // 三次拟位置环

    // 参数初始化
    PID.is_running = 0;         // 默认停止
    PID.base_target_speed = 0;  // 基础速度 0
    PID.target_angle = 0;       // 目标角度 0 (走直线)
    PID.target_yaw_speed = 0;       // 目标角速度 0 (走直线)
    PID.yaw_control_base_speed = 0; // 实际生效的基准速度
    vL = 0;
    vR = 0;

    // // 左轮速度环PID
    // PID_Speed_L.Kp = 105.32f; PID_Speed_L.Ki = 7.15f; PID_Speed_L.Kd = 0.00f;
    // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 7000.0f;

    // // 右轮速度环PID
    // PID_Speed_R.Kp = 105.00f; PID_Speed_R.Ki = 7.09f; PID_Speed_R.Kd = 0.00f;
    // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 7000.0f;

        // 左轮速度环PID
    PID_Speed_L.Kp = 40.32f; PID_Speed_L.Ki = 5.50f; PID_Speed_L.Kd = 0.00f;
    PID_Speed_L.output = 0; PID_Speed_L.output_limit = 7000.0f;

    // 右轮速度环PID
    PID_Speed_R.Kp = 40.00f; PID_Speed_R.Ki = 5.50f; PID_Speed_R.Kd = 0.00f;
    PID_Speed_R.output = 0; PID_Speed_R.output_limit = 7000.0f;

    // // 左轮前进环PID
    // PID_Speed_F_L.Kp = 0.00f; PID_Speed_F_L.Ki = 0.00f; PID_Speed_F_L.Kd = 0.00f;
    // PID_Speed_F_L.integral_limit= 0; PID_Speed_F_L.output_limit = 6000;

    // // 左轮前进环PID
    // PID_Speed_F_R.Kp = 0.00f; PID_Speed_F_R.Ki = 0.00f; PID_Speed_F_R.Kd = 0.00f;
    // PID_Speed_F_R.integral_limit= 0; PID_Speed_F_R.output_limit = 6000;
    
    // // 角速度环PID
    // PID_YawSpeed.Kp = 0.00f;  
    // PID_YawSpeed.Ki = 0.0f;   // 角速度环一般不需要 I，容易导致滞后
    // PID_YawSpeed.Kd = 0.0f;  // 抑制震荡   
    // PID_YawSpeed.output_limit = 40.0f;  PID_YawSpeed.integral_limit = 100.0f;
    
    // // 角度环PID
    // PID_Angle.Kp = 0.0f;     
    // PID_Angle.Ki = 0.0f;      // 角度环也不建议给 I，除非走直线总是歪
    // PID_Angle.Kd = 0.0f;      // 抑制震荡
    // PID_Angle.output_limit = 360.0f; // 最大允许角速度 (度/秒)
    // PID_Angle.integral_limit = 100.0f;

    
    // 模糊PID
    KP_Base = 0.1f;   KD_Base = 0.8f;  
    PID_Angle.Kp = 0.0f;  PID_Angle.Ki = 0.0f; PID_Angle.Kd = 0.0f;   
    PID_Angle.output_limit =  360.0f; PID_Angle.integral_limit = 100.0f;
    ERROR_MAX = 40.0f; KP_Fuzzy = 0.0f; KD_Fuzzy = 0.0f;

    // // 三次拟位式PID
    // PID_Cube.Kp_a = 80.00f ;  PID_Cube.Kp_b = 0.00f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 12.50f ; PID_Cube.Kd_b = 0 ;
    // PID_Cube.output_limit = 100.0f; PID_Cube.integral_limit = 100 ;
    // 三次拟位式PID
    PID_Cube.Kp_a = 0.085f ;  PID_Cube.Kp_b = 0.0003f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 1.20f ; PID_Cube.Kd_b = 0 ;
    PID_Cube.output_limit = 100.0f; PID_Cube.integral_limit = 100 ;
    reset_curve_slowdown_state(0.0f);
}

#define MOTOR_DEAD_ZONE 500 
static int32_t Add_Dead_Zone(int32_t pwm) {
    int32_t abs_pwm = abs(pwm);
    int32_t compensation = 0;
    if (abs_pwm < 200) return 0;
    if (abs_pwm <= 1000) {
        compensation = MOTOR_DEAD_ZONE;
    }
    // else if (abs_pwm < 1500) {
    //     float ratio = (1500.0f - abs_pwm) / 500.0f;
    //     compensation = (int32_t)(MOTOR_DEAD_ZONE * ratio);
    // }
    // else {
    //     compensation = 0;
    // }
    if (pwm > 0) return pwm + compensation;
    if (pwm < 0) return pwm - compensation;
    return pwm;
}


void BayWatcher_Middle_Loop(void* arg){

}
void BayWatcher_Outter_Loop(void* arg){
    if (PID.is_running == 0) {
        PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0;
        PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0;
        PID.pwm_out_L = 0; PID.pwm_out_R = 0;
        motor_sys.Stop();
        return;
    }
//     PID.vision_yaw = pure_angle;
//     // get_updataPD_pid(PID.current_angle, &PID_Angle.Kp, &PID_Angle.Kd);
//     PID.target_yaw_speed = Calc_Pos_PID(&PID_Angle, PID.target_angle, PID.vision_yaw);//右偏为-，左偏为+
//     if(PID.target_yaw_speed > 25.0f)  PID.target_yaw_speed = 25.0f;
//     if(PID.target_yaw_speed < -25.0f) PID.target_yaw_speed = -25.0f;
// //     PID.speed_adjust = Calc_Pos_PID(&PID_Angle, PID.target_angle, PID.vision_yaw);//右偏为-，左偏为+
// //     if(PID.speed_adjust > 25.0f) PID.speed_adjust = 25.0f;
// //     if(PID.speed_adjust < -25.0f) PID.speed_adjust = -25.0f;
            PID.vision_yaw = pure_angle;
            // printf("%.1f\n",PID.vision_yaw);
            PID.current_angle = PID.vision_yaw; 
            // PID.current_angle = 0;
            get_updataPD_pid(PID.current_angle, &PID_Angle.Kp, &PID_Angle.Kd);
            PID.speed_adjust = Calc_Pos_PID(&PID_Angle, PID.target_angle, PID.current_angle);//右偏为-，左偏为+
            // printf("%.1f\n",PID.target_yaw_speed);
            if(PID.speed_adjust > 35.0f) PID.speed_adjust = 35.0f;
            if(PID.speed_adjust < -35.0f) PID.speed_adjust = -35.0f;
}

void BayWatcher_Inner_Loop(void* arg){
    if (PID.is_running == 0) {
        PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0;
        PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0;
        PID.pwm_out_L = 0; PID.pwm_out_R = 0;
        motor_sys.Stop();
        return;
    }
    // imu_sys.update();
    PID.speed_adjust = Calc_Pos_PID(&PID_YawSpeed, PID.target_yaw_speed, imu_sys.raw_gz);
    if(PID.speed_adjust > 1000.0f) PID.speed_adjust = 1000.0f;
    if(PID.speed_adjust < -1000.0f) PID.speed_adjust = -1000.0f;

    PID.speed_adjust = 0;
    vL = encoder_sys.getSpeedL();
    vR = encoder_sys.getSpeedR();
    v_avg = (vL + vR)*0.5 ;
    
    if(vL>=-0.01 && vL<=0.01) vL =0.0f;
    if(vR>=-0.01 && vR<=0.01) vR =0.0f;

    const float effective_base_speed = update_curve_slowdown_base_speed(PID.base_target_speed);
    int32_t pid_out_L = (int32_t)Calc_Pos_PID(&PID_Speed_F_L, effective_base_speed, v_avg);
    int32_t pid_out_R = (int32_t)Calc_Pos_PID(&PID_Speed_F_R, effective_base_speed, v_avg);
    // if (PID.speed_adjust > 0) {
    //         PID.target_speed_L = PID.base_target_speed + (PID.speed_adjust * 0.5f);
    //         PID.target_speed_R = PID.base_target_speed - PID.speed_adjust;
    //     } else {
    //         PID.target_speed_L = PID.base_target_speed + PID.speed_adjust;
    //         PID.target_speed_R = PID.base_target_speed - (PID.speed_adjust * 0.5f);
    //     }

    // int32_t pid_out_L = (int32_t)Calc_Pos_PID(&PID_Speed_F_L, PID.target_speed_L, v_avg);
    // int32_t pid_out_R = (int32_t)Calc_Pos_PID(&PID_Speed_F_R, PID.target_speed_R, v_avg);

    PID.pwm_out_L = pid_out_L+PID.speed_adjust;  
    PID.pwm_out_R = pid_out_R-PID.speed_adjust;

    // PID.pwm_out_L = 1000;
    // PID.pwm_out_R = 1000;

    motor_sys.Motor2_Set(PID.pwm_out_L);
    motor_sys.Motor1_Set(PID.pwm_out_R);

}


void BayWatcher_Set_BaseSpeed(float speed)
{
    PID.base_target_speed = speed;
    if (!PID.is_running)
    {
        PID.yaw_control_base_speed = speed;
    }
}


void BayWatcher_Set_TargetAngle(float angle) {PID.target_angle      = angle; }

void BayWatcher_Control_Loop(void* arg) {

    // // //视觉请求停车（斑马线等）：直接停电机并复位控制器，避免转向环继续输出导致自旋。
    // // if (zebra_stop)
    // // {
    // //     PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0; PID_Speed_L.last_error = 0;
    // //     PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0; PID_Speed_R.last_error = 0;
    // //     PID_Angle.integral = 0; PID_Angle.last_error = 0;
    // //     PID_YawSpeed.integral = 0; PID_YawSpeed.last_error = 0;

    // //     Robot.target_yaw_speed = 0;
    // //     Robot.speed_adjust = 0;
    // //     Robot.target_speed_L = 0;
    // //     Robot.target_speed_R = 0;
    // //     Robot.pwm_out_L = 0;
    // //     Robot.pwm_out_R = 0;
    // //     Motors.Stop();
    // //     return;
    // // }

    if (PID.is_running == 0) {
        PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0;
        PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0;
        PID.pwm_out_L = 0; PID.pwm_out_R = 0;
        motor_sys.Stop();
        return;
    }

    // float safe_base_speed = speed_ramper.Calc_Ramped_Speed(PID.base_target_speed, global_ramp_step);

    // 读取速度 
    vL = encoder_sys.getSpeedL();
    vR = encoder_sys.getSpeedR();

    if(vL>=-0.01 && vL<=0.01) vL =0.0f;
    if(vR>=-0.01 && vR<=0.01) vR =0.0f;

    const float effective_base_speed = update_curve_slowdown_base_speed(PID.base_target_speed);

    // const float effective_base_speed = update_curve_slowdown_base_speed(safe_base_speed);
    // const float effective_speed_adjust = PID.speed_adjust * (std::fabs(effective_base_speed) / (std::fabs(PID.base_target_speed) + 1e-4f));
    // PID.target_speed_L = effective_base_speed + effective_speed_adjust;
    // PID.target_speed_R = effective_base_speed - effective_speed_adjust;
    PID.target_speed_L = effective_base_speed + (PID.speed_adjust);
    PID.target_speed_R = effective_base_speed - (PID.speed_adjust);
    // PID.target_speed_L = effective_base_speed + effective_speed_adjust;
    // PID.target_speed_R = effective_base_speed - effective_speed_adjust;

    // 计算 PID
    int32_t pid_out_L = (int32_t)Calc_Inc_PID(&PID_Speed_L, PID.target_speed_L, vL);
    int32_t pid_out_R = (int32_t)Calc_Inc_PID(&PID_Speed_R, PID.target_speed_R, vR);

    PID.pwm_out_L = pid_out_L;  
    PID.pwm_out_R = pid_out_R;

    // PID.pwm_out_L = 1000;
    // PID.pwm_out_R = 1000;

    // PID.pwm_out_L = 5000;
    // PID.pwm_out_R = 5000;

    // 输出
    motor_sys.Motor2_Set(PID.pwm_out_L);
    motor_sys.Motor1_Set(PID.pwm_out_R);
}

void BayWatcher_Cube_Loop(void* arg){
    if (PID.is_running == 0) {
        PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0;
        PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0;
        PID.pwm_out_L = 0; PID.pwm_out_R = 0;
        motor_sys.Stop();
        return;
    }
    PID.vision_yaw = pure_angle;
    // imu_sys.update();
    // imu_sys.raw_gz =PID_Cube.gyro ;
    PID_Cube.gyro = 0;
    PID.speed_adjust = Cube_Pos_PID(&PID_Cube, PID.target_angle, PID.vision_yaw);//右偏为-，左偏为+
    if(PID.speed_adjust > 35.0f)  PID.speed_adjust = 35.0f;
    if(PID.speed_adjust < -35.0f) PID.speed_adjust = -35.0f;
}

// ======================= 发车控制分区 ============================

// 全局发车配置
StartMode global_start_mode = StartMode::NO_ESC;
float global_ramp_step = 1.0f;

void BayWatcher_Start_Car(void) {
    vL = 0; // 重置滤波状态
    vR=  0;
    // 手动启动：解除斑马线锁停
    // zebra_stop = 0;
    PID.is_running = 1;
    esc_sys.is_running = 1;
    esc_sys.start();
    // Logger.Log_Event("START_LOGGING");
    PID_Speed_L.prev_error = 0; PID_Speed_L.last_error = 0; PID_Speed_L.output = 0;
    PID_Speed_R.prev_error = 0; PID_Speed_R.last_error = 0; PID_Speed_R.output = 0;
    PID_Angle.integral = 0; PID_Angle.last_error = 0;
    PID_Speed_F_L.integral = 0; PID_Speed_F_L.last_error = 0; PID_Speed_F_L.output = 0;
    PID_Speed_F_R.integral = 0; PID_Speed_F_R.last_error = 0; PID_Speed_F_R.output = 0;
    PID.base_target_speed = 0;
    reset_curve_slowdown_state(0.0f);
}

void BayWatcher_Start_Car(uint8_t mode, float target_speed, uint32_t esc_wait_ms, float ramp_step) {
    PID.base_target_speed = 0.0f; 
    global_ramp_step = ramp_step;
    speed_ramper.Reset();

    vL = 0; vR = 0;
    PID_Speed_L.prev_error = 0; PID_Speed_L.last_error = 0; PID_Speed_L.output = 0;
    PID_Speed_R.prev_error = 0; PID_Speed_R.last_error = 0; PID_Speed_R.output = 0;
    PID_Angle.integral = 0; PID_Angle.last_error = 0;
    PID_Speed_F_L.integral = 0; PID_Speed_F_L.last_error = 0; PID_Speed_F_L.output = 0;
    PID_Speed_F_R.integral = 0; PID_Speed_F_R.last_error = 0; PID_Speed_F_R.output = 0;
    reset_curve_slowdown_state(0.0f);

    PID.is_running = 1; 
    if (mode == 0) { // === 模式0：带负压延时发车 ===
        esc_sys.is_running = 1;
        esc_sys.start();
        
        // 直接休眠延时！不影响 5ms 的底层 PID 锁死车轮
        usleep(esc_wait_ms * 1000); 
    } 
    else {           // === 模式1：无负压直接发车 ===
        esc_sys.is_running = 0;
        esc_sys.stop();
    }
    PID.base_target_speed = target_speed;
}

void BayWatcher_Stop_Car(void) {
    PID.is_running = 0;
    esc_sys.is_running = 0;
    PID.base_target_speed = 0;
    reset_curve_slowdown_state(0.0f);
    motor_sys.Stop();
    esc_sys.stop();
}

void BayWatcher_Set_Start_Mode(uint8_t mode) {
    global_start_mode = (mode == 0) ? StartMode::WITH_ESC : StartMode::NO_ESC;
}

// void BayWatcher_Start_Car_Sequence(float target_speed, uint32_t esc_wait_ms, float ramp_step) {
//     PID.base_target_speed = 0.0f; 
//     global_ramp_step = ramp_step;
//     speed_ramper.Reset();    // 初始化速度

//     vL = 0; vR = 0;
//     PID_Speed_L.prev_error = 0; PID_Speed_L.last_error = 0; PID_Speed_L.output = 0;
//     PID_Speed_R.prev_error = 0; PID_Speed_R.last_error = 0; PID_Speed_R.output = 0;
//     reset_curve_slowdown_state(0.0f);

//     PID.is_running = 1; 
//     // 直接使用阻塞延时
//     if (global_start_mode == StartMode::WITH_ESC) {
//         esc_sys.is_running = 1;
//         esc_sys.start();
//         usleep(esc_wait_ms * 1000); 
//     } 
//     else {
//         esc_sys.is_running = 0;
//         esc_sys.stop();
//     }

//     PID.base_target_speed = target_speed;
// }
// ======================= 电调控制分区 ============================ 
void BayWatcher_Set_BaseEscPwm(float percentage){
    if(percentage>1.00) percentage = 1;
    if(percentage<0.00) percentage = 0;
    ESC.base_target_pwm = percentage ;
}
