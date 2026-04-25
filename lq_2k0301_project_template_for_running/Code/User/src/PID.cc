#include "main.hpp"
#include <cmath>

#pragma region Ackerman

const float PI_VAL = 3.14159265f;
const float ACKERMAN_CONST = 1450.0f;  // Ackerman越小,差速就越大
// const float ACKERMAN_CONST = 2050.0f;  // Ackerman越小,差速就越大
// const float STEER_LIMIT = 730.0f;      // 转向输出限幅
// const float STEER_LIMIT = 1030.0f;      // 转向输出限幅
const float STEER_LIMIT = 530.0f;      // 转向输出限幅
const float FACTOR_LIMIT = 1.23;        // 差速比例输出限幅

#pragma endregion

#pragma region 弯道减速
// ============================ 弯道减速配置 ============================
// 基于图像侧输出的 preview_curve_angle_deg，对基础巡线速度做连续减速。
// 设计目标：
// 1. 菜单/VOFA 设置的 base_target_speed 仍表示“手动设定速度”
// 2. 控制环内部再生成一个 yaw_control_base_speed 作为“实际生效速度”
// 3. 急弯时快速减速，出弯时更平缓地恢复，避免速度指令突然跳变
// 调参建议：
// - 觉得弯道减速介入太早：调大 CURV_LOW / CURV_HIGH
// - 觉得弯道减速不够狠：调小 MIN_SPEED_RATIO
// - 觉得出弯恢复太慢：调大 ALPHA_UP
// - 觉得进弯减速还不够快：调大 ALPHA_DOWN
// 总开关：1=启用 preview_curve_angle_deg 弯道减速，0=完全关闭，恢复为手动基准速度直通。
#ifndef BW_ENABLE_CURVE_SLOWDOWN
#define BW_ENABLE_CURVE_SLOWDOWN 1
#endif

// 环岛减速开关：
// 使用位置：PID.cc / curve_slowdown_roundabout_target_speed。
// 作用：控制“环岛阶段是否启用固定减速”。
// 1：进入环岛后不再使用 preview_curve_angle_deg，直接切到固定减速速度。
// 0：进入环岛后不减速，yaw_control_base_speed 立即回到手动基准速度。
// 推荐：如果环岛内曲率波动较大、连续减速会导致速度抖动，保持开启更稳。
#ifndef BW_ENABLE_ROUNDABOUT_CURVE_SLOWDOWN
#define BW_ENABLE_ROUNDABOUT_CURVE_SLOWDOWN 1
// #define BW_ENABLE_ROUNDABOUT_CURVE_SLOWDOWN 0
#endif

// 环岛固定减速比例：
// 使用位置：PID.cc / curve_slowdown_roundabout_target_speed。
// 作用：当环岛减速开关开启时，环岛内实际生效速度 = 手动基准速度 * 该比例。
// 调小：环岛更慢、更稳。
// 调大：环岛更快、更接近普通巡线速度。
// 默认值 0.92 约等于当前转角减速链在较小转角区间内的典型减速强度。
#ifndef BW_ROUNDABOUT_FIXED_SLOWDOWN_RATIO
#define BW_ROUNDABOUT_FIXED_SLOWDOWN_RATIO 0.92f
#endif

// 转角减速下限：preview_curve_angle_deg 低于该值时，不触发减速。
// 调大：更晚开始减速；调小：更早开始减速。
#ifndef BW_CURVE_SLOWDOWN_CURV_LOW
#define BW_CURVE_SLOWDOWN_CURV_LOW PUREANGLE_PREVIEW_CURVE_LOW
#endif

// 转角减速上限：preview_curve_angle_deg 高于该值时，认为进入“最大减速区”。
// 调大：只有更急的弯才会触发最大减速；调小：更容易打满减速。
#ifndef BW_CURVE_SLOWDOWN_CURV_HIGH
#define BW_CURVE_SLOWDOWN_CURV_HIGH PUREANGLE_PREVIEW_CURVE_HIGH
#endif

// 最小速度比例：急弯时实际生效速度不会低于“手动设定速度 * 该比例”。
// 例如 0.65 表示最大只减到 65%。
// 调小：急弯更慢、更稳；调大：减速更弱。
#ifndef BW_CURVE_SLOWDOWN_MIN_SPEED_RATIO
// #define BW_CURVE_SLOWDOWN_MIN_SPEED_RATIO 0.85f
#define BW_CURVE_SLOWDOWN_MIN_SPEED_RATIO 0.99f
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
#define BW_CURVE_SLOWDOWN_ALPHA_DOWN 0.16f
#endif

// 出弯恢复平滑系数：目标速度变大时使用。
// 调大：恢复速度更快；调小：恢复更慢，更不容易在弯后突然窜车。
#ifndef BW_CURVE_SLOWDOWN_ALPHA_UP
#define BW_CURVE_SLOWDOWN_ALPHA_UP 0.008f
#endif

#pragma endregion
#pragma region 减速函数
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
        t = (preview_curve_angle_deg - curve_low) / (curve_high - curve_low);
        t = clampf_pid(t, 0.0f, 1.0f);
    }
    else if (preview_curve_angle_deg >= curve_high)
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

static bool g_zebra_control_latched = false;

static void reset_pid_runtime_state_to_idle()
{
    PID.is_running = 0;
    PID.startup_state = 0;
    PID.startup_delay_cnt = 0;
    PID.startup_delay_target = 0;
    PID.startup_esc_target = 0.0f;
    PID.startup_esc_step = 0.0f;
    PID.startup_speed_target = 0.0f;
    PID.startup_speed_step = 0.0f;

    PID.base_target_speed = 0.0f;
    PID.target_angle = 0.0f;
    PID.vision_yaw = 0.0f;
    PID.current_angle = 0.0f;
    PID.current_yaw_speed = 0.0f;
    PID.target_yaw_speed = 0.0f;
    PID.speed_adjust = 0.0f;
    PID.target_speed_L = 0.0f;
    PID.target_speed_R = 0.0f;
    PID.pwm_out_L = 0;
    PID.pwm_out_R = 0;
    PID.yaw_control_base_speed = 0.0f;

    PID_Speed_L.error = 0.0f;
    PID_Speed_L.prev_error = 0.0f;
    PID_Speed_L.last_error = 0.0f;
    PID_Speed_L.error_i = 0.0f;
    PID_Speed_L.output = 0.0f;

    PID_Speed_R.error = 0.0f;
    PID_Speed_R.prev_error = 0.0f;
    PID_Speed_R.last_error = 0.0f;
    PID_Speed_R.error_i = 0.0f;
    PID_Speed_R.output = 0.0f;

    PID_Speed_F_L.error = 0.0f;
    PID_Speed_F_L.integral = 0.0f;
    PID_Speed_F_L.last_error = 0.0f;
    PID_Speed_F_L.output = 0.0f;

    PID_Speed_F_R.error = 0.0f;
    PID_Speed_F_R.integral = 0.0f;
    PID_Speed_F_R.last_error = 0.0f;
    PID_Speed_F_R.output = 0.0f;

    PID_Angle.error = 0.0f;
    PID_Angle.integral = 0.0f;
    PID_Angle.last_error = 0.0f;
    PID_Angle.output = 0.0f;

    PID_YawSpeed.error = 0.0f;
    PID_YawSpeed.integral = 0.0f;
    PID_YawSpeed.last_error = 0.0f;
    PID_YawSpeed.output = 0.0f;

    PID_Cube.error = 0.0f;
    PID_Cube.integral = 0.0f;
    PID_Cube.last_error = 0.0f;
    PID_Cube.gyro = 0.0f;
    PID_Cube.output = 0.0f;

    vL = 0.0f;
    vR = 0.0f;
    reset_curve_slowdown_state(0.0f);
}

static bool handle_zebra_stop_request()
{
    if (!zebra_stop)
    {
        g_zebra_control_latched = false;
        return false;
    }

    if (!g_zebra_control_latched ||
        PID.is_running != 0 ||
        PID.startup_state != 0 ||
        esc_sys.is_running)
    {
        BayWatcher_Stop_Car();
        g_zebra_control_latched = true;
        printf("[ZEBRA] control stop -> reset to idle\r\n");
    }

    return true;
}

} // namespace
#pragma endregion


#pragma region PID
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
                   (pid->Kp_b * pid->error * fabsf(pid->error)) +
                   (pid->Ki * pid->integral) + 
                   (pid->Kd_a * (pid->error - pid->last_error))+
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
#pragma endregion

#pragma region Init & Loop
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
    // PID_Speed_L.Kp = 42.32f; PID_Speed_L.Ki = 10.50f; PID_Speed_L.Kd = 0.00f;
    PID_Speed_L.Kp = 42.32f; PID_Speed_L.Ki = 12.50f; PID_Speed_L.Kd = 0.00f;
    PID_Speed_L.output = 0; PID_Speed_L.output_limit = 8500.0f;

    // 右轮速度环PID
    // PID_Speed_R.Kp = 42.00f; PID_Speed_R.Ki = 10.50f; PID_Speed_R.Kd = 0.00f;
    PID_Speed_R.Kp = 42.00f; PID_Speed_R.Ki = 12.50f; PID_Speed_R.Kd = 0.00f;
    PID_Speed_R.output = 0; PID_Speed_R.output_limit = 8500.0f;

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
    // PID_Cube.Kp_a = 13.50f ;  PID_Cube.Kp_b = 0.1430f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 450.00f ; PID_Cube.Kd_b = 0.00000f;
    // PID_Cube.Kp_a = 25.50f ;  PID_Cube.Kp_b = 0.800f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 2000.10f ; PID_Cube.Kd_b = 0.00000f;
    // PID_Cube.Kp_a = 20.50f ;  PID_Cube.Kp_b = 1.000f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 2000.10f ; PID_Cube.Kd_b = 0.00000f;
    // PID_Cube.Kp_a = 32.50f ;  PID_Cube.Kp_b = 0.800f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 2000.10f ; PID_Cube.Kd_b = 0.00000f;
    // PID_Cube.Kp_a = 10.50f ;  PID_Cube.Kp_b = 0.55f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 800.10f ; PID_Cube.Kd_b = 0.00000f;
    // PID_Cube.Kp_a = 25.50f ;  PID_Cube.Kp_b = 0.850f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 2000.10f ; PID_Cube.Kd_b = 0.00000f;
    // PID_Cube.Kp_a = 10.50f ;  PID_Cube.Kp_b = 0.55f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 2000.10f ; PID_Cube.Kd_b = 0.00000f;
    PID_Cube.Kp_a = 5.50f ;  PID_Cube.Kp_b = 0.45f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 800.10f ; PID_Cube.Kd_b = 0.00000f;
    PID_Cube.output_limit = STEER_LIMIT; PID_Cube.integral_limit = 100 ;
    reset_curve_slowdown_state(0.0f);

    // // 同步全局配置给电调系统
    // esc_sys.enable_esc_diff = cfg_esc_diff_enable;
    // esc_sys.esc_diff_limit = cfg_esc_diff_limit;
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
    if (handle_zebra_stop_request()) {
        return;
    }
}
void BayWatcher_Outter_Loop(void* arg){
    if (handle_zebra_stop_request()) {
        return;
    }
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
    if (handle_zebra_stop_request()) {
        return;
    }
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
#pragma endregion

#pragma region Config & Loop
// ======================= 全局发车配置 ============================
// bool cfg_esc_soft_start = true;    // true=开启电调(负压)发车延时与软启，false=跳过
bool cfg_esc_soft_start = false;    // true=开启电调(负压)发车延时与软启，false=跳过
// bool cfg_motor_soft_start = false;  // true=开启底盘电机软启，false=跳过直接输出给定速度
bool cfg_motor_soft_start = true;  // true=开启底盘电机软启，false=跳过直接输出给定速度
// bool cfg_esc_diff_enable = true;   // true=开启负压差速，false=关闭 (同步给菜单控制)
bool cfg_esc_diff_enable = false;   // true=开启负压差速，false=关闭 (同步给菜单控制)
float cfg_esc_diff_limit = 15.0f;  // 负压差速补偿最大限幅值 (百分比，支持在菜单中设置)

float global_ramp_step = 1.0f;           // (旧有代码预留) 全局基础提速步长
uint32_t g_startup_delay_ms = 5000;       // [发车阶段1]：按下发车后，死锁车轮并在原地等待的毫秒数 (用于手离开车体)
float g_startup_esc_target = 0.0f;      // [发车阶段2]：负压风扇/电调起转的最终目标百分比 (例如 25.0f 表示 25%)
float g_startup_esc_step = 0.0f;         // [发车阶段2]：风扇软起步长，每个控制周期增加的风扇推力 (数值越小风扇起转越平滑)
float g_startup_speed_step = 0.5f;       // [发车阶段3]：底盘电机软起步长，每个控制周期底盘提速的增量 (防烧胎打滑)

// ===============================================================

void BayWatcher_Control_Loop(void* arg) {
    if (handle_zebra_stop_request()) {
        return;
    }

    if (PID.is_running == 0) {
        PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0;
        PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0;
        PID.pwm_out_L = 0; PID.pwm_out_R = 0;
        motor_sys.Stop();
        return;
    }

    // 调用电调更新，确保 UI 数据下发及软启动状态接力
    esc_sys.Update_Tick();

    // ================== 发车状态机 ==================
    // 本状态机将接管电调与底盘的双重软启动，保证高速车辆发车平稳不打滑、不烧胎。
    if (PID.startup_state > 0 && PID.startup_state < 4) {
        static uint32_t startup_log_cnt = 0;
        startup_log_cnt++;

        if (PID.startup_state == 1) {
            // --- 阶段 1：初始延时阶段 ---
            PID.base_target_speed = 0.0f; // 强制锁死车轮
            PID.startup_delay_cnt++;
            if (startup_log_cnt % 50 == 0) { // 每 250ms 打印一次
                printf("[Startup] 阶段1: 启动前等待中... %d / %d ms\n", PID.startup_delay_cnt * 5, PID.startup_delay_target * 5);
            }
            if (PID.startup_delay_cnt >= PID.startup_delay_target) {
                PID.startup_state = 2; // 时间到，切入下个阶段
                printf("[Startup] 阶段1完成！进入阶段2: 等待电调软起动...\n");
                esc_sys.Start_Soft(PID.startup_esc_target, PID.startup_esc_step); // 唤醒电调软启动
            }
        } 
        else if (PID.startup_state == 2) {
            // --- 阶段 2：等待负压软起动完成 ---
            PID.base_target_speed = 0.0f; // 负压没满，车轮依然死锁！
            if (!esc_sys.is_soft_starting) {
                PID.startup_state = 3; // 负压满电，切入下个阶段
                printf("[Startup] 阶段2完成！电调已满载。进入阶段3: 底盘电机软起动...\n");
            }
        }
        else if (PID.startup_state == 3) {
            // --- 阶段 3：底盘车速软起动 ---
            if (!cfg_motor_soft_start) {
                // 不使用速度软启，一瞬间给满
                PID.base_target_speed = PID.startup_speed_target;
                PID.startup_state = 4;
                printf("[Startup] 跳过电机软起动，直接满速进入巡航！\n");
            } else {
                PID.base_target_speed += PID.startup_speed_step;
                if (startup_log_cnt % 20 == 0) {
                    printf("[Startup] 阶段3: 电机软起动爬升中... 目标车速: %.1f / %.1f\n", PID.base_target_speed, PID.startup_speed_target);
                }
                if (PID.base_target_speed >= PID.startup_speed_target) {
                    PID.base_target_speed = PID.startup_speed_target;
                    PID.startup_state = 4; // 发车大满贯，切入终极巡航状态
                    printf("[Startup] 阶段3完成！发车大满贯，切入终极巡航状态！\n");
                }
            }
        }
        
        // 处于等待和电调软起期间时，不让底盘有任何动作，直接返回
        if (PID.startup_state >= 1 && PID.startup_state <= 2) {
            PID_Speed_L.output = 0; PID_Speed_R.output = 0;
            PID.pwm_out_L = 0; PID.pwm_out_R = 0;
            motor_sys.Stop();
            return;
        }
    }
    // ==============================================================

    // 读取两侧编码器速度 (如果太小当做 0 处理)
    vL = encoder_sys.getSpeedL();
    vR = encoder_sys.getSpeedR();

    if(vL>=-0.01 && vL<=0.01) vL =0.0f;
    if(vR>=-0.01 && vR<=0.01) vR =0.0f;

    // 考虑到前方可能是弯道，动态应用弯道减速
    // const float effective_base_speed = update_curve_slowdown_base_speed(PID.base_target_speed);
    const float effective_base_speed = PID.base_target_speed;

    // if (PID.speed_adjust > 0) {
    //     PID.target_speed_L = effective_base_speed + (PID.speed_adjust * 0.3f);
    //     PID.target_speed_R = effective_base_speed - PID.speed_adjust;
    // } else {
    //     PID.target_speed_L = effective_base_speed + PID.speed_adjust;
    //     PID.target_speed_R = effective_base_speed - (PID.speed_adjust * 0.3f);
    // }

    // PID.target_speed_L = effective_base_speed + PID.speed_adjust;
    // PID.target_speed_R = effective_base_speed - PID.speed_adjust;

    float factor = tanf(PID.speed_adjust * PI_VAL / ACKERMAN_CONST) * 0.55f;
    if( factor>= FACTOR_LIMIT) factor = FACTOR_LIMIT;
    if( factor<= -FACTOR_LIMIT) factor = -FACTOR_LIMIT;
    if (PID.speed_adjust >= 0) {
        PID.target_speed_L = effective_base_speed * (1.0f + 0.5f * factor);
        PID.target_speed_R = effective_base_speed * (1.0f - 1.0f * factor);
    } else {
        PID.target_speed_L = effective_base_speed * (1.0f + 1.0f * factor);
        PID.target_speed_R = effective_base_speed * (1.0f - 0.5f * factor);
    }

    // if (PID.speed_adjust >= 0) {
    //     PID.target_speed_L = effective_base_speed * (1.0f + 1.0f * factor);
    //     PID.target_speed_R = effective_base_speed * (1.0f - 0.5f * factor);
    // } else {
    //     PID.target_speed_L = effective_base_speed * (1.0f + 0.5f * factor);
    //     PID.target_speed_R = effective_base_speed * (1.0f - 1.0f * factor);
    // }


    // printf("%.2f\n",factor);
    // if (PID.speed_adjust >= 0) {
    //     PID.target_speed_L = effective_base_speed * (1.0f + 0.3f * factor);
    //     PID.target_speed_R = effective_base_speed * (1.0f - 1.0f * factor);
    // } else {
    //     PID.target_speed_L = effective_base_speed * (1.0f + 1.0f * factor);
    //     PID.target_speed_R = effective_base_speed * (1.0f - 0.3f * factor);
    // }
    // printf("%.2f\n",factor);
    // printf("%.2f\n",factor);
    // printf("%.2f\n",factor);

    // 计算底层速度环增量式 PID
    int32_t pid_out_L = (int32_t)Calc_Inc_PID(&PID_Speed_L, PID.target_speed_L, vL);
    int32_t pid_out_R = (int32_t)Calc_Inc_PID(&PID_Speed_R, PID.target_speed_R, vR);

    // 保存全局打印/日志状态用
    PID.pwm_out_L = pid_out_L;  
    PID.pwm_out_R = pid_out_R;

    // PID.pwm_out_L = 1000;  
    // PID.pwm_out_R = 1000;

    motor_sys.Motor2_Set(PID.pwm_out_L);
    motor_sys.Motor1_Set(PID.pwm_out_R);
    // motor_sys.Motor2_Set(PID.pwm_out_R);
    // motor_sys.Motor1_Set(PID.pwm_out_L);
}

void BayWatcher_Cube_Loop(void* arg){
    if (handle_zebra_stop_request()) {
        return;
    }
    if (PID.is_running == 0) {
        PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0;
        PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0;
        PID.pwm_out_L = 0; PID.pwm_out_R = 0;
        motor_sys.Stop();
        return;
    }
    PID.vision_yaw = pure_angle;
    imu_sys.update();
    // imu_sys.raw_gz =PID_Cube.gyro ;
    PID_Cube.gyro = imu_sys.raw_gz ;
    // PID_Cube.gyro = 0;


    float scale_x = 1.5f; 
    float exp_neg_angle = expf(-fabsf(PID.vision_yaw / scale_x));
    float adjust = fabsf((exp_neg_angle - 1.0f) / (exp_neg_angle + 1.0f)) * 0.5f + 0.5f;

    // PID.vision_yaw = 0;
    // PID.speed_adjust = Cube_Pos_PID(&PID_Cube, PID.target_angle, PID.vision_yaw);//右偏为-，左偏为+
    float raw_turn = Cube_Pos_PID(&PID_Cube, PID.target_angle, PID.vision_yaw);//右偏为-，左偏为+
    PID.speed_adjust = raw_turn * adjust ;
    if(PID.speed_adjust > STEER_LIMIT)  PID.speed_adjust = STEER_LIMIT;
    if(PID.speed_adjust < -STEER_LIMIT) PID.speed_adjust = -STEER_LIMIT;
    // printf("%.2f\n",PID.speed_adjust);
    // PID.speed_adjust = 0;
}
#pragma endregion

#pragma region Start & Stop
// ======================= 发车控制分区 ============================

void BayWatcher_Start_Car(void) {
     // const float target_spd = (std::fabs(PID.base_target_speed) > 1e-4f) ? PID.base_target_speed : 10.0f;
    zebra_stop = false;
    g_zebra_control_latched = false;
    reset_pid_runtime_state_to_idle();
    vL = 0; // 重置滤波状态
    vR=  0;
    // 手动启动：解除斑马线锁停
    // zebra_stop = 0;
    PID.is_running = 1;
    esc_sys.is_running = false;
    // esc_sys.start();
    // Logger.Log_Event("START_LOGGING");
    PID_Speed_L.prev_error = 0; PID_Speed_L.last_error = 0; PID_Speed_L.output = 0;
    PID_Speed_R.prev_error = 0; PID_Speed_R.last_error = 0; PID_Speed_R.output = 0;
    PID_Angle.integral = 0; PID_Angle.last_error = 0;
    PID_Speed_F_L.integral = 0; PID_Speed_F_L.last_error = 0; PID_Speed_F_L.output = 0;
    PID_Speed_F_R.integral = 0; PID_Speed_F_R.last_error = 0; PID_Speed_F_R.output = 0;
    PID.base_target_speed = 0;
    reset_curve_slowdown_state(0.0f);
    // const float target_spd = (std::fabs(PID.base_target_speed) > 1e-4f) ? PID.base_target_speed : 10.0f;
    // BayWatcher_Sequence_Start(g_startup_delay_ms, g_startup_esc_target, g_startup_esc_step, target_spd, g_startup_speed_step);
}

void BayWatcher_Start_Car(uint8_t mode, float target_speed, uint32_t esc_wait_ms, float ramp_step) {
    zebra_stop = false;
    g_zebra_control_latched = false;
    reset_pid_runtime_state_to_idle();
    global_ramp_step = ramp_step;
    speed_ramper.Reset();

    PID.is_running = 1; 
    if (mode == 0) { // === 模式0：带负压延时发车 ===
        esc_sys.is_running = true;
        esc_sys.start();
        usleep(esc_wait_ms * 1000); 
    } 
    else {           // === 模式1：无负压直接发车 ===
        esc_sys.is_running = false;
        esc_sys.stop();
    }
    PID.base_target_speed = target_speed;
}

void BayWatcher_Stop_Car(void) {
    motor_sys.Stop();
    esc_sys.stop();
    esc_sys.is_running = false;
    reset_pid_runtime_state_to_idle();
}

void BayWatcher_Set_Start_Mode(uint8_t mode) {
    if (mode == 0) {
        cfg_esc_soft_start = true;
        cfg_motor_soft_start = true;
    } else {
        cfg_esc_soft_start = false;
        cfg_motor_soft_start = false;
    }
}
#pragma endregion

#pragma region Auto Start
// ======================= 电调控制分区 ============================ 
void BayWatcher_Sequence_Start(uint32_t delay_ms, float esc_target, float esc_step, float speed_target, float speed_step) {
    // 1. 记录发车参数
    PID.startup_delay_target = delay_ms / 5; // 假设控制环是 5ms (例如1000ms延时 = 200次)
    PID.startup_delay_cnt = 0;
    
    PID.startup_esc_target = esc_target;
    PID.startup_esc_step = esc_step;
    PID.startup_speed_target = speed_target;
    PID.startup_speed_step = speed_step;
    
    // 2. 状态机切入配置
    PID.is_running = 1;
    esc_sys.is_running = true; 
    
    if (!cfg_esc_soft_start) {
        // 关闭电调软启时，瞬间赋予满偏参数，并跳过延时和等待阶段
        esc_sys.Start_Soft(esc_target, 100.0f); 
        esc_sys.is_soft_starting = false;
        PID.base_target_speed = (!cfg_motor_soft_start) ? speed_target : 0.0f;
        PID.startup_state = cfg_motor_soft_start ? 3 : 4; 
    } else {
        // 从零开始走延时软启
        PID.base_target_speed = 0.0f; 
        PID.startup_state = 1; 
    }
    
    // 3. 清理历史误差
    vL = 0; vR = 0;
    PID_Speed_L.prev_error = 0; PID_Speed_L.last_error = 0; PID_Speed_L.output = 0;
    PID_Speed_R.prev_error = 0; PID_Speed_R.last_error = 0; PID_Speed_R.output = 0;
    PID_Angle.integral = 0; PID_Angle.last_error = 0;
    PID_Speed_F_L.integral = 0; PID_Speed_F_L.last_error = 0; PID_Speed_F_L.output = 0;
    PID_Speed_F_R.integral = 0; PID_Speed_F_R.last_error = 0; PID_Speed_F_R.output = 0;
    reset_curve_slowdown_state(0.0f);
    
    printf("[System] 发车序列启动 -> 阶段1：原地死锁等待 %d ms...\n", delay_ms);
}

void BayWatcher_Set_BaseEscPwm(float percentage){
    // 统一转换为 0~100 的量纲，并应用限幅
    percentage = clampf_pid(percentage, 0.0f, 100.0f);
    // 直接下发给新的电调系统（大步长瞬间达到目标，跳过软起）
    esc_sys.Start_Soft(percentage, 100.0f);
}

// 留给外部调用的负压差速接口
void BayWatcher_Set_EscDiffConfig(float enable, float ratio){
    esc_sys.enable_esc_diff = clampf_pid(enable, 0.0f, 1.0f);
    esc_sys.esc_diff_ratio = clampf_pid(ratio, 0.0f, 10.0f);
}
#pragma endregion
