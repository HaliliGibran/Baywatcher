#include "main.hpp"
#include <cmath>
#include <cstdio>


float big_langd_add = 0.0f;

static float Calc_Inc_PID(Bay_IncPID_t *pid, float target, float measured);


#pragma region Ackerman

const float PI_VAL = 3.14159265f;
const float ACKERMAN_CONST = 1450.0f;  // Ackerman越小,差速就越大
// const float ACKERMAN_CONST = 2050.0f;  // Ackerman越小,差速就越大
// const float STEER_LIMIT = 730.0f;      // 转向输出限幅
// const float STEER_LIMIT = 1030.0f;      // 转向输出限幅
const float STEER_LIMIT = 530.0f;      // 转向输出限幅
const float FACTOR_LIMIT = 1.23f;        // 差速比例输出限幅
// const float FACTOR_LIMIT = 1.25f;        // 差速比例输出限幅
// const float FACTOR_LIMIT = 1.2f;        // 差速比例输出限幅
const float REMOTE_RECOG_SPEED_RATIO_NO_BLOCK_EPS = 1e-4f;

static inline bool remote_bypass_active()
{
    FollowLine forced_mode = FollowLine::MIXED;
    if (!image_remote_recognition_get_forced_follow_mode(&forced_mode))
    {
        return false;
    }

    return forced_mode == FollowLine::MIDLEFT ||
           forced_mode == FollowLine::MIDRIGHT;
}

static inline bool remote_follow_no_reverse_limit_active()
{
    return (BW_REMOTE_FOLLOW_NO_REVERSE_ENABLE != 0) && remote_bypass_active();
}

static inline bool remote_recognition_straight_accel_block_active()
{
    if (remote_bypass_active())
    {
        return true;
    }

    // 识别板输出 u/绕行动作时会给速度比例 < 1；这里只用它判断“识别减速介入”，不依赖具体减速幅度。
    return image_remote_recognition_get_speed_ratio_override() <
           (1.0f - REMOTE_RECOG_SPEED_RATIO_NO_BLOCK_EPS);
}

static inline float apply_remote_speed_cap(float speed)
{
    float speed_cap = 0.0f;
    if (image_remote_recognition_get_speed_cap_override(&speed_cap) && speed > speed_cap)
    {
        return speed_cap;
    }
    return speed;
}

static inline void apply_remote_target_speed_cap(float* left_speed, float* right_speed)
{
    float speed_cap = 0.0f;
    if (!image_remote_recognition_get_speed_cap_override(&speed_cap))
    {
        return;
    }

    if (left_speed != nullptr && *left_speed > speed_cap)
    {
        *left_speed = speed_cap;
    }
    if (right_speed != nullptr && *right_speed > speed_cap)
    {
        *right_speed = speed_cap;
    }
}

static inline float clamp_remote_follow_factor_no_reverse(float factor,
                                                          float forward_base_speed)
{
    if (!remote_follow_no_reverse_limit_active())
    {
        return factor;
    }

    if (forward_base_speed <= 0.0f)
    {
        return 0.0f;
    }

    const float no_reverse_limit = 1.0f - BW_REMOTE_FOLLOW_NO_REVERSE_FACTOR_MARGIN;
    const float factor_limit = (FACTOR_LIMIT < no_reverse_limit) ? FACTOR_LIMIT : no_reverse_limit;
    if (factor > factor_limit)
    {
        return factor_limit;
    }
    if (factor < -factor_limit)
    {
        return -factor_limit;
    }
    return factor;
}

#pragma endregion

#pragma region 长直道加速

// bool cfg_straight_accel_enable = true;      // 是否开启直道加速
bool cfg_straight_accel_enable = false;      // 是否开启直道加速

// float cfg_straight_accel_max_add = 8.0f;    // 作用上限：直道加速最大补偿速度
float cfg_straight_accel_max_add = 6.0f;
// float cfg_straight_accel_max_add = 5.6f;
// float cfg_straight_accel_max_add = 4.0f;    
// float cfg_straight_accel_max_add = 2.0f;    

//  最小阈值：在此范围内视为绝对直道，补偿拉满 (100% max_add)
float cfg_straight_accel_curve_min_th = 5.4f; // 适应取最大曲率算法，垫高底线容忍直道毛刺
float cfg_straight_accel_yaw_min_th = 3.6f;    // 直道 yaw 控制在 -5 到 5，满分阈值设为 3.5

// 3. 最大阈值：超过此值视为入弯，一票否决，加速清零
float cfg_straight_accel_curve_max_th = 7.5f; // 相应拉高最高阈值
float cfg_straight_accel_yaw_max_th = 6.8f;    // 超过 7 度绝对不是直道

// 4. 剧烈程度 (保持 0.4 激进模式)
float cfg_straight_accel_intensity = 0.4f;  

#define CURVE_HISTORY_SIZE 10

static float current_straight_add = 0.0f;             
static int continuous_straight_frames = 0;            
static float last_pure_angle = 0.0f;                  
static float curve_history[CURVE_HISTORY_SIZE] = {0}; 
static int curve_history_index = 0;                   

static float curve_median_history[5] = {0.0f};
static int curve_median_index = 0;
static float curve_smoothed = 0.0f;                   

static void reset_straight_acceleration_state() {
    current_straight_add = 0.0f;
    continuous_straight_frames = 0;
    last_pure_angle = 0.0f;
    for (int i = 0; i < CURVE_HISTORY_SIZE; i++) curve_history[i] = 0.0f;
    curve_history_index = 0;
    for (int i = 0; i < 5; i++) curve_median_history[i] = 0.0f;
    curve_median_index = 0;
    curve_smoothed = 0.0f;
}

static float update_straight_acceleration(float pure_angle, float preview_curve) {
    // ================== 核心元素拦截区 ==================
    // 元素互斥逻辑：只要不在普通赛道 (NORMAL)，立刻清零直道状态机并断油
    // 完美实现环岛和十字路口内部不进行任何直道加速
    if (!cfg_straight_accel_enable ||
        element_type != ElementType::NORMAL ||
        std::fabs(PID.base_target_speed) <= 1e-4f ||
        remote_recognition_straight_accel_block_active()) {
        reset_straight_acceleration_state();
        return 0.0f;
    }
    // ====================================================

    float abs_yaw = std::fabs(pure_angle);
    float abs_curve = std::fabs(preview_curve);
    
    // 获取物理角速度 (原始数据量级较大)
    float abs_gz = std::fabs(imu_sys.raw_gz);
    
    float delta_yaw = std::fabs(pure_angle - last_pure_angle);
    last_pure_angle = pure_angle;

    // 5帧时间中值滤波器 (消除孤立噪声尖峰)
    curve_median_history[curve_median_index] = abs_curve;
    curve_median_index = (curve_median_index + 1) % 5;
    float sorted_curves[5];
    for (int i = 0; i < 5; i++) sorted_curves[i] = curve_median_history[i];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4 - i; j++) {
            if (sorted_curves[j] > sorted_curves[j + 1]) {
                float temp = sorted_curves[j];
                sorted_curves[j] = sorted_curves[j + 1];
                sorted_curves[j + 1] = temp;
            }
        }
    }
    float curve_filtered = sorted_curves[2]; 

    // 非对称一阶低通滤波
    float beta = (curve_filtered > curve_smoothed) ? 0.35f : 0.08f;
    curve_smoothed += beta * (curve_filtered - curve_smoothed);

    // 计算 Curve 积分视野
    curve_history[curve_history_index] = curve_filtered;
    curve_history_index = (curve_history_index + 1) % CURVE_HISTORY_SIZE;
    float curve_sum = 0.0f;
    for (int i = 0; i < CURVE_HISTORY_SIZE; i++) curve_sum += curve_history[i];
    float curve_avg = curve_sum / CURVE_HISTORY_SIZE;

    float target_add = 0.0f;

    float max_delta_yaw_th = 3.0f; // 放宽一点横向跳变容忍度
    float max_curve_avg_th = cfg_straight_accel_curve_max_th * 0.8f; 

    // ================== 核心拦截区 ==================
    if (abs_yaw >= cfg_straight_accel_yaw_max_th || 
        curve_smoothed >= cfg_straight_accel_curve_max_th ||
        delta_yaw >= max_delta_yaw_th ||
        curve_avg >= max_curve_avg_th ||
        abs_gz >= 3000.0f) // S弯天敌：如果物理角速度 > 3000，一票否决！
    {
        continuous_straight_frames = 0; 
        target_add = 0.0f;
    } else {
        continuous_straight_frames++;
        
        // 延长考核期：连续 10 帧 (50ms) 满足上述所有条件，才认为是真直道
        if (continuous_straight_frames > 10) { 
            float yaw_score = 1.0f;
            if (abs_yaw > cfg_straight_accel_yaw_min_th) {
                float yaw_diff = cfg_straight_accel_yaw_max_th - cfg_straight_accel_yaw_min_th;
                yaw_score = 1.0f - (abs_yaw - cfg_straight_accel_yaw_min_th) / yaw_diff;
            }

            float curve_score = 1.0f;
            if (curve_smoothed > cfg_straight_accel_curve_min_th) {
                float curve_diff = cfg_straight_accel_curve_max_th - cfg_straight_accel_curve_min_th;
                curve_score = 1.0f - (curve_smoothed - cfg_straight_accel_curve_min_th) / curve_diff;
            }
            
            float final_score = (yaw_score < curve_score) ? yaw_score : curve_score;
            if (final_score < 0.0f) final_score = 0.0f;
            if (final_score > 1.0f) final_score = 1.0f;

            float power_score = std::pow(final_score, cfg_straight_accel_intensity);
            target_add = cfg_straight_accel_max_add * power_score;
        } else {
            target_add = 0.0f;
        }
    }

    // 最终低通输出
    float alpha_up = 0.08f;   
    float alpha_down = 0.40f; 
    float alpha = (target_add > current_straight_add) ? alpha_up : alpha_down;
    
    current_straight_add += alpha * (target_add - current_straight_add);
    if (current_straight_add < 0.01f) current_straight_add = 0.0f;

    return current_straight_add;
}

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

#pragma region Zebra Reset

static bool g_zebra_control_latched = false;

static constexpr uint32_t CLOSED_LOOP_STOP_PERIOD_MS = 5;
static constexpr uint32_t CLOSED_LOOP_STOP_TIMEOUT_MS = 700;
static constexpr uint32_t CLOSED_LOOP_STOP_TIMEOUT_TICKS =
    (CLOSED_LOOP_STOP_TIMEOUT_MS + CLOSED_LOOP_STOP_PERIOD_MS - 1) / CLOSED_LOOP_STOP_PERIOD_MS;
static constexpr uint32_t CLOSED_LOOP_STOP_STABLE_TICKS = 8;
static constexpr float CLOSED_LOOP_STOP_SPEED_EPS = 0.05f;

struct ClosedLoopStopState {
    bool active = false;
    uint32_t ticks = 0;
    uint32_t stable_ticks = 0;
};

static ClosedLoopStopState g_closed_loop_stop;

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
    reset_straight_acceleration_state();
}

static bool closed_loop_stop_active()
{
    return g_closed_loop_stop.active;
}

static void reset_direction_pid_for_stop()
{
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
}

static void begin_closed_loop_stop()
{
    esc_sys.stop();
    esc_sys.is_running = false;

    g_closed_loop_stop.active = true;
    g_closed_loop_stop.ticks = 0;
    g_closed_loop_stop.stable_ticks = 0;

    PID.is_running = 1;
    PID.startup_state = 4;
    PID.startup_delay_cnt = 0;
    PID.startup_delay_target = 0;
    PID.startup_esc_target = 0.0f;
    PID.startup_esc_step = 0.0f;
    PID.startup_speed_target = 0.0f;
    PID.startup_speed_step = 0.0f;

    PID.base_target_speed = 0.0f;
    PID.target_angle = 0.0f;
    PID.target_yaw_speed = 0.0f;
    PID.speed_adjust = 0.0f;
    PID.target_speed_L = 0.0f;
    PID.target_speed_R = 0.0f;
    PID.yaw_control_base_speed = 0.0f;
    big_langd_add = 0.0f;

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

    reset_direction_pid_for_stop();
    reset_curve_slowdown_state(0.0f);
    reset_straight_acceleration_state();
}

static void finish_closed_loop_stop()
{
    g_closed_loop_stop.active = false;
    g_closed_loop_stop.ticks = 0;
    g_closed_loop_stop.stable_ticks = 0;

    motor_sys.Stop();
    reset_pid_runtime_state_to_idle();
}

static bool process_closed_loop_stop_if_active()
{
    if (!g_closed_loop_stop.active)
    {
        return false;
    }

    vL = encoder_sys.getSpeedL();
    vR = encoder_sys.getSpeedR();

    if (std::fabs(vL) <= CLOSED_LOOP_STOP_SPEED_EPS &&
        std::fabs(vR) <= CLOSED_LOOP_STOP_SPEED_EPS)
    {
        g_closed_loop_stop.stable_ticks++;
    }
    else
    {
        g_closed_loop_stop.stable_ticks = 0;
    }

    g_closed_loop_stop.ticks++;

    if (g_closed_loop_stop.stable_ticks >= CLOSED_LOOP_STOP_STABLE_TICKS ||
        g_closed_loop_stop.ticks >= CLOSED_LOOP_STOP_TIMEOUT_TICKS)
    {
        finish_closed_loop_stop();
        return true;
    }

    PID.is_running = 1;
    PID.startup_state = 4;
    PID.base_target_speed = 0.0f;
    PID.target_angle = 0.0f;
    PID.target_yaw_speed = 0.0f;
    PID.speed_adjust = 0.0f;
    PID.target_speed_L = 0.0f;
    PID.target_speed_R = 0.0f;
    PID.yaw_control_base_speed = 0.0f;
    big_langd_add = 0.0f;
    esc_sys.current_factor = 0.0f;

    const int32_t pid_out_L = (int32_t)Calc_Inc_PID(&PID_Speed_L, 0.0f, vL);
    const int32_t pid_out_R = (int32_t)Calc_Inc_PID(&PID_Speed_R, 0.0f, vR);

    PID.pwm_out_L = pid_out_L;
    PID.pwm_out_R = pid_out_R;

    motor_sys.Motor2_Set(PID.pwm_out_L);
    motor_sys.Motor1_Set(PID.pwm_out_R);
    return true;
}

static bool handle_zebra_stop_request()
{
    if (!zebra_stop)
    {
        g_zebra_control_latched = false;
        return false;
    }

    if (g_closed_loop_stop.active)
    {
        return false;
    }

    if (!g_zebra_control_latched ||
        PID.is_running != 0 ||
        PID.startup_state != 0 ||
        esc_sys.is_running)
    {
        BayWatcher_Stop_Car();
        g_zebra_control_latched = true;
        if (g_closed_loop_stop.active)
        {
            printf("[ZEBRA] control stop -> closed-loop stop\r\n");
        }
        else
        {
            printf("[ZEBRA] control stop -> reset to idle\r\n");
        }
    }

    return !g_closed_loop_stop.active;
}

} // namespace
#pragma endregion


#pragma region Turnover PT

// 翻车测试开关
// #define DEBUG_IMU_TEST 1
#define DEBUG_IMU_TEST 0

static bool handle_rollover_protection()
{   // ================== 边界配置 ==================
    const float ROLLOVER_THRESHOLD = 3800.0f; // 翻车硬阈值：低于 3000 判定为异常
    const int CONFIRM_CHECKS = 8;             // 连续 8 次低频检查均异常才触发紧急抱死

    // ================== 蒸馏器 ==================
    // 降频计数器：让 5ms 周期调用的函数，每 10 次才真正往下走一次
    // 实际判断周期变为：5ms * 10 = 50ms (20Hz)，对龙芯 CPU 而言算力开销直接稀释到千万分之一级
    static int loop_divider = 0;
    if (++loop_divider % 10 != 0) {
        return false; 
    }

    // 状态状态计数器（只有在 50ms 的执行周期内才会自增/清零）
    static int abnormal_low_freq_cnt = 0; 

    // 如果车子本身就处于停止状态，清空计数，不参与判断
    if (PID.is_running == 0) {
        abnormal_low_freq_cnt = 0; 
        return false;
    }

    // ================== 仅针对 Z 轴重力的非线性判据 ==================
    // 你的日志显示，瞬态颠簸曾跌到过 2634，采用连续低频确认可以完美过滤这种瞬态毛刺
    if (imu_sys.raw_az < ROLLOVER_THRESHOLD) {
        abnormal_low_freq_cnt++;
        
        // 50ms 检查一次，连续 3 次通过检查 -> 说明车子在 150ms 内一直四轮朝天
        if (abnormal_low_freq_cnt >= CONFIRM_CHECKS) {
            
            // 瞬间切断底盘电机和电调负压，复位所有 PID 积分
            BayWatcher_Stop_Car();
            
            printf("\n=================================================\n");
            printf("[ALARM] Turnover Protection!\n");
            printf("[ALARM] current g: %d \n", (int)imu_sys.raw_az);
            printf("[ALARM] Turnover Protection!\n");
            printf("=================================================\n\n");
            
            abnormal_low_freq_cnt = 0; 
            return true;
        }
    } else {
        // 只要有一帧数据回升到 3000 以上（比如上坡、颠簸复位），计数器立刻清零
        abnormal_low_freq_cnt = 0; 
    }

    return false;
}// namespace

#pragma endregion

#pragma region VOFA Ctrl


// bool cfg_vofa_remote_enable = true;
bool cfg_vofa_remote_enable = false;
float vofa_target_speed_adjust = 0.0f;


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

#pragma region Circle PIDs
// ============================ 环岛分段 PID 参数 ============================
struct Cube_PID_Param_t {
    float Kp_a;
    float Kp_b;
    float Ki;
    float Kd_a;
    float Kd_b;
    float output_limit;
    float integral_limit;
};

bool cfg_circle_pid_enable = true; // true=启用环岛入环/出环特调PID，false=完全使用普通巡线PID
// bool cfg_circle_pid_enable = false; // true=启用环岛入环/出环特调PID，false=完全使用普通巡线PID

static Cube_PID_Param_t cube_pid_normal_param;

// 是否已经把“普通巡线 PID 参数”保存下来。
// false：说明 BayWatcher_Control_Init() 还没保存过，不能恢复普通参数。
// true ：说明 cube_pid_normal_param 里已经有初始化后的普通巡线参数。
static bool if_cube_pid_normal_saved = false;

enum class CubePIDSpecialOwner {
    NONE,
    CIRCLE,
    CROSSING,
};

// 当前 PID_Cube 特调参数的归属。
// NONE：当前就是普通巡线参数。
// CIRCLE/CROSSING：当前由对应元素接管，只有 owner 自己负责退出时恢复普通参数。
static CubePIDSpecialOwner cube_pid_special_owner = CubePIDSpecialOwner::NONE;

static CircleState last_circle_pid_state = CircleState::CIRCLE_NONE;
static CircleDirection last_circle_pid_direction = CircleDirection::CIRCLE_DIR_NONE;

// 20.04+6
// 左环岛特调参数：只区分入环和出环。
static const Cube_PID_Param_t left_circle_in_pid  = {6.845f, 0.5398f, 0.0f, 310.0f, 0.00100f, STEER_LIMIT, 100.0f};
static const Cube_PID_Param_t left_circle_out_pid = {6.845f, 0.5398f, 0.0f, 310.0f, 0.00100f, STEER_LIMIT, 100.0f};

// 右环岛特调参数：初值与左环岛相同，后续按实车表现分开修。
static const Cube_PID_Param_t right_circle_in_pid  = {6.925f, 0.5398f, 0.0f, 310.0f, 0.00100f, STEER_LIMIT, 100.0f};
static const Cube_PID_Param_t right_circle_out_pid = {6.925f, 0.5398f, 0.0f, 310.0f, 0.00100f, STEER_LIMIT, 100.0f};

static const Cube_PID_Param_t& CubePID_Get_Base_Param();

static void CubePID_Save_Normal_Param()
{
    cube_pid_normal_param.Kp_a = PID_Cube.Kp_a;
    cube_pid_normal_param.Kp_b = PID_Cube.Kp_b;
    cube_pid_normal_param.Ki = PID_Cube.Ki;
    cube_pid_normal_param.Kd_a = PID_Cube.Kd_a;
    cube_pid_normal_param.Kd_b = PID_Cube.Kd_b;
    cube_pid_normal_param.output_limit = PID_Cube.output_limit;
    cube_pid_normal_param.integral_limit = PID_Cube.integral_limit;

    if_cube_pid_normal_saved = true;
    cube_pid_special_owner = CubePIDSpecialOwner::NONE;
}

static void CubePID_Clear_State()
{
    PID_Cube.error = 0.0f;
    PID_Cube.integral = 0.0f;
    PID_Cube.last_error = 0.0f;
    PID_Cube.output = 0.0f;
}

static void CubePID_Apply_Param(const Cube_PID_Param_t& param)
{
    PID_Cube.Kp_a = param.Kp_a;
    PID_Cube.Kp_b = param.Kp_b;
    PID_Cube.Ki = param.Ki;
    PID_Cube.Kd_a = param.Kd_a;
    PID_Cube.Kd_b = param.Kd_b;
    PID_Cube.output_limit = param.output_limit;
    PID_Cube.integral_limit = param.integral_limit;

    CubePID_Clear_State();
}

static void circle_pid_update_last_state()
{
    last_circle_pid_state = circle_state;
    last_circle_pid_direction = circle_direction;
}

static const Cube_PID_Param_t& circle_pid_get_left_param(CircleState state)
{
    switch (state) {
        case CircleState::CIRCLE_IN:      return left_circle_in_pid;
        case CircleState::CIRCLE_OUT:     return left_circle_out_pid;
        default:                          return CubePID_Get_Base_Param();
    }
}

static const Cube_PID_Param_t& circle_pid_get_right_param(CircleState state)
{
    switch (state) {
        case CircleState::CIRCLE_IN:      return right_circle_in_pid;
        case CircleState::CIRCLE_OUT:     return right_circle_out_pid;
        default:                          return CubePID_Get_Base_Param();
    }
}

static void circle_pid_update_by_state()
{
    if (!if_cube_pid_normal_saved) {
        CubePID_Save_Normal_Param();
    }

    // 环岛 PID 总开关。
    // false：不使用任何环岛特调参数，效果等同于旧代码一直使用初始化的 PID_Cube。
    // 如果关闭开关时正处在特调参数里，先恢复普通巡线参数，避免把 IN/OUT 参数留在车上。
    if (!cfg_circle_pid_enable) {
        if (cube_pid_special_owner == CubePIDSpecialOwner::CIRCLE) {
            CubePID_Apply_Param(CubePID_Get_Base_Param());
            cube_pid_special_owner = CubePIDSpecialOwner::NONE;
        }

        circle_pid_update_last_state();
        return;
    }

    // 当前环岛状态/方向有没有变。
    // false：状态没变，PID 参数也不需要动，直接返回。
    // true ：可能进入 IN/OUT，也可能离开 IN/OUT，需要往下判断是否切参数。
    const bool if_circle_pid_state_changed =
        circle_state != last_circle_pid_state ||
        circle_direction != last_circle_pid_direction;

    if (!if_circle_pid_state_changed) {
        return;
    }

    const Cube_PID_Param_t* target_param = &CubePID_Get_Base_Param();

    // 当前这一帧是否应该使用环岛特调参数。
    // false：当前不在左/右入环、左/右出环，目标参数就是普通巡线参数。
    // true ：当前正处于左/右入环或左/右出环，需要切到对应的特调参数。
    bool if_use_circle_special_pid = false;

    if ((element_type == ElementType::CIRCLE ||
         circle_state != CircleState::CIRCLE_NONE) &&
        circle_direction == CircleDirection::CIRCLE_DIR_LEFT) {
        if (circle_state == CircleState::CIRCLE_IN || circle_state == CircleState::CIRCLE_OUT) {
            target_param = &circle_pid_get_left_param(circle_state);
            if_use_circle_special_pid = true;
        }
    } else if ((element_type == ElementType::CIRCLE ||
                circle_state != CircleState::CIRCLE_NONE) &&
               circle_direction == CircleDirection::CIRCLE_DIR_RIGHT) {
        if (circle_state == CircleState::CIRCLE_IN || circle_state == CircleState::CIRCLE_OUT) {
            target_param = &circle_pid_get_right_param(circle_state);
            if_use_circle_special_pid = true;
        }
    }

    // 只有两种情况才真正写 PID：
    // 1. if_use_circle_special_pid=true：进入 IN/OUT 特调段，要写入特调参数。
    // 2. cube_pid_special_owner==CIRCLE：上一段还在用环岛特调参数，现在离开了，要恢复普通巡线参数。
    // 如果一直在 BEGIN/RUNNING/END 这些普通段，这里不会反复写普通参数，也不会反复清 PID 历史。
    if (if_use_circle_special_pid || cube_pid_special_owner == CubePIDSpecialOwner::CIRCLE) {
        CubePID_Apply_Param(*target_param);
        cube_pid_special_owner = if_use_circle_special_pid ? CubePIDSpecialOwner::CIRCLE
                                                           : CubePIDSpecialOwner::NONE;
    }

    circle_pid_update_last_state();
}

#pragma endregion

#pragma region Cross PIDs
// ============================ 十字分段 PID 参数 ============================

// 十字间特调
bool cfg_crossing_between_enable = true; // true=启用完整十字流程 between 档位切换，false=强制使用初始化参数
// bool cfg_crossing_between_enable = false; // true=启用完整十字流程 between 档位切换，false=强制使用初始化参数

// 十字特调
bool cfg_crossing_pid_enable = true; // true=启用十字 IN/RUNNING 特调PID，false=完全使用普通巡线PID
// bool cfg_crossing_pid_enable = false; // true=启用十字 IN/RUNNING 特调PID，false=完全使用普通巡线PID

bool Crossing_Between = false; // 两个完整十字流程之间的基础 PID 档位：false=初始化参数，true=between参数
uint32_t cfg_crossing_between_timeout_ticks = 250; // 连续多少个 Cube_Loop 周期碰不到十字后复位，0=不超时复位

enum class CrossingBetweenFlowStage {
    WAIT_IN,
    WAIT_RUNNING,
    WAIT_EXIT,
};

static CrossingBetweenFlowStage crossing_between_flow_stage = CrossingBetweenFlowStage::WAIT_IN;
static uint32_t crossing_between_no_crossing_ticks = 0;
static CrossingState last_crossing_pid_state = CrossingState::CROSSING_NONE;

// 20.04+6
static const Cube_PID_Param_t crossing_between_pid = {6.745f, 0.5298f, 0.0f, 310.10f, 0.00100f, STEER_LIMIT, 100.0f};

static const Cube_PID_Param_t crossing_in_pid      = {6.845f, 0.5298f, 0.0f, 300.10f, 0.00100f, STEER_LIMIT, 100.0f};
static const Cube_PID_Param_t crossing_running_pid = {6.845f, 0.5298f, 0.0f, 300.00f, 0.00100f, STEER_LIMIT, 100.0f};

static const Cube_PID_Param_t& CubePID_Get_Base_Param()
{
    return Crossing_Between ? crossing_between_pid : cube_pid_normal_param;
}

static void crossing_between_apply_base_if_idle()
{
    if (cube_pid_special_owner == CubePIDSpecialOwner::NONE)
    {
        CubePID_Apply_Param(CubePID_Get_Base_Param());
    }
}

static void crossing_between_reset_to_zero(bool log_exit)
{
    if (!Crossing_Between)
    {
        return;
    }

    Crossing_Between = false;
    crossing_between_apply_base_if_idle();
    if (log_exit)
    {
        std::printf("[CROSSING]Exit_Between\r\n");
    }
}

static void crossing_between_toggle()
{
    Crossing_Between = !Crossing_Between;
    crossing_between_no_crossing_ticks = 0;
    crossing_between_apply_base_if_idle();
    if (Crossing_Between)
    {
        std::printf("[CROSSING]Just_Between\r\n");
    }
}

static void crossing_between_update_by_state()
{
    if (!if_cube_pid_normal_saved) {
        CubePID_Save_Normal_Param();
    }

    if (!cfg_crossing_between_enable)
    {
        crossing_between_flow_stage = CrossingBetweenFlowStage::WAIT_IN;
        crossing_between_no_crossing_ticks = 0;
        crossing_between_reset_to_zero(true);
        return;
    }

    switch (crossing_between_flow_stage)
    {
        case CrossingBetweenFlowStage::WAIT_IN:
            if (crossing_state == CrossingState::CROSSING_IN)
            {
                crossing_between_flow_stage = CrossingBetweenFlowStage::WAIT_RUNNING;
            }
            break;

        case CrossingBetweenFlowStage::WAIT_RUNNING:
            if (crossing_state == CrossingState::CROSSING_RUNNING)
            {
                crossing_between_flow_stage = CrossingBetweenFlowStage::WAIT_EXIT;
            }
            else if (crossing_state == CrossingState::CROSSING_NONE)
            {
                crossing_between_flow_stage = CrossingBetweenFlowStage::WAIT_IN;
            }
            break;

        case CrossingBetweenFlowStage::WAIT_EXIT:
            if (crossing_state == CrossingState::CROSSING_NONE)
            {
                crossing_between_toggle();
                crossing_between_flow_stage = CrossingBetweenFlowStage::WAIT_IN;
            }
            break;
    }

    if (crossing_state != CrossingState::CROSSING_NONE)
    {
        crossing_between_no_crossing_ticks = 0;
        return;
    }

    if (cfg_crossing_between_timeout_ticks == 0)
    {
        return;
    }

    if (crossing_between_no_crossing_ticks < cfg_crossing_between_timeout_ticks)
    {
        ++crossing_between_no_crossing_ticks;
    }

    if (crossing_between_no_crossing_ticks >= cfg_crossing_between_timeout_ticks)
    {
        crossing_between_flow_stage = CrossingBetweenFlowStage::WAIT_IN;
        crossing_between_reset_to_zero(true);
    }
}

static void crossing_pid_update_last_state()
{
    last_crossing_pid_state = crossing_state;
}

static const Cube_PID_Param_t& crossing_pid_get_param(CrossingState state)
{
    switch (state) {
        case CrossingState::CROSSING_IN:      return crossing_in_pid;
        case CrossingState::CROSSING_RUNNING: return crossing_running_pid;
        default:                              return CubePID_Get_Base_Param();
    }
}

static void crossing_pid_update_by_state()
{
    if (!if_cube_pid_normal_saved) {
        CubePID_Save_Normal_Param();
    }

    // 十字 PID 总开关。
    // 关闭时只恢复“由十字接管”的参数，不碰环岛正在使用的参数。
    if (!cfg_crossing_pid_enable) {
        if (cube_pid_special_owner == CubePIDSpecialOwner::CROSSING) {
            CubePID_Apply_Param(CubePID_Get_Base_Param());
            cube_pid_special_owner = CubePIDSpecialOwner::NONE;
        }

        crossing_pid_update_last_state();
        return;
    }

    const bool if_crossing_pid_state_changed =
        crossing_state != last_crossing_pid_state;

    if (!if_crossing_pid_state_changed) {
        return;
    }

    const bool if_use_crossing_special_pid =
        (element_type == ElementType::CROSSING ||
         crossing_state != CrossingState::CROSSING_NONE) &&
        (crossing_state == CrossingState::CROSSING_IN ||
         crossing_state == CrossingState::CROSSING_RUNNING);

    const Cube_PID_Param_t& target_param =
        if_use_crossing_special_pid ? crossing_pid_get_param(crossing_state)
                                    : CubePID_Get_Base_Param();

    // 进入 IN/RUNNING 时写十字参数；离开十字时恢复普通巡线参数。
    if (if_use_crossing_special_pid || cube_pid_special_owner == CubePIDSpecialOwner::CROSSING) {
        CubePID_Apply_Param(target_param);
        cube_pid_special_owner = if_use_crossing_special_pid ? CubePIDSpecialOwner::CROSSING
                                                             : CubePIDSpecialOwner::NONE;
    }

    crossing_pid_update_last_state();
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

    // // 无负压1
    // // 左轮速度环PID
    // // PID_Speed_L.Kp = 42.32f; PID_Speed_L.Ki = 12.50f; PID_Speed_L.Kd = 0.00f;
    // PID_Speed_L.Kp = 42.32f; PID_Speed_L.Ki = 12.50f; PID_Speed_L.Kd = 0.00f;
    // // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 8500.0f;
    // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 9999.0f;


    // // 右轮速度环PID
    // // PID_Speed_R.Kp = 42.00f; PID_Speed_R.Ki = 12.50f; PID_Speed_R.Kd = 0.00f;
    // PID_Speed_R.Kp = 42.00f; PID_Speed_R.Ki = 12.50f; PID_Speed_R.Kd = 0.00f;
    // // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 8500.0f;
    // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 9999.0f;


    // // 有负压 19-21.20
    // // 左轮速度环PID
    // // PID_Speed_L.Kp = 42.32f; PID_Speed_L.Ki = 12.50f; PID_Speed_L.Kd = 0.00f;
    // PID_Speed_L.Kp = 111.32f; PID_Speed_L.Ki = 25.60f; PID_Speed_L.Kd = 0.00f;
    // // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 8500.0f;
    // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 9999.0f;


    // // 右轮速度环PID
    // // PID_Speed_R.Kp = 42.00f; PID_Speed_R.Ki = 12.50f; PID_Speed_R.Kd = 0.00f;
    // PID_Speed_R.Kp = 111.00f; PID_Speed_R.Ki = 25.50f; PID_Speed_R.Kd = 0.00f;
    // // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 8500.0f;
    // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 9999.0f;

    // // 有负压 19-21.20
    // // 左轮速度环PID
    // // PID_Speed_L.Kp = 42.32f; PID_Speed_L.Ki = 12.50f; PID_Speed_L.Kd = 0.00f;
    // PID_Speed_L.Kp = 113.32f; PID_Speed_L.Ki = 26.00f; PID_Speed_L.Kd = 0.00f;
    // // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 8500.0f;
    // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 9999.0f;


    // // 右轮速度环PID
    // // PID_Speed_R.Kp = 42.00f; PID_Speed_R.Ki = 12.50f; PID_Speed_R.Kd = 0.00f;
    // PID_Speed_R.Kp = 113.00f; PID_Speed_R.Ki = 26.00f; PID_Speed_R.Kd = 0.00f;
    // // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 8500.0f;
    // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 9999.0f;

    // 有负压 19-21.20 燕大
    // 左轮速度环PID
    // PID_Speed_L.Kp = 42.32f; PID_Speed_L.Ki = 12.50f; PID_Speed_L.Kd = 0.00f;
    PID_Speed_L.Kp = 122.32f; PID_Speed_L.Ki = 27.50f; PID_Speed_L.Kd = 0.00f;
    // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 8500.0f;
    PID_Speed_L.output = 0; PID_Speed_L.output_limit = 9999.0f;


    // 右轮速度环PID
    // PID_Speed_R.Kp = 42.00f; PID_Speed_R.Ki = 12.50f; PID_Speed_R.Kd = 0.00f;
    PID_Speed_R.Kp = 122.00f; PID_Speed_R.Ki = 27.55f; PID_Speed_R.Kd = 0.00f;
    // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 8500.0f;
    PID_Speed_R.output = 0; PID_Speed_R.output_limit = 9999.0f;

    // // 有负压 19-21.20 燕大
    // // 左轮速度环PID
    // // PID_Speed_L.Kp = 42.32f; PID_Speed_L.Ki = 12.50f; PID_Speed_L.Kd = 0.00f;
    // PID_Speed_L.Kp = 132.32f; PID_Speed_L.Ki = 29.50f; PID_Speed_L.Kd = 0.00f;
    // // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 8500.0f;
    // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 9999.0f;


    // // 右轮速度环PID
    // // PID_Speed_R.Kp = 42.00f; PID_Speed_R.Ki = 12.50f; PID_Speed_R.Kd = 0.00f;
    // PID_Speed_R.Kp = 132.00f; PID_Speed_R.Ki = 29.55f; PID_Speed_R.Kd = 0.00f;
    // // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 8500.0f;
    // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 9999.0f;

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
    // // 无负压 17.66 0.4
    // PID_Cube.Kp_a = 6.588f ;  PID_Cube.Kp_b = 0.493f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 833.10f ; PID_Cube.Kd_b = 0.00000f;

    // // 有负压 15.90 25%
    // PID_Cube.Kp_a = 5.75f ;  PID_Cube.Kp_b = 0.465f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 813.10f ; PID_Cube.Kd_b = 0.00010f;

    // 有负压 16.50 25%
    // PID_Cube.Kp_a = 5.77f ;  PID_Cube.Kp_b = 0.4655f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 815.10f ; PID_Cube.Kd_b = 0.00010f;

    // // 有负压 17.52 0.4 30-40%
    // PID_Cube.Kp_a = 6.55f ;  PID_Cube.Kp_b = 0.488f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 836.10f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 18 0.4 60%
    // PID_Cube.Kp_a = 6.55f ;  PID_Cube.Kp_b = 0.485f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 850.10f ; PID_Cube.Kd_b = 0.00200f;

    // // 有负压 19 0.4 60%
    // PID_Cube.Kp_a = 6.545f ;  PID_Cube.Kp_b = 0.485f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 620.10f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 19.05 0.4 60%
    // PID_Cube.Kp_a = 6.545f ;  PID_Cube.Kp_b = 0.485f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 618.10f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 20.60 0.4 60%
    // PID_Cube.Kp_a = 6.556f ;  PID_Cube.Kp_b = 0.4845f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 595.10f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 21.00 0.4 60%
    // PID_Cube.Kp_a = 6.56f ;  PID_Cube.Kp_b = 0.4845f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 400.10f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 21.20 0.4 60%
    // PID_Cube.Kp_a = 6.565f ;  PID_Cube.Kp_b = 0.4844f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 452.10f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 21.25 0.35 60%
    // PID_Cube.Kp_a = 6.565f ;  PID_Cube.Kp_b = 0.4846f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 410.10f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 21.25 0.35 60%
    // PID_Cube.Kp_a = 6.5685f ;  PID_Cube.Kp_b = 0.4846f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 410.10f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 19.92 - 20.10 0.40 70%
    // PID_Cube.Kp_a = 6.565f ;  PID_Cube.Kp_b = 0.4844f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 302.10f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 20.42 - 20.80 0.40 70%
    // PID_Cube.Kp_a = 6.565f ;  PID_Cube.Kp_b = 0.4846f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 302.10f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 21.00 0.40 70% abandaned
    // PID_Cube.Kp_a = 6.605f ;  PID_Cube.Kp_b = 0.4848f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 302.50f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 20.42 - 20.80 0.40 70%
    // PID_Cube.Kp_a = 6.565f ;  PID_Cube.Kp_b = 0.4846f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 302.10f ; PID_Cube.Kd_b = 0.00100f;

    // // 有负压 20.04 0.40 70% 燕大
    // PID_Cube.Kp_a = 6.57f ;  PID_Cube.Kp_b = 0.4848f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 302.10f ; PID_Cube.Kd_b = 0.00100f;

    // 有负压 20.04 0.40 70% 7.5
    PID_Cube.Kp_a = 6.745f ;  PID_Cube.Kp_b = 0.5298f ;  PID_Cube.Ki = 0 ; PID_Cube.Kd_a = 310.10f ; PID_Cube.Kd_b = 0.00100f;

    PID_Cube.output_limit = STEER_LIMIT; PID_Cube.integral_limit = 100 ;
    CubePID_Save_Normal_Param();
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
    if (handle_zebra_stop_request() || closed_loop_stop_active()) {
        return;
    }
}
void BayWatcher_Outter_Loop(void* arg){
    if (handle_zebra_stop_request() || closed_loop_stop_active()) {
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
            if (cfg_vofa_remote_enable) {
                PID.speed_adjust = 0.0f;
            } else {
                PID.speed_adjust = Calc_Pos_PID(&PID_Angle, PID.target_angle, PID.current_angle);//右偏为-，左偏为+
                // printf("%.1f\n",PID.target_yaw_speed);
                if(PID.speed_adjust > 35.0f) PID.speed_adjust = 35.0f;
                if(PID.speed_adjust < -35.0f) PID.speed_adjust = -35.0f;
            }
}

void BayWatcher_Inner_Loop(void* arg){
    if (handle_zebra_stop_request()) {
        return;
    }
    if (process_closed_loop_stop_if_active()) {
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

    const float remote_speed_scale =
        clampf_pid(image_remote_recognition_get_speed_ratio_override(), 0.0f, 1.0f);
    const float effective_base_speed = apply_remote_speed_cap(
        update_curve_slowdown_base_speed(PID.base_target_speed) * remote_speed_scale);
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
bool cfg_start_delay = true;        // true=开启发车死锁延时，false=跳过
// bool cfg_start_delay = false;        // true=开启发车死锁延时，false=跳过
bool cfg_esc_soft_start = true;    // true=开启电调(负压)发车延时与软启，false=跳过
// bool cfg_esc_soft_start = false;    // true=开启电调(负压)发车延时与软启，false=跳过
bool cfg_motor_soft_start = false;  // true=开启底盘电机软启，false=跳过直接输出给定速度
// bool cfg_motor_soft_start = true;  // true=开启底盘电机软启，false=跳过直接输出给定速度
bool cfg_closed_loop_stop_enable = false; // true=速度闭环降到0后再断PWM，false=原急停
// bool cfg_closed_loop_stop_enable = true; // true=速度闭环降到0后再断PWM，false=原急停
// bool cfg_esc_diff_enable = true;   // true=开启负压差速，false=关闭 (同步给菜单控制)
bool cfg_esc_diff_enable = false;   // true=开启负压差速，false=关闭 (同步给菜单控制)
float cfg_esc_diff_limit = 0.0f;  // 负压差速补偿最大限幅值 (百分比，支持在菜单中设置)

float global_ramp_step = 1.0f;           // (旧有代码预留) 全局基础提速步长
uint32_t g_startup_delay_ms = 0.0f;       // [发车阶段1]：按下发车后，死锁车轮并在原地等待的毫秒数 (用于手离开车体)
float g_startup_esc_target = 70.0f;      // [发车阶段2]：负压风扇/电调起转的最终目标百分比 (例如 25.0f 表示 25%)
float g_startup_esc_step = 0.075f;         // [发车阶段2]：风扇软起步长，每个控制周期增加的风扇推力 (数值越小风扇起转越平滑)
float g_startup_speed_step = 0.05f;       // [发车阶段3]：底盘电机软起步长，每个控制周期底盘提速的增量 (防烧胎打滑)

// ===============================================================

void BayWatcher_Control_Loop(void* arg) {
    if (handle_zebra_stop_request()) {
        return;
    }
    if (process_closed_loop_stop_if_active()) {
        return;
    }
    //底盘速度环翻车拦截
    if (handle_rollover_protection()) {
        return;
    }

    if (PID.is_running == 0) {
        PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0;
        PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0;
        PID.pwm_out_L = 0; PID.pwm_out_R = 0;
        motor_sys.Stop();
        return;
    }

    // 调用电调更新，软起动
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
                if (cfg_esc_soft_start) {
                    PID.startup_state = 2; // 时间到，切入下个阶段
                    printf("[Startup] 阶段1完成！进入阶段2: 等待电调软起动...\n");
                    esc_sys.Start_Soft(PID.startup_esc_target, PID.startup_esc_step); // 唤醒电调软启动
                } else {
                    esc_sys.Start_Soft(PID.startup_esc_target, 0.02f); 
                    esc_sys.is_soft_starting = false;
                    
                    if (cfg_motor_soft_start) {
                        PID.startup_state = 3;
                        printf("[Startup] 阶段1完成！跳过阶段2，直接进入阶段3：底盘软起动...\n");
                    } else {
                        PID.base_target_speed = PID.startup_speed_target;
                        PID.startup_state = 4;
                        printf("[Startup] 阶段1完成！跳过后续软启，直接满速起步！\n");
                    }
                }
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
    const float remote_speed_scale = zebra_rush_active
        ? 1.0f
        : clampf_pid(image_remote_recognition_get_speed_ratio_override(), 0.0f, 1.0f);
    const float zebra_speed_scale =
        clampf_pid(zebra_speed_ratio_override, 0.0f, BW_ZEBRA_RUSH_SPEED_RATIO);
    const float effective_base_speed =
        PID.base_target_speed * remote_speed_scale * zebra_speed_scale;

    // if (PID.speed_adjust > 0) {
    //     PID.target_speed_L = effective_base_speed + (PID.speed_adjust * 0.3f);
    //     PID.target_speed_R = effective_base_speed - PID.speed_adjust;
    // } else {
    //     PID.target_speed_L = effective_base_speed + PID.speed_adjust;
    //     PID.target_speed_R = effective_base_speed - (PID.speed_adjust * 0.3f);
    // }

    // PID.target_speed_L = effective_base_speed + PID.speed_adjust;
    // PID.target_speed_R = effective_base_speed - PID.speed_adjust;

    // --- 状态机解耦保护 ---
    // 发车软起动期间（阶段1~3），强制车辆走直线纯净爬坡，屏蔽转向偏差和直道加速补偿的影响，防止速度突变破坏软启动
    float safe_speed_adjust = PID.speed_adjust;
    if (PID.startup_state >= 1 && PID.startup_state <= 3) {
        safe_speed_adjust = 0.0f;
    }

    // 直道加速加成
    if (cfg_straight_accel_enable == false || (PID.startup_state >= 1 && PID.startup_state <= 3)) {
        big_langd_add = 0.0f;
    } else {
        big_langd_add = update_straight_acceleration(pure_angle, preview_curve_angle_deg);
    }

    const float capped_forward_speed =
        apply_remote_speed_cap(effective_base_speed + big_langd_add);

    float factor = tanf(safe_speed_adjust * PI_VAL / ACKERMAN_CONST) * 0.55f;

    if (cfg_vofa_remote_enable) {
        factor = vofa_target_speed_adjust;
    }

    if( factor>= FACTOR_LIMIT) factor = FACTOR_LIMIT;
    if( factor<= -FACTOR_LIMIT) factor = -FACTOR_LIMIT;

    factor = clamp_remote_follow_factor_no_reverse(
        factor,
        capped_forward_speed);

    esc_sys.current_factor = factor; // 给电调传递当前的 factor，用于动态负压

    // 0.5
    // if (PID.speed_adjust >= 0) {
    //     PID.target_speed_L = (effective_base_speed + big_langd_add) * (1.0f + 0.5f * factor);
    //     PID.target_speed_R = (effective_base_speed + big_langd_add) * (1.0f - 1.0f * factor);
    // } else {
    //     PID.target_speed_L = (effective_base_speed + big_langd_add) * (1.0f + 1.0f * factor);
    //     PID.target_speed_R = (effective_base_speed + big_langd_add) * (1.0f - 0.5f * factor);
    // }

    //0.4
    if (factor >= 0) {
        PID.target_speed_L = capped_forward_speed * (1.0f + 0.4f * factor);
        PID.target_speed_R = capped_forward_speed * (1.0f - 1.0f * factor);
    } else {
        PID.target_speed_L = capped_forward_speed * (1.0f + 1.0f * factor);
        PID.target_speed_R = capped_forward_speed * (1.0f - 0.4f * factor);
    }
    apply_remote_target_speed_cap(&PID.target_speed_L, &PID.target_speed_R);

    // //0.35
    // if (PID.speed_adjust >= 0) {
    //     PID.target_speed_L = (effective_base_speed + big_langd_add) * (1.0f + 0.35f * factor);
    //     PID.target_speed_R = (effective_base_speed + big_langd_add) * (1.0f - 1.0f * factor);
    // } else {
    //     PID.target_speed_L = (effective_base_speed + big_langd_add) * (1.0f + 1.0f * factor);
    //     PID.target_speed_R = (effective_base_speed + big_langd_add) * (1.0f - 0.35f * factor);
    // }

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

    // PID.pwm_out_L = 3000;  
    // PID.pwm_out_R = 3000;

    motor_sys.Motor2_Set(PID.pwm_out_L);
    motor_sys.Motor1_Set(PID.pwm_out_R);
    // motor_sys.Motor2_Set(PID.pwm_out_R);
    // motor_sys.Motor1_Set(PID.pwm_out_L);
}

void BayWatcher_Cube_Loop(void* arg){
    if (handle_zebra_stop_request() || closed_loop_stop_active()) {
        return;
    }
    //底盘方向环翻车拦截
    if (handle_rollover_protection()) {
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
    crossing_between_update_by_state();
    circle_pid_update_by_state();
    crossing_pid_update_by_state();

#if DEBUG_IMU_TEST
    // ==================== IMU Z轴数据测试 ====================
    static int imu_test_cnt = 0;
    if (++imu_test_cnt % 20 == 0) {
        // raw_az: Z轴重力加速度 (用于测算翻车保护的阈值)
        // raw_gz: Z轴角速度 (用于测算 S弯/直角弯的陀螺仪阻尼)
        printf("[IMU 原始] Z轴重力: %8d | Z轴角速度: %8d\n\n", 
               (int)imu_sys.raw_az, (int)imu_sys.raw_gz);
    }
    // ================================================================
#endif


    float scale_x = 1.5f; 
    float exp_neg_angle = expf(-fabsf(PID.vision_yaw / scale_x));
    float adjust = fabsf((exp_neg_angle - 1.0f) / (exp_neg_angle + 1.0f)) * 0.5f + 0.5f;

    // PID.vision_yaw = 0;//速度环调试
    // PID.speed_adjust = Cube_Pos_PID(&PID_Cube, PID.target_angle, PID.vision_yaw);//右偏为-，左偏为+
    if (cfg_vofa_remote_enable) {
        PID.speed_adjust = 0.0f;
    } else {
        float raw_turn = Cube_Pos_PID(&PID_Cube, PID.target_angle, PID.vision_yaw);//右偏为-，左偏为+
        PID.speed_adjust = raw_turn * adjust ;
        if(PID.speed_adjust > STEER_LIMIT)  PID.speed_adjust = STEER_LIMIT;
        if(PID.speed_adjust < -STEER_LIMIT) PID.speed_adjust = -STEER_LIMIT;
    }
    // printf("%.2f\n",PID.speed_adjust);
    // PID.speed_adjust = 0;
}
#pragma endregion

#pragma region Start & Stop
// ======================= 发车控制分区 ============================

void BayWatcher_Start_Car(void) {
    if (PID.is_running) return;
    // 暂存设定好的目标速度，避免被 reset_pid_runtime_state_to_idle 清空
    float target_spd = PID.base_target_speed;
    if (std::fabs(target_spd) < 1e-4f) {
        target_spd = 0.0f; // 如果未设定速度，默认给 0.0f
    }

    zebra_stop = false;
    g_zebra_control_latched = false;
    
    // 彻底重置底层状态，防止上一次发车的积分和误差残留
    reset_pid_runtime_state_to_idle();
    
    // 正式启动多级软起动状态机
    BayWatcher_Sequence_Start(g_startup_delay_ms, g_startup_esc_target, g_startup_esc_step, target_spd, g_startup_speed_step);
}



void BayWatcher_Stop_Car(void) {
    if (cfg_closed_loop_stop_enable && PID.is_running != 0) {
        if (!closed_loop_stop_active()) {
            begin_closed_loop_stop();
        }
        return;
    }

    g_closed_loop_stop.active = false;
    g_closed_loop_stop.ticks = 0;
    g_closed_loop_stop.stable_ticks = 0;
    motor_sys.Stop();
    esc_sys.stop();
    esc_sys.is_running = false;
    reset_pid_runtime_state_to_idle();
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
    
    // if (!cfg_esc_soft_start) {
    //     // 关闭电调软启时，瞬间赋予满偏参数，并跳过延时和等待阶段
    //     esc_sys.Start_Soft(esc_target, 100.0f); 
    //     esc_sys.is_soft_starting = false;
    //     PID.base_target_speed = (!cfg_motor_soft_start) ? speed_target : 0.0f;
    //     PID.startup_state = cfg_motor_soft_start ? 3 : 4; 
    // } else {
    //     // 从零开始走延时软启
    //     PID.base_target_speed = 0.0f; 
    //     PID.startup_state = 1; 
    // }

    if (cfg_start_delay) {
        PID.base_target_speed = 0.0f; 
        PID.startup_state = 1; 
        printf("[System] 发车序列启动 -> 阶段1：原地死锁等待 %d ms...\n", delay_ms);
    } else if (cfg_esc_soft_start) {
        PID.base_target_speed = 0.0f; 
        PID.startup_state = 2;
        esc_sys.Start_Soft(esc_target, esc_step);
        printf("[System] 发车序列启动 -> 跳过阶段1，直接进入阶段2：电调软启中...\n");
    } else {
        esc_sys.Start_Soft(esc_target, 0.02f); 
        esc_sys.is_soft_starting = false;
        
        if (cfg_motor_soft_start) {
            PID.base_target_speed = 0.0f;
            PID.startup_state = 3;
            printf("[System] 发车序列启动 -> 跳过前置阶段，直接进入阶段3：底盘软启中...\n");
        } else {
            PID.base_target_speed = speed_target;
            PID.startup_state = 4;
            printf("[System] 发车序列启动 -> 跳过所有发车限制，直接满速起步！\n");
        }
    }
    
    // 3. 清理历史误差
    vL = 0; vR = 0;
    PID_Speed_L.prev_error = 0; PID_Speed_L.last_error = 0; PID_Speed_L.output = 0;
    PID_Speed_R.prev_error = 0; PID_Speed_R.last_error = 0; PID_Speed_R.output = 0;
    PID_Angle.integral = 0; PID_Angle.last_error = 0;
    PID_Speed_F_L.integral = 0; PID_Speed_F_L.last_error = 0; PID_Speed_F_L.output = 0;
    PID_Speed_F_R.integral = 0; PID_Speed_F_R.last_error = 0; PID_Speed_F_R.output = 0;
    reset_curve_slowdown_state(0.0f);
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

void BayWatcher_Set_BaseEscPwm(float percentage){
    // // 统一转换为 0~100 的量纲，并应用限幅
    // percentage = clampf_pid(percentage, 0.0f, 100.0f);
    // // 直接下发给新的电调系统（大步长瞬间达到目标，跳过软起）
    // esc_sys.Start_Soft(percentage, 100.0f);
}

// 留给外部调用的负压差速接口
void BayWatcher_Set_EscDiffConfig(float enable, float ratio){
    esc_sys.enable_esc_diff = clampf_pid(enable, 0.0f, 1.0f);
    esc_sys.esc_diff_ratio = clampf_pid(ratio, 0.0f, 10.0f);
}
#pragma endregion
