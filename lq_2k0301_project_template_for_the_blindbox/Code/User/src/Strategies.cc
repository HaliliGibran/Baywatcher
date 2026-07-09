#include "main.hpp"

struct PID3Gain {
    float kp;
    float ki;
    float kd;
};

struct CubePIDGain {
    float kp_a;
    float kp_b;
    float ki;
    float kd_a;
    float kd_b;
};

struct BayWatcherStrategy {
    const char* name;
    float start_speed;

    PID3Gain speed_l;
    PID3Gain speed_r;
    CubePIDGain cube;

    bool start_delay_enable;
    bool esc_soft_start_enable;
    bool motor_soft_start_enable;
    uint32_t startup_delay_ms;
    float startup_esc_target;
    float startup_esc_step;
    float startup_speed_step;

    bool vofa_remote_enable;
    bool straight_accel_enable;
    float straight_accel_max_add;
    float straight_accel_curve_min_th;
    float straight_accel_yaw_min_th;
    float straight_accel_curve_max_th;
    float straight_accel_yaw_max_th;
    float straight_accel_intensity;

    bool dynamic_esc_enable;
    float dynamic_esc_base;
    float dynamic_esc_max;
    float dynamic_esc_threshold;
    float dynamic_esc_factor_max;
    float dynamic_esc_lp_alpha;
};

static const BayWatcherStrategy k_strategies[] = {
    {
        "Plan1 Race",
        20.0f,
        {113.32f, 26.00f, 0.00f},
        {113.00f, 26.00f, 0.00f},
        {6.565f, 0.4844f, 0.0f, 302.10f, 0.00100f},
        true,
        true,
        false,
        0,
        70.0f,
        0.075f,
        0.05f,
        false,
        false,
        8.0f,
        5.4f,
        3.6f,
        6.5f,
        6.8f,
        0.4f,
        false,
        30.0f,
        45.0f,
        0.45f,
        1.00f,
        0.4f,
    },
    {
        "Plan2 Safe",
        16.0f,
        {100.00f, 22.00f, 0.00f},
        {100.00f, 22.00f, 0.00f},
        {5.900f, 0.4600f, 0.0f, 360.00f, 0.00100f},
        true,
        true,
        true,
        300,
        55.0f,
        0.060f,
        0.04f,
        false,
        false,
        4.0f,
        5.4f,
        3.6f,
        6.5f,
        6.8f,
        0.4f,
        false,
        25.0f,
        40.0f,
        0.45f,
        1.00f,
        0.3f,
    },
    {
        "Plan3 Fast",
        22.0f,
        {118.00f, 28.00f, 0.00f},
        {118.00f, 28.00f, 0.00f},
        {6.700f, 0.4900f, 0.0f, 280.00f, 0.00120f},
        true,
        true,
        false,
        0,
        75.0f,
        0.085f,
        0.06f,
        false,
        true,
        6.0f,
        5.4f,
        3.6f,
        6.5f,
        6.8f,
        0.4f,
        false,
        35.0f,
        50.0f,
        0.45f,
        1.00f,
        0.4f,
    },
};

static uint8_t g_active_strategy = 0;

uint8_t BayWatcher_Strategy_Count(void)
{
    return static_cast<uint8_t>(sizeof(k_strategies) / sizeof(k_strategies[0]));
}

uint8_t BayWatcher_Strategy_Active(void)
{
    return g_active_strategy;
}

const char* BayWatcher_Strategy_Name(uint8_t strategy_id)
{
    if (strategy_id == 0 || strategy_id > BayWatcher_Strategy_Count()) {
        return "Boot Default";
    }
    return k_strategies[strategy_id - 1].name;
}

void BayWatcher_Apply_Strategy(uint8_t strategy_id)
{
    if (strategy_id == 0 || strategy_id > BayWatcher_Strategy_Count()) {
        return;
    }

    const BayWatcherStrategy& p = k_strategies[strategy_id - 1];

    PID.base_target_speed = p.start_speed;
    if (!PID.is_running) {
        PID.yaw_control_base_speed = p.start_speed;
    }

    PID_Speed_L.Kp = p.speed_l.kp;
    PID_Speed_L.Ki = p.speed_l.ki;
    PID_Speed_L.Kd = p.speed_l.kd;
    PID_Speed_L.output_limit = 9999.0f;

    PID_Speed_R.Kp = p.speed_r.kp;
    PID_Speed_R.Ki = p.speed_r.ki;
    PID_Speed_R.Kd = p.speed_r.kd;
    PID_Speed_R.output_limit = 9999.0f;

    PID_Cube.Kp_a = p.cube.kp_a;
    PID_Cube.Kp_b = p.cube.kp_b;
    PID_Cube.Ki = p.cube.ki;
    PID_Cube.Kd_a = p.cube.kd_a;
    PID_Cube.Kd_b = p.cube.kd_b;

    cfg_start_delay = p.start_delay_enable;
    cfg_esc_soft_start = p.esc_soft_start_enable;
    cfg_motor_soft_start = p.motor_soft_start_enable;
    g_startup_delay_ms = p.startup_delay_ms;
    g_startup_esc_target = p.startup_esc_target;
    g_startup_esc_step = p.startup_esc_step;
    g_startup_speed_step = p.startup_speed_step;

    cfg_vofa_remote_enable = p.vofa_remote_enable;
    cfg_straight_accel_enable = p.straight_accel_enable;
    cfg_straight_accel_max_add = p.straight_accel_max_add;
    cfg_straight_accel_curve_min_th = p.straight_accel_curve_min_th;
    cfg_straight_accel_yaw_min_th = p.straight_accel_yaw_min_th;
    cfg_straight_accel_curve_max_th = p.straight_accel_curve_max_th;
    cfg_straight_accel_yaw_max_th = p.straight_accel_yaw_max_th;
    cfg_straight_accel_intensity = p.straight_accel_intensity;

    esc_sys.enable_esc_diff = false;
    esc_sys.esc_diff_ratio = 0.0f;
    esc_sys.esc_diff_limit = 0.0f;
    esc_sys.enable_dynamic_esc = p.dynamic_esc_enable;
    esc_sys.dynamic_esc_base = p.dynamic_esc_base;
    esc_sys.dynamic_esc_max = p.dynamic_esc_max;
    esc_sys.dynamic_esc_threshold = p.dynamic_esc_threshold;
    esc_sys.dynamic_esc_factor_max = p.dynamic_esc_factor_max;
    esc_sys.lp_alpha = p.dynamic_esc_lp_alpha;
    esc_sys.filtered_target_base = p.dynamic_esc_base;

    if (!PID.is_running) {
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

        PID_Cube.error = 0.0f;
        PID_Cube.integral = 0.0f;
        PID_Cube.last_error = 0.0f;
        PID_Cube.gyro = 0.0f;
        PID_Cube.output = 0.0f;
    }

    g_active_strategy = strategy_id;
    printf("[Strategy] applied %s, start speed %.1f\n", p.name, p.start_speed);
}
