#include "common.h"
#include "image_data.h"
#include "image_handle.h"
#include "image_midline_process.h"
#include "image_math.h"
#include "element.h"
#include "element/circle.h"
#include "element/crossing.h"
#include "element/zebra.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <opencv2/core.hpp>

// 纯跟踪角丢线补偿的状态缓存（文件内静态，全局复用）
typedef struct
{
    float last_meas;      // 最近一次有效测量角
    float last_out;       // 最近一次输出角（可能外推）
    float rate;           // 外推斜率（deg/frame）
    float reacq_from;     // 回线软切换起点
    int32_t lost_frames;  // 连续丢线帧
    int32_t reacq_remain; // 回线软切换剩余帧
    uint8_t had_meas;     // 是否曾有有效测量
} pure_angle_lost_state_t;

// 作用域：文件内静态，全局共享一份丢线补偿状态
static pure_angle_lost_state_t g_pure_angle_lost = {0};
static bool g_remote_inner_follow_log_active = false;
static bool g_remote_inner_follow_log_circle = false;
static bool g_remote_crossing_far_follow_log_active = false;
static FollowLine g_remote_crossing_far_follow_log_mode = FollowLine::MIXED;
static bool g_remote_crossing_offset_lock_active = false;
static FollowLine g_remote_crossing_offset_lock_mode = FollowLine::MIXED;
static bool g_remote_crossing_offset_lock_inner = false;
static int32_t g_circle_gate_candidate_streak = 0;
static CircleDirection g_circle_gate_candidate_direction =
    CircleDirection::CIRCLE_DIR_NONE;

// 功能: 限幅单步变化量（用于丢线趋势外推的步长夹紧）
// 类型: 局部功能函数
// 关键参数: v-待限幅值, max_abs-允许的最大绝对值
static inline float clip_step(float v, float max_abs)
{
    return fclip(v, -max_abs, max_abs);
}

static bool update_circle_gate_candidate_confirmation(bool enabled)
{
    CircleDirection direction = CircleDirection::CIRCLE_DIR_NONE;
    const bool raw_candidate =
        enabled && track_get_circle_candidate_direction(&direction);
    if (!raw_candidate)
    {
        g_circle_gate_candidate_streak = 0;
        g_circle_gate_candidate_direction = CircleDirection::CIRCLE_DIR_NONE;
        return false;
    }

    if (direction != g_circle_gate_candidate_direction)
    {
        g_circle_gate_candidate_direction = direction;
        g_circle_gate_candidate_streak = 1;
    }
    else if (g_circle_gate_candidate_streak < 1000)
    {
        ++g_circle_gate_candidate_streak;
    }

    const int32_t confirm_frames =
        std::max<int32_t>(1, BW_CIRCLE_RECOGNITION_GATE_CANDIDATE_CONFIRM_FRAMES);
    return g_circle_gate_candidate_streak >= confirm_frames;
}

// 功能: pure_angle 丢线补偿（趋势外推+回线软切换）
// 类型: 图像处理函数
// 关键参数: measured_angle-当帧测量值, has_valid_measure-是否有有效测量, allow_trend_extrap-是否允许趋势外推
static float pure_angle_apply_lost_strategy(float measured_angle,
                                            bool has_valid_measure,
                                            bool allow_trend_extrap)
{
    if (has_valid_measure)
    {
        if (!g_pure_angle_lost.had_meas)
        {
            g_pure_angle_lost.had_meas = 1;
            g_pure_angle_lost.last_meas = measured_angle;
            g_pure_angle_lost.last_out = measured_angle;
            g_pure_angle_lost.rate = 0.0f;
            g_pure_angle_lost.lost_frames = 0;
            g_pure_angle_lost.reacq_remain = 0;
            return measured_angle;
        }

        // 从丢线恢复：记录软切换起点
        if (g_pure_angle_lost.lost_frames > 0)
        {
            g_pure_angle_lost.reacq_from = g_pure_angle_lost.last_out;
            g_pure_angle_lost.reacq_remain = PUREANGLE_REACQ_BLEND_FRAMES;
            g_pure_angle_lost.lost_frames = 0;
        }

        // 用测量差分估计“趋势”，并限幅 + 轻微滤波
        float step = measured_angle - g_pure_angle_lost.last_meas;
        step = clip_step(step, PUREANGLE_LOST_MAX_STEP_DEG);
        g_pure_angle_lost.rate = 0.5f * g_pure_angle_lost.rate + 0.5f * step;

        g_pure_angle_lost.last_meas = measured_angle;
        g_pure_angle_lost.last_out = measured_angle;

        // 回线软切换
        if (g_pure_angle_lost.reacq_remain > 0)
        {
            float t = (float)(PUREANGLE_REACQ_BLEND_FRAMES - g_pure_angle_lost.reacq_remain + 1)
                    / (float)PUREANGLE_REACQ_BLEND_FRAMES;
            float blended = (1.0f - t) * g_pure_angle_lost.reacq_from + t * measured_angle;
            g_pure_angle_lost.reacq_remain--;
            g_pure_angle_lost.last_out = blended;
            return blended;
        }

        return measured_angle;
    }

    // 无有效测量
    if (!g_pure_angle_lost.had_meas)
    {
        return 0.0f;
    }

    // 不允许趋势外推（特殊元素阶段等）：保持上次测量
    if (!allow_trend_extrap || !PUREANGLE_LOST_TREND_ENABLE)
    {
        g_pure_angle_lost.last_out = g_pure_angle_lost.last_meas;
        g_pure_angle_lost.lost_frames++;
        return g_pure_angle_lost.last_out;
    }

    // 允许趋势外推：短期继续迭代，长期向 0 衰减
    if (g_pure_angle_lost.lost_frames < PUREANGLE_LOST_TREND_FRAMES)
    {
        g_pure_angle_lost.rate = clip_step(g_pure_angle_lost.rate, PUREANGLE_LOST_MAX_STEP_DEG);
        g_pure_angle_lost.last_out += g_pure_angle_lost.rate;
        g_pure_angle_lost.rate *= PUREANGLE_LOST_RATE_DECAY;
    }
    else
    {
        g_pure_angle_lost.last_out *= PUREANGLE_LOST_RETURN_ZERO_DECAY;
        g_pure_angle_lost.rate *= PUREANGLE_LOST_RATE_DECAY;
    }

    g_pure_angle_lost.lost_frames++;
    g_pure_angle_lost.last_out = fclip(g_pure_angle_lost.last_out, -80.0f, 80.0f);
    return g_pure_angle_lost.last_out;
}

namespace {

// 作用域: 文件内静态，斑马线停车门控状态
// 说明：把“触发 -> 延时停车 -> 释放冷却”几个阶段的瞬时量集中在这里，避免散落全局。
struct zebra_gate_state_t
{
    bool prev_stop;
    uint64_t cooldown_until_ms;
    uint64_t recount_sleep_until_ms;
    bool stripe_visible;
    bool pending_stop;
    bool rush_active;
    bool special_lock_active;
    int rush_count;
    uint64_t stop_deadline_ms;
};

// 作用域: 文件内静态，环岛阶段平均角缓存
// 说明：只服务于出环阶段的短时保护，不作为常规 pure_angle 的长期滤波器。
struct circle_angle_cache_t
{
    float sum;
    int count;
};

static zebra_gate_state_t g_zebra_gate = {0};
static circle_angle_cache_t g_circle_angle = {0};

// 功能: 获取图像链统一毫秒时间戳
// 类型: 局部功能函数
static uint64_t image_now_ms()
{
    return (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

static inline int zebra_stop_on_count()
{
    return BW_ZEBRA_STOP_ON_COUNT > 0 ? BW_ZEBRA_STOP_ON_COUNT : 1;
}

static void sync_zebra_runtime_outputs()
{
    zebra_rush_active = g_zebra_gate.rush_active;
    zebra_special_state_locked = g_zebra_gate.special_lock_active;
    zebra_speed_ratio_override = g_zebra_gate.rush_active ? BW_ZEBRA_RUSH_SPEED_RATIO : 1.0f;
}

static void clear_zebra_runtime_state(bool keep_cooldown)
{
    const uint64_t cooldown_until_ms = keep_cooldown ? g_zebra_gate.cooldown_until_ms : 0;
    g_zebra_gate = {0};
    g_zebra_gate.cooldown_until_ms = cooldown_until_ms;
    sync_zebra_runtime_outputs();
}

// 功能: 清空当前帧可见的巡线输出量
// 类型: 局部功能函数
// 说明:
// - 只清视觉链本帧/下一帧会直接读取的量
// - 不主动改动环岛平均角缓存，保持与旧逻辑一致
static void reset_image_processing_outputs()
{
    reset_pts(pts_left);
    reset_pts(pts_right);
    image_reset_far_line_state();
    image_reset_midline_path_state();
    image_reset_tracking_observation_state();
    g_pure_angle_lost = {0};
    ResetPureAnglePreviewTransitionState();
}

// 作用域: 文件内局部枚举，边线搜索结果
// 说明：
// - 普通巡线或 vehicle 特殊起点寻线成功，都返回 OK
// - 仅 vehicle 特殊起点无法建立时，才回退到“保持识别瞬间偏航角”兜底
enum track_search_result_t
{
    TRACK_SEARCH_OK = 0,
    TRACK_SEARCH_VEHICLE_FALLBACK_HOLD,
};

static bool row_has_black_near_core(const uint8_t (&img)[IMAGE_H][IMAGE_W],
                                    int y,
                                    int cx,
                                    int half_width)
{
    if (y < 0 || y >= IMAGE_H)
    {
        return false;
    }

    if (half_width < 0)
    {
        half_width = 0;
    }

    int x0 = cx - half_width;
    int x1 = cx + half_width;
    if (x0 < 0) x0 = 0;
    if (x1 >= IMAGE_W) x1 = IMAGE_W - 1;

    for (int x = x0; x <= x1; ++x)
    {
        if (img[y][x] == BLACK_IN_GRAY)
        {
            return true;
        }
    }
    return false;
}

static bool vehicle_window_all_white(const uint8_t (&img)[IMAGE_H][IMAGE_W], int y)
{
    const int half_width = BW_REMOTE_VEHICLE_SKIP_HALF_WIDTH;
    for (int dy = 0; dy < 5; ++dy)
    {
        const int row = y - dy;
        if (row < 0)
        {
            return false;
        }
        if (row_has_black_near_core(img, row, SET_IMAGE_CORE_X, half_width))
        {
            return false;
        }
    }
    return true;
}

// 功能: vehicle 窗口内，从中心列向上找到更适合的寻线起始行
// 类型: 局部功能函数
// 关键参数:
// - img: 当前帧二值图
// - out_start_y: 输出最终寻线起始行
// 说明：
// - 先从 SET_IMAGE_CORE_Y 往上找离开黑区的位置
// - 再检查上方 5 行是否仍有黑点；若有，则按“最远黑点优先”继续跳过
// - 如果一路退到上半区边界附近仍找不到可靠起点，则放弃本帧 vehicle 寻线
static bool try_find_vehicle_search_start_y(const uint8_t (&img)[IMAGE_H][IMAGE_W], int* out_start_y)
{
    if (out_start_y == nullptr)
    {
        return false;
    }

    const int kMinSearchY = IMAGE_H / 2;
    int temp_y = std::min(SET_IMAGE_CORE_Y, IMAGE_H - 1);
    while (temp_y > (kMinSearchY + 4))
    {
        if (vehicle_window_all_white(img, temp_y))
        {
            *out_start_y = temp_y;
            return true;
        }
        --temp_y;
    }

    return false;
}

// 功能: 常规巡线前的左右边线搜索与处理
// 类型: 局部功能函数
static track_search_result_t process_track_edges(const uint8_t (&img)[IMAGE_H][IMAGE_W],
                                                 bool vehicle_active)
{
    reset_pts(pts_left);
    reset_pts(pts_right);

    int search_y = SET_IMAGE_CORE_Y;
    const bool abandon_bypass_skip_black =
        BW_ABANDON_BYPASS_SKIP_CENTER_BLACK_ENABLE != 0;
    if (vehicle_active || abandon_bypass_skip_black)
    {
        if (!try_find_vehicle_search_start_y(img, &search_y))
        {
            return vehicle_active
                ? TRACK_SEARCH_VEHICLE_FALLBACK_HOLD
                : TRACK_SEARCH_OK;
        }
    }

    if (abandon_bypass_skip_black)
    {
        bool left_start_found = false;
        bool right_start_found = false;
        int32_t left_start_pt[2] = {0, 0};
        int32_t right_start_pt[2] = {0, 0};
        SearchLine_LptEx(img, SET_IMAGE_CORE_X, search_y,
                         pts_left.pts, &pts_left.pts_count,
                         true, &left_start_found, left_start_pt, true);
        SearchLine_RptEx(img, SET_IMAGE_CORE_X, search_y,
                         pts_right.pts, &pts_right.pts_count,
                         true, &right_start_found, right_start_pt, true);
    }
    else
    {
        SearchLine_Lpt(img, SET_IMAGE_CORE_X, search_y, pts_left.pts, &pts_left.pts_count);
        SearchLine_Rpt(img, SET_IMAGE_CORE_X, search_y, pts_right.pts, &pts_right.pts_count);
    }

    process_line(true, pts_left);
    process_line(false, pts_right);
    return TRACK_SEARCH_OK;
}

// 功能: 处理斑马线最终锁停/释放生命周期
// 类型: 局部功能函数
// 返回值: true 表示当前已进入最终停车锁停阶段，应直接 return
static bool handle_zebra_stop_lifecycle(uint64_t t_ms)
{
    if (g_zebra_gate.prev_stop && !zebra_stop)
    {
        g_zebra_gate.cooldown_until_ms = t_ms + (uint64_t)ZEBRA_COOLDOWN_MS;
        printf("[ZEBRA] release\r\n");
        clear_zebra_runtime_state(true);
        reset_image_processing_outputs();
    }

    if (zebra_stop)
    {
        if (!g_zebra_gate.prev_stop)
        {
            printf("[ZEBRA] stop\r\n");
            track_force_reset();
        }

        clear_zebra_runtime_state(false);
        reset_image_processing_outputs();
        g_zebra_gate.prev_stop = true;
        return true;
    }

    g_zebra_gate.prev_stop = false;
    return false;
}

// 功能: 斑马线冲线状态机
// 类型: 局部功能函数
// 返回值: true 表示当前应锁定普通巡线态并强制 MIXED 冲线
static bool update_zebra_rush_state(const uint8_t (&img)[IMAGE_H][IMAGE_W], uint64_t t_ms)
{
    const bool zebra_can_check = (t_ms >= g_zebra_gate.cooldown_until_ms);
    const bool raw_zebra_now = zebra_can_check && zebra_detection(img);
    const bool recount_sleeping =
        (g_zebra_gate.rush_count > 0 &&
         t_ms < g_zebra_gate.recount_sleep_until_ms);
    const bool allow_new_zebra_hit =
        (!recount_sleeping || g_zebra_gate.stripe_visible);
    const bool zebra_now =
        g_zebra_gate.stripe_visible ? raw_zebra_now : (raw_zebra_now && allow_new_zebra_hit);

    if (zebra_now && !g_zebra_gate.stripe_visible)
    {
        g_zebra_gate.stripe_visible = true;
        g_zebra_gate.rush_active = true;
        g_zebra_gate.special_lock_active = true;
        g_zebra_gate.pending_stop = false;
        g_zebra_gate.stop_deadline_ms = 0;
        g_zebra_gate.rush_count++;

        const int required_count = zebra_stop_on_count();
        if (g_zebra_gate.rush_count < required_count)
        {
            g_zebra_gate.recount_sleep_until_ms =
                t_ms + static_cast<uint64_t>(BW_ZEBRA_RECOUNT_SLEEP_MS);
        }

        image_remote_recognition_reset();
        track_force_reset();
        follow_mode = FollowLine::MIXED;

        if (g_zebra_gate.rush_count < required_count)
        {
            printf("[ZEBRA] rush #%d/%d -> lock mixed, resume after disappear\r\n",
                   g_zebra_gate.rush_count,
                   required_count);
            printf("[ZEBRA] recount sleep %d ms\r\n", (int)BW_ZEBRA_RECOUNT_SLEEP_MS);
        }
        else
        {
            printf("[ZEBRA] rush #%d/%d -> lock mixed, stop after disappear\r\n",
                   g_zebra_gate.rush_count,
                   required_count);
        }
    }

    if (!zebra_now && g_zebra_gate.stripe_visible)
    {
        g_zebra_gate.stripe_visible = false;
        g_zebra_gate.rush_active = false;
        g_zebra_gate.special_lock_active = false;

        if (g_zebra_gate.rush_count >= zebra_stop_on_count())
        {
            g_zebra_gate.pending_stop = true;
            g_zebra_gate.stop_deadline_ms = t_ms + (uint64_t)ZEBRA_STOP_DELAY_MS;
            printf("[ZEBRA] rush clear -> stop in %d ms\r\n", (int)ZEBRA_STOP_DELAY_MS);
        }
        else
        {
            printf("[ZEBRA] rush #%d/%d clear -> resume normal track\r\n",
                   g_zebra_gate.rush_count,
                   zebra_stop_on_count());
        }
    }

    if (!g_zebra_gate.rush_active &&
        g_zebra_gate.pending_stop &&
        g_zebra_gate.stop_deadline_ms > 0 &&
        t_ms >= g_zebra_gate.stop_deadline_ms)
    {
        g_zebra_gate.pending_stop = false;
        g_zebra_gate.stop_deadline_ms = 0;
        zebra_stop = true;
    }

    sync_zebra_runtime_outputs();
    return g_zebra_gate.special_lock_active;
}

// 功能: 元素检测与状态机推进
// 类型: 局部功能函数
static void update_track_state_machine(const uint8_t (&img)[IMAGE_H][IMAGE_W])
{
    element_detect();

    if (element_type == ElementType::CIRCLE)
    {
        roundabout_update();
    }

    if (element_type == ElementType::CROSSING)
    {
        crossing_update();
        if (crossing_state != CrossingState::CROSSING_NONE)
        {
            crossing_far_line_check(img);
        }
        else
        {
            image_reset_far_line_state();
        }
    }

    if (element_type == ElementType::NORMAL)
    {
        if_find_far_line = false;
    }
}

// 功能: 拷贝点列到目标缓存
// 类型: 局部功能函数
// 关键参数: src/src_count-源点列, dst/dst_count-目标点列
static void copy_point_line(const float (&src)[PT_MAXLEN][2], int32_t src_count,
                            float (&dst)[PT_MAXLEN][2], int32_t* dst_count)
{
    if (dst_count == nullptr)
    {
        return;
    }

    int32_t n = src_count;
    if (n < 0)
    {
        n = 0;
    }
    if (n > PT_MAXLEN)
    {
        n = PT_MAXLEN;
    }

    *dst_count = n;
    for (int32_t i = 0; i < n; ++i)
    {
        dst[i][0] = src[i][0];
        dst[i][1] = src[i][1];
    }
}

static int32_t compute_preview_curve_split_index_for_mixed(int32_t left_mid_count,
                                                           int32_t right_mid_count,
                                                           FollowLine mode)
{
    if (mode != FollowLine::MIXED)
    {
        return -1;
    }

    if (left_mid_count <= 0 || right_mid_count <= 0)
    {
        return -1;
    }

    const int32_t overlap_count =
        (left_mid_count < right_mid_count) ? left_mid_count : right_mid_count;
    const int32_t full_count =
        (left_mid_count > right_mid_count) ? left_mid_count : right_mid_count;

    if (overlap_count <= 0 || full_count <= overlap_count)
    {
        return -1;
    }

    return overlap_count;
}

static float remote_follow_segment_heading_deg(const float (&line)[PT_MAXLEN][2],
                                               int32_t from,
                                               int32_t to)
{
    if (from < 0) from = 0;
    if (to < 0) to = 0;
    if (from > PT_MAXLEN - 1) from = PT_MAXLEN - 1;
    if (to > PT_MAXLEN - 1) to = PT_MAXLEN - 1;

    const float dy = line[to][0] - line[from][0];
    const float dx = line[to][1] - line[from][1];
    const float forward = -dy;
    if (dx * dx + forward * forward <= 1e-8f)
    {
        return 0.0f;
    }

    return -atan2f(dx, forward) * 180.0f / PI32;
}

static float remote_follow_normalize_delta_deg(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float estimate_remote_follow_signed_curve_deg(const float (&line)[PT_MAXLEN][2],
                                                     int32_t line_count)
{
    int32_t n = line_count;
    if (n > PT_MAXLEN)
    {
        n = PT_MAXLEN;
    }
    if (n < 3)
    {
        return 0.0f;
    }

    int32_t k = PUREANGLE_PREVIEW_CURV_DIST;
    if (k < 1)
    {
        k = 1;
    }
    if (k > (n - 1) / 2)
    {
        k = (n - 1) / 2;
    }
    if (k < 1)
    {
        k = 1;
    }

    const float near_heading = remote_follow_segment_heading_deg(line, 0, k);
    const float far_heading = remote_follow_segment_heading_deg(line, k, n - 1);
    const float delta = remote_follow_normalize_delta_deg(far_heading - near_heading);

    if (std::fabs(delta) >= BW_REMOTE_FOLLOW_INNER_CURVE_THRESHOLD_DEG)
    {
        return delta;
    }
    if (std::fabs(far_heading) >= BW_REMOTE_FOLLOW_INNER_CURVE_THRESHOLD_DEG)
    {
        return far_heading;
    }
    return 0.0f;
}

static bool is_remote_follow_inner(bool is_left,
                                   const float (&forced_line)[PT_MAXLEN][2],
                                   int32_t forced_count)
{
    const float signed_curve_deg =
        estimate_remote_follow_signed_curve_deg(forced_line, forced_count);
    if (is_left)
    {
        return signed_curve_deg >= BW_REMOTE_FOLLOW_INNER_CURVE_THRESHOLD_DEG;
    }
    return signed_curve_deg <= -BW_REMOTE_FOLLOW_INNER_CURVE_THRESHOLD_DEG;
}

static bool is_circle_running_inner_follow(bool is_left)
{
    if (circle_state != CircleState::CIRCLE_RUNNING)
    {
        return false;
    }

    return (circle_direction == CircleDirection::CIRCLE_DIR_LEFT && is_left) ||
           (circle_direction == CircleDirection::CIRCLE_DIR_RIGHT && !is_left);
}

// 功能: 远端 w/s 锁边时，基于锁定侧边线生成绕行 path
// 类型: 局部功能函数
// 关键参数: forced_mode-锁定到左/右边线
// 说明：CROSSING_IN 红色仍可见时优先使用近线，并在对应侧角点处截断；
// 对应角点无效、红色丢失保持或进入 RUNNING 时使用远线。
// 十字内外切线时共用并锁定同一推移类型与比例。
static bool build_path_from_remote_follow_override(FollowLine forced_mode)
{
    follow_mode = forced_mode;
    midline.preview_curve_split_index = -1;

    const bool crossing_active =
        crossing_state != CrossingState::CROSSING_NONE;
    const bool crossing_in_visible_sign =
        crossing_state == CrossingState::CROSSING_IN &&
        image_remote_recognition_is_visible_sign_follow_active();
    bool use_crossing_far_edge =
        crossing_active && !crossing_in_visible_sign;

    bool is_left = false;
    if (forced_mode == FollowLine::MIDLEFT)
    {
        is_left = true;
    }
    else if (forced_mode == FollowLine::MIDRIGHT)
    {
        is_left = false;
    }
    else
    {
        return false;
    }

    pts_well_processed* src = is_left
        ? (use_crossing_far_edge ? &pts_far_left : &pts_left)
        : (use_crossing_far_edge ? &pts_far_right : &pts_right);
    if (crossing_in_visible_sign && !use_crossing_far_edge)
    {
        const bool near_corner_valid =
            src->pts_resample_count > 0 &&
            src->corner_found &&
            src->corner_id >= 0 &&
            src->corner_id < src->pts_resample_count;
        if (!near_corner_valid)
        {
            use_crossing_far_edge = true;
            src = is_left ? &pts_far_left : &pts_far_right;
        }
    }

    if (use_crossing_far_edge && !if_find_far_line)
    {
        g_remote_crossing_far_follow_log_active = false;
        g_remote_crossing_far_follow_log_mode = FollowLine::MIXED;
        follow_mode = FollowLine::MIXED;
        return false;
    }

    if (use_crossing_far_edge)
    {
        if (!g_remote_crossing_far_follow_log_active ||
            g_remote_crossing_far_follow_log_mode != forced_mode)
        {
            std::printf("[十字绕行] 识别绕行线使用%s远边线\n",
                        is_left ? "左侧" : "右侧");
        }
        g_remote_crossing_far_follow_log_active = true;
        g_remote_crossing_far_follow_log_mode = forced_mode;
    }
    else
    {
        g_remote_crossing_far_follow_log_active = false;
        g_remote_crossing_far_follow_log_mode = FollowLine::MIXED;
    }

    if (src->pts_resample_count <= 0)
    {
        if (use_crossing_far_edge)
        {
            follow_mode = FollowLine::MIXED;
        }
        return false;
    }

    int32_t source_count = src->pts_resample_count;
    if (crossing_in_visible_sign && !use_crossing_far_edge)
    {
        // 近线点序从车端向远端，保留角点本身，排除角点后的十字内部线段。
        source_count = src->corner_id + 1;
    }

    const bool circle_running =
        circle_state == CircleState::CIRCLE_RUNNING &&
        (circle_direction == CircleDirection::CIRCLE_DIR_LEFT ||
         circle_direction == CircleDirection::CIRCLE_DIR_RIGHT);
    const bool circle_inner_follow =
        circle_running && is_circle_running_inner_follow(is_left);
    const bool use_crossing_offset_lock =
        !circle_running &&
        g_remote_crossing_offset_lock_active &&
        g_remote_crossing_offset_lock_mode == forced_mode;
    const bool use_crossing_offset_profile =
        crossing_active || use_crossing_offset_lock;
    const float initial_offset_ratio = circle_running
        ? (circle_inner_follow
            ? BW_REMOTE_FOLLOW_CIRCLE_INNER_OFFSET_RATIO
            : BW_REMOTE_FOLLOW_CIRCLE_OUTER_OFFSET_RATIO)
        : (use_crossing_offset_profile
            ? (use_crossing_offset_lock && g_remote_crossing_offset_lock_inner
                ? BW_REMOTE_FOLLOW_CROSSING_INNER_OFFSET_RATIO
                : BW_REMOTE_FOLLOW_CROSSING_OUTER_OFFSET_RATIO)
            : BW_REMOTE_FOLLOW_OUTER_OFFSET_RATIO);

    float forced_line[PT_MAXLEN][2] = {};
    int32_t forced_count = 0;
    BuildRemoteFollowOuterLine(is_left,
                               src->pts_resample, &source_count,
                               forced_line, &forced_count,
                               initial_offset_ratio);
    if (forced_count <= 0)
    {
        if (use_crossing_far_edge)
        {
            follow_mode = FollowLine::MIXED;
        }
        return false;
    }

    const bool inner_follow = circle_running
        ? circle_inner_follow
        : (use_crossing_offset_lock
            ? g_remote_crossing_offset_lock_inner
            : is_remote_follow_inner(is_left, forced_line, forced_count));
    if (inner_follow && !circle_running &&
        !(use_crossing_offset_lock && g_remote_crossing_offset_lock_inner))
    {
        forced_count = 0;
        BuildRemoteFollowOuterLine(is_left,
                                   src->pts_resample, &source_count,
                                   forced_line, &forced_count,
                                   use_crossing_offset_profile
                                       ? BW_REMOTE_FOLLOW_CROSSING_INNER_OFFSET_RATIO
                                       : BW_REMOTE_FOLLOW_INNER_OFFSET_RATIO);
        if (forced_count <= 0)
        {
            if (use_crossing_far_edge)
            {
                follow_mode = FollowLine::MIXED;
            }
            return false;
        }
    }

    copy_point_line(forced_line, forced_count, midline.mid, &midline.mid_count);
    copy_point_line(forced_line, forced_count, midline.path, &midline.path_count);

    if (midline.mid_count <= 0 || midline.path_count <= 0)
    {
        if (use_crossing_far_edge)
        {
            follow_mode = FollowLine::MIXED;
        }
        return false;
    }

    if (crossing_active && !use_crossing_offset_lock)
    {
        g_remote_crossing_offset_lock_active = true;
        g_remote_crossing_offset_lock_mode = forced_mode;
        g_remote_crossing_offset_lock_inner = inner_follow;
    }

    if (inner_follow &&
        (!g_remote_inner_follow_log_active ||
         g_remote_inner_follow_log_circle != circle_inner_follow))
    {
        printf("%s\n", circle_inner_follow ? "环岛内绕" : "内绕");
        g_remote_inner_follow_log_active = true;
        g_remote_inner_follow_log_circle = circle_inner_follow;
    }
    else if (!inner_follow)
    {
        g_remote_inner_follow_log_active = false;
        g_remote_inner_follow_log_circle = false;
    }

    CalculatePureAngleFromPath(midline.path, midline.path_count, &pure_angle);
    image_remote_recognition_set_follow_path_state(true, inner_follow);
    return true;
}

// 功能: 砖块压制期间，只清空环岛运行态，不让环岛状态机继续推进
// 类型: 局部功能函数
static void clear_circle_runtime_for_brick()
{
    roundabout_reset();
    if (element_type == ElementType::CIRCLE)
    {
        element_type = ElementType::NORMAL;
    }
    follow_mode = FollowLine::MIXED;
    image_reset_far_line_state();
}

// 功能: 根据当前识别覆盖/元素状态选择最终中线来源
// 类型: 局部功能函数
static void build_midline_from_current_state()
{
    if (element_type == ElementType::CROSSING && if_find_far_line)
    {
        MID(pts_far_left.mid, &pts_far_left.mid_count,
            pts_far_right.mid, &pts_far_right.mid_count,
            midline.mid, &midline.mid_count,
            follow_mode);
        midline.preview_curve_split_index =
            compute_preview_curve_split_index_for_mixed(pts_far_left.mid_count,
                                                        pts_far_right.mid_count,
                                                        follow_mode);
        return;
    }

    MID(pts_left.mid, &pts_left.mid_count,
        pts_right.mid, &pts_right.mid_count,
        midline.mid, &midline.mid_count,
        follow_mode);
    midline.preview_curve_split_index =
        compute_preview_curve_split_index_for_mixed(pts_left.mid_count,
                                                    pts_right.mid_count,
                                                    follow_mode);
}

// 功能: 收到 bl/br 后，在送入 path 前把中线向红砖反方向侧移
// 类型: 局部功能函数
// 关键参数: direction-+1 向右侧移，-1 向左侧移
static void apply_remote_brick_avoid_shift_to_midline(int direction)
{
    if (direction == 0 || midline.mid_count <= 0)
    {
        return;
    }

    int n = midline.mid_count;
    if (n > PT_MAXLEN)
    {
        n = PT_MAXLEN;
    }

    const float offset = ROADWIDTH * PIXPERMETER * BW_REMOTE_BRICK_AVOID_OFFSET_RATIO;
    if (!(offset > 0.0f))
    {
        return;
    }

    int span = (int)(ANGLEDIST / RESAMPLEDIST + 0.5f);
    if (span < 1)
    {
        span = 1;
    }

    float shifted[PT_MAXLEN][2] = {};
    for (int i = 0; i < n; ++i)
    {
        int im = i - span;
        if (im < 0)
        {
            im = 0;
        }
        int ip = i + span;
        if (ip > n - 1)
        {
            ip = n - 1;
        }

        const float dx = midline.mid[ip][1] - midline.mid[im][1];
        const float dy = midline.mid[ip][0] - midline.mid[im][0];
        const float len2 = dx * dx + dy * dy;
        if (len2 <= 1e-12f)
        {
            if (i > 0)
            {
                shifted[i][0] = shifted[i - 1][0];
                shifted[i][1] = shifted[i - 1][1];
            }
            else
            {
                shifted[i][0] = midline.mid[i][0];
                shifted[i][1] = midline.mid[i][1];
            }
            continue;
        }

        float inv_len = fast_rsqrt(len2);
        inv_len = inv_len * (1.5f - 0.5f * len2 * inv_len * inv_len);
        const float cosv = dx * inv_len;
        const float sinv = dy * inv_len;

        if (direction > 0)
        {
            shifted[i][1] = midline.mid[i][1] - sinv * offset;
            shifted[i][0] = midline.mid[i][0] + cosv * offset;
        }
        else
        {
            shifted[i][1] = midline.mid[i][1] + sinv * offset;
            shifted[i][0] = midline.mid[i][0] - cosv * offset;
        }
    }

    copy_point_line(shifted, n, midline.mid, &midline.mid_count);
}

// 功能: 从最终中线构建路径并计算当帧测量角
// 类型: 局部功能函数
static void build_path_and_measure_pure_angle()
{
    int brick_shift_direction = 0;
    if (image_remote_recognition_get_brick_avoid_shift_direction(&brick_shift_direction))
    {
        apply_remote_brick_avoid_shift_to_midline(brick_shift_direction);
    }

    BuildPathFromCoreToMidlineArc(midline.mid, midline.mid_count,
                                  midline.path, &midline.path_count,
                                  RESAMPLEDIST * PIXPERMETER);
    CalculatePureAngleFromPath(midline.path, midline.path_count, &pure_angle);
}

// 功能: 根据赛道状态决定当前帧是否允许丢线趋势外推
// 类型: 局部功能函数
static bool should_allow_trend_extrap(bool has_valid_measure)
{
    const bool no_special_state = (crossing_state == CrossingState::CROSSING_NONE &&
                                   circle_state == CircleState::CIRCLE_NONE);
    const bool hard_lost_both = (pts_left.pts_count == 0 && pts_right.pts_count == 0);

    if (no_special_state)
    {
        return hard_lost_both;
    }

    return !has_valid_measure;
}

// 功能: 更新环岛平均角缓存
// 类型: 局部功能函数
static void update_circle_average_cache(bool has_valid_measure, float measured_angle)
{
    if (circle_state == CircleState::CIRCLE_IN)
    {
        g_circle_angle.sum = 0.0f;
        g_circle_angle.count = 0;
        circle_average_angle = 0.0f;
        return;
    }

    if (circle_state == CircleState::CIRCLE_RUNNING && has_valid_measure)
    {
        g_circle_angle.sum += measured_angle;
        g_circle_angle.count++;
        circle_average_angle = g_circle_angle.sum / (float)g_circle_angle.count;
    }
}

// 功能: 判断环岛出环阶段是否应启用平均角保护
// 类型: 局部功能函数
static bool should_use_circle_average()
{
    if (circle_state != CircleState::CIRCLE_OUT || g_circle_angle.count <= 0)
    {
        return false;
    }

    const bool is_right = (circle_direction == CircleDirection::CIRCLE_DIR_RIGHT);
    const bool tracking_line_lost = is_right ? (pts_right.pts_count == 0) : (pts_left.pts_count == 0);
    return tracking_line_lost;
}

// 功能: 对测量角应用环岛保护与丢线补偿
// 类型: 局部功能函数
static float finalize_pure_angle_output(float measured_angle, bool has_valid_measure)
{
    const bool allow_trend_extrap = should_allow_trend_extrap(has_valid_measure);
    update_circle_average_cache(has_valid_measure, measured_angle);

    if (should_use_circle_average())
    {
        return pure_angle_apply_lost_strategy(circle_average_angle, true, false);
    }

    return pure_angle_apply_lost_strategy(measured_angle, has_valid_measure, allow_trend_extrap);
}

} // namespace

// 图像处理逻辑链条（image_process 总控）：
// 1) 斑马线停车逻辑（锁停/延时/冷却）与视觉量清空
// 2) 边线搜索（左右）与边线处理（逆透视→滤波→重采样→曲率/角点/中线）
// 3) 元素检测与状态机更新（环岛/十字/斑马线）
// 4) 远端线探测（十字）与中线融合（MIXED/MIDLEFT/MIDRIGHT）
// 5) 路径构建（从 core 到中线弧形并轨）与 pure_angle 计算
// 6) 丢线补偿（趋势外推/回线软切换）输出纯跟踪角
// 功能: 图像处理总控入口（输入二值图，输出纯跟踪角/状态）
// 类型: 图像处理总控函数（image_process）
// 关键参数: img-二值图像(0/255)，尺寸 IMAGE_H x IMAGE_W
void img_processing(const uint8_t (&img)[IMAGE_H][IMAGE_W])
{
    const uint64_t t_ms = image_now_ms();
    image_remote_recognition_tick(t_ms);
    if (handle_zebra_stop_lifecycle(t_ms))
    {
        update_circle_gate_candidate_confirmation(false);
        image_remote_recognition_set_follow_path_state(false, false);
        return;
    }

    // follow_mode 由上层策略决定，这里只消费，不在主链入口硬重置。
    const bool vehicle_active = image_remote_recognition_is_vehicle_active(t_ms);
    const track_search_result_t track_search = process_track_edges(img, vehicle_active);
    const bool zebra_special_lock = update_zebra_rush_state(img, t_ms);
    if (track_search == TRACK_SEARCH_VEHICLE_FALLBACK_HOLD)
    {
        update_circle_gate_candidate_confirmation(false);
        float hold_yaw = 0.0f;

        if (image_remote_recognition_try_get_hold_yaw(t_ms, &hold_yaw))
        {
            reset_image_processing_outputs();
            image_remote_recognition_set_follow_path_state(false, false);
            pure_angle = hold_yaw;
            return;
        }

        reset_image_processing_outputs();
        image_remote_recognition_set_follow_path_state(false, false);
        return;
    }

    // 环岛门控必须抢在远端接管分支前更新；候选使用独立的短确认，
    // 既早于 element_detect 的环岛状态确认，又过滤十字角点不同步造成的单帧误报。
    const bool circle_candidate =
        update_circle_gate_candidate_confirmation(!zebra_special_lock);
    image_circle_recognition_gate_update(circle_candidate, t_ms);

    const bool remote_circle_block = image_remote_recognition_should_block_circle(t_ms);
    const bool remote_route_active = image_remote_recognition_should_freeze_state_machine(t_ms);

    //=====================================================================================

    // //测试赛道宽度矫正PIXPERMETER
    // float roadwidth_pix_sqr = (100.0f) * (100.0f);
    // int id_roadwidth_test = 0;
    // for(int i= -5 ; i < 5; i++)
    // {
    //     float dx=pts_left.pts_resample[10][1]- pts_right.pts_resample[10 + i][1];
    //     float dy=pts_left.pts_resample[10][0]- pts_right.pts_resample[10 + i][0];
    //     float roadwidth_pix_temp_sqr = (dx*dx + dy*dy);

    //     if(roadwidth_pix_temp_sqr < roadwidth_pix_sqr)
    //     {
    //         roadwidth_pix_sqr = roadwidth_pix_temp_sqr;
    //         id_roadwidth_test =10 + i;
    //     }
    // }
    // printf("赛道宽度像素值: %.2f pix\r\n",  Q_sqrt(roadwidth_pix_sqr));
    // printf("理论赛道宽度像素值: %.2f pix\r\n",  ROADWIDTH * PIXPERMETER);
    // printf("中线偏移量: %.2f pix\r\n",  PIXPERMETER * ROADWIDTH / 2);
    // float PIXPERMETER_NEW = Q_sqrt(roadwidth_pix_sqr) / ROADWIDTH ;
    // printf("PIXPERMETER_NEW: %.2f pix/m\r\n",  PIXPERMETER_NEW);
    // printf("id_roadwidth_test"": %d\r\n",  id_roadwidth_test);
    // return;

    //=========================================================================================================

    // //测试中线偏移量矫正PIXPERMETER_ACROSS
    // float mid_div_pix_sqr = (100.0f) * (100.0f);
    // int id_div_test = 0;
    // for(int i=  0; i < pts_right.mid_count; i++)
    // {
    //     float dx= pts_left.mid[10][1] - pts_right.mid[ i][1];
    //     float dy= pts_left.mid[10][0] - pts_right.mid[i][0];
    //     float mid_div_pix_temp_sqr = (dx*dx + dy*dy);

    //     if(mid_div_pix_temp_sqr < mid_div_pix_sqr)
    //     {
    //         mid_div_pix_sqr = mid_div_pix_temp_sqr;
    //         id_div_test = i;
    //     }
    // }
    // printf("中线偏移量像素值: %.2f pix\r\n",  Q_sqrt(mid_div_pix_sqr));
    // printf("id_div_test"": %d\r\n",  id_div_test);
    // printf("中线左点:(%.2f, %.2f), 右点:(%.2f, %.2f)\r\n", 
    //         pts_left.mid[10][1], pts_left.mid[10][0],
    //         pts_right.mid[id_div_test][1], pts_right.mid[id_div_test][0]
    // );
    // return;

    if (zebra_special_lock)
    {
        track_reset_element_runtime_state(true);
        image_reset_far_line_state();
        follow_mode = FollowLine::MIXED;
    }
    else if (remote_route_active && circle_state != CircleState::CIRCLE_RUNNING)
    {
        // 普通赛道远端 w/s/v 期间冻结元素状态机。
        // CIRCLE_RUNNING 仍推进环岛状态机；一旦切到 OUT，门控会立即清掉远端接管。
        // 不清 element_type/circle_state/crossing_state，便于退出远端接管后继续沿原上下文恢复。
        // 只有 b/bl/br（remote_circle_block）会走清状态机分支。
    }
    else if (remote_circle_block)
    {
        clear_circle_runtime_for_brick();
    }
    else
    {
        update_track_state_machine(img);
    }

    // 状态机可能在本帧进入 RUNNING（放行）或从 RUNNING 进入 OUT（阻断），
    // 因此在选择最终 path 前再同步一次门控。
    image_circle_recognition_gate_update(circle_candidate, t_ms);

    FollowLine forced_follow_mode = FollowLine::MIXED;
    const bool remote_follow_locked =
        image_remote_recognition_get_forced_follow_mode(&forced_follow_mode);
    if (!remote_follow_locked ||
        (g_remote_crossing_offset_lock_active &&
         g_remote_crossing_offset_lock_mode != forced_follow_mode))
    {
        g_remote_crossing_offset_lock_active = false;
        g_remote_crossing_offset_lock_mode = FollowLine::MIXED;
        g_remote_crossing_offset_lock_inner = false;
    }
    bool remote_follow_override_applied = false;
    if (remote_follow_locked && !zebra_special_lock)
    {
        remote_follow_override_applied =
            build_path_from_remote_follow_override(forced_follow_mode);
    }

    if (!remote_follow_override_applied)
    {
        g_remote_inner_follow_log_active = false;
        g_remote_inner_follow_log_circle = false;
        image_remote_recognition_set_follow_path_state(false, false);
        build_midline_from_current_state();
        build_path_and_measure_pure_angle();
    }

    const bool has_valid_measure = (midline.mid_count > 0 && midline.path_count > 0);
    pure_angle = finalize_pure_angle_output(pure_angle, has_valid_measure);
}

// 功能: OpenCV Mat 版本图像处理入口（进行合法性/连续性校验后转调）
// 类型: 图像处理总控函数（image_process）
// 关键参数: binary-CV_8UC1 二值图(需连续, 尺寸 IMAGE_H x IMAGE_W)
void img_processing(const cv::Mat& binary)
{
    if (binary.empty())
    {
        return;
    }
    if (binary.type() != CV_8UC1 || binary.rows != IMAGE_H || binary.cols != IMAGE_W)
    {
        return;
    }
    // 需要连续且 stride==IMAGE_W，才能安全地把 data 解释成 [H][W]
    if (!binary.isContinuous() || binary.step != IMAGE_W)
    {
        // 退化路径：拷贝到连续缓冲再处理（保持行为正确）
        uint8_t tmp[IMAGE_H][IMAGE_W];
        for (int y = 0; y < IMAGE_H; ++y)
        {
            std::memcpy(tmp[y], binary.ptr<uint8_t>(y), IMAGE_W);
        }
        img_processing(tmp);
        return;
    }

    const uint8_t (&img_ref)[IMAGE_H][IMAGE_W] = *reinterpret_cast<const uint8_t (*)[IMAGE_H][IMAGE_W]>(binary.data);
    img_processing(img_ref);
}
