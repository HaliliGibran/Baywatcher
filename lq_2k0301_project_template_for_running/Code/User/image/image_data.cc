#include "image_data.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>

namespace {

constexpr uint64_t kRemoteVehicleHoldMs = static_cast<uint64_t>(BW_REMOTE_VEHICLE_HOLD_MS);
constexpr uint64_t kRemoteBrickBlockMs = static_cast<uint64_t>(BW_REMOTE_BRICK_BLOCK_MS);

enum class remote_follow_state_t : uint8_t
{
    NONE = 0,
    VEHICLE_ROUTE,
    LEFT_EDGE_ROUTE,
    RIGHT_EDGE_ROUTE,
};

struct remote_recognition_runtime_t
{
    bool last_seq_valid;
    uint8_t last_seq;
    BoardVisionCode current_code;
    uint64_t last_rx_ms;
    uint64_t vehicle_hold_until_ms;
    float vehicle_hold_yaw;
    remote_follow_state_t follow_state;
    uint64_t brick_block_until_ms;
    bool follow_path_active;
    bool inner_follow_active;
};

remote_recognition_runtime_t g_remote_recognition = {
    false,
    0,
    BoardVisionCode::UNKNOWN,
    0,
    0,
    0.0f,
    remote_follow_state_t::NONE,
    0,
    false,
    false,
};

constexpr uint32_t kRemoteSpeedCapReasonCloth = 1u << 0;
constexpr float kRemoteSpeedCapLogMinDelta = 0.02f;

bool g_remote_speed_cap_log_active = false;
float g_remote_speed_cap_log_value = 0.0f;
uint32_t g_remote_speed_cap_log_reasons = 0;
bool g_remote_follow_speed_log_active = false;
bool g_remote_follow_speed_log_inner = false;
bool g_remote_follow_speed_log_released = false;
float g_remote_follow_speed_log_ratio = 1.0f;
std::atomic<bool> g_circle_recognition_gate_blocked{false};
uint64_t g_circle_recognition_gate_hold_until_ms = 0;

void remote_vehicle_route_apply(float current_pure_angle, uint64_t t_ms)
{
    g_remote_recognition.follow_state = remote_follow_state_t::VEHICLE_ROUTE;
    g_remote_recognition.vehicle_hold_yaw = current_pure_angle;
    g_remote_recognition.vehicle_hold_until_ms = t_ms + kRemoteVehicleHoldMs;
}

bool is_remote_brick_code(BoardVisionCode code)
{
    return code == BoardVisionCode::BRICK ||
           code == BoardVisionCode::BRICK_LEFT ||
           code == BoardVisionCode::BRICK_RIGHT;
}

float sanitize_remote_speed_cap(float cap)
{
    return (cap < 0.0f) ? 0.0f : cap;
}

void add_remote_speed_cap_candidate(bool active,
                                    float cap,
                                    uint32_t reason,
                                    bool* has_cap,
                                    float* best_cap,
                                    uint32_t* reasons)
{
    if (!active || has_cap == nullptr || best_cap == nullptr || reasons == nullptr)
    {
        return;
    }

    cap = sanitize_remote_speed_cap(cap);
    if (!(*has_cap) || cap < *best_cap)
    {
        *has_cap = true;
        *best_cap = cap;
        *reasons = reason;
        return;
    }

    const float diff = cap - *best_cap;
    if (diff > -1e-4f && diff < 1e-4f)
    {
        *reasons |= reason;
    }
}

void print_remote_speed_cap_log(bool active, float cap, uint32_t reasons)
{
    if (!active)
    {
        if (g_remote_speed_cap_log_active)
        {
            std::printf("[远端减速] 解除\n");
            g_remote_speed_cap_log_active = false;
            g_remote_speed_cap_log_value = 0.0f;
            g_remote_speed_cap_log_reasons = 0;
        }
        return;
    }

    float cap_diff = cap - g_remote_speed_cap_log_value;
    if (cap_diff < 0.0f)
    {
        cap_diff = -cap_diff;
    }

    const bool changed =
        !g_remote_speed_cap_log_active ||
        g_remote_speed_cap_log_reasons != reasons ||
        cap_diff >= kRemoteSpeedCapLogMinDelta;
    if (!changed)
    {
        return;
    }

    std::printf("[远端减速] 上限=%.2f, 原因=%s, pure_angle=%.1f\n",
                cap,
                (reasons & kRemoteSpeedCapReasonCloth) ? "色布" : "",
                pure_angle);
    g_remote_speed_cap_log_active = true;
    g_remote_speed_cap_log_value = cap;
    g_remote_speed_cap_log_reasons = reasons;
}

const char* circle_gate_reason_text(bool circle_candidate, bool hold_active)
{
    if (circle_state == CircleState::CIRCLE_RUNNING)
    {
        return "circle_running";
    }
    if (circle_candidate)
    {
        return "circle_candidate";
    }
    switch (circle_state)
    {
    case CircleState::CIRCLE_BEGIN: return "circle_begin";
    case CircleState::CIRCLE_IN: return "circle_in";
    case CircleState::CIRCLE_OUT: return "circle_out";
    case CircleState::CIRCLE_END: return "circle_end";
    default: return hold_active ? "post_hold" : "normal";
    }
}

void reset_remote_recognition_runtime(bool reset_follow_mode)
{
    g_remote_recognition.last_seq_valid = false;
    g_remote_recognition.last_seq = 0;
    g_remote_recognition.current_code = BoardVisionCode::UNKNOWN;
    g_remote_recognition.last_rx_ms = 0;
    g_remote_recognition.vehicle_hold_until_ms = 0;
    g_remote_recognition.vehicle_hold_yaw = 0.0f;
    g_remote_recognition.follow_state = remote_follow_state_t::NONE;
    g_remote_recognition.brick_block_until_ms = 0;
    image_remote_recognition_set_follow_path_state(false, false);
    if (reset_follow_mode)
    {
        follow_mode = FollowLine::MIXED;
    }
}

} // namespace

bool zebra_stop = false;
bool zebra_rush_active = false;
bool zebra_special_state_locked = false;
float zebra_speed_ratio_override = 1.0f;

pts_well_processed pts_left;
pts_well_processed pts_right;
pts_well_processed pts_far_left;
pts_well_processed pts_far_right;

bool if_find_far_line = false;

void image_reset_far_line_state()
{
    if_find_far_line = false;
    reset_pts(pts_far_left);
    reset_pts(pts_far_right);
}

midline_data midline;

void image_reset_midline_path_state()
{
    midline.mid_count = 0;
    midline.preview_curve_split_index = -1;
    midline.path_count = 0;
}

FollowLine follow_mode = FollowLine::MIXED;

ElementType element_type = ElementType::NORMAL;
CircleState circle_state = CircleState::CIRCLE_NONE;
CircleDirection circle_direction = CircleDirection::CIRCLE_DIR_NONE;
CrossingState crossing_state = CrossingState::CROSSING_NONE;

float pure_angle = 0.0f;
float circle_average_angle = 0.0f;
float preview_curve_angle_deg = 0.0f;
float preview_img_y = (float)PUREANGLE_PREVIEW_BASE_IMAGE_Y;

void image_reset_tracking_observation_state()
{
    pure_angle = 0.0f;
    preview_curve_angle_deg = 0.0f;
    preview_img_y = (float)PUREANGLE_PREVIEW_BASE_IMAGE_Y;
}

track_debug_status g_track_debug = {0, 0, 0};

void image_remote_recognition_reset()
{
    reset_remote_recognition_runtime(true);
}

void image_circle_recognition_gate_update(bool circle_candidate, uint64_t t_ms)
{
    if (BW_CIRCLE_RECOGNITION_GATE_ENABLE == 0)
    {
        g_circle_recognition_gate_hold_until_ms = 0;
        g_circle_recognition_gate_blocked.store(false);
        return;
    }

    bool blocked = false;
    if (circle_state == CircleState::CIRCLE_RUNNING)
    {
        g_circle_recognition_gate_hold_until_ms = 0;
    }
    else
    {
        const bool circle_phase_block =
            circle_state == CircleState::CIRCLE_BEGIN ||
            circle_state == CircleState::CIRCLE_IN ||
            circle_state == CircleState::CIRCLE_OUT ||
            circle_state == CircleState::CIRCLE_END;
        if (circle_candidate || circle_phase_block)
        {
            g_circle_recognition_gate_hold_until_ms =
                t_ms + static_cast<uint64_t>(BW_CIRCLE_RECOGNITION_GATE_HOLD_MS);
        }
        blocked = circle_candidate ||
                  circle_phase_block ||
                  t_ms < g_circle_recognition_gate_hold_until_ms;
    }

    const bool previous = g_circle_recognition_gate_blocked.exchange(blocked);
    if (blocked)
    {
        const bool reset_follow_mode =
            element_type == ElementType::NORMAL &&
            circle_state == CircleState::CIRCLE_NONE &&
            crossing_state == CrossingState::CROSSING_NONE;
        reset_remote_recognition_runtime(reset_follow_mode);
    }

    if (previous != blocked)
    {
        const bool hold_active =
            !circle_candidate &&
            circle_state == CircleState::CIRCLE_NONE &&
            t_ms < g_circle_recognition_gate_hold_until_ms;
        std::printf("[环岛识别门控] %s, 原因=%s\n",
                    blocked ? "BLOCK" : "ALLOW",
                    circle_gate_reason_text(circle_candidate, hold_active));
    }
}

bool image_circle_recognition_gate_is_blocked()
{
    return BW_CIRCLE_RECOGNITION_GATE_ENABLE != 0 &&
           g_circle_recognition_gate_blocked.load();
}

void image_remote_recognition_apply_state(BoardVisionCode code,
                                          uint8_t seq,
                                          float current_pure_angle,
                                          uint64_t t_ms)
{
    if (code == BoardVisionCode::INVALID || image_circle_recognition_gate_is_blocked())
    {
        return;
    }

    g_remote_recognition.last_rx_ms = t_ms;

    if (g_remote_recognition.last_seq_valid && g_remote_recognition.last_seq == seq)
    {
        if (code == g_remote_recognition.current_code &&
            (code == BoardVisionCode::VEHICLE ||
             code == BoardVisionCode::NO_RESULT))
        {
            remote_vehicle_route_apply(current_pure_angle, t_ms);
        }
        return;
    }

    g_remote_recognition.last_seq_valid = true;
    g_remote_recognition.last_seq = seq;

    const BoardVisionCode previous_code = g_remote_recognition.current_code;
    g_remote_recognition.current_code = code;
    const bool edge_follow_code =
        code == BoardVisionCode::WEAPON || code == BoardVisionCode::SUPPLY;
    if (!edge_follow_code || code != previous_code)
    {
        image_remote_recognition_set_follow_path_state(false, false);
    }

    if (code == BoardVisionCode::UNKNOWN)
    {
        g_remote_recognition.follow_state = remote_follow_state_t::NONE;
        g_remote_recognition.vehicle_hold_until_ms = 0;
        g_remote_recognition.vehicle_hold_yaw = 0.0f;
        g_remote_recognition.brick_block_until_ms = 0;
        follow_mode = FollowLine::MIXED;
        return;
    }

    if (is_remote_brick_code(code))
    {
        g_remote_recognition.follow_state = remote_follow_state_t::NONE;
        g_remote_recognition.vehicle_hold_until_ms = 0;
        g_remote_recognition.vehicle_hold_yaw = 0.0f;
        follow_mode = FollowLine::MIXED;
        if (!is_remote_brick_code(previous_code) &&
            (circle_state == CircleState::CIRCLE_BEGIN ||
             circle_state == CircleState::CIRCLE_IN))
        {
            g_remote_recognition.brick_block_until_ms = t_ms + kRemoteBrickBlockMs;
        }
        return;
    }

    if (code == BoardVisionCode::CLOTH_STOP)
    {
        g_remote_recognition.follow_state = remote_follow_state_t::NONE;
        g_remote_recognition.vehicle_hold_until_ms = 0;
        g_remote_recognition.vehicle_hold_yaw = 0.0f;
        g_remote_recognition.brick_block_until_ms = 0;
        follow_mode = FollowLine::MIXED;
        return;
    }

    if (code == BoardVisionCode::WEAPON)
    {
        g_remote_recognition.follow_state = remote_follow_state_t::LEFT_EDGE_ROUTE;
        g_remote_recognition.vehicle_hold_until_ms = 0;
        g_remote_recognition.vehicle_hold_yaw = 0.0f;
        g_remote_recognition.brick_block_until_ms = 0;
        return;
    }

    if (code == BoardVisionCode::SUPPLY)
    {
        g_remote_recognition.follow_state = remote_follow_state_t::RIGHT_EDGE_ROUTE;
        g_remote_recognition.vehicle_hold_until_ms = 0;
        g_remote_recognition.vehicle_hold_yaw = 0.0f;
        g_remote_recognition.brick_block_until_ms = 0;
        return;
    }

    if (code == BoardVisionCode::VEHICLE)
    {
        g_remote_recognition.brick_block_until_ms = 0;
        remote_vehicle_route_apply(current_pure_angle, t_ms);
        return;
    }

    if (code == BoardVisionCode::NO_RESULT)
    {
        g_remote_recognition.brick_block_until_ms = 0;
        remote_vehicle_route_apply(current_pure_angle, t_ms);
        return;
    }

    g_remote_recognition.follow_state = remote_follow_state_t::NONE;
    g_remote_recognition.vehicle_hold_until_ms = 0;
    g_remote_recognition.vehicle_hold_yaw = 0.0f;
    g_remote_recognition.brick_block_until_ms = 0;
}

void image_remote_recognition_tick(uint64_t t_ms)
{
    if (g_remote_recognition.last_rx_ms > 0 &&
        t_ms >= g_remote_recognition.last_rx_ms + static_cast<uint64_t>(BW_REMOTE_STATE_STALE_MS))
    {
        image_remote_recognition_reset();
        return;
    }
}

bool image_remote_recognition_try_get_hold_yaw(uint64_t t_ms, float* hold_yaw)
{
    if (image_circle_recognition_gate_is_blocked() ||
        hold_yaw == nullptr ||
        t_ms >= g_remote_recognition.vehicle_hold_until_ms)
    {
        return false;
    }

    *hold_yaw = g_remote_recognition.vehicle_hold_yaw;
    return true;
}

float image_remote_recognition_get_speed_ratio_override()
{
    const bool u_slowdown_active =
        !image_circle_recognition_gate_is_blocked() &&
        (BW_REMOTE_U_SLOWDOWN_ENABLE != 0) &&
        g_remote_recognition.current_code == BoardVisionCode::NO_RESULT;
    const float ratio = u_slowdown_active
        ? std::max(0.0f, std::min(BW_REMOTE_U_SLOWDOWN_RATIO, 1.0f))
        : 1.0f;

    static bool last_active = false;
    static float last_ratio = 1.0f;
    if (u_slowdown_active != last_active || std::fabs(ratio - last_ratio) >= 0.01f)
    {
        if (u_slowdown_active)
        {
            std::printf("[远端减速] u基础速度倍率=%.2f\n", ratio);
        }
        else if (last_active)
        {
            std::printf("[远端减速] u基础速度倍率解除\n");
        }
        last_active = u_slowdown_active;
        last_ratio = ratio;
    }
    return ratio;
}

bool image_remote_recognition_get_speed_cap_override(float* out_cap)
{
    if (out_cap == nullptr)
    {
        return false;
    }

    bool has_cap = false;
    float best_cap = 0.0f;
    uint32_t reasons = 0;

    add_remote_speed_cap_candidate(
        !image_circle_recognition_gate_is_blocked() &&
        g_remote_recognition.current_code == BoardVisionCode::CLOTH_STOP,
        BW_REMOTE_CLOTH_STOP_SPEED_CAP,
        kRemoteSpeedCapReasonCloth,
        &has_cap,
        &best_cap,
        &reasons);
    print_remote_speed_cap_log(has_cap, best_cap, reasons);
    if (!has_cap)
    {
        return false;
    }

    *out_cap = best_cap;
    return true;
}

bool image_remote_recognition_is_vehicle_active(uint64_t t_ms)
{
    return !image_circle_recognition_gate_is_blocked() &&
           (g_remote_recognition.follow_state == remote_follow_state_t::VEHICLE_ROUTE) &&
           (t_ms < g_remote_recognition.vehicle_hold_until_ms);
}

bool image_remote_recognition_should_freeze_state_machine(uint64_t t_ms)
{
    if (image_circle_recognition_gate_is_blocked())
    {
        return false;
    }

    if (g_remote_recognition.follow_state == remote_follow_state_t::LEFT_EDGE_ROUTE ||
        g_remote_recognition.follow_state == remote_follow_state_t::RIGHT_EDGE_ROUTE)
    {
        return true;
    }

    return g_remote_recognition.current_code == BoardVisionCode::VEHICLE &&
           image_remote_recognition_is_vehicle_active(t_ms);
}

bool image_remote_recognition_get_forced_follow_mode(FollowLine* out_mode)
{
    if (image_circle_recognition_gate_is_blocked() || out_mode == nullptr)
    {
        return false;
    }

    if (g_remote_recognition.follow_state == remote_follow_state_t::LEFT_EDGE_ROUTE)
    {
        *out_mode = FollowLine::MIDLEFT;
        return true;
    }

    if (g_remote_recognition.follow_state == remote_follow_state_t::RIGHT_EDGE_ROUTE)
    {
        *out_mode = FollowLine::MIDRIGHT;
        return true;
    }

    return false;
}

void image_remote_recognition_set_follow_path_state(bool active, bool inner_follow)
{
    if (!active && g_remote_follow_speed_log_active)
    {
        std::printf("[绕行平均速度] 解除\n");
        g_remote_follow_speed_log_active = false;
        g_remote_follow_speed_log_inner = false;
        g_remote_follow_speed_log_released = false;
        g_remote_follow_speed_log_ratio = 1.0f;
    }
    g_remote_recognition.follow_path_active = active;
    g_remote_recognition.inner_follow_active = active && inner_follow;
}

bool image_remote_recognition_get_follow_average_speed_ratio(float* out_ratio)
{
    if (image_circle_recognition_gate_is_blocked() ||
        out_ratio == nullptr ||
        !g_remote_recognition.follow_path_active)
    {
        return false;
    }

    float ratio = g_remote_recognition.inner_follow_active
        ? BW_REMOTE_FOLLOW_INNER_AVERAGE_SPEED_RATIO
        : BW_REMOTE_FOLLOW_OUTER_AVERAGE_SPEED_RATIO;
    ratio = std::max(0.0f, std::min(ratio, 1.0f));
    const bool small_angle_released =
        std::fabs(pure_angle) <= BW_REMOTE_FOLLOW_SPEED_RELEASE_ANGLE_DEG;
    if (small_angle_released)
    {
        ratio = 1.0f;
    }

    if (!g_remote_follow_speed_log_active ||
        g_remote_follow_speed_log_inner != g_remote_recognition.inner_follow_active ||
        g_remote_follow_speed_log_released != small_angle_released ||
        std::fabs(g_remote_follow_speed_log_ratio - ratio) >= 0.01f)
    {
        std::printf("[绕行平均速度] %s倍率=%.2f%s, pure_angle=%.1f\n",
                    g_remote_recognition.inner_follow_active ? "内绕" : "外绕",
                    ratio,
                    small_angle_released ? "（小角解除）" : "",
                    pure_angle);
    }
    g_remote_follow_speed_log_active = true;
    g_remote_follow_speed_log_inner = g_remote_recognition.inner_follow_active;
    g_remote_follow_speed_log_released = small_angle_released;
    g_remote_follow_speed_log_ratio = ratio;

    *out_ratio = ratio;
    return true;
}

bool image_remote_recognition_should_block_circle(uint64_t t_ms)
{
    return !image_circle_recognition_gate_is_blocked() &&
           t_ms < g_remote_recognition.brick_block_until_ms;
}

bool image_remote_recognition_get_brick_avoid_shift_direction(int* out_direction)
{
    if (image_circle_recognition_gate_is_blocked() || out_direction == nullptr)
    {
        return false;
    }

    if (g_remote_recognition.current_code == BoardVisionCode::BRICK_LEFT)
    {
        *out_direction = 1;
        return true;
    }

    if (g_remote_recognition.current_code == BoardVisionCode::BRICK_RIGHT)
    {
        *out_direction = -1;
        return true;
    }

    return false;
}
