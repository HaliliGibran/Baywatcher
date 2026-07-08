#include "image_data.h"

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
};

void remote_vehicle_route_apply(float current_pure_angle, uint64_t t_ms)
{
    g_remote_recognition.follow_state = remote_follow_state_t::VEHICLE_ROUTE;
    g_remote_recognition.vehicle_hold_yaw = current_pure_angle;
    g_remote_recognition.vehicle_hold_until_ms = t_ms + kRemoteVehicleHoldMs;
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
    g_remote_recognition.last_seq_valid = false;
    g_remote_recognition.last_seq = 0;
    g_remote_recognition.current_code = BoardVisionCode::UNKNOWN;
    g_remote_recognition.last_rx_ms = 0;
    g_remote_recognition.vehicle_hold_until_ms = 0;
    g_remote_recognition.vehicle_hold_yaw = 0.0f;
    g_remote_recognition.follow_state = remote_follow_state_t::NONE;
    g_remote_recognition.brick_block_until_ms = 0;
    follow_mode = FollowLine::MIXED;
}

void image_remote_recognition_apply_state(BoardVisionCode code,
                                          uint8_t seq,
                                          float current_pure_angle,
                                          uint64_t t_ms)
{
    if (code == BoardVisionCode::INVALID)
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

    if (code == BoardVisionCode::UNKNOWN)
    {
        g_remote_recognition.follow_state = remote_follow_state_t::NONE;
        g_remote_recognition.vehicle_hold_until_ms = 0;
        g_remote_recognition.vehicle_hold_yaw = 0.0f;
        g_remote_recognition.brick_block_until_ms = 0;
        follow_mode = FollowLine::MIXED;
        return;
    }

    if (code == BoardVisionCode::BRICK)
    {
        g_remote_recognition.follow_state = remote_follow_state_t::NONE;
        g_remote_recognition.vehicle_hold_until_ms = 0;
        g_remote_recognition.vehicle_hold_yaw = 0.0f;
        follow_mode = FollowLine::MIXED;
        if (previous_code != BoardVisionCode::BRICK &&
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
    if (hold_yaw == nullptr || t_ms >= g_remote_recognition.vehicle_hold_until_ms)
    {
        return false;
    }

    *hold_yaw = g_remote_recognition.vehicle_hold_yaw;
    return true;
}

float image_remote_recognition_get_speed_ratio_override()
{
    if (g_remote_recognition.current_code == BoardVisionCode::VEHICLE)
    {
        return 1.0f;
    }

    if (g_remote_recognition.current_code == BoardVisionCode::NO_RESULT)
    {
        return BW_REMOTE_U_SLOWDOWN_RATIO;
    }

    if (g_remote_recognition.current_code == BoardVisionCode::CLOTH_STOP)
    {
        return BW_REMOTE_CLOTH_STOP_SPEED_CAP;
    }

    return 1.0f;
}

bool image_remote_recognition_get_speed_cap_override(float* out_cap)
{
    if (out_cap == nullptr)
    {
        return false;
    }

    if (g_remote_recognition.current_code == BoardVisionCode::NO_RESULT)
    {
        *out_cap = BW_REMOTE_U_SLOWDOWN_RATIO;
        if (*out_cap < 0.0f)
        {
            *out_cap = 0.0f;
        }
        return true;
    }

    if (g_remote_recognition.current_code == BoardVisionCode::CLOTH_STOP)
    {
        *out_cap = BW_REMOTE_CLOTH_STOP_SPEED_CAP;
        if (*out_cap < 0.0f)
        {
            *out_cap = 0.0f;
        }
        return true;
    }

    return false;
}

bool image_remote_recognition_is_vehicle_active(uint64_t t_ms)
{
    return (g_remote_recognition.follow_state == remote_follow_state_t::VEHICLE_ROUTE) &&
           (t_ms < g_remote_recognition.vehicle_hold_until_ms);
}

bool image_remote_recognition_should_freeze_state_machine(uint64_t t_ms)
{
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
    if (out_mode == nullptr)
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

bool image_remote_recognition_should_block_circle(uint64_t t_ms)
{
    return t_ms < g_remote_recognition.brick_block_until_ms;
}
