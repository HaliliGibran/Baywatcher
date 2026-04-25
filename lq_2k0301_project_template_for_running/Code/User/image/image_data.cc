#include "image_data.h"

namespace {

constexpr uint64_t kRemoteVehicleHoldMs = static_cast<uint64_t>(BW_REMOTE_VEHICLE_HOLD_MS);

enum class remote_follow_state_t : uint8_t
{
    NONE = 0,
    VEHICLE_ROUTE,
    LEFT_EDGE_ROUTE,
    RIGHT_EDGE_ROUTE,
};

enum class remote_aggressive_turn_state_t : uint8_t
{
    NONE = 0,
    FORCE_LEFT_30,
    FORCE_RIGHT_30,
};

struct remote_recognition_runtime_t
{
    bool last_seq_valid;
    uint8_t last_seq;
    BoardVisionCode current_code;
    uint64_t vehicle_hold_until_ms;
    float vehicle_hold_yaw;
    remote_follow_state_t follow_state;
    remote_aggressive_turn_state_t aggressive_turn_state;
    bool block_circle_until_n;
};

remote_recognition_runtime_t g_remote_recognition = {
    false,
    0,
    BoardVisionCode::UNKNOWN,
    0,
    0.0f,
    remote_follow_state_t::NONE,
    remote_aggressive_turn_state_t::NONE,
    false,
};

} // namespace

bool zebra_stop = false;

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
    g_remote_recognition.vehicle_hold_until_ms = 0;
    g_remote_recognition.vehicle_hold_yaw = 0.0f;
    g_remote_recognition.follow_state = remote_follow_state_t::NONE;
    g_remote_recognition.aggressive_turn_state = remote_aggressive_turn_state_t::NONE;
    g_remote_recognition.block_circle_until_n = false;
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

    if (g_remote_recognition.last_seq_valid && g_remote_recognition.last_seq == seq)
    {
        return;
    }

    g_remote_recognition.last_seq_valid = true;
    g_remote_recognition.last_seq = seq;

    g_remote_recognition.current_code = code;

    if (code == BoardVisionCode::UNKNOWN)
    {
        g_remote_recognition.follow_state = remote_follow_state_t::NONE;
        g_remote_recognition.aggressive_turn_state = remote_aggressive_turn_state_t::NONE;
        g_remote_recognition.vehicle_hold_until_ms = 0;
        g_remote_recognition.vehicle_hold_yaw = 0.0f;
        g_remote_recognition.block_circle_until_n = false;
        follow_mode = FollowLine::MIXED;
        return;
    }

    if (code == BoardVisionCode::BRICK)
    {
        g_remote_recognition.follow_state = remote_follow_state_t::NONE;
        g_remote_recognition.aggressive_turn_state = remote_aggressive_turn_state_t::NONE;
        g_remote_recognition.vehicle_hold_until_ms = 0;
        g_remote_recognition.vehicle_hold_yaw = 0.0f;
        if (circle_state == CircleState::CIRCLE_BEGIN ||
            circle_state == CircleState::CIRCLE_IN)
        {
            g_remote_recognition.block_circle_until_n = true;
        }
        return;
    }

    if (code == BoardVisionCode::WEAPON)
    {
        g_remote_recognition.follow_state = remote_follow_state_t::LEFT_EDGE_ROUTE;
        g_remote_recognition.aggressive_turn_state = remote_aggressive_turn_state_t::FORCE_LEFT_30;
        g_remote_recognition.vehicle_hold_until_ms = 0;
        g_remote_recognition.vehicle_hold_yaw = 0.0f;
        return;
    }

    if (code == BoardVisionCode::SUPPLY)
    {
        g_remote_recognition.follow_state = remote_follow_state_t::RIGHT_EDGE_ROUTE;
        g_remote_recognition.aggressive_turn_state = remote_aggressive_turn_state_t::FORCE_RIGHT_30;
        g_remote_recognition.vehicle_hold_until_ms = 0;
        g_remote_recognition.vehicle_hold_yaw = 0.0f;
        return;
    }

    if (code == BoardVisionCode::VEHICLE ||
        code == BoardVisionCode::NO_RESULT)
    {
        g_remote_recognition.follow_state = remote_follow_state_t::VEHICLE_ROUTE;
        g_remote_recognition.aggressive_turn_state = remote_aggressive_turn_state_t::NONE;
        g_remote_recognition.vehicle_hold_yaw = current_pure_angle;
        g_remote_recognition.vehicle_hold_until_ms = t_ms + kRemoteVehicleHoldMs;
        return;
    }

    g_remote_recognition.follow_state = remote_follow_state_t::NONE;
    g_remote_recognition.aggressive_turn_state = remote_aggressive_turn_state_t::NONE;
    g_remote_recognition.vehicle_hold_until_ms = 0;
    g_remote_recognition.vehicle_hold_yaw = 0.0f;
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

bool image_remote_recognition_is_vehicle_active(uint64_t t_ms)
{
    return (g_remote_recognition.follow_state == remote_follow_state_t::VEHICLE_ROUTE) &&
           (t_ms < g_remote_recognition.vehicle_hold_until_ms);
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

bool image_remote_recognition_get_aggressive_turn_override(float raw_pure_angle, float* out_override)
{
    if (out_override == nullptr ||
        BW_REMOTE_SIGN_AGGRESSIVE_TURN_ENABLE == 0 ||
        BW_REMOTE_SIGN_AGGRESSIVE_ABS_PURE_ANGLE <= 0.0f)
    {
        return false;
    }

    if (g_remote_recognition.aggressive_turn_state == remote_aggressive_turn_state_t::FORCE_LEFT_30)
    {
        if (raw_pure_angle < 0.0f)
        {
            g_remote_recognition.aggressive_turn_state = remote_aggressive_turn_state_t::NONE;
            return false;
        }

        *out_override = BW_REMOTE_SIGN_AGGRESSIVE_ABS_PURE_ANGLE;
        return true;
    }

    if (g_remote_recognition.aggressive_turn_state == remote_aggressive_turn_state_t::FORCE_RIGHT_30)
    {
        if (raw_pure_angle > 0.0f)
        {
            g_remote_recognition.aggressive_turn_state = remote_aggressive_turn_state_t::NONE;
            return false;
        }

        *out_override = -BW_REMOTE_SIGN_AGGRESSIVE_ABS_PURE_ANGLE;
        return true;
    }

    return false;
}

bool image_remote_recognition_should_block_circle()
{
    return g_remote_recognition.block_circle_until_n;
}
