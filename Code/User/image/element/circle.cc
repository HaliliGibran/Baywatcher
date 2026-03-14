#include "image_headfile.h"
#include "element/circle.h"

#include <cstdio>

// 功能: 环岛状态转字符串（日志用）
// 类型: 局部功能函数
// 关键参数: s-环岛状态
static inline const char* circle_state_str(CircleState s)
{
    switch (s)
    {
        case CircleState::CIRCLE_NONE: return "NONE";
        case CircleState::CIRCLE_BEGIN: return "BEGIN";
        case CircleState::CIRCLE_IN: return "IN";
        case CircleState::CIRCLE_RUNNING: return "RUNNING";
        case CircleState::CIRCLE_OUT: return "OUT";
        case CircleState::CIRCLE_END: return "END";
        default: return "UNKNOWN";
    }
}

// 功能: 环岛方向转字符串（日志用）
// 类型: 局部功能函数
// 关键参数: d-环岛方向
static inline const char* circle_dir_str_local(CircleDirection d)
{
    switch (d)
    {
        case CircleDirection::CIRCLE_DIR_NONE: return "NONE";
        case CircleDirection::CIRCLE_DIR_LEFT: return "LEFT";
        case CircleDirection::CIRCLE_DIR_RIGHT: return "RIGHT";
        default: return "UNKNOWN";
    }
}

// 功能: 环岛方向转中文（日志用）
// 类型: 局部功能函数
// 关键参数: d-环岛方向
static inline const char* circle_dir_cn(CircleDirection d)
{
    switch (d)
    {
        case CircleDirection::CIRCLE_DIR_LEFT: return "左";
        case CircleDirection::CIRCLE_DIR_RIGHT: return "右";
        default: return "无";
    }
}

namespace
{
// 作用域: 匿名命名空间内结构体，环岛单侧计数器
struct SideCounter
{
    int lost_count = 0;
    int found_count = 0;
    bool lost_flag = false;
    int run_to_out_counter = 0;
};

// 功能: 重置环岛单侧计数器
// 类型: 局部功能函数
// 关键参数: c-计数器对象
static void reset_counter(SideCounter& c)
{
    c.lost_count = 0;
    c.found_count = 0;
    c.lost_flag = false;
    c.run_to_out_counter = 0;
}

// 功能: BEGIN 阶段计数更新（丢线->找线）
// 类型: 局部功能函数
// 关键参数: outer_line_missing-外侧边线是否丢失
static void update_begin_phase(SideCounter& c, bool outer_line_missing)
{
    if (outer_line_missing)
    {
        c.found_count = 0;
        c.lost_count++;
        if (c.lost_count >= FRAME_THRESHOLD_roundabout_begin_to_in_lost_line_counter)
        {
            c.lost_flag = true;
        }
    }
    else
    {
        c.lost_count = 0;
        if (c.lost_flag)
        {
            c.found_count++;
        }
    }
}

// 功能: 判断是否满足 BEGIN->IN 条件
// 类型: 局部功能函数
// 关键参数: c-计数器对象
static bool begin_to_in_ready(const SideCounter& c)
{
    return c.lost_flag && (c.found_count >= FRAME_THRESHOLD_roundabout_begin_to_in_found_line_counter);
}

// 功能: END 阶段计数更新（丢线->找线）
// 类型: 局部功能函数
// 关键参数: outer_line_missing-外侧边线是否丢失
static void update_end_phase(SideCounter& c, bool outer_line_missing)
{
    // 与 begin 同理：先连续丢线，再连续找线才算退出环岛
    if (outer_line_missing)
    {
        c.found_count = 0;
        c.lost_count++;
        if (c.lost_count >= FRAME_THRESHOLD_roundabout_end_lost_line_counter)
        {
            c.lost_flag = true;
        }
    }
    else
    {
        c.lost_count = 0;
        if (c.lost_flag)
        {
            c.found_count++;
        }
    }
}

// 功能: 判断是否满足 END->NONE 条件
// 类型: 局部功能函数
// 关键参数: c-计数器对象
static bool end_to_none_ready(const SideCounter& c)
{
    return c.lost_flag && (c.found_count >= FRAME_THRESHOLD_roundabout_end_found_line_counter);
}

// 作用域: 匿名命名空间内静态变量，左/右环岛计数器
static SideCounter g_left_c;
static SideCounter g_right_c;
} // namespace

// 功能: 环岛状态机复位
// 类型: 图像处理函数
// 关键参数: 无
void roundabout_reset()
{
    circle_state = CircleState::CIRCLE_NONE;
    circle_direction = CircleDirection::CIRCLE_DIR_NONE;
    follow_mode = FollowLine::MIXED;
    reset_counter(g_left_c);
    reset_counter(g_right_c);
}


// 功能: 环岛状态机更新（更新 follow_mode/circle_state）
// 类型: 图像处理函数
// 关键参数: 无（使用全局 pts_left/pts_right 等）
void roundabout_update()
{
    const CircleState prev_state = circle_state;
    const CircleDirection prev_dir = circle_direction;

    // 不在环岛元素时，复位环岛状态机
    if (element_type != ElementType::CIRCLE)
    {
        circle_state = CircleState::CIRCLE_NONE;
        circle_direction = CircleDirection::CIRCLE_DIR_NONE;
        reset_counter(g_left_c);
        reset_counter(g_right_c);

        if (circle_state != prev_state || circle_direction != prev_dir)
        {
            std::printf("[TRACK] %s %s->%s\r\n"
                        "vote(cross=%d circle=%d)\r\n"
                        "protect=%d\r\n",
                        circle_dir_cn(circle_direction),
                        circle_state_str(prev_state), circle_state_str(circle_state),
                        g_track_debug.crossing_vote, g_track_debug.circle_vote, g_track_debug.protect);
        }
        return;
    }

    // 方向未知时不推进（由 element_detect 赋值）
    if (circle_direction != CircleDirection::CIRCLE_DIR_LEFT && circle_direction != CircleDirection::CIRCLE_DIR_RIGHT)
    {
        follow_mode = FollowLine::MIXED;

        if (circle_state != prev_state || circle_direction != prev_dir)
        {
            std::printf("[TRACK] %s %s->%s\r\n"
                        "vote(cross=%d circle=%d)\r\n"
                        "protect=%d\r\n",
                        circle_dir_cn(circle_direction),
                        circle_state_str(prev_state), circle_state_str(circle_state),
                        g_track_debug.crossing_vote, g_track_debug.circle_vote, g_track_debug.protect);
        }
        return;
    }

    const bool is_right = (circle_direction == CircleDirection::CIRCLE_DIR_RIGHT);
    SideCounter& c = is_right ? g_right_c : g_left_c;

    switch (circle_state)
    {
        case CircleState::CIRCLE_BEGIN:
        {
            // BEGIN：贴外侧跟线（右环岛跟左中线，左环岛跟右中线）
            follow_mode = is_right ? FollowLine::MIDLEFT : FollowLine::MIDRIGHT;

            const bool outer_missing = is_right ? (pts_right.pts_count <= 0) : (pts_left.pts_count <= 0);
            update_begin_phase(c, outer_missing);
            if (begin_to_in_ready(c))
            {
                circle_state = CircleState::CIRCLE_IN;
                reset_counter(c);
            }
            break;
        }

        case CircleState::CIRCLE_IN:
        {
            // IN：切到内侧中线（右环岛跟右中线，左环岛跟左中线）
            follow_mode = is_right ? FollowLine::MIDRIGHT : FollowLine::MIDLEFT;

            // 入环阶段：观察对侧是否出现“弯曲边线”，作为进入环内 RUNNING 的标志
            const bool opposite_curve = is_right ? (pts_left.is_curve != 0) : (pts_right.is_curve != 0);
            if (opposite_curve)
            {
                circle_state = CircleState::CIRCLE_RUNNING;
                c.run_to_out_counter = 0;
            }
            break;
        }

        case CircleState::CIRCLE_RUNNING:
        {
            // RUNNING：贴外侧更稳
            follow_mode = is_right ? FollowLine::MIDLEFT : FollowLine::MIDRIGHT;

            // 环内到出环：检测到“对侧角点”连续出现
            const bool opposite_corner = is_right ? pts_left.corner_found : pts_right.corner_found;
            if (opposite_corner)
            {
                c.run_to_out_counter++;
                if (c.run_to_out_counter >= FRAME_THRESHOLD_roundabout_running_to_out_corner_counter)
                {
                    circle_state = CircleState::CIRCLE_OUT;
                    c.run_to_out_counter = 0;
                }
            }
            else
            {
                c.run_to_out_counter = 0;
            }
            break;
        }

        case CircleState::CIRCLE_OUT:
        {
            // OUT：切到内侧中线准备出环
            follow_mode = is_right ? FollowLine::MIDRIGHT : FollowLine::MIDLEFT;

            const bool opposite_straight = is_right ? pts_left.is_straight : pts_right.is_straight;
            if (opposite_straight)
            {
                circle_state = CircleState::CIRCLE_END;
                reset_counter(c);
            }
            break;
        }

        case CircleState::CIRCLE_END:
        {
            // END：回到外侧跟线，等待“丢线->找线”完成退出
            follow_mode = is_right ? FollowLine::MIDLEFT : FollowLine::MIDRIGHT;

            const bool outer_missing = is_right ? (pts_right.pts_count <= 0) : (pts_left.pts_count <= 0);
            update_end_phase(c, outer_missing);
            if (end_to_none_ready(c))
            {
                reset_counter(c);
                circle_state = CircleState::CIRCLE_NONE;
                circle_direction = CircleDirection::CIRCLE_DIR_NONE;
            }
            break;
        }

        default:
        {
            circle_state = CircleState::CIRCLE_BEGIN;
            reset_counter(c);
            follow_mode = FollowLine::MIXED;
            break;
        }
    }

    if (circle_state != prev_state || circle_direction != prev_dir)
    {
        std::printf("[TRACK] %s %s->%s\r\n"
                    "vote(cross=%d circle=%d)\r\n"
                    "protect=%d\r\n",
                    circle_dir_cn(circle_direction),
                    circle_state_str(prev_state), circle_state_str(circle_state),
                    g_track_debug.crossing_vote, g_track_debug.circle_vote, g_track_debug.protect);
    }
}
