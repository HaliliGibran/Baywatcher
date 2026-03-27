#include "image_headfile.h"

#include <atomic>
#include <cstdio>

// 作用域: 文件内静态，全局“强制复位”请求（由 track_force_reset 置位）
static std::atomic<bool> g_track_force_reset{false};

namespace {

// 功能: 把公开元素运行态收回到普通赛道
// 类型: 局部功能函数
// 关键参数:
// - reset_follow_mode-是否同步把 follow_mode 收回 MIXED
// - clear_far_line-是否清空十字远端线缓存
static void set_runtime_normal_state(bool reset_follow_mode, bool clear_far_line)
{
    element_type = ElementType::NORMAL;
    circle_state = CircleState::CIRCLE_NONE;
    crossing_state = CrossingState::CROSSING_NONE;
    circle_direction = CircleDirection::CIRCLE_DIR_NONE;
    if (reset_follow_mode)
    {
        follow_mode = FollowLine::MIXED;
    }
    if (clear_far_line)
    {
        image_reset_far_line_state();
    }
}

// 功能: 把公开元素运行态切到十字
// 类型: 局部功能函数
// 关键参数: 无
// 说明：这里只负责 owner 切换，不处理十字内部计数和补线细节。
static void enter_crossing_runtime_state()
{
    element_type = ElementType::CROSSING;
    if (crossing_state == CrossingState::CROSSING_NONE)
    {
        crossing_state = CrossingState::CROSSING_IN;
    }
    circle_state = CircleState::CIRCLE_NONE;
    circle_direction = CircleDirection::CIRCLE_DIR_NONE;
}

// 功能: 把公开元素运行态切到环岛
// 类型: 局部功能函数
// 关键参数: direction-当前判定到的环岛方向
// 说明：circle_state 只在首次进入时推进到 BEGIN，避免重复覆盖环岛内部阶段。
static void enter_circle_runtime_state(CircleDirection direction)
{
    element_type = ElementType::CIRCLE;
    if (circle_state == CircleState::CIRCLE_NONE)
    {
        circle_state = CircleState::CIRCLE_BEGIN;
    }
    crossing_state = CrossingState::CROSSING_NONE;
    circle_direction = direction;
}

} // namespace

// 功能: 清空元素公开运行态
// 类型: 全局功能函数
// 关键参数: reset_follow_mode-是否同步重置跟线模式
void track_reset_element_runtime_state(bool reset_follow_mode)
{
    set_runtime_normal_state(reset_follow_mode, true);
}

// 功能: 强制复位赛道元素状态（清空状态机与中线缓存）
// 类型: 全局功能函数
// 关键参数: 无
void track_force_reset()
{
    // 先把全局状态置回默认（立刻生效）
    set_runtime_normal_state(true, true);

    // 复位各状态机内部计数器
    roundabout_reset();
    crossing_reset();

    // 最后请求 element_detect 内部投票/保护帧等静态量清零
    g_track_force_reset.store(true);
}

// 功能: 元素类型转字符串（日志用）
// 类型: 局部功能函数
// 关键参数: t-元素类型
static inline const char* element_type_str(ElementType t)
{
    switch (t)
    {
        case ElementType::NORMAL: return "NORMAL";
        case ElementType::CIRCLE: return "CIRCLE";
        case ElementType::CROSSING: return "CROSSING";
        default: return "UNKNOWN";
    }
}

// 功能: 环岛方向转字符串（日志用）
// 类型: 局部功能函数
// 关键参数: d-环岛方向
static inline const char* circle_dir_str(CircleDirection d)
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

// 功能: 整型夹紧
// 类型: 局部功能函数
// 关键参数: v-输入值, lo/hi-范围
static inline int clamp_i32(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 功能: 取浮点绝对值
// 类型: 局部功能函数
// 关键参数: v-输入值
static inline float absf(float v)
{
    return (v >= 0.0f) ? v : -v;
}

// 功能: 判断某一窗口内是否为直线（基于曲率阈值）
// 类型: 局部功能函数
// 关键参数: ctx-边线上下文, center_id-中心索引, halfwin-窗口半宽
static bool window_is_straight(const pts_well_processed& ctx, int center_id, int halfwin)
{
    const int n = (ctx.curvature_num > PT_MAXLEN) ? PT_MAXLEN : ((ctx.curvature_num < 0) ? 0 : ctx.curvature_num);
    if (n <= 0)
    {
        return false;
    }

    // ANGLE_THRESHOLD_is_straight 在 common.h 里是“角度(度)”，这里换算到 1-cos(theta)
    static const float straight_thr = 1.0f - cosf((ANGLE_THRESHOLD_is_straight / 180.0f) * PI32);

    const int c = clamp_i32(center_id, 0, n - 1);
    const int s = clamp_i32(c - halfwin, 0, n - 1);
    const int e = clamp_i32(c + halfwin, 0, n - 1);
    int good = 0;
    const int need = ((e - s + 1) * 2) / 3; // 约 2/3 通过即可
    for (int i = s; i <= e; ++i)
    {
        if (ctx.curvature[i] < straight_thr)
        {
            ++good;
        }
    }
    return good >= need;
}

// 功能: 元素判定与状态机入口（投票 + 保护帧）
// 类型: 图像处理函数
// 关键参数: 无（使用全局 pts_left/pts_right 等）
void element_detect()
{
    // 可选输出
    const bool lcorner = pts_left.corner_found;
    const bool rcorner = pts_right.corner_found;
    // 通过“连续帧计数 + 保护帧”做抗抖，避免单帧误检导致 element_type 来回跳。
    static int crossing_vote = 0;
    static int circle_vote = 0;
    static int protect = 0;
    static ElementType last = ElementType::NORMAL;

    // 仅在“状态切换”时打印（避免刷屏）
    static ElementType log_last_type = ElementType::NORMAL;
    static CircleDirection log_last_dir = CircleDirection::CIRCLE_DIR_NONE;
    static bool log_crossing_lock = false;

    auto publish_debug = [&]()
    {
        g_track_debug.crossing_vote = crossing_vote;
        g_track_debug.circle_vote = circle_vote;
        g_track_debug.protect = protect;
    };

    auto log_element_if_changed = [&]()
    {
        if (element_type != log_last_type || circle_direction != log_last_dir)
        {
            std::printf("[TRACK] element %s->%s\r\n"
                        "dir=%s\r\n"
                        "vote(cross=%d circle=%d)\r\n"
                        "protect=%d\r\n",
                        element_type_str(log_last_type), element_type_str(element_type),
                        circle_dir_cn(circle_direction),
                        crossing_vote, circle_vote, protect);
            log_last_type = element_type;
            log_last_dir = circle_direction;
        }
    };

    // 手动复位（例如键盘输入 'c'）：清空投票/保护帧/锁定态，并把元素状态回到 NORMAL。
    if (g_track_force_reset.exchange(false))
    {
        set_runtime_normal_state(true, true);

        crossing_vote = 0;
        circle_vote = 0;
        protect = 0;
        last = ElementType::NORMAL;
        log_last_type = ElementType::NORMAL;
        log_last_dir = CircleDirection::CIRCLE_DIR_NONE;
        publish_debug();
        std::printf("[TRACK] reset NONE\r\nreason=key=c\r\n");
        return;
    }

    // 候选判定
    bool want_crossing = false;
    bool want_circle = false;
    CircleDirection want_circle_dir = CircleDirection::CIRCLE_DIR_NONE;

    // 十字：双角点
    if (lcorner && rcorner)
    {
        if (pts_left.corner_id <= ID_THRESHOLD_crossing_state_change &&
            pts_right.corner_id <= ID_THRESHOLD_crossing_state_change)
        {
            want_crossing = true;
        }
    }

    // 环岛：单侧角点 + 对侧在对应位置附近“更像直线”
    if (!want_crossing)
    {
        const int win = WINDOW_THRESHOLD_roundabout_opposite_straightness; // 窗口半宽（点数）
        if (lcorner && !rcorner)
        {
            want_circle = window_is_straight(pts_right, pts_left.corner_id, win);
            if (want_circle) want_circle_dir = CircleDirection::CIRCLE_DIR_LEFT;
        }
        else if (rcorner && !lcorner)
        {
            want_circle = window_is_straight(pts_left, pts_right.corner_id, win);
            if (want_circle) want_circle_dir = CircleDirection::CIRCLE_DIR_RIGHT;
        }
    }

    // 更新投票：
    if (want_crossing) crossing_vote = (crossing_vote < 1000) ? (crossing_vote + 1) : crossing_vote;
    else crossing_vote = (crossing_vote > 0) ? (crossing_vote - 1) : 0;

    if (want_circle) circle_vote = (circle_vote < 1000) ? (circle_vote + 1) : circle_vote;
    else circle_vote = (circle_vote > 0) ? (circle_vote - 1) : 0;

    // 保护帧倒计时：用于短暂抗抖（不影响“状态机正在运行”的锁定）
    if (protect > 0)
    {
        --protect;
    }

    // 误入复位检测：在环岛/十字状态时，如果两侧边线都判定为“大直线”，则必然不是环岛/十字，直接复位。
    // 注：这里使用 process_line 输出的 is_straight（整体判直道）而不是 window_is_straight（局部窗口），避免误判/迟滞。
    if ((last == ElementType::CIRCLE || last == ElementType::CROSSING) && pts_left.is_straight && pts_right.is_straight)
    {
        // 强制复位所有状态与计数
        set_runtime_normal_state(false, false);
        last = element_type;
        circle_vote = 0;
        crossing_vote = 0;
        protect = 0;
        publish_debug();
        std::printf("[TRACK] reset NONE\r\nreason=force_reset_straight\r\n"
                    "vote(cross=%d circle=%d)\r\nprotect=%d\r\n",
                    crossing_vote, circle_vote, protect);
        return;
    }

    // ====== 优先级锁定（低优先级不能覆盖高优先级正在运行的状态机） ======
    // 十字优先级最高：只要 crossing_state 还在推进，就禁止环岛/normal 覆盖。
    if (last == ElementType::CROSSING)
    {
        if (crossing_state != CrossingState::CROSSING_NONE || protect > 0)
        {
            element_type = ElementType::CROSSING;
            if (!log_crossing_lock)
            {
                log_crossing_lock = true;
                std::printf("[TRACK] crossing LOCK\r\n");
            }
            log_element_if_changed();
            return;
        }
        // 十字状态机已结束且无保护帧，允许降级
        last = ElementType::NORMAL;
    }

    // 环岛次优先级：只要 circle_state 还在推进，就禁止 normal 覆盖。
    if (last == ElementType::CIRCLE && !want_crossing)
    {
        if (circle_state != CircleState::CIRCLE_NONE || protect > 0)
        {
            element_type = ElementType::CIRCLE;
            return;
        }
        // 环岛状态机已结束且无保护帧，允许降级
        last = ElementType::NORMAL;
    }

    // 优先级判断：CROSSING > CIRCLE > NORMAL
    if (crossing_vote >= FRAME_THRESHOLD_roundabout_or_crossing_frame)
    {
        // 十字始终可覆盖其他状态
        enter_crossing_runtime_state();
        last = element_type;
        protect = FRAME_THRESHOLD_one_corner_crossing_protect_frame;
        circle_vote = 0;
        log_element_if_changed();
        return;
    }

    if (circle_vote >= FRAME_THRESHOLD_roundabout_or_crossing_frame)
    {
        // 环岛可以覆盖 NORMAL，但不能覆盖正在进行的十字（上面已处理）
        enter_circle_runtime_state(want_circle_dir);
        last = element_type;
        protect = FRAME_THRESHOLD_roundabout_protect_frame;
        crossing_vote = 0;
        log_element_if_changed();
        return;
    }

    // 否则为 NORMAL
    set_runtime_normal_state(false, false);
    last = element_type;
    log_element_if_changed();
}
