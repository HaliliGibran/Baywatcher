#include "image_headfile.h"

#include <atomic>
#include <cstdio>

// 作用域: 文件内静态，全局“强制复位”请求（由 track_force_reset 置位）
static std::atomic<bool> g_track_force_reset{false};

namespace {

struct sign_loss_element_confirmation_t
{
    bool active;
    ElementType type;
    CircleDirection circle_direction;
    int total_frames;
    int consecutive_pass_frames;
    int consecutive_fail_frames;
};

static sign_loss_element_confirmation_t g_sign_loss_element_confirmation = {
    false,
    ElementType::NORMAL,
    CircleDirection::CIRCLE_DIR_NONE,
    0,
    0,
    0,
};

static void reset_sign_loss_element_confirmation()
{
    g_sign_loss_element_confirmation.active = false;
    g_sign_loss_element_confirmation.type = ElementType::NORMAL;
    g_sign_loss_element_confirmation.circle_direction = CircleDirection::CIRCLE_DIR_NONE;
    g_sign_loss_element_confirmation.total_frames = 0;
    g_sign_loss_element_confirmation.consecutive_pass_frames = 0;
    g_sign_loss_element_confirmation.consecutive_fail_frames = 0;
}

static const char* sign_loss_element_confirmation_text(
    ElementType type,
    CircleDirection circle_direction)
{
    if (type == ElementType::CROSSING)
    {
        return "十字";
    }
    if (circle_direction == CircleDirection::CIRCLE_DIR_LEFT)
    {
        return "左环岛";
    }
    if (circle_direction == CircleDirection::CIRCLE_DIR_RIGHT)
    {
        return "右环岛";
    }
    return "元素";
}

static bool sign_loss_element_confirmation_passes(
    ElementType type,
    CircleDirection circle_direction)
{
    if (type == ElementType::CROSSING)
    {
        return pts_left.corner_found && pts_right.corner_found;
    }
    if (circle_direction == CircleDirection::CIRCLE_DIR_LEFT)
    {
        return pts_left.corner_found;
    }
    if (circle_direction == CircleDirection::CIRCLE_DIR_RIGHT)
    {
        return pts_right.corner_found;
    }
    return false;
}

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

void track_reset_element_runtime_state(bool reset_follow_mode)
{
    reset_sign_loss_element_confirmation();
    set_runtime_normal_state(reset_follow_mode, true);
}

// 功能: 强制复位赛道元素状态（清空状态机与中线缓存）
// 类型: 全局功能函数
// 关键参数: 无
void track_force_reset()
{
    reset_sign_loss_element_confirmation();
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

struct element_candidate_result_t
{
    bool want_crossing;
    bool want_circle;
    CircleDirection want_circle_dir;
};

static element_candidate_result_t detect_current_element_candidate()
{
    element_candidate_result_t result = {
        false,
        false,
        CircleDirection::CIRCLE_DIR_NONE,
    };
    const bool lcorner = pts_left.corner_found;
    const bool rcorner = pts_right.corner_found;

    if (lcorner && rcorner &&
        pts_left.corner_id <= ID_THRESHOLD_crossing_state_change &&
        pts_right.corner_id <= ID_THRESHOLD_crossing_state_change)
    {
        result.want_crossing = true;
        return result;
    }

    const int win = WINDOW_THRESHOLD_roundabout_opposite_straightness;
    if (lcorner && !rcorner)
    {
        result.want_circle = window_is_straight(pts_right, pts_left.corner_id, win);
        if (result.want_circle)
        {
            result.want_circle_dir = CircleDirection::CIRCLE_DIR_LEFT;
        }
    }
    else if (rcorner && !lcorner)
    {
        result.want_circle = window_is_straight(pts_left, pts_right.corner_id, win);
        if (result.want_circle)
        {
            result.want_circle_dir = CircleDirection::CIRCLE_DIR_RIGHT;
        }
    }
    return result;
}

bool track_get_circle_candidate_direction(CircleDirection* out_direction)
{
    const element_candidate_result_t candidate = detect_current_element_candidate();
    if (out_direction != nullptr)
    {
        *out_direction = candidate.want_circle
            ? candidate.want_circle_dir
            : CircleDirection::CIRCLE_DIR_NONE;
    }
    return candidate.want_circle;
}

bool track_sign_loss_element_entry_confirmation_pending()
{
    return g_sign_loss_element_confirmation.active;
}

// 功能: 元素判定与状态机入口（投票 + 保护帧）
// 类型: 图像处理函数
// 关键参数: 无（使用全局 pts_left/pts_right 等）
void element_detect()
{
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
        reset_sign_loss_element_confirmation();
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

    const element_candidate_result_t candidate = detect_current_element_candidate();
    const bool want_crossing = candidate.want_crossing;
    const bool want_circle = candidate.want_circle;
    const CircleDirection want_circle_dir = candidate.want_circle_dir;

    if (g_sign_loss_element_confirmation.active)
    {
        const bool runtime_matches =
            (g_sign_loss_element_confirmation.type == ElementType::CROSSING &&
             element_type == ElementType::CROSSING &&
             crossing_state == CrossingState::CROSSING_IN) ||
            (g_sign_loss_element_confirmation.type == ElementType::CIRCLE &&
             element_type == ElementType::CIRCLE &&
             circle_state == CircleState::CIRCLE_BEGIN &&
             circle_direction == g_sign_loss_element_confirmation.circle_direction);
        if (!runtime_matches)
        {
            reset_sign_loss_element_confirmation();
        }
        else
        {
            const bool pass = sign_loss_element_confirmation_passes(
                g_sign_loss_element_confirmation.type,
                g_sign_loss_element_confirmation.circle_direction);
            ++g_sign_loss_element_confirmation.total_frames;
            if (pass)
            {
                ++g_sign_loss_element_confirmation.consecutive_pass_frames;
                g_sign_loss_element_confirmation.consecutive_fail_frames = 0;
            }
            else
            {
                g_sign_loss_element_confirmation.consecutive_pass_frames = 0;
                ++g_sign_loss_element_confirmation.consecutive_fail_frames;
            }

            const bool confirmed =
                g_sign_loss_element_confirmation.consecutive_pass_frames >=
                FRAME_THRESHOLD_sign_loss_element_confirm_pass;
            const bool rejected =
                g_sign_loss_element_confirmation.consecutive_fail_frames >=
                    FRAME_THRESHOLD_sign_loss_element_confirm_fail ||
                g_sign_loss_element_confirmation.total_frames >=
                    FRAME_THRESHOLD_sign_loss_element_confirm_window;
            if (confirmed)
            {
                std::printf("[h元素确认] %s连续%d帧通过，允许状态机推进\r\n",
                            sign_loss_element_confirmation_text(
                                g_sign_loss_element_confirmation.type,
                                g_sign_loss_element_confirmation.circle_direction),
                            g_sign_loss_element_confirmation.consecutive_pass_frames);
                reset_sign_loss_element_confirmation();
            }
            else if (rejected)
            {
                const ElementType rejected_type = g_sign_loss_element_confirmation.type;
                const CircleDirection rejected_direction =
                    g_sign_loss_element_confirmation.circle_direction;
                const int rejected_total = g_sign_loss_element_confirmation.total_frames;
                const int rejected_fail =
                    g_sign_loss_element_confirmation.consecutive_fail_frames;
                reset_sign_loss_element_confirmation();
                set_runtime_normal_state(true, true);
                roundabout_reset();
                crossing_reset();
                crossing_vote = 0;
                circle_vote = 0;
                protect = 0;
                last = ElementType::NORMAL;
                log_crossing_lock = false;
                publish_debug();
                std::printf("[h元素确认] %s失败，总帧=%d，连续失败=%d，回退NORMAL\r\n",
                            sign_loss_element_confirmation_text(
                                rejected_type,
                                rejected_direction),
                            rejected_total,
                            rejected_fail);
                log_element_if_changed();
                return;
            }
            else
            {
                publish_debug();
                return;
            }
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

    // 误入复位检测仅用于环岛；十字只允许由自身状态机退出。
    // 注：这里使用 process_line 输出的 is_straight（整体判直道）而不是 window_is_straight（局部窗口），避免误判/迟滞。
    if (last == ElementType::CIRCLE &&
        pts_left.is_straight && pts_right.is_straight)
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
        if (image_remote_recognition_is_sign_loss_hold_active())
        {
            g_sign_loss_element_confirmation.active = true;
            g_sign_loss_element_confirmation.type = ElementType::CROSSING;
            g_sign_loss_element_confirmation.circle_direction = CircleDirection::CIRCLE_DIR_NONE;
            g_sign_loss_element_confirmation.total_frames = 1;
            const bool first_pass = sign_loss_element_confirmation_passes(
                ElementType::CROSSING,
                CircleDirection::CIRCLE_DIR_NONE);
            g_sign_loss_element_confirmation.consecutive_pass_frames = first_pass ? 1 : 0;
            g_sign_loss_element_confirmation.consecutive_fail_frames = first_pass ? 0 : 1;
            std::printf("[h元素确认] 十字进入IN，启动%d帧确认窗\r\n",
                        FRAME_THRESHOLD_sign_loss_element_confirm_window);
        }
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
        if (image_remote_recognition_is_sign_loss_hold_active())
        {
            g_sign_loss_element_confirmation.active = true;
            g_sign_loss_element_confirmation.type = ElementType::CIRCLE;
            g_sign_loss_element_confirmation.circle_direction = want_circle_dir;
            g_sign_loss_element_confirmation.total_frames = 1;
            const bool first_pass = sign_loss_element_confirmation_passes(
                ElementType::CIRCLE,
                want_circle_dir);
            g_sign_loss_element_confirmation.consecutive_pass_frames = first_pass ? 1 : 0;
            g_sign_loss_element_confirmation.consecutive_fail_frames = first_pass ? 0 : 1;
            std::printf("[h元素确认] %s进入BEGIN，启动%d帧确认窗\r\n",
                        sign_loss_element_confirmation_text(
                            ElementType::CIRCLE,
                            want_circle_dir),
                        FRAME_THRESHOLD_sign_loss_element_confirm_window);
        }
        log_element_if_changed();
        return;
    }

    // 否则为 NORMAL
    set_runtime_normal_state(false, false);
    last = element_type;
    log_element_if_changed();
}
