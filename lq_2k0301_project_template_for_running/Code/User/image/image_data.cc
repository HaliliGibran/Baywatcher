#include "image_data.h"
#include "TargetHandler.h"

namespace {

constexpr uint64_t kRemoteVehicleHoldMs = static_cast<uint64_t>(BW_REMOTE_VEHICLE_HOLD_MS);

// 作用域: 文件内静态，运行板对“最近一次远端识别事件”的缓存
// 设计目的：
// 1. 让串口收包线程只负责解释远端事件，不直接碰底层 PID
// 2. 对重复发送的同一事件做 seq 去重
// 3. 把一次性绕行动作 owner 收回到运行板本地
struct remote_recognition_runtime_t
{
    bool last_seq_valid;
    uint8_t last_seq;
    // 当前 vehicle 保持航向窗口的截止时刻与目标角
    uint64_t vehicle_hold_until_ms;
    float vehicle_hold_yaw;
};

// 作用域: 文件内静态，全局唯一的远端识别运行时缓存
remote_recognition_runtime_t g_remote_recognition = {
    false,
    0,
    0,
    0.0f,
};

static bool remote_action_busy(uint64_t t_ms)
{
    return handler_sys.Is_Executing() || (t_ms < g_remote_recognition.vehicle_hold_until_ms);
}

static TargetBoardType map_event_to_target(BoardActionEvent action)
{
    switch (action)
    {
    case BoardActionEvent::WEAPON:
        return TargetBoardType::WEAPON;
    case BoardActionEvent::SUPPLY:
        return TargetBoardType::SUPPLIES;
    default:
        return TargetBoardType::NONE;
    }
}

} // namespace

// 作用域: 全局变量，斑马线停车请求标志（视觉侧置位）
bool zebra_stop = false;

// 作用域: 全局变量，左右边线处理上下文
pts_well_processed pts_left;
pts_well_processed pts_right;
// 作用域: 全局变量，十字远端线处理上下文
pts_well_processed pts_far_left;
pts_well_processed pts_far_right;

// 作用域: 全局变量，十字远端线是否找到
bool if_find_far_line = false;

void image_reset_far_line_state()
{
    if_find_far_line = false;
    reset_pts(pts_far_left);
    reset_pts(pts_far_right);
}

// 作用域: 全局变量，中线/路径数据
midline_data midline;

void image_reset_midline_path_state()
{
    midline.mid_count = 0;
    midline.path_count = 0;
}

// 作用域: 全局变量，跟线模式（默认混合）
FollowLine follow_mode = FollowLine::MIXED;

// 功能: 清空远端识别动作缓存
// 类型: 全局状态辅助函数
// 关键参数: 无
void image_remote_recognition_reset()
{
    g_remote_recognition.last_seq_valid = false;
    g_remote_recognition.last_seq = 0;
    g_remote_recognition.vehicle_hold_until_ms = 0;
    g_remote_recognition.vehicle_hold_yaw = 0.0f;
}

// 功能: 把串口事件解释成运行板本地动作状态
// 类型: 全局状态更新函数
// 关键参数:
// - action / seq: 识别板发来的事件及其序号
// - current_pure_angle: 收到 vehicle 事件当下的纯跟踪角
// - t_ms: 当前时间戳（毫秒）
// 设计说明：
// - 重复发送的同一 seq 只消费一次
// - 执行动作期间到达的新事件直接丢弃，不排队
// - vehicle 锁存 pure_angle 1000ms，weapon/supply 直接触发 TargetHandler
void image_remote_recognition_apply_event(BoardActionEvent action,
                                          uint8_t seq,
                                          float current_pure_angle,
                                          uint64_t t_ms)
{
    if (action == BoardActionEvent::NONE)
    {
        return;
    }

    if (g_remote_recognition.last_seq_valid && g_remote_recognition.last_seq == seq)
    {
        return;
    }

    g_remote_recognition.last_seq_valid = true;
    g_remote_recognition.last_seq = seq;

    if (remote_action_busy(t_ms))
    {
        return;
    }

    if (action == BoardActionEvent::VEHICLE)
    {
        g_remote_recognition.vehicle_hold_yaw = current_pure_angle;
        g_remote_recognition.vehicle_hold_until_ms = t_ms + kRemoteVehicleHoldMs;
        return;
    }

    g_remote_recognition.vehicle_hold_until_ms = 0;
    g_remote_recognition.vehicle_hold_yaw = 0.0f;

    const TargetBoardType target = map_event_to_target(action);
    if (target != TargetBoardType::NONE)
    {
        handler_sys.Start_Action(target);
    }
}

// 功能: 查询当前是否处于远端 vehicle 的保持航向窗口
// 类型: 全局状态查询函数
// 关键参数:
// - t_ms: 当前时间戳（毫秒）
// - hold_yaw: 输出锁存的目标航向角
bool image_remote_recognition_try_get_hold_yaw(uint64_t t_ms, float* hold_yaw)
{
    if (hold_yaw == nullptr || t_ms >= g_remote_recognition.vehicle_hold_until_ms)
    {
        return false;
    }

    *hold_yaw = g_remote_recognition.vehicle_hold_yaw;
    return true;
}

// 作用域: 全局变量，当前元素类型（默认普通）
ElementType element_type = ElementType::NORMAL;

// 作用域: 全局变量，环岛状态与方向
CircleState circle_state = CircleState::CIRCLE_NONE;
CircleDirection circle_direction = CircleDirection::CIRCLE_DIR_NONE;

// 作用域: 全局变量，十字状态机状态
CrossingState crossing_state = CrossingState::CROSSING_NONE;

// 作用域: 全局变量，纯跟踪角输出（度）
float pure_angle = 0.0f;

// 作用域: 全局变量，环岛 RUNNING 阶段纯跟踪角的平均值
float circle_average_angle = 0.0f;

// 作用域: 全局变量，当前帧平均曲率观测量
float average_curvature = 0.0f;

// 作用域: 全局变量，当前帧预瞄图像行观测量
float preview_img_y = (float)PUREANGLE_PREVIEW_BASE_IMAGE_Y;

void image_reset_tracking_observation_state()
{
    pure_angle = 0.0f;
    average_curvature = 0.0f;
    preview_img_y = (float)PUREANGLE_PREVIEW_BASE_IMAGE_Y;
}

// 作用域: 全局变量，元素检测投票/保护帧调试信息
track_debug_status g_track_debug = {0, 0, 0};
