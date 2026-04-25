#ifndef SMARTCAR_IMAGE_DATA_H
#define SMARTCAR_IMAGE_DATA_H

#include <cstdint>
#include "common.h"
#include "Communication.h"

// 作用域: 全局变量，斑马线停车请求标志（视觉侧置位，控制层处理）
extern bool zebra_stop;

// -------------------- 单侧边线处理上下文 --------------------
struct pts_well_processed
{
    // 原始边线点（像素坐标，pts[i][0]=y, pts[i][1]=x）
    int32_t pts[PT_MAXLEN][2];
    int32_t pts_count; // 原始点数量（作用域: 结构体成员）

    // 逆透视后的边线点（像素坐标）
    float pts_inv[PT_MAXLEN][2];
    int32_t pts_inv_count;

    // 滤波后的边线点
    float pts_filter[PT_MAXLEN][2];
    int32_t pts_filter_count;

    // 重采样后的边线点（严格等距）
    float pts_resample[PT_MAXLEN][2];
    int32_t pts_resample_count;

    // 重采样点对应的输入点索引（用于把 corner_id 从 pts_resample 映射回 pts_filter）
    // 约定：resample_src_id[i] 是生成 pts_resample[i] 时更接近的 pts_filter 输入点索引。
    int32_t resample_src_id[PT_MAXLEN]; // 重采样点映射回 pts_filter 的索引

    // 弯曲强度（无符号，越大越“弯”）：1 - cos(theta)，范围约 [0, 2]
    float curvature[PT_MAXLEN];
    int curvature_num;

    // 极大值抑制后的弯曲强度
    float curvature_nms[PT_MAXLEN];
    int curvature_nms_num;

    bool is_straight; // 是否判定为直道
    int is_curve;     // 是否判定为曲线（1/0）

    bool corner_found; // 是否检测到角点
    int corner_id;     // 角点在 pts_resample 中的索引
    float corner[2];   // 角点坐标（y,x）

    // 单侧中线（由边线内偏得到）
    float mid[PT_MAXLEN][2];
    int32_t mid_count;
};

// 功能: 清空 pts_well_processed 内部计数/状态（不清数组内容）
// 类型: 全局功能函数
// 关键参数: p-待复位的边线处理上下文
static inline void reset_pts(pts_well_processed& p)
{
    p.pts_count = 0;
    p.pts_inv_count = 0;
    p.pts_filter_count = 0;
    p.pts_resample_count = 0;
    for (int i = 0; i < PT_MAXLEN; ++i)
    {
        p.resample_src_id[i] = -1;
    }
    p.curvature_num = 0;
    p.curvature_nms_num = 0;
    p.is_straight = false;
    p.is_curve = 0;
    p.corner_found = false;
    p.corner_id = 0;
    p.corner[0] = 0.0f;
    p.corner[1] = 0.0f;
    p.mid_count = 0;
}

// 作用域: 全局变量，左右边线处理上下文（每帧更新）
extern pts_well_processed pts_left;
extern pts_well_processed pts_right;

// -------------------- 十字远端线缓存 --------------------
// 十字远端线专用缓冲（与 pts_left/pts_right 同结构，便于复用 process_line 流水线）
// 作用域: 全局变量，十字远端线处理上下文（仅十字阶段使用）
extern pts_well_processed pts_far_left;
extern pts_well_processed pts_far_right;

// 十字远端线是否找到（由 crossing_far_line_check 维护）
// 作用域: 全局变量，十字远端线是否找到
extern bool if_find_far_line;

// 功能: 清空十字远端线缓存与有效标志
// 类型: 全局状态辅助函数
// 关键参数: 无
// 说明：统一管理 if_find_far_line / pts_far_left / pts_far_right 的复位，减少跨文件散写。
void image_reset_far_line_state();

// -------------------- 融合中线 / 最终路径 --------------------
struct midline_data
{
    float mid[PT_MAXLEN][2];   // 中线点列（融合后）
    int32_t mid_count;

    float path[PT_MAXLEN][2];  // 路径点列（从 core 并轨到中线）
    int32_t path_count;
};

// 作用域: 全局变量，中线/路径数据（供控制/纯跟踪角计算）
extern midline_data midline;

// 功能: 清空融合中线与路径输出
// 类型: 全局状态辅助函数
// 关键参数: 无
// 说明：只清 count，不清数组内容；供图像主链短路或复位时统一调用。
void image_reset_midline_path_state();

// -------------------- 跟线模式 --------------------
// 跟线模式（作用域: 全局枚举类型）
enum class FollowLine:uint8_t
{
    MIDLEFT,
    MIDRIGHT,
    MIXED
};

// 作用域: 全局变量，跟线模式（MIXED/MIDLEFT/MIDRIGHT）
extern FollowLine follow_mode;

// -------------------- 元素状态机 --------------------
// 元素类型（作用域: 全局枚举类型）
enum class ElementType:uint8_t
{
    NORMAL,
    CIRCLE,
    CROSSING
};
// 作用域: 全局变量，当前元素类型
extern ElementType element_type;

// 环岛状态机状态（作用域: 全局枚举类型）
enum class CircleState:uint8_t
{
    CIRCLE_NONE,        // 非环岛模式
    CIRCLE_BEGIN,       // 开始阶段
    CIRCLE_IN,          // 入环阶段
    CIRCLE_RUNNING,     // 环内行驶阶段
    CIRCLE_OUT,         // 出环阶段
    CIRCLE_END          // 结束阶段
};
// 作用域: 全局变量，环岛状态机当前状态
extern CircleState circle_state;

// 环岛方向（作用域: 全局枚举类型）
enum class CircleDirection:uint8_t
{
    CIRCLE_DIR_NONE,
    CIRCLE_DIR_LEFT,
    CIRCLE_DIR_RIGHT
};
// 作用域: 全局变量，环岛行驶方向（左/右）
extern CircleDirection circle_direction;

// 十字状态机状态（作用域: 全局枚举类型）
enum class CrossingState:uint8_t
{
    CROSSING_NONE,      // 非十字路口模式
    CROSSING_IN,     // 十字路口进入阶段
    CROSSING_RUNNING    // 十字路口丢线阶段
};
// 作用域: 全局变量，十字状态机当前状态
extern CrossingState crossing_state;


// -------------------- 调试观测与控制输出 --------------------
// 调试：用于把 element_detect 的投票/保护帧等信息提供给其它模块（环岛/十字打印等）。
// 说明：这是“调试观测数据”，不参与算法决策。
struct track_debug_status
{
    int crossing_vote; // 十字投票计数
    int circle_vote;   // 环岛投票计数
    int protect;       // 保护帧计数
};

// 作用域: 全局变量，元素检测投票/保护帧调试信息
extern track_debug_status g_track_debug;
// 作用域: 全局变量，纯跟踪角输出（度，右转负/左转正）
extern float pure_angle;

// 作用域: 全局变量，环岛 RUNNING 阶段纯跟踪角的平均值
extern float circle_average_angle;

// 作用域: 全局变量，MidLineSuggestPureAnglePreviewImageY 当前帧的稳健局部转角观测量（单位：deg）
extern float preview_curve_angle_deg;

// 作用域: 全局变量，CalculatePureAngleFromPath 当前帧实际参与 pure_angle 计算的预瞄图像行
extern float preview_img_y;

// 功能: 清空图像主链输出观测量
// 类型: 全局状态辅助函数
// 关键参数: 无
// 说明：统一管理 pure_angle / preview_curve_angle_deg / preview_img_y 的复位语义。
void image_reset_tracking_observation_state();

// -------------------- 双板远端识别动作接口层 --------------------
// 功能: 清空运行板远端事件缓存
// 类型: 全局状态辅助函数
// 关键参数: 无
void image_remote_recognition_reset();

// 功能: 把识别板串口状态解释成运行板本地动作
// 类型: 全局状态更新函数
// 关键参数:
// - code/seq: 识别板发送的状态码与序号
// - current_pure_angle: 收到 vehicle 事件当下的 pure_angle
// - t_ms: 当前时间戳（毫秒）
void image_remote_recognition_apply_state(BoardVisionCode code,
                                          uint8_t seq,
                                          float current_pure_angle,
                                          uint64_t t_ms);

// 功能: 查询当前是否处于远端 vehicle 的保持航向窗口
// 类型: 全局状态查询函数
// 关键参数:
// - t_ms: 当前时间戳（毫秒）
bool image_remote_recognition_is_vehicle_active(uint64_t t_ms);

// 功能: 查询当前是否处于远端 vehicle 的保持航向窗口，并返回锁存航向角
// 类型: 全局状态查询函数
// 关键参数:
// - t_ms: 当前时间戳（毫秒）
// - hold_yaw: 输出锁存航向角
bool image_remote_recognition_try_get_hold_yaw(uint64_t t_ms, float* hold_yaw);

// 功能: 查询当前是否锁定到单边边线跟线
// 类型: 全局状态查询函数
// 关键参数:
// - out_mode: 输出 MIDLEFT / MIDRIGHT
bool image_remote_recognition_get_forced_follow_mode(FollowLine* out_mode);

// 功能: 查询当前是否启用远端 w/s 激进转角接管
// 类型: 全局状态查询函数
// 关键参数:
// - raw_pure_angle: 当帧未接管前的原始几何 pure_angle
// - out_override: 输出接管角
// 说明:
// - w: 输出 +BW_REMOTE_SIGN_AGGRESSIVE_ABS_PURE_ANGLE，直到 raw_pure_angle < 0
// - s: 输出 -BW_REMOTE_SIGN_AGGRESSIVE_ABS_PURE_ANGLE，直到 raw_pure_angle > 0
bool image_remote_recognition_get_aggressive_turn_override(float raw_pure_angle, float* out_override);

// 功能: 查询当前是否因砖块观测而持续压制环岛状态机
// 类型: 全局状态查询函数
// 关键参数: 无
bool image_remote_recognition_should_block_circle();


#endif
