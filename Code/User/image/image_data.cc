#include "headfile.h"
#include "image_data.h"

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

// 作用域: 全局变量，中线/路径数据
midline_data midline;

// 作用域: 全局变量，跟线模式（默认混合）
FollowLine follow_mode = FollowLine::MIXED;

// 作用域: 全局变量，识别结果驱动的巡线覆盖模式（默认关闭）
RecognitionFollowOverride recognition_follow_override = RecognitionFollowOverride::NONE;

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

// 作用域: 全局变量，元素检测投票/保护帧调试信息
track_debug_status g_track_debug = {0, 0, 0};


