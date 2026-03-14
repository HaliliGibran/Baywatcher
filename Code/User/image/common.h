#ifndef _USER_COMMON_H_
#define _USER_COMMON_H_



// 图像尺寸（全局宏，所有图像处理函数共享）
#define IMAGE_H               (120)   // 图像高度（像素）
#define IMAGE_W               (160)   // 图像宽度（像素）

#define PI32                  (3.1415926535898f) // 圆周率（float）

//绘图参数（全局宏）
#define DRAWRADIUS           (1)      // 绘图半径（像素）
#define BLACK_IN_GRAY        (0)      // 二值图黑色像素值
#define WHITE_IN_GRAY        (255)    // 二值图白色像素值

//物理参数（全局宏）
#define PIXPERMETER           (68.05f) // 逆透视后每米像素数
#define ROADWIDTH             (0.45f)  // 赛道宽度（米）

#define PT_MAXLEN             (80)    // 点列最大长度（左右线/中线/路径均使用）

#define SET_IMAGE_CORE_X   (80)  // 图像核心点X（像素）
#define SET_IMAGE_CORE_Y   (115) // 图像核心点Y（像素）

#define SEARCH_LINE_START_OFFSET  (20)   // 寻线起点横向偏移（像素）
#define SELFADAPT_KERNELSIZE  (7)        // 自适应滤波核尺寸（奇数）
#define FILTER_KERNELSIZE     (7)        // 边线滤波核尺寸（奇数）
#define SELFADAPT_OFFSET      (8)        // 自适应阈值偏移
#define RESAMPLEDIST          (0.02f)    // 重采样间距（米）
#define ANGLEDIST             (0.2f)     // 角度计算“跨度”（米）

//中线（全局宏）
#define MIXED_LINE_DIFF_THRESHOLD_PIX    (0.1f)  // 左右中线混合差异阈值（像素）
#define MIXED_POINT_NUM_THRESHOLD       (5)      // 混合中线最少重合点数
#define PATH_NEAR_ID_FOR_PURE_ANGLE      (4)      // pure_angle 使用的近端路径点索引
// 用于“路径并轨到中线”的参考图像行（越小越看远，越大越看近）
#define PATH_BLEND_REF_IMAGE_Y         (2*IMAGE_H/3)

#define FRAMENONE             (3) // 预留：状态保持帧数（未使用/保留）
#define FRAMETOLEFT           (5) // 预留：左切换保护帧（未使用/保留）
#define FRAMETORIGHT          (5) // 预留：右切换保护帧（未使用/保留）











//车辆属性（全局宏）
#define DISTANCE_FROM_VIEW_TO_CAR   (0.23f)   // 图像(SET_IMAGE_CORE)到车轴距离（米）
#define WIDTH_OF_CAR                (0.18f)   // 车体宽度（米）








//特征量（全局宏）
//角点
#define ANGLE_THRESHOLD_get_corners_conf_max             (120.0f)   // 角点检测置信度上限（度）
#define ANGLE_THRESHOLD_get_corners_conf_min             (60.0f)    // 角点检测置信度下限（度）
#define ID_THRESHOLD_get_corners_near_detect             (15)       // 角点近距离索引上限
//曲线
#define CURVE_THRESHOLD       (10.0f * PI32 / 180.0f)      // 曲线判断角度阈值（弧度）
//直线
#define ANGLE_THRESHOLD_is_straight         (8.0f)         // 直线判断角度阈值（度）



//左右线切换点数差阈值（全局宏）
#define PTS_THRESHOLD_follow_left               (0) // 左跟线点数差阈值（保留）


//元素判定（全局宏）
//双角点切换十字状态的id阈值
#define ID_THRESHOLD_crossing_state_change      (20) // 十字判定角点索引阈值
//环岛保护帧数
#define FRAME_THRESHOLD_roundabout_protect_frame        (3) // 环岛状态保护帧
//环岛或斜入十字判定帧数
#define FRAME_THRESHOLD_roundabout_or_crossing_frame (3) // 元素投票确认帧
//斜入十字保护帧数
#define FRAME_THRESHOLD_one_corner_crossing_protect_frame (3) // 十字保护帧
//环岛保护帧数
#define FRAME_THRESHOLD_roundabout_protect_frame (3) // 环岛保护帧（重复宏，保留）
//角点对侧直线性窗口判断阈值
#define WINDOW_THRESHOLD_roundabout_opposite_straightness    (10) // 对侧直线性窗口半宽（点数）




//元素帧数阈值（全局宏）
//十字路口丢线帧数阈值
#define FRAME_THRESHOLD_crossing_lost_line_counter       (2)    // 十字入内阶段允许的连续丢线帧
//环岛入环丢线阈值
#define FRAME_THRESHOLD_roundabout_begin_to_in_lost_line_counter       (2) // BEGIN->IN 丢线阈值
//环岛入环重新找线阈值
#define FRAME_THRESHOLD_roundabout_begin_to_in_found_line_counter       (2) // BEGIN->IN 找线阈值
//环岛出环岛角点连续判定阈值
#define FRAME_THRESHOLD_roundabout_running_to_out_corner_counter       (0) // RUNNING->OUT 角点连续帧
//环岛结束丢线阈值
#define FRAME_THRESHOLD_roundabout_end_lost_line_counter       (2)    // END 退出丢线阈值
//环岛结束重新找线阈值
#define FRAME_THRESHOLD_roundabout_end_found_line_counter       (2)    // END 退出找线阈值



//赛道类型角度阈值（全局宏）
//曲线判断角度
#define ANGLE_THRESHOLD_track_type_curve            (15.0f ) // 曲线判定阈值（度）
//急弯判断角度
#define ANGLE_THRESHOLD_track_type_sharp_curve      (35.0f ) // 急弯判定阈值（度）
//急弯保护
#define FRAME_THRESHOLD_sharp_curve_protect_frame   ( 25 )  // 急弯保护帧







#endif /* USER_CAMERA_COMMON_H_ */
