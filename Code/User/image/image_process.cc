#include "headfile.h"
#include "common.h"
#include "image_data.h"
#include "image_handle.h"
#include "image_midline_process.h"
#include "image_math.h"
#include "element.h"
#include "element/circle.h"
#include "element/crossing.h"
#include "element/zebra.h"

// 丢线时 pure_angle 的“趋势外推”策略：短期按趋势继续迭代，长期衰减回零，回线做软切换。
// 这些宏可在 common.h 中提前定义以覆盖默认值。
#ifndef PUREANGLE_LOST_TREND_ENABLE
#define PUREANGLE_LOST_TREND_ENABLE 1
#endif

#ifndef PUREANGLE_LOST_TREND_FRAMES
#define PUREANGLE_LOST_TREND_FRAMES 15
#endif

#ifndef PUREANGLE_LOST_MAX_STEP_DEG
#define PUREANGLE_LOST_MAX_STEP_DEG 6.0f
#endif

#ifndef PUREANGLE_LOST_RATE_DECAY
#define PUREANGLE_LOST_RATE_DECAY 0.85f
#endif

#ifndef PUREANGLE_LOST_RETURN_ZERO_DECAY
#define PUREANGLE_LOST_RETURN_ZERO_DECAY 0.92f
#endif

#ifndef PUREANGLE_REACQ_BLEND_FRAMES
#define PUREANGLE_REACQ_BLEND_FRAMES 4
#endif

// 纯跟踪角丢线补偿的状态缓存（文件内静态，全局复用）
typedef struct
{
    float last_meas;      // 最近一次有效测量角
    float last_out;       // 最近一次输出角（可能外推）
    float rate;           // 外推斜率（deg/frame）
    float reacq_from;     // 回线软切换起点
    int32_t lost_frames;  // 连续丢线帧
    int32_t reacq_remain; // 回线软切换剩余帧
    uint8_t had_meas;     // 是否曾有有效测量
} pure_angle_lost_state_t;

// 作用域：文件内静态，全局共享一份丢线补偿状态
static pure_angle_lost_state_t g_pure_angle_lost = {0};

// 功能: 限幅单步变化量（用于丢线趋势外推的步长夹紧）
// 类型: 局部功能函数
// 关键参数: v-待限幅值, max_abs-允许的最大绝对值
static inline float clip_step(float v, float max_abs)
{
    return fclip(v, -max_abs, max_abs);
}

// 功能: 复制一条点列到输出缓冲（用于强制边线巡线）
// 类型: 局部功能函数
// 关键参数: src/src_count-输入点列, dst/dst_count-输出点列
static inline void copy_track_line(const float (&src)[PT_MAXLEN][2], int32_t src_count,
                                   float (&dst)[PT_MAXLEN][2], int32_t* dst_count)
{
    if (dst_count == nullptr)
    {
        return;
    }

    if (src_count <= 0)
    {
        *dst_count = 0;
        return;
    }

    if (src_count > PT_MAXLEN)
    {
        src_count = PT_MAXLEN;
    }

    std::memcpy(&dst[0], &src[0], (size_t)src_count * sizeof(src[0]));
    *dst_count = src_count;
}

// 功能: pure_angle 丢线补偿（趋势外推+回线软切换）
// 类型: 图像处理函数
// 关键参数: measured_angle-当帧测量值, has_valid_measure-是否有有效测量, allow_trend_extrap-是否允许趋势外推
static float pure_angle_apply_lost_strategy(float measured_angle,
                                            bool has_valid_measure,
                                            bool allow_trend_extrap)
{
    if (has_valid_measure)
    {
        if (!g_pure_angle_lost.had_meas)
        {
            g_pure_angle_lost.had_meas = 1;
            g_pure_angle_lost.last_meas = measured_angle;
            g_pure_angle_lost.last_out = measured_angle;
            g_pure_angle_lost.rate = 0.0f;
            g_pure_angle_lost.lost_frames = 0;
            g_pure_angle_lost.reacq_remain = 0;
            return measured_angle;
        }

        // 从丢线恢复：记录软切换起点
        if (g_pure_angle_lost.lost_frames > 0)
        {
            g_pure_angle_lost.reacq_from = g_pure_angle_lost.last_out;
            g_pure_angle_lost.reacq_remain = PUREANGLE_REACQ_BLEND_FRAMES;
            g_pure_angle_lost.lost_frames = 0;
        }

        // 用测量差分估计“趋势”，并限幅 + 轻微滤波
        float step = measured_angle - g_pure_angle_lost.last_meas;
        step = clip_step(step, PUREANGLE_LOST_MAX_STEP_DEG);
        g_pure_angle_lost.rate = 0.5f * g_pure_angle_lost.rate + 0.5f * step;

        g_pure_angle_lost.last_meas = measured_angle;
        g_pure_angle_lost.last_out = measured_angle;

        // 回线软切换
        if (g_pure_angle_lost.reacq_remain > 0)
        {
            float t = (float)(PUREANGLE_REACQ_BLEND_FRAMES - g_pure_angle_lost.reacq_remain + 1)
                    / (float)PUREANGLE_REACQ_BLEND_FRAMES;
            float blended = (1.0f - t) * g_pure_angle_lost.reacq_from + t * measured_angle;
            g_pure_angle_lost.reacq_remain--;
            g_pure_angle_lost.last_out = blended;
            return blended;
        }

        return measured_angle;
    }

    // 无有效测量
    if (!g_pure_angle_lost.had_meas)
    {
        return 0.0f;
    }

    // 不允许趋势外推（特殊元素阶段等）：保持上次测量
    if (!allow_trend_extrap || !PUREANGLE_LOST_TREND_ENABLE)
    {
        g_pure_angle_lost.last_out = g_pure_angle_lost.last_meas;
        g_pure_angle_lost.lost_frames++;
        return g_pure_angle_lost.last_out;
    }

    // 允许趋势外推：短期继续迭代，长期向 0 衰减
    if (g_pure_angle_lost.lost_frames < PUREANGLE_LOST_TREND_FRAMES)
    {
        g_pure_angle_lost.rate = clip_step(g_pure_angle_lost.rate, PUREANGLE_LOST_MAX_STEP_DEG);
        g_pure_angle_lost.last_out += g_pure_angle_lost.rate;
        g_pure_angle_lost.rate *= PUREANGLE_LOST_RATE_DECAY;
    }
    else
    {
        g_pure_angle_lost.last_out *= PUREANGLE_LOST_RETURN_ZERO_DECAY;
        g_pure_angle_lost.rate *= PUREANGLE_LOST_RATE_DECAY;
    }

    g_pure_angle_lost.lost_frames++;
    g_pure_angle_lost.last_out = fclip(g_pure_angle_lost.last_out, -80.0f, 80.0f);
    return g_pure_angle_lost.last_out;
}

// 图像处理逻辑链条（image_process 总控）：
// 1) 斑马线停车逻辑（锁停/延时/冷却）与视觉量清空
// 2) 边线搜索（左右）与边线处理（逆透视→滤波→重采样→曲率/角点/中线）
// 3) 元素检测与状态机更新（环岛/十字/斑马线）
// 4) 远端线探测（十字）与中线融合（MIXED/MIDLEFT/MIDRIGHT）
// 5) 路径构建（从 core 到中线弧形并轨）与 pure_angle 计算
// 6) 丢线补偿（趋势外推/回线软切换）输出纯跟踪角
// 功能: 图像处理总控入口（输入二值图，输出纯跟踪角/状态）
// 类型: 图像处理总控函数（image_process）
// 关键参数: img-二值图像(0/255)，尺寸 IMAGE_H x IMAGE_W
void img_processing(const uint8_t (&img)[IMAGE_H][IMAGE_W])
{
    // system tick：用 steady_clock 提供毫秒级时间基准（与帧率解耦）
    // 作用域: 局部 lambda，获取当前毫秒时间戳
    auto now_ms = []() -> uint64_t {
        return (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    };

    // 作用域: 静态局部变量，用于跨帧维持斑马线状态/计时
    static bool prev_zebra_stop = false;
    static uint64_t zebra_cooldown_until_ms = 0;
    static bool zebra_pending = false;
    static uint64_t zebra_pending_start_ms = 0;

    // 作用域: 局部变量，当前帧毫秒时间戳
    const uint64_t t_ms = now_ms();

    // 作用域: 局部 lambda，用于统一清空本帧/后续帧视觉量
    auto zebra_reset_image_quantities = [&]()
    {
        // 清空边线/中线/路径缓存，避免后续控制继续使用旧的测量量
        reset_pts(pts_left);
        reset_pts(pts_right);
        reset_pts(pts_far_left);
        reset_pts(pts_far_right);
        if_find_far_line = false;
        midline.mid_count = 0;
        midline.path_count = 0;
        pure_angle = 0.0f;
        // 丢线外推内部状态清零，避免“旧趋势”影响解锁后的第一段
        g_pure_angle_lost = {0};
    };

    // 手动 Start 会在控制层清 zebra_stop，这里用边沿检测打印并进入 cooldown
    if (prev_zebra_stop && !zebra_stop)
    {
        zebra_cooldown_until_ms = t_ms + (uint64_t)ZEBRA_COOLDOWN_MS;
        printf("[ZEBRA] release\r\n");
        // 解锁时也清一次视觉缓存，避免刚解锁就拿到“停前残留”的边线/中线
        zebra_reset_image_quantities();
    }

    // 斑马线停车：一旦触发就锁死停车，直到手动 Start 解锁。停电机在控制层处理。
    if (zebra_stop)
    {
        if (!prev_zebra_stop)
        {
            printf("[ZEBRA] stop\r\n");
            // 进入锁停时：强制复位元素/状态机投票，并清空视觉量
            track_force_reset();
        }

        // 锁停期间：持续保持中性角 + 清空视觉量（不让外推/旧中线参与控制）
        zebra_reset_image_quantities();

        prev_zebra_stop = true;
        return;
    }

    prev_zebra_stop = false;

    // 已检测到斑马线，延迟停车阶段：
    // - 立刻清状态/清视觉量（中线等全部置空）
    // - 保持 pure_angle=0 直行
    // - 到达延迟时间后，置位 zebra_stop 进入锁停
    if (zebra_pending)
    {
        zebra_reset_image_quantities();
        if (t_ms - zebra_pending_start_ms >= (uint64_t)ZEBRA_STOP_DELAY_MS)
        {
            zebra_pending = false;
            zebra_stop = true;
            // stop 日志在 zebra_stop 分支统一打印
        }
        return;
    }

    // 初始化
    reset_pts(pts_left);
    reset_pts(pts_right);
    // follow_mode 由外部逻辑（按键/上层策略）决定；这里不要每帧强制覆盖，
    // 否则会导致模式切换失效，并可能造成你观察到的“跳变”来自人为重置。


//边线检测
    // 左边界
    SearchLine_Lpt(img,
                SET_IMAGE_CORE_X,                        // 初始X坐标
                SET_IMAGE_CORE_Y,                        // 初始Y坐标
                pts_left.pts,                   // 输出点集数组
                &pts_left.pts_count);           // 输出点数量
    // 右边界
    SearchLine_Rpt(img, 
                SET_IMAGE_CORE_X,                         // 初始X坐标
               SET_IMAGE_CORE_Y,                        // 初始Y坐标
               pts_right.pts,                   // 输出点集数组
               &pts_right.pts_count);           // 输出点数量


//边线处理
    //左
    process_line(true, pts_left);
    //右
    process_line(false, pts_right);

    if (t_ms >= zebra_cooldown_until_ms && element_type == ElementType::NORMAL && pts_left.is_straight && pts_right.is_straight)
    {
        if (zebra_detection(img))
        {
            // 触发时立刻复位状态机 + 清视觉量，并进入“延迟停车”阶段
            track_force_reset();
            zebra_reset_image_quantities();
            zebra_pending = true;
            zebra_pending_start_ms = t_ms;
            printf("[ZEBRA] detected, delay %d ms\r\n", (int)ZEBRA_STOP_DELAY_MS);
            return;
        }
    }

// //测试赛道宽度矫正PIXPERMETER
    // float roadwidth_pix_sqr = (100.0f) * (100.0f);
    // int id_roadwidth_test = 0;
    // for(int i= -5 ; i < 5; i++)
    // {
    //     float dx=pts_left.pts_resample[10][1]- pts_right.pts_resample[10 + i][1];
    //     float dy=pts_left.pts_resample[10][0]- pts_right.pts_resample[10 + i][0];
    //     float roadwidth_pix_temp_sqr = (dx*dx + dy*dy);

    //     if(roadwidth_pix_temp_sqr < roadwidth_pix_sqr)
    //     {
    //         roadwidth_pix_sqr = roadwidth_pix_temp_sqr;
    //         id_roadwidth_test =10 + i;
    //     }
    // }
    // printf("赛道宽度像素值: %.2f pix\r\n",  Q_sqrt(roadwidth_pix_sqr));
    // printf("理论赛道宽度像素值: %.2f pix\r\n",  ROADWIDTH * PIXPERMETER);
    // printf("中线偏移量: %.2f pix\r\n",  PIXPERMETER * ROADWIDTH / 2);
    // float PIXPERMETER_NEW = Q_sqrt(roadwidth_pix_sqr) / ROADWIDTH ;
    // printf("PIXPERMETER_NEW: %.2f pix/m\r\n",  PIXPERMETER_NEW);
    // printf("id_roadwidth_test"": %d\r\n",  id_roadwidth_test);
    // return;

    // //测试中线偏移量矫正PIXPERMETER_ACROSS
    // float mid_div_pix_sqr = (100.0f) * (100.0f);
    // int id_div_test = 0;
    // for(int i=  0; i < pts_right.mid_count; i++)
    // {
    //     float dx= pts_left.mid[10][1] - pts_right.mid[ i][1];
    //     float dy= pts_left.mid[10][0] - pts_right.mid[i][0];
    //     float mid_div_pix_temp_sqr = (dx*dx + dy*dy);

    //     if(mid_div_pix_temp_sqr < mid_div_pix_sqr)
    //     {
    //         mid_div_pix_sqr = mid_div_pix_temp_sqr;
    //         id_div_test = i;
    //     }
    // }
    // printf("中线偏移量像素值: %.2f pix\r\n",  Q_sqrt(mid_div_pix_sqr));
    // printf("id_div_test"": %d\r\n",  id_div_test);
    // printf("中线左点:(%.2f, %.2f), 右点:(%.2f, %.2f)\r\n", 
    //         pts_left.mid[10][1], pts_left.mid[10][0],
    //         pts_right.mid[id_div_test][1], pts_right.mid[id_div_test][0]
    // );
    // return;


    // [Recognition Chain Step 8] 识别结果驱动的巡线覆盖入口。
    // 作用：当主循环给出 FORCE_LEFT / FORCE_RIGHT 时，图像侧停止元素状态机干预，
    // 直接把某一侧边线当作本帧的最终中线来源。
    const bool recognition_override_active =
        (recognition_follow_override != RecognitionFollowOverride::NONE);

//元素检测（每帧都跑一次，用于投票衰减/保护帧）
    if (!recognition_override_active)
    {
        element_detect();

    //环岛
        if(element_type == ElementType::CIRCLE)
        {
            roundabout_update();
        }

    //十字
        if(element_type == ElementType::CROSSING)
        {
            crossing_update();
            crossing_far_line_check(img);
        }
        if(element_type == ElementType::NORMAL)
            if_find_far_line = false;
    }
    else
    {
        // [Recognition Chain Step 8A] 覆盖开启时屏蔽元素状态机。
        // 作用：避免环岛/十字等状态机继续改写 follow_mode，和识别结果相互打架。
        element_type = ElementType::NORMAL;
        circle_state = CircleState::CIRCLE_NONE;
        crossing_state = CrossingState::CROSSING_NONE;
        circle_direction = CircleDirection::CIRCLE_DIR_NONE;
        if_find_far_line = false;
        reset_pts(pts_far_left);
        reset_pts(pts_far_right);
    }

    if (recognition_follow_override == RecognitionFollowOverride::FORCE_LEFT)
    {
        // [Recognition Chain Step 8B] weapon 结果对应左线接管。
        copy_track_line(pts_left.pts_resample, pts_left.pts_resample_count,
                        midline.mid, &midline.mid_count);
    }
    else if (recognition_follow_override == RecognitionFollowOverride::FORCE_RIGHT)
    {
        // [Recognition Chain Step 8C] supply 结果对应右线接管。
        copy_track_line(pts_right.pts_resample, pts_right.pts_resample_count,
                        midline.mid, &midline.mid_count);
    }
    else if(element_type == ElementType::CROSSING && if_find_far_line)
    {
        MID(pts_far_left.mid, &pts_far_left.mid_count,
            pts_far_right.mid, &pts_far_right.mid_count,
            midline.mid, &midline.mid_count,
            follow_mode);
    }
    else
    {
        MID(pts_left.mid, &pts_left.mid_count,
            pts_right.mid, &pts_right.mid_count,
            midline.mid, &midline.mid_count,
            follow_mode);
    }

    BuildPathFromCoreToMidlineArc(midline.mid, midline.mid_count,
                                  midline.path, &midline.path_count,
                                  RESAMPLEDIST*PIXPERMETER);

    // [Recognition Chain Step 9] 中线覆盖后继续走统一路径生成与 pure_angle 输出。
    // 作用：识别链最终并不直接输出舵量，而是复用现有巡线路径控制出口。
    CalculatePureAngleFromPath(midline.path, midline.path_count, &pure_angle);

    // 丢线处理：
    // - 普通赛道：仅在“双边全丢”时允许趋势外推（避免单侧误检导致外推漂移）
    // - 元素阶段：禁止因为丢线去“改用另一侧线/切换跟线来源”，因此当缺测时允许用趋势外推接管
    bool has_valid_measure = (midline.mid_count > 0 && midline.path_count > 0);
    bool no_special_state = (crossing_state == CrossingState::CROSSING_NONE
                          && circle_state == CircleState::CIRCLE_NONE);
    bool hard_lost_both = (pts_left.pts_count == 0 && pts_right.pts_count == 0);
    bool allow_trend_extrap = false;

    if (no_special_state)
    {
        allow_trend_extrap = hard_lost_both;
    }
    else
    {
        // 元素阶段：只要当前帧没有有效测量，就允许外推（用于维持控制稳定）
        allow_trend_extrap = !has_valid_measure;
    }

    // 环岛平均角保护
    static float circle_angle_sum = 0.0f;
    static int circle_angle_count = 0;

    // 1. 状态机清理与累加
    if (circle_state == CircleState::CIRCLE_IN) 
    {
        // 入环前清空累计值，为 RUNNING 阶段做准备
        circle_angle_sum = 0.0f;
        circle_angle_count = 0;
        circle_average_angle = 0.0f;
    } 
    else if (circle_state == CircleState::CIRCLE_RUNNING && has_valid_measure) 
    {
        // RUNNING 阶段：累加有效测量值并求均值
        circle_angle_sum += pure_angle;
        circle_angle_count++;
        circle_average_angle = circle_angle_sum / (float)circle_angle_count;
    }

    // 2. OUT 阶段防丢线判定
    bool use_circle_avg = false;
    if (circle_state == CircleState::CIRCLE_OUT) 
    {
        bool is_right = (circle_direction == CircleDirection::CIRCLE_DIR_RIGHT);
        // OUT 阶段切内侧：右环岛跟右线，左环岛跟左线。判断该侧线点数是否为 0
        bool tracking_line_lost = is_right ? (pts_right.pts_count == 0) : (pts_left.pts_count == 0);
        
        // 如果巡线侧丢线，并且之前成功算出了平均值
        if (tracking_line_lost && circle_angle_count > 0) 
        {
            use_circle_avg = true;
        }
    }

    3. 应用输出
    全部注释-->环岛不采取任何保护；注释else-->环岛采取平均
    if (use_circle_avg) 
    {
        pure_angle = circle_average_angle;
        // 关键技巧：伪造一次有效测量输入给丢线补偿器，
        // 避免退出 OUT 阶段恢复正常巡线时，补偿器里的遗留状态导致输出跳变
        pure_angle = pure_angle_apply_lost_strategy(pure_angle, true, false);
    } 
    else 
    {
        // 正常走原有的趋势预测或直出
        pure_angle = pure_angle_apply_lost_strategy(pure_angle, has_valid_measure, allow_trend_extrap);
    }

}

// 功能: OpenCV Mat 版本图像处理入口（进行合法性/连续性校验后转调）
// 类型: 图像处理总控函数（image_process）
// 关键参数: binary-CV_8UC1 二值图(需连续, 尺寸 IMAGE_H x IMAGE_W)
void img_processing(const cv::Mat& binary)
{
    if (binary.empty())
    {
        return;
    }
    if (binary.type() != CV_8UC1 || binary.rows != IMAGE_H || binary.cols != IMAGE_W)
    {
        return;
    }
    // 需要连续且 stride==IMAGE_W，才能安全地把 data 解释成 [H][W]
    if (!binary.isContinuous() || binary.step != IMAGE_W)
    {
        // 退化路径：拷贝到连续缓冲再处理（保持行为正确）
        uint8_t tmp[IMAGE_H][IMAGE_W];
        for (int y = 0; y < IMAGE_H; ++y)
        {
            memcpy(tmp[y], binary.ptr<uint8_t>(y), IMAGE_W);
        }
        img_processing(tmp);
        return;
    }

    const uint8_t (&img_ref)[IMAGE_H][IMAGE_W] = *reinterpret_cast<const uint8_t (*)[IMAGE_H][IMAGE_W]>(binary.data);
    img_processing(img_ref);
}
