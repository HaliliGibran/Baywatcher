#include "common.h"
#include "image_data.h"
#include "image_handle.h"
#include "image_midline_process.h"
#include "image_math.h"
#include "element.h"
#include "element/circle.h"
#include "element/crossing.h"
#include "element/zebra.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <opencv2/core.hpp>

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

// 纯跟踪角趋势前馈的状态缓存（文件内静态，全局复用）
typedef struct
{
    float last_angle;     // 上一帧最终角度
    uint8_t had_last;     // 是否已有历史角度
} pure_angle_pre_ctrl_state_t;

// 作用域：文件内静态，全局共享一份趋势前馈状态
static pure_angle_pre_ctrl_state_t g_pure_angle_pre = {0};

// 功能: 限幅单步变化量（用于丢线趋势外推的步长夹紧）
// 类型: 局部功能函数
// 关键参数: v-待限幅值, max_abs-允许的最大绝对值
static inline float clip_step(float v, float max_abs)
{
    return fclip(v, -max_abs, max_abs);
}

// 功能: pure_angle 趋势前馈补偿
// 类型: 图像处理函数
// 关键参数: angle-当前最终角度, enable_comp-当前帧是否允许启用前馈补偿
static float pure_angle_apply_pre_control(float angle, bool enable_comp)
{
    if (!g_pure_angle_pre.had_last)
    {
        g_pure_angle_pre.last_angle = angle;
        g_pure_angle_pre.had_last = 1;
        return angle;
    }

    const float delta = angle - g_pure_angle_pre.last_angle;
    g_pure_angle_pre.last_angle = angle;

    if (!PUREANGLE_PRE_CTRL_ENABLE || !enable_comp)
    {
        return angle;
    }

    // 仅在“当前已经明显在转弯 + 当前帧角度仍朝同一方向继续增大”时补偿：
    // - abs(angle) 太小：多半还是直道/轻微抖动，不补
    // - abs(delta) 太小：当前没有明显增长趋势，不补
    // - angle*delta <= 0：说明正在回正或换向，不补
    if (std::fabs(angle) < PUREANGLE_PRE_CTRL_START_DEG ||
        std::fabs(delta) < PUREANGLE_PRE_CTRL_DELTA_START_DEG ||
        angle * delta <= 0.0f)
    {
        return angle;
    }

    float extra = PUREANGLE_PRE_CTRL_GAIN * delta;
    extra = fclip(extra, -PUREANGLE_PRE_CTRL_MAX_EXTRA_DEG, PUREANGLE_PRE_CTRL_MAX_EXTRA_DEG);
    return fclip(angle + extra, -80.0f, 80.0f);
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

namespace {

// 作用域: 文件内静态，斑马线停车门控状态
// 说明：把“触发 -> 延时停车 -> 释放冷却”几个阶段的瞬时量集中在这里，避免散落全局。
struct zebra_gate_state_t
{
    bool prev_stop;
    uint64_t cooldown_until_ms;
    bool pending_stop;
    uint64_t pending_start_ms;
};

// 作用域: 文件内静态，环岛阶段平均角缓存
// 说明：只服务于出环阶段的短时保护，不作为常规 pure_angle 的长期滤波器。
struct circle_angle_cache_t
{
    float sum;
    int count;
};

static zebra_gate_state_t g_zebra_gate = {0};
static circle_angle_cache_t g_circle_angle = {0};

// 功能: 获取图像链统一毫秒时间戳
// 类型: 局部功能函数
static uint64_t image_now_ms()
{
    return (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// 功能: 清空当前帧可见的巡线输出量
// 类型: 局部功能函数
// 说明:
// - 只清视觉链本帧/下一帧会直接读取的量
// - 不主动改动环岛平均角缓存，保持与旧逻辑一致
static void reset_image_processing_outputs()
{
    reset_pts(pts_left);
    reset_pts(pts_right);
    image_reset_far_line_state();
    image_reset_midline_path_state();
    image_reset_tracking_observation_state();
    g_pure_angle_lost = {0};
    g_pure_angle_pre = {0};
    ResetPureAnglePreviewTransitionState();
}

// 功能: 常规巡线前的左右边线搜索与处理
// 类型: 局部功能函数
static void process_track_edges(const uint8_t (&img)[IMAGE_H][IMAGE_W])
{
    reset_pts(pts_left);
    reset_pts(pts_right);

    SearchLine_Lpt(img, SET_IMAGE_CORE_X, SET_IMAGE_CORE_Y, pts_left.pts, &pts_left.pts_count);
    SearchLine_Rpt(img, SET_IMAGE_CORE_X, SET_IMAGE_CORE_Y, pts_right.pts, &pts_right.pts_count);

    process_line(true, pts_left);
    process_line(false, pts_right);
}

// 功能: 处理斑马线锁停、解锁和延时停车的生命周期
// 类型: 局部功能函数
// 返回值: true 表示本帧已经被斑马线链路接管，应直接 return
static bool handle_zebra_stop_lifecycle(uint64_t t_ms)
{
    if (g_zebra_gate.prev_stop && !zebra_stop)
    {
        g_zebra_gate.cooldown_until_ms = t_ms + (uint64_t)ZEBRA_COOLDOWN_MS;
        printf("[ZEBRA] release\r\n");
        reset_image_processing_outputs();
    }

    if (zebra_stop)
    {
        if (!g_zebra_gate.prev_stop)
        {
            printf("[ZEBRA] stop\r\n");
            track_force_reset();
        }

        reset_image_processing_outputs();
        g_zebra_gate.prev_stop = true;
        return true;
    }

    g_zebra_gate.prev_stop = false;

    if (g_zebra_gate.pending_stop)
    {
        reset_image_processing_outputs();
        if (t_ms - g_zebra_gate.pending_start_ms >= (uint64_t)ZEBRA_STOP_DELAY_MS)
        {
            g_zebra_gate.pending_stop = false;
            zebra_stop = true;
        }
        return true;
    }

    return false;
}

// 功能: 触发斑马线延时停车
// 类型: 局部功能函数
// 返回值: true 表示本帧命中斑马线并已进入 pending 阶段
static bool try_trigger_zebra_pending_stop(const uint8_t (&img)[IMAGE_H][IMAGE_W], uint64_t t_ms)
{
    const bool zebra_can_check =
        (t_ms >= g_zebra_gate.cooldown_until_ms &&
         element_type == ElementType::NORMAL &&
         pts_left.is_straight &&
         pts_right.is_straight);

    if (!zebra_can_check || !zebra_detection(img))
    {
        return false;
    }

    track_force_reset();
    reset_image_processing_outputs();
    g_zebra_gate.pending_stop = true;
    g_zebra_gate.pending_start_ms = t_ms;
    printf("[ZEBRA] detected, delay %d ms\r\n", (int)ZEBRA_STOP_DELAY_MS);
    return true;
}

// 功能: 元素检测与状态机推进
// 类型: 局部功能函数
static void update_track_state_machine(const uint8_t (&img)[IMAGE_H][IMAGE_W])
{
    element_detect();

    if (element_type == ElementType::CIRCLE)
    {
        roundabout_update();
    }

    if (element_type == ElementType::CROSSING)
    {
        crossing_update();
        crossing_far_line_check(img);
    }

    if (element_type == ElementType::NORMAL)
    {
        if_find_far_line = false;
    }
}

// 功能: 根据当前识别覆盖/元素状态选择最终中线来源
// 类型: 局部功能函数
static void build_midline_from_current_state()
{
    if (element_type == ElementType::CROSSING && if_find_far_line)
    {
        MID(pts_far_left.mid, &pts_far_left.mid_count,
            pts_far_right.mid, &pts_far_right.mid_count,
            midline.mid, &midline.mid_count,
            follow_mode);
        return;
    }

    MID(pts_left.mid, &pts_left.mid_count,
        pts_right.mid, &pts_right.mid_count,
        midline.mid, &midline.mid_count,
        follow_mode);
}

// 功能: 从最终中线构建路径并计算当帧测量角
// 类型: 局部功能函数
static void build_path_and_measure_pure_angle()
{
    BuildPathFromCoreToMidlineArc(midline.mid, midline.mid_count,
                                  midline.path, &midline.path_count,
                                  RESAMPLEDIST * PIXPERMETER);
    CalculatePureAngleFromPath(midline.path, midline.path_count, &pure_angle);
}

// 功能: 根据赛道状态决定当前帧是否允许丢线趋势外推
// 类型: 局部功能函数
static bool should_allow_trend_extrap(bool has_valid_measure)
{
    const bool no_special_state = (crossing_state == CrossingState::CROSSING_NONE &&
                                   circle_state == CircleState::CIRCLE_NONE);
    const bool hard_lost_both = (pts_left.pts_count == 0 && pts_right.pts_count == 0);

    if (no_special_state)
    {
        return hard_lost_both;
    }

    return !has_valid_measure;
}

// 功能: 更新环岛平均角缓存
// 类型: 局部功能函数
static void update_circle_average_cache(bool has_valid_measure, float measured_angle)
{
    if (circle_state == CircleState::CIRCLE_IN)
    {
        g_circle_angle.sum = 0.0f;
        g_circle_angle.count = 0;
        circle_average_angle = 0.0f;
        return;
    }

    if (circle_state == CircleState::CIRCLE_RUNNING && has_valid_measure)
    {
        g_circle_angle.sum += measured_angle;
        g_circle_angle.count++;
        circle_average_angle = g_circle_angle.sum / (float)g_circle_angle.count;
    }
}

// 功能: 判断环岛出环阶段是否应启用平均角保护
// 类型: 局部功能函数
static bool should_use_circle_average()
{
    if (circle_state != CircleState::CIRCLE_OUT || g_circle_angle.count <= 0)
    {
        return false;
    }

    const bool is_right = (circle_direction == CircleDirection::CIRCLE_DIR_RIGHT);
    const bool tracking_line_lost = is_right ? (pts_right.pts_count == 0) : (pts_left.pts_count == 0);
    return tracking_line_lost;
}

// 功能: 对测量角应用环岛保护与丢线补偿
// 类型: 局部功能函数
static float finalize_pure_angle_output(float measured_angle, bool has_valid_measure)
{
    const bool allow_trend_extrap = should_allow_trend_extrap(has_valid_measure);
    update_circle_average_cache(has_valid_measure, measured_angle);

    if (should_use_circle_average())
    {
        return pure_angle_apply_lost_strategy(circle_average_angle, true, false);
    }

    return pure_angle_apply_lost_strategy(measured_angle, has_valid_measure, allow_trend_extrap);
}

} // namespace

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
    const uint64_t t_ms = image_now_ms();
    if (handle_zebra_stop_lifecycle(t_ms))
    {
        return;
    }

    // follow_mode 由上层策略决定，这里只消费，不在主链入口硬重置。
    process_track_edges(img);

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


    if (try_trigger_zebra_pending_stop(img, t_ms))
    {
        return;
    }

    update_track_state_machine(img);
    build_midline_from_current_state();
    build_path_and_measure_pure_angle();

    const bool has_valid_measure = (midline.mid_count > 0 && midline.path_count > 0);
    pure_angle = finalize_pure_angle_output(pure_angle, has_valid_measure);

    // 趋势前馈只在存在有效测量时启用，避免把纯外推/纯保护输出继续放大。
    pure_angle = pure_angle_apply_pre_control(pure_angle, has_valid_measure);

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
            std::memcpy(tmp[y], binary.ptr<uint8_t>(y), IMAGE_W);
        }
        img_processing(tmp);
        return;
    }

    const uint8_t (&img_ref)[IMAGE_H][IMAGE_W] = *reinterpret_cast<const uint8_t (*)[IMAGE_H][IMAGE_W]>(binary.data);
    img_processing(img_ref);
}
