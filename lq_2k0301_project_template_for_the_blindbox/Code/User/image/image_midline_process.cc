#include "common.h"
#include "image_data.h"
#include "image_handle.h"
#include "image_math.h"
#include "image_midline_process.h"
#include "transform_table.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// 功能: 计数值夹紧到 [0, PT_MAXLEN]
// 类型: 局部功能函数
// 关键参数: v-输入计数值
static inline int32_t clamp_count_i32(int32_t v)
{
    if (v < 0) return 0;
    if (v > PT_MAXLEN) return PT_MAXLEN;
    return v;
}

// 功能: 按 y 值截断/对齐点列起点
// 类型: 局部功能函数
// 关键参数: line/count-输入输出点列, start_y-期望起点 y
static inline void trim_to_start_y(float (&line)[PT_MAXLEN][2], int32_t* count, float start_y)
{
    if (count == nullptr)
    {
        return;
    }
    int32_t n = clamp_count_i32(*count);
    *count = n;
    if (n <= 0)
    {
        return;
    }

    // 找到一个“尽量接近 start_y”的起点；若存在 y>=start_y，则优先取第一个满足者
    int start_idx = 0;
    float best_abs = line[0][0] - start_y;
    if (best_abs < 0.0f) best_abs = -best_abs;

    int first_ge = -1;
    for (int i = 0; i < n; ++i)
    {
        const float y = line[i][0];
        if (first_ge < 0 && y >= start_y)
        {
            first_ge = i;
            break;
        }
        float a = y - start_y;
        if (a < 0.0f) a = -a;
        if (a < best_abs)
        {
            best_abs = a;
            start_idx = i;
        }
    }
    if (first_ge >= 0)
    {
        start_idx = first_ge;
    }

    if (start_idx <= 0)
    {
        return;
    }

    const int32_t out_n = n - start_idx;
    // 使用 memmove 进行就地搬移，比逐点拷贝更快
    std::memmove(&line[0], &line[start_idx], (size_t)out_n * sizeof(line[0]));
    *count = out_n;
}

// 功能: 复制点列（含长度夹紧）
// 类型: 局部功能函数
// 关键参数: dst/src-目标与来源点列, count-复制数量
static inline void copy_line(float (&dst)[PT_MAXLEN][2], const float (&src)[PT_MAXLEN][2], int32_t count)
{
    if (count <= 0)
    {
        return;
    }
    if (count > PT_MAXLEN)
    {
        count = PT_MAXLEN;
    }
    std::memcpy(&dst[0], &src[0], (size_t)count * sizeof(src[0]));
}

// 功能: 构造中线（基于左右候选中线）
// 类型: 图像处理函数
// 关键参数: mid_left/mid_right-候选中线(会原地修改), mode-跟线模式
void MID(float (&mid_left)[PT_MAXLEN][2], int32_t* mid_left_count,
                      float (&mid_right)[PT_MAXLEN][2], int32_t* mid_right_count,
                      float (&mid)[PT_MAXLEN][2], int32_t* mid_count,
                      FollowLine &mode)
{
    if (mid_left_count == nullptr || mid_right_count == nullptr || mid_count == nullptr)
    {
        return;
    }

    *mid_left_count = clamp_count_i32(*mid_left_count);
    *mid_right_count = clamp_count_i32(*mid_right_count);

    if (*mid_left_count == 0 && *mid_right_count == 0)
    {
        *mid_count = 0;
        return;
    }

    const float core_y = UndistInverseMapH[SET_IMAGE_CORE_Y][SET_IMAGE_CORE_X];

    //截头
    if(*mid_left_count > 0 && mid_left[0][0] > core_y)
    {
        int count1 = 0 ;
        for (int i = 0; i < *mid_left_count && count1 < PT_MAXLEN; i++)
        {
            if(mid_left[i][0] > core_y)
                continue;

            mid_left[count1][0] = mid_left[i][0];
            mid_left[count1][1] = mid_left[i][1];
            count1++;
        }
        *mid_left_count = count1;
    }

    if(*mid_right_count > 0 && mid_right[0][0] > core_y)
    {
        int count2 = 0 ;
        for (int i = 0; i < *mid_right_count && count2 < PT_MAXLEN; i++)
        {
            if(mid_right[i][0] > core_y)
                continue;

            mid_right[count2][0] = mid_right[i][0];
            mid_right[count2][1] = mid_right[i][1];
            count2++;
        }
        *mid_right_count = count2;
    }

    // “起点对齐”：三种模式下都做。
    // 目的：即使当前只跟左/右，也让两条候选中线同步到同一 y 起点，避免切换跟线模式时输出跳变。
    if (*mid_left_count > 0 && *mid_right_count > 0)
    {
        const float start_y = (mid_left[0][0] < mid_right[0][0]) ? mid_left[0][0] : mid_right[0][0];
        trim_to_start_y(mid_left, mid_left_count, start_y);
        trim_to_start_y(mid_right, mid_right_count, start_y);
    }

    // 防御：mode 异常时默认 MIXED
    if (mode != FollowLine::MIXED && mode != FollowLine::MIDLEFT && mode != FollowLine::MIDRIGHT)
    {
        mode = FollowLine::MIXED;
    }

    if(mode == FollowLine::MIXED)
    {        
        
        if (*mid_left_count == 0 && *mid_right_count == 0)
        {
            *mid_count = 0;
            return;
        }

        // 一侧缺失：直接使用另一侧，避免输出空中线导致控制跳变。
        if (*mid_left_count == 0)
        {
            const int n = clamp_count_i32(*mid_right_count);
            copy_line(mid, mid_right, n);
            *mid_count = n;
            return;
        }
        if (*mid_right_count == 0)
        {
            const int n = clamp_count_i32(*mid_left_count);
            copy_line(mid, mid_left, n);
            *mid_count = n;
            return;
        }

        const int min_count = (*mid_left_count < *mid_right_count) ? *mid_left_count : *mid_right_count;
        const bool pick_right_tail = (*mid_right_count > *mid_left_count);
        const float (*tail_src)[2] = pick_right_tail ? mid_right : mid_left;
        const int tail_count = pick_right_tail ? *mid_right_count : *mid_left_count;

        // MIXED 模式统一规则：
        // 1) 能一一对应的重合段先做混合平均
        // 2) 若重合平均段已经足够长，则不再接单边尾段，避免远端单边噪声拉偏中线
        // 3) 重合段较短时，才把点更多一侧的剩余段接到后面
        for (int i = 0; i < min_count; ++i)
        {
            mid[i][1] = (mid_left[i][1] + mid_right[i][1]) / 2.0f ;
            mid[i][0] = (mid_left[i][0] + mid_right[i][0]) / 2.0f ;
        }

        int out_count = min_count;
        if (min_count <= 50)
        {
            for (int i = min_count; i < tail_count && out_count < PT_MAXLEN; ++i)
            {
                mid[out_count][0] = tail_src[i][0];
                mid[out_count][1] = tail_src[i][1];
                ++out_count;
            }
        }
        *mid_count = out_count;
    }
    
    
    if(mode == FollowLine::MIDLEFT)
    {
        // 单边跟线：优先使用左侧候选中线。
        // 注意：mid_left/mid_right 本身已由 GetMidLine_Left/Right 按半车道宽“向内偏移”，
        // 因此它们都是“中心线估计”，缺失时可直接回退到另一侧。
        if (*mid_left_count > 0)
        {
            const int n = clamp_count_i32(*mid_left_count);
            copy_line(mid, mid_left, n);
            *mid_count = n;
            return;
        }
        if (*mid_right_count > 0 && element_type == ElementType::NORMAL)
        {
            const int n = clamp_count_i32(*mid_right_count);
            copy_line(mid, mid_right, n);
            *mid_count = n;
            return;
        }
        *mid_count = 0;
        return;
    }
    else if(mode == FollowLine::MIDRIGHT)
    {
        if (*mid_right_count > 0)
        {
            const int n = clamp_count_i32(*mid_right_count);
            copy_line(mid, mid_right, n);
            *mid_count = n;
            return;
        }
        if (*mid_left_count > 0 && element_type == ElementType::NORMAL)
        {
            const int n = clamp_count_i32(*mid_left_count);
            copy_line(mid, mid_left, n);
            *mid_count = n;
            return;
        }
        *mid_count = 0;
        return;
    }
}

// 功能: 根据中线局部几何，为 pure_angle 建议一个更前瞻的预瞄图像行
// 类型: 图像处理函数
// 关键参数: mid/mid_count-输入中线, base_img_y-默认预瞄图像行
// 说明:
// - 普通直道/缓弯：尽量保持在 base_img_y 附近，避免过分激进
// - 前方存在明显弯道：把预瞄行往更小的 y 推，让控制更早感知转向趋势
// - 当前版本使用“mid 分段 -> 局部转角 -> 短窗中位数 -> 最大稳健转角”作为连续几何量
static inline float curvature_to_angle_deg(float curvature)
{
    float cosv = 1.0f - curvature;
    if (cosv > 1.0f) cosv = 1.0f;
    else if (cosv < -1.0f) cosv = -1.0f;
    return acosf(cosv) * (180.0f / PI32);
}

static float compute_preview_robust_angle_from_mid_segment(const float (&mid)[PT_MAXLEN][2],
                                                           int32_t start_index,
                                                           int32_t segment_count)
{
    if (start_index < 0)
    {
        start_index = 0;
    }
    if (segment_count <= 0)
    {
        return 0.0f;
    }
    if (start_index >= PT_MAXLEN)
    {
        return 0.0f;
    }
    if (segment_count > PT_MAXLEN - start_index)
    {
        segment_count = PT_MAXLEN - start_index;
    }
    if (segment_count < 3)
    {
        return 0.0f;
    }

    float segment_mid[PT_MAXLEN][2] = {};
    for (int32_t i = 0; i < segment_count; ++i)
    {
        segment_mid[i][0] = mid[start_index + i][0];
        segment_mid[i][1] = mid[start_index + i][1];
    }

    float curvature[PT_MAXLEN] = {0.0f};
    int curvature_count = segment_count;
    local_curvature_points(segment_mid, &curvature_count,
                           curvature, &curvature_count,
                           PUREANGLE_PREVIEW_CURV_DIST);

    float angle_deg_chain[PT_MAXLEN] = {0.0f};
    for (int i = 0; i < curvature_count; ++i)
    {
        if (curvature[i] <= 0.0f)
        {
            continue;
        }

        float angle_deg = curvature_to_angle_deg(curvature[i]);
        if (angle_deg > PUREANGLE_PREVIEW_ANGLE_CLAMP_MAX)
        {
            angle_deg = PUREANGLE_PREVIEW_ANGLE_CLAMP_MAX;
        }
        angle_deg_chain[i] = angle_deg;
    }

    int robust_window = PUREANGLE_PREVIEW_ROBUST_WINDOW;
    if (robust_window < 1)
    {
        robust_window = 1;
    }
    if ((robust_window & 1) == 0)
    {
        robust_window += 1;
    }
    const int half_window = robust_window >> 1;

    float robust_angle_deg = 0.0f;
    for (int i = 0; i < curvature_count; ++i)
    {
        float window_angles[PT_MAXLEN];
        int window_count = 0;

        int j0 = i - half_window;
        if (j0 < 0)
        {
            j0 = 0;
        }
        int j1 = i + half_window;
        if (j1 > curvature_count - 1)
        {
            j1 = curvature_count - 1;
        }

        for (int j = j0; j <= j1; ++j)
        {
            window_angles[window_count] = angle_deg_chain[j];
            window_count++;
        }

        if (window_count <= 0)
        {
            continue;
        }

        std::sort(window_angles, window_angles + window_count);

        float window_angle_deg = 0.0f;
        if ((window_count & 1) != 0)
        {
            window_angle_deg = window_angles[window_count >> 1];
        }
        else
        {
            const int idx1 = (window_count >> 1) - 1;
            const int idx2 = (window_count >> 1);
            window_angle_deg = 0.5f * (window_angles[idx1] + window_angles[idx2]);
        }

        if (window_angle_deg > robust_angle_deg)
        {
            robust_angle_deg = window_angle_deg;
        }
    }

    return robust_angle_deg;
}

int MidLineSuggestPureAnglePreviewImageY(const float (&mid)[PT_MAXLEN][2], int32_t mid_count,
                                         int base_img_y,
                                         int32_t preview_curve_split_index)
{
    if (base_img_y < 0) base_img_y = 0;
    if (base_img_y > IMAGE_H - 1) base_img_y = IMAGE_H - 1;

    int preview_img_y_out = base_img_y;
    if (mid_count < 3)
    {
        preview_curve_angle_deg = 0.0f;
        return preview_img_y_out;
    }

    // 1) 中线局部转角链：
    //    默认整条 mid 作为单段处理。
    //    但在 MIXED 且存在“前段双边混合、后段单边延长”的情况下，
    //    不再跨越拼接缝直接算局部转角，而是拆成两段分别算，再取整体最大稳健转角。
    int total_count = (mid_count > PT_MAXLEN) ? PT_MAXLEN : (int)mid_count;
    float robust_angle_deg = 0.0f;
    if (preview_curve_split_index > 0 && preview_curve_split_index < total_count)
    {
        robust_angle_deg =
            compute_preview_robust_angle_from_mid_segment(mid, 0, preview_curve_split_index);

        const float tail_robust_angle_deg =
            compute_preview_robust_angle_from_mid_segment(mid,
                                                          preview_curve_split_index,
                                                          total_count - preview_curve_split_index);
        if (tail_robust_angle_deg > robust_angle_deg)
        {
            robust_angle_deg = tail_robust_angle_deg;
        }
    }
    else
    {
        robust_angle_deg =
            compute_preview_robust_angle_from_mid_segment(mid, 0, total_count);
    }

    preview_curve_angle_deg = robust_angle_deg;

    int shift = 0;
    const float curve_low_deg = PUREANGLE_PREVIEW_CURVE_LOW;
    float curve_high_deg = PUREANGLE_PREVIEW_CURVE_HIGH;
    if (curve_high_deg < curve_low_deg)
    {
        curve_high_deg = curve_low_deg;
    }
    if (curve_high_deg > curve_low_deg)
    {
        float t = (preview_curve_angle_deg - curve_low_deg) / (curve_high_deg - curve_low_deg);
        t = fclip(t, 0.0f, 1.0f);
        shift = (int)std::lroundf((float)PUREANGLE_PREVIEW_SHIFT_MAX * t);
    }
    else if (preview_curve_angle_deg >= curve_high_deg)
    {
        shift = PUREANGLE_PREVIEW_SHIFT_MAX;
    }

    preview_img_y_out = base_img_y - shift;
    if (preview_img_y_out < PUREANGLE_PREVIEW_MIN_IMAGE_Y)
    {
        preview_img_y_out = PUREANGLE_PREVIEW_MIN_IMAGE_Y;
    }
    if (preview_img_y_out > IMAGE_H - 1)
    {
        preview_img_y_out = IMAGE_H - 1;
    }
    return preview_img_y_out;
}

// 功能: 角度归一化到 [-PI, PI]
// 类型: 局部功能函数
// 关键参数: a-输入角度（弧度）
static inline float mid_norm_angle(float a)
{
    while (a > PI32) a -= 2.0f * PI32;
    while (a < -PI32) a += 2.0f * PI32;
    return a;
}

// 功能: 二维向量点乘
// 类型: 局部功能函数
// 关键参数: (ay,ax)/(by,bx)-输入向量
static inline float mid_dot(float ay, float ax, float by, float bx)
{
    return ay * by + ax * bx;
}

// 功能: 估计中线在 idx 处的切向量
// 类型: 局部功能函数
// 关键参数: mid/mid_count-中线点列, idx-索引, tan_y/tan_x-输出切向量
static bool mid_estimate_tangent(const float (&mid)[PT_MAXLEN][2], int32_t mid_count, int32_t idx,
                                float* tan_y, float* tan_x)
{
    if (tan_y == nullptr || tan_x == nullptr) return false;
    if (mid_count < 2 || idx < 0 || idx >= mid_count) return false;

    int32_t i0 = idx;
    int32_t i1 = idx;
    if (idx == 0)
    {
        i0 = 0;
        i1 = 1;
    }
    else if (idx == mid_count - 1)
    {
        i0 = mid_count - 2;
        i1 = mid_count - 1;
    }
    else
    {
        i0 = idx - 1;
        i1 = idx + 1;
    }

    float dy = mid[i1][0] - mid[i0][0];
    float dx = mid[i1][1] - mid[i0][1];
    float n = std::sqrt(dy * dy + dx * dx);
    if (n < 1e-6f) return false;
    *tan_y = dy / n;
    *tan_x = dx / n;
    return true;
}

// 功能: 已知切点与切向量、另一点，求圆心与半径
// 类型: 局部功能函数
// 关键参数: (py,px)-切点, (ty,tx)-切向量, (qy,qx)-另一点, cy/cx/radius-输出
static bool mid_arc_center_through_point_with_tangent(float py, float px, float ty, float tx,
                                                     float qy, float qx,
                                                     float* cy, float* cx, float* radius)
{
    if (cy == nullptr || cx == nullptr || radius == nullptr) return false;

    // 圆心在切点 P 的法线方向：C = P + n * R（n 为单位法向，±）。
    // 且 |C-Q| = R，可得 R = |Q-P|^2 / (2 * dot(n, Q-P))。
    float vy = qy - py;
    float vx = qx - px;
    float d2 = vy * vy + vx * vx;
    if (d2 < 1e-8f) return false;

    // 两个法向候选
    float n1y = -tx;
    float n1x = ty;
    float n2y = tx;
    float n2x = -ty;

    float den1 = 2.0f * mid_dot(n1y, n1x, vy, vx);
    float den2 = 2.0f * mid_dot(n2y, n2x, vy, vx);

    bool ok1 = std::fabs(den1) > 1e-6f;
    bool ok2 = std::fabs(den2) > 1e-6f;
    if (!ok1 && !ok2) return false;

    float r1 = ok1 ? (d2 / den1) : -1.0f;
    float r2 = ok2 ? (d2 / den2) : -1.0f;

    bool r1_pos = ok1 && (r1 > 0.0f);
    bool r2_pos = ok2 && (r2 > 0.0f);
    if (!r1_pos && !r2_pos) return false;

    bool pick1 = false;
    if (r1_pos && !r2_pos)
    {
        pick1 = true;
    }
    else if (!r1_pos && r2_pos)
    {
        pick1 = false;
    }
    else
    {
        // 都可用时取分母绝对值更大者更稳定
        pick1 = (std::fabs(den1) >= std::fabs(den2));
    }

    float ny = pick1 ? n1y : n2y;
    float nx = pick1 ? n1x : n2x;
    float r = pick1 ? r1 : r2;

    *cy = py + ny * r;
    *cx = px + nx * r;
    *radius = r;
    return true;
}

// 功能: 从 core 渐进融合并轨到中线并生成路径点列
// 类型: 图像处理函数
// 关键参数: mid/mid_count-中线点列, path/path_count-输出路径, step-采样间距(像素)
void BuildPathFromCoreToMidlineArc(const float (&mid)[PT_MAXLEN][2], int32_t mid_count,
                                  float (&path)[PT_MAXLEN][2], int32_t* path_count,
                                  float step)
{
    if (path_count == nullptr) return;
    *path_count = 0;
    if (mid_count <= 0) return;
    // step 为像素单位；默认用“0.02m 对应的像素距离”
    if (step <= 1e-6f) step = RESAMPLEDIST * PIXPERMETER;

    // core 点（逆透视后的世界坐标）
    const float core_y = UndistInverseMapH[SET_IMAGE_CORE_Y][SET_IMAGE_CORE_X];
    const float core_x = UndistInverseMapW[SET_IMAGE_CORE_Y][SET_IMAGE_CORE_X];

    // ===== 渐进融合并轨（随 idx 变化） =====
    // 目标：path 从 core 出发，逐步逼近中线；在 idx 处完全并入中线，后续与中线重合。
    // idx 的选取：取“y 接近某个前视位置”的中线点（空间一致，比分固定第 N 点更稳）。

    // 选一个图像行作为“前视参考”，由宏控制。
    int ref_img_y = PATH_BLEND_REF_IMAGE_Y;
    if (ref_img_y < 0) ref_img_y = 0;
    if (ref_img_y > IMAGE_H - 1) ref_img_y = IMAGE_H - 1;

    // 参考 y（世界/逆透视坐标系）。注意：y 应来自 H 表。
    const float ref_y = UndistInverseMapH[ref_img_y][SET_IMAGE_CORE_X];

    // 防御：表无效时回退到“第 10 个点/最后一个点”
    bool ref_valid = (ref_y >= 0.0f);
    int32_t idx_fallback = 9;
    if (idx_fallback >= mid_count) idx_fallback = mid_count - 1;
    if (idx_fallback < 0) return;

    int32_t target_idx = -1;
    if (ref_valid)
    {
        // 优先在“core 前方（y<=core_y）”里找最接近 ref_y 的点，避免误选到 core 后方。
        float best_abs = 1e30f;
        for (int32_t i = 0; i < mid_count; ++i)
        {
            if (mid[i][0] > core_y)
            {
                continue;
            }
            float a = mid[i][0] - ref_y;
            if (a < 0.0f) a = -a;
            if (a < best_abs)
            {
                best_abs = a;
                target_idx = i;
            }
        }

        // 若没找到（极端情况），放宽条件再找一次
        if (target_idx < 0)
        {
            best_abs = 1e30f;
            for (int32_t i = 0; i < mid_count; ++i)
            {
                float a = mid[i][0] - ref_y;
                if (a < 0.0f) a = -a;
                if (a < best_abs)
                {
                    best_abs = a;
                    target_idx = i;
                }
            }
        }
    }
    if (target_idx < 0)
    {
        target_idx = idx_fallback;
    }

    // 防御：target_idx 至少为 0
    if (target_idx < 0) target_idx = 0;
    if (target_idx > mid_count - 1) target_idx = mid_count - 1;

    const float end_y = mid[target_idx][0];
    const float end_x = mid[target_idx][1];

    int32_t w = 0;
    const float dup_thr2 = (0.25f * step) * (0.25f * step);

    // 1) 0..target_idx 渐进融合：
    // mid_base(i) = core + t*(end-core)，t=i/target_idx
    // path(i) = (1-t)*mid_base(i) + t*mid(i)
    if (target_idx <= 0)
    {
        // 退化：只有一个融合点，直接从 core 接到 mid
        path[w][0] = core_y;
        path[w][1] = core_x;
        ++w;

        for (int32_t i = 0; i < mid_count && w < PT_MAXLEN; ++i)
        {
            // 避免与 core 重复
            const float dy = mid[i][0] - path[w - 1][0];
            const float dx = mid[i][1] - path[w - 1][1];
            if (dy * dy + dx * dx <= dup_thr2)
            {
                continue;
            }
            path[w][0] = mid[i][0];
            path[w][1] = mid[i][1];
            ++w;
        }
        *path_count = w;

        GetLinesResample(path, path_count, path, path_count, step, nullptr);

        return;
    }

    for (int32_t i = 0; i <= target_idx && w < PT_MAXLEN; ++i)
    {
        const float t = (float)i / (float)target_idx; // 0..1

        const float base_y = core_y + (end_y - core_y) * t;
        const float base_x = core_x + (end_x - core_x) * t;

        const float out_y = base_y * (1.0f - t) + mid[i][0] * t;
        const float out_x = base_x * (1.0f - t) + mid[i][1] * t;

        if (w > 0)
        {
            const float dy = out_y - path[w - 1][0];
            const float dx = out_x - path[w - 1][1];
            if (dy * dy + dx * dx <= dup_thr2)
            {
                continue;
            }
        }

        path[w][0] = out_y;
        path[w][1] = out_x;
        ++w;
    }

    // 2) target_idx 之后：直接贴中线（避免重复 target_idx）
    for (int32_t i = target_idx + 1; i < mid_count && w < PT_MAXLEN; ++i)
    {
        const float dy = mid[i][0] - path[w - 1][0];
        const float dx = mid[i][1] - path[w - 1][1];
        if (dy * dy + dx * dx <= dup_thr2)
        {
            continue;
        }
        path[w][0] = mid[i][0];
        path[w][1] = mid[i][1];
        ++w;
    }

    *path_count = w;

    GetLinesResample(path, path_count, path, path_count, step, nullptr);


}
