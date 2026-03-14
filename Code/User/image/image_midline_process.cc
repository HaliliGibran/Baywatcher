#include "image_headfile.h"
#include <cstring>
#include <cmath>

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

        int min_count = (*mid_left_count < *mid_right_count) ? *mid_left_count : *mid_right_count ;
        if(min_count < MIXED_POINT_NUM_THRESHOLD)
        {
            // 重合段太短时，混合平均不可靠：选点更多的一侧。
            const bool pick_right = (*mid_right_count >= *mid_left_count);
            const int n = pick_right ? clamp_count_i32(*mid_right_count) : clamp_count_i32(*mid_left_count);
            if (pick_right)
            {
                copy_line(mid, mid_right, n);
            }
            else
            {
                copy_line(mid, mid_left, n);
            }
            *mid_count = n;
            return;
        }

        for(int i = 0 ; i < min_count ; ++i)
        {
            mid[i][1] = (mid_left[i][1] + mid_right[i][1]) / 2.0f ;
            mid[i][0] = (mid_left[i][0] + mid_right[i][0]) / 2.0f ;
        }
        *mid_count = min_count ;
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








// 功能: 计算带符号的曲率强度 k
// 类型: 局部功能函数
// 关键参数: span-取向量跨度, k_out-输出曲率强度
// - k = sign(cross) * sin^2(theta)，score=sin^2(theta)=cross^2/(|v1|^2|v2|^2)
static void local_signed_curvature(const float (&pts)[PT_MAXLEN][2], int n, int span,
                                   float (&k_out)[PT_MAXLEN])
{
    if (n <= 0)
    {
        return;
    }

    if (n > PT_MAXLEN)
    {
        n = PT_MAXLEN;
    }

    if (span <= 0)
    {
        for (int i = 0; i < n; ++i)
        {
            k_out[i] = 0.0f;
        }
        return;
    }

    for (int i = 0; i < n; ++i)
    {
        if (i == 0 || i >= n - 1)
        {
            k_out[i] = 0.0f;
            continue;
        }

        int i1 = i - span;
        if (i1 < 0)
        {
            i1 = 0;
        }
        int i2 = i + span;
        if (i2 > n - 1)
        {
            i2 = n - 1;
        }

        const float dx1 = pts[i][1] - pts[i1][1];
        const float dy1 = pts[i][0] - pts[i1][0];
        const float dx2 = pts[i2][1] - pts[i][1];
        const float dy2 = pts[i2][0] - pts[i][0];

        const float n1 = dx1 * dx1 + dy1 * dy1;
        const float n2 = dx2 * dx2 + dy2 * dy2;
        const float den = n1 * n2;
        if (den <= 1e-12f)
        {
            k_out[i] = 0.0f;
            continue;
        }

        const float cross = dx1 * dy2 - dy1 * dx2;
        float score = (cross * cross) / den; // sin^2
        if (score < 0.0f)
        {
            score = 0.0f;
        }
        else if (score > 1.0f)
        {
            score = 1.0f;
        }
        const float sgn = (cross >= 0.0f) ? 1.0f : -1.0f;
        k_out[i] = sgn * score;
    }
}


// 功能: 3x3 线性方程组求解（高斯消元）
// 类型: 局部功能函数
// 关键参数: A/b-输入矩阵与向量, x-输出解
static bool solve_3x3(float A[3][3], float b[3], float x[3])
{
    for (int col = 0; col < 3; ++col)
    {
        // 选主元：找当前列绝对值最大的行
        int pivot = col;
        float max_abs = A[col][col];
        if (max_abs < 0.0f)
        {
            max_abs = -max_abs;
        }
        for (int r = col + 1; r < 3; ++r)
        {
            float v = A[r][col];
            if (v < 0.0f)
            {
                v = -v;
            }
            if (v > max_abs)
            {
                max_abs = v;
                pivot = r;
            }
        }
        if (max_abs <= 1e-12f)
        {
            return false;
        }
        if (pivot != col)
        {
            for (int c = col; c < 3; ++c)
            {
                float tmp = A[col][c];
                A[col][c] = A[pivot][c];
                A[pivot][c] = tmp;
            }
            float tb = b[col];
            b[col] = b[pivot];
            b[pivot] = tb;
        }

        // 消元
        const float inv_p = 1.0f / A[col][col];
        for (int r = col + 1; r < 3; ++r)
        {
            const float f = A[r][col] * inv_p;
            if (f == 0.0f)
            {
                continue;
            }
            A[r][col] = 0.0f;
            for (int c = col + 1; c < 3; ++c)
            {
                A[r][c] -= f * A[col][c];
            }
            b[r] -= f * b[col];
        }
    }

    // 回代
    for (int i = 2; i >= 0; --i)
    {
        float s = b[i];
        for (int c = i + 1; c < 3; ++c)
        {
            s -= A[i][c] * x[c];
        }
        x[i] = s / A[i][i];
    }
    return true;
}


// 功能: 圆弧拟合（Kasa 代数最小二乘）
// 类型: 局部功能函数
// 关键参数: pts/start/end-拟合点段, cx/cy/r2-输出圆心与半径平方
// 圆弧拟合（Kåsa algebraic fit, least squares）：
// 模型：x^2 + y^2 = 2*cx*x + 2*cy*y + c
// 通过最小二乘解 (2cx, 2cy, c)，再得到 R^2 = cx^2 + cy^2 + c。
// 说明：
// - 这是“代数拟合”，对噪声较稳且实现简单，适合 PT_MAXLEN 小点集快速抽象。
// - 为避免 sqrt，我们用 (d^2 - R^2)^2 作为残差度量。
static bool fit_circle_kasa(const float (&pts)[PT_MAXLEN][2], int start, int end,
                            float* cx, float* cy, float* r2, float* rel_err)
{
    const int n = end - start;
    if (n < 3)
    {
        return false;
    }

    double Sx = 0.0, Sy = 0.0;
    double Sxx = 0.0, Syy = 0.0, Sxy = 0.0;
    double Sz = 0.0, Sxz = 0.0, Syz = 0.0;

    for (int i = start; i < end; ++i)
    {
        const double x = (double)pts[i][1];
        const double y = (double)pts[i][0];
        const double z = x * x + y * y;

        Sx += x;
        Sy += y;
        Sxx += x * x;
        Syy += y * y;
        Sxy += x * y;
        Sz += z;
        Sxz += x * z;
        Syz += y * z;
    }

    // 正规方程：
    // [Sxx Sxy Sx] [A] = [Sxz]
    // [Sxy Syy Sy] [B]   [Syz]
    // [Sx  Sy  n ] [C]   [Sz ]
    // 其中 A=2cx, B=2cy
    float A[3][3] = {
        {(float)Sxx, (float)Sxy, (float)Sx},
        {(float)Sxy, (float)Syy, (float)Sy},
        {(float)Sx,  (float)Sy,  (float)n}
    };
    float b[3] = {(float)Sxz, (float)Syz, (float)Sz};
    float sol[3] = {0.0f, 0.0f, 0.0f};
    if (!solve_3x3(A, b, sol))
    {
        return false;
    }

    const float A0 = sol[0];
    const float B0 = sol[1];
    const float C0 = sol[2];

    const float ccx = 0.5f * A0;
    const float ccy = 0.5f * B0;
    const float rr2 = ccx * ccx + ccy * ccy + C0;
    if (rr2 <= 1e-6f)
    {
        return false;
    }

    // 残差：mean( (d^2 - R^2)^2 ) / R^4，越小越好。
    double err = 0.0;
    for (int i = start; i < end; ++i)
    {
        const double x = (double)pts[i][1] - (double)ccx;
        const double y = (double)pts[i][0] - (double)ccy;
        const double d2 = x * x + y * y;
        const double e = d2 - (double)rr2;
        err += e * e;
    }

    if (cx) *cx = ccx;
    if (cy) *cy = ccy;
    if (r2) *r2 = rr2;
    if (rel_err)
    {
        const double denom = (double)rr2 * (double)rr2;
        *rel_err = (float)(err / (denom * (double)n + 1e-12));
    }
    return true;
}


// 功能: 中线几何抽象（直线/圆弧拟合并等距采样）
// 类型: 图像处理函数
// 关键参数: mid_in/mid_in_count-输入中线, mid_out/mid_out_count-输出中线, dist-采样步长
// 对中线做几何抽象（面向后续控制/几何理解）：
//
// 目标：把“抖动的等距点列”变成“更几何化、更可信”的点列：
// - 近似直线：拟合为一条直线，再按 dist 等距取点（去掉噪声对法向/角度的影响）
// - 近似圆弧：拟合为一段圆弧，再按 dist（弧长步长）等距取点（用于圆环/连续弯更稳）
//
// 流程：
// 1) 先整体严格等距重采样到 dist（保证后续分段/拟合不受点密度不均影响）
// 2) 计算带符号曲率强度 k（k = sign(cross) * sin^2(theta)），再做窗口平均平滑压制少量离群点
// 3) 用 |k| 与 straight_thr (sin^2(straight_angle)) 做直/曲初分段，并合并短 run，避免抖动切换
// 4) 逐段处理：
//    4.1 直线段：用 2x2 协方差最大特征向量作为方向（等价 PCA），再按 dist 重采样
//    4.2 曲线段：对段内点做圆拟合（Kåsa），若拟合好则按 dist 采样圆弧，否则回退保留原等距点
void MidLineGeometryAbstract(const float (&mid_in)[PT_MAXLEN][2], int* mid_in_count,
                             float (&mid_out)[PT_MAXLEN][2], int* mid_out_count,
                             float dist, int curvature_span, int smooth_halfwin,
                             float straight_angle_deg, int min_run_len)
{
    if (mid_out_count == nullptr)
    {
        return;
    }
    if (mid_in_count == nullptr || *mid_in_count <= 0 || dist <= 0.0f)
    {
        *mid_out_count = 0;
        return;
    }

    // 1) 整体重采样（严格 dist）
    // 说明：这里复用 GetLinesResample 的“严格等距”步进策略。
    // 注意：GetLinesResample 不强行补尾点（保证相邻点间距严格为 dist），所以 res_n 可能比输入少 1。
    float resampled[PT_MAXLEN][2];
    int res_n = 0;
    float tmp_in[PT_MAXLEN][2];
    int tmp_n = *mid_in_count;
    if (tmp_n > PT_MAXLEN)
    {
        tmp_n = PT_MAXLEN;
    }
    for (int i = 0; i < tmp_n; ++i)
    {
        tmp_in[i][0] = mid_in[i][0];
        tmp_in[i][1] = mid_in[i][1];
    }
    GetLinesResample(tmp_in, &tmp_n, resampled, &res_n, dist, nullptr);
    if (res_n <= 0)
    {
        *mid_out_count = 0;
        return;
    }

    // 2) 曲率强度（带符号） + 轻平滑（窗口平均，压制个别离群点）
    // k 的绝对值表示“弯的强度”（0..1），符号表示“整体左右方向”（坐标系相关，但一致即可）。
    float k[PT_MAXLEN];
    float k_smooth[PT_MAXLEN];
    local_signed_curvature(resampled, res_n, curvature_span, k);

    const int hw = (smooth_halfwin > 0) ? smooth_halfwin : 0;
    for (int i = 0; i < res_n; ++i)
    {
        if (hw == 0)
        {
            k_smooth[i] = k[i];
            continue;
        }

        int s = i - hw;
        if (s < 0)
        {
            s = 0;
        }
        int e = i + hw;
        if (e > res_n - 1)
        {
            e = res_n - 1;
        }

        float sum = 0.0f;
        for (int j = s; j <= e; ++j)
        {
            sum += k[j];
        }
        k_smooth[i] = sum / (float)(e - s + 1);
    }

    // 直线阈值：用 sin^2(theta)
    // 若把“直线”定义为 |theta| < straight_angle，则对应 |sin(theta)| < sin(straight_angle)，
    // 这里直接用 sin^2 规避 atan/acos 等反三角：
    //   |k| = sin^2(theta) < sin^2(straight_angle)
    float straight_thr = 0.0f;
    if (straight_angle_deg > 0.0f)
    {
        const float rad = straight_angle_deg * (PI32 / 180.0f);
        const float s = sinf(rad);
        straight_thr = s * s;
    }

    const int min_len = (min_run_len > 1) ? min_run_len : 2;

    // 初始标签：abs(k_smooth) < thr => straight, else curve
    uint8_t label[PT_MAXLEN];
    for (int i = 0; i < res_n; ++i)
    {
        float a = k_smooth[i];
        if (a < 0.0f)
        {
            a = -a;
        }
        label[i] = (a < straight_thr) ? 0 : 1;
    }

    // 小段合并：长度 < min_len 的 run 并入相邻更长的 run
    for (int pass = 0; pass < 2; ++pass)
    {
        int i = 0;
        while (i < res_n)
        {
            const uint8_t cur = label[i];
            int j = i + 1;
            while (j < res_n && label[j] == cur)
            {
                ++j;
            }
            const int len = j - i;
            if (len < min_len)
            {
                const uint8_t left = (i > 0) ? label[i - 1] : cur;
                const uint8_t right = (j < res_n) ? label[j] : cur;
                const uint8_t fill = (left == right) ? left : right;
                for (int t = i; t < j; ++t)
                {
                    label[t] = fill;
                }
            }
            i = j;
        }
    }

    // 3/4) 输出：分段处理
    // 说明：
    // - push_point 做了“连续重复点”保护，避免拟合/采样时插入 0 距离点。
    // - 为保持“等距 dist”的一致性：直线/圆弧采样都不强行补齐段尾点。
    int out_n = 0;
    auto push_point = [&](float y, float x)
    {
        if (out_n >= PT_MAXLEN)
        {
            return;
        }
        if (out_n > 0)
        {
            const float dx = x - mid_out[out_n - 1][1];
            const float dy = y - mid_out[out_n - 1][0];
            if (dx * dx + dy * dy <= 1e-10f)
            {
                return;
            }
        }
        mid_out[out_n][0] = y;
        mid_out[out_n][1] = x;
        ++out_n;
    };

    int i = 0;
    while (i < res_n && out_n < PT_MAXLEN)
    {
        const uint8_t cur = label[i];
        int j = i + 1;
        while (j < res_n && label[j] == cur)
        {
            ++j;
        }

        const int seg_len = j - i;
        if (cur == 0 && seg_len >= 2)
        {
            // 直线段：用 PCA 拟合方向（等价：求 2x2 协方差矩阵最大特征向量）
            // 设点偏移为 (dx,dy)，协方差矩阵：
            //   [ sxx sxy ]
            //   [ sxy syy ]
            // 最大特征向量即“方差最大方向”，可认为是拟合直线方向。
            double mx = 0.0;
            double my = 0.0;
            for (int t = i; t < j; ++t)
            {
                mx += resampled[t][1];
                my += resampled[t][0];
            }
            mx /= (double)seg_len;
            my /= (double)seg_len;

            double sxx = 0.0;
            double sxy = 0.0;
            double syy = 0.0;
            for (int t = i; t < j; ++t)
            {
                const double dx = (double)resampled[t][1] - mx;
                const double dy = (double)resampled[t][0] - my;
                sxx += dx * dx;
                sxy += dx * dy;
                syy += dy * dy;
            }

            // 用闭式解避免三角函数：
            // 对称矩阵 [[a,b],[b,c]]，特征值：
            //   λ = 0.5*(a+c ± sqrt((a-c)^2 + 4b^2))
            // 取最大特征值 λ1 后，其特征向量可取 v=(b, λ1-a) 或 v=(λ1-c, b)。
            const float a = (float)sxx;
            const float b = (float)sxy;
            const float c = (float)syy;
            const float disc = (a - c) * (a - c) + 4.0f * b * b;
            const float sqrt_disc = Q_sqrt(disc);
            const float lambda1 = 0.5f * (a + c + sqrt_disc);

            float dirx = b;
            float diry = lambda1 - a;
            if ((dirx * dirx + diry * diry) <= 1e-12f)
            {
                dirx = lambda1 - c;
                diry = b;
            }
            const float vlen2 = dirx * dirx + diry * diry;
            float inv_vlen = fast_rsqrt(vlen2);
            inv_vlen = inv_vlen * (1.5f - 0.5f * vlen2 * inv_vlen * inv_vlen);
            dirx *= inv_vlen;
            diry *= inv_vlen;

            // 方向朝向从段首到段尾
            const float ex = resampled[j - 1][1] - resampled[i][1];
            const float ey = resampled[j - 1][0] - resampled[i][0];
            if (dirx * ex + diry * ey < 0.0f)
            {
                dirx = -dirx;
                diry = -diry;
            }

            const float sx = resampled[i][1];
            const float sy = resampled[i][0];
            push_point(sy, sx);

            // 沿拟合方向走到段尾投影长度（不强行补尾点）
            const float t_end = ex * dirx + ey * diry;
            float walked = dist;
            const float eps = 1e-6f;
            while (walked + eps <= t_end && out_n < PT_MAXLEN)
            {
                push_point(sy + diry * walked, sx + dirx * walked);
                walked += dist;
            }
        }
        else
        {
            // 曲线段：优先做“圆弧抽象”，失败则回退保留等距点
            //
            // 圆弧抽象的意义：
            // - 连续弯/圆环里，点列噪声会导致法向/曲率抖动，直接保留点会让后续几何判断不稳。
            // - 用“拟合圆弧 + 等距采样”可以得到更干净、更一致的中线几何。
            //
            // 拟合条件：点数>=3 且拟合相对误差较小。
            float cx = 0.0f, cy = 0.0f, rr2 = 0.0f, err = 0.0f;
            bool ok = (seg_len >= 3) && fit_circle_kasa(resampled, i, j, &cx, &cy, &rr2, &err);

            // 经验阈值：代数拟合的相对误差越小越接近圆
            // 这里取 1e-2~1e-3 都可，根据你的噪声水平调。
            const float max_rel_err = 1e-2f;
            if (!ok || err > max_rel_err)
            {
                // 回退：保留原等距点
                for (int t = i; t < j && out_n < PT_MAXLEN; ++t)
                {
                    push_point(resampled[t][0], resampled[t][1]);
                }
            }
            else
            {
                // 以段首点作为圆弧起点，拟合半径 R，角步长 dθ = dist / R（dist 为弧长步长）
                float inv_r = fast_rsqrt(rr2);
                inv_r = inv_r * (1.5f - 0.5f * rr2 * inv_r * inv_r);
                const float dtheta = dist * inv_r;

                // 若 dtheta 太小或太大，圆弧采样不稳定，回退
                if (dtheta <= 1e-6f || dtheta > 1.0f)
                {
                    for (int t = i; t < j && out_n < PT_MAXLEN; ++t)
                    {
                        push_point(resampled[t][0], resampled[t][1]);
                    }
                }
                else
                {
                    // 预计算一次旋转（每步旋转 dtheta），避免每点调用 sin/cos
                    const float cosd = cosf(dtheta);
                    const float sind = sinf(dtheta);

                    // 起始半径向量 v0 = P0 - C
                    const float sx = resampled[i][1];
                    const float sy = resampled[i][0];
                    float vx0 = sx - cx;
                    float vy0 = sy - cy;

                    // 目标段尾点，用于选择旋转方向（+dtheta 或 -dtheta）
                    const float exx = resampled[j - 1][1];
                    const float eyy = resampled[j - 1][0];

                    // 旋转 steps 次后，哪个方向更接近段尾，就选哪个方向。
                    const int steps = seg_len - 1;
                    auto rotate_steps = [&](float vx, float vy, float sgn, float* outx, float* outy)
                    {
                        const float s = (sgn >= 0.0f) ? sind : -sind;
                        for (int t = 0; t < steps; ++t)
                        {
                            const float nx = vx * cosd - vy * s;
                            const float ny = vx * s + vy * cosd;
                            vx = nx;
                            vy = ny;
                        }
                        *outx = vx;
                        *outy = vy;
                    };

                    float vpx, vpy, vmx, vmy;
                    rotate_steps(vx0, vy0, +1.0f, &vpx, &vpy);
                    rotate_steps(vx0, vy0, -1.0f, &vmx, &vmy);

                    const float px_end = cx + vpx;
                    const float py_end = cy + vpy;
                    const float mx_end = cx + vmx;
                    const float my_end = cy + vmy;

                    const float dpe2 = (px_end - exx) * (px_end - exx) + (py_end - eyy) * (py_end - eyy);
                    const float dme2 = (mx_end - exx) * (mx_end - exx) + (my_end - eyy) * (my_end - eyy);
                    const float sgn = (dpe2 <= dme2) ? 1.0f : -1.0f;
                    const float s = (sgn >= 0.0f) ? sind : -sind;

                    // 输出圆弧点：P(t) = C + R * rot(t*dθ) * v0
                    // 注意：不强行补尾点，输出点数约为 seg_len。
                    push_point(sy, sx);
                    float vx = vx0;
                    float vy = vy0;
                    for (int t = 0; t < steps && out_n < PT_MAXLEN; ++t)
                    {
                        const float nx = vx * cosd - vy * s;
                        const float ny = vx * s + vy * cosd;
                        vx = nx;
                        vy = ny;
                        push_point(cy + vy, cx + vx);
                    }
                }
            }
        }

        i = j;
    }

    *mid_out_count = out_n;
}


// =========================
// 赛道几何“段级抽象/判别”
// =========================
// 说明（世界坐标系 / 米制）：
// - 这里假设 mid_in 已经是在逆透视后的世界坐标（单位米），x/y 的正方向取决于你的标定约定。
// - “顺/逆时针”的符号来自 local_signed_curvature 的 cross 符号：它与坐标系手性有关。
//   你只要保证符号一致即可：比如在你的坐标系里 cross<0 被你定义为“右转”。

enum MidTrackPrimitiveType
{
    MID_PRIM_LINE = 0,
    MID_PRIM_ARC  = 1
};

enum MidTrackState
{
    MID_TRACK_UNKNOWN = 0,
    MID_TRACK_RIGHT_ANGLE,
    MID_TRACK_LEFT_ANGLE,
    MID_TRACK_S_CURVE,
    MID_TRACK_ROUNDABOUT
};

// 一个几何段（原语）的最小描述
struct MidTrackPrimitive
{
    int type;           // MidTrackPrimitiveType
    int start;          // 在重采样点列中的起点 index（含）
    int end;            // 终点 index（不含）
    float len_m;        // 段长度（米），line 用投影长度，arc 用弧长近似

    // 直线段：单位方向向量（世界坐标）
    float dir_x;
    float dir_y;

    // 圆弧段：拟合圆心/半径/扫角
    float cx;
    float cy;
    float radius_m;
    float sweep_rad;    // 有符号扫角（弧度）；符号代表转向方向（坐标系相关）
    float fit_rel_err;  // 圆拟合相对误差（越小越接近圆）

    // 曲率方向投票：>0 / <0
    float turn_vote;
};


// 从中线点列提取“直线/圆弧”段（用于后续模式识别/状态机）
// - 复用与 MidLineGeometryAbstract 相同的分段方式（曲率强度 sin^2 + 平滑 + 短段合并）
// - 每段输出：类型、长度、方向/圆心半径、扫角、拟合误差、转向投票
// 功能: 从中线提取几何原语（线段/圆弧）
// 类型: 局部功能函数
// 关键参数: mid_in/mid_in_count-输入中线, prim/prim_count-输出原语
static __attribute__((unused)) void MidLineExtractPrimitives(const float (&mid_in)[PT_MAXLEN][2], int* mid_in_count,
                                                             MidTrackPrimitive* prim, int* prim_count,
                                                             float dist, int curvature_span, int smooth_halfwin,
                                                             float straight_angle_deg, int min_run_len)
{
    if (prim_count == nullptr)
    {
        return;
    }
    *prim_count = 0;
    if (mid_in_count == nullptr || *mid_in_count <= 0 || prim == nullptr || dist <= 0.0f)
    {
        return;
    }

    // 1) 统一点密度：严格等距重采样
    float resampled[PT_MAXLEN][2];
    int res_n = 0;
    float tmp_in[PT_MAXLEN][2];
    int tmp_n = *mid_in_count;
    if (tmp_n > PT_MAXLEN)
    {
        tmp_n = PT_MAXLEN;
    }
    for (int i = 0; i < tmp_n; ++i)
    {
        tmp_in[i][0] = mid_in[i][0];
        tmp_in[i][1] = mid_in[i][1];
    }
    GetLinesResample(tmp_in, &tmp_n, resampled, &res_n, dist, nullptr);
    if (res_n < 3)
    {
        return;
    }

    // 2) 带符号曲率强度 k（sin^2），并做窗口平均平滑
    float k[PT_MAXLEN];
    float k_smooth[PT_MAXLEN];
    local_signed_curvature(resampled, res_n, curvature_span, k);

    const int hw = (smooth_halfwin > 0) ? smooth_halfwin : 0;
    for (int i = 0; i < res_n; ++i)
    {
        if (hw == 0)
        {
            k_smooth[i] = k[i];
            continue;
        }
        int s = i - hw;
        if (s < 0) s = 0;
        int e = i + hw;
        if (e > res_n - 1) e = res_n - 1;
        float sum = 0.0f;
        for (int j = s; j <= e; ++j)
        {
            sum += k[j];
        }
        k_smooth[i] = sum / (float)(e - s + 1);
    }

    // 3) 直线阈值：sin^2(straight_angle)
    float straight_thr = 0.0f;
    if (straight_angle_deg > 0.0f)
    {
        const float rad = straight_angle_deg * (PI32 / 180.0f);
        const float s = sinf(rad);
        straight_thr = s * s;
    }

    const int min_len = (min_run_len > 1) ? min_run_len : 2;

    uint8_t label[PT_MAXLEN];
    for (int i = 0; i < res_n; ++i)
    {
        float a = k_smooth[i];
        if (a < 0.0f) a = -a;
        label[i] = (a < straight_thr) ? 0 : 1;
    }

    for (int pass = 0; pass < 2; ++pass)
    {
        int i = 0;
        while (i < res_n)
        {
            const uint8_t cur = label[i];
            int j = i + 1;
            while (j < res_n && label[j] == cur)
            {
                ++j;
            }
            const int len = j - i;
            if (len < min_len)
            {
                const uint8_t left = (i > 0) ? label[i - 1] : cur;
                const uint8_t right = (j < res_n) ? label[j] : cur;
                const uint8_t fill = (left == right) ? left : right;
                for (int t = i; t < j; ++t)
                {
                    label[t] = fill;
                }
            }
            i = j;
        }
    }

    // 4) 输出段
    int out = 0;
    int i = 0;
    while (i < res_n && out < 16) // 近端通常段数很少，16 足够
    {
        const uint8_t cur = label[i];
        int j = i + 1;
        while (j < res_n && label[j] == cur)
        {
            ++j;
        }

        MidTrackPrimitive p = {};
        p.start = i;
        p.end = j;
        p.turn_vote = 0.0f;

        // 段内转向投票（忽略端点）
        for (int t = i + 1; t < j - 1; ++t)
        {
            p.turn_vote += k_smooth[t];
        }

        const int seg_len = j - i;
        if (cur == 0 && seg_len >= 2)
        {
            p.type = MID_PRIM_LINE;

            // PCA 方向（与 MidLineGeometryAbstract 一致）
            double mx = 0.0;
            double my = 0.0;
            for (int t = i; t < j; ++t)
            {
                mx += resampled[t][1];
                my += resampled[t][0];
            }
            mx /= (double)seg_len;
            my /= (double)seg_len;

            double sxx = 0.0;
            double sxy = 0.0;
            double syy = 0.0;
            for (int t = i; t < j; ++t)
            {
                const double dx = (double)resampled[t][1] - mx;
                const double dy = (double)resampled[t][0] - my;
                sxx += dx * dx;
                sxy += dx * dy;
                syy += dy * dy;
            }

            const float a = (float)sxx;
            const float b = (float)sxy;
            const float c = (float)syy;
            const float disc = (a - c) * (a - c) + 4.0f * b * b;
            const float sqrt_disc = Q_sqrt(disc);
            const float lambda1 = 0.5f * (a + c + sqrt_disc);

            float dirx = b;
            float diry = lambda1 - a;
            if ((dirx * dirx + diry * diry) <= 1e-12f)
            {
                dirx = lambda1 - c;
                diry = b;
            }
            const float vlen2 = dirx * dirx + diry * diry;
            float inv_vlen = fast_rsqrt(vlen2);
            inv_vlen = inv_vlen * (1.5f - 0.5f * vlen2 * inv_vlen * inv_vlen);
            dirx *= inv_vlen;
            diry *= inv_vlen;

            const float ex = resampled[j - 1][1] - resampled[i][1];
            const float ey = resampled[j - 1][0] - resampled[i][0];
            if (dirx * ex + diry * ey < 0.0f)
            {
                dirx = -dirx;
                diry = -diry;
            }

            p.dir_x = dirx;
            p.dir_y = diry;
            p.len_m = ex * dirx + ey * diry;
            if (p.len_m < 0.0f) p.len_m = -p.len_m;
        }
        else
        {
            p.type = MID_PRIM_ARC;
            p.dir_x = 0.0f;
            p.dir_y = 0.0f;

            float cx = 0.0f, cy = 0.0f, rr2 = 0.0f, err = 1e30f;
            bool ok = (seg_len >= 3) && fit_circle_kasa(resampled, i, j, &cx, &cy, &rr2, &err);
            p.cx = cx;
            p.cy = cy;
            p.fit_rel_err = err;
            if (!ok)
            {
                p.radius_m = 0.0f;
                p.sweep_rad = 0.0f;
                p.len_m = (float)(seg_len - 1) * dist;
            }
            else
            {
                p.radius_m = Q_sqrt(rr2);

                // 扫角：atan2( v0 x v1, v0 · v1 )
                const float v0x = resampled[i][1] - cx;
                const float v0y = resampled[i][0] - cy;
                const float v1x = resampled[j - 1][1] - cx;
                const float v1y = resampled[j - 1][0] - cy;
                const float cr = v0x * v1y - v0y * v1x;
                const float dt = v0x * v1x + v0y * v1y;
                float sweep = atan2f(cr, dt);

                // 用段内转向投票统一 sweep 的符号（抑制端点噪声）
                if (p.turn_vote < 0.0f)
                {
                    if (sweep > 0.0f) sweep = -sweep;
                }
                else if (p.turn_vote > 0.0f)
                {
                    if (sweep < 0.0f) sweep = -sweep;
                }
                p.sweep_rad = sweep;

                float abs_sweep = sweep;
                if (abs_sweep < 0.0f) abs_sweep = -abs_sweep;
                p.len_m = p.radius_m * abs_sweep;
            }
        }

        prim[out] = p;
        ++out;
        i = j;
    }

    *prim_count = out;
}


// 基于段级原语做赛道状态判别（单帧）。
// 建议你在外层再加“连续帧确认/滞回”，避免高速弯/丢边造成状态抖动。
// 功能: 赛道状态判别（原语序列）
// 类型: 局部功能函数
// 关键参数: prim/prim_count-输入原语, out_turn_deg-输出转角估计(度)
static MidTrackState MidLineClassifyTrackEx(const MidTrackPrimitive* prim, int prim_count, float* out_turn_deg);

// 扩展版：输出“左右拐大概多少度”（单位：度）。
// - 约定：右转为负，左转为正（由 ARC sweep 符号决定，坐标系相关）。
// - 角度大小以“两段直线夹角”为主（更稳），圆弧扫角仅用于辅助确认。
// 功能: 赛道状态判别（扩展版）
// 类型: 局部功能函数
// 关键参数: prim/prim_count-输入原语, out_turn_deg-输出转角估计(度)
static MidTrackState MidLineClassifyTrackEx(const MidTrackPrimitive* prim, int prim_count, float* out_turn_deg)
{
    if (out_turn_deg != nullptr)
    {
        *out_turn_deg = 0.0f;
    }
    if (prim == nullptr || prim_count <= 0)
    {
        return MID_TRACK_UNKNOWN;
    }

    // ===== 环岛：长圆弧主导 + 拟合误差小 + 扫角较大 =====
    // 米制阈值建议：
    // - sweep > 120°（2.094rad）更像“在环岛内”，如果你只看近端，可以放宽到 90°。
    // - 半径范围需根据你的赛道标定调整。
    for (int i = 0; i < prim_count; ++i)
    {
        if (prim[i].type != MID_PRIM_ARC)
        {
            continue;
        }
        float abs_sweep = prim[i].sweep_rad;
        if (abs_sweep < 0.0f) abs_sweep = -abs_sweep;

        if (prim[i].fit_rel_err < 1e-2f && abs_sweep > 2.094f && prim[i].radius_m > 0.15f)
        {
            return MID_TRACK_ROUNDABOUT;
        }
    }

    // ===== 角度弯：LINE -> ARC -> LINE =====
    // 需求：
    // - “左右拐大概多少度”以两段直线夹角为准（主判据）
    // - 圆弧段扫角为辅助判断（用于抑制误分段/噪声）
    // - 直角弯额外判别：例如 80°~100° 可近似认为直角弯（可按赛道再收紧到 85°~95°）
    if (prim_count >= 3 && prim[0].type == MID_PRIM_LINE && prim[1].type == MID_PRIM_ARC && prim[2].type == MID_PRIM_LINE)
    {
        // 用 atan2(cross, dot) 得到“有符号”的两直线夹角（-pi~pi）
        // 注意：符号与坐标系手性相关；这里仍采用 sweep 的符号来决定左右（更稳定）。
        const float d0x = prim[0].dir_x;
        const float d0y = prim[0].dir_y;
        const float d2x = prim[2].dir_x;
        const float d2y = prim[2].dir_y;

        const float dot = d0x * d2x + d0y * d2y;
        const float cross = d0x * d2y - d0y * d2x;

        // 主判据：两直线夹角（绝对角度，单位：度）
        float line_turn_rad = atan2f(cross, dot);
        float line_turn_deg = line_turn_rad * (180.0f / PI32);
        float abs_line_turn_deg = line_turn_deg;
        if (abs_line_turn_deg < 0.0f) abs_line_turn_deg = -abs_line_turn_deg;

        // 辅助判据：圆弧扫角（绝对角度，单位：度）
        float arc_turn_deg = prim[1].sweep_rad * (180.0f / PI32);
        float abs_arc_turn_deg = arc_turn_deg;
        if (abs_arc_turn_deg < 0.0f) abs_arc_turn_deg = -abs_arc_turn_deg;

        // 直角弯区间（可按需要调整）
        const float kRightAngleMinDeg = 80.0f;
        const float kRightAngleMaxDeg = 100.0f;

        // 圆弧作为“辅助确认”：允许更宽松一些，避免因为圆拟合/分段误差把直角弯拒掉。
        // 这里用 [line-25, line+25] 作为一致性窗口；也可改成固定区间（例如 60~120）。
        float arc_min = abs_line_turn_deg - 25.0f;
        float arc_max = abs_line_turn_deg + 25.0f;
        if (arc_min < 0.0f) arc_min = 0.0f;
        const bool arc_supports = (abs_arc_turn_deg >= arc_min && abs_arc_turn_deg <= arc_max);

        const bool is_right_angle_like = (abs_line_turn_deg >= kRightAngleMinDeg && abs_line_turn_deg <= kRightAngleMaxDeg && arc_supports);

        if (is_right_angle_like)
        {
            // 左/右由 sweep 符号决定（坐标系相关）。这里约定：sweep<0 为“右转”。
            const bool is_right = (prim[1].sweep_rad < 0.0f);
            if (out_turn_deg != nullptr)
            {
                *out_turn_deg = is_right ? -abs_line_turn_deg : abs_line_turn_deg;
            }
            return is_right ? MID_TRACK_RIGHT_ANGLE : MID_TRACK_LEFT_ANGLE;
        }
    }

    // ===== 连续弯（S 弯）：弧段方向交替 =====
    int arc_cnt = 0;
    int sign_changes = 0;
    float last_sign = 0.0f;
    for (int i = 0; i < prim_count; ++i)
    {
        if (prim[i].type != MID_PRIM_ARC)
        {
            continue;
        }
        if (prim[i].len_m < 0.20f)
        {
            continue; // 太短的弧段多半是噪声分段
        }

        ++arc_cnt;
        const float s = (prim[i].sweep_rad >= 0.0f) ? 1.0f : -1.0f;
        if (last_sign != 0.0f && s != last_sign)
        {
            ++sign_changes;
        }
        last_sign = s;
    }
    if (arc_cnt >= 2 && sign_changes >= 1)
    {
        return MID_TRACK_S_CURVE;
    }

    return MID_TRACK_UNKNOWN;
}

// 功能: 赛道状态判别（简化封装）
// 类型: 局部功能函数
// 关键参数: prim/prim_count-输入原语
static __attribute__((unused)) MidTrackState MidLineClassifyTrack(const MidTrackPrimitive* prim, int prim_count)
{
    return MidLineClassifyTrackEx(prim, prim_count, nullptr);
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

// 功能: 从 core 以圆弧并轨到中线并生成路径点列
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
