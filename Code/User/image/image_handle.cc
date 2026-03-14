#include "common.h"
#include "transform_table.h"
#include "headfile.h"
#include "image_math.h"
#include "image_data.h"

// 迷宫法爬线方向表（作用域: 文件内常量）
const int32_t direction_front[4][2] = {{0,  -1},{1,  0},{0,  1},{-1, 0}};      // 正前方向
const int32_t direction_frontleft[4][2] = {{-1, -1},{1,  -1},{1,  1},{-1, 1}}; // 前左方向
const int32_t direction_frontright[4][2] ={{1,  -1},{1,  1},{-1, 1},{-1, -1}}; // 前右方向

// 功能: 迷宫法爬左边线
// 类型: 图像处理函数
// 关键参数: img-二值图, h/w-起点坐标, pts/line_num-输出点列
void SearchLineAdaptive_Left(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t h, int32_t w, int32_t (&pts)[PT_MAXLEN][2], int32_t* line_num)
{
    int step = 0 , dir = 0 , turn = 0 , pts_len = 0; // 局部: 迷宫爬线状态与输出长度
    const int (*df)[2] = direction_front;
    const int (*dfl)[2] = direction_frontleft;
    while (step < PT_MAXLEN && turn < 4)
    {
        if (!(w > 1 && w < IMAGE_W - 2 && h > 1 && h < IMAGE_H - 2))
        {
            break;
        }

        int fh = h + df[dir][1], fw = w + df[dir][0];
        int flh = h + dfl[dir][1], flw = w + dfl[dir][0];
        int front = img[fh][fw];
        int frontleft = img[flh][flw];

        if (front == BLACK_IN_GRAY)
        {
            dir = (dir + 1) & 3;
            turn++;
            continue;
        }

        if (frontleft == BLACK_IN_GRAY)
        {
            w += df[dir][0];
            h += df[dir][1];
        }
        else
        {
            w += dfl[dir][0];
            h += dfl[dir][1];
            dir = (dir + 3) & 3;
        }

        step++;
        turn = 0;

        if (w > 5 && w < IMAGE_W - 5)
        {
            if (pts_len >= PT_MAXLEN)
            {
                break;
            }

            pts[pts_len][1] = w;
            pts[pts_len][0] = h;
            pts_len++;
        }
    }
    *line_num = pts_len;
}

// 功能: 迷宫法爬右边线
// 类型: 图像处理函数
// 关键参数: img-二值图, h/w-起点坐标, pts/line_num-输出点列
void SearchLineAdaptive_Right(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t h, int32_t w, int32_t (&pts)[PT_MAXLEN][2], int32_t* line_num)
{
    int step = 0 , dir = 0 , turn = 0 , pts_len = 0; // 局部: 迷宫爬线状态与输出长度
    const int (*df)[2] = direction_front;
    const int (*dfr)[2] = direction_frontright;
    while (step < PT_MAXLEN && turn < 4)
    {
        if (!(w > 1 && w < IMAGE_W - 2 && h > 1 && h < IMAGE_H - 2))
        {
            break;
        }

        int fh = h + df[dir][1], fw = w + df[dir][0];
        int frh = h + dfr[dir][1], frw = w + dfr[dir][0];
        int front = img[fh][fw];
        int frontright = img[frh][frw];

        if (front == BLACK_IN_GRAY)
        {
            dir = (dir + 3) & 3;
            turn++;
            continue;
        }

        if (frontright == BLACK_IN_GRAY)
        {
            w += df[dir][0];
            h += df[dir][1];
        }
        else
        {
            w += dfr[dir][0];
            h += dfr[dir][1];
            dir = (dir + 1) & 3;
        }

        step++;
        turn = 0;

        if (w > 5 && w < IMAGE_W - 5)
        {
            if (pts_len >= PT_MAXLEN)
            {
                break;
            }

            pts[pts_len][1] = w;
            pts[pts_len][0] = h;
            pts_len++;
        }
    }
    *line_num = pts_len;
}


// 功能: 左边线搜索（扩展版，可选只找起点）
// 类型: 图像处理函数
// 关键参数: climb_line-是否爬线, out_find_start_pt/out_start_pt-起点输出
void SearchLine_LptEx(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t start_x, int32_t start_y,
                     int32_t (&pts)[PT_MAXLEN][2], int32_t* pts_count,
                     bool climb_line, bool* out_find_start_pt, int32_t (&out_start_pt)[2])
{
    if (pts_count == nullptr)
    {
        return;
    }

    bool found = false;
    if (out_find_start_pt)
    {
        *out_find_start_pt = false;
    }
    out_start_pt[0] = 0;
    out_start_pt[1] = 0;

    int32_t x_cur = start_x + SEARCH_LINE_START_OFFSET , y_cur = start_y;
    if (x_cur < 1) x_cur = 1;
    if (x_cur > IMAGE_W - 2) x_cur = IMAGE_W - 2;
    if (y_cur < 0) y_cur = 0;
    if (y_cur > IMAGE_H - 1) y_cur = IMAGE_H - 1;
    const uint8_t* row = img[y_cur];

    if(element_type != ElementType::CROSSING)
    {
        while ((!found) && y_cur > IMAGE_H * 2 / 3)
        {
            for (int i = x_cur - 1; i > 1; --i)
            {
                if (row[i] == WHITE_IN_GRAY && row[i-1] == BLACK_IN_GRAY)
                {
                    x_cur = i;
                    found = true;
                    break;
                }
            }

            if (found)
            {
                out_start_pt[0] = y_cur;
                out_start_pt[1] = x_cur;
                if (out_find_start_pt)
                {
                    *out_find_start_pt = true;
                }
            }
            else
            {
                y_cur--;
                row = img[y_cur];
            }
        }

    }
    else if(element_type == ElementType::CROSSING)
    {
        for (int i = x_cur - 1; i > 1; --i)
        {
            if (row[i] == WHITE_IN_GRAY && row[i-1] == BLACK_IN_GRAY)
            {
                x_cur = i;
                found = true;
                break;
            }
        }

        if (found)
        {
            out_start_pt[0] = y_cur;
            out_start_pt[1] = x_cur;
            if (out_find_start_pt)
            {
                *out_find_start_pt = true;
            }
        }
    }

    if (climb_line && found)
    {
        SearchLineAdaptive_Left(img, y_cur, x_cur, pts, pts_count);
    }
    else
    {
        *pts_count = 0;
    }
}

// 功能: 左边线搜索（兼容旧接口）
// 类型: 图像处理函数
// 关键参数: img-二值图, start_x/start_y-起点搜索中心
void SearchLine_Lpt(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t start_x, int32_t start_y,
                   int32_t (&pts)[PT_MAXLEN][2], int32_t* pts_count)
{
    bool dummy_found = false;
    int32_t dummy_pt[2] = {0, 0};
    SearchLine_LptEx(img, start_x, start_y, pts, pts_count, true, &dummy_found, dummy_pt);
}


// 功能: 右边线搜索（扩展版，可选只找起点）
// 类型: 图像处理函数
// 关键参数: climb_line-是否爬线, out_find_start_pt/out_start_pt-起点输出
void SearchLine_RptEx(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t start_x, int32_t start_y,
                     int32_t (&pts)[PT_MAXLEN][2], int32_t* pts_count,
                     bool climb_line, bool* out_find_start_pt, int32_t (&out_start_pt)[2])
{
    if (pts_count == nullptr)
    {
        return;
    }

    bool found = false;
    if (out_find_start_pt)
    {
        *out_find_start_pt = false;
    }
    out_start_pt[0] = 0;
    out_start_pt[1] = 0;

    int32_t x_cur = start_x - SEARCH_LINE_START_OFFSET , y_cur =start_y;
    if (x_cur < 1) x_cur = 1;
    if (x_cur > IMAGE_W - 2) x_cur = IMAGE_W - 2;
    if (y_cur < 0) y_cur = 0;
    if (y_cur > IMAGE_H - 1) y_cur = IMAGE_H - 1;
    const uint8_t* row = img[y_cur];

    // 十字寻远线需要在任意 y 上探测起点：不做“只在底部 1/3 扫描”的限制。
    if (element_type == ElementType::CROSSING)
    {
        for (int i = x_cur; i < IMAGE_W - 2; ++i)
        {
            if (row[i] == WHITE_IN_GRAY && row[i + 1] == BLACK_IN_GRAY)
            {
                x_cur = i;
                found = true;
                break;
            }
        }

        if (found)
        {
            out_start_pt[0] = y_cur;
            out_start_pt[1] = x_cur;
            if (out_find_start_pt)
            {
                *out_find_start_pt = true;
            }
        }
    }
    else
    {
        while ((!found) && y_cur > IMAGE_H * 2 / 3)
        {
            for (int i = x_cur; i < IMAGE_W - 2; ++i)
            {
                if (row[i] == WHITE_IN_GRAY && row[i + 1] == BLACK_IN_GRAY)
                {
                    x_cur = i;
                    found = true;
                    break;
                }
            }

            if (found)
            {
                out_start_pt[0] = y_cur;
                out_start_pt[1] = x_cur;
                if (out_find_start_pt)
                {
                    *out_find_start_pt = true;
                }
            }
            else
            {
                y_cur--;
                row = img[y_cur];
            }
        }
    }

    if (climb_line && found)
    {
        SearchLineAdaptive_Right(img, y_cur, x_cur, pts, pts_count);
    }
    else
    {
        *pts_count = 0;
    }
}

// 功能: 右边线搜索（兼容旧接口）
// 类型: 图像处理函数
// 关键参数: img-二值图, start_x/start_y-起点搜索中心
void SearchLine_Rpt(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t start_x, int32_t start_y,
                   int32_t (&pts)[PT_MAXLEN][2], int32_t* pts_count)
{
    bool dummy_found = false;
    int32_t dummy_pt[2] = {0, 0};
    SearchLine_RptEx(img, start_x, start_y, pts, pts_count, true, &dummy_found, dummy_pt);
}


// 功能: 逆透视变换（查表映射到俯视坐标）
// 类型: 图像处理函数
// 关键参数: pts_in/in_count-输入点列, pts_out/out_count-输出点列
void InversePerspectiveTransform(int32_t (&pts_in)[PT_MAXLEN][2], int32_t* in_count, float (&pts_out)[PT_MAXLEN][2], int32_t* out_count)
{
    if (in_count == nullptr || out_count == nullptr)
    {
        return;
    }

    int n = *in_count;
    if (n <= 0)
    {
        *out_count = 0;
        return;
    }

    // 并行化（编译开启 OpenMP（GCC/Clang -fopenmp，MSVC /openmp））
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif

    for (int i = 0; i < n; ++i)
    {
        int y = pts_in[i][0];
        int x = pts_in[i][1];
        if (y < 0) y = 0;
        if (y > IMAGE_H - 1) y = IMAGE_H - 1;
        if (x < 0) x = 0;
        if (x > IMAGE_W - 1) x = IMAGE_W - 1;

        const float* rowW = UndistInverseMapW[y];
        const float* rowH = UndistInverseMapH[y];

        float wx = rowW[x];
        float hy = rowH[x];

        // 表里无效点通常以 -1 标记；此外也要防御极端越界值。
        // 这里不要把无效点强行 fclip 到边界（会造成大量点聚到角上，视觉上“乱飞”）。
        const bool valid = (wx >= 0.0f && wx <= (float)(IMAGE_W - 1)) &&
                           (hy >= 0.0f && hy <= (float)(IMAGE_H - 1));
        if (!valid)
        {
            pts_out[i][1] = -1.0f;
            pts_out[i][0] = -1.0f;
            continue;
        }

        pts_out[i][1] = fclip(wx, 0.0f, (float)(IMAGE_W - 1));
        pts_out[i][0] = fclip(hy, 0.0f, (float)(IMAGE_H - 1));
    }

    *out_count = n;
}


// 功能: 边线滤波（两次 box 平均形成三角滤波）
// 类型: 图像处理函数
// 关键参数: kernelsize-滤波核尺寸（会修正为奇数）
void GetLinesFilter(float (&pts_in)[PT_MAXLEN][2], int32_t* pts_in_count, float (&pts_out)[PT_MAXLEN][2], int32_t* pts_out_count, int32_t kernelsize)
{
    if (pts_in_count == nullptr || pts_out_count == nullptr)
    {
        return;
    }
    if (*pts_in_count <= 0)
    {
        *pts_out_count = 0;
        return;
    }

    // kernelsize 取奇数（若为偶数则向下取最近奇数）；B 为 box 长度，radius = (B-1)/2 用于两次 box
    if ((kernelsize & 1) == 0)
    {
        --kernelsize;
    }
    const int B = (kernelsize + 1) / 2;
    const int radius = (B - 1) / 2;
    int n = *pts_in_count;
    if (n > PT_MAXLEN) n = PT_MAXLEN;

    // 复用缓冲以避免频繁分配；使用 thread_local 便于未来多线程/并行调用。
    // 作用域: 线程局部静态缓存，用于避免频繁分配
    static thread_local float vx_buf[PT_MAXLEN];
    static thread_local float vy_buf[PT_MAXLEN];
    static thread_local float bx_buf[PT_MAXLEN];
    static thread_local float by_buf[PT_MAXLEN];

    for (int i = 0; i < n; ++i)
    {
        // 工程内点坐标约定：pts[*][0]=y, pts[*][1]=x
        vx_buf[i] = pts_in[i][1];
        vy_buf[i] = pts_in[i][0];
    }

    // 单次 box average（边界复制/replicate）：窗口为 [i-radius, i+radius]，超界用端点值填充。
    // 这样两次 box average 等价三角滤波，但不会在边界把值“压向 0”（原实现的固定 B^2 归一化会导致乱飞）。
    auto box_avg_replicate = [&](const float* in, float* out)
    {
        // 前缀和（double 防止累计误差）；只用到 [0,n)
        static thread_local double prefix[PT_MAXLEN + 1];
        prefix[0] = 0.0;
        for (int i = 0; i < n; ++i)
        {
            prefix[i + 1] = prefix[i] + (double)in[i];
        }

        for (int i = 0; i < n; ++i)
        {
            int l = i - radius;
            int r = i + radius;

            int cl = l;
            int cr = r;
            if (cl < 0) cl = 0;
            if (cr > n - 1) cr = n - 1;

            double sum = prefix[cr + 1] - prefix[cl];

            // 边界复制补偿
            if (l < 0) sum += (double)in[0] * (double)(-l);
            if (r > n - 1) sum += (double)in[n - 1] * (double)(r - (n - 1));

            const int win_len = 2 * radius + 1;
            out[i] = (float)(sum / (double)win_len);
        }
    };

    // 两次 box average -> 三角滤波
    box_avg_replicate(vx_buf, bx_buf);
    box_avg_replicate(vy_buf, by_buf);
    box_avg_replicate(bx_buf, vx_buf);
    box_avg_replicate(by_buf, vy_buf);

    for (int i = 0; i < n; ++i)
    {
        pts_out[i][1] = vx_buf[i];
        pts_out[i][0] = vy_buf[i];
    }

    *pts_out_count = n;
}


// 功能: 有效点筛选（越界点剔除）
// 类型: 图像处理函数
// 关键参数: pts/count-输入输出点列（原地筛选）
void FilterValidPoints(float (&pts)[PT_MAXLEN][2], int32_t* count)
{
    if (count == nullptr)
    {
        return;
    }
    int32_t n = *count;
    if (n <= 0)
    {
        *count = 0;
        return;
    }

    if (n > PT_MAXLEN) n = PT_MAXLEN;

    int32_t write = 0;
    for (int32_t i = 0; i < n; ++i)
    {
        const int x = (int)pts[i][1];
        const int y = (int)pts[i][0];

        bool inside = (unsigned)(x - 1) < (unsigned)(IMAGE_W - 1) &&
                      (unsigned)(y - 1) < (unsigned)(IMAGE_H - 1);

        if (inside)
        {
            pts[write][0] = pts[i][0];
            pts[write][1] = pts[i][1];
            ++write;
        }
    }
    *count = write;
}


// 功能: 点列重采样（严格等距）
// 类型: 图像处理函数
// 关键参数: dist-采样间距(像素), out_src_index-可选输出源索引映射
void GetLinesResample(float (&pts_in)[PT_MAXLEN][2], int32_t* num1, float (&pts_out)[PT_MAXLEN][2], int32_t* num2, float dist, int32_t* out_src_index)
{
    if (num1 == nullptr || num2 == nullptr)
    {
        return;
    }
    if (*num1 <= 0 || dist <= 0.0f)
    {
        *num2 = 0;
        return;
    }

    int n_in = *num1;
    if (n_in > PT_MAXLEN) n_in = PT_MAXLEN;

    // 先过滤输入点，保证后续插值点都在图像范围内，从而不需要对 pts_out 再过滤（避免破坏严格等距）
    float in_pts[PT_MAXLEN][2];
    int32_t in_src[PT_MAXLEN];
    int n = 0;
    for (int i = 0; i < n_in && n < PT_MAXLEN; ++i)
    {
        const int x = (int)pts_in[i][1];
        const int y = (int)pts_in[i][0];
        bool inside = (unsigned)(x - 1) < (unsigned)(IMAGE_W - 1) &&
                      (unsigned)(y - 1) < (unsigned)(IMAGE_H - 1);
        if (inside)
        {
            in_pts[n][0] = pts_in[i][0];
            in_pts[n][1] = pts_in[i][1];
            in_src[n] = i;
            ++n;
        }
    }

    if (n <= 0)
    {
        *num2 = 0;
        return;
    }

    int out_len = 0;
    pts_out[out_len][0] = in_pts[0][0];
    pts_out[out_len][1] = in_pts[0][1];
    if (out_src_index)
    {
        out_src_index[out_len] = in_src[0];
    }
    ++out_len;

    float cur_x = in_pts[0][1];
    float cur_y = in_pts[0][0];
    float need = dist; // 距离下一个采样点还需要走的距离

    for (int i = 0; i < n - 1 && out_len < PT_MAXLEN; ++i)
    {
        const float end_x = in_pts[i + 1][1];
        const float end_y = in_pts[i + 1][0];

        const float start_x = in_pts[i][1];
        const float start_y = in_pts[i][0];

        const float dx = end_x - cur_x;
        const float dy = end_y - cur_y;
        const float seg2 = dx * dx + dy * dy;
        if (seg2 <= 1e-12f)
        {
            cur_x = end_x;
            cur_y = end_y;
            continue;
        }

        // inv_len ≈ 1/sqrt(seg2)，用 fast_rsqrt + 额外一轮牛顿迭代（仍无 sqrt）
        float inv_len = fast_rsqrt(seg2);
        inv_len = inv_len * (1.5f - 0.5f * seg2 * inv_len * inv_len);

        const float ux = dx * inv_len;
        const float uy = dy * inv_len;
        float seg_len = seg2 * inv_len; // ≈ sqrt(seg2)

        // 允许极小误差避免卡死
        const float eps = 1e-6f;

        while (seg_len + eps >= need && out_len < PT_MAXLEN)
        {
            cur_x += ux * need;
            cur_y += uy * need;

            pts_out[out_len][0] = cur_y;
            pts_out[out_len][1] = cur_x;

            if (out_src_index)
            {
                const float dsx = cur_x - start_x;
                const float dsy = cur_y - start_y;
                const float dex = cur_x - end_x;
                const float dey = cur_y - end_y;
                const float ds2 = dsx * dsx + dsy * dsy;
                const float de2 = dex * dex + dey * dey;
                out_src_index[out_len] = (ds2 <= de2) ? in_src[i] : in_src[i + 1];
            }
            ++out_len;

            seg_len -= need;
            need = dist;
        }

        // 走到段末，扣除剩余长度
        cur_x = end_x;
        cur_y = end_y;
        if (seg_len > 0.0f)
        {
            need -= seg_len;
            if (need < eps)
            {
                need = dist;
            }
        }
    }

    // 注意：不强行追加末尾点，否则最后一段可能小于 dist，破坏“相邻点严格为 dist”
    *num2 = out_len;
}


// 功能: 计算局部弯曲强度（1-cos）
// 类型: 图像处理函数
// 关键参数: dist-两端点跨度(点数), curv_out-输出曲率强度
// 定义：curv = 1 - cos(theta)，其中 theta 为两段向量夹角。
// - 优点：
//   - 单调覆盖 [0, pi]，不会像 sin^2 那样对 60°/120°给出同值。
//   - 不需要 atan2f；只需 fast_rsqrt 做向量归一化。
// - 输出范围：理论上 [0, 2]（theta=0 -> 0；theta=pi -> 2）。
void local_curvature_points(const float (&pts_in)[PT_MAXLEN][2], int* pts_in_count, float (&curv_out)[PT_MAXLEN], int* curv_out_count, int dist)
{
    if (curv_out_count == nullptr)
    {
        return;
    }

    if (pts_in_count == nullptr || *pts_in_count <= 0)
    {
        *curv_out_count = 0;
        return;
    }

    int n = *pts_in_count;
    if (n > PT_MAXLEN)
    {
        n = PT_MAXLEN;
    }

    // dist<=0 时无法形成有效局部向量，直接输出 0
    if (dist <= 0)
    {
        for (int i = 0; i < n; ++i)
        {
            curv_out[i] = 0.0f;
        }
        *curv_out_count = n;
        return;
    }

    for (int i = 0; i < n; ++i)
    {
        if (i == 0 || i >= n - 1)
        {
            curv_out[i] = 0.0f;
            continue;
        }

        int i1 = i - dist;
        if (i1 < 0)
        {
            i1 = 0;
        }

        int i2 = i + dist;
        if (i2 > n - 1)
        {
            i2 = n - 1;
        }

        const float dx1 = pts_in[i][1] - pts_in[i1][1];
        const float dy1 = pts_in[i][0] - pts_in[i1][0];
        const float dx2 = pts_in[i2][1] - pts_in[i][1];
        const float dy2 = pts_in[i2][0] - pts_in[i][0];

        // 任一向量近似为 0 时，角度不可靠，按 0 处理（避免 atan2(0,0)）
        if ((dx1 * dx1 + dy1 * dy1) <= 1e-12f || (dx2 * dx2 + dy2 * dy2) <= 1e-12f)
        {
            curv_out[i] = 0.0f;
            continue;
        }

        const float len1_2 = dx1 * dx1 + dy1 * dy1;
        const float len2_2 = dx2 * dx2 + dy2 * dy2;

        // inv_len ≈ 1/sqrt(x)，fast_rsqrt + 一轮牛顿迭代（无 sqrt）
        float inv1 = fast_rsqrt(len1_2);
        inv1 = inv1 * (1.5f - 0.5f * len1_2 * inv1 * inv1);
        float inv2 = fast_rsqrt(len2_2);
        inv2 = inv2 * (1.5f - 0.5f * len2_2 * inv2 * inv2);

        const float dot = dx1 * dx2 + dy1 * dy2;
        float cosv = dot * inv1 * inv2;
        // 数值夹紧，避免 fast_rsqrt 误差导致 cos 超出 [-1,1]
        if (cosv > 1.0f) cosv = 1.0f;
        else if (cosv < -1.0f) cosv = -1.0f;

        curv_out[i] = 1.0f - cosv;
    }

    *curv_out_count = n;
}


// 功能: 曲率极大值抑制（1D）
// 类型: 图像处理函数
// 关键参数: kernel-窗口大小
// 输入为 curvature(>=0)。实现按窗口内“严格小于最大值”抑制，等值平台会保留。
void nms_curvature(float (&curv_in)[PT_MAXLEN], int* curv_in_count, float (&curv_out)[PT_MAXLEN], int* curv_out_count, int kernel)
{
    if (curv_out_count == nullptr)
    {
        return;
    }

    if (curv_in_count == nullptr || *curv_in_count <= 0)
    {
        *curv_out_count = 0;
        return;
    }

    if (kernel <= 1)
    {
        for (int i = 0; i < *curv_in_count; ++i)
        {
            curv_out[i] = curv_in[i];
        }
        *curv_out_count = *curv_in_count;
        return;
    }

    const int half = kernel >> 1;
    int n = *curv_in_count;
    if (n > PT_MAXLEN)
    {
        n = PT_MAXLEN;
    }

    for (int i = 0; i < n; ++i)
    {
        const float cur = curv_in[i];
        float out = cur;

        int start = i - half;
        if (start < 0)
        {
            start = 0;
        }
        int end = i + half;
        if (end > n - 1)
        {
            end = n - 1;
        }

        // 只要窗口内存在更大的值，就抑制
        for (int j = start; j <= end; ++j)
        {
            const float aj = curv_in[j];
            if (aj > cur)
            {
                out = 0.0f;
                break;
            }
        }

        curv_out[i] = out;
    }
    *curv_out_count = n;
}


// 功能: 根据左边线生成中线（向右偏移半车道）
// 类型: 图像处理函数
// 关键参数: dist-偏移距离(像素), approx_num-导数近似跨度
void GetMidLine_Left(float (&pts_left)[PT_MAXLEN][2], int32_t* pts_left_count, float (&mid_left)[PT_MAXLEN][2], int32_t* mid_left_count, int32_t approx_num, float dist)
{
    if (mid_left_count == nullptr)
    {
        return;
    }

    if (pts_left_count == nullptr || *pts_left_count <= 0)
    {
        *mid_left_count = 0;
        return;
    }

    int n = *pts_left_count;
    if (n > PT_MAXLEN)
    {
        n = PT_MAXLEN;
    }
    const int a = (approx_num > 0) ? approx_num : 1;

    for (int i = 0; i < n; ++i)
    {
        int im = i - a;
        if (im < 0)
        {
            im = 0;
        }
        int ip = i + a;
        if (ip > n - 1)
        {
            ip = n - 1;
        }

        const float dx = pts_left[ip][1] - pts_left[im][1];
        const float dy = pts_left[ip][0] - pts_left[im][0];
        const float len2 = dx * dx + dy * dy;

        if (len2 <= 1e-12f)
        {
            if (i > 0)
            {
                mid_left[i][1] = mid_left[i - 1][1];
                mid_left[i][0] = mid_left[i - 1][0];
            }
            else
            {
                mid_left[i][1] = pts_left[i][1];
                mid_left[i][0] = pts_left[i][0];
            }
            continue;
        }

        // inv_len ≈ 1/sqrt(len2)，用 fast_rsqrt + 一轮牛顿迭代避免 sqrt/除法
        float inv_len = fast_rsqrt(len2);
        inv_len = inv_len * (1.5f - 0.5f * len2 * inv_len * inv_len);

        const float cosv = dx * inv_len;
        const float sinv = dy * inv_len;

        mid_left[i][1] = pts_left[i][1] - sinv * dist;
        mid_left[i][0] = pts_left[i][0] + cosv * dist;
    }

    *mid_left_count = n;
}


// 功能: 根据右边线生成中线（向左偏移半车道）
// 类型: 图像处理函数
// 关键参数: dist-偏移距离(像素), approx_num-导数近似跨度
void GetMidLine_Right(float (&pts_right)[PT_MAXLEN][2], int32_t* pts_right_count, float (&mid_right)[PT_MAXLEN][2], int32_t* mid_right_count ,int32_t approx_num, float dist)
{
    if (mid_right_count == nullptr)
    {
        return;
    }

    if (pts_right_count == nullptr || *pts_right_count <= 0)
    {
        *mid_right_count = 0;
        return;
    }

    int n = *pts_right_count;
    if (n > PT_MAXLEN)
    {
        n = PT_MAXLEN;
    }
    const int a = (approx_num > 0) ? approx_num : 1;

    for (int i = 0; i < n; ++i)
    {
        int im = i - a;
        if (im < 0)
        {
            im = 0;
        }
        int ip = i + a;
        if (ip > n - 1)
        {
            ip = n - 1;
        }

        const float dx = pts_right[ip][1] - pts_right[im][1];
        const float dy = pts_right[ip][0] - pts_right[im][0];
        const float len2 = dx * dx + dy * dy;

        if (len2 <= 1e-12f)
        {
            if (i > 0)
            {
                mid_right[i][1] = mid_right[i - 1][1];
                mid_right[i][0] = mid_right[i - 1][0];
            }
            else
            {
                mid_right[i][1] = pts_right[i][1];
                mid_right[i][0] = pts_right[i][0];
            }
            continue;
        }

        float inv_len = fast_rsqrt(len2);
        inv_len = inv_len * (1.5f - 0.5f * len2 * inv_len * inv_len);

        const float cosv = dx * inv_len;
        const float sinv = dy * inv_len;

        mid_right[i][1] = pts_right[i][1] + sinv * dist;
        mid_right[i][0] = pts_right[i][0] - cosv * dist;
    }

    *mid_right_count = n;
}




// 功能: 曲线判定（基于曲率峰值）
// 类型: 图像处理函数
// 关键参数: curvature-曲率数组, threshold-阈值(1-cos)
// 曲线判断（单侧边线/中线通用）：
// - 原实现用“相隔 3 点的角度差”做判定，容易受噪声/局部抖动影响。
// - 这里改为：对 |angle| 做一个小窗口的“局部极大值”检测（等价轻量 NMS），
//   只要存在显著角度峰值，就认为该点列包含明显弯曲。
// - 这里的 curvature 是弯曲强度：curv = 1 - cos(theta)，范围约 [0, 2]。
// - threshold 单位：同 curvature（不是弧度）。
int is_curve(float (&curvature)[PT_MAXLEN], int* count, float threshold)
{
    if (count == nullptr || *count < 7)
    {
        return 0;
    }

    int n = *count;
    if (n > PT_MAXLEN)
    {
        n = PT_MAXLEN;
    }
    const int half = 3; // 7 点窗口

    for (int i = half; i < n - half; ++i)
    {
        float ai = curvature[i];
        if (ai <= threshold)
        {
            continue;
        }

        bool is_peak = true;
        for (int j = i - half; j <= i + half; ++j)
        {
            if (j == i)
            {
                continue;
            }
            float aj = curvature[j];
            if (aj > ai)
            {
                is_peak = false;
                break;
            }
        }

        if (is_peak)
        {
            return 1;
        }
    }

    return 0;
}


// 功能: 单侧边线角点检测
// 类型: 图像处理函数
// 关键参数: pts_resample-重采样点, curvature_nms-极大值抑制曲率, Lpt_found/Lpt_rpts_id-输出
void get_corner(
    float (&pts_resample)[PT_MAXLEN][2], int* pts_resample_count,
    float (&curvature)[PT_MAXLEN], float (&curvature_nms)[PT_MAXLEN],
    bool* Lpt_found, int* Lpt_rpts_id, float (&Lpt_rpts)[2],
    bool* is_straight
) 
{
    if (pts_resample_count == nullptr || Lpt_found == nullptr || Lpt_rpts_id == nullptr || is_straight == nullptr)
    {
        return;
    }

    // 初始化输出参数
    *Lpt_found = false;
    *Lpt_rpts_id = 0;
    Lpt_rpts[0] = 0.0f;
    Lpt_rpts[1] = 0.0f;

    int n = *pts_resample_count;
    if (n > PT_MAXLEN)
    {
        n = PT_MAXLEN;
    }
    if (n <= 0)
    {
        *is_straight = false;
        return;
    }

    // 初始化直道判断：点数足够时先假设直道，后续遇到显著角度峰值再否决。
    *is_straight = (n >= 10);

    // 角度对比的“远邻”间隔（以重采样间距换算为索引）
    // 这些都由宏常量决定，做成静态常量避免每帧重复计算。
    static const int k = []() -> int {
        int v = (int)lround(ANGLEDIST / RESAMPLEDIST);
        return (v < 1) ? 1 : v;
    }();

    static const int straight_check_end_max = (int)(1.2f / RESAMPLEDIST);

    // 角度阈值仍沿用 common.h 的“度”配置，但在弯曲强度域转换为 1-cos(theta)
    static const float conf_min = 1.0f - cosf((ANGLE_THRESHOLD_get_corners_conf_min / 180.0f) * PI32);
    static const float conf_max = 1.0f - cosf((ANGLE_THRESHOLD_get_corners_conf_max / 180.0f) * PI32);
    static const float straight_thr = 1.0f - cosf((ANGLE_THRESHOLD_is_straight / 180.0f) * PI32);

    // 仅在近端做直道否决（原逻辑保持：约 1.2m 范围内出现显著角度变化就判非直道）
    int straight_check_end = straight_check_end_max;
    if (straight_check_end > n - 1) straight_check_end = n - 1;

    for (int i = 0; i < n; ++i)
    {
        // 跳过非极大值抑制后无效的角度点
        if (curvature_nms[i] == 0.0f) continue;

        // 计算置信度（当前点角度与前后点平均角度的差值）
        int im1 = i - k;
        if (im1 < 0) im1 = 0;
        int ip1 = i + k;
        if (ip1 > n - 1) ip1 = n - 1;

        const float ai = curvature[i];
        const float am = curvature[im1];
        const float ap = curvature[ip1];
        const float conf = ai - 0.5f * (am + ap);

        // 检测L型角点
        if (!(*Lpt_found) && conf > conf_min && conf < conf_max)
        {
            *Lpt_rpts_id = i;
            Lpt_rpts[1] = pts_resample[i][1];  // x坐标
            Lpt_rpts[0] = pts_resample[i][0];  // y坐标
            *Lpt_found = true;
        }

        // 直道判断（存在显著角度变化则判定为非直道）
        if (*is_straight && i <= straight_check_end && conf > straight_thr)
        {
            *is_straight = false;
        }

        // 已找到角点且确认非直道，提前退出循环
        if (*Lpt_found && !(*is_straight)) break;
    }
}


// 功能: 通用边线处理流程
// 类型: 图像处理函数
// 关键参数: is_left-左/右线, pts_in/in_count-输入边线点, mid_line/mid_count-输出中线
void process_line(
    bool is_left,
    int32_t (&pts_in)[PT_MAXLEN][2], int32_t* in_count,
    float (&pts_inv)[PT_MAXLEN][2], int32_t* inv_count,
    float (&pts_filter)[PT_MAXLEN][2], int32_t* filter_count,
    float (&pts_resample)[PT_MAXLEN][2], int32_t* resample_count,
    float (&curvature)[PT_MAXLEN], int* curvature_num,
    float (&curvature_nms)[PT_MAXLEN], int* curvature_nms_num,
    float (&mid_line)[PT_MAXLEN][2], int32_t* mid_count,
    bool* is_straight, 
    bool* corner_found, int* corner_id, float (&corner_pts)[2],
    int* is_curve_int,
    int32_t (&resample_src_id)[PT_MAXLEN]
) 
{
    if (in_count == nullptr || *in_count <= 0)
    {
        if (inv_count) *inv_count = 0;
        if (filter_count) *filter_count = 0;
        if (resample_count) *resample_count = 0;
        if (curvature_num) *curvature_num = 0;
        if (curvature_nms_num) *curvature_nms_num = 0;
        if (mid_count) *mid_count = 0;
        if (corner_found) *corner_found = false;
        if (corner_id) *corner_id = 0;
        if (is_straight) *is_straight = false;
        if (is_curve_int) *is_curve_int = 0;
        return;
    }

    if (inv_count == nullptr || filter_count == nullptr || resample_count == nullptr || curvature_num == nullptr || curvature_nms_num == nullptr || mid_count == nullptr ||
        is_straight == nullptr || corner_found == nullptr || corner_id == nullptr || is_curve_int == nullptr)
    {
        return;
    }

    // 先统一清零输出，避免上层误用“上一次帧”的残留结果
    *inv_count = 0;
    *filter_count = 0;
    *resample_count = 0;
    *curvature_num = 0;
    *curvature_nms_num = 0;
    *mid_count = 0;
    *corner_found = false;
    *corner_id = 0;
    *is_straight = false;
    *is_curve_int = 0;
    corner_pts[0] = 0.0f;
    corner_pts[1] = 0.0f;

    // 由宏常量决定的参数，静态预计算（每帧会被调用两次：左右边线）。
    static const int span = []() -> int {
        int v = (int)lround(ANGLEDIST / RESAMPLEDIST);
        return (v < 1) ? 1 : v;
    }();
    static const int kernel = span * 2 + 1;
    static const float resample_dist_pix = RESAMPLEDIST * PIXPERMETER;
    static const float half_width_pix = PIXPERMETER * ROADWIDTH * 0.5f;

    // 防御：异常配置直接退出（正常情况下这些宏常量应为正）
    if (!(resample_dist_pix > 0.0f) || !(half_width_pix > 0.0f))
    {
        return;
    }

    InversePerspectiveTransform(pts_in, in_count, pts_inv, inv_count);

    // 逆透视查表后先过滤无效点，避免后续滤波/重采样被 -1/越界值污染。
    FilterValidPoints(pts_inv, inv_count);

    if (*inv_count <= 1)
    {
        return;
    }

    GetLinesFilter(pts_inv, inv_count, pts_filter, filter_count, (int)round(FILTER_KERNELSIZE));
    FilterValidPoints(pts_filter, filter_count);
    if (*filter_count <= 1)
    {
        return;
    }

    GetLinesResample(pts_filter, filter_count, pts_resample, resample_count, resample_dist_pix, resample_src_id);
    if (*resample_count <= 2)
    {
        return;
    }

    local_curvature_points(pts_resample, resample_count, curvature, curvature_num, span);
    nms_curvature(curvature, curvature_num, curvature_nms, curvature_nms_num, kernel);

    if (is_left)
    {
        GetMidLine_Left(pts_resample, resample_count, mid_line, mid_count, span, half_width_pix);
    } 
    else 
    {
        GetMidLine_Right(pts_resample, resample_count, mid_line, mid_count, span, half_width_pix);
    }

    GetLinesResample(mid_line, mid_count, mid_line, mid_count, resample_dist_pix, nullptr);
    
    get_corner(
        pts_resample, resample_count,
        curvature, curvature_nms,
        corner_found, corner_id, corner_pts,
        is_straight
    );

    // 直/曲判断：为增强抗噪，不再用“近端出现 1 个峰值就判曲线”。
    // 改为：近端至少有 5 个点（连续）满足阈值才判曲线，避免孤立噪声点触发。
    // CURVE_THRESHOLD 在 common.h 里是“角度（弧度）”，这里换算到 1-cos(theta)
    static const float curve_thr = 1.0f - cosf(CURVE_THRESHOLD);
    const int check_n = (*curvature_num < 15) ? *curvature_num : 15;

    int curve = 0;
    if (check_n >= 5)
    {
        int consecutive = 0;
        for (int i = 0; i < check_n; ++i)
        {
            const float a = curvature[i];
            if (a > curve_thr)
            {
                consecutive++;
                if (consecutive >= 5)
                {
                    curve = 1;
                    break;
                }
            }
            else
            {
                consecutive = 0;
            }
        }
    }
    *is_curve_int = curve;
    if (curve)
    {
        *is_straight = false;
    }
}


// 功能: 由路径点列计算 pure_angle
// 类型: 图像处理函数
// 关键参数: path/path_count-路径点列, out_pure_angle-输出角度
void CalculatePureAngleFromPath(const float (&path)[PT_MAXLEN][2], int32_t path_count, float* out_pure_angle)
{
    // 判断有效
    if (out_pure_angle == nullptr)
    {
        return;
    }
    if (path_count <= 0)
    {
        *out_pure_angle = 0.0f;
        return;
    }
    
    // 基于车轴位置到路径近点（不修改全局/宏配置）
    int idx = PATH_NEAR_ID_FOR_PURE_ANGLE;
    if (idx < 0) idx = 0;
    if (idx > (path_count - 1)) idx = (path_count - 1);

    const float car_x = UndistInverseMapW[SET_IMAGE_CORE_Y][SET_IMAGE_CORE_X];

    const float dx = path[idx][1] - car_x;
    const float dy = path[0][0] - path[idx][0];
    const float forward = dy + DISTANCE_FROM_VIEW_TO_CAR * PIXPERMETER;0

    // 用 atan2f 替代 dx/forward，避免 forward 很小时数值爆炸
    float angle = -atan2f(dx, forward) * 180.0f / PI32;
    angle = fclip(angle, -80.0f, 80.0f);
    *out_pure_angle = angle;
}
