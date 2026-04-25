#include "zebra.h"
#include "common.h"
#include "image_data.h"
#include <stdio.h>
#include <cmath>
#include <limits>

// 功能: 在一条边线上按目标 y 获取 x，优先精确匹配，其次做线性插值，最后允许最近点小偏差兜底
// 类型: 局部功能函数
static bool zebra_find_edge_x_at_y(const int32_t (&pts)[PT_MAXLEN][2], int pts_count, int target_y, float* out_x)
{
    if (out_x == nullptr || pts_count <= 0)
    {
        return false;
    }

    for (int i = 0; i < pts_count; ++i)
    {
        if (pts[i][0] == target_y)
        {
            *out_x = static_cast<float>(pts[i][1]);
            return true;
        }
    }

    for (int i = 0; i < pts_count - 1; ++i)
    {
        const int y0 = pts[i][0];
        const int y1 = pts[i + 1][0];
        const int min_y = (y0 < y1) ? y0 : y1;
        const int max_y = (y0 > y1) ? y0 : y1;
        if (target_y < min_y || target_y > max_y || y0 == y1)
        {
            continue;
        }

        const float t = static_cast<float>(target_y - y0) / static_cast<float>(y1 - y0);
        *out_x = static_cast<float>(pts[i][1]) + t * static_cast<float>(pts[i + 1][1] - pts[i][1]);
        return true;
    }

    int best_abs_dy = std::numeric_limits<int>::max();
    int best_x = 0;
    for (int i = 0; i < pts_count; ++i)
    {
        const int abs_dy = std::abs(pts[i][0] - target_y);
        if (abs_dy < best_abs_dy)
        {
            best_abs_dy = abs_dy;
            best_x = pts[i][1];
        }
    }

    if (best_abs_dy <= ZEBRA_EDGE_MATCH_Y_TOL)
    {
        *out_x = static_cast<float>(best_x);
        return true;
    }

    return false;
}

// 功能: 在单条扫描线上统计黑白 run-length，并按斑马线规则判断是否命中
// 类型: 局部功能函数
static bool zebra_check_row_pattern(const uint8_t (&img)[IMAGE_H][IMAGE_W], int y, int x_left, int x_right)
{
    if (y < 0 || y >= IMAGE_H)
    {
        return false;
    }

    int scan_begin = x_left + ZEBRA_EDGE_MARGIN;
    int scan_end = x_right - ZEBRA_EDGE_MARGIN;
    if (scan_begin < 0) scan_begin = 0;
    if (scan_end > IMAGE_W - 1) scan_end = IMAGE_W - 1;
    if (scan_end - scan_begin + 1 < 8)
    {
        return false;
    }

    int runs[PT_MAXLEN] = {0};
    uint8_t colors[PT_MAXLEN] = {0};
    int run_count = 0;

    int current_color = img[y][scan_begin];
    int current_len = 1;
    for (int x = scan_begin + 1; x <= scan_end; ++x)
    {
        const int color = img[y][x];
        if (color == current_color)
        {
            current_len++;
        }
        else
        {
            if (run_count >= PT_MAXLEN)
            {
                break;
            }
            runs[run_count] = current_len;
            colors[run_count] = static_cast<uint8_t>(current_color);
            run_count++;
            current_color = color;
            current_len = 1;
        }
    }

    if (run_count < PT_MAXLEN)
    {
        runs[run_count] = current_len;
        colors[run_count] = static_cast<uint8_t>(current_color);
        run_count++;
    }

    // 忽略首尾色块，只看赛道内部更稳定的交替结构。
    if (run_count <= 3)
    {
        return false;
    }
    const int start_idx = 1;
    const int end_idx = run_count - 2;
    const int effective_run_count = end_idx - start_idx + 1;
    if (effective_run_count <= 0)
    {
        return false;
    }

    int white_runs[PT_MAXLEN] = {0};
    int black_runs[PT_MAXLEN] = {0};
    int white_count = 0;
    int black_count = 0;

    for (int i = start_idx; i <= end_idx; ++i)
    {
        if (colors[i] == WHITE_IN_GRAY)
        {
            white_runs[white_count++] = runs[i];
        }
        else if (colors[i] == BLACK_IN_GRAY)
        {
            black_runs[black_count++] = runs[i];
        }
    }

    const int transitions = effective_run_count - 1;
    const bool enough_color_runs =
        (white_count >= ZEBRA_MIN_COLOR_RUNS && black_count >= ZEBRA_MIN_COLOR_RUNS);
    const bool enough_switches = (transitions >= ZEBRA_MIN_SWITCHES);
    if (!enough_color_runs && !enough_switches)
    {
        return false;
    }
    return true;
}

// 功能: 斑马线检测（同 y 左右边线之间横向扫描，检查黑白交替与色块宽度近似一致）
// 类型: 图像处理函数
// 关键参数: img-二值图
bool zebra_detection(const uint8_t (&img)[IMAGE_H][IMAGE_W])
{
    int hit_rows = 0;
    bool has_last_y = false;
    int last_y = 0;

    for (int i = 0; i < NEAR_DETECT_MAX && i < pts_left.pts_count; i += ZEBRA_ROW_SAMPLE_STEP)
    {
        const int y = pts_left.pts[i][0];
        if (y < 0 || y >= IMAGE_H)
        {
            continue;
        }

        // 避免连续使用几乎同一条扫描线。
        if (has_last_y && std::abs(y - last_y) < ZEBRA_ROW_SAMPLE_STEP)
        {
            continue;
        }

        float left_x = static_cast<float>(pts_left.pts[i][1]);
        float right_x = 0.0f;
        if (!zebra_find_edge_x_at_y(pts_right.pts, pts_right.pts_count, y, &right_x))
        {
            continue;
        }

        int x_left = static_cast<int>(std::lround(left_x));
        int x_right = static_cast<int>(std::lround(right_x));
        if (x_left > x_right)
        {
            const int tmp = x_left;
            x_left = x_right;
            x_right = tmp;
        }

        if (x_right - x_left + 1 < 12)
        {
            continue;
        }

        last_y = y;
        has_last_y = true;
        if (zebra_check_row_pattern(img, y, x_left, x_right))
        {
            hit_rows++;
            if (hit_rows >= ZEBRA_MIN_HIT_ROWS)
            {
                return true;
            }
        }
    }

    return false;
}
