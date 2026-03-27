#include "zebra.h"
#include "common.h"
#include "image_data.h"
#include <stdio.h>
#include <cmath>

// 功能: 判定一组色块长度是否接近一致
// 类型: 局部功能函数
// 关键参数: runs/run_count-色块长度数组, mean_out-输出平均长度
static bool zebra_check_run_group(const int* runs, int run_count, float* mean_out)
{
    if (mean_out != nullptr)
    {
        *mean_out = 0.0f;
    }
    if (runs == nullptr || run_count <= 0)
    {
        return false;
    }

    float sum = 0.0f;
    for (int i = 0; i < run_count; ++i)
    {
        sum += (float)runs[i];
    }

    const float mean = sum / (float)run_count;
    if (mean_out != nullptr)
    {
        *mean_out = mean;
    }
    if (mean <= 1e-4f)
    {
        return false;
    }

    const float max_allowed_diff = mean * ZEBRA_RUN_SPREAD_RATIO;
    for (int i = 0; i < run_count; ++i)
    {
        if (std::fabs((float)runs[i] - mean) > max_allowed_diff)
        {
            return false;
        }
    }
    return true;
}

// 功能: 斑马线检测（黑白交替 + 色块长度一致性）
// 类型: 图像处理函数
// 关键参数: img-二值图
bool zebra_detection(const uint8_t (&img)[IMAGE_H][IMAGE_W])
{
    // 黑白交替检测 + 黑白色块长度检测
    for (int i = 0; i < NEAR_DETECT_MAX && i < pts_left.pts_count ; i++) 
    {        
        int y = pts_left.pts[i][0]; // 局部: 当前检测点 y
        int x = pts_left.pts[i][1]; // 局部: 当前检测点 x

        // 防御：避免非法点导致越界访问
        if (y < 0 || y >= IMAGE_H) continue;
        if (x < 0 || x >= (IMAGE_W - 1)) continue;

        const int scan_begin = x + 1;
        int scan_end = x + ZEBRA_STEP;
        // if (scan_end > 99) scan_end = 99;
        if (scan_end > IMAGE_W - 1) scan_end = IMAGE_W - 1;
        if (scan_begin >= scan_end) continue;

        int runs[PT_MAXLEN] = {0};
        uint8_t colors[PT_MAXLEN] = {0};
        int run_count = 0;

        int current_color = img[y][scan_begin];
        int current_len = 1;

        // 从左侧点向右侧提取连续黑白色块长度
        for (int current_x = scan_begin + 1; current_x <= scan_end; ++current_x)
        {
            const int color = img[y][current_x];
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
                colors[run_count] = (uint8_t)current_color;
                run_count++;
                current_color = color;
                current_len = 1;
            }
        }

        if (run_count < PT_MAXLEN)
        {
            runs[run_count] = current_len;
            colors[run_count] = (uint8_t)current_color;
            run_count++;
        }

        if (run_count < ZEBRA_MIN_RUNS)
        {
            continue;
        }

        int white_runs[PT_MAXLEN] = {0};
        int black_runs[PT_MAXLEN] = {0};
        int white_count = 0;
        int black_count = 0;

        for (int k = 0; k < run_count; ++k)
        {
            if (colors[k] == WHITE_IN_GRAY)
            {
                white_runs[white_count++] = runs[k];
            }
            else
            {
                black_runs[black_count++] = runs[k];
            }
        }

        if (white_count < 2 || black_count < 2)
        {
            continue;
        }

        if (BW_ENABLE_ZEBRA_RUN_LENGTH_CHECK == 0)
        {
            return true;
        }

        float white_mean = 0.0f;
        float black_mean = 0.0f;
        if (!zebra_check_run_group(white_runs, white_count, &white_mean) ||
            !zebra_check_run_group(black_runs, black_count, &black_mean))
        {
            continue;
        }

        const float mean_ref = (white_mean > black_mean) ? white_mean : black_mean;
        if (mean_ref <= 1e-4f)
        {
            continue;
        }

        // 白黑块均值接近，才认为更像规则斑马线。
        if (std::fabs(white_mean - black_mean) <= mean_ref * ZEBRA_RUN_DIFF_RATIO)
        {
            return true;
        }
    }

    return false;
}
