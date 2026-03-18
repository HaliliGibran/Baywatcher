#include <cmath>
#include <cstdio>
#include "common.h"
#include "image_data.h"
#include "image_handle.h"
#include "image_math.h"

// 功能: 十字状态转字符串（日志用）
// 类型: 局部功能函数
// 关键参数: s-十字状态
static inline const char* crossing_state_str(CrossingState s)
{
    switch (s)
    {
        case CrossingState::CROSSING_NONE: return "NONE";
        case CrossingState::CROSSING_IN: return "IN";
        case CrossingState::CROSSING_RUNNING: return "RUNNING";
        default: return "UNKNOWN";
    }
}

// 角点在 pts_filter 中的最近点索引（给“远端线/十字”相关逻辑使用）。
// 说明：pts_left.corner_id / pts_right.corner_id 是重采样数组(pts_resample)中的索引；
// 这里额外给出 pts_filter（未重采样）中的近似对应索引。
// 作用域: 文件内全局变量，供十字远端线计算使用
int pts_left_corner_id = -1;
int pts_right_corner_id = -1;

// 作用域: 文件内静态变量，十字状态机计数与上次状态
static int g_lost_line_counter = 0;
static CrossingState g_last_state = CrossingState::CROSSING_NONE;

// 功能: 十字状态机复位
// 类型: 图像处理函数
// 关键参数: 无
void crossing_reset()
{
    g_lost_line_counter = 0;
    g_last_state = CrossingState::CROSSING_NONE;
    crossing_state = CrossingState::CROSSING_NONE;
    pts_left_corner_id = -1;
    pts_right_corner_id = -1;
    image_reset_far_line_state();
}

// 功能: 十字路口状态机更新
// 类型: 图像处理函数
// 关键参数: 无（使用全局 pts_left/pts_right 等）
void crossing_update()
{
    const CrossingState prev_state = crossing_state;

    if (g_last_state != crossing_state)
    {
        g_lost_line_counter = 0;
        g_last_state = crossing_state;
    }

    switch (crossing_state) 
    {
        case CrossingState::CROSSING_IN:
        {
            if (pts_left.pts_count == 0 && pts_right.pts_count == 0) 
            {
                g_lost_line_counter++;
            } 
            else 
            {
                g_lost_line_counter = 0;
            }

            if (g_lost_line_counter >= FRAME_THRESHOLD_crossing_lost_line_counter) // 连续数帧未检测到边线，进入十字中心阶段
            {
                crossing_state = CrossingState::CROSSING_RUNNING;
                g_lost_line_counter = 0;
            }
        }
        break;

        case CrossingState::CROSSING_RUNNING:
        {    
            if (pts_left.pts_count > 0 && pts_right.pts_count > 0)
                crossing_state = CrossingState::CROSSING_NONE;
            break;
        }    
        default:
            crossing_state = CrossingState::CROSSING_NONE;
            break;
    }

    if (crossing_state != prev_state)
    {
        std::printf("[TRACK] %s->%s\r\n"
                    "vote(cross=%d circle=%d)\r\n"
                    "protect=%d\r\n",
                    crossing_state_str(prev_state), crossing_state_str(crossing_state),
                    g_track_debug.crossing_vote, g_track_debug.circle_vote, g_track_debug.protect);
    }
}


// 功能: 十字远端线检测（补足中线）
// 类型: 图像处理函数
// 关键参数: img-二值图
void crossing_far_line_check(const uint8_t (&img)[IMAGE_H][IMAGE_W])
{
    image_reset_far_line_state();


    if(pts_left.corner_found && pts_right.corner_found && pts_left.corner_id >=9 && pts_right.corner_id >=9)
    {
        if (pts_left.corner_found && pts_left.corner_id >= 0 && pts_left.corner_id < pts_left.pts_resample_count)
        {
            pts_left_corner_id = (int)pts_left.resample_src_id[pts_left.corner_id];
        }
        else
        {
            pts_left_corner_id = -1;
        }

        if (pts_right.corner_found && pts_right.corner_id >= 0 && pts_right.corner_id < pts_right.pts_resample_count)
        {
            pts_right_corner_id = (int)pts_right.resample_src_id[pts_right.corner_id];
        }
        else
        {
            pts_right_corner_id = -1;
        }

        if(pts_left_corner_id < 0 || pts_right_corner_id < 0)
        {
            return;
        }

        // 防御：resample_src_id 的索引来源不一定严格等同于 pts[] 的索引体系，先夹紧到有效范围。
        if (pts_left.pts_count <= 0 || pts_right.pts_count <= 0)
        {
            return;
        }
        if (pts_left_corner_id >= pts_left.pts_count) pts_left_corner_id = pts_left.pts_count - 1;
        if (pts_right_corner_id >= pts_right.pts_count) pts_right_corner_id = pts_right.pts_count - 1;

        int32_t lx = pts_left.pts[pts_left_corner_id][1];
        int32_t ly = pts_left.pts[pts_left_corner_id][0];
        int32_t rx = pts_right.pts[pts_right_corner_id][1];
        int32_t ry = pts_right.pts[pts_right_corner_id][0];

        int32_t dx = rx - lx;
        int32_t dy = ry - ly;

        // 约定：左点在左、右点在右。若出现 dx<0（极少数异常），交换以保证方向一致性。
        if (dx < 0)
        {
            const int32_t tx = lx; lx = rx; rx = tx;
            const int32_t ty = ly; ly = ry; ry = ty;
            dx = -dx;
            dy = -dy;
        }

        const int mid_x = round((lx + rx) * 0.5f);
        const int mid_y = round((ly + ry) * 0.5f);

        const int compare_dist_x = fabs(dx) + fabs(dy);

        // === 方向性优化 ===
        // 目标：从角点连线的中点向“远处”(图像上方)试探。
        // 使用连线向量 v=(dx,dy) 的法向量 n=(dy,-dx)。当 dx>0 时，n 的 y 分量为负，天然指向上方。
        // 用 L=max(|dx|,|dy|) 做归一化，避免 dy≈0 时的除法爆炸与方向抖动。
        const int32_t adx = (dx >= 0) ? dx : -dx;
        const int32_t ady = (dy >= 0) ? dy : -dy;
        const int32_t L = (adx > ady) ? adx : ady;
        if (L <= 0)
        {
            return;
        }

        const int y_limit = (ly > ry) ? ly : ry; // 只在“角点以上(更远处)”尝试

        // 每步大约 1 像素，最多向前试探到图像上半部分。
        const int max_step = IMAGE_H;
        for (int step = 1; step <= max_step; ++step)
        {
            const int start_x = mid_x + (int)lroundf(((float)step * (float)dy) / (float)L);
            const int start_y = mid_y - (int)lroundf(((float)step * (float)dx) / (float)L);

            if (!(start_x > 10 && start_x < IMAGE_W - 10 && start_y > 10))
            {
                continue;
            }

            if (start_y > y_limit)
            {
                continue;
            }

            bool if_find_far_left_start_pt = false;
            bool if_find_far_right_start_pt = false;
            int32_t far_left_start_pt[2] = {0, 0};
            int32_t far_right_start_pt[2] = {0, 0};
            int32_t dummy_count = 0;

            // 先只找起点，不爬线
            SearchLine_LptEx(img, start_x, start_y, pts_far_left.pts, &dummy_count, false, &if_find_far_left_start_pt, far_left_start_pt);
            SearchLine_RptEx(img, start_x, start_y, pts_far_right.pts, &dummy_count, false, &if_find_far_right_start_pt, far_right_start_pt);

            if(if_find_far_left_start_pt && if_find_far_right_start_pt)
            {
                if(far_right_start_pt[1] - far_left_start_pt[1] > 0 && far_right_start_pt[1] - far_left_start_pt[1] < compare_dist_x)
                {
                    SearchLineAdaptive_Left(img, far_left_start_pt[0], far_left_start_pt[1], pts_far_left.pts, &pts_far_left.pts_count);
                    SearchLineAdaptive_Right(img, far_right_start_pt[0], far_right_start_pt[1], pts_far_right.pts, &pts_far_right.pts_count);

                    // 走完整流水线，得到远端 mid
                    process_line(true, pts_far_left);
                    process_line(false, pts_far_right);

                    if_find_far_line = (pts_far_left.mid_count > 0 && pts_far_right.mid_count > 0);
                    break;
                }   
            }

        }

    }
    else
    {
        int start_y = SET_IMAGE_CORE_Y;
        int start_y1 = IMAGE_H;
        int start_y2 = IMAGE_H;
        if(pts_left.pts_count > 0)
            start_y1 = pts_left.pts[clip(pts_left.pts_count - 1 , 0 , pts_left.pts_count - 1)][0] ;
        if(pts_right.pts_count > 0)
            start_y2 = pts_right.pts[clip(pts_right.pts_count - 1 , 0 , pts_right.pts_count - 1)][0] ;

        int start_y_min = (start_y1 < start_y2)? start_y1 : start_y2;
        start_y = (start_y_min < start_y)? start_y_min : start_y;
        for (; start_y > 20; start_y--)
        {
            const int start_x = SET_IMAGE_CORE_X;

            bool if_find_far_left_start_pt = false;
            bool if_find_far_right_start_pt = false;
            int32_t far_left_start_pt[2] = {0, 0};
            int32_t far_right_start_pt[2] = {0, 0};
            int32_t dummy_count = 0;

            // 先只找起点，不爬线
            SearchLine_LptEx(img, start_x, start_y, pts_far_left.pts, &dummy_count, false, &if_find_far_left_start_pt, far_left_start_pt);
            SearchLine_RptEx(img, start_x, start_y, pts_far_right.pts, &dummy_count, false, &if_find_far_right_start_pt, far_right_start_pt);

            if(if_find_far_left_start_pt && if_find_far_right_start_pt && far_right_start_pt[1] - far_left_start_pt[1] < 50)
            {
                if(far_right_start_pt[1] - far_left_start_pt[1] > 0)
                {
                    SearchLineAdaptive_Left(img, far_left_start_pt[0], far_left_start_pt[1], pts_far_left.pts, &pts_far_left.pts_count);
                    SearchLineAdaptive_Right(img, far_right_start_pt[0], far_right_start_pt[1], pts_far_right.pts, &pts_far_right.pts_count);

                    // 走完整流水线，得到远端 mid
                    process_line(true, pts_far_left);
                    process_line(false, pts_far_right);

                    if_find_far_line = (pts_far_left.mid_count > 0 && pts_far_right.mid_count > 0);
                    break;
                }   
            }

        }

    }
}
