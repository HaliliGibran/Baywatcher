#include "zebra.h"
#include "common.h"
#include "image_data.h"
#include <stdio.h>


// 功能: 斑马线检测（黑白交替计数）
// 类型: 图像处理函数
// 关键参数: img-二值图
bool zebra_detection(const uint8_t (&img)[IMAGE_H][IMAGE_W])
{
    // 黑白交替检测
    for (int i = 0; i < NEAR_DETECT_MAX && i < pts_left.pts_count ; i++) 
    {        
        int y = pts_left.pts[i][0]; // 局部: 当前检测点 y
        int x = pts_left.pts[i][1]; // 局部: 当前检测点 x

        // 防御：避免非法点导致越界访问
        if (y < 0 || y >= IMAGE_H) continue;
        if (x < 0 || x >= (IMAGE_W - 2)) continue;
        
        int left_zebra_white_black_change_count = 0;
        bool current_x_is_black = false;

        // 从左侧点向右侧搜索ZEBRA_STEP步
        for (int step = 1; step <= ZEBRA_STEP && (x + step) < 100 && (x + step + 1) < IMAGE_W; step++) 
        {

            int current_x = x + step;
            if (current_x_is_black == false && img[y][current_x] == WHITE_IN_GRAY && img[y][current_x + 1] == WHITE_IN_GRAY) 
            {
                current_x_is_black = true;
                left_zebra_white_black_change_count++;
            } 
            else if (current_x_is_black == true && img[y][current_x] == BLACK_IN_GRAY && img[y][current_x + 1] == BLACK_IN_GRAY) 
            {
                current_x_is_black = false;
                left_zebra_white_black_change_count++;
            }

            if(left_zebra_white_black_change_count >= 5) 
            {
                break;
            }

            
        }

        if (left_zebra_white_black_change_count >= 5)
        {
            return true;
        }

    }

    return false;
}
