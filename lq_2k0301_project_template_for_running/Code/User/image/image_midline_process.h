#ifndef SMARTCAR_IMAGE_MIDLINE_PROCESS_H
#define SMARTCAR_IMAGE_MIDLINE_PROCESS_H

#include "image_data.h"

// 功能: 构造中线（基于左右候选中线）
// 类型: 图像处理函数
// 关键参数: mid_left/mid_right-输入候选中线(会被原地修改), mode-跟线模式
// - 可能会原地修改 mid_left/mid_right（截头、起点对齐、截取重合段）
// - 输出 mid 及 mid_count
void MID(float (&mid_left)[PT_MAXLEN][2], int32_t* mid_left_count,
         float (&mid_right)[PT_MAXLEN][2], int32_t* mid_right_count,
         float (&mid)[PT_MAXLEN][2], int32_t* mid_count,
         FollowLine& mode);

// 功能: 生成“路径点列”（从 core 以圆弧并轨到中线）
// 类型: 图像处理函数
// 关键参数: mid/mid_count-中线点列, step-路径点间距(像素)
// 生成“路径点列”：从 IMAGE_CORE（做过逆透视后的世界坐标点）用一段圆弧切入 mid 中线，
// 然后继续沿 mid 前进。
// - mid/path 坐标约定同工程其他点列：pts[i][0]=y, pts[i][1]=x（逆透视后的俯视图像素坐标）
// - step 为路径点间距（像素单位），建议用 RESAMPLEDIST*PIXPERMETER
// - 连接点默认选择 mid 上“离 core 足够远”的一个点，避免太短导致数值不稳定
void BuildPathFromCoreToMidlineArc(const float (&mid)[PT_MAXLEN][2], int32_t mid_count,
                                  float (&path)[PT_MAXLEN][2], int32_t* path_count,
                                  float step);

// 功能: 根据中线局部几何给出 pure_angle 的建议预瞄图像行
// 类型: 图像处理函数
// 关键参数: mid/mid_count-输入中线, base_img_y-默认预瞄图像行
// 返回: 建议使用的预瞄图像行（已做边界限制，越小表示看得越远）
int MidLineSuggestPureAnglePreviewImageY(const float (&mid)[PT_MAXLEN][2], int32_t mid_count,
                                         int base_img_y);

#endif
