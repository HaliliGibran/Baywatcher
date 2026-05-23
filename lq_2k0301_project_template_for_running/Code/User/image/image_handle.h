#ifndef SMARTCAR_IMAGE_HANDLE_H
#define SMARTCAR_IMAGE_HANDLE_H

#include <cstdint>
#include "common.h"
#include "image_data.h"

// 功能: 边线寻线（左线起点 + 爬线）
// 类型: 图像处理函数
// 关键参数: img-二值图, start_x/start_y-起点搜索中心, pts/pts_count-输出边线点列
void SearchLine_Lpt(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t start_x, int32_t start_y,
                   int32_t (&pts)[PT_MAXLEN][2], int32_t* pts_count);
// 功能: 边线寻线（右线起点 + 爬线）
// 类型: 图像处理函数
// 关键参数: img-二值图, start_x/start_y-起点搜索中心, pts/pts_count-输出边线点列
void SearchLine_Rpt(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t start_x, int32_t start_y,
                   int32_t (&pts)[PT_MAXLEN][2], int32_t* pts_count);

// 功能: 左侧爬线（迷宫法）
// 类型: 图像处理函数
// 关键参数: img-二值图, h/w-起点像素坐标, pts/line_num-输出点列
void SearchLineAdaptive_Left(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t h, int32_t w,
                            int32_t (&pts)[PT_MAXLEN][2], int32_t* line_num);
// 功能: 右侧爬线（迷宫法）
// 类型: 图像处理函数
// 关键参数: img-二值图, h/w-起点像素坐标, pts/line_num-输出点列
void SearchLineAdaptive_Right(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t h, int32_t w,
                             int32_t (&pts)[PT_MAXLEN][2], int32_t* line_num);

// 功能: 左侧寻线扩展版（可只找起点或完整爬线）
// 类型: 图像处理函数
// 关键参数: climb_line-是否执行爬线, out_find_start_pt/out_start_pt-起点输出
void SearchLine_LptEx(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t start_x, int32_t start_y,
                     int32_t (&pts)[PT_MAXLEN][2], int32_t* pts_count,
                     bool climb_line, bool* out_find_start_pt, int32_t (&out_start_pt)[2]);
// 功能: 右侧寻线扩展版（可只找起点或完整爬线）
// 类型: 图像处理函数
// 关键参数: climb_line-是否执行爬线, out_find_start_pt/out_start_pt-起点输出
void SearchLine_RptEx(const uint8_t (&img)[IMAGE_H][IMAGE_W], int32_t start_x, int32_t start_y,
                     int32_t (&pts)[PT_MAXLEN][2], int32_t* pts_count,
                     bool climb_line, bool* out_find_start_pt, int32_t (&out_start_pt)[2]);

// 功能: 点列重采样（严格等距）
// 类型: 图像处理函数
// 关键参数: dist-采样间距(像素), out_src_index-可选输出源索引映射
void GetLinesResample(float (&pts_in)[PT_MAXLEN][2], int32_t* num1,
                      float (&pts_out)[PT_MAXLEN][2], int32_t* num2, float dist,
                      int32_t* out_src_index = nullptr);

// 功能: 计算点列局部弯曲强度（1-cos）
// 类型: 图像处理函数
// 关键参数: dist-局部跨度(点数), curv_out-输出曲率
void local_curvature_points(const float (&pts_in)[PT_MAXLEN][2], int* pts_in_count,
                            float (&curv_out)[PT_MAXLEN], int* curv_out_count, int dist);

// 功能: 曲率一维极大值抑制
// 类型: 图像处理函数
// 关键参数: kernel-NMS窗口大小, curv_out-抑制后曲率
void nms_curvature(float (&curv_in)[PT_MAXLEN], int* curv_in_count,
                   float (&curv_out)[PT_MAXLEN], int* curv_out_count, int kernel);

// 功能: 通用边线处理流水线（逆透视→滤波→重采样→曲率/角点→中线）
// 类型: 图像处理函数
// 关键参数: is_left-是否为左线, pts_in/pts_count-输入边线点, mid_line/mid_count-输出中线
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
    int32_t (&resample_src_id)[PT_MAXLEN]);

// 功能: process_line 的结构体封装版（避免长参数列表）
// 类型: 图像处理函数
// 关键参数: ctx-边线处理上下文（输入/输出均在此结构体）
static inline void process_line(bool is_left, pts_well_processed& ctx)
{
    process_line(is_left,
                 ctx.pts, &ctx.pts_count,
                 ctx.pts_inv, &ctx.pts_inv_count,
                 ctx.pts_filter, &ctx.pts_filter_count,
                 ctx.pts_resample, &ctx.pts_resample_count,
                 ctx.curvature, &ctx.curvature_num,
                 ctx.curvature_nms, &ctx.curvature_nms_num,
                 ctx.mid, &ctx.mid_count,
                 &ctx.is_straight,
                 &ctx.corner_found, &ctx.corner_id, ctx.corner,
                 &ctx.is_curve,
                 ctx.resample_src_id);
}

// 功能: 由路径点列计算“pure angle”（度，右转为负/左转为正）
// 类型: 图像处理函数
// 关键参数: path/path_count-路径点列, out_pure_angle-输出角度
// 说明:
// - 当前版本使用“车轴 -> 预瞄点”的割线方向计算偏航角
// - 预瞄点默认取 PUREANGLE_PREVIEW_BASE_IMAGE_Y 对应位置
// - 若中线几何显示前方已有明显弯道，会自动把预瞄点向更远处前推
void CalculatePureAngleFromPath(const float (&path)[PT_MAXLEN][2], int32_t path_count, float* out_pure_angle);

// 功能: 复位 pure_angle 预瞄过渡状态
// 类型: 图像处理函数
// 关键参数: 无
void ResetPureAnglePreviewTransitionState();

#endif
