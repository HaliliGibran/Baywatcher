#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

enum class RoiMethod : int
{
    DIRECT_RED_QUAD = 0,
    IPM_SQUARE_FROM_TOP_EDGE = 1,
};

struct RoiExtractionResult
{
    cv::Mat roi_bgr;
    RoiMethod roi_method = RoiMethod::IPM_SQUARE_FROM_TOP_EDGE;
    std::string status = "miss";
    std::string ipm_reason;

    bool has_blob_box = false;
    cv::Rect blob_box;

    bool has_blob_area = false;
    double blob_area = 0.0;

    bool has_loose_blob_box = false;
    cv::Rect loose_blob_box;

    bool has_loose_blob_area = false;
    double loose_blob_area = 0.0;

    bool has_max_red_contour_box = false;
    cv::Rect max_red_contour_box;

    bool has_max_red_contour_area = false;
    double max_red_contour_area = 0.0;

    bool has_prewhite_max_red_contour_area = false;
    double prewhite_max_red_contour_area = 0.0;

    bool white_crop_clipped = false;
    std::string max_red_reject_stage = "none";

    bool has_search_rect = false;
    cv::Rect search_rect;

    bool has_white_x_range = false;
    int white_x_min = 0;
    int white_x_max = 0;
    int merged_white_span_count = 0;
    std::string white_range_source = "fallback";

    bool touches_search_top = false;

    bool loose_ipm_valid = false;
    std::string loose_ipm_reason = "none";

    bool has_support_rect = false;
    cv::Rect support_rect;

    std::vector<cv::Point2f> blob_quad;
    std::vector<cv::Point2f> blob_quad_final;
    std::vector<cv::Point2f> roi_quad;
    std::vector<cv::Point2f> roi_quad_final;

    bool has_quad_red_fill = false;
    float quad_red_fill = 0.0f;

    bool has_quad_core_fill = false;
    float quad_core_fill = 0.0f;

    bool has_ipm_top_width = false;
    float ipm_top_width = 0.0f;

    bool has_ipm_bottom_width = false;
    float ipm_bottom_width = 0.0f;

    bool has_ipm_left_height = false;
    float ipm_left_height = 0.0f;

    bool has_ipm_right_height = false;
    float ipm_right_height = 0.0f;

    bool has_ipm_backproject_height_ratio = false;
    float ipm_backproject_height_ratio = 0.0f;
};

struct RoiQualityMetrics
{
    bool valid = false;
    std::string reason;
    double gray_std_top = 0.0;
    double canny_density_top = 0.0;
    double lap_var_top = 0.0;
};

RoiMethod DefaultRoiMethod();
const char* RoiMethodName(RoiMethod method);

RoiExtractionResult ExtractRotatedRoi(const cv::Mat& frame_bgr,
                                      int output_size,
                                      RoiMethod roi_method,
                                      const cv::Rect* search_rect = nullptr);

RoiQualityMetrics ComputeLowInformationRoiMetrics(const cv::Mat& roi_bgr,
                                                  RoiMethod roi_method,
                                                  const RoiExtractionResult& roi_result);

void DrawRoiDebugOverlay(cv::Mat& image_bgr, const RoiExtractionResult& result);
