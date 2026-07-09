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


    std::string target_type = "unknown";

    bool has_candidate_area = false;
    double candidate_area = 0.0;

    bool has_candidate_center = false;
    float candidate_center_x = 0.0f;
    float candidate_center_y = 0.0f;

    bool has_candidate_size = false;
    int candidate_width = 0;
    int candidate_height = 0;

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

    bool has_reference_x_range = false;
    int reference_x_min = 0;
    int reference_x_max = 0;
    int merged_reference_span_count = 0;
    std::string reference_range_source = "fallback";

    bool has_track_left_boundary = false;
    std::vector<cv::Point> track_left_boundary;

    bool has_track_right_boundary = false;
    std::vector<cv::Point> track_right_boundary;

    bool has_track_region_polygon = false;
    std::vector<cv::Point> track_region_polygon;

    bool has_track_classify_point = false;
    cv::Point track_classify_point;

    bool has_track_classify_bounds = false;
    int track_classify_left_x = 0;
    int track_classify_right_x = 0;
    int track_classify_row_y = 0;

    bool touches_search_top = false;
    bool touch_top_expand = false;
    bool touch_top_skip_refine = false;
    bool touch_top_brick_override = false;

    bool has_expanded_blob_area = false;
    double expanded_blob_area = 0.0;

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

    double timing_search_rect_ms = 0.0;
    double timing_track_boundary_ms = 0.0;
    double timing_red_mask_ms = 0.0;
    double timing_red_band_ms = 0.0;
    double timing_track_classify_ms = 0.0;
    double timing_roi_build_warp_ms = 0.0;
};

struct RoiQualityMetrics
{
    bool valid = false;
    std::string reason;
    double gray_std_top = 0.0;
    double canny_density_top = 0.0;
    double lap_var_top = 0.0;
};

struct RoiTrackRedPrefilterResult
{
    bool has_track_boundaries = false;

    bool has_early_marker_red = false;
    cv::Rect early_marker_rect;

    bool has_recognition_marker_red = false;
    cv::Rect recognition_marker_rect;

    bool has_recognition_brick_red = false;
    cv::Rect recognition_brick_rect;
};

RoiMethod DefaultRoiMethod();
const char* RoiMethodName(RoiMethod method);

bool DetectTrackAwareRedPrefilter(const cv::Mat& frame_bgr,
                                  int early_y_min,
                                  int early_y_max,
                                  int recognition_y_min,
                                  int recognition_y_max,
                                  RoiTrackRedPrefilterResult* out_result);

RoiExtractionResult ExtractRotatedRoi(const cv::Mat& frame_bgr,
                                      int output_size,
                                      RoiMethod roi_method,
                                      bool render_debug);

RoiQualityMetrics ComputeLowInformationRoiMetrics(const cv::Mat& roi_bgr,
                                                  RoiMethod roi_method,
                                                  const RoiExtractionResult& roi_result);

void DrawRoiDebugOverlay(cv::Mat& image_bgr, const RoiExtractionResult& result);
