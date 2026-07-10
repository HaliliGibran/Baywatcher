#include "roi_runtime_geometry.h"

#include "common.h"
#include "recognition_white_reference.h"
#include "transform_table.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

namespace {

using steady_clock_t = std::chrono::steady_clock;

static double elapsed_ms(const steady_clock_t::time_point& begin,
                         const steady_clock_t::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

constexpr int kIpmFrameWidth = BW_RECOG_TRANSFORM_TABLE_WIDTH;
constexpr int kIpmFrameHeight = BW_RECOG_TRANSFORM_TABLE_HEIGHT;
constexpr double kMinRoiEdgeLength = 4.0;
constexpr double kMinBackprojectedQuadArea = 6.0;


constexpr int kTaskBrickSearchYMin = BW_RECOG_BRICK_SEARCH_Y_MIN;
constexpr int kTaskBrickSearchYMax = BW_RECOG_BRICK_SEARCH_Y_MAX;
constexpr int kTaskTrackBoundaryYMin = BW_RECOG_TRACK_BOUNDARY_Y_MIN;
constexpr int kTaskTrackTraceTopY = kTaskTrackBoundaryYMin;
constexpr int kTaskMarkerTriggerYMin = BW_RECOG_TRIGGER_SEARCH_Y_MIN;
constexpr int kTaskMarkerExpandedYMin =
    kTaskMarkerTriggerYMin - BW_RECOG_MARKER_ROI_TOP_EXPAND_PIXELS;
constexpr int kTaskMarkerSearchYMin =
    (kTaskMarkerExpandedYMin > kTaskTrackBoundaryYMin)
        ? kTaskMarkerExpandedYMin
        : kTaskTrackBoundaryYMin;
constexpr int kTaskMarkerSearchYMax = BW_RECOG_TRIGGER_SEARCH_Y_MAX;
constexpr int kTaskRedScoreThreshold = 140;
constexpr int kTaskRedMinR = 90;
constexpr int kTaskRedDomThreshold = 80;
constexpr int kTaskEdgeExpandStep = 4;
constexpr int kTaskEdgeExpandMaxSteps = 12;
constexpr int kTaskMinBandArea = 8;
constexpr int kTaskMinBandWidth = 3;
constexpr int kTaskMinBandHeight = 2;
constexpr double kTaskMinBandAspectRatio = 1.4;
constexpr int kWhiteReferenceRowY = BW_RECOG_WHITE_REFERENCE_ROW_Y;
constexpr int kWhiteMaxSaturation = 60;
constexpr int kWhiteMinValue = 150;
constexpr int kWhiteMinRgb = 165;
constexpr int kWhiteMaxChannelDiff = 40;
constexpr int kWhiteMinSpanWidth = 60;
constexpr int kTrackBrickOuterExpandPixels = BW_RECOG_TRACK_BRICK_OUTER_EXPAND_PIXELS;
constexpr int kTrackBrickInnerMaxPixels = BW_RECOG_TRACK_BRICK_INNER_MAX_PIXELS;
constexpr int kTrackArtificialBlackBorderWidth = 1;
constexpr int kTrackMazeMaxSteps = BW_RECOG_TRACK_MAZE_MAX_STEPS;
constexpr int kTrackLazyMaskHalfWindow = BW_RECOG_TRACK_LAZY_MASK_HALF_WINDOW;
constexpr unsigned char kTrackWhitePixel = 255;
constexpr unsigned char kTrackNonWhitePixel = 0;

constexpr int kTrackDirectionFront[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
constexpr int kTrackDirectionFrontLeft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
constexpr int kTrackDirectionFrontRight[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

constexpr double kIpmShallowLowInfoMaxHeightRatio = 0.45;
constexpr double kIpmShallowLowInfoMaxGrayStd = 12.0;
constexpr double kIpmShallowLowInfoMaxCannyDensity = 0.05;
constexpr double kIpmShallowLowInfoMaxLapVar = 40.0;

const cv::Matx33d kFinalToUndist(
    0.576841020499166, -0.564282531194324, 67.2523674241966,
    -0.0194045231729093, 0.0244401737967942, 30.2201147504512,
    -4.45632798574019e-05, -0.00351632130124832, 1.0);

constexpr double kBaseCalibrationFrameWidth = 320.0;
constexpr double kBaseCalibrationFrameHeight = 240.0;
constexpr double kFx = 93.799205262881;
constexpr double kFy = 93.7590442901101;
constexpr double kSkew = 0.0;
constexpr double kCx = 162.317197932494;
constexpr double kCy = 119.745422640966;
constexpr double kK1 = -0.0187626068381286;
constexpr double kK2 = -0.00611272459781888;
constexpr double kK3 = 0.0;
constexpr double kP1 = 0.0;
constexpr double kP2 = 0.0;

struct DirectEdgeInfo
{
    cv::Point2f center;
    cv::Point2f midpoint;
    cv::Point2f edge_vector;
    cv::Point2f left_point;
    cv::Point2f right_point;
    int left_index = 0;
    int right_index = 1;
    float edge_length = 0.0f;
};

struct FinalEdgeInfo
{
    cv::Point2f center_final;
    cv::Point2f midpoint_final;
    cv::Point2f final_edge_vector;
    cv::Point2f left_point_raw;
    cv::Point2f right_point_raw;
    cv::Point2f left_point_final;
    cv::Point2f right_point_final;
    int left_index = 0;
    int right_index = 1;
    float raw_edge_length = 0.0f;
    float final_edge_length = 0.0f;
};

struct BuildRoiQuadResult
{
    bool valid = false;
    std::string status = "blob_only";
    std::string ipm_reason;
    std::vector<cv::Point2f> roi_quad;
    std::vector<cv::Point2f> blob_quad_final;
    std::vector<cv::Point2f> roi_quad_final;
    bool has_raw_height_ratio = false;
    float raw_height_ratio = 0.0f;
};

struct TaskTrackBoundaryState
{
    bool valid = false;
    int top_y = 0;
    int bottom_y = 0;
    int seed_y = 0;
    int seed_center_x = -1;
    int seed_left_x = -1;
    int seed_right_x = -1;
    int envelope_x_min = 0;
    int envelope_x_max = 0;
    std::vector<cv::Point> left_points;
    std::vector<cv::Point> right_points;
    std::vector<cv::Point> region_polygon;
    std::vector<int> region_left_x_by_row;
    std::vector<int> region_right_x_by_row;
};

enum class TaskTrackCandidateType
{
    UNKNOWN = 0,
    MARKER,
    ROADBLOCK,
};

struct TaskTrackClassification
{
    TaskTrackCandidateType type = TaskTrackCandidateType::UNKNOWN;
    cv::Point classify_point = cv::Point(-1, -1);
    int left_boundary_x = -1;
    int right_boundary_x = -1;
    int boundary_row_y = -1;
};

struct TaskTrackRowSearchBands
{
    bool has_marker = false;
    int marker_x0 = -1;
    int marker_x1 = -1;
    bool has_left_brick = false;
    int left_brick_x0 = -1;
    int left_brick_x1 = -1;
    bool has_right_brick = false;
    int right_brick_x0 = -1;
    int right_brick_x1 = -1;
};

static int TrackBrickInnerPixelsAtRow(int row_y)
{
    if (kTrackBrickInnerMaxPixels <= 0)
    {
        return 0;
    }

    const int top_y = kTaskBrickSearchYMin;
    const int bottom_y = std::max(top_y, kTaskBrickSearchYMax - 1);
    if (row_y <= top_y || bottom_y <= top_y)
    {
        return 0;
    }
    if (row_y >= bottom_y)
    {
        return kTrackBrickInnerMaxPixels;
    }

    const int span = bottom_y - top_y;
    const int offset = row_y - top_y;
    return (offset * kTrackBrickInnerMaxPixels + span / 2) / span;
}

static TaskTrackRowSearchBands BuildTaskTrackRowSearchBands(int left_x,
                                                            int right_x,
                                                            int image_width,
                                                            int row_y)
{
    TaskTrackRowSearchBands bands;
    if (image_width <= 0 || left_x < 0 || right_x < left_x)
    {
        return bands;
    }

    const bool left_on_artificial_edge =
        left_x <= kTrackArtificialBlackBorderWidth;
    const bool right_on_artificial_edge =
        right_x >= image_width - 1 - kTrackArtificialBlackBorderWidth;
    const int inner_pixels = TrackBrickInnerPixelsAtRow(row_y);

    // The outermost black columns are synthetic crawl boundaries, not physical brick edges.
    bands.left_brick_x0 = std::max(0, left_x - kTrackBrickOuterExpandPixels);
    bands.left_brick_x1 = std::min(
        image_width - 1,
        left_x + (left_on_artificial_edge ? 0 : inner_pixels));
    bands.has_left_brick = bands.left_brick_x0 <= bands.left_brick_x1;

    bands.right_brick_x0 = std::max(
        0,
        right_x - (right_on_artificial_edge ? 0 : inner_pixels));
    bands.right_brick_x1 = std::min(image_width - 1, right_x + kTrackBrickOuterExpandPixels);
    bands.has_right_brick = bands.right_brick_x0 <= bands.right_brick_x1;

    if (bands.has_left_brick &&
        bands.has_right_brick &&
        bands.left_brick_x1 >= bands.right_brick_x0)
    {
        const int split_x = (left_x + right_x) / 2;
        bands.left_brick_x1 = std::min(bands.left_brick_x1, split_x);
        bands.right_brick_x0 = std::max(bands.right_brick_x0, split_x + 1);
        bands.has_left_brick = bands.left_brick_x0 <= bands.left_brick_x1;
        bands.has_right_brick = bands.right_brick_x0 <= bands.right_brick_x1;
    }

    bands.marker_x0 = std::max(
        left_x + 1,
        bands.has_left_brick ? bands.left_brick_x1 + 1 : left_x + 1);
    bands.marker_x1 = std::min(
        right_x - 1,
        bands.has_right_brick ? bands.right_brick_x0 - 1 : right_x - 1);
    bands.marker_x0 = std::max(0, bands.marker_x0);
    bands.marker_x1 = std::min(image_width - 1, bands.marker_x1);
    bands.has_marker = bands.marker_x0 <= bands.marker_x1;
    return bands;
}

static BuildRoiQuadResult BuildRoiQuadFromBlobQuad(const std::vector<cv::Point2f>& blob_quad,
                                                   int image_width,
                                                   int image_height,
                                                   RoiMethod roi_method);
static bool ClampRectToImage(const cv::Rect& rect,
                             int image_width,
                             int image_height,
                             cv::Rect* out_rect);

static bool IsFinitePoint(const cv::Point2f& p)
{
    return std::isfinite(p.x) && std::isfinite(p.y);
}

static bool MergeRowSpanIntoEnvelope(int start,
                                     int end,
                                     int min_span_width,
                                     int* merged_start,
                                     int* merged_end,
                                     int* merged_count)
{
    if (merged_start == nullptr || merged_end == nullptr || merged_count == nullptr)
    {
        return false;
    }
    if (start < 0 || end < start || end - start + 1 < min_span_width)
    {
        return false;
    }

    if (*merged_count == 0)
    {
        *merged_start = start;
        *merged_end = end;
    }
    else
    {
        *merged_start = std::min(*merged_start, start);
        *merged_end = std::max(*merged_end, end);
    }
    ++(*merged_count);
    return true;
}

static int ClampIntValue(int value, int min_value, int max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

struct TaskWhiteReferenceStats
{
    bool valid = false;
    int sample_count = 0;
    float mean_b = 0.0f;
    float mean_g = 0.0f;
    float mean_r = 0.0f;
    float mean_luma = 0.0f;
};

struct TaskTrackWhiteThresholds
{
    int max_saturation = kWhiteMaxSaturation;
    int min_value = kWhiteMinValue;
    int min_rgb = kWhiteMinRgb;
    int max_channel_diff = kWhiteMaxChannelDiff;
};

struct TaskStrictRedThresholds
{
    int red_score = kTaskRedScoreThreshold;
    int min_r = kTaskRedMinR;
    int dom = kTaskRedDomThreshold;
};

static bool IsLooseTaskWhiteReferenceSeed(const cv::Vec3b& bgr, const cv::Vec3b& hsv)
{
    const int max_rgb = std::max(std::max(static_cast<int>(bgr[0]), static_cast<int>(bgr[1])),
                                 static_cast<int>(bgr[2]));
    const int min_rgb = std::min(std::min(static_cast<int>(bgr[0]), static_cast<int>(bgr[1])),
                                 static_cast<int>(bgr[2]));
    return hsv[1] <= static_cast<unsigned char>(BW_RECOG_WHITE_REF_SEED_MAX_SATURATION) &&
           hsv[2] >= static_cast<unsigned char>(BW_RECOG_WHITE_REF_SEED_MIN_VALUE) &&
           min_rgb >= BW_RECOG_WHITE_REF_SEED_MIN_RGB &&
           (max_rgb - min_rgb) <= BW_RECOG_WHITE_REF_SEED_MAX_CHANNEL_DIFF;
}

static bool ComputeTaskWhiteReferenceStatsRaw(const cv::Mat& frame_bgr,
                                              TaskWhiteReferenceStats* out_stats)
{
    if (out_stats == nullptr)
    {
        return false;
    }
    *out_stats = TaskWhiteReferenceStats();
    if (frame_bgr.empty())
    {
        return false;
    }

    const int row_y = std::max(0, std::min(kWhiteReferenceRowY, frame_bgr.rows - 1));
    const int half_height = std::max(0, BW_RECOG_WHITE_REF_STATS_HALF_HEIGHT);
    const int y0 = std::max(0, row_y - half_height);
    const int y1 = std::min(frame_bgr.rows - 1, row_y + half_height);
    if (y1 < y0)
    {
        return false;
    }

    const cv::Mat roi_bgr = frame_bgr.rowRange(y0, y1 + 1);
    cv::Mat roi_hsv;
    cv::cvtColor(roi_bgr, roi_hsv, cv::COLOR_BGR2HSV);

    double sum_b = 0.0;
    double sum_g = 0.0;
    double sum_r = 0.0;
    double sum_luma = 0.0;
    int sample_count = 0;

    for (int y = 0; y < roi_bgr.rows; ++y)
    {
        const cv::Vec3b* bgr_row = roi_bgr.ptr<cv::Vec3b>(y);
        const cv::Vec3b* hsv_row = roi_hsv.ptr<cv::Vec3b>(y);
        for (int x = 0; x < roi_bgr.cols; ++x)
        {
            if (!IsLooseTaskWhiteReferenceSeed(bgr_row[x], hsv_row[x]))
            {
                continue;
            }

            const float b = static_cast<float>(bgr_row[x][0]);
            const float g = static_cast<float>(bgr_row[x][1]);
            const float r = static_cast<float>(bgr_row[x][2]);
            sum_b += b;
            sum_g += g;
            sum_r += r;
            sum_luma += 0.114 * b + 0.587 * g + 0.299 * r;
            ++sample_count;
        }
    }

    if (sample_count <= 0)
    {
        return false;
    }

    out_stats->valid = true;
    out_stats->sample_count = sample_count;
    out_stats->mean_b = static_cast<float>(sum_b / sample_count);
    out_stats->mean_g = static_cast<float>(sum_g / sample_count);
    out_stats->mean_r = static_cast<float>(sum_r / sample_count);
    out_stats->mean_luma = static_cast<float>(sum_luma / sample_count);
    return true;
}

static bool GetTaskWhiteReferenceStats(const cv::Mat& frame_bgr,
                                       TaskWhiteReferenceStats* out_stats)
{
    if (out_stats == nullptr)
    {
        return false;
    }
    *out_stats = TaskWhiteReferenceStats();

    RecognitionWhiteReferenceStats shared_stats;
    if (recognition_white_reference::GetCurrentStats(&shared_stats) && shared_stats.valid)
    {
        out_stats->valid = true;
        out_stats->sample_count = shared_stats.sample_count;
        out_stats->mean_b = shared_stats.mean_b;
        out_stats->mean_g = shared_stats.mean_g;
        out_stats->mean_r = shared_stats.mean_r;
        out_stats->mean_luma = shared_stats.mean_luma;
        return true;
    }

    return ComputeTaskWhiteReferenceStatsRaw(frame_bgr, out_stats);
}

static TaskTrackWhiteThresholds BuildTaskTrackWhiteThresholds(const TaskWhiteReferenceStats& stats)
{
    TaskTrackWhiteThresholds thresholds;
    if (!stats.valid)
    {
        return thresholds;
    }

    const float reference_luma = std::max(stats.mean_luma, 1.0f);
    thresholds.min_value = ClampIntValue(
        static_cast<int>(std::lround(reference_luma * BW_RECOG_TRACK_WHITE_MIN_LUMA_RATIO)),
        80,
        255);
    thresholds.min_rgb = ClampIntValue(
        static_cast<int>(std::lround(reference_luma * BW_RECOG_TRACK_WHITE_MIN_RGB_RATIO)),
        70,
        255);
    thresholds.max_channel_diff = ClampIntValue(
        static_cast<int>(std::lround(reference_luma * BW_RECOG_TRACK_WHITE_MAX_CHANNEL_DIFF_RATIO)),
        12,
        96);
    return thresholds;
}

static TaskStrictRedThresholds BuildTaskStrictRedThresholds(const TaskWhiteReferenceStats& stats)
{
    TaskStrictRedThresholds thresholds;
    if (!stats.valid)
    {
        return thresholds;
    }

    const float reference_luma = std::max(stats.mean_luma, 1.0f);
    thresholds.red_score = ClampIntValue(
        static_cast<int>(std::lround(reference_luma * BW_RECOG_TASK_RED_SCORE_RELATIVE_RATIO)),
        BW_RECOG_TASK_RED_SCORE_MIN_FLOOR,
        BW_RECOG_TASK_RED_SCORE_MAX_CEIL);
    thresholds.min_r = ClampIntValue(
        static_cast<int>(std::lround(reference_luma * BW_RECOG_TASK_RED_MIN_R_RELATIVE_RATIO)),
        BW_RECOG_TASK_RED_MIN_R_FLOOR,
        BW_RECOG_TASK_RED_MIN_R_MAX_CEIL);
    thresholds.dom = ClampIntValue(
        static_cast<int>(std::lround(reference_luma * BW_RECOG_TASK_RED_DOM_RELATIVE_RATIO)),
        BW_RECOG_TASK_RED_DOM_MIN_FLOOR,
        BW_RECOG_TASK_RED_DOM_MAX_CEIL);
    return thresholds;
}

static bool IsTaskStrictRedPixel(const cv::Vec3b& bgr, const TaskStrictRedThresholds& thresholds)
{
    const cv::Vec3b adjusted_bgr = recognition_white_reference::ApplyGainsToPixel(bgr);
    const int b = static_cast<int>(adjusted_bgr[0]);
    const int g = static_cast<int>(adjusted_bgr[1]);
    const int r = static_cast<int>(adjusted_bgr[2]);
    const int red_score = 2 * r - g - b;
    const int dom = r - std::max(g, b);
    return red_score >= thresholds.red_score &&
           r >= thresholds.min_r &&
           dom >= thresholds.dom;
}

static bool IsTaskPrefilterRedPixel(const cv::Vec3b& bgr,
                                    const TaskStrictRedThresholds& thresholds)
{
    return IsTaskStrictRedPixel(bgr, thresholds);
}

struct TaskPrefilterRedBounds
{
    int min_x = std::numeric_limits<int>::max();
    int max_x = -1;
    int min_y = std::numeric_limits<int>::max();
    int max_y = -1;
    int count = 0;

    void Add(int x, int y)
    {
        ++count;
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }

    bool ToRect(int min_pixel_count, int min_width, int min_height, cv::Rect* out_rect) const
    {
        if (count < min_pixel_count || max_x < min_x || max_y < min_y)
        {
            return false;
        }
        const int width = max_x - min_x + 1;
        const int height = max_y - min_y + 1;
        if (width < min_width || height < min_height)
        {
            return false;
        }
        if (out_rect != nullptr)
        {
            *out_rect = cv::Rect(min_x, min_y, width, height);
        }
        return true;
    }
};

static bool ComputeWhiteEnvelopeXRangeOnReferenceRow(const cv::Mat& frame_bgr,
                                                     const TaskTrackWhiteThresholds& white_thresholds,
                                                     int* out_x_min,
                                                     int* out_x_max,
                                                     int* out_span_count)
{
    if (out_x_min == nullptr || out_x_max == nullptr || frame_bgr.empty())
    {
        return false;
    }
    if (out_span_count != nullptr)
    {
        *out_span_count = 0;
    }

    const int image_height = frame_bgr.rows;
    const int image_width = frame_bgr.cols;
    if (image_height <= 0 || image_width <= 0)
    {
        return false;
    }

    const int row_y = std::max(0, std::min(kWhiteReferenceRowY, image_height - 1));
    cv::Mat row_bgr = frame_bgr.row(row_y).clone();
    recognition_white_reference::ApplyGainsToMat(&row_bgr);
    cv::Mat row_hsv;
    cv::cvtColor(row_bgr, row_hsv, cv::COLOR_BGR2HSV);

    int merged_start = -1;
    int merged_end = -1;
    int merged_count = 0;
    int start = -1;
    for (int x = 0; x < image_width; ++x)
    {
        const cv::Vec3b bgr = row_bgr.at<cv::Vec3b>(0, x);
        const cv::Vec3b hsv = row_hsv.at<cv::Vec3b>(0, x);
        const int max_rgb = std::max(std::max(static_cast<int>(bgr[0]), static_cast<int>(bgr[1])),
                                     static_cast<int>(bgr[2]));
        const int min_rgb = std::min(std::min(static_cast<int>(bgr[0]), static_cast<int>(bgr[1])),
                                     static_cast<int>(bgr[2]));
        const bool is_white =
            hsv[1] <= static_cast<unsigned char>(white_thresholds.max_saturation) &&
            hsv[2] >= static_cast<unsigned char>(white_thresholds.min_value) &&
            min_rgb >= white_thresholds.min_rgb &&
            (max_rgb - min_rgb) <= white_thresholds.max_channel_diff;
        if (is_white && start < 0)
        {
            start = x;
        }
        else if (!is_white && start >= 0)
        {
            MergeRowSpanIntoEnvelope(
                start, x - 1, kWhiteMinSpanWidth, &merged_start, &merged_end, &merged_count);
            start = -1;
        }
    }
    if (start >= 0)
    {
        MergeRowSpanIntoEnvelope(
            start, image_width - 1, kWhiteMinSpanWidth, &merged_start, &merged_end, &merged_count);
    }

    if (merged_start < 0 || merged_end < merged_start || merged_count <= 0)
    {
        return false;
    }

    *out_x_min = merged_start;
    *out_x_max = merged_end;
    if (out_span_count != nullptr)
    {
        *out_span_count = merged_count;
    }
    return true;
}

static bool IsTaskTrackWhitePixelFastSv(const cv::Vec3b& bgr,
                                        const TaskTrackWhiteThresholds& thresholds)
{
    const cv::Vec3b adjusted_bgr = recognition_white_reference::ApplyGainsToPixel(bgr);
    const int max_rgb = std::max(std::max(static_cast<int>(adjusted_bgr[0]), static_cast<int>(adjusted_bgr[1])),
                                 static_cast<int>(adjusted_bgr[2]));
    const int min_rgb = std::min(std::min(static_cast<int>(adjusted_bgr[0]), static_cast<int>(adjusted_bgr[1])),
                                 static_cast<int>(adjusted_bgr[2]));
    const int saturation = (max_rgb <= 0)
        ? 0
        : ((max_rgb - min_rgb) * 255 + max_rgb / 2) / max_rgb;
    return saturation <= thresholds.max_saturation &&
           max_rgb >= thresholds.min_value &&
           min_rgb >= thresholds.min_rgb &&
           (max_rgb - min_rgb) <= thresholds.max_channel_diff;
}

static void PrepareTaskWhiteLazyMaskBuffers(int cache_rows,
                                            int cols,
                                            cv::Mat* values,
                                            cv::Mat* valid)
{
    if (values == nullptr || valid == nullptr)
    {
        return;
    }
    if (cache_rows <= 0 || cols <= 0)
    {
        values->release();
        valid->release();
        return;
    }

    static thread_local cv::Mat values_scratch;
    static thread_local cv::Mat valid_scratch;
    values_scratch.create(cache_rows, cols, CV_8UC1);
    valid_scratch.create(cache_rows, cols, CV_8UC1);
    values_scratch.setTo(0);
    valid_scratch.setTo(0);
    *values = values_scratch;
    *valid = valid_scratch;
}

struct TaskWhiteLazyMaskCache
{
    const cv::Mat& frame_bgr;
    const TaskTrackWhiteThresholds& thresholds;
    const int rows;
    const int cols;
    const int y_min;
    const int y_max;
    const int cache_rows;
    const int half_window;
    cv::Mat values;
    cv::Mat valid;

    TaskWhiteLazyMaskCache(const cv::Mat& frame,
                           const TaskTrackWhiteThresholds& white_thresholds,
                           int trace_top_y,
                           int trace_bottom_y)
        : frame_bgr(frame),
          thresholds(white_thresholds),
          rows(frame.rows),
          cols(frame.cols),
          y_min(std::max(0, std::min(trace_top_y, std::max(0, frame.rows - 1)))),
          y_max(std::max(y_min, std::min(trace_bottom_y, std::max(0, frame.rows - 1)))),
          cache_rows(std::max(0, y_max - y_min + 1)),
          half_window(std::max(0, kTrackLazyMaskHalfWindow)),
          values(),
          valid()
    {
        PrepareTaskWhiteLazyMaskBuffers(cache_rows, frame.cols, &values, &valid);
    }

    bool IsInside(int y, int x) const
    {
        return !frame_bgr.empty() &&
               y >= y_min &&
               y <= y_max &&
               x >= 0 &&
               x < cols;
    }

    void EnsureRange(int y, int x0, int x1)
    {
        if (frame_bgr.empty() || y < y_min || y > y_max || cols <= 0)
        {
            return;
        }

        const int left = std::max(0, std::min(x0, cols - 1));
        const int right = std::max(left, std::min(x1, cols - 1));
        const int row_index = y - y_min;
        const cv::Vec3b* bgr_row = frame_bgr.ptr<cv::Vec3b>(y);
        unsigned char* value_row = values.ptr<unsigned char>(row_index);
        unsigned char* valid_row = valid.ptr<unsigned char>(row_index);
        for (int x = left; x <= right; ++x)
        {
            if (valid_row[x] != 0)
            {
                continue;
            }
            value_row[x] = IsTaskTrackWhitePixelFastSv(bgr_row[x], thresholds)
                ? kTrackWhitePixel
                : kTrackNonWhitePixel;
            valid_row[x] = 1;
        }
    }

    unsigned char At(int y, int x)
    {
        if (!IsInside(y, x))
        {
            return kTrackNonWhitePixel;
        }
        EnsureRange(y, x - half_window, x + half_window);
        return values.ptr<unsigned char>(y - y_min)[x];
    }

    const unsigned char* FullRow(int y)
    {
        if (frame_bgr.empty() || y < y_min || y > y_max)
        {
            return nullptr;
        }
        EnsureRange(y, 0, cols - 1);
        return values.ptr<unsigned char>(y - y_min);
    }
};

static bool IsSimpleLeftBoundaryPixel(const unsigned char* row, int cols, int x)
{
    return row != nullptr &&
           x > 0 &&
           x < cols &&
           row[x] == kTrackWhitePixel &&
           row[x - 1] == kTrackNonWhitePixel;
}

static bool IsSimpleRightBoundaryPixel(const unsigned char* row, int cols, int x)
{
    return row != nullptr &&
           x >= 0 &&
           x < cols - 1 &&
           row[x] == kTrackWhitePixel &&
           row[x + 1] == kTrackNonWhitePixel;
}

static bool FindLeftTrackBoundarySeedFromCenter(const unsigned char* row,
                                                int cols,
                                                int center_x,
                                                int* out_x)
{
    if (out_x == nullptr || row == nullptr || cols <= 2)
    {
        return false;
    }

    const int x_start = std::max(1, std::min(center_x, cols - 2));
    for (int x = x_start; x >= 1; --x)
    {
        if (IsSimpleLeftBoundaryPixel(row, cols, x))
        {
            *out_x = x;
            return true;
        }
    }
    return false;
}

static bool FindRightTrackBoundarySeedFromCenter(const unsigned char* row,
                                                 int cols,
                                                 int center_x,
                                                 int* out_x)
{
    if (out_x == nullptr || row == nullptr || cols <= 2)
    {
        return false;
    }

    const int x_start = std::max(1, std::min(center_x, cols - 2));
    for (int x = x_start; x < cols - 1; ++x)
    {
        if (IsSimpleRightBoundaryPixel(row, cols, x))
        {
            *out_x = x;
            return true;
        }
    }
    return false;
}

static void RemoveOverlappingTrackBoundaryPoints(std::vector<cv::Point>* left_points,
                                                 std::vector<cv::Point>* right_points)
{
    if (left_points == nullptr || right_points == nullptr || left_points->empty() || right_points->empty())
    {
        return;
    }

    std::vector<cv::Point> filtered_right;
    filtered_right.reserve(right_points->size());
    for (size_t i = 0; i < right_points->size(); ++i)
    {
        if (std::find(left_points->begin(), left_points->end(), (*right_points)[i]) == left_points->end())
        {
            filtered_right.push_back((*right_points)[i]);
        }
    }
    right_points->swap(filtered_right);
}

static std::vector<cv::Point> BuildTrackBoundaryDisplayPoints(int seed_x,
                                                              int seed_y,
                                                              const std::vector<cv::Point>& trace_points)
{
    std::vector<cv::Point> points;
    if (seed_x < 0 || seed_y < 0)
    {
        return points;
    }

    points.reserve(trace_points.size() + 1);
    points.push_back(cv::Point(seed_x, seed_y));
    points.insert(points.end(), trace_points.begin(), trace_points.end());
    return points;
}

static std::vector<cv::Point> BuildTrackRegionPolygon(const TaskTrackBoundaryState& state)
{
    if (!state.region_polygon.empty())
    {
        return state.region_polygon;
    }

    std::vector<cv::Point> polygon;
    if (!state.valid || state.seed_left_x < 0 || state.seed_right_x < 0)
    {
        return polygon;
    }

    const cv::Point bottom_left(state.seed_left_x, state.seed_y);
    const cv::Point bottom_right(state.seed_right_x, state.seed_y);
    const cv::Point top_right =
        state.right_points.empty() ? bottom_right : state.right_points.back();

    polygon.reserve(state.left_points.size() + state.right_points.size() + 4);
    polygon.push_back(bottom_left);
    polygon.insert(polygon.end(), state.left_points.begin(), state.left_points.end());
    if (polygon.empty() || polygon.back() != top_right)
    {
        polygon.push_back(top_right);
    }
    for (std::vector<cv::Point>::const_reverse_iterator it = state.right_points.rbegin();
         it != state.right_points.rend();
         ++it)
    {
        if (polygon.empty() || polygon.back() != *it)
        {
            polygon.push_back(*it);
        }
    }
    if (polygon.empty() || polygon.back() != bottom_right)
    {
        polygon.push_back(bottom_right);
    }
    return polygon;
}

static bool RasterizeTrackRegionPolygon(const std::vector<cv::Point>& polygon,
                                        int image_width,
                                        int image_height,
                                        std::vector<int>* out_left_x_by_row,
                                        std::vector<int>* out_right_x_by_row)
{
    if (out_left_x_by_row == nullptr || out_right_x_by_row == nullptr ||
        polygon.size() < 4 || image_width <= 0 || image_height <= 0)
    {
        return false;
    }

    out_left_x_by_row->assign(image_height, -1);
    out_right_x_by_row->assign(image_height, -1);

    cv::Rect polygon_rect;
    if (!ClampRectToImage(
            cv::boundingRect(polygon), image_width, image_height, &polygon_rect))
    {
        return false;
    }

    std::vector<cv::Point> local_polygon;
    local_polygon.reserve(polygon.size());
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        local_polygon.push_back(polygon[i] - polygon_rect.tl());
    }

    cv::Mat local_mask = cv::Mat::zeros(polygon_rect.height, polygon_rect.width, CV_8UC1);
    std::vector<std::vector<cv::Point>> polygons(1, local_polygon);
    cv::fillPoly(local_mask, polygons, cv::Scalar(255));

    bool has_region_row = false;
    for (int local_y = 0; local_y < local_mask.rows; ++local_y)
    {
        const unsigned char* row = local_mask.ptr<unsigned char>(local_y);
        int left_x = -1;
        int right_x = -1;
        for (int local_x = 0; local_x < local_mask.cols; ++local_x)
        {
            if (row[local_x] == 0)
            {
                continue;
            }
            if (left_x < 0)
            {
                left_x = local_x + polygon_rect.x;
            }
            right_x = local_x + polygon_rect.x;
        }
        if (left_x < 0 || right_x < left_x)
        {
            continue;
        }

        const int image_y = local_y + polygon_rect.y;
        (*out_left_x_by_row)[image_y] = left_x;
        (*out_right_x_by_row)[image_y] = right_x;
        has_region_row = true;
    }
    return has_region_row;
}

static void TraceTaskWhiteBoundaryLeftMaze(TaskWhiteLazyMaskCache* white_cache,
                                           int start_y,
                                           int start_x,
                                           std::vector<cv::Point>* out_points)
{
    if (out_points == nullptr)
    {
        return;
    }
    out_points->clear();
    if (white_cache == nullptr || white_cache->frame_bgr.empty())
    {
        return;
    }

    int h = start_y;
    int w = start_x;
    int step = 0;
    int dir = 0;
    int turn = 0;
    while (step < kTrackMazeMaxSteps && turn < 4)
    {
        if (!(w >= 0 && w < white_cache->cols && h > 0 && h < white_cache->rows - 1))
        {
            break;
        }

        const int fh = h + kTrackDirectionFront[dir][1];
        const int fw = w + kTrackDirectionFront[dir][0];
        const int flh = h + kTrackDirectionFrontLeft[dir][1];
        const int flw = w + kTrackDirectionFrontLeft[dir][0];
        const unsigned char front = white_cache->At(fh, fw);
        const unsigned char front_left = white_cache->At(flh, flw);

        if (front == kTrackNonWhitePixel)
        {
            dir = (dir + 1) & 3;
            ++turn;
            continue;
        }

        if (front_left == kTrackNonWhitePixel)
        {
            w += kTrackDirectionFront[dir][0];
            h += kTrackDirectionFront[dir][1];
        }
        else
        {
            w += kTrackDirectionFrontLeft[dir][0];
            h += kTrackDirectionFrontLeft[dir][1];
            dir = (dir + 3) & 3;
        }

        ++step;
        turn = 0;
        out_points->push_back(cv::Point(w, h));

        if (h <= kTaskTrackTraceTopY)
        {
            break;
        }
    }
}

static void TraceTaskWhiteBoundaryRightMaze(TaskWhiteLazyMaskCache* white_cache,
                                            int start_y,
                                            int start_x,
                                            std::vector<cv::Point>* out_points)
{
    if (out_points == nullptr)
    {
        return;
    }
    out_points->clear();
    if (white_cache == nullptr || white_cache->frame_bgr.empty())
    {
        return;
    }

    int h = start_y;
    int w = start_x;
    int step = 0;
    int dir = 0;
    int turn = 0;
    while (step < kTrackMazeMaxSteps && turn < 4)
    {
        if (!(w >= 0 && w < white_cache->cols && h > 0 && h < white_cache->rows - 1))
        {
            break;
        }

        const int fh = h + kTrackDirectionFront[dir][1];
        const int fw = w + kTrackDirectionFront[dir][0];
        const int frh = h + kTrackDirectionFrontRight[dir][1];
        const int frw = w + kTrackDirectionFrontRight[dir][0];
        const unsigned char front = white_cache->At(fh, fw);
        const unsigned char front_right = white_cache->At(frh, frw);

        if (front == kTrackNonWhitePixel)
        {
            dir = (dir + 3) & 3;
            ++turn;
            continue;
        }

        if (front_right == kTrackNonWhitePixel)
        {
            w += kTrackDirectionFront[dir][0];
            h += kTrackDirectionFront[dir][1];
        }
        else
        {
            w += kTrackDirectionFrontRight[dir][0];
            h += kTrackDirectionFrontRight[dir][1];
            dir = (dir + 1) & 3;
        }

        ++step;
        turn = 0;
        out_points->push_back(cv::Point(w, h));

        if (h <= kTaskTrackTraceTopY)
        {
            break;
        }
    }
}

static bool FindTrackRegionBoundsAtRow(const TaskTrackBoundaryState& state,
                                       int row_y,
                                       int* out_left_x,
                                       int* out_right_x,
                                       int* out_row_y)
{
    if (out_left_x == nullptr || out_right_x == nullptr || out_row_y == nullptr ||
        state.region_left_x_by_row.empty() || state.region_right_x_by_row.empty())
    {
        return false;
    }

    const int rows = std::min(static_cast<int>(state.region_left_x_by_row.size()),
                              static_cast<int>(state.region_right_x_by_row.size()));
    if (rows <= 0)
    {
        return false;
    }

    const int y = std::max(0, std::min(row_y, rows - 1));
    if (state.region_left_x_by_row[y] >= 0 &&
        state.region_right_x_by_row[y] >= 0 &&
        state.region_left_x_by_row[y] <= state.region_right_x_by_row[y])
    {
        *out_left_x = state.region_left_x_by_row[y];
        *out_right_x = state.region_right_x_by_row[y];
        *out_row_y = y;
        return true;
    }

    return false;
}

static bool BuildTaskTrackBoundaryState(const cv::Mat& frame_bgr,
                                        const TaskTrackWhiteThresholds& white_thresholds,
                                        TaskTrackBoundaryState* out_state)
{
    if (out_state == nullptr)
    {
        return false;
    }
    *out_state = TaskTrackBoundaryState();
    if (frame_bgr.empty())
    {
        return false;
    }

    const int rows = frame_bgr.rows;
    const int cols = frame_bgr.cols;
    const int top_y = std::max(0, std::min(kTaskTrackTraceTopY, rows - 1));
    const int bottom_y = std::max(top_y, std::min(kWhiteReferenceRowY, rows - 1));
    if (bottom_y <= top_y)
    {
        return false;
    }

    int white_x_min = 0;
    int white_x_max = 0;
    if (!ComputeWhiteEnvelopeXRangeOnReferenceRow(
            frame_bgr, white_thresholds, &white_x_min, &white_x_max, nullptr))
    {
        return false;
    }

    TaskWhiteLazyMaskCache white_cache(frame_bgr, white_thresholds, top_y, bottom_y);
    int left_seed_x = -1;
    int right_seed_x = -1;
    const int seed_center_x = (white_x_min + white_x_max) / 2;
    const unsigned char* seed_row = white_cache.FullRow(bottom_y);
    FindLeftTrackBoundarySeedFromCenter(seed_row, cols, seed_center_x, &left_seed_x);
    FindRightTrackBoundarySeedFromCenter(seed_row, cols, seed_center_x, &right_seed_x);

    if (left_seed_x < 0 && right_seed_x < 0)
    {
        return false;
    }

    out_state->top_y = top_y;
    out_state->bottom_y = bottom_y;
    out_state->seed_y = bottom_y;
    out_state->seed_center_x = seed_center_x;
    out_state->seed_left_x = left_seed_x;
    out_state->seed_right_x = right_seed_x;
    if (left_seed_x >= 0)
    {
        TraceTaskWhiteBoundaryLeftMaze(&white_cache, bottom_y, left_seed_x, &out_state->left_points);
    }
    if (right_seed_x >= 0)
    {
        TraceTaskWhiteBoundaryRightMaze(&white_cache, bottom_y, right_seed_x, &out_state->right_points);
    }
    RemoveOverlappingTrackBoundaryPoints(&out_state->left_points, &out_state->right_points);

    int envelope_x_min = cols - 1;
    int envelope_x_max = 0;
    bool has_any_envelope_point = false;
    if (left_seed_x >= 0)
    {
        envelope_x_min = std::min(envelope_x_min, left_seed_x);
        envelope_x_max = std::max(envelope_x_max, left_seed_x);
        has_any_envelope_point = true;
    }
    if (right_seed_x >= 0)
    {
        envelope_x_min = std::min(envelope_x_min, right_seed_x);
        envelope_x_max = std::max(envelope_x_max, right_seed_x);
        has_any_envelope_point = true;
    }
    for (size_t i = 0; i < out_state->left_points.size(); ++i)
    {
        envelope_x_min = std::min(envelope_x_min, out_state->left_points[i].x);
        envelope_x_max = std::max(envelope_x_max, out_state->left_points[i].x);
        has_any_envelope_point = true;
    }
    for (size_t i = 0; i < out_state->right_points.size(); ++i)
    {
        envelope_x_min = std::min(envelope_x_min, out_state->right_points[i].x);
        envelope_x_max = std::max(envelope_x_max, out_state->right_points[i].x);
        has_any_envelope_point = true;
    }

    if (!has_any_envelope_point)
    {
        return false;
    }

    out_state->envelope_x_min = envelope_x_min;
    out_state->envelope_x_max = envelope_x_max;
    out_state->valid =
        left_seed_x >= 0 &&
        right_seed_x >= 0 &&
        left_seed_x < right_seed_x &&
        !out_state->left_points.empty() &&
        !out_state->right_points.empty();
    if (!out_state->valid)
    {
        return false;
    }

    out_state->region_polygon = BuildTrackRegionPolygon(*out_state);
    if (!RasterizeTrackRegionPolygon(
            out_state->region_polygon,
            cols,
            rows,
            &out_state->region_left_x_by_row,
            &out_state->region_right_x_by_row))
    {
        out_state->valid = false;
        return false;
    }
    return true;
}

static cv::Rect BuildTaskTrackSearchRect(const TaskTrackBoundaryState& state,
                                         int image_width,
                                         int image_height,
                                         int y_min,
                                         int y_max)
{
    if (!state.valid || image_width <= 0 || image_height <= 0)
    {
        return cv::Rect();
    }

    const int rect_y0 = std::max(0, std::min(y_min, image_height - 1));
    const int rect_y1 = std::max(rect_y0 + 1, std::min(y_max, image_height));
    int track_x_min = image_width - 1;
    int track_x_max = 0;
    bool has_bounds_in_range = false;
    for (int y = rect_y0; y < rect_y1; ++y)
    {
        int left_x = -1;
        int right_x = -1;
        int boundary_y = -1;
        if (!FindTrackRegionBoundsAtRow(state, y, &left_x, &right_x, &boundary_y))
        {
            continue;
        }
        track_x_min = std::min(track_x_min, left_x);
        track_x_max = std::max(track_x_max, right_x);
        has_bounds_in_range = true;
    }
    if (!has_bounds_in_range)
    {
        return cv::Rect();
    }
    const int x0 = std::max(0, track_x_min - kTrackBrickOuterExpandPixels);
    const int x1 = std::min(image_width - 1, track_x_max + kTrackBrickOuterExpandPixels);
    cv::Rect rect(
        x0,
        rect_y0,
        std::max(1, x1 - x0 + 1),
        std::max(1, rect_y1 - rect_y0));
    cv::Rect clamped;
    if (!ClampRectToImage(rect, image_width, image_height, &clamped))
    {
        return cv::Rect();
    }
    return clamped;
}

static TaskTrackClassification ClassifyTaskCandidateByTrackBoundary(
    const TaskTrackBoundaryState& state,
    const cv::Rect& candidate_box,
    int image_width)
{
    TaskTrackClassification result;
    if (!state.valid || candidate_box.width <= 0 || candidate_box.height <= 0)
    {
        return result;
    }

    const int classify_x = candidate_box.x + candidate_box.width / 2;
    const int classify_y = candidate_box.y + candidate_box.height - 1;
    result.classify_point = cv::Point(classify_x, classify_y);

    if (state.region_polygon.size() < 4)
    {
        return result;
    }

    int left_x = -1;
    int right_x = -1;
    int boundary_y = -1;
    if (!FindTrackRegionBoundsAtRow(state, classify_y, &left_x, &right_x, &boundary_y))
    {
        return result;
    }

    result.left_boundary_x = left_x;
    result.right_boundary_x = right_x;
    result.boundary_row_y = boundary_y;
    const TaskTrackRowSearchBands bands =
        BuildTaskTrackRowSearchBands(left_x, right_x, image_width, classify_y);
    if (bands.has_marker &&
        classify_x >= bands.marker_x0 &&
        classify_x <= bands.marker_x1)
    {
        result.type = TaskTrackCandidateType::MARKER;
        return result;
    }

    const int candidate_left = candidate_box.x;
    const int candidate_right = candidate_box.x + candidate_box.width - 1;
    const bool touches_left_outer_band =
        bands.has_left_brick &&
        candidate_left <= bands.left_brick_x1 &&
        candidate_right >= bands.left_brick_x0;
    const bool touches_right_outer_band =
        bands.has_right_brick &&
        candidate_right >= bands.right_brick_x0 &&
        candidate_left <= bands.right_brick_x1;
    if (touches_left_outer_band || touches_right_outer_band)
    {
        result.type = TaskTrackCandidateType::ROADBLOCK;
    }
    return result;
}

static std::vector<cv::Point2f> OrderQuadPointsCanonical(const std::vector<cv::Point2f>& points)
{
    if (points.size() != 4)
    {
        return points;
    }

    std::vector<cv::Point2f> ordered = points;
    cv::Point2f center(0.0f, 0.0f);
    for (size_t i = 0; i < ordered.size(); ++i)
    {
        center += ordered[i];
    }
    center *= 0.25f;
    std::sort(
        ordered.begin(),
        ordered.end(),
        [&center](const cv::Point2f& a, const cv::Point2f& b) {
            const float angle_a = std::atan2(a.y - center.y, a.x - center.x);
            const float angle_b = std::atan2(b.y - center.y, b.x - center.x);
            return angle_a < angle_b;
        });
    return ordered;
}

static bool ClampRectToImage(const cv::Rect& rect,
                             int image_width,
                             int image_height,
                             cv::Rect* out_rect)
{
    if (image_width <= 0 || image_height <= 0 || out_rect == nullptr)
    {
        return false;
    }

    const int x1 = std::max(0, std::min(rect.x, image_width - 1));
    const int y1 = std::max(0, std::min(rect.y, image_height - 1));
    const int x2 = std::max(x1 + 1, std::min(rect.x + rect.width, image_width));
    const int y2 = std::max(y1 + 1, std::min(rect.y + rect.height, image_height));
    if (x2 <= x1 || y2 <= y1)
    {
        return false;
    }

    *out_rect = cv::Rect(x1, y1, x2 - x1, y2 - y1);
    return true;
}

static float SegmentLength(const cv::Point2f& p1, const cv::Point2f& p2)
{
    const cv::Point2f delta = p2 - p1;
    return std::sqrt(delta.x * delta.x + delta.y * delta.y);
}

static float MapValueBilinear(const float table[kIpmFrameHeight][kIpmFrameWidth], float x, float y)
{
    x = std::max(0.0f, std::min(x, static_cast<float>(kIpmFrameWidth - 1)));
    y = std::max(0.0f, std::min(y, static_cast<float>(kIpmFrameHeight - 1)));

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, kIpmFrameWidth - 1);
    const int y1 = std::min(y0 + 1, kIpmFrameHeight - 1);
    const float tx = x - static_cast<float>(x0);
    const float ty = y - static_cast<float>(y0);

    const float top =
        table[y0][x0] * (1.0f - tx) +
        table[y0][x1] * tx;
    const float bottom =
        table[y1][x0] * (1.0f - tx) +
        table[y1][x1] * tx;
    return top * (1.0f - ty) + bottom * ty;
}

static bool RawPointToFinalPoint(float x,
                                 float y,
                                 int image_width,
                                 int image_height,
                                 cv::Point2f* out_point)
{
    if (out_point == nullptr ||
        image_width != kIpmFrameWidth ||
        image_height != kIpmFrameHeight)
    {
        return false;
    }

    out_point->x = MapValueBilinear(UndistInverseMapW, x, y);
    out_point->y = MapValueBilinear(UndistInverseMapH, x, y);
    return IsFinitePoint(*out_point);
}

static bool RawQuadToFinalQuad(const std::vector<cv::Point2f>& quad,
                               int image_width,
                               int image_height,
                               std::vector<cv::Point2f>* out_quad)
{
    if (out_quad == nullptr || quad.size() != 4)
    {
        return false;
    }

    out_quad->clear();
    out_quad->reserve(4);
    for (size_t i = 0; i < quad.size(); ++i)
    {
        cv::Point2f mapped;
        if (!RawPointToFinalPoint(quad[i].x, quad[i].y, image_width, image_height, &mapped))
        {
            out_quad->clear();
            return false;
        }
        out_quad->push_back(mapped);
    }
    return true;
}

static bool FinalToRaw(float xf,
                       float yf,
                       int image_width,
                       int image_height,
                       cv::Point2f* out_point)
{
    if (out_point == nullptr || image_width <= 0 || image_height <= 0)
    {
        return false;
    }

    if (!std::isfinite(xf) || !std::isfinite(yf))
    {
        return false;
    }

    const double final_scale_x =
        kBaseCalibrationFrameWidth / std::max(1.0, static_cast<double>(kIpmFrameWidth));
    const double final_scale_y =
        kBaseCalibrationFrameHeight / std::max(1.0, static_cast<double>(kIpmFrameHeight));
    const cv::Vec3d vec =
        kFinalToUndist * cv::Vec3d(xf * final_scale_x, yf * final_scale_y, 1.0);
    const double z = vec[2];
    if (!std::isfinite(z) || std::fabs(z) < 1e-12)
    {
        return false;
    }

    const double xu = vec[0] / z;
    const double yu = vec[1] / z;
    const double y = (yu - kCy) / kFy;
    const double x = (xu - kCx - kSkew * y) / kFx;

    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double radial = 1.0 + kK1 * r2 + kK2 * r4 + kK3 * r6;

    const double xd = x * radial + 2.0 * kP1 * x * y + kP2 * (r2 + 2.0 * x * x);
    const double yd = y * radial + kP1 * (r2 + 2.0 * y * y) + 2.0 * kP2 * x * y;

    const double xr = kFx * xd + kSkew * yd + kCx;
    const double yr = kFy * yd + kCy;
    if (!std::isfinite(xr) || !std::isfinite(yr))
    {
        return false;
    }

    const double raw_scale_x =
        std::max(1.0, static_cast<double>(image_width)) / kBaseCalibrationFrameWidth;
    const double raw_scale_y =
        std::max(1.0, static_cast<double>(image_height)) / kBaseCalibrationFrameHeight;
    out_point->x = static_cast<float>(xr * raw_scale_x);
    out_point->y = static_cast<float>(yr * raw_scale_y);
    return true;
}

static double QuadArea(const std::vector<cv::Point2f>& quad)
{
    if (quad.size() != 4)
    {
        return 0.0;
    }
    return std::fabs(cv::contourArea(quad));
}

static bool QuadIsConvex(const std::vector<cv::Point2f>& quad)
{
    if (quad.size() != 4)
    {
        return false;
    }
    std::vector<cv::Point> contour;
    contour.reserve(quad.size());
    for (size_t i = 0; i < quad.size(); ++i)
    {
        contour.push_back(cv::Point(
            static_cast<int>(std::lround(quad[i].x)),
            static_cast<int>(std::lround(quad[i].y))));
    }
    return cv::isContourConvex(contour);
}

static double QuadHeightRatio(const std::vector<cv::Point2f>& quad)
{
    if (quad.size() != 4)
    {
        return 0.0;
    }
    const double top_width = SegmentLength(quad[0], quad[1]);
    const double bottom_width = SegmentLength(quad[3], quad[2]);
    const double left_height = SegmentLength(quad[0], quad[3]);
    const double right_height = SegmentLength(quad[1], quad[2]);
    const double width_sum = top_width + bottom_width;
    if (width_sum <= 1e-6)
    {
        return 0.0;
    }
    return (left_height + right_height) / width_sum;
}

static bool QuadInsideImage(const std::vector<cv::Point2f>& quad, int image_width, int image_height)
{
    if (quad.size() != 4)
    {
        return false;
    }

    for (size_t i = 0; i < quad.size(); ++i)
    {
        const cv::Point2f& p = quad[i];
        if (!IsFinitePoint(p) ||
            p.x < 0.0f ||
            p.y < 0.0f ||
            p.x >= static_cast<float>(image_width) ||
            p.y >= static_cast<float>(image_height))
        {
            return false;
        }
    }
    return true;
}

static std::vector<cv::Point2f> RotatedBoxPointsFromContour(const std::vector<cv::Point>& contour)
{
    const cv::RotatedRect rect = cv::minAreaRect(contour);
    cv::Point2f points[4];
    rect.points(points);
    return OrderQuadPointsCanonical(std::vector<cv::Point2f>(points, points + 4));
}

static bool SelectUpperLongEdge(const std::vector<cv::Point2f>& quad, DirectEdgeInfo* out_info)
{
    if (out_info == nullptr || quad.size() != 4)
    {
        return false;
    }

    struct Edge
    {
        int index;
        cv::Point2f p1;
        cv::Point2f p2;
        cv::Point2f midpoint;
        float length;
    };

    cv::Point2f center(0.0f, 0.0f);
    for (size_t i = 0; i < quad.size(); ++i)
    {
        center += quad[i];
    }
    center *= 0.25f;

    std::vector<Edge> edges;
    edges.reserve(4);
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point2f& p1 = quad[i];
        const cv::Point2f& p2 = quad[(i + 1) % 4];
        edges.push_back({i, p1, p2, (p1 + p2) * 0.5f, SegmentLength(p1, p2)});
    }

    const float pair0_avg = 0.5f * (edges[0].length + edges[2].length);
    const float pair1_avg = 0.5f * (edges[1].length + edges[3].length);
    Edge chosen = (pair0_avg >= pair1_avg)
        ? ((edges[0].midpoint.y <= edges[2].midpoint.y) ? edges[0] : edges[2])
        : ((edges[1].midpoint.y <= edges[3].midpoint.y) ? edges[1] : edges[3]);

    cv::Point2f left_point = chosen.p1;
    cv::Point2f right_point = chosen.p2;
    int left_index = chosen.index;
    int right_index = (chosen.index + 1) % 4;
    if (left_point.x > right_point.x)
    {
        std::swap(left_point, right_point);
        std::swap(left_index, right_index);
    }

    out_info->center = center;
    out_info->midpoint = chosen.midpoint;
    out_info->edge_vector = right_point - left_point;
    out_info->left_point = left_point;
    out_info->right_point = right_point;
    out_info->left_index = left_index;
    out_info->right_index = right_index;
    out_info->edge_length = chosen.length;
    return true;
}

static bool SelectUpperRawLongEdgeInFinal(const std::vector<cv::Point2f>& raw_quad,
                                          const std::vector<cv::Point2f>& final_quad,
                                          FinalEdgeInfo* out_info)
{
    if (out_info == nullptr || raw_quad.size() != 4 || final_quad.size() != 4)
    {
        return false;
    }

    struct Edge
    {
        int index;
        cv::Point2f raw_p1;
        cv::Point2f raw_p2;
        cv::Point2f final_p1;
        cv::Point2f final_p2;
        cv::Point2f midpoint_final;
        float raw_length;
        float final_length;
    };

    cv::Point2f center_final(0.0f, 0.0f);
    for (size_t i = 0; i < final_quad.size(); ++i)
    {
        center_final += final_quad[i];
    }
    center_final *= 0.25f;

    std::vector<Edge> edges;
    edges.reserve(4);
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point2f& raw_p1 = raw_quad[i];
        const cv::Point2f& raw_p2 = raw_quad[(i + 1) % 4];
        const cv::Point2f& final_p1 = final_quad[i];
        const cv::Point2f& final_p2 = final_quad[(i + 1) % 4];
        edges.push_back(
            {i,
             raw_p1,
             raw_p2,
             final_p1,
             final_p2,
             (final_p1 + final_p2) * 0.5f,
             SegmentLength(raw_p1, raw_p2),
             SegmentLength(final_p1, final_p2)});
    }

    const float pair0_avg = 0.5f * (edges[0].raw_length + edges[2].raw_length);
    const float pair1_avg = 0.5f * (edges[1].raw_length + edges[3].raw_length);
    Edge chosen = (pair0_avg >= pair1_avg)
        ? ((edges[0].midpoint_final.y <= edges[2].midpoint_final.y) ? edges[0] : edges[2])
        : ((edges[1].midpoint_final.y <= edges[3].midpoint_final.y) ? edges[1] : edges[3]);

    cv::Point2f left_raw = chosen.raw_p1;
    cv::Point2f right_raw = chosen.raw_p2;
    cv::Point2f left_final = chosen.final_p1;
    cv::Point2f right_final = chosen.final_p2;
    int left_index = chosen.index;
    int right_index = (chosen.index + 1) % 4;
    if (left_final.x > right_final.x)
    {
        std::swap(left_raw, right_raw);
        std::swap(left_final, right_final);
        std::swap(left_index, right_index);
    }

    out_info->center_final = center_final;
    out_info->midpoint_final = chosen.midpoint_final;
    out_info->final_edge_vector = right_final - left_final;
    out_info->left_point_raw = left_raw;
    out_info->right_point_raw = right_raw;
    out_info->left_point_final = left_final;
    out_info->right_point_final = right_final;
    out_info->left_index = left_index;
    out_info->right_index = right_index;
    out_info->raw_edge_length = chosen.raw_length;
    out_info->final_edge_length = chosen.final_length;
    return true;
}

static bool UpwardSquareNormal(const cv::Point2f& edge_vector,
                               const cv::Point2f* center_to_edge,
                               cv::Point2f* out_normal)
{
    if (out_normal == nullptr)
    {
        return false;
    }

    const float edge_length = std::sqrt(edge_vector.x * edge_vector.x + edge_vector.y * edge_vector.y);
    if (edge_length <= 1e-6f)
    {
        return false;
    }

    const cv::Point2f edge_unit = edge_vector * (1.0f / edge_length);
    cv::Point2f normal(-edge_unit.y, edge_unit.x);
    if (center_to_edge != nullptr &&
        (normal.x * center_to_edge->x + normal.y * center_to_edge->y) < 0.0f)
    {
        normal = -normal;
    }
    if (normal.y > 0.0f)
    {
        normal = -normal;
    }

    *out_normal = normal;
    return true;
}

static BuildRoiQuadResult BuildDirectSquareRoiQuadFromBlobQuad(const std::vector<cv::Point2f>& blob_quad,
                                                               int image_width,
                                                               int image_height)
{
    BuildRoiQuadResult result;
    result.status = "blob_only";

    DirectEdgeInfo edge_info;
    if (!SelectUpperLongEdge(blob_quad, &edge_info))
    {
        return result;
    }

    if (edge_info.edge_length < kMinRoiEdgeLength)
    {
        return result;
    }

    const cv::Point2f center_to_edge = edge_info.midpoint - edge_info.center;
    cv::Point2f normal;
    if (!UpwardSquareNormal(edge_info.edge_vector, &center_to_edge, &normal))
    {
        return result;
    }

    const cv::Point2f bottom_left = edge_info.left_point;
    const cv::Point2f bottom_right = edge_info.right_point;
    const cv::Point2f top_left = bottom_left + normal * edge_info.edge_length;
    const cv::Point2f top_right = bottom_right + normal * edge_info.edge_length;
    std::vector<cv::Point2f> roi_quad;
    roi_quad.push_back(top_left);
    roi_quad.push_back(top_right);
    roi_quad.push_back(bottom_right);
    roi_quad.push_back(bottom_left);
    if (!QuadInsideImage(roi_quad, image_width, image_height))
    {
        return result;
    }

    result.valid = true;
    result.status = "ok";
    result.roi_quad = roi_quad;
    return result;
}

static BuildRoiQuadResult BuildIpmSquareRoiQuadFromBlobQuad(const std::vector<cv::Point2f>& blob_quad,
                                                            int image_width,
                                                            int image_height)
{
    BuildRoiQuadResult result;
    result.status = "ipm_square_invalid";

    std::vector<cv::Point2f> blob_quad_final;
    if (!RawQuadToFinalQuad(blob_quad, image_width, image_height, &blob_quad_final))
    {
        return result;
    }

    result.blob_quad_final = blob_quad_final;

    FinalEdgeInfo edge_info;
    if (!SelectUpperRawLongEdgeInFinal(blob_quad, blob_quad_final, &edge_info))
    {
        return result;
    }

    if (edge_info.final_edge_length < kMinRoiEdgeLength)
    {
        return result;
    }

    cv::Point2f normal;
    if (!UpwardSquareNormal(edge_info.final_edge_vector, nullptr, &normal))
    {
        return result;
    }

    const cv::Point2f bottom_left_final = edge_info.left_point_final;
    const cv::Point2f bottom_right_final = edge_info.right_point_final;
    const cv::Point2f top_left_final = bottom_left_final + normal * edge_info.final_edge_length;
    const cv::Point2f top_right_final = bottom_right_final + normal * edge_info.final_edge_length;
    std::vector<cv::Point2f> roi_quad_final;
    roi_quad_final.push_back(top_left_final);
    roi_quad_final.push_back(top_right_final);
    roi_quad_final.push_back(bottom_right_final);
    roi_quad_final.push_back(bottom_left_final);
    result.roi_quad_final = roi_quad_final;

    if (!QuadIsConvex(roi_quad_final) ||
        QuadArea(roi_quad_final) < kMinBackprojectedQuadArea)
    {
        return result;
    }

    cv::Point2f top_left_raw;
    cv::Point2f top_right_raw;
    if (!FinalToRaw(top_left_final.x, top_left_final.y, image_width, image_height, &top_left_raw) ||
        !FinalToRaw(top_right_final.x, top_right_final.y, image_width, image_height, &top_right_raw))
    {
        result.status = "ipm_backproject_invalid";
        return result;
    }

    std::vector<cv::Point2f> roi_quad;
    roi_quad.push_back(top_left_raw);
    roi_quad.push_back(top_right_raw);
    roi_quad.push_back(edge_info.right_point_raw);
    roi_quad.push_back(edge_info.left_point_raw);

    result.has_raw_height_ratio = true;
    result.raw_height_ratio = static_cast<float>(QuadHeightRatio(roi_quad));

    if (!QuadInsideImage(roi_quad, image_width, image_height) ||
        !QuadIsConvex(roi_quad) ||
        QuadArea(roi_quad) < kMinBackprojectedQuadArea)
    {
        result.status = "ipm_backproject_invalid";
        result.ipm_reason = "backproject_geometry_invalid";
        return result;
    }

    result.valid = true;
    result.status = "ok";
    result.roi_quad = roi_quad;
    return result;
}

static BuildRoiQuadResult BuildRoiQuadFromBlobQuad(const std::vector<cv::Point2f>& blob_quad,
                                                   int image_width,
                                                   int image_height,
                                                   RoiMethod roi_method)
{
    if (roi_method == RoiMethod::DIRECT_RED_QUAD)
    {
        return BuildDirectSquareRoiQuadFromBlobQuad(blob_quad, image_width, image_height);
    }
    return BuildIpmSquareRoiQuadFromBlobQuad(blob_quad, image_width, image_height);
}

static cv::Mat WarpRoiFromQuad(const cv::Mat& frame_bgr,
                               const std::vector<cv::Point2f>& roi_quad,
                               int output_size)
{
    std::vector<cv::Point2f> destination;
    destination.push_back(cv::Point2f(0.0f, 0.0f));
    destination.push_back(cv::Point2f(static_cast<float>(output_size), 0.0f));
    destination.push_back(cv::Point2f(static_cast<float>(output_size), static_cast<float>(output_size)));
    destination.push_back(cv::Point2f(0.0f, static_cast<float>(output_size)));

    const cv::Mat transform = cv::getPerspectiveTransform(roi_quad, destination);
    cv::Mat roi;
    cv::warpPerspective(frame_bgr, roi, transform, cv::Size(output_size, output_size));
    recognition_white_reference::ApplyGainsToMat(&roi);
    return roi;
}

static void DrawRectIfValid(cv::Mat& image, const cv::Rect& rect, const cv::Scalar& color, int thickness)
{
    if (rect.width > 0 && rect.height > 0)
    {
        cv::rectangle(image, rect, color, thickness);
    }
}

static void DrawQuadIfValid(cv::Mat& image, const std::vector<cv::Point2f>& quad, const cv::Scalar& color, int thickness)
{
    if (quad.size() != 4)
    {
        return;
    }

    std::vector<cv::Point> polygon;
    polygon.reserve(quad.size());
    for (size_t i = 0; i < quad.size(); ++i)
    {
        polygon.push_back(cv::Point(
            static_cast<int>(std::lround(quad[i].x)),
            static_cast<int>(std::lround(quad[i].y))));
    }
    std::vector<std::vector<cv::Point>> polygons(1, polygon);
    cv::polylines(image, polygons, true, color, thickness);
}

} // namespace

namespace {

static cv::Mat BuildTaskMarkerRedMaskLocalInTrackInterior(const cv::Mat& frame_bgr,
                                                          const TaskTrackBoundaryState& state,
                                                          const cv::Rect& rect,
                                                          int y_min,
                                                          int y_max,
                                                          const TaskStrictRedThresholds& red_thresholds,
                                                          cv::Rect* out_image_rect)
{
    if (out_image_rect != nullptr)
    {
        *out_image_rect = cv::Rect();
    }

    cv::Rect clamped;
    if (frame_bgr.empty() || !state.valid ||
        !ClampRectToImage(rect, frame_bgr.cols, frame_bgr.rows, &clamped))
    {
        return cv::Mat();
    }
    if (out_image_rect != nullptr)
    {
        *out_image_rect = clamped;
    }

    cv::Mat local_mask = cv::Mat::zeros(clamped.height, clamped.width, CV_8UC1);
    const int y0 = std::max(clamped.y, std::max(0, std::min(y_min, frame_bgr.rows)));
    const int y1 = std::min(
        clamped.y + clamped.height,
        std::max(y0, std::min(y_max, frame_bgr.rows)));
    for (int y = y0; y < y1; ++y)
    {
        int left_x = -1;
        int right_x = -1;
        int boundary_y = -1;
        if (!FindTrackRegionBoundsAtRow(state, y, &left_x, &right_x, &boundary_y))
        {
            continue;
        }

        const TaskTrackRowSearchBands bands =
            BuildTaskTrackRowSearchBands(left_x, right_x, frame_bgr.cols, y);
        if (!bands.has_marker)
        {
            continue;
        }

        const int x0 = std::max(clamped.x, bands.marker_x0);
        const int x1 = std::min(clamped.x + clamped.width - 1, bands.marker_x1);
        if (x1 < x0)
        {
            continue;
        }

        const cv::Vec3b* row_ptr = frame_bgr.ptr<cv::Vec3b>(y);
        unsigned char* mask_ptr = local_mask.ptr<unsigned char>(y - clamped.y);
        for (int x = x0; x <= x1; ++x)
        {
            if (IsTaskStrictRedPixel(row_ptr[x], red_thresholds))
            {
                mask_ptr[x - clamped.x] = 255;
            }
        }
    }
    return local_mask;
}

static void TaskTouchedSides(const cv::Mat& mask,
                             const cv::Rect& rect,
                             bool* touch_left,
                             bool* touch_top,
                             bool* touch_right,
                             bool* touch_bottom)
{
    if (touch_left != nullptr) *touch_left = false;
    if (touch_top != nullptr) *touch_top = false;
    if (touch_right != nullptr) *touch_right = false;
    if (touch_bottom != nullptr) *touch_bottom = false;
    if (mask.empty() || rect.width <= 0 || rect.height <= 0)
    {
        return;
    }
    const cv::Mat crop = mask(rect);
    if (touch_left != nullptr) *touch_left = cv::countNonZero(crop.col(0)) > 0;
    if (touch_top != nullptr) *touch_top = cv::countNonZero(crop.row(0)) > 0;
    if (touch_right != nullptr) *touch_right = cv::countNonZero(crop.col(crop.cols - 1)) > 0;
    if (touch_bottom != nullptr) *touch_bottom = cv::countNonZero(crop.row(crop.rows - 1)) > 0;
}

static bool FindTrackBrickRedInOuterBand(const cv::Mat& frame_bgr,
                                         const TaskStrictRedThresholds& red_thresholds,
                                         const TaskTrackBoundaryState& state,
                                         cv::Rect* out_box,
                                         double* out_area,
                                         TaskTrackClassification* out_classification,
                                         int y_min,
                                         int y_max)
{
    if (out_box == nullptr || out_area == nullptr || frame_bgr.empty() || !state.valid)
    {
        return false;
    }

    int min_x = frame_bgr.cols;
    int min_y = frame_bgr.rows;
    int max_x = -1;
    int max_y = -1;
    int area = 0;
    int best_row_y = -1;
    int best_left_x = -1;
    int best_right_x = -1;

    const int y0 = std::max(0, std::min(y_min, frame_bgr.rows));
    const int y1 = std::max(y0, std::min(y_max, frame_bgr.rows));
    for (int y = y0; y < y1; ++y)
    {
        int left_x = -1;
        int right_x = -1;
        int boundary_y = -1;
        if (!FindTrackRegionBoundsAtRow(state, y, &left_x, &right_x, &boundary_y))
        {
            continue;
        }

        bool row_has_red = false;
        const cv::Vec3b* row = frame_bgr.ptr<cv::Vec3b>(y);
        const TaskTrackRowSearchBands bands =
            BuildTaskTrackRowSearchBands(left_x, right_x, frame_bgr.cols, y);
        if (bands.has_left_brick)
        {
            for (int x = bands.left_brick_x0; x <= bands.left_brick_x1; ++x)
            {
                if (!IsTaskStrictRedPixel(row[x], red_thresholds))
                {
                    continue;
                }
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
                ++area;
                row_has_red = true;
            }
        }

        if (bands.has_right_brick)
        {
            for (int x = bands.right_brick_x0; x <= bands.right_brick_x1; ++x)
            {
                if (!IsTaskStrictRedPixel(row[x], red_thresholds))
                {
                    continue;
                }
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
                ++area;
                row_has_red = true;
            }
        }

        if (row_has_red && y >= best_row_y)
        {
            best_row_y = boundary_y;
            best_left_x = left_x;
            best_right_x = right_x;
        }
    }

    if (area <= 0 || max_x < min_x || max_y < min_y)
    {
        return false;
    }

    *out_box = cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
    *out_area = static_cast<double>(area);
    if (out_classification != nullptr)
    {
        out_classification->type = TaskTrackCandidateType::ROADBLOCK;
        out_classification->classify_point =
            cv::Point(out_box->x + out_box->width / 2, out_box->y + out_box->height - 1);
        out_classification->left_boundary_x = best_left_x;
        out_classification->right_boundary_x = best_right_x;
        out_classification->boundary_row_y = best_row_y;
    }
    return true;
}

static cv::Rect ExpandTaskRect(const cv::Rect& rect,
                               int image_width,
                               int image_height,
                               bool touch_left,
                               bool touch_top,
                               bool touch_right,
                               bool touch_bottom)
{
    cv::Rect expanded(
        rect.x - (touch_left ? kTaskEdgeExpandStep : 0),
        rect.y - (touch_top ? kTaskEdgeExpandStep : 0),
        rect.width + (touch_left ? kTaskEdgeExpandStep : 0) + (touch_right ? kTaskEdgeExpandStep : 0),
        rect.height + (touch_top ? kTaskEdgeExpandStep : 0) + (touch_bottom ? kTaskEdgeExpandStep : 0));
    cv::Rect clamped;
    if (!ClampRectToImage(expanded, image_width, image_height, &clamped))
    {
        return rect;
    }
    return clamped;
}

static bool ChooseLowestTaskRedBand(const cv::Mat& mask,
                                    const cv::Rect& rect,
                                    std::vector<cv::Point>* out_contour,
                                    cv::Rect* out_box,
                                    double* out_area)
{
    if (out_contour == nullptr || out_box == nullptr || out_area == nullptr || mask.empty() || rect.width <= 0 || rect.height <= 0)
    {
        return false;
    }
    const cv::Mat crop = mask(rect);
    if (cv::countNonZero(crop) <= 0)
    {
        return false;
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int num_labels = cv::connectedComponentsWithStats(crop, labels, stats, centroids, 8, CV_32S);

    bool found = false;
    int best_bottom_y = std::numeric_limits<int>::min();
    double best_area = 0.0;
    cv::Rect best_box;
    int best_label = -1;

    for (int label = 1; label < num_labels; ++label)
    {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        const int left = stats.at<int>(label, cv::CC_STAT_LEFT);
        const int top = stats.at<int>(label, cv::CC_STAT_TOP);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        const double aspect_ratio = static_cast<double>(width) / std::max(1, height);
        if (area < kTaskMinBandArea ||
            width < kTaskMinBandWidth ||
            height < kTaskMinBandHeight ||
            aspect_ratio < kTaskMinBandAspectRatio)
        {
            continue;
        }

        const cv::Rect box(rect.x + left, rect.y + top, width, height);
        const int bottom_y = box.y + box.height;
        if (!found || bottom_y > best_bottom_y || (bottom_y == best_bottom_y && area > best_area))
        {
            found = true;
            best_bottom_y = bottom_y;
            best_area = static_cast<double>(area);
            best_box = box;
            best_label = label;
        }
    }

    if (!found || best_label < 0)
    {
        return false;
    }

    cv::Mat component_mask = cv::Mat::zeros(crop.size(), CV_8UC1);
    component_mask.setTo(255, labels == best_label);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(component_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
    {
        return false;
    }

    auto contour_it = std::max_element(
        contours.begin(),
        contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return std::fabs(cv::contourArea(a)) < std::fabs(cv::contourArea(b));
        });
    std::vector<cv::Point> best_contour = *contour_it;
    for (cv::Point& pt : best_contour)
    {
        pt.x += rect.x;
        pt.y += rect.y;
    }

    *out_contour = best_contour;
    *out_box = best_box;
    *out_area = best_area;
    return true;
}

} // namespace

RoiMethod DefaultRoiMethod()
{
    return RoiMethod::IPM_SQUARE_FROM_TOP_EDGE;
}

const char* RoiMethodName(RoiMethod method)
{
    switch (method)
    {
    case RoiMethod::DIRECT_RED_QUAD:
        return "direct_red_quad";
    case RoiMethod::IPM_SQUARE_FROM_TOP_EDGE:
        return "ipm_square_from_top_edge";
    default:
        return "unknown";
    }
}

bool DetectTrackAwareRedPrefilter(const cv::Mat& frame_bgr,
                                  int early_y_min,
                                  int early_y_max,
                                  int recognition_y_min,
                                  int recognition_y_max,
                                  bool collect_debug_geometry,
                                  RoiTrackRedPrefilterResult* out_result)
{
    if (out_result == nullptr)
    {
        return false;
    }
    *out_result = RoiTrackRedPrefilterResult();
    if (frame_bgr.empty())
    {
        return false;
    }

    TaskWhiteReferenceStats white_ref_stats;
    if (!GetTaskWhiteReferenceStats(frame_bgr, &white_ref_stats) || !white_ref_stats.valid)
    {
        return false;
    }
    const TaskTrackWhiteThresholds white_thresholds =
        BuildTaskTrackWhiteThresholds(white_ref_stats);
    const TaskStrictRedThresholds red_thresholds =
        BuildTaskStrictRedThresholds(white_ref_stats);

    TaskTrackBoundaryState track_state;
    if (!BuildTaskTrackBoundaryState(frame_bgr, white_thresholds, &track_state))
    {
        return false;
    }
    out_result->has_track_boundaries = true;
    if (collect_debug_geometry)
    {
        out_result->track_left_boundary =
            BuildTrackBoundaryDisplayPoints(track_state.seed_left_x, track_state.seed_y, track_state.left_points);
        out_result->track_right_boundary =
            BuildTrackBoundaryDisplayPoints(track_state.seed_right_x, track_state.seed_y, track_state.right_points);
        out_result->track_region_polygon = track_state.region_polygon;
    }

    const int rows = frame_bgr.rows;
    const int cols = frame_bgr.cols;
    const int early_y0 = std::max(0, std::min(early_y_min, rows));
    const int early_y1 = std::max(early_y0, std::min(early_y_max, rows));
    const int recog_y0 = std::max(0, std::min(recognition_y_min, rows));
    const int recog_y1 = std::max(recog_y0, std::min(recognition_y_max, rows));
    const int scan_y0 = std::min(early_y0, recog_y0);
    const int scan_y1 = std::max(early_y1, recog_y1);
    if (scan_y1 <= scan_y0 || cols <= 0)
    {
        return true;
    }

    TaskPrefilterRedBounds early_marker_bounds;
    TaskPrefilterRedBounds recognition_marker_bounds;
    TaskPrefilterRedBounds recognition_brick_bounds;

    for (int y = scan_y0; y < scan_y1; ++y)
    {
        int left_x = -1;
        int right_x = -1;
        int boundary_y = -1;
        if (!FindTrackRegionBoundsAtRow(track_state, y, &left_x, &right_x, &boundary_y))
        {
            continue;
        }

        const bool in_early_band = y >= early_y0 && y < early_y1;
        const bool in_recognition_band = y >= recog_y0 && y < recog_y1;
        if (!in_early_band && !in_recognition_band)
        {
            continue;
        }

        const cv::Vec3b* row = frame_bgr.ptr<cv::Vec3b>(y);
        const TaskTrackRowSearchBands bands =
            BuildTaskTrackRowSearchBands(left_x, right_x, cols, y);
        if (bands.has_marker)
        {
            for (int x = bands.marker_x0; x <= bands.marker_x1; ++x)
            {
                if (!IsTaskPrefilterRedPixel(row[x], red_thresholds))
                {
                    continue;
                }
                if (in_early_band)
                {
                    early_marker_bounds.Add(x, y);
                }
                if (in_recognition_band)
                {
                    recognition_marker_bounds.Add(x, y);
                }
            }
        }

        if (!in_recognition_band)
        {
            continue;
        }

        if (bands.has_left_brick)
        {
            for (int x = bands.left_brick_x0; x <= bands.left_brick_x1; ++x)
            {
                if (!IsTaskPrefilterRedPixel(row[x], red_thresholds))
                {
                    continue;
                }
                recognition_brick_bounds.Add(x, y);
            }
        }

        if (bands.has_right_brick)
        {
            for (int x = bands.right_brick_x0; x <= bands.right_brick_x1; ++x)
            {
                if (!IsTaskPrefilterRedPixel(row[x], red_thresholds))
                {
                    continue;
                }
                recognition_brick_bounds.Add(x, y);
            }
        }
    }

    out_result->has_early_marker_red =
        early_marker_bounds.ToRect(24, 4, 4, &out_result->early_marker_rect);
    out_result->has_recognition_marker_red =
        recognition_marker_bounds.ToRect(8, 2, 2, &out_result->recognition_marker_rect);
    out_result->has_recognition_brick_red =
        recognition_brick_bounds.ToRect(1, 1, 1, &out_result->recognition_brick_rect);
    return true;
}

static void FillCandidateFields(RoiExtractionResult* result,
                                const cv::Rect& candidate_box,
                                double candidate_area)
{
    if (result == nullptr)
    {
        return;
    }

    result->has_loose_blob_box = true;
    result->loose_blob_box = candidate_box;
    result->has_loose_blob_area = true;
    result->loose_blob_area = candidate_area;
    result->has_blob_box = true;
    result->blob_box = candidate_box;
    result->has_blob_area = true;
    result->blob_area = candidate_area;
    result->has_candidate_area = true;
    result->candidate_area = candidate_area;
    result->has_candidate_center = true;
    result->candidate_center_x = static_cast<float>(candidate_box.x + candidate_box.width * 0.5f);
    result->candidate_center_y = static_cast<float>(candidate_box.y + candidate_box.height * 0.5f);
    result->has_candidate_size = true;
    result->candidate_width = candidate_box.width;
    result->candidate_height = candidate_box.height;
}

static bool DetectTrackBrickOnly(const cv::Mat& frame_bgr,
                                 const TaskStrictRedThresholds& red_thresholds,
                                 const TaskTrackBoundaryState& track_state,
                                 int y_min,
                                 int y_max,
                                 RoiExtractionResult* result)
{
    if (result == nullptr)
    {
        return false;
    }

    cv::Rect brick_box;
    double brick_area = 0.0;
    TaskTrackClassification brick_classification;
    const auto brick_band_begin = steady_clock_t::now();
    const bool has_brick = FindTrackBrickRedInOuterBand(
        frame_bgr,
        red_thresholds,
        track_state,
        &brick_box,
        &brick_area,
        &brick_classification,
        y_min,
        y_max);
    result->timing_red_band_ms += elapsed_ms(brick_band_begin, steady_clock_t::now());
    if (!has_brick)
    {
        return false;
    }

    FillCandidateFields(result, brick_box, brick_area);
    if (brick_classification.classify_point.x >= 0 && brick_classification.classify_point.y >= 0)
    {
        result->has_track_classify_point = true;
        result->track_classify_point = brick_classification.classify_point;
    }
    if (brick_classification.left_boundary_x >= 0 && brick_classification.right_boundary_x >= 0)
    {
        result->has_track_classify_bounds = true;
        result->track_classify_left_x = brick_classification.left_boundary_x;
        result->track_classify_right_x = brick_classification.right_boundary_x;
        result->track_classify_row_y = brick_classification.boundary_row_y;
    }
    result->target_type = "roadblock";
    result->status = "roadblock";
    return true;
}

static bool TryDetectTrackBrickFallback(const cv::Mat& frame_bgr,
                                        const TaskStrictRedThresholds& red_thresholds,
                                        const TaskTrackBoundaryState& track_state,
                                        int image_width,
                                        int image_height,
                                        RoiExtractionResult* result)
{
    if (result == nullptr || frame_bgr.empty() || !track_state.valid)
    {
        return false;
    }

    if (!DetectTrackBrickOnly(
            frame_bgr,
            red_thresholds,
            track_state,
            kTaskBrickSearchYMin,
            kTaskBrickSearchYMax,
            result))
    {
        return false;
    }

    const cv::Rect brick_search_rect = BuildTaskTrackSearchRect(
        track_state, image_width, image_height, kTaskBrickSearchYMin, kTaskBrickSearchYMax);
    if (brick_search_rect.width > 0 && brick_search_rect.height > 0)
    {
        result->search_rect = brick_search_rect;
        result->has_search_rect = true;
    }
    return true;
}


RoiExtractionResult ExtractRotatedRoi(const cv::Mat& frame_bgr,
                                      int output_size,
                                      RoiMethod roi_method,
                                      bool render_debug)
{
    RoiExtractionResult result;
    result.roi_method = roi_method;

    if (frame_bgr.empty())
    {
        return result;
    }

    const int image_width = frame_bgr.cols;
    const int image_height = frame_bgr.rows;

    TaskWhiteReferenceStats white_ref_stats;
    GetTaskWhiteReferenceStats(frame_bgr, &white_ref_stats);
    const TaskTrackWhiteThresholds white_thresholds =
        BuildTaskTrackWhiteThresholds(white_ref_stats);
    const TaskStrictRedThresholds red_thresholds =
        BuildTaskStrictRedThresholds(white_ref_stats);

    TaskTrackBoundaryState track_state;
    const auto track_boundary_begin = steady_clock_t::now();
    const bool has_track_boundaries = BuildTaskTrackBoundaryState(
        frame_bgr,
        white_thresholds,
        &track_state);
    result.timing_track_boundary_ms = elapsed_ms(track_boundary_begin, steady_clock_t::now());
    if (render_debug)
    {
        result.track_left_boundary =
            BuildTrackBoundaryDisplayPoints(track_state.seed_left_x, track_state.seed_y, track_state.left_points);
        result.track_right_boundary =
            BuildTrackBoundaryDisplayPoints(track_state.seed_right_x, track_state.seed_y, track_state.right_points);
        result.has_track_left_boundary = !result.track_left_boundary.empty();
        result.has_track_right_boundary = !result.track_right_boundary.empty();
    }

    if (has_track_boundaries)
    {
        if (render_debug)
        {
            result.track_region_polygon = track_state.region_polygon;
            result.has_track_region_polygon = (result.track_region_polygon.size() >= 4);
        }
    }
    else
    {
        result.status = "track_boundary_miss";
        return result;
    }

    const auto search_rect_begin = steady_clock_t::now();
    result.search_rect = BuildTaskTrackSearchRect(
        track_state, image_width, image_height, kTaskMarkerSearchYMin, kTaskMarkerSearchYMax);
    result.timing_search_rect_ms = elapsed_ms(search_rect_begin, steady_clock_t::now());
    if (result.search_rect.width <= 0 || result.search_rect.height <= 0)
    {
        result.status = "track_boundary_miss";
        return result;
    }
    result.reference_range_source = "track_interior";
    result.has_search_rect = true;

    const auto red_mask_begin = steady_clock_t::now();
    cv::Mat marker_red_mask;
    cv::Rect marker_mask_image_rect;
    const auto rebuild_marker_red_mask = [&]() {
        marker_red_mask = BuildTaskMarkerRedMaskLocalInTrackInterior(
            frame_bgr,
            track_state,
            result.search_rect,
            kTaskMarkerSearchYMin,
            kTaskMarkerSearchYMax,
            red_thresholds,
            &marker_mask_image_rect);
    };
    rebuild_marker_red_mask();
    result.timing_red_mask_ms = elapsed_ms(red_mask_begin, steady_clock_t::now());

    int expand_steps = 0;
    while (expand_steps < kTaskEdgeExpandMaxSteps)
    {
        bool touch_left = false;
        bool touch_top = false;
        bool touch_right = false;
        bool touch_bottom = false;
        const cv::Rect local_mask_rect(0, 0, marker_red_mask.cols, marker_red_mask.rows);
        TaskTouchedSides(marker_red_mask, local_mask_rect, &touch_left, &touch_top, &touch_right, &touch_bottom);
        if (has_track_boundaries)
        {
            // Track boundaries already define the allowed brick outer band.
            touch_left = false;
            touch_right = false;
        }
        if (!(touch_left || touch_top || touch_right || touch_bottom))
        {
            break;
        }
        const cv::Rect expanded_rect = ExpandTaskRect(
            result.search_rect,
            image_width,
            image_height,
            touch_left,
            touch_top,
            touch_right,
            touch_bottom);
        if (expanded_rect == result.search_rect)
        {
            break;
        }
        result.search_rect = expanded_rect;
        const auto red_mask_rebuild_begin = steady_clock_t::now();
        rebuild_marker_red_mask();
        result.timing_red_mask_ms += elapsed_ms(red_mask_rebuild_begin, steady_clock_t::now());
        ++expand_steps;
    }

    std::vector<cv::Point> candidate_contour;
    cv::Rect candidate_box;
    double candidate_area = 0.0;
    TaskTrackClassification track_classification;
    const auto red_band_begin = steady_clock_t::now();
    const cv::Rect marker_mask_rect(0, 0, marker_red_mask.cols, marker_red_mask.rows);
    const bool has_candidate = ChooseLowestTaskRedBand(
        marker_red_mask, marker_mask_rect, &candidate_contour, &candidate_box, &candidate_area);
    if (!has_candidate)
    {
        result.timing_red_band_ms += elapsed_ms(red_band_begin, steady_clock_t::now());
        if (TryDetectTrackBrickFallback(
                frame_bgr,
                red_thresholds,
                track_state,
                image_width,
                image_height,
                &result))
        {
            return result;
        }
        result.status = "miss";
        return result;
    }
    const cv::Point marker_mask_offset = marker_mask_image_rect.tl();
    for (cv::Point& pt : candidate_contour)
    {
        pt += marker_mask_offset;
    }
    candidate_box.x += marker_mask_offset.x;
    candidate_box.y += marker_mask_offset.y;
    result.timing_red_band_ms += elapsed_ms(red_band_begin, steady_clock_t::now());

    const int candidate_bottom_y = candidate_box.y + candidate_box.height - 1;
    if (candidate_bottom_y < kTaskMarkerTriggerYMin)
    {
        if (TryDetectTrackBrickFallback(
                frame_bgr,
                red_thresholds,
                track_state,
                image_width,
                image_height,
                &result))
        {
            return result;
        }
        result.status = "marker_above_trigger_y";
        return result;
    }

    FillCandidateFields(&result, candidate_box, candidate_area);

    const auto track_classify_begin = steady_clock_t::now();
    if (has_track_boundaries && track_classification.type == TaskTrackCandidateType::UNKNOWN)
    {
        track_classification =
            ClassifyTaskCandidateByTrackBoundary(track_state, candidate_box, image_width);
    }
    result.timing_track_classify_ms = elapsed_ms(track_classify_begin, steady_clock_t::now());

    if (track_classification.classify_point.x >= 0 && track_classification.classify_point.y >= 0)
    {
        result.has_track_classify_point = true;
        result.track_classify_point = track_classification.classify_point;
    }
    if (track_classification.left_boundary_x >= 0 && track_classification.right_boundary_x >= 0)
    {
        result.has_track_classify_bounds = true;
        result.track_classify_left_x = track_classification.left_boundary_x;
        result.track_classify_right_x = track_classification.right_boundary_x;
        result.track_classify_row_y = track_classification.boundary_row_y;
    }

    if (track_classification.type != TaskTrackCandidateType::MARKER)
    {
        if (TryDetectTrackBrickFallback(
                frame_bgr,
                red_thresholds,
                track_state,
                image_width,
                image_height,
                &result))
        {
            return result;
        }
        result.status = has_track_boundaries ? "track_classify_miss" : "track_boundary_miss";
        return result;
    }
    result.target_type = "marker";

    const auto roi_build_warp_begin = steady_clock_t::now();
    result.blob_quad = RotatedBoxPointsFromContour(candidate_contour);
    const BuildRoiQuadResult build_result =
        BuildRoiQuadFromBlobQuad(result.blob_quad, image_width, image_height, roi_method);
    result.blob_quad_final = build_result.blob_quad_final;
    result.roi_quad_final = build_result.roi_quad_final;
    if (build_result.has_raw_height_ratio)
    {
        result.has_ipm_backproject_height_ratio = true;
        result.ipm_backproject_height_ratio = build_result.raw_height_ratio;
    }
    result.ipm_reason = build_result.ipm_reason;
    if (!build_result.valid)
    {
        result.timing_roi_build_warp_ms = elapsed_ms(roi_build_warp_begin, steady_clock_t::now());
        result.status = build_result.status;
        return result;
    }

    result.roi_quad = build_result.roi_quad;
    result.roi_bgr = WarpRoiFromQuad(frame_bgr, result.roi_quad, output_size);
    if (result.roi_bgr.empty())
    {
        result.timing_roi_build_warp_ms = elapsed_ms(roi_build_warp_begin, steady_clock_t::now());
        result.status = "ipm_backproject_invalid";
        return result;
    }
    result.timing_roi_build_warp_ms = elapsed_ms(roi_build_warp_begin, steady_clock_t::now());

    result.status = "rotated_roi";
    return result;
}


RoiQualityMetrics ComputeLowInformationRoiMetrics(const cv::Mat& roi_bgr,
                                                  RoiMethod roi_method,
                                                  const RoiExtractionResult& roi_result)
{
    RoiQualityMetrics metrics;
#if BW_RECOG_ROI_LOW_INFO_FILTER_ENABLE == 0
    (void)roi_bgr;
    (void)roi_method;
    (void)roi_result;
    metrics.valid = true;
    metrics.reason = "disabled";
    return metrics;
#endif

    if (roi_bgr.empty())
    {
        metrics.reason = "empty_roi";
        return metrics;
    }

    cv::Mat gray;
    if (roi_bgr.channels() == 1)
    {
        gray = roi_bgr;
    }
    else
    {
        cv::cvtColor(roi_bgr, gray, cv::COLOR_BGR2GRAY);
    }

    const int inspect_height = std::max(1, static_cast<int>(std::lround(gray.rows * 0.75)));
    const cv::Mat inspect_region = gray.rowRange(0, inspect_height);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(inspect_region, mean, stddev);
    metrics.gray_std_top = stddev[0];

    cv::Mat canny;
    cv::Canny(inspect_region, canny, 40, 120);
    metrics.canny_density_top =
        static_cast<double>(cv::countNonZero(canny)) /
        std::max(1.0, static_cast<double>(canny.total()));

    cv::Mat lap;
    cv::Laplacian(inspect_region, lap, CV_64F);
    cv::Scalar lap_mean;
    cv::Scalar lap_stddev;
    cv::meanStdDev(lap, lap_mean, lap_stddev);
    metrics.lap_var_top = lap_stddev[0] * lap_stddev[0];

    metrics.valid = !(
        metrics.gray_std_top < 8.0 &&
        metrics.canny_density_top < 0.003 &&
        metrics.lap_var_top < 20.0);
    metrics.reason = metrics.valid ? "ok" : "low_info_reject";

    if (roi_method == RoiMethod::IPM_SQUARE_FROM_TOP_EDGE &&
        roi_result.has_ipm_backproject_height_ratio &&
        roi_result.ipm_backproject_height_ratio < kIpmShallowLowInfoMaxHeightRatio &&
        metrics.gray_std_top < kIpmShallowLowInfoMaxGrayStd &&
        metrics.canny_density_top < kIpmShallowLowInfoMaxCannyDensity &&
        metrics.lap_var_top < kIpmShallowLowInfoMaxLapVar)
    {
        metrics.valid = false;
        metrics.reason = "ipm_shallow_low_info_reject";
    }

    return metrics;
}

void DrawRoiDebugOverlay(cv::Mat& image_bgr, const RoiExtractionResult& result)
{
    if (image_bgr.empty())
    {
        return;
    }

    if (result.has_search_rect)
    {
        DrawRectIfValid(image_bgr, result.search_rect, cv::Scalar(255, 128, 0), 1);
    }
    if (result.has_reference_x_range)
    {
        cv::line(
            image_bgr,
            cv::Point(result.reference_x_min, 0),
            cv::Point(result.reference_x_min, image_bgr.rows - 1),
            cv::Scalar(0, 255, 0),
            1,
            cv::LINE_AA);
        cv::line(
            image_bgr,
            cv::Point(result.reference_x_max, 0),
            cv::Point(result.reference_x_max, image_bgr.rows - 1),
              cv::Scalar(0, 255, 0),
              1,
              cv::LINE_AA);
        if (result.merged_reference_span_count > 1)
        {
            std::ostringstream span_text;
            span_text << "ref_spans=" << result.merged_reference_span_count;
            cv::putText(image_bgr, span_text.str(), cv::Point(16, 84),
                        cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }
    }
    if (!result.track_left_boundary.empty())
    {
        cv::polylines(
            image_bgr,
            std::vector<std::vector<cv::Point>>(1, result.track_left_boundary),
            false,
            cv::Scalar(255, 0, 0),
            2,
            cv::LINE_AA);
    }
    if (!result.track_right_boundary.empty())
    {
        cv::polylines(
            image_bgr,
            std::vector<std::vector<cv::Point>>(1, result.track_right_boundary),
            false,
            cv::Scalar(0, 255, 255),
            2,
            cv::LINE_AA);
    }
    if (result.has_track_region_polygon && result.track_region_polygon.size() >= 4)
    {
        cv::polylines(
            image_bgr,
            std::vector<std::vector<cv::Point>>(1, result.track_region_polygon),
            true,
            cv::Scalar(255, 0, 255),
            1,
            cv::LINE_AA);
    }
    if (result.has_track_classify_bounds)
    {
        cv::line(
            image_bgr,
            cv::Point(result.track_classify_left_x, result.track_classify_row_y),
            cv::Point(result.track_classify_right_x, result.track_classify_row_y),
            cv::Scalar(0, 255, 0),
            1,
            cv::LINE_AA);
    }
    if (result.has_track_classify_point)
    {
        cv::circle(
            image_bgr,
            result.track_classify_point,
            4,
            (result.target_type == "roadblock") ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0),
            -1,
            cv::LINE_AA);
    }
    if (result.has_support_rect)
    {
        DrawRectIfValid(image_bgr, result.support_rect, cv::Scalar(255, 0, 255), 2);
    }
    if (result.has_loose_blob_box)
    {
        DrawRectIfValid(image_bgr, result.loose_blob_box, cv::Scalar(0, 165, 255), 2);
    }
    if (result.has_blob_box)
    {
        DrawRectIfValid(image_bgr, result.blob_box, cv::Scalar(0, 0, 255), 2);
    }
    DrawQuadIfValid(image_bgr, result.blob_quad, cv::Scalar(0, 0, 255), 2);
    DrawQuadIfValid(image_bgr, result.roi_quad, cv::Scalar(255, 255, 0), 2);

    cv::putText(
        image_bgr,
        result.status,
        cv::Point(16, 28),
        cv::FONT_HERSHEY_SIMPLEX,
        0.70,
        cv::Scalar(0, 255, 255),
        2,
        cv::LINE_AA);
    cv::putText(
        image_bgr,
        RoiMethodName(result.roi_method),
        cv::Point(16, 56),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(255, 220, 0),
        2,
        cv::LINE_AA);
}

