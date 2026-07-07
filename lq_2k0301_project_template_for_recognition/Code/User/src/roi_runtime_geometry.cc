#include "roi_runtime_geometry.h"

#include "common.h"
#include "transform_table.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace {

constexpr int kRedMinArea = BW_RECOG_RED_MIN_AREA;
constexpr int kLooseRedMinArea = BW_RECOG_LOOSE_RED_MIN_AREA;
constexpr double kMinBlobFill = 0.08;
constexpr int kCoreRedMinArea = 100;
constexpr double kCoreMinBlobFill = 0.18;
constexpr double kMinQuadRedFill = 0.95;
constexpr double kMinQuadCoreFill = 0.85;
constexpr int kQuadFillErodePixels = 2;
constexpr int kMaxRedYExclusive = BW_RECOG_RED_MASK_MAX_Y;
constexpr int kIpmFrameWidth = BW_RECOG_TRANSFORM_TABLE_WIDTH;
constexpr int kIpmFrameHeight = BW_RECOG_TRANSFORM_TABLE_HEIGHT;
constexpr double kIpmMinWidthHeightRatio = 1.15;
constexpr double kMinRoiEdgeLength = 4.0;
constexpr double kMinBackprojectedQuadArea = 6.0;


constexpr int kTaskSearchYMin = BW_RECOG_TRIGGER_SEARCH_Y_MIN;
constexpr int kTaskSearchYMax = BW_RECOG_TRIGGER_SEARCH_Y_MAX;
constexpr int kTaskTrackBoundaryYMin = BW_RECOG_TRACK_BOUNDARY_Y_MIN;
constexpr int kTaskTrackTraceTopY = BW_RECOG_PROCESS_KEEP_Y_MIN;
constexpr int kTaskRedScoreThreshold = 140;
constexpr int kTaskRedMinR = 90;
constexpr int kTaskRedDomThreshold = 80;
constexpr int kTaskEdgeExpandStep = 4;
constexpr int kTaskEdgeExpandMaxSteps = 12;
constexpr int kTaskMinBandArea = 8;
constexpr int kTaskMinBandWidth = 3;
constexpr int kTaskMinBandHeight = 2;
constexpr double kTaskMinBandAspectRatio = 1.4;
constexpr int kStripRejectMaxHeight = 34;
constexpr double kStripRejectMinAspectRatio = 1.55;
constexpr double kStripSupportMinAreaRatio = 1.25;
constexpr double kStripSupportMinHeightRatio = 1.35;
constexpr double kStripSupportMinWidthRatio = 1.05;
constexpr double kStripSupportMinXOverlapRatio = 0.55;
constexpr int kStripSupportMaxTopGap = 8;
constexpr int kStripSupportMinBelowPixels = 8;
constexpr int kWhiteReferenceRowY = BW_RECOG_WHITE_REFERENCE_ROW_Y;
constexpr int kWhiteMaxSaturation = 60;
constexpr int kWhiteMinValue = 150;
constexpr int kWhiteMinRgb = 165;
constexpr int kWhiteMaxChannelDiff = 40;
constexpr int kWhiteMinSpanWidth = 60;
constexpr int kTrackBrickOuterExpandPixels = BW_RECOG_TRACK_BRICK_OUTER_EXPAND_PIXELS;
constexpr int kTrackMazeMaxSteps = BW_RECOG_TRACK_MAZE_MAX_STEPS;
constexpr unsigned char kTrackWhitePixel = 255;
constexpr unsigned char kTrackNonWhitePixel = 0;

constexpr int kTrackDirectionFront[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
constexpr int kTrackDirectionFrontLeft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
constexpr int kTrackDirectionFrontRight[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

constexpr int kMorphKernelSize = 5;
constexpr int kMorphOpenIters = 1;
constexpr int kMorphCloseIters = 2;

constexpr double kGeometryBorrowMaxRawHeight = 18.0;
constexpr double kGeometryBorrowMinAspectRatio = 2.2;
constexpr double kGeometryBorrowMinHeightRatio = 1.5;
constexpr double kGeometryBorrowMinAngleDelta = 0.75;
constexpr int kGeometryBorrowMinPadX = 6;
constexpr int kGeometryBorrowMinPadTop = 10;
constexpr int kGeometryBorrowMinPadBottom = 6;

constexpr double kIpmShallowLowInfoMaxHeightRatio = 0.45;
constexpr double kIpmShallowLowInfoMaxGrayStd = 12.0;
constexpr double kIpmShallowLowInfoMaxCannyDensity = 0.05;
constexpr double kIpmShallowLowInfoMaxLapVar = 40.0;

const cv::Scalar kLowRed1(0, 50, 50);
const cv::Scalar kHighRed1(10, 255, 255);
const cv::Scalar kLowRed2(160, 50, 50);
const cv::Scalar kHighRed2(180, 255, 255);
const cv::Scalar kCoreLowRed1(0, 90, 70);
const cv::Scalar kCoreHighRed1(10, 255, 255);
const cv::Scalar kCoreLowRed2(160, 90, 70);
const cv::Scalar kCoreHighRed2(180, 255, 255);
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

struct QuadFillMetrics
{
    bool valid = false;
    std::string reason;
    float red_fill = 0.0f;
    float core_fill = 0.0f;
};

struct IpmQuadMetrics
{
    bool valid = false;
    std::string reason;
    float top_width = 0.0f;
    float bottom_width = 0.0f;
    float left_height = 0.0f;
    float right_height = 0.0f;
};

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
    std::vector<int> left_x_by_row;
    std::vector<int> right_x_by_row;
    std::vector<cv::Point> left_points;
    std::vector<cv::Point> right_points;
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

static void TrimMaskToMaxRedY(cv::Mat& mask)
{
    if (!mask.empty() && mask.rows > kMaxRedYExclusive)
    {
        mask.rowRange(kMaxRedYExclusive, mask.rows).setTo(cv::Scalar(0));
    }
}

static cv::Mat PostprocessMask(const cv::Mat& mask)
{
    cv::Mat output = mask.clone();
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kMorphKernelSize, kMorphKernelSize));
    cv::morphologyEx(output, output, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), kMorphOpenIters);
    cv::morphologyEx(output, output, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), kMorphCloseIters);
    return output;
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

static bool ComputeTaskWhiteReferenceStats(const cv::Mat& frame_bgr,
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
    const int b = static_cast<int>(bgr[0]);
    const int g = static_cast<int>(bgr[1]);
    const int r = static_cast<int>(bgr[2]);
    const int red_score = 2 * r - g - b;
    const int dom = r - std::max(g, b);
    return red_score >= thresholds.red_score &&
           r >= thresholds.min_r &&
           dom >= thresholds.dom;
}

static bool ComputeWhiteEnvelopeXRangeOnReferenceRow(const cv::Mat& frame_bgr,
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

    TaskWhiteReferenceStats white_ref_stats;
    ComputeTaskWhiteReferenceStats(frame_bgr, &white_ref_stats);
    const TaskTrackWhiteThresholds white_thresholds =
        BuildTaskTrackWhiteThresholds(white_ref_stats);

    const int row_y = std::max(0, std::min(kWhiteReferenceRowY, image_height - 1));
    const cv::Mat row_bgr = frame_bgr.row(row_y).clone();
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

static bool ComputeStrictRedEnvelopeXRangeOnReferenceRow(const cv::Mat& frame_bgr,
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

    TaskWhiteReferenceStats white_ref_stats;
    ComputeTaskWhiteReferenceStats(frame_bgr, &white_ref_stats);
    const TaskStrictRedThresholds red_thresholds =
        BuildTaskStrictRedThresholds(white_ref_stats);

    const int row_y = std::max(0, std::min(kWhiteReferenceRowY, image_height - 1));
    const cv::Mat row_bgr = frame_bgr.row(row_y);

    int merged_start = -1;
    int merged_end = -1;
    int merged_count = 0;
    int start = -1;
    for (int x = 0; x < image_width; ++x)
    {
        const bool is_red = IsTaskStrictRedPixel(row_bgr.at<cv::Vec3b>(0, x), red_thresholds);
        if (is_red && start < 0)
        {
            start = x;
        }
        else if (!is_red && start >= 0)
        {
            MergeRowSpanIntoEnvelope(
                start, x - 1, kTaskMinBandWidth, &merged_start, &merged_end, &merged_count);
            start = -1;
        }
    }
    if (start >= 0)
    {
        MergeRowSpanIntoEnvelope(
            start, image_width - 1, kTaskMinBandWidth, &merged_start, &merged_end, &merged_count);
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

static bool ComputeReferenceEnvelopeXRangeOnRow(const cv::Mat& frame_bgr,
                                                int* out_x_min,
                                                int* out_x_max,
                                                int* out_span_count,
                                                std::string* out_source)
{
    if (out_x_min == nullptr || out_x_max == nullptr || frame_bgr.empty())
    {
        return false;
    }
    if (out_span_count != nullptr)
    {
        *out_span_count = 0;
    }
    if (out_source != nullptr)
    {
        *out_source = "fallback";
    }

    int white_x_min = 0;
    int white_x_max = 0;
    int white_span_count = 0;
    const bool has_white = ComputeWhiteEnvelopeXRangeOnReferenceRow(
        frame_bgr, &white_x_min, &white_x_max, &white_span_count);

    int red_x_min = 0;
    int red_x_max = 0;
    int red_span_count = 0;
    const bool has_red = ComputeStrictRedEnvelopeXRangeOnReferenceRow(
        frame_bgr, &red_x_min, &red_x_max, &red_span_count);

    if (!has_white && !has_red)
    {
        return false;
    }

    int x_min = std::numeric_limits<int>::max();
    int x_max = std::numeric_limits<int>::min();
    if (has_white)
    {
        x_min = std::min(x_min, white_x_min);
        x_max = std::max(x_max, white_x_max);
    }
    if (has_red)
    {
        x_min = std::min(x_min, red_x_min);
        x_max = std::max(x_max, red_x_max);
    }

    *out_x_min = x_min;
    *out_x_max = x_max;
    if (out_span_count != nullptr)
    {
        *out_span_count = white_span_count + red_span_count;
    }
    if (out_source != nullptr)
    {
        if (has_white && has_red)
        {
            *out_source = "white_red_envelope";
        }
        else if (has_white)
        {
            *out_source = "white_only";
        }
        else
        {
            *out_source = "red_only";
        }
    }
    return true;
}

static cv::Rect ApplyReferenceXRangeToRect(const cv::Rect& rect,
                                           const cv::Mat& frame_bgr,
                                           int* out_x_min,
                                           int* out_x_max,
                                           std::string* out_source,
                                           int* out_span_count)
{
    int reference_x_min = 0;
    int reference_x_max = 0;
    int span_count = 0;
    std::string detected_source = "fallback";
    if (!ComputeReferenceEnvelopeXRangeOnRow(
            frame_bgr, &reference_x_min, &reference_x_max, &span_count, &detected_source))
    {
        if (out_source != nullptr)
        {
            *out_source = "fallback";
        }
        return rect;
    }

    if (out_x_min != nullptr)
    {
        *out_x_min = reference_x_min;
    }
    if (out_x_max != nullptr)
    {
        *out_x_max = reference_x_max;
    }
    if (out_span_count != nullptr)
    {
        *out_span_count = span_count;
    }

    const int x1 = std::max(rect.x, reference_x_min);
    const int x2 = std::min(rect.x + rect.width, reference_x_max + 1);
    if (x2 <= x1)
    {
        if (out_source != nullptr)
        {
            *out_source = "fallback";
        }
        return rect;
    }

    if (out_source != nullptr)
    {
        *out_source = detected_source;
    }
    return cv::Rect(x1, rect.y, x2 - x1, rect.height);
}

static bool IsTaskTrackWhitePixel(const cv::Vec3b& bgr,
                                  const cv::Vec3b& hsv,
                                  const TaskTrackWhiteThresholds& thresholds)
{
    const int max_rgb = std::max(std::max(static_cast<int>(bgr[0]), static_cast<int>(bgr[1])),
                                 static_cast<int>(bgr[2]));
    const int min_rgb = std::min(std::min(static_cast<int>(bgr[0]), static_cast<int>(bgr[1])),
                                 static_cast<int>(bgr[2]));
    return hsv[1] <= static_cast<unsigned char>(thresholds.max_saturation) &&
           hsv[2] >= static_cast<unsigned char>(thresholds.min_value) &&
           min_rgb >= thresholds.min_rgb &&
           (max_rgb - min_rgb) <= thresholds.max_channel_diff;
}

static cv::Mat BuildTaskWhiteTrackMask(const cv::Mat& frame_bgr)
{
    cv::Mat mask = cv::Mat::zeros(frame_bgr.size(), CV_8UC1);
    if (frame_bgr.empty())
    {
        return mask;
    }

    TaskWhiteReferenceStats white_ref_stats;
    ComputeTaskWhiteReferenceStats(frame_bgr, &white_ref_stats);
    const TaskTrackWhiteThresholds white_thresholds =
        BuildTaskTrackWhiteThresholds(white_ref_stats);

    cv::Mat hsv;
    cv::cvtColor(frame_bgr, hsv, cv::COLOR_BGR2HSV);

    const int y0 = std::max(0, std::min(kTaskTrackTraceTopY, frame_bgr.rows - 1));
    const int y1 = std::max(y0, std::min(kWhiteReferenceRowY, frame_bgr.rows - 1));
    for (int y = y0; y <= y1; ++y)
    {
        const cv::Vec3b* bgr_row = frame_bgr.ptr<cv::Vec3b>(y);
        const cv::Vec3b* hsv_row = hsv.ptr<cv::Vec3b>(y);
        unsigned char* mask_row = mask.ptr<unsigned char>(y);
        for (int x = 0; x < frame_bgr.cols; ++x)
        {
            if (IsTaskTrackWhitePixel(bgr_row[x], hsv_row[x], white_thresholds))
            {
                mask_row[x] = kTrackWhitePixel;
            }
        }
    }
    return mask;
}

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
    std::vector<cv::Point> polygon;
    if (!state.valid || state.seed_left_x < 0 || state.seed_right_x < 0)
    {
        return polygon;
    }

    const cv::Point bottom_left(state.seed_left_x, state.seed_y);
    const cv::Point bottom_right(state.seed_right_x, state.seed_y);
    const cv::Point top_left =
        state.left_points.empty() ? bottom_left : state.left_points.back();
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

static void TraceTaskWhiteBoundaryLeftMaze(const cv::Mat& white_mask,
                                           int start_y,
                                           int start_x,
                                           std::vector<cv::Point>* out_points)
{
    if (out_points == nullptr)
    {
        return;
    }
    out_points->clear();
    if (white_mask.empty())
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
        if (!(w > 0 && w < white_mask.cols - 1 && h > 0 && h < white_mask.rows - 1))
        {
            break;
        }

        const int fh = h + kTrackDirectionFront[dir][1];
        const int fw = w + kTrackDirectionFront[dir][0];
        const int flh = h + kTrackDirectionFrontLeft[dir][1];
        const int flw = w + kTrackDirectionFrontLeft[dir][0];
        const unsigned char front = white_mask.at<unsigned char>(fh, fw);
        const unsigned char front_left = white_mask.at<unsigned char>(flh, flw);

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

static void TraceTaskWhiteBoundaryRightMaze(const cv::Mat& white_mask,
                                            int start_y,
                                            int start_x,
                                            std::vector<cv::Point>* out_points)
{
    if (out_points == nullptr)
    {
        return;
    }
    out_points->clear();
    if (white_mask.empty())
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
        if (!(w > 0 && w < white_mask.cols - 1 && h > 0 && h < white_mask.rows - 1))
        {
            break;
        }

        const int fh = h + kTrackDirectionFront[dir][1];
        const int fw = w + kTrackDirectionFront[dir][0];
        const int frh = h + kTrackDirectionFrontRight[dir][1];
        const int frw = w + kTrackDirectionFrontRight[dir][0];
        const unsigned char front = white_mask.at<unsigned char>(fh, fw);
        const unsigned char front_right = white_mask.at<unsigned char>(frh, frw);

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

static void RasterizeTaskTrackBoundaryPoints(const std::vector<cv::Point>& points,
                                             bool is_left,
                                             std::vector<int>* x_by_row)
{
    if (x_by_row == nullptr)
    {
        return;
    }
    for (size_t i = 0; i < points.size(); ++i)
    {
        const cv::Point& pt = points[i];
        if (pt.y < 0 || pt.y >= static_cast<int>(x_by_row->size()))
        {
            continue;
        }
        int& row_x = (*x_by_row)[pt.y];
        if (row_x < 0)
        {
            row_x = pt.x;
            continue;
        }
        row_x = is_left ? std::min(row_x, pt.x) : std::max(row_x, pt.x);
    }
}

static bool BuildTaskTrackBoundaryState(const cv::Mat& frame_bgr,
                                        int reference_x_min,
                                        int reference_x_max,
                                        bool has_reference_x_range,
                                        TaskTrackBoundaryState* out_state)
{
    (void)reference_x_min;
    (void)reference_x_max;
    (void)has_reference_x_range;

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
    if (!ComputeWhiteEnvelopeXRangeOnReferenceRow(frame_bgr, &white_x_min, &white_x_max, nullptr))
    {
        return false;
    }

    cv::Mat white_mask = BuildTaskWhiteTrackMask(frame_bgr);
    int left_seed_x = -1;
    int right_seed_x = -1;
    const int seed_center_x = (white_x_min + white_x_max) / 2;
    const unsigned char* seed_row = white_mask.ptr<unsigned char>(bottom_y);
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
    out_state->left_x_by_row.assign(rows, -1);
    out_state->right_x_by_row.assign(rows, -1);
    if (left_seed_x >= 0)
    {
        out_state->left_x_by_row[bottom_y] = left_seed_x;
        TraceTaskWhiteBoundaryLeftMaze(white_mask, bottom_y, left_seed_x, &out_state->left_points);
    }
    if (right_seed_x >= 0)
    {
        out_state->right_x_by_row[bottom_y] = right_seed_x;
        TraceTaskWhiteBoundaryRightMaze(white_mask, bottom_y, right_seed_x, &out_state->right_points);
    }
    RemoveOverlappingTrackBoundaryPoints(&out_state->left_points, &out_state->right_points);

    if (!out_state->left_points.empty())
    {
        RasterizeTaskTrackBoundaryPoints(out_state->left_points, true, &out_state->left_x_by_row);
    }
    if (!out_state->right_points.empty())
    {
        RasterizeTaskTrackBoundaryPoints(out_state->right_points, false, &out_state->right_x_by_row);
    }

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
    return out_state->valid;
}

static cv::Rect BuildTaskTrackSearchRect(const TaskTrackBoundaryState& state,
                                         int image_width,
                                         int image_height)
{
    const int x0 = std::max(0, state.envelope_x_min - kTrackBrickOuterExpandPixels);
    const int x1 = std::min(image_width - 1, state.envelope_x_max + kTrackBrickOuterExpandPixels);
    cv::Rect rect(
        x0,
        std::max(0, std::min(kTaskSearchYMin, image_height - 1)),
        std::max(1, x1 - x0 + 1),
        std::max(1, std::min(kTaskSearchYMax, image_height) - std::max(0, std::min(kTaskSearchYMin, image_height - 1))));
    cv::Rect clamped;
    if (!ClampRectToImage(rect, image_width, image_height, &clamped))
    {
        return rect;
    }
    return clamped;
}

static TaskTrackClassification ClassifyTaskCandidateByTrackBoundary(
    const TaskTrackBoundaryState& state,
    const cv::Rect& candidate_box)
{
    TaskTrackClassification result;
    if (!state.valid || candidate_box.width <= 0 || candidate_box.height <= 0)
    {
        return result;
    }

    const int classify_x = candidate_box.x + candidate_box.width / 2;
    const int classify_y = candidate_box.y + candidate_box.height - 1;
    result.classify_point = cv::Point(classify_x, classify_y);

    const std::vector<cv::Point> polygon = BuildTrackRegionPolygon(state);
    if (polygon.size() < 4)
    {
        return result;
    }

    result.left_boundary_x = state.seed_left_x;
    result.right_boundary_x = state.seed_right_x;
    result.boundary_row_y = state.seed_y;
    const double inside = cv::pointPolygonTest(polygon, cv::Point2f((float)classify_x, (float)classify_y), false);
    if (inside >= 0.0)
    {
        result.type = TaskTrackCandidateType::MARKER;
        return result;
    }

    result.type = TaskTrackCandidateType::ROADBLOCK;
    return result;
}

static TaskTrackClassification ClassifyTaskCandidateByReferenceEnvelope(int reference_x_min,
                                                                        int reference_x_max,
                                                                        const cv::Rect& candidate_box)
{
    TaskTrackClassification result;
    if (candidate_box.width <= 0 || candidate_box.height <= 0 || reference_x_min >= reference_x_max)
    {
        return result;
    }

    const int classify_x = candidate_box.x + candidate_box.width / 2;
    const int classify_y = candidate_box.y + candidate_box.height - 1;
    result.classify_point = cv::Point(classify_x, classify_y);
    result.left_boundary_x = reference_x_min;
    result.right_boundary_x = reference_x_max;
    result.boundary_row_y = kWhiteReferenceRowY;

    if (classify_x >= reference_x_min && classify_x <= reference_x_max)
    {
        result.type = TaskTrackCandidateType::MARKER;
        return result;
    }

    if ((classify_x >= reference_x_min - kTrackBrickOuterExpandPixels && classify_x < reference_x_min) ||
        (classify_x > reference_x_max && classify_x <= reference_x_max + kTrackBrickOuterExpandPixels))
    {
        result.type = TaskTrackCandidateType::ROADBLOCK;
        return result;
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

static void BuildRedMasks(const cv::Mat& frame_bgr,
                          const cv::Rect* processing_rect,
                          cv::Mat* red_mask_out,
                          cv::Mat* core_red_mask_out)
{
    cv::Mat red_mask = cv::Mat::zeros(frame_bgr.size(), CV_8UC1);
    cv::Mat core_red_mask = cv::Mat::zeros(frame_bgr.size(), CV_8UC1);
    if (frame_bgr.empty())
    {
        *red_mask_out = red_mask;
        *core_red_mask_out = core_red_mask;
        return;
    }

    int x0 = 0;
    int y0 = 0;
    int x1 = frame_bgr.cols;
    int y1 = std::min(frame_bgr.rows, kMaxRedYExclusive);
    if (processing_rect != nullptr)
    {
        x0 = std::max(0, std::min(processing_rect->x, frame_bgr.cols - 1));
        y0 = std::max(0, std::min(processing_rect->y, frame_bgr.rows - 1));
        x1 = std::max(x0 + 1, std::min(processing_rect->x + processing_rect->width, frame_bgr.cols));
        y1 = std::max(y0 + 1, std::min(processing_rect->y + processing_rect->height, frame_bgr.rows));
        y1 = std::min(y1, kMaxRedYExclusive);
    }
    if (x1 <= x0 || y1 <= y0)
    {
        *red_mask_out = red_mask;
        *core_red_mask_out = core_red_mask;
        return;
    }

    const cv::Rect crop_rect(x0, y0, x1 - x0, y1 - y0);
    cv::Mat hsv;
    cv::cvtColor(frame_bgr(crop_rect), hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask_low;
    cv::Mat mask_high;
    cv::inRange(hsv, kLowRed1, kHighRed1, mask_low);
    cv::inRange(hsv, kLowRed2, kHighRed2, mask_high);
    cv::Mat crop_red_mask;
    cv::bitwise_or(mask_low, mask_high, crop_red_mask);

    cv::Mat core_low;
    cv::Mat core_high;
    cv::inRange(hsv, kCoreLowRed1, kCoreHighRed1, core_low);
    cv::inRange(hsv, kCoreLowRed2, kCoreHighRed2, core_high);
    cv::Mat crop_core_red_mask;
    cv::bitwise_or(core_low, core_high, crop_core_red_mask);

    crop_red_mask.copyTo(red_mask(crop_rect));
    crop_core_red_mask.copyTo(core_red_mask(crop_rect));
    TrimMaskToMaxRedY(red_mask);
    TrimMaskToMaxRedY(core_red_mask);

    *red_mask_out = red_mask;
    *core_red_mask_out = core_red_mask;
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

static cv::Mat RestrictMaskToRect(const cv::Mat& mask, const cv::Rect& rect)
{
    cv::Mat limited = cv::Mat::zeros(mask.size(), mask.type());
    mask(rect).copyTo(limited(rect));
    return limited;
}

static int RectIntersectionArea(const cv::Rect& a, const cv::Rect& b)
{
    const int x1 = std::max(a.x, b.x);
    const int y1 = std::max(a.y, b.y);
    const int x2 = std::min(a.x + a.width, b.x + b.width);
    const int y2 = std::min(a.y + a.height, b.y + b.height);
    if (x2 <= x1 || y2 <= y1)
    {
        return 0;
    }
    return (x2 - x1) * (y2 - y1);
}

static double HorizontalOverlapRatio(const cv::Rect& a, const cv::Rect& b)
{
    const int x1 = std::max(a.x, b.x);
    const int x2 = std::min(a.x + a.width, b.x + b.width);
    if (x2 <= x1)
    {
        return 0.0;
    }
    return static_cast<double>(x2 - x1) /
           std::max(1.0, static_cast<double>(std::min(a.width, b.width)));
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

static std::vector<cv::Point2f> RotatedBoxPointsFromRect(const cv::RotatedRect& rect)
{
    cv::Point2f points[4];
    rect.points(points);
    return OrderQuadPointsCanonical(std::vector<cv::Point2f>(points, points + 4));
}

static cv::RotatedRect NormalizeRectLongEdgeHorizontal(const cv::RotatedRect& rect)
{
    cv::RotatedRect normalized = rect;
    if (normalized.size.width < normalized.size.height)
    {
        std::swap(normalized.size.width, normalized.size.height);
        normalized.angle += 90.0f;
    }
    return normalized;
}

static bool BorrowThinStripeGeometryQuad(const cv::Mat& frame_bgr,
                                         const std::vector<cv::Point>& refined_contour,
                                         const cv::Rect& refined_box,
                                         std::vector<cv::Point2f>* borrowed_quad)
{
    if (borrowed_quad == nullptr || frame_bgr.empty() || refined_contour.empty())
    {
        return false;
    }

    const cv::RotatedRect raw_rect =
        NormalizeRectLongEdgeHorizontal(cv::minAreaRect(refined_contour));
    const double raw_width = std::max(raw_rect.size.width, 0.0f);
    const double raw_height = std::max(raw_rect.size.height, 0.0f);
    if (raw_height <= 0.0)
    {
        return false;
    }

    const double raw_aspect = raw_width / raw_height;
    if (raw_height > kGeometryBorrowMaxRawHeight || raw_aspect < kGeometryBorrowMinAspectRatio)
    {
        return false;
    }

    const int pad_x = std::max(kGeometryBorrowMinPadX, refined_box.width / 4);
    const int pad_top = std::max(kGeometryBorrowMinPadTop, refined_box.height * 2);
    const int pad_bottom = std::max(kGeometryBorrowMinPadBottom, refined_box.height / 2);
    cv::Rect local_rect(
        refined_box.x - pad_x,
        refined_box.y - pad_top,
        refined_box.width + pad_x * 2,
        refined_box.height + pad_top + pad_bottom);
    if (!ClampRectToImage(local_rect, frame_bgr.cols, frame_bgr.rows, &local_rect))
    {
        return false;
    }

    cv::Mat hsv;
    cv::cvtColor(frame_bgr(local_rect), hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask_low;
    cv::Mat mask_high;
    cv::inRange(hsv, kLowRed1, kHighRed1, mask_low);
    cv::inRange(hsv, kLowRed2, kHighRed2, mask_high);
    cv::Mat geometry_mask;
    cv::bitwise_or(mask_low, mask_high, geometry_mask);
    geometry_mask = PostprocessMask(geometry_mask);
    if (geometry_mask.empty())
    {
        return false;
    }

    std::vector<std::vector<cv::Point>> local_contours(1);
    local_contours[0].reserve(refined_contour.size());
    for (size_t i = 0; i < refined_contour.size(); ++i)
    {
        local_contours[0].push_back(refined_contour[i] - local_rect.tl());
    }
    cv::Mat raw_local_mask = cv::Mat::zeros(local_rect.height, local_rect.width, CV_8UC1);
    cv::drawContours(raw_local_mask, local_contours, -1, cv::Scalar(255), cv::FILLED);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int component_count =
        cv::connectedComponentsWithStats(geometry_mask, labels, stats, centroids, 8, CV_32S);
    if (component_count <= 1)
    {
        return false;
    }

    int best_component_id = -1;
    int best_component_area = -1;
    for (int component_id = 1; component_id < component_count; ++component_id)
    {
        const cv::Mat component_mask = labels == component_id;
        const int overlap = cv::countNonZero(component_mask & (raw_local_mask > 0));
        if (overlap <= 0)
        {
            continue;
        }

        const int component_area = stats.at<int>(component_id, cv::CC_STAT_AREA);
        if (component_area > best_component_area)
        {
            best_component_area = component_area;
            best_component_id = component_id;
        }
    }

    if (best_component_id < 0)
    {
        return false;
    }

    cv::Mat best_component_mask = cv::Mat::zeros(local_rect.height, local_rect.width, CV_8UC1);
    best_component_mask.setTo(cv::Scalar(255), labels == best_component_id);
    std::vector<std::vector<cv::Point>> component_contours;
    cv::findContours(best_component_mask, component_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (component_contours.empty())
    {
        return false;
    }

    std::vector<cv::Point> geometry_contour =
        *std::max_element(component_contours.begin(), component_contours.end(),
                          [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                              return cv::contourArea(a) < cv::contourArea(b);
                          });
    for (size_t i = 0; i < geometry_contour.size(); ++i)
    {
        geometry_contour[i] += local_rect.tl();
    }

    const cv::RotatedRect geometry_rect =
        NormalizeRectLongEdgeHorizontal(cv::minAreaRect(geometry_contour));
    if (geometry_rect.size.height < raw_rect.size.height * kGeometryBorrowMinHeightRatio)
    {
        return false;
    }

    if (std::abs(geometry_rect.angle - raw_rect.angle) < kGeometryBorrowMinAngleDelta)
    {
        return false;
    }

    std::vector<cv::Point2f> geometry_quad = RotatedBoxPointsFromContour(geometry_contour);
    if (geometry_quad.size() != 4)
    {
        return false;
    }

    const cv::Point2f& top_left = geometry_quad[0];
    const cv::Point2f& top_right = geometry_quad[1];
    const cv::Point2f& bottom_right = geometry_quad[2];
    const cv::Point2f& bottom_left = geometry_quad[3];
    const double avg_height = 0.5 * (
        SegmentLength(top_left, bottom_left) +
        SegmentLength(top_right, bottom_right));
    if (avg_height <= 1e-6)
    {
        return false;
    }

    const float top_alpha = static_cast<float>(std::max(
        0.0,
        std::min(1.0, 1.0 - raw_rect.size.height / avg_height)));
    const cv::Point2f adjusted_top_left =
        top_left + (bottom_left - top_left) * top_alpha;
    const cv::Point2f adjusted_top_right =
        top_right + (bottom_right - top_right) * top_alpha;

    borrowed_quad->clear();
    borrowed_quad->push_back(adjusted_top_left);
    borrowed_quad->push_back(adjusted_top_right);
    borrowed_quad->push_back(bottom_right);
    borrowed_quad->push_back(bottom_left);
    return true;
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

static cv::Mat QuadPolygonMask(const cv::Size& shape, const std::vector<cv::Point2f>& quad)
{
    cv::Mat mask = cv::Mat::zeros(shape, CV_8UC1);
    std::vector<cv::Point> polygon;
    polygon.reserve(quad.size());
    for (size_t i = 0; i < quad.size(); ++i)
    {
        polygon.push_back(cv::Point(
            static_cast<int>(std::lround(quad[i].x)),
            static_cast<int>(std::lround(quad[i].y))));
    }
    cv::fillConvexPoly(mask, polygon, cv::Scalar(255));
    return mask;
}

static float QuadFillRatio(const cv::Mat& mask, const std::vector<cv::Point2f>& quad)
{
    cv::Mat polygon_mask = QuadPolygonMask(mask.size(), quad);
    if (kQuadFillErodePixels > 0)
    {
        const int kernel_size = kQuadFillErodePixels * 2 + 1;
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
        cv::Mat eroded;
        cv::erode(polygon_mask, eroded, kernel);
        if (cv::countNonZero(eroded) > 0)
        {
            polygon_mask = eroded;
        }
    }

    const int area = cv::countNonZero(polygon_mask);
    if (area <= 0)
    {
        return 0.0f;
    }

    cv::Mat overlap;
    cv::bitwise_and(mask, mask, overlap, polygon_mask);
    return static_cast<float>(cv::countNonZero(overlap)) / static_cast<float>(area);
}

static QuadFillMetrics QuadRedFillMetrics(const std::vector<cv::Point2f>& quad,
                                          const cv::Mat& red_mask,
                                          const cv::Mat& core_red_mask)
{
    QuadFillMetrics metrics;
    metrics.red_fill = QuadFillRatio(red_mask, quad);
    metrics.core_fill = QuadFillRatio(core_red_mask, quad);
    metrics.valid =
        metrics.red_fill >= static_cast<float>(kMinQuadRedFill) &&
        metrics.core_fill >= static_cast<float>(kMinQuadCoreFill);
    metrics.reason = metrics.valid ? "ok" : "quad_fill_reject";
    return metrics;
}

static bool RectBottomWithinMaxRedY(const cv::Rect& rect)
{
    return rect.y + rect.height <= kMaxRedYExclusive;
}

static IpmQuadMetrics HorizontalIpmQuadMetrics(const std::vector<cv::Point2f>& quad,
                                               int image_width,
                                               int image_height)
{
    IpmQuadMetrics metrics;
    if (quad.size() != 4)
    {
        metrics.reason = "quad_shape_invalid";
        return metrics;
    }

    std::vector<cv::Point2f> ipm_quad;
    if (!RawQuadToFinalQuad(quad, image_width, image_height, &ipm_quad))
    {
        metrics.reason = "frame_size_mismatch";
        return metrics;
    }

    struct Edge
    {
        int index;
        cv::Point2f p1;
        cv::Point2f p2;
        cv::Point2f midpoint;
        float length;
    };

    std::vector<Edge> edges;
    edges.reserve(4);
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point2f& p1 = ipm_quad[i];
        const cv::Point2f& p2 = ipm_quad[(i + 1) % 4];
        edges.push_back({i, p1, p2, (p1 + p2) * 0.5f, SegmentLength(p1, p2)});
    }

    const float pair0_avg = 0.5f * (edges[0].length + edges[2].length);
    const float pair1_avg = 0.5f * (edges[1].length + edges[3].length);

    Edge top_edge;
    Edge bottom_edge;
    Edge left_edge;
    Edge right_edge;
    if (pair0_avg >= pair1_avg)
    {
        top_edge = (edges[0].midpoint.y <= edges[2].midpoint.y) ? edges[0] : edges[2];
        bottom_edge = (edges[0].midpoint.y > edges[2].midpoint.y) ? edges[0] : edges[2];
        left_edge = (edges[1].midpoint.x <= edges[3].midpoint.x) ? edges[1] : edges[3];
        right_edge = (edges[1].midpoint.x > edges[3].midpoint.x) ? edges[1] : edges[3];
    }
    else
    {
        top_edge = (edges[1].midpoint.y <= edges[3].midpoint.y) ? edges[1] : edges[3];
        bottom_edge = (edges[1].midpoint.y > edges[3].midpoint.y) ? edges[1] : edges[3];
        left_edge = (edges[0].midpoint.x <= edges[2].midpoint.x) ? edges[0] : edges[2];
        right_edge = (edges[0].midpoint.x > edges[2].midpoint.x) ? edges[0] : edges[2];
    }

    metrics.top_width = top_edge.length;
    metrics.bottom_width = bottom_edge.length;
    metrics.left_height = left_edge.length;
    metrics.right_height = right_edge.length;

    const double ratio = kIpmMinWidthHeightRatio;
    metrics.valid =
        metrics.top_width >= metrics.left_height * ratio &&
        metrics.top_width >= metrics.right_height * ratio &&
        metrics.bottom_width >= metrics.left_height * ratio &&
        metrics.bottom_width >= metrics.right_height * ratio;
    metrics.reason = metrics.valid ? "ok" : "ipm_ratio_reject";
    return metrics;
}

static bool BlobScore(const std::vector<cv::Point>& contour,
                      const cv::Size& frame_size,
                      const cv::Mat& red_mask,
                      const cv::Mat& core_red_mask,
                      double min_blob_fill,
                      double* out_score,
                      cv::Rect* out_box)
{
    if (out_score == nullptr || out_box == nullptr)
    {
        return false;
    }

    const double area = cv::contourArea(contour);
    if (area < kRedMinArea)
    {
        return false;
    }

    const cv::Rect rect = cv::boundingRect(contour);
    if (rect.width <= 0 || rect.height <= 0 || !RectBottomWithinMaxRedY(rect))
    {
        return false;
    }

    const double perimeter = cv::arcLength(contour, true);
    if (perimeter <= 0.0)
    {
        return false;
    }

    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.04 * perimeter, true);

    const double fill_rate = area / (static_cast<double>(rect.width) * rect.height + 1e-6);
    if (fill_rate < min_blob_fill)
    {
        return false;
    }

    const std::vector<cv::Point2f> blob_quad = RotatedBoxPointsFromContour(contour);
    const QuadFillMetrics quad_fill_metrics = QuadRedFillMetrics(blob_quad, red_mask, core_red_mask);
    if (!quad_fill_metrics.valid)
    {
        return false;
    }

    const IpmQuadMetrics ipm_metrics = HorizontalIpmQuadMetrics(blob_quad, frame_size.width, frame_size.height);
    if (!ipm_metrics.valid)
    {
        return false;
    }

    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    const double hull_area = cv::contourArea(hull);
    const double solidity = area / (hull_area + 1e-6);

    const double frame_area = static_cast<double>(frame_size.width) * frame_size.height;
    const double area_score = std::min(area / std::max(frame_area * 0.15, 1.0), 1.0);
    const double polygon_score = 1.0 - std::min(std::abs(static_cast<double>(approx.size()) - 6.0) / 8.0, 1.0);
    const double score =
        0.55 * area_score +
        0.20 * fill_rate +
        0.15 * solidity +
        0.10 * polygon_score;

    *out_score = score;
    *out_box = rect;
    return true;
}

static bool SelectBestBlobFromMask(const cv::Mat& mask,
                                   const cv::Mat& core_red_mask,
                                   const cv::Size& frame_size,
                                   int min_area,
                                   double min_blob_fill,
                                   std::vector<cv::Point>* best_contour,
                                   cv::Rect* best_box)
{
    if (best_contour == nullptr || best_box == nullptr)
    {
        return false;
    }

    cv::Mat contour_input = mask.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contour_input, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool found = false;
    double best_area = -1.0;
    int best_bottom_y = -1;
    double best_score = -1.0;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        const double area = cv::contourArea(contours[i]);
        if (area < min_area)
        {
            continue;
        }

        double score = 0.0;
        cv::Rect box;
        if (!BlobScore(contours[i], frame_size, mask, core_red_mask, min_blob_fill, &score, &box))
        {
            continue;
        }

        const int bottom_y = box.y + box.height;
        if (!found ||
            area > best_area ||
            (area == best_area && bottom_y > best_bottom_y) ||
            (area == best_area && bottom_y == best_bottom_y && score > best_score))
        {
            found = true;
            best_area = area;
            best_bottom_y = bottom_y;
            best_score = score;
            *best_contour = contours[i];
            *best_box = box;
        }
    }

    return found;
}

static bool SelectLargestLooseBlobFromMask(const cv::Mat& mask,
                                           int min_area,
                                           std::vector<cv::Point>* best_contour,
                                           cv::Rect* best_box,
                                           double* best_area)
{
    if (best_contour == nullptr || best_box == nullptr || best_area == nullptr)
    {
        return false;
    }

    cv::Mat contour_input = mask.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contour_input, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool found = false;
    double area_max = -1.0;
    int best_bottom_y = -1;
    cv::Rect chosen_box;
    std::vector<cv::Point> chosen_contour;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        const double area = cv::contourArea(contours[i]);
        if (area < min_area)
        {
            continue;
        }

        const cv::Rect box = cv::boundingRect(contours[i]);
        const int bottom_y = box.y + box.height;
        if (!found ||
            area > area_max ||
            (area == area_max && bottom_y > best_bottom_y))
        {
            found = true;
            area_max = area;
            best_bottom_y = bottom_y;
            chosen_box = box;
            chosen_contour = contours[i];
        }
    }

    if (!found)
    {
        return false;
    }

    *best_contour = chosen_contour;
    *best_box = chosen_box;
    *best_area = area_max;
    return true;
}

static bool BoxesOverlapWithPadding(const cv::Rect& a, const cv::Rect& b, int pad)
{
    const cv::Rect padded_a(a.x - pad, a.y - pad, a.width + pad * 2, a.height + pad * 2);
    return (padded_a & b).area() > 0;
}

static bool SelectLargestOverlappingLooseBlobFromMask(const cv::Mat& mask,
                                                      int min_area,
                                                      const cv::Rect& seed_box,
                                                      std::vector<cv::Point>* best_contour,
                                                      cv::Rect* best_box,
                                                      double* best_area)
{
    if (best_contour == nullptr || best_box == nullptr || best_area == nullptr)
    {
        return false;
    }

    cv::Mat contour_input = mask.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contour_input, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool found = false;
    double area_max = -1.0;
    int best_bottom_y = -1;
    cv::Rect chosen_box;
    std::vector<cv::Point> chosen_contour;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        const double area = cv::contourArea(contours[i]);
        if (area < min_area)
        {
            continue;
        }

        const cv::Rect box = cv::boundingRect(contours[i]);
        if (!BoxesOverlapWithPadding(seed_box, box, 8))
        {
            continue;
        }

        const int bottom_y = box.y + box.height;
        if (!found ||
            area > area_max ||
            (area == area_max && bottom_y > best_bottom_y))
        {
            found = true;
            area_max = area;
            best_bottom_y = bottom_y;
            chosen_box = box;
            chosen_contour = contours[i];
        }
    }

    if (!found)
    {
        return false;
    }

    *best_contour = chosen_contour;
    *best_box = chosen_box;
    *best_area = area_max;
    return true;
}

static bool ExpandTouchTopLooseBlobUpward(const cv::Mat& frame_bgr,
                                          const cv::Rect& processing_rect,
                                          const cv::Rect& seed_box,
                                          std::vector<cv::Point>* out_contour,
                                          cv::Rect* out_box,
                                          double* out_area,
                                          cv::Rect* out_expanded_rect,
                                          cv::Mat* out_expanded_red_mask,
                                          cv::Mat* out_expanded_core_red_mask)
{
    if (out_contour == nullptr || out_box == nullptr || out_area == nullptr || frame_bgr.empty())
    {
        return false;
    }

    cv::Rect expanded_rect = processing_rect;
    expanded_rect.height += expanded_rect.y;
    expanded_rect.y = 0;
    if (expanded_rect.width <= 0 || expanded_rect.height <= 0)
    {
        return false;
    }

    cv::Mat expanded_red_mask;
    cv::Mat expanded_core_red_mask;
    BuildRedMasks(frame_bgr, &expanded_rect, &expanded_red_mask, &expanded_core_red_mask);
    const bool found = SelectLargestOverlappingLooseBlobFromMask(
        expanded_red_mask,
        kLooseRedMinArea,
        seed_box,
        out_contour,
        out_box,
        out_area);
    if (!found)
    {
        return false;
    }

    if (out_expanded_rect != nullptr)
    {
        *out_expanded_rect = expanded_rect;
    }
    if (out_expanded_red_mask != nullptr)
    {
        *out_expanded_red_mask = expanded_red_mask;
    }
    if (out_expanded_core_red_mask != nullptr)
    {
        *out_expanded_core_red_mask = expanded_core_red_mask;
    }
    return true;
}

static bool RefineBlobContourWithCore(const std::vector<cv::Point>& candidate_contour,
                                      const cv::Mat& core_red_mask,
                                      std::vector<cv::Point>* refined_contour,
                                      cv::Rect* refined_box)
{
    if (refined_contour == nullptr || refined_box == nullptr || candidate_contour.empty())
    {
        return false;
    }

    const cv::Rect rect = cv::boundingRect(candidate_contour);
    if (rect.width <= 0 || rect.height <= 0)
    {
        return false;
    }

    cv::Mat region = core_red_mask(rect).clone();
    if (region.empty())
    {
        *refined_contour = candidate_contour;
        *refined_box = rect;
        return true;
    }

    std::vector<std::vector<cv::Point>> local_contours(1);
    local_contours[0].reserve(candidate_contour.size());
    for (size_t i = 0; i < candidate_contour.size(); ++i)
    {
        local_contours[0].push_back(candidate_contour[i] - rect.tl());
    }

    cv::Mat contour_mask = cv::Mat::zeros(rect.height, rect.width, CV_8UC1);
    cv::drawContours(contour_mask, local_contours, -1, cv::Scalar(255), cv::FILLED);
    cv::bitwise_and(region, contour_mask, region);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int component_count =
        cv::connectedComponentsWithStats(region, labels, stats, centroids, 8, CV_32S);
    if (component_count <= 1)
    {
        *refined_contour = candidate_contour;
        *refined_box = rect;
        return true;
    }

    const double center_x = rect.width / 2.0;
    const double center_y = rect.height / 2.0;
    int best_component_id = -1;
    double best_component_score = -1.0;

    for (int component_id = 1; component_id < component_count; ++component_id)
    {
        const int comp_x = stats.at<int>(component_id, cv::CC_STAT_LEFT);
        const int comp_y = stats.at<int>(component_id, cv::CC_STAT_TOP);
        const int comp_w = stats.at<int>(component_id, cv::CC_STAT_WIDTH);
        const int comp_h = stats.at<int>(component_id, cv::CC_STAT_HEIGHT);
        const int comp_area = stats.at<int>(component_id, cv::CC_STAT_AREA);
        if (comp_area < kCoreRedMinArea || comp_w <= 0 || comp_h <= 0)
        {
            continue;
        }

        const double component_fill =
            static_cast<double>(comp_area) / (static_cast<double>(comp_w) * comp_h + 1e-6);
        if (component_fill < kCoreMinBlobFill)
        {
            continue;
        }

        const double comp_center_x = comp_x + comp_w / 2.0;
        const double comp_center_y = comp_y + comp_h / 2.0;
        const double center_distance =
            std::hypot(comp_center_x - center_x, comp_center_y - center_y);
        const double score = comp_area - center_distance * 4.0;
        if (score > best_component_score)
        {
            best_component_score = score;
            best_component_id = component_id;
        }
    }

    if (best_component_id < 0)
    {
        *refined_contour = candidate_contour;
        *refined_box = rect;
        return true;
    }

    cv::Mat best_component_mask = cv::Mat::zeros(rect.height, rect.width, CV_8UC1);
    best_component_mask.setTo(cv::Scalar(255), labels == best_component_id);

    std::vector<std::vector<cv::Point>> component_contours;
    cv::findContours(
        best_component_mask,
        component_contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE);
    if (component_contours.empty())
    {
        *refined_contour = candidate_contour;
        *refined_box = rect;
        return true;
    }

    size_t best_index = 0;
    double best_area = cv::contourArea(component_contours[0]);
    for (size_t i = 1; i < component_contours.size(); ++i)
    {
        const double contour_area = cv::contourArea(component_contours[i]);
        if (contour_area > best_area)
        {
            best_area = contour_area;
            best_index = i;
        }
    }

    refined_contour->clear();
    refined_contour->reserve(component_contours[best_index].size());
    for (size_t i = 0; i < component_contours[best_index].size(); ++i)
    {
        refined_contour->push_back(component_contours[best_index][i] + rect.tl());
    }
    *refined_box = cv::boundingRect(*refined_contour);
    return true;
}

static bool FindSupportingRedRectBelow(const cv::Rect& selected_rect,
                                       const cv::Mat& red_mask,
                                       cv::Rect* out_rect)
{
    if (out_rect == nullptr ||
        selected_rect.width <= 0 ||
        selected_rect.height <= 0)
    {
        return false;
    }

    const int sw = selected_rect.width;
    const int sh = selected_rect.height;
    if (sh > kStripRejectMaxHeight ||
        static_cast<double>(sw) / std::max(1.0, static_cast<double>(sh)) < kStripRejectMinAspectRatio)
    {
        return false;
    }

    cv::Mat contour_input = red_mask.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contour_input, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double selected_box_area = static_cast<double>(sw) * sh;
    const int selected_bottom = selected_rect.y + selected_rect.height;
    bool found = false;
    double best_score = -1.0;
    cv::Rect best_rect;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        const double contour_area = cv::contourArea(contours[i]);
        if (contour_area < selected_box_area * kStripSupportMinAreaRatio)
        {
            continue;
        }

        const cv::Rect rect = cv::boundingRect(contours[i]);
        if (RectIntersectionArea(rect, selected_rect) >= static_cast<int>(selected_box_area * 0.7))
        {
            continue;
        }
        if (rect.width <= 0 || rect.height <= 0)
        {
            continue;
        }
        if (rect.height < sh * kStripSupportMinHeightRatio)
        {
            continue;
        }
        if (rect.width < sw * kStripSupportMinWidthRatio)
        {
            continue;
        }
        if (HorizontalOverlapRatio(rect, selected_rect) < kStripSupportMinXOverlapRatio)
        {
            continue;
        }
        if (rect.y > selected_bottom + kStripSupportMaxTopGap)
        {
            continue;
        }

        const int below_pixels = rect.y + rect.height - selected_bottom;
        if (below_pixels < std::max(kStripSupportMinBelowPixels, static_cast<int>(std::lround(sh * 0.35))))
        {
            continue;
        }

        const double score = contour_area + below_pixels * 10.0 + rect.width;
        if (!found || score > best_score)
        {
            found = true;
            best_score = score;
            best_rect = rect;
        }
    }

    if (found)
    {
        *out_rect = best_rect;
    }
    return found;
}

static std::string DiagnoseLooseBlobRejectStage(const std::vector<cv::Point>& contour,
                                                const cv::Mat& red_mask,
                                                const cv::Mat& core_red_mask,
                                                const cv::Size& frame_size)
{
    if (contour.empty())
    {
        return "no_valid_loose_blob";
    }

    const double area = cv::contourArea(contour);
    if (area < kLooseRedMinArea)
    {
        return "red_min_area_reject";
    }

    const cv::Rect rect = cv::boundingRect(contour);
    if (rect.width <= 0 || rect.height <= 0)
    {
        return "rect_invalid";
    }

    if (!RectBottomWithinMaxRedY(rect))
    {
        return "y_limit_reject";
    }

    const double fill_rate = area / (static_cast<double>(rect.width) * rect.height + 1e-6);
    if (fill_rate < kMinBlobFill)
    {
        return "fill_rate_reject";
    }

    std::vector<cv::Point> refined_contour;
    cv::Rect refined_box;
    if (!RefineBlobContourWithCore(contour, core_red_mask, &refined_contour, &refined_box))
    {
        return "refine_reject";
    }

    cv::Rect support_rect;
    if (FindSupportingRedRectBelow(refined_box, red_mask, &support_rect))
    {
        return "strip_reject";
    }

    if (!RectBottomWithinMaxRedY(refined_box))
    {
        return "y_limit_reject";
    }

    const std::vector<cv::Point2f> blob_quad = RotatedBoxPointsFromContour(refined_contour);
    const QuadFillMetrics quad_fill_metrics = QuadRedFillMetrics(blob_quad, red_mask, core_red_mask);
    if (!quad_fill_metrics.valid)
    {
        return quad_fill_metrics.reason;
    }

    const IpmQuadMetrics ipm_metrics = HorizontalIpmQuadMetrics(blob_quad, frame_size.width, frame_size.height);
    if (!ipm_metrics.valid)
    {
        return ipm_metrics.reason;
    }

    const BuildRoiQuadResult build_result =
        BuildRoiQuadFromBlobQuad(blob_quad, frame_size.width, frame_size.height, DefaultRoiMethod());
    if (!build_result.valid)
    {
        return build_result.status;
    }

    return "rotated_roi";
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

static cv::Mat BuildTaskStrictRedMaskRect(const cv::Mat& frame_bgr, const cv::Rect& rect)
{
    cv::Mat full_mask = cv::Mat::zeros(frame_bgr.size(), CV_8UC1);
    cv::Rect clamped;
    if (frame_bgr.empty() || !ClampRectToImage(rect, frame_bgr.cols, frame_bgr.rows, &clamped))
    {
        return full_mask;
    }

    TaskWhiteReferenceStats white_ref_stats;
    ComputeTaskWhiteReferenceStats(frame_bgr, &white_ref_stats);
    const TaskStrictRedThresholds red_thresholds =
        BuildTaskStrictRedThresholds(white_ref_stats);

    for (int y = clamped.y; y < clamped.y + clamped.height; ++y)
    {
        const cv::Vec3b* row_ptr = frame_bgr.ptr<cv::Vec3b>(y);
        unsigned char* mask_ptr = full_mask.ptr<unsigned char>(y);
        for (int x = clamped.x; x < clamped.x + clamped.width; ++x)
        {
            if (IsTaskStrictRedPixel(row_ptr[x], red_thresholds))
            {
                mask_ptr[x] = 255;
            }
        }
    }
    return full_mask;
}

static bool ComputeTaskRedXRange(const cv::Mat& mask, int y_min, int y_max, int* out_x_min, int* out_x_max)
{
    if (out_x_min == nullptr || out_x_max == nullptr || mask.empty())
    {
        return false;
    }
    const int row0 = std::max(0, y_min);
    const int row1 = std::min(mask.rows, y_max);
    if (row1 <= row0)
    {
        return false;
    }
    int min_x = std::numeric_limits<int>::max();
    int max_x = std::numeric_limits<int>::min();
    for (int y = row0; y < row1; ++y)
    {
        const unsigned char* row_ptr = mask.ptr<unsigned char>(y);
        for (int x = 0; x < mask.cols; ++x)
        {
            if (row_ptr[x] > 0)
            {
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
            }
        }
    }
    if (min_x == std::numeric_limits<int>::max() || max_x < min_x)
    {
        return false;
    }
    *out_x_min = min_x;
    *out_x_max = max_x;
    return true;
}

static cv::Rect BuildTaskSearchRect(const cv::Mat& frame_bgr,
                                    int* out_reference_x_min,
                                    int* out_reference_x_max,
                                    bool* out_has_reference_x_range,
                                    int* out_reference_span_count,
                                    std::string* out_reference_range_source)
{
    const int image_width = frame_bgr.cols;
    const int image_height = frame_bgr.rows;
    cv::Rect base_y_rect(0, kTaskSearchYMin, image_width, std::max(1, kTaskSearchYMax - kTaskSearchYMin));
    ClampRectToImage(base_y_rect, image_width, image_height, &base_y_rect);

    int reference_x_min = 0;
    int reference_x_max = 0;
    int reference_span_count = 0;
    std::string reference_range_source = "fallback";
    const bool has_reference_x_range = ComputeReferenceEnvelopeXRangeOnRow(
        frame_bgr,
        &reference_x_min,
        &reference_x_max,
        &reference_span_count,
        &reference_range_source);
    if (out_has_reference_x_range != nullptr)
    {
        *out_has_reference_x_range = has_reference_x_range;
    }
    if (out_reference_range_source != nullptr)
    {
        *out_reference_range_source = reference_range_source;
    }
    if (has_reference_x_range && out_reference_x_min != nullptr && out_reference_x_max != nullptr)
    {
        *out_reference_x_min = reference_x_min;
        *out_reference_x_max = reference_x_max;
    }
    if (has_reference_x_range && out_reference_span_count != nullptr)
    {
        *out_reference_span_count = reference_span_count;
    }

    const cv::Mat base_mask = BuildTaskStrictRedMaskRect(frame_bgr, base_y_rect);
    int red_x_min = 0;
    int red_x_max = 0;
    const bool has_red_x_range = ComputeTaskRedXRange(base_mask, kTaskSearchYMin, kTaskSearchYMax, &red_x_min, &red_x_max);

    int x0 = 0;
    int x1 = image_width - 1;
    if (has_reference_x_range || has_red_x_range)
    {
        x0 = image_width - 1;
        x1 = 0;
        if (has_reference_x_range)
        {
            x0 = std::min(x0, reference_x_min);
            x1 = std::max(x1, reference_x_max);
        }
        if (has_red_x_range)
        {
            x0 = std::min(x0, red_x_min);
            x1 = std::max(x1, red_x_max);
        }
    }

    cv::Rect task_rect(x0, kTaskSearchYMin, std::max(1, x1 - x0 + 1), std::max(1, kTaskSearchYMax - kTaskSearchYMin));
    ClampRectToImage(task_rect, image_width, image_height, &task_rect);
    return task_rect;
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
    std::vector<cv::Point> best_contour;
    cv::Rect best_box;

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

        cv::Mat component_mask = cv::Mat::zeros(crop.size(), CV_8UC1);
        component_mask.setTo(255, labels == label);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(component_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty())
        {
            continue;
        }

        auto contour_it = std::max_element(
            contours.begin(),
            contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return std::fabs(cv::contourArea(a)) < std::fabs(cv::contourArea(b));
            });
        std::vector<cv::Point> contour = *contour_it;
        for (cv::Point& pt : contour)
        {
            pt.x += rect.x;
            pt.y += rect.y;
        }
        const cv::Rect box(rect.x + left, rect.y + top, width, height);
        const int bottom_y = box.y + box.height;
        if (!found || bottom_y > best_bottom_y || (bottom_y == best_bottom_y && area > best_area))
        {
            found = true;
            best_bottom_y = bottom_y;
            best_area = static_cast<double>(area);
            best_contour = contour;
            best_box = box;
        }
    }

    if (!found)
    {
        return false;
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


RoiExtractionResult ExtractRotatedRoi(const cv::Mat& frame_bgr,
                                      int output_size,
                                      RoiMethod roi_method)
{
    RoiExtractionResult result;
    result.roi_method = roi_method;

    if (frame_bgr.empty())
    {
        return result;
    }

    const int image_width = frame_bgr.cols;
    const int image_height = frame_bgr.rows;

    int reference_x_min = 0;
    int reference_x_max = 0;
    int reference_span_count = 0;
    bool has_reference_x_range = false;
    std::string reference_range_source = "fallback";
    const cv::Rect fallback_search_rect = BuildTaskSearchRect(
        frame_bgr,
        &reference_x_min,
        &reference_x_max,
        &has_reference_x_range,
        &reference_span_count,
        &reference_range_source);
    result.reference_range_source = reference_range_source;
    if (has_reference_x_range)
    {
        result.has_reference_x_range = true;
        result.reference_x_min = reference_x_min;
        result.reference_x_max = reference_x_max;
        result.merged_reference_span_count = reference_span_count;
    }

    TaskTrackBoundaryState track_state;
    const bool has_track_boundaries = BuildTaskTrackBoundaryState(
        frame_bgr,
        reference_x_min,
        reference_x_max,
        has_reference_x_range,
        &track_state);
    result.track_left_boundary =
        BuildTrackBoundaryDisplayPoints(track_state.seed_left_x, track_state.seed_y, track_state.left_points);
    result.track_right_boundary =
        BuildTrackBoundaryDisplayPoints(track_state.seed_right_x, track_state.seed_y, track_state.right_points);
    result.has_track_left_boundary = !result.track_left_boundary.empty();
    result.has_track_right_boundary = !result.track_right_boundary.empty();
    if (has_track_boundaries)
    {
        result.track_region_polygon = BuildTrackRegionPolygon(track_state);
        result.has_track_region_polygon = (result.track_region_polygon.size() >= 4);
        result.search_rect = BuildTaskTrackSearchRect(track_state, image_width, image_height);
    }
    else
    {
        result.search_rect = fallback_search_rect;
    }
    result.has_search_rect = true;

    cv::Mat red_mask = BuildTaskStrictRedMaskRect(frame_bgr, result.search_rect);
    int expand_steps = 0;
    while (expand_steps < kTaskEdgeExpandMaxSteps)
    {
        bool touch_left = false;
        bool touch_top = false;
        bool touch_right = false;
        bool touch_bottom = false;
        TaskTouchedSides(red_mask, result.search_rect, &touch_left, &touch_top, &touch_right, &touch_bottom);
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
        red_mask = BuildTaskStrictRedMaskRect(frame_bgr, result.search_rect);
        ++expand_steps;
    }

    std::vector<cv::Point> candidate_contour;
    cv::Rect candidate_box;
    double candidate_area = 0.0;
    if (!ChooseLowestTaskRedBand(red_mask, result.search_rect, &candidate_contour, &candidate_box, &candidate_area))
    {
        result.status = "miss";
        return result;
    }

    result.has_loose_blob_box = true;
    result.loose_blob_box = candidate_box;
    result.has_loose_blob_area = true;
    result.loose_blob_area = candidate_area;
    result.has_blob_box = true;
    result.blob_box = candidate_box;
    result.has_blob_area = true;
    result.blob_area = candidate_area;
    result.has_candidate_area = true;
    result.candidate_area = candidate_area;
    result.has_candidate_center = true;
    result.candidate_center_x = static_cast<float>(candidate_box.x + candidate_box.width * 0.5f);
    result.candidate_center_y = static_cast<float>(candidate_box.y + candidate_box.height * 0.5f);
    result.has_candidate_size = true;
    result.candidate_width = candidate_box.width;
    result.candidate_height = candidate_box.height;

    TaskTrackClassification track_classification;
    if (has_track_boundaries)
    {
        track_classification = ClassifyTaskCandidateByTrackBoundary(track_state, candidate_box);
    }

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

    if (track_classification.type == TaskTrackCandidateType::ROADBLOCK)
    {
        result.target_type = "roadblock";
        result.status = "roadblock";
        return result;
    }
    if (track_classification.type != TaskTrackCandidateType::MARKER)
    {
        result.status = has_track_boundaries ? "track_classify_miss" : "track_boundary_miss";
        return result;
    }
    result.target_type = "marker";

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
        result.status = build_result.status;
        return result;
    }

    result.roi_quad = build_result.roi_quad;
    result.roi_bgr = WarpRoiFromQuad(frame_bgr, result.roi_quad, output_size);
    if (result.roi_bgr.empty())
    {
        result.status = "ipm_backproject_invalid";
        return result;
    }

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

