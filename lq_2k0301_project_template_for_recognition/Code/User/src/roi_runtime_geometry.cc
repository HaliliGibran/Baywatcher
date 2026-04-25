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
constexpr int kIpmFrameWidth = 640;
constexpr int kIpmFrameHeight = 480;
constexpr double kIpmMinWidthHeightRatio = 1.15;
constexpr double kMinRoiEdgeLength = 8.0;
constexpr double kMinBackprojectedQuadArea = 24.0;

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
constexpr int kWhiteMinSpanWidth = 120;

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
    0.654444770734335, -0.560692760554098, 109.148186992145,
    0.0145082574708702, 0.126600103480591, -5.30231712603921,
    8.7399141390752e-05, -0.0018172029477988, 1.0);

constexpr double kFx = 213.30162531947;
constexpr double kFy = 213.264390305945;
constexpr double kSkew = 0.0;
constexpr double kCx = 311.94214815483;
constexpr double kCy = 201.628620112633;
constexpr double kK1 = -0.0585577404122723;
constexpr double kK2 = -0.00511005281610996;
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

static BuildRoiQuadResult BuildRoiQuadFromBlobQuad(const std::vector<cv::Point2f>& blob_quad,
                                                   int image_width,
                                                   int image_height,
                                                   RoiMethod roi_method);

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

static bool ComputeWhiteXRangeOnRow(const cv::Mat& frame_bgr,
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
    const cv::Mat row_bgr = frame_bgr.row(row_y).clone();
    cv::Mat row_hsv;
    cv::cvtColor(row_bgr, row_hsv, cv::COLOR_BGR2HSV);

    int merged_start = -1;
    int merged_end = -1;
    int merged_count = 0;
    int start = -1;
    for (int x = 0; x < image_width; ++x)
    {
        const cv::Vec3b hsv = row_hsv.at<cv::Vec3b>(0, x);
        const bool is_white =
            hsv[1] <= static_cast<unsigned char>(kWhiteMaxSaturation) &&
            hsv[2] >= static_cast<unsigned char>(kWhiteMinValue);
        if (is_white && start < 0)
        {
            start = x;
        }
        else if (!is_white && start >= 0)
        {
            const int end = x - 1;
            if (end - start + 1 >= kWhiteMinSpanWidth)
            {
                if (merged_count == 0)
                {
                    merged_start = start;
                    merged_end = end;
                }
                else
                {
                    merged_start = std::min(merged_start, start);
                    merged_end = std::max(merged_end, end);
                }
                ++merged_count;
            }
            start = -1;
        }
    }
    if (start >= 0)
    {
        const int end = image_width - 1;
        if (end - start + 1 >= kWhiteMinSpanWidth)
        {
            if (merged_count == 0)
            {
                merged_start = start;
                merged_end = end;
            }
            else
            {
                merged_start = std::min(merged_start, start);
                merged_end = std::max(merged_end, end);
            }
            ++merged_count;
        }
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

static cv::Rect ApplyWhiteXRangeToRect(const cv::Rect& rect,
                                       const cv::Mat& frame_bgr,
                                       int* out_x_min,
                                       int* out_x_max,
                                       std::string* out_source,
                                       int* out_span_count)
{
    int white_x_min = 0;
    int white_x_max = 0;
    int span_count = 0;
    if (!ComputeWhiteXRangeOnRow(frame_bgr, &white_x_min, &white_x_max, &span_count))
    {
        if (out_source != nullptr)
        {
            *out_source = "fallback";
        }
        return rect;
    }

    if (out_x_min != nullptr)
    {
        *out_x_min = white_x_min;
    }
    if (out_x_max != nullptr)
    {
        *out_x_max = white_x_max;
    }
    if (out_span_count != nullptr)
    {
        *out_span_count = span_count;
    }

    const int x1 = std::max(rect.x, white_x_min);
    const int x2 = std::min(rect.x + rect.width, white_x_max + 1);
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
        *out_source = "detected";
    }
    return cv::Rect(x1, rect.y, x2 - x1, rect.height);
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

static bool FinalToRaw(float xf, float yf, cv::Point2f* out_point)
{
    if (out_point == nullptr)
    {
        return false;
    }

    const cv::Vec3d vec = kFinalToUndist * cv::Vec3d(xf, yf, 1.0);
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

    out_point->x = static_cast<float>(xr);
    out_point->y = static_cast<float>(yr);
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
    if (!FinalToRaw(top_left_final.x, top_left_final.y, &top_left_raw) ||
        !FinalToRaw(top_right_final.x, top_right_final.y, &top_right_raw))
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

RoiMethod DefaultRoiMethod()
{
    return (BW_RECOG_ROI_METHOD == 0)
        ? RoiMethod::DIRECT_RED_QUAD
        : RoiMethod::IPM_SQUARE_FROM_TOP_EDGE;
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
                                      RoiMethod roi_method,
                                      const cv::Rect* search_rect)
{
    RoiExtractionResult result;
    result.roi_method = roi_method;

    if (frame_bgr.empty())
    {
        return result;
    }

    const int image_height = frame_bgr.rows;
    const int image_width = frame_bgr.cols;

    cv::Rect base_search_rect(0, 0, image_width, image_height);
    cv::Rect clamped_search_rect;
    if (search_rect != nullptr && ClampRectToImage(*search_rect, image_width, image_height, &clamped_search_rect))
    {
        base_search_rect = clamped_search_rect;
    }

    int white_x_min = 0;
    int white_x_max = 0;
    result.search_rect = ApplyWhiteXRangeToRect(
        base_search_rect,
        frame_bgr,
        &white_x_min,
        &white_x_max,
        &result.white_range_source,
        &result.merged_white_span_count);
    result.has_search_rect = true;
    if (result.white_range_source == "detected")
    {
        result.has_white_x_range = true;
        result.white_x_min = white_x_min;
        result.white_x_max = white_x_max;
    }

    cv::Mat full_red_mask;
    cv::Mat full_core_red_mask;
    BuildRedMasks(frame_bgr, &result.search_rect, &full_red_mask, &full_core_red_mask);

    std::vector<cv::Point> prewhite_contour;
    cv::Rect prewhite_box;
    double prewhite_area = 0.0;
    if (SelectLargestLooseBlobFromMask(
            full_red_mask,
            kLooseRedMinArea,
            &prewhite_contour,
            &prewhite_box,
            &prewhite_area))
    {
        result.has_prewhite_max_red_contour_area = true;
        result.prewhite_max_red_contour_area = prewhite_area;
    }

    cv::Mat red_mask = full_red_mask;
    cv::Mat core_red_mask = full_core_red_mask;

    std::vector<cv::Point> max_red_contour;
    cv::Rect max_red_box;
    double max_red_area = 0.0;
    if (SelectLargestLooseBlobFromMask(
            red_mask,
            kLooseRedMinArea,
            &max_red_contour,
            &max_red_box,
            &max_red_area))
    {
        result.has_max_red_contour_area = true;
        result.max_red_contour_area = max_red_area;
        result.has_max_red_contour_box = true;
        result.max_red_contour_box = max_red_box;
        result.max_red_reject_stage =
            DiagnoseLooseBlobRejectStage(max_red_contour, red_mask, core_red_mask, frame_bgr.size());
    }
    else if (result.has_prewhite_max_red_contour_area)
    {
        result.max_red_reject_stage = "dynamic_search_reject";
    }

    // 当前主链已直接在动态白条宽度内构建红色掩膜，white_crop 仅保留为调试字段；
    // 不再依赖“全宽预筛 vs 动态裁切后”差异去判定是否被白边裁掉。
    result.white_crop_clipped = false;

    std::vector<cv::Point> candidate_contour;
    cv::Rect candidate_box;
    double candidate_area = 0.0;
    if (!SelectLargestLooseBlobFromMask(
            red_mask,
            kLooseRedMinArea,
            &candidate_contour,
            &candidate_box,
            &candidate_area))
    {
        return result;
    }

    result.has_loose_blob_area = true;
    result.loose_blob_area = candidate_area;
    result.has_loose_blob_box = true;
    result.loose_blob_box = candidate_box;
    result.touches_search_top = (candidate_box.y <= BW_RECOG_TRIGGER_SEARCH_Y_MIN);
    const std::vector<cv::Point2f> loose_blob_quad =
        RotatedBoxPointsFromContour(candidate_contour);
    const IpmQuadMetrics loose_ipm_metrics =
        HorizontalIpmQuadMetrics(loose_blob_quad, image_width, image_height);
    result.loose_ipm_valid = loose_ipm_metrics.valid;
    result.loose_ipm_reason = loose_ipm_metrics.reason;

    std::vector<cv::Point> refined_contour;
    cv::Rect refined_box;
    if (!RefineBlobContourWithCore(candidate_contour, core_red_mask, &refined_contour, &refined_box))
    {
        return result;
    }

    result.has_blob_box = true;
    result.blob_box = refined_box;
    result.blob_area = std::fabs(cv::contourArea(refined_contour));
    result.blob_quad = RotatedBoxPointsFromContour(refined_contour);

    cv::Rect support_rect;
    if (FindSupportingRedRectBelow(refined_box, red_mask, &support_rect))
    {
        result.has_support_rect = true;
        result.support_rect = support_rect;
        result.status = "strip_reject";
        return result;
    }

    if (!RectBottomWithinMaxRedY(refined_box))
    {
        result.status = "y_limit_reject";
        return result;
    }

    const QuadFillMetrics quad_fill_metrics =
        QuadRedFillMetrics(result.blob_quad, red_mask, core_red_mask);
    result.has_quad_red_fill = true;
    result.quad_red_fill = quad_fill_metrics.red_fill;
    result.has_quad_core_fill = true;
    result.quad_core_fill = quad_fill_metrics.core_fill;
    if (!quad_fill_metrics.valid)
    {
        result.status = "quad_fill_reject";
        return result;
    }

    const IpmQuadMetrics ipm_metrics =
        HorizontalIpmQuadMetrics(result.blob_quad, image_width, image_height);
    result.has_ipm_top_width = true;
    result.ipm_top_width = ipm_metrics.top_width;
    result.has_ipm_bottom_width = true;
    result.ipm_bottom_width = ipm_metrics.bottom_width;
    result.has_ipm_left_height = true;
    result.ipm_left_height = ipm_metrics.left_height;
    result.has_ipm_right_height = true;
    result.ipm_right_height = ipm_metrics.right_height;
    result.ipm_reason = ipm_metrics.reason;
    if (!ipm_metrics.valid)
    {
        result.status = "ipm_reject";
        return result;
    }

    const BuildRoiQuadResult build_result =
        BuildRoiQuadFromBlobQuad(result.blob_quad, image_width, image_height, roi_method);
    result.blob_quad_final = build_result.blob_quad_final;
    result.roi_quad_final = build_result.roi_quad_final;
    if (build_result.has_raw_height_ratio)
    {
        result.has_ipm_backproject_height_ratio = true;
        result.ipm_backproject_height_ratio = build_result.raw_height_ratio;
    }
    if (!build_result.ipm_reason.empty())
    {
        result.ipm_reason = build_result.ipm_reason;
    }

    if (!build_result.valid)
    {
        result.status = build_result.status;
        return result;
    }

    result.roi_quad = build_result.roi_quad;
    result.roi_bgr = WarpRoiFromQuad(frame_bgr, result.roi_quad, output_size);
    if (result.roi_bgr.empty())
    {
        result.status = (roi_method == RoiMethod::IPM_SQUARE_FROM_TOP_EDGE)
            ? "ipm_backproject_invalid"
            : "blob_only";
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
    if (result.has_white_x_range)
    {
        cv::line(
            image_bgr,
            cv::Point(result.white_x_min, 0),
            cv::Point(result.white_x_min, image_bgr.rows - 1),
            cv::Scalar(0, 255, 0),
            1,
            cv::LINE_AA);
        cv::line(
            image_bgr,
            cv::Point(result.white_x_max, 0),
            cv::Point(result.white_x_max, image_bgr.rows - 1),
              cv::Scalar(0, 255, 0),
              1,
              cv::LINE_AA);
        if (result.merged_white_span_count > 1)
        {
            std::ostringstream span_text;
            span_text << "white_spans=" << result.merged_white_span_count;
            cv::putText(image_bgr, span_text.str(), cv::Point(16, 84),
                        cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }
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
