#include "recognition_white_reference.h"

#include "common.h"

#include <algorithm>
#include <cmath>

namespace {

struct NormalizeState
{
    bool initialized = false;
    int frames_until_stats_update = 0;
    RecognitionWhiteReferenceStats raw_stats;
    RecognitionWhiteReferenceStats adjusted_stats;
    RecognitionWhiteReferenceGains gains;
};

static float clamp_float(float value, float min_value, float max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

static unsigned char clamp_channel(float value)
{
    return cv::saturate_cast<unsigned char>(std::lround(value));
}

static NormalizeState& State()
{
    static NormalizeState state;
    return state;
}

static bool IsWhiteReferenceSeed(const cv::Vec3b& bgr, const cv::Vec3b& hsv)
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

static bool ComputeRawStats(const cv::Mat& frame_bgr, RecognitionWhiteReferenceStats* out_stats)
{
    if (out_stats == nullptr)
    {
        return false;
    }
    *out_stats = RecognitionWhiteReferenceStats();
    if (frame_bgr.empty())
    {
        return false;
    }

    const int row_y = std::max(0, std::min(BW_RECOG_WHITE_REFERENCE_ROW_Y, frame_bgr.rows - 1));
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
            if (!IsWhiteReferenceSeed(bgr_row[x], hsv_row[x]))
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

static RecognitionWhiteReferenceGains BuildTargetGains(const RecognitionWhiteReferenceStats& stats)
{
    RecognitionWhiteReferenceGains gains;
    if (!stats.valid)
    {
        return gains;
    }

    const float safe_luma = std::max(stats.mean_luma, 1.0f);
    const float safe_b = std::max(stats.mean_b, 1.0f);
    const float safe_g = std::max(stats.mean_g, 1.0f);
    const float safe_r = std::max(stats.mean_r, 1.0f);
    const float target_luma = std::max(1.0f, BW_RECOG_WHITE_REF_NORMALIZE_TARGET_LUMA);
    const float avg_channel = (stats.mean_b + stats.mean_g + stats.mean_r) / 3.0f;

    const float luma_gain = clamp_float(
        target_luma / safe_luma,
        BW_RECOG_WHITE_REF_NORMALIZE_LUMA_GAIN_MIN,
        BW_RECOG_WHITE_REF_NORMALIZE_LUMA_GAIN_MAX);
    const float wb_gain_b = clamp_float(
        avg_channel / safe_b,
        BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MIN,
        BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MAX);
    const float wb_gain_g = clamp_float(
        avg_channel / safe_g,
        BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MIN,
        BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MAX);
    const float wb_gain_r = clamp_float(
        avg_channel / safe_r,
        BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MIN,
        BW_RECOG_WHITE_REF_NORMALIZE_WB_GAIN_MAX);

    gains.initialized = true;
    gains.gain_b = luma_gain * wb_gain_b;
    gains.gain_g = luma_gain * wb_gain_g;
    gains.gain_r = luma_gain * wb_gain_r;
    return gains;
}

static RecognitionWhiteReferenceStats BuildAdjustedStats(const RecognitionWhiteReferenceStats& raw_stats,
                                                         const RecognitionWhiteReferenceGains& gains)
{
    RecognitionWhiteReferenceStats adjusted = raw_stats;
    if (!raw_stats.valid || !gains.initialized)
    {
        return adjusted;
    }

    adjusted.mean_b = static_cast<float>(clamp_channel(raw_stats.mean_b * gains.gain_b));
    adjusted.mean_g = static_cast<float>(clamp_channel(raw_stats.mean_g * gains.gain_g));
    adjusted.mean_r = static_cast<float>(clamp_channel(raw_stats.mean_r * gains.gain_r));
    adjusted.mean_luma =
        0.114f * adjusted.mean_b +
        0.587f * adjusted.mean_g +
        0.299f * adjusted.mean_r;
    return adjusted;
}

} // namespace

namespace recognition_white_reference {

void UpdateFromFrame(const cv::Mat& frame_bgr, bool allow_adapt)
{
#if BW_RECOG_WHITE_REF_NORMALIZE_ENABLE == 0
    (void)frame_bgr;
    (void)allow_adapt;
    NormalizeState& state = State();
    state.initialized = true;
    state.gains.initialized = true;
    state.gains.gain_b = 1.0f;
    state.gains.gain_g = 1.0f;
    state.gains.gain_r = 1.0f;
#else
    if (frame_bgr.empty())
    {
        return;
    }

    NormalizeState& state = State();
    const int update_interval = std::max(1, BW_RECOG_WHITE_REF_STATS_UPDATE_INTERVAL_FRAMES);
    const bool should_update_stats =
        allow_adapt &&
        (!state.initialized || state.frames_until_stats_update <= 0);

    RecognitionWhiteReferenceStats raw_stats;
    const bool has_stats = should_update_stats ? ComputeRawStats(frame_bgr, &raw_stats) : false;
    if (allow_adapt && should_update_stats && has_stats && raw_stats.valid)
    {
        const RecognitionWhiteReferenceGains target_gains = BuildTargetGains(raw_stats);
        const float alpha = clamp_float(BW_RECOG_WHITE_REF_NORMALIZE_ALPHA, 0.0f, 1.0f);

        state.raw_stats = raw_stats;
        if (!state.initialized || !state.gains.initialized)
        {
            state.gains = target_gains;
        }
        else
        {
            state.gains.gain_b += (target_gains.gain_b - state.gains.gain_b) * alpha;
            state.gains.gain_g += (target_gains.gain_g - state.gains.gain_g) * alpha;
            state.gains.gain_r += (target_gains.gain_r - state.gains.gain_r) * alpha;
            state.gains.initialized = true;
        }
        state.adjusted_stats = BuildAdjustedStats(state.raw_stats, state.gains);
        state.initialized = true;
        state.frames_until_stats_update = update_interval - 1;
    }
    else if (allow_adapt && should_update_stats)
    {
        state.frames_until_stats_update = 0;
    }
    else if (allow_adapt && state.frames_until_stats_update > 0)
    {
        --state.frames_until_stats_update;
    }
#endif
}

bool GetCurrentStats(RecognitionWhiteReferenceStats* out_stats)
{
    if (out_stats == nullptr)
    {
        return false;
    }
    const NormalizeState& state = State();
    *out_stats = state.adjusted_stats;
    return out_stats->valid;
}

bool GetCurrentGains(RecognitionWhiteReferenceGains* out_gains)
{
    if (out_gains == nullptr)
    {
        return false;
    }
    const NormalizeState& state = State();
    *out_gains = state.gains;
    return out_gains->initialized;
}

cv::Vec3b ApplyGainsToPixel(const cv::Vec3b& bgr)
{
    RecognitionWhiteReferenceGains gains;
    if (!GetCurrentGains(&gains))
    {
        return bgr;
    }
    return ApplyGainsToPixel(bgr, gains);
}

cv::Vec3b ApplyGainsToPixel(const cv::Vec3b& bgr, const RecognitionWhiteReferenceGains& gains)
{
    if (!gains.initialized)
    {
        return bgr;
    }
    return cv::Vec3b(
        clamp_channel(static_cast<float>(bgr[0]) * gains.gain_b),
        clamp_channel(static_cast<float>(bgr[1]) * gains.gain_g),
        clamp_channel(static_cast<float>(bgr[2]) * gains.gain_r));
}

void ApplyGainsToMat(cv::Mat* image_bgr)
{
    if (image_bgr == nullptr || image_bgr->empty())
    {
        return;
    }

    RecognitionWhiteReferenceGains gains;
    if (!GetCurrentGains(&gains) || !gains.initialized)
    {
        return;
    }

    const cv::Matx33f gain_matrix(
        gains.gain_b, 0.0f, 0.0f,
        0.0f, gains.gain_g, 0.0f,
        0.0f, 0.0f, gains.gain_r);
    cv::Mat adjusted;
    cv::transform(*image_bgr, adjusted, gain_matrix);
    adjusted.copyTo(*image_bgr);
}

cv::Mat MakeGainAppliedCopy(const cv::Mat& image_bgr)
{
    if (image_bgr.empty())
    {
        return cv::Mat();
    }
    cv::Mat adjusted = image_bgr.clone();
    ApplyGainsToMat(&adjusted);
    return adjusted;
}

} // namespace recognition_white_reference
