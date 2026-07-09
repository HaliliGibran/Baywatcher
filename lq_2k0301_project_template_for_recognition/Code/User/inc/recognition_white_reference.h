#pragma once

#include <opencv2/opencv.hpp>

struct RecognitionWhiteReferenceStats
{
    bool valid = false;
    int sample_count = 0;
    float mean_b = 0.0f;
    float mean_g = 0.0f;
    float mean_r = 0.0f;
    float mean_luma = 0.0f;
};

struct RecognitionWhiteReferenceGains
{
    bool initialized = false;
    float gain_b = 1.0f;
    float gain_g = 1.0f;
    float gain_r = 1.0f;
};

namespace recognition_white_reference {

void UpdateFromFrame(const cv::Mat& frame_bgr, bool allow_adapt);

bool GetCurrentStats(RecognitionWhiteReferenceStats* out_stats);
bool GetCurrentGains(RecognitionWhiteReferenceGains* out_gains);

cv::Vec3b ApplyGainsToPixel(const cv::Vec3b& bgr);
cv::Vec3b ApplyGainsToPixel(const cv::Vec3b& bgr, const RecognitionWhiteReferenceGains& gains);

void ApplyGainsToMat(cv::Mat* image_bgr);
cv::Mat MakeGainAppliedCopy(const cv::Mat& image_bgr);

} // namespace recognition_white_reference
