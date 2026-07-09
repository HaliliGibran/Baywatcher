#pragma once

#include <cstddef>

namespace recognition_mlp_weights {

constexpr int kInputSize = 32;
constexpr int kChannels = 3;
constexpr int kInputElements = 3072;
constexpr int kHiddenUnits = 128;
constexpr int kClassCount = 8;

extern const float kMean[kChannels];
extern const float kStd[kChannels];
extern const float kFc1Weight[kHiddenUnits * kInputElements];
extern const float kFc1Bias[kHiddenUnits];
extern const float kFc2Weight[kClassCount * kHiddenUnits];
extern const float kFc2Bias[kClassCount];

} // namespace recognition_mlp_weights
