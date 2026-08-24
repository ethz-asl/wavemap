#ifndef WAVEMAP_LAYERED_TYPES_WEIGHTED_MEAN_STATE_H_
#define WAVEMAP_LAYERED_TYPES_WEIGHTED_MEAN_STATE_H_

#include <wavemap/core/common.h>

namespace wavemap::layered {

// Sufficient statistics for a true scalar running weighted mean. Zero weight
// explicitly represents an unobserved voxel and is not a measurement of zero.
struct WeightedMeanState {
  FloatingPoint weighted_sum = 0.f;
  FloatingPoint total_weight = 0.f;

  FloatingPoint valueOr(FloatingPoint fallback = 0.f) const {
    return 0.f < total_weight ? weighted_sum / total_weight : fallback;
  }

  bool operator==(const WeightedMeanState& other) const {
    return weighted_sum == other.weighted_sum &&
           total_weight == other.total_weight;
  }
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_TYPES_WEIGHTED_MEAN_STATE_H_
