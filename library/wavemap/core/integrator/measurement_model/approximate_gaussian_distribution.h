#ifndef WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_APPROXIMATE_GAUSSIAN_DISTRIBUTION_H_
#define WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_APPROXIMATE_GAUSSIAN_DISTRIBUTION_H_

#include "wavemap/core/common.h"

namespace wavemap {
struct ApproximateGaussianDistribution {
  // TODO(victorr): Implement and add unit tests
  static FloatingPoint probability(FloatingPoint /*t*/) { return 0.f; }

  // TODO(victorr): Add unit tests
  static FloatingPoint cumulative(FloatingPoint t) {
    const FloatingPoint t_plus_three = t + 3.f;
    const FloatingPoint three_min_t = 3.f - t;
    if (t < -3.f) {
      return 0.f;
    } else if (t <= -1.f) {
      return (1.f / 48.f) * t_plus_three * t_plus_three * t_plus_three;
    } else if (t < 1.f) {
      return (1.f / 2.f) + (1.f / 24.f) * t * t_plus_three * three_min_t;
    } else if (t <= 3.f) {
      return 1.f - (1.f / 48.f) * three_min_t * three_min_t * three_min_t;
    } else {
      return 1.f;
    }
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_APPROXIMATE_GAUSSIAN_DISTRIBUTION_H_
