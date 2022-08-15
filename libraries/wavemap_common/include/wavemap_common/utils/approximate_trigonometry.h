#ifndef WAVEMAP_COMMON_UTILS_APPROXIMATE_TRIGONOMETRY_H_
#define WAVEMAP_COMMON_UTILS_APPROXIMATE_TRIGONOMETRY_H_

#include <functional>
#include <limits>

#include "wavemap_common/common.h"

namespace wavemap::approximate {
// NOTE: Aside from generally being faster than std::atan2, a major advantage
//       of this atan2 approximation is that it's branch-free and easily gets
//       vectorized by GCC (e.g. nearby calls and calls in loops).
struct atan2
    : public std::binary_function<FloatingPoint, FloatingPoint, FloatingPoint> {
  static constexpr FloatingPoint kWorstCaseError = 0.011f;

  FloatingPoint operator()(FloatingPoint y, FloatingPoint x) const {
    FloatingPoint abs_y =
        std::abs(y) + std::numeric_limits<FloatingPoint>::epsilon();
    FloatingPoint r =
        (x - std::copysign(abs_y, x)) / (abs_y + std::abs(x));  // NOLINT
    FloatingPoint angle = kHalfPi - std::copysign(kQuarterPi, x);
    angle += (0.1963f * r * r - 0.9817f) * r;
    return std::copysign(angle, y);
  }
};
}  // namespace wavemap::approximate

#endif  // WAVEMAP_COMMON_UTILS_APPROXIMATE_TRIGONOMETRY_H_
