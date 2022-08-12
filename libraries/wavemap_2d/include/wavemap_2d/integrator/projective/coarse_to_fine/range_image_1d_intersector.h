#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_1D_INTERSECTOR_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_1D_INTERSECTOR_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/aabb.h>
#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap_common/utils/angle_utils.h>
#include <wavemap_common/utils/type_utils.h>

#include "wavemap_2d/integrator/projective/coarse_to_fine/hierarchical_range_image_1d.h"

namespace wavemap {
class RangeImage1DIntersector {
 public:
  enum class IntersectionType : int {
    kFullyUnknown,
    kFreeOrUnknown,
    kPossiblyOccupied
  };
  static std::string getIntersectionTypeStr(
      IntersectionType intersection_type) {
    static constexpr std::array<const char*, 3> kIntersectionTypeStrs(
        {"fully_unknown", "free_or_unknown", "possibly_occupied"});
    return kIntersectionTypeStrs[to_underlying(intersection_type)];
  }

  struct MinMaxAnglePair {
    FloatingPoint min_angle = std::numeric_limits<FloatingPoint>::max();
    FloatingPoint max_angle = std::numeric_limits<FloatingPoint>::lowest();
  };

  explicit RangeImage1DIntersector(std::shared_ptr<RangeImage1D> range_image)
      : hierarchical_range_image_(std::move(range_image)) {}

  // NOTE: When the AABB is right behind the sensor, the angle range will wrap
  //       around at +-PI and a min_angle >= max_angle will be returned.
  static MinMaxAnglePair getAabbMinMaxProjectedAngle(
      const Transformation2D& T_W_C, const AABB<Point2D>& W_aabb);

  IntersectionType determineIntersectionType(
      const Transformation2D& T_W_C, const AABB<Point2D>& W_cell_aabb,
      const CircularProjector& circular_projector) const;

 private:
  HierarchicalRangeImage1D hierarchical_range_image_;

  // NOTE: Aside from generally being faster than std::atan2, a major advantage
  //       of this atan2 approximation is that it's branch-free and easily gets
  //       vectorized by GCC (e.g. nearby calls and calls in loops).
  static constexpr FloatingPoint kWorstCaseAtan2ApproxError = 0.011f;
  static FloatingPoint atan2_approx(FloatingPoint y, FloatingPoint x) {
    FloatingPoint abs_y =
        std::abs(y) + std::numeric_limits<FloatingPoint>::epsilon();
    FloatingPoint r =
        (x - std::copysign(abs_y, x)) / (abs_y + std::abs(x));  // NOLINT
    FloatingPoint angle = kHalfPi - std::copysign(kQuarterPi, x);
    angle += (0.1963f * r * r - 0.9817f) * r;
    return std::copysign(angle, y);
  }

  friend class CoarseToFineIntegratorTest_ApproxAtan2_Test;
};
}  // namespace wavemap

#include "wavemap_2d/integrator/projective/coarse_to_fine/impl/range_image_1d_intersector_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_1D_INTERSECTOR_H_
