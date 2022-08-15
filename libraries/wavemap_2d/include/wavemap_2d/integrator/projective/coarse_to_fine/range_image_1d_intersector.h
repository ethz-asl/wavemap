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
#include <wavemap_common/integrator/projective/intersection_type.h>
#include <wavemap_common/utils/angle_utils.h>
#include <wavemap_common/utils/type_utils.h>

#include "wavemap_2d/integrator/projective/coarse_to_fine/hierarchical_range_image_1d.h"

namespace wavemap {
class RangeImage1DIntersector {
 public:
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
};
}  // namespace wavemap

#include "wavemap_2d/integrator/projective/coarse_to_fine/impl/range_image_1d_intersector_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_1D_INTERSECTOR_H_
