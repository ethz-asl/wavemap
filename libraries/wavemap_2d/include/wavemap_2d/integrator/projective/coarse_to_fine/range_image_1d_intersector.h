#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_1D_INTERSECTOR_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_1D_INTERSECTOR_H_

#include <limits>
#include <memory>
#include <utility>

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/aabb.h>
#include <wavemap_common/integrator/projective/update_type.h>

#include "wavemap_2d/integrator/projective/coarse_to_fine/hierarchical_range_image_1d.h"
#include "wavemap_2d/integrator/projective/range_image_1d.h"

namespace wavemap {
class RangeImage1DIntersector {
 public:
  struct MinMaxAnglePair {
    FloatingPoint min_angle = std::numeric_limits<FloatingPoint>::max();
    FloatingPoint max_angle = std::numeric_limits<FloatingPoint>::lowest();
  };

  RangeImage1DIntersector(std::shared_ptr<RangeImage1D> range_image,
                          FloatingPoint max_range,
                          FloatingPoint angle_threshold,
                          FloatingPoint range_threshold_in_front_of_surface,
                          FloatingPoint range_threshold_behind_surface)
      : hierarchical_range_image_(std::move(range_image)),
        max_range_(max_range),
        angle_threshold_(angle_threshold),
        range_threshold_in_front_(range_threshold_in_front_of_surface),
        range_threshold_behind_(range_threshold_behind_surface) {}

  // NOTE: When the AABB is right behind the sensor, the angle range will wrap
  //       around at +-PI and a min_angle >= max_angle will be returned.
  static MinMaxAnglePair getAabbMinMaxProjectedAngle(
      const Transformation2D& T_W_C, const AABB<Point2D>& W_aabb);

  UpdateType determineUpdateType(
      const Transformation2D& T_W_C, const AABB<Point2D>& W_cell_aabb,
      const CircularProjector& circular_projector) const;

 private:
  HierarchicalRangeImage1D hierarchical_range_image_;

  const FloatingPoint max_range_;
  const FloatingPoint angle_threshold_;
  const FloatingPoint range_threshold_in_front_;
  const FloatingPoint range_threshold_behind_;
};
}  // namespace wavemap

#include "wavemap_2d/integrator/projective/coarse_to_fine/impl/range_image_1d_intersector_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_1D_INTERSECTOR_H_
