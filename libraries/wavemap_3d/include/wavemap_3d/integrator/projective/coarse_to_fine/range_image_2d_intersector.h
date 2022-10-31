#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_2D_INTERSECTOR_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_2D_INTERSECTOR_H_

#include <limits>
#include <memory>
#include <utility>

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/aabb.h>
#include <wavemap_common/integrator/projection_model/image_2d/image_2d_projection_model.h>
#include <wavemap_common/integrator/projective/intersection_type.h>

#include "wavemap_3d/integrator/projective/coarse_to_fine/hierarchical_range_bounds_2d.h"
#include "wavemap_3d/integrator/projective/range_image_2d.h"

namespace wavemap {
class RangeImage2DIntersector {
 public:
  RangeImage2DIntersector(
      std::shared_ptr<const RangeImage2D> range_image,
      std::shared_ptr<const Image2DProjectionModel> projection_model,
      FloatingPoint min_range, FloatingPoint max_range,
      FloatingPoint angle_threshold,
      FloatingPoint range_threshold_in_front_of_surface,
      FloatingPoint range_threshold_behind_surface)
      : y_axis_wraps_around_(projection_model->sensorAxisIsPeriodic().y()),
        hierarchical_range_image_(std::move(range_image), y_axis_wraps_around_,
                                  min_range),
        projection_model_(std::move(projection_model)),
        min_range_(min_range),
        max_range_(max_range),
        angle_threshold_(angle_threshold),
        range_threshold_in_front_(range_threshold_in_front_of_surface),
        range_threshold_behind_(range_threshold_behind_surface) {}

  IntersectionType determineIntersectionType(
      const Transformation3D& T_W_C, const AABB<Point3D>& W_cell_aabb) const;

 private:
  const bool y_axis_wraps_around_;
  const HierarchicalRangeBounds2D hierarchical_range_image_;

  const std::shared_ptr<const Image2DProjectionModel> projection_model_;

  const FloatingPoint min_range_;
  const FloatingPoint max_range_;
  const FloatingPoint angle_threshold_;
  const FloatingPoint range_threshold_in_front_;
  const FloatingPoint range_threshold_behind_;
};
}  // namespace wavemap

#include "wavemap_3d/integrator/projective/coarse_to_fine/impl/range_image_2d_intersector_inl.h"

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_2D_INTERSECTOR_H_
