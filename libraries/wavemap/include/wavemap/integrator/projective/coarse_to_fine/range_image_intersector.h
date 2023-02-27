#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_

#include <limits>
#include <memory>
#include <utility>

#include "wavemap/common.h"
#include "wavemap/data_structure/aabb.h"
#include "wavemap/data_structure/image.h"
#include "wavemap/integrator/measurement_model/measurement_model_base.h"
#include "wavemap/integrator/projection_model/projector_base.h"
#include "wavemap/integrator/projective/coarse_to_fine/hierarchical_range_bounds.h"
#include "wavemap/integrator/projective/update_type.h"

namespace wavemap {
class RangeImageIntersector {
 public:
  RangeImageIntersector(Image<>::ConstPtr range_image,
                        ProjectorBase::ConstPtr projection_model,
                        const MeasurementModelBase& measurement_model,
                        FloatingPoint min_range, FloatingPoint max_range)
      : y_axis_wraps_around_(projection_model->sensorAxisIsPeriodic().y()),
        hierarchical_range_image_(std::move(range_image), y_axis_wraps_around_,
                                  min_range, projection_model.get()),
        projection_model_(std::move(projection_model)),
        min_range_(min_range),
        max_range_(max_range),
        angle_threshold_(measurement_model.getPaddingAngle()),
        range_threshold_in_front_(measurement_model.getPaddingSurfaceFront()),
        range_threshold_behind_(measurement_model.getPaddingSurfaceBack()) {}

  UpdateType determineUpdateType(const AABB<Point3D>& W_cell_aabb,
                                 const Transformation3D::RotationMatrix& R_C_W,
                                 const Point3D& t_W_C) const;

 private:
  const bool y_axis_wraps_around_;
  const HierarchicalRangeBounds hierarchical_range_image_;

  const ProjectorBase::ConstPtr projection_model_;

  const FloatingPoint min_range_;
  const FloatingPoint max_range_;
  const FloatingPoint angle_threshold_;
  const FloatingPoint range_threshold_in_front_;
  const FloatingPoint range_threshold_behind_;
};
}  // namespace wavemap

#include "wavemap/integrator/projective/coarse_to_fine/impl/range_image_intersector_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_
