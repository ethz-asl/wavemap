#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_INTERSECTOR_INL_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_INTERSECTOR_INL_H_

#include <algorithm>
#include <bitset>
#include <limits>

#include "wavemap/core/integrator/measurement_model/continuous_beam.h"

namespace wavemap {
inline UpdateType RangeImageIntersector::determineUpdateType(
    const AABB<Point3D>& W_cell_aabb,
    const Transformation3D::RotationMatrix& R_C_W, const Point3D& t_W_C) const {
  if (W_cell_aabb.containsPoint(t_W_C)) {
    return UpdateType::kPossiblyOccupied;
  }

  // Get the min and max angles for any point in the cell projected into the
  // range image
  auto [min_sensor_coordinates, max_sensor_coordinates] =
      projection_model_->cartesianToSensorAABB(W_cell_aabb, R_C_W, t_W_C);
  if (max_range_ < min_sensor_coordinates.z() ||
      max_sensor_coordinates.z() < min_range_) {
    return UpdateType::kFullyUnobserved;
  }

  // Pad the min and max angles with the measurement model's angle threshold to
  // account for the beam's non-zero width (angular uncertainty)
  const Index2D min_image_index = projection_model_->imageToFloorIndex(
      min_sensor_coordinates.head<2>() - Vector2D::Constant(angle_threshold_));
  const Index2D max_image_index = projection_model_->imageToCeilIndex(
      max_sensor_coordinates.head<2>() + Vector2D::Constant(angle_threshold_));

  // If the angle wraps around, we can't always use the hierarchical range image
  const bool x_range_wraps_around = max_image_index.x() < min_image_index.x();
  const bool y_range_wraps_around = max_image_index.y() < min_image_index.y();
  if (x_range_wraps_around || (!y_axis_wraps_around_ && y_range_wraps_around)) {
    const bool x_range_fully_outside_fov =
        max_image_index.x() < 0 &&
        projection_model_->getNumRows() < min_image_index.x();
    const bool y_range_fully_outside_fov =
        max_image_index.y() < 0 &&
        projection_model_->getNumColumns() < min_image_index.y();
    if ((!x_range_wraps_around || x_range_fully_outside_fov) &&
        (y_axis_wraps_around_ || !y_range_wraps_around ||
         y_range_fully_outside_fov)) {
      // No parts of the cell can be affected by the measurement update
      return UpdateType::kFullyUnobserved;
    } else {
      // Make sure the cell gets enqueued for refinement, as we can't
      // guarantee anything about its children
      return UpdateType::kPossiblyOccupied;
    }
  }

  // Check if the cell is outside the FoV
  if ((max_image_index.array() < 0).any() ||
      (projection_model_->getDimensions().array() <= min_image_index.array())
          .any()) {
    return UpdateType::kFullyUnobserved;
  }

  // Check if the cell overlaps with the approximate but conservative distance
  // bounds of the hierarchical range image
  const FloatingPoint min_z_coordinate =
      min_sensor_coordinates.z() - range_threshold_behind_;
  const FloatingPoint max_z_coordinate =
      max_sensor_coordinates.z() + range_threshold_in_front_;
  return hierarchical_range_image_.getUpdateType(
      min_image_index, max_image_index, min_z_coordinate, max_z_coordinate);
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_INTERSECTOR_INL_H_
