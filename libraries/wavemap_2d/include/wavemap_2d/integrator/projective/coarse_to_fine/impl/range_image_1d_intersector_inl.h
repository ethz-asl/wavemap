#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_1D_INTERSECTOR_INL_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_1D_INTERSECTOR_INL_H_

#include <algorithm>

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap_common/utils/angle_utils.h>
#include <wavemap_common/utils/approximate_trigonometry.h>

namespace wavemap {
// NOTE: Despite being branch-heavy, this implementation still outperforms all
//       fully vectorized branch-free versions we tried. Possibly because we
//       always only really need 2 of the 4 angles and branch-prediction works
//       reasonably well for our query pattern. This might change for 3D
//       and/or if we batch queries together.
inline RangeImage1DIntersector::MinMaxAnglePair
RangeImage1DIntersector::getAabbMinMaxProjectedAngle(
    const Transformation2D& T_W_C, const AABB<Point2D>& W_aabb) {
  // If the sensor is contained in the AABB, it overlaps with the full range
  if (W_aabb.containsPoint(T_W_C.getPosition())) {
    return {-kPi, kPi};
  }

  // Translate the AABB into frame C, but do not yet rotate it
  const Point2D W_t_C_min = W_aabb.min - T_W_C.getPosition();
  const Point2D W_t_C_max = W_aabb.max - T_W_C.getPosition();

  // Find the min and max angles for the AABB's corners
  MinMaxAnglePair angle_pair;
  const bool aabb_crosses_x_axis =
      std::signbit(W_t_C_min.y()) != std::signbit(W_t_C_max.y());
  const bool aabb_crosses_y_axis =
      std::signbit(W_t_C_min.x()) != std::signbit(W_t_C_max.x());
  const bool aabb_fully_in_left_quadrants =
      !std::signbit(W_t_C_min.y()) && !std::signbit(W_t_C_max.y());
  const bool aabb_fully_in_upper_quadrants =
      !std::signbit(W_t_C_min.x()) && !std::signbit(W_t_C_max.x());
  if (aabb_crosses_x_axis) {
    // NOTE: The AABB cannot overlap with the origin as this case was already
    //       addressed at the start of the method.
    if (aabb_fully_in_upper_quadrants) {
      // AABB crosses both upper quadrants
      angle_pair.min_angle = approximate::atan2()(W_t_C_min.y(), W_t_C_min.x());
      angle_pair.max_angle = approximate::atan2()(W_t_C_max.y(), W_t_C_min.x());
    } else {
      // AABB crosses both lower quadrants
      angle_pair.min_angle = approximate::atan2()(W_t_C_max.y(), W_t_C_max.x());
      angle_pair.max_angle = approximate::atan2()(W_t_C_min.y(), W_t_C_max.x());
    }
  } else {
    if (aabb_fully_in_left_quadrants) {
      if (aabb_crosses_y_axis) {
        // AABB crosses both left quadrants
        angle_pair.min_angle =
            approximate::atan2()(W_t_C_min.y(), W_t_C_max.x());
        angle_pair.max_angle =
            approximate::atan2()(W_t_C_min.y(), W_t_C_min.x());
      } else if (aabb_fully_in_upper_quadrants) {
        // AABB is fully in the upper left quadrant
        angle_pair.min_angle =
            approximate::atan2()(W_t_C_min.y(), W_t_C_max.x());
        angle_pair.max_angle =
            approximate::atan2()(W_t_C_max.y(), W_t_C_min.x());
      } else {
        // AABB is fully in the bottom left quadrant
        angle_pair.min_angle =
            approximate::atan2()(W_t_C_max.y(), W_t_C_max.x());
        angle_pair.max_angle =
            approximate::atan2()(W_t_C_min.y(), W_t_C_min.x());
      }
    } else {
      if (aabb_crosses_y_axis) {
        // AABB crosses both right quadrants
        angle_pair.min_angle =
            approximate::atan2()(W_t_C_max.y(), W_t_C_min.x());
        angle_pair.max_angle =
            approximate::atan2()(W_t_C_max.y(), W_t_C_max.x());
      } else if (aabb_fully_in_upper_quadrants) {
        // AABB is fully in the upper right quadrant
        angle_pair.min_angle =
            approximate::atan2()(W_t_C_min.y(), W_t_C_min.x());
        angle_pair.max_angle =
            approximate::atan2()(W_t_C_max.y(), W_t_C_max.x());
      } else {
        // AABB is fully in the bottom right quadrant
        angle_pair.min_angle =
            approximate::atan2()(W_t_C_max.y(), W_t_C_min.x());
        angle_pair.max_angle =
            approximate::atan2()(W_t_C_min.y(), W_t_C_max.x());
      }
    }
  }

  // Make the angle range conservative by padding it with the worst-case error
  // of our atan approximation (in the direction that makes the range largest)
  if (kPi < angle_pair.max_angle - angle_pair.min_angle) {
    angle_pair.min_angle += approximate::atan2::kWorstCaseError;
    angle_pair.max_angle -= approximate::atan2::kWorstCaseError;
  } else {
    angle_pair.min_angle -= approximate::atan2::kWorstCaseError;
    angle_pair.max_angle += approximate::atan2::kWorstCaseError;
  }

  // Rotate the min/max angles we found into frame C
  angle_pair.min_angle -= T_W_C.getRotation().angle();
  angle_pair.max_angle -= T_W_C.getRotation().angle();

  // Make sure the angles are still normalized within [-Pi, Pi]
  angle_pair.min_angle = angle_math::normalize_near(angle_pair.min_angle);
  angle_pair.max_angle = angle_math::normalize_near(angle_pair.max_angle);

  return angle_pair;
}

inline IntersectionType RangeImage1DIntersector::determineIntersectionType(
    const Transformation2D& T_W_C, const AABB<Point2D>& W_cell_aabb,
    const CircularProjector& circular_projector) const {
  // Get the min and max distances from any point in the cell (which is an
  // axis-aligned cube) to the sensor's center
  // NOTE: The min distance is 0 if the cell contains the sensor's center.
  const FloatingPoint d_C_cell_closest =
      W_cell_aabb.minDistanceTo(T_W_C.getPosition());
  if (max_range_ < d_C_cell_closest) {
    return IntersectionType::kFullyUnknown;
  }
  const FloatingPoint d_C_cell_furthest =
      W_cell_aabb.maxDistanceTo(T_W_C.getPosition());

  // Get the min and max angles for any point in the cell projected into the
  // range image
  auto [min_angle, max_angle] = getAabbMinMaxProjectedAngle(T_W_C, W_cell_aabb);

  // Pad the min and max angles with the BeamModel's angle threshold to
  // account for the beam's non-zero width (angular uncertainty)
  min_angle -= angle_threshold_;
  max_angle += angle_threshold_;

  // If the angle wraps around Pi, we can't use the hierarchical range image
  if (const bool angle_range_wraps_pi = max_angle < min_angle;
      angle_range_wraps_pi) {
    if (max_angle < circular_projector.getMinAngle() &&
        circular_projector.getMaxAngle() < min_angle) {
      // No parts of the cell can be affected by the measurement update
      return IntersectionType::kFullyUnknown;
    } else {
      // Make sure the cell gets enqueued for refinement, as we can't
      // guarantee anything about its children
      return IntersectionType::kPossiblyOccupied;
    }
  }

  // Check if the cell is outside the observed range
  if (circular_projector.getMaxAngle() < min_angle ||
      max_angle < circular_projector.getMinAngle()) {
    return IntersectionType::kFullyUnknown;
  }

  // Convert the angles to range image indices
  const IndexElement min_image_idx =
      std::max(0, circular_projector.angleToFloorIndex(min_angle));
  const IndexElement max_image_idx =
      std::min(circular_projector.getNumCells() - 1,
               circular_projector.angleToCeilIndex(max_angle));

  // Check if the cell overlaps with the approximate but conservative distance
  // bounds of the hierarchical range image
  const Bounds distance_bounds =
      hierarchical_range_image_.getRangeBounds(min_image_idx, max_image_idx);
  if (distance_bounds.upper + range_threshold_behind_ < d_C_cell_closest) {
    return IntersectionType::kFullyUnknown;
  } else if (d_C_cell_furthest <
             distance_bounds.lower - range_threshold_in_front_) {
    return IntersectionType::kFreeOrUnknown;
  } else {
    return IntersectionType::kPossiblyOccupied;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_1D_INTERSECTOR_INL_H_
