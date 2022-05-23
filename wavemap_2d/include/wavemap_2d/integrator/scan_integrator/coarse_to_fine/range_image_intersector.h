#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/generic/aabb.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/hierarchical_range_image.h"
#include "wavemap_2d/utils/angle_utils.h"
#include "wavemap_2d/utils/type_utils.h"

namespace wavemap_2d {
class RangeImageIntersector {
 public:
  enum class IntersectionType : int {
    kFullyUnknown,
    kFreeOrUnknown,
    kPossiblyOccupied
  };
  struct MinMaxAnglePair {
    FloatingPoint min_angle = std::numeric_limits<FloatingPoint>::max();
    FloatingPoint max_angle = std::numeric_limits<FloatingPoint>::lowest();
  };

  explicit RangeImageIntersector(const RangeImage& range_image)
      : hierarchical_range_image_(range_image) {}

  // NOTE: When the AABB is right behind the sensor, the angle range will wrap
  //       around at +-PI and a min_angle >= max_angle will be returned.
  // NOTE: Despite being branch-heavy, this implementation still outperforms all
  //       fully vectorized branch-free versions we tried. Possibly because we
  //       always only really need 2 of the 4 angles and branch-prediction works
  //       reasonably well for our query pattern. This might change for 3D
  //       and/or if we batch queries together.
  static MinMaxAnglePair getAabbMinMaxProjectedAngle(
      const Transformation& T_W_C, const AABB<Point>& W_aabb) {
    // If the sensor is contained in the AABB, it overlaps with the full range
    if (W_aabb.containsPoint(T_W_C.getPosition())) {
      return {-M_PIf32, M_PIf32};
    }

    // Translate the AABB into frame C, but do not yet rotate it
    const Point W_t_C_min = W_aabb.min - T_W_C.getPosition();
    const Point W_t_C_max = W_aabb.max - T_W_C.getPosition();

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
        angle_pair.min_angle = atan2_approx(W_t_C_min.y(), W_t_C_min.x());
        angle_pair.max_angle = atan2_approx(W_t_C_max.y(), W_t_C_min.x());
      } else {
        // AABB crosses both lower quadrants
        angle_pair.min_angle = atan2_approx(W_t_C_max.y(), W_t_C_max.x());
        angle_pair.max_angle = atan2_approx(W_t_C_min.y(), W_t_C_max.x());
      }
    } else {
      if (aabb_fully_in_left_quadrants) {
        if (aabb_crosses_y_axis) {
          // AABB crosses both left quadrants
          angle_pair.min_angle = atan2_approx(W_t_C_min.y(), W_t_C_max.x());
          angle_pair.max_angle = atan2_approx(W_t_C_min.y(), W_t_C_min.x());
        } else if (aabb_fully_in_upper_quadrants) {
          // AABB is fully in the upper left quadrant
          angle_pair.min_angle = atan2_approx(W_t_C_min.y(), W_t_C_max.x());
          angle_pair.max_angle = atan2_approx(W_t_C_max.y(), W_t_C_min.x());
        } else {
          // AABB is fully in the bottom left quadrant
          angle_pair.min_angle = atan2_approx(W_t_C_max.y(), W_t_C_max.x());
          angle_pair.max_angle = atan2_approx(W_t_C_min.y(), W_t_C_min.x());
        }
      } else {
        if (aabb_crosses_y_axis) {
          // AABB crosses both right quadrants
          angle_pair.min_angle = atan2_approx(W_t_C_max.y(), W_t_C_min.x());
          angle_pair.max_angle = atan2_approx(W_t_C_max.y(), W_t_C_max.x());
        } else if (aabb_fully_in_upper_quadrants) {
          // AABB is fully in the upper right quadrant
          angle_pair.min_angle = atan2_approx(W_t_C_min.y(), W_t_C_min.x());
          angle_pair.max_angle = atan2_approx(W_t_C_max.y(), W_t_C_max.x());
        } else {
          // AABB is fully in the bottom right quadrant
          angle_pair.min_angle = atan2_approx(W_t_C_max.y(), W_t_C_min.x());
          angle_pair.max_angle = atan2_approx(W_t_C_min.y(), W_t_C_max.x());
        }
      }
    }

    // Make the angle range conservative by padding it with the worst-case error
    // of our atan approximation (in the direction that makes the range largest)
    if (M_PIf32 < angle_pair.max_angle - angle_pair.min_angle) {
      angle_pair.min_angle += kWorstCaseAtan2ApproxError;
      angle_pair.max_angle -= kWorstCaseAtan2ApproxError;
    } else {
      angle_pair.min_angle -= kWorstCaseAtan2ApproxError;
      angle_pair.max_angle += kWorstCaseAtan2ApproxError;
    }

    // Rotate the min/max angles we found into frame C
    angle_pair.min_angle -= T_W_C.getRotation().angle();
    angle_pair.max_angle -= T_W_C.getRotation().angle();

    // Make sure the angles are still normalized within [-Pi, Pi]
    angle_pair.min_angle = angle_math::normalize_near(angle_pair.min_angle);
    angle_pair.max_angle = angle_math::normalize_near(angle_pair.max_angle);

    return angle_pair;
  }

  IntersectionType determineIntersectionType(
      const RangeImage& range_image, const Transformation& T_W_C,
      const AABB<Point>& W_cell_aabb) const {
    // Get the min and max distances from any point in the cell (which is an
    // axis-aligned cube) to the sensor's center
    // NOTE: The min distance is 0 if the cell contains the sensor's center.
    const FloatingPoint d_C_cell_closest =
        W_cell_aabb.minDistanceTo(T_W_C.getPosition());
    if (BeamModel::kRangeMax < d_C_cell_closest) {
      return IntersectionType::kFullyUnknown;
    }
    const FloatingPoint d_C_cell_furthest =
        W_cell_aabb.maxDistanceTo(T_W_C.getPosition());

    // Get the min and max angles for any point in the cell projected into the
    // range image
    auto [min_angle, max_angle] =
        getAabbMinMaxProjectedAngle(T_W_C, W_cell_aabb);
    const bool angle_range_wraps_pi = max_angle < min_angle;

    // Pad the min and max angles with the BeamModel's angle threshold to
    // account for the beam's non-zero width (angular uncertainty)
    min_angle -= BeamModel::kAngleThresh;
    max_angle += BeamModel::kAngleThresh;

    // If the angle wraps around Pi, we can't use the hierarchical range image
    if (angle_range_wraps_pi) {
      if (max_angle < range_image.getMinAngle() &&
          range_image.getMaxAngle() < min_angle) {
        // No parts of the cell can be affected by the measurement update
        return IntersectionType::kFullyUnknown;
      } else {
        // Make sure the cell gets enqueued for refinement, as we can't
        // guarantee anything about its children
        return IntersectionType::kPossiblyOccupied;
      }
    }

    // Check if the cell is outside the observed range
    if (range_image.getMaxAngle() < min_angle ||
        max_angle < range_image.getMinAngle()) {
      return IntersectionType::kFullyUnknown;
    }

    // Convert the angles to range image indices
    const RangeImageIndex min_image_idx =
        std::max(0, range_image.angleToFloorIndex(min_angle));
    const RangeImageIndex max_image_idx = std::min(
        range_image.getNumBeams() - 1, range_image.angleToCeilIndex(max_angle));

    // Check if the cell overlaps with the approximate but conservative distance
    // bounds of the hierarchical range image
    const Bounds distance_bounds =
        hierarchical_range_image_.getRangeBounds(min_image_idx, max_image_idx);
    if (distance_bounds.upper + BeamModel::kRangeDeltaThresh <
        d_C_cell_closest) {
      return IntersectionType::kFullyUnknown;
    } else if (d_C_cell_furthest <
               distance_bounds.lower - BeamModel::kRangeDeltaThresh) {
      return IntersectionType::kFreeOrUnknown;
    } else {
      return IntersectionType::kPossiblyOccupied;
    }
  }

  static std::string getIntersectionTypeStr(
      IntersectionType intersection_type) {
    const std::vector<std::string> kIntersectionTypeStrs(
        {"kFullyUnknown", "kFreeOrUnknown", "kPossiblyOccupied"});
    return kIntersectionTypeStrs[to_underlying(intersection_type)];
  }

 private:
  HierarchicalRangeImage hierarchical_range_image_;

  // NOTE: Aside from generally being faster than std::atan2, a major advantage
  //       of this atan2 approximation is that it's branch-free and easily gets
  //       vectorized by GCC (e.g. nearby calls and calls in loops).
  static constexpr FloatingPoint kWorstCaseAtan2ApproxError = 0.011f;
  static FloatingPoint atan2_approx(FloatingPoint y, FloatingPoint x) {
    FloatingPoint abs_y =
        std::abs(y) + std::numeric_limits<FloatingPoint>::epsilon();
    FloatingPoint r =
        (x - std::copysign(abs_y, x)) / (abs_y + std::abs(x));  // NOLINT
    FloatingPoint angle = M_PIf32 / 2.f - std::copysign(M_PIf32 / 4.f, x);
    angle += (0.1963f * r * r - 0.9817f) * r;
    return std::copysign(angle, y);
  }

  friend class CoarseToFineIntegratorTest_ApproxAtan2_Test;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_
