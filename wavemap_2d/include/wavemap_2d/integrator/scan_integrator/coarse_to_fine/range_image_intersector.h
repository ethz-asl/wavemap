#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/generic/aabb.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/hierarchical_range_image.h"
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
        angle_pair.min_angle = safe_atan(W_t_C_min.y(), W_t_C_min.x());
        angle_pair.max_angle = safe_atan(W_t_C_max.y(), W_t_C_min.x());
      } else {
        // AABB crosses both lower quadrants
        angle_pair.min_angle =
            safe_atan(W_t_C_max.y(), W_t_C_max.x()) + M_PIf32;
        angle_pair.max_angle =
            safe_atan(W_t_C_min.y(), W_t_C_max.x()) - M_PIf32;
      }
    } else {
      if (aabb_fully_in_left_quadrants) {
        if (aabb_crosses_y_axis) {
          // AABB crosses both left quadrants
          angle_pair.min_angle =
              M_PI_2f32 - safe_atan(W_t_C_max.x(), W_t_C_min.y());
          angle_pair.max_angle =
              M_PI_2f32 - safe_atan(W_t_C_min.x(), W_t_C_min.y());
        } else if (aabb_fully_in_upper_quadrants) {
          // AABB is fully in the upper left quadrant
          angle_pair.min_angle = safe_atan(W_t_C_min.y(), W_t_C_max.x());
          angle_pair.max_angle = safe_atan(W_t_C_max.y(), W_t_C_min.x());
        } else {
          // AABB is fully in the bottom left quadrant
          angle_pair.min_angle =
              M_PI_2f32 - safe_atan(W_t_C_max.x(), W_t_C_max.y());
          angle_pair.max_angle =
              M_PI_2f32 - safe_atan(W_t_C_min.x(), W_t_C_min.y());
        }
      } else {
        if (aabb_crosses_y_axis) {
          // AABB crosses both right quadrants
          angle_pair.min_angle =
              -M_PI_2f32 - safe_atan(W_t_C_min.x(), W_t_C_max.y());
          angle_pair.max_angle =
              -M_PI_2f32 - safe_atan(W_t_C_max.x(), W_t_C_max.y());
        } else if (aabb_fully_in_upper_quadrants) {
          // AABB is fully in the upper right quadrant
          angle_pair.min_angle = safe_atan(W_t_C_min.y(), W_t_C_min.x());
          angle_pair.max_angle = safe_atan(W_t_C_max.y(), W_t_C_max.x());
        } else {
          // AABB is fully in the bottom right quadrant
          angle_pair.min_angle =
              -M_PI_2f32 - safe_atan(W_t_C_min.x(), W_t_C_max.y());
          angle_pair.max_angle =
              -M_PI_2f32 - safe_atan(W_t_C_max.x(), W_t_C_min.y());
        }
      }
    }

    // Rotate the min/max angles we found into frame C
    angle_pair.min_angle -= T_W_C.getRotation().angle();
    angle_pair.max_angle -= T_W_C.getRotation().angle();

    // Normalize the angles to [-Pi, Pi]
    if (angle_pair.min_angle < -M_PIf32) {
      angle_pair.min_angle += 2.f * M_PIf32;
    } else if (M_PIf32 < angle_pair.min_angle) {
      angle_pair.min_angle -= 2.f * M_PIf32;
    }
    if (angle_pair.max_angle < -M_PIf32) {
      angle_pair.max_angle += 2.f * M_PIf32;
    } else if (M_PIf32 < angle_pair.max_angle) {
      angle_pair.max_angle -= 2.f * M_PIf32;
    }

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

    // Convert the angles to range image indices
    // NOTE: We pad the min and max angles with the BeamModel's angle threshold
    //       to account for the beam's non-zero width (angular uncertainty).
    min_angle -= BeamModel::kAngleThresh;
    max_angle += BeamModel::kAngleThresh;
    if (max_angle < range_image.getMinAngle() ||
        range_image.getMaxAngle() < min_angle) {
      return IntersectionType::kFullyUnknown;
    }
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

  // TODO(victorr): Consider replacing this method with conservative
  //                approximations (slightly smaller or equal for min_angle and
  //                slightly greater or equal for max_angle).
  //                This could be worth it as the atan calls currently make up a
  //                very large portion of the program's runtime, but its results
  //                are only used to query the hierarchical range image, which
  //                only returns approximate (but conservative) results.
  static FloatingPoint safe_atan(FloatingPoint y, FloatingPoint x) {
    if (x == 0.f) {
      if (y == 0.f) {
        if (std::signbit(x)) {
          return M_PIf32;
        } else {
          return 0.f;
        }
      } else {
        if (std::signbit(y)) {
          return -M_PI_2f32;
        } else {
          return M_PI_2f32;
        }
      }
    } else {
      return std::atan(y / x);
    }
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_
