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
  static std::string getIntersectionTypeStr(
      IntersectionType intersection_type) {
    const std::vector<std::string> kIntersectionTypeStrs(
        {"kFullyUnknown", "kFreeOrUnknown", "kPossiblyOccupied"});
    return kIntersectionTypeStrs[to_underlying(intersection_type)];
  }

  explicit RangeImageIntersector(const RangeImage& range_image)
      : hierarchical_range_image_(range_image) {}

  IntersectionType determineIntersectionType(
      const RangeImage& range_image, const Point& t_W_C,
      const AABB<Point>& W_cell_aabb,
      const Eigen::Matrix<FloatingPoint, 2, 4>& C_cell_corners) const {
    // Get the min and max distances from any point in the cell (which is an
    // axis-aligned cube) to the sensor's center
    // NOTE: The min distance is 0 if the cell contains the sensor's center.
    const FloatingPoint d_C_cell_closest = W_cell_aabb.minDistanceTo(t_W_C);
    if (BeamModel::kRangeMax < d_C_cell_closest) {
      return IntersectionType::kFullyUnknown;
    } else if (d_C_cell_closest < kEpsilon) {
      return IntersectionType::kPossiblyOccupied;
    }
    const FloatingPoint d_C_cell_furthest = W_cell_aabb.maxDistanceTo(t_W_C);

    // Get the min and max angles for any point in the cell projected into the
    // range image
    FloatingPoint min_angle = std::numeric_limits<FloatingPoint>::max();
    FloatingPoint max_angle = std::numeric_limits<FloatingPoint>::lowest();
    if ((0.f < C_cell_corners.row(0).array()).all()) {
      const Eigen::Matrix<FloatingPoint, 4, 1> tans =
          C_cell_corners.row(1).array() / C_cell_corners.row(0).array();
      const FloatingPoint min_tan = tans.minCoeff();
      const FloatingPoint max_tan = tans.maxCoeff();
      min_angle = std::atan(min_tan);
      max_angle = std::atan(max_tan);
    } else {
      Eigen::Matrix<FloatingPoint, 4, 1> angles;
      for (int corner_idx = 0; corner_idx < 4; ++corner_idx) {
        angles[corner_idx] =
            RangeImage::bearingToAngle(C_cell_corners.col(corner_idx));
      }
      min_angle = angles.minCoeff();
      max_angle = angles.maxCoeff();
    }

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

 private:
  HierarchicalRangeImage hierarchical_range_image_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_
