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
  enum class IntersectionType {
    kFullyInside,
    kIntersectsBoundary,
    kFullyOutside
  };
  static std::string getIntersectionTypeStr(
      IntersectionType intersection_type) {
    const std::vector<std::string> kIntersectionTypeStrs(
        {"kFullyInside", "kIntersectsBoundary", "kFullyOutside"});
    return kIntersectionTypeStrs[to_underlying(intersection_type)];
  }

  explicit RangeImageIntersector(const RangeImage& range_image)
      : hierarchical_range_image_(range_image) {}

  IntersectionType determineIntersectionType(
      const RangeImage& range_image, const Point& t_W_C,
      const AABB<Point>& W_cell_aabb,
      const Eigen::Matrix<FloatingPoint, 2, 4>& C_cell_corners) const {
    const FloatingPoint d_C_cell_closest = W_cell_aabb.minDistanceTo(t_W_C);
    const FloatingPoint d_C_cell_furthest = W_cell_aabb.maxDistanceFrom(t_W_C);
    if (d_C_cell_closest < kEpsilon) {
      return IntersectionType::kIntersectsBoundary;
    }

    FloatingPoint min_angle = std::numeric_limits<FloatingPoint>::max();
    FloatingPoint max_angle = std::numeric_limits<FloatingPoint>::lowest();
    for (int corner_idx = 0; corner_idx < 4; ++corner_idx) {
      const FloatingPoint corner_angle =
          RangeImage::bearingToAngle(C_cell_corners.col(corner_idx));
      min_angle = std::min(min_angle, corner_angle);
      max_angle = std::max(max_angle, corner_angle);
    }
    if (max_angle < range_image.getMinAngle() ||
        range_image.getMaxAngle() < min_angle) {
      return IntersectionType::kFullyOutside;
    } else if (min_angle <= range_image.getMinAngle() ||
               range_image.getMaxAngle() <= max_angle) {
      return IntersectionType::kIntersectsBoundary;
    }
    const RangeImageIndex min_image_idx =
        range_image.angleToFloorIndex(min_angle);
    const RangeImageIndex max_image_idx =
        range_image.angleToCeilIndex(max_angle);

    const Bounds distance_bounds =
        hierarchical_range_image_.getRangeBounds(min_image_idx, max_image_idx);
    constexpr FloatingPoint kMarginFactor = 1.02f;  // E.g. 1.02 == 2% margin
    if (kMarginFactor * distance_bounds.upper < d_C_cell_closest) {
      return IntersectionType::kFullyOutside;
    } else if (kMarginFactor * d_C_cell_furthest < distance_bounds.lower) {
      return IntersectionType::kFullyInside;
    } else {
      return IntersectionType::kIntersectsBoundary;
    }
  }

 private:
  HierarchicalRangeImage hierarchical_range_image_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_
