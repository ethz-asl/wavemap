#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_

#include <algorithm>

#include "wavemap_2d/common.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/hierarchical_range_image.h"

namespace wavemap_2d {
class RangeImageIntersector {
 public:
  enum class IntersectionType {
    kFullyInside,
    kIntersectsBoundary,
    kFullyOutside
  };

  explicit RangeImageIntersector(const RangeImage& range_image)
      : range_image_bounds_pyramid_(range_image) {}

  IntersectionType determineIntersectionType(
      const RangeImage& range_image, const Point& C_sphere_center,
      FloatingPoint sphere_center_distance, FloatingPoint sphere_radius) const {
    const FloatingPoint min_node_distance =
        sphere_center_distance - sphere_radius;
    const FloatingPoint max_node_distance =
        sphere_center_distance + sphere_radius;
    if (min_node_distance < kEpsilon) {
      return IntersectionType::kIntersectsBoundary;
    }

    const FloatingPoint sphere_center_angle =
        RangeImage::bearingToAngle(C_sphere_center);
    const FloatingPoint half_sphere_angle =
        std::atan2(sphere_radius, sphere_center_distance);
    const FloatingPoint min_angle = sphere_center_angle - half_sphere_angle;
    const FloatingPoint max_angle = sphere_center_angle + half_sphere_angle;
    if (max_angle < range_image.getMinAngle() ||
        range_image.getMaxAngle() < min_angle) {
      return IntersectionType::kFullyOutside;
    }

    const RangeImageIndex lower_idx =
        std::max(range_image.angleToFloorIndex(min_angle), 0);
    const RangeImageIndex upper_idx = std::min(
        range_image.angleToCeilIndex(max_angle), range_image.getNumBeams() - 1);

    // TODO(victorr): Make this fast using the range bound pyramid
    const FloatingPoint max_image_distance =
        range_image.getData()
            .block(1, lower_idx, 1, upper_idx - lower_idx)
            .array()
            .maxCoeff();
    const FloatingPoint min_image_distance =
        range_image.getData()
            .block(1, lower_idx, 1, upper_idx - lower_idx)
            .array()
            .minCoeff();

    if (max_node_distance < min_image_distance) {
      return IntersectionType::kFullyInside;
    } else if (max_image_distance < min_node_distance) {
      return IntersectionType::kFullyOutside;
    } else {
      return IntersectionType::kIntersectsBoundary;
    }
  }

 private:
  HierarchicalRangeImage range_image_bounds_pyramid_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_RANGE_IMAGE_INTERSECTOR_H_
