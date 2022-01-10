#include "wavemap_2d_ground_truth/occupancy_grid_creator.h"

#include <wavemap_2d/integrator/ray_iterator.h>

#include "wavemap_2d_ground_truth/geometry.h"

namespace wavemap_2d::ground_truth {
OccupancyGridCreator::OccupancyGridCreator(FloatingPoint resolution,
                                           FloatingPoint slice_height)
    : occupancy_grid_(resolution), slice_height_(slice_height) {}

void OccupancyGridCreator::integrateTriangle(const Triangle& triangle) {
  const Plane xy_plane{Point3D::UnitZ(), slice_height_};
  LineSegment intersecting_segment;
  if (triangle.intersectsPlane(xy_plane, intersecting_segment)) {
    CHECK_NEAR(intersecting_segment.start_point.z(), slice_height_, kEpsilon);
    CHECK_NEAR(intersecting_segment.end_point.z(), slice_height_, kEpsilon);
    Ray intersection_ray(intersecting_segment.start_point.head<2>(),
                         intersecting_segment.end_point.head<2>(),
                         occupancy_grid_.getResolution());
    for (const Index& index : intersection_ray) {
      occupancy_grid_.setCellValue(index, 1.f);
    }
  }
}

void OccupancyGridCreator::floodfillUnoccupied(FloatingPoint distance_value) {
  // TODO(victorr): Implement BFS based flood fill
}

}  // namespace wavemap_2d::ground_truth
