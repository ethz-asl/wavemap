#include "wavemap_2d_ground_truth/occupancy_grid_creator.h"

#include <unordered_set>

#include <wavemap_2d/indexing/index.h>
#include <wavemap_2d/indexing/index_hashes.h>
#include <wavemap_2d/iterator/ray_iterator.h>

#include "wavemap_2d_ground_truth/geometry.h"

namespace wavemap_2d::ground_truth {
const std::vector<Index> OccupancyGridCreator::kNeighborOffsets = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1},
};

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

void OccupancyGridCreator::floodfillUnoccupied(const Index& start_index) {
  std::queue<Index> open_queue;
  std::unordered_set<Index, VoxbloxIndexHash> closed_set;

  const Index min_index = occupancy_grid_.getMinIndex();
  const Index max_index = occupancy_grid_.getMaxIndex();

  open_queue.emplace(start_index);
  while (!open_queue.empty()) {
    const Index current_index = open_queue.front();
    open_queue.pop();

    for (const Index& neighbor_offset : kNeighborOffsets) {
      const Index neighbor_index = current_index + neighbor_offset;
      if ((neighbor_index.array() < min_index.array() ||
           max_index.array() < neighbor_index.array())
              .any()) {
        continue;
      }

      if (closed_set.count(neighbor_index)) {
        continue;
      }
      closed_set.emplace(neighbor_index);

      const FloatingPoint neighbor_value =
          occupancy_grid_.getCellValue(neighbor_index);
      if (neighbor_value <= 0.f) {
        occupancy_grid_.setCellValue(neighbor_index, -1.f);
        open_queue.emplace(neighbor_index);
      }
    }
  }
}
}  // namespace wavemap_2d::ground_truth
