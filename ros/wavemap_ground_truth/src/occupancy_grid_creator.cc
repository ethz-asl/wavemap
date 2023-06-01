#include "wavemap_ground_truth/occupancy_grid_creator.h"

#include <unordered_set>

#include <wavemap_common/indexing/index_hashes.h>
#include <wavemap_common/iterator/ray_iterator.h>

#include "wavemap_ground_truth/geometry.h"

namespace wavemap::ground_truth {
const std::vector<Index2D> OccupancyGridCreator::kNeighborOffsets = {
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
    Ray<2> intersection_ray(intersecting_segment.start_point.head<2>(),
                            intersecting_segment.end_point.head<2>(),
                            occupancy_grid_.getMinCellWidth());
    for (const Index2D& index : intersection_ray) {
      occupancy_grid_.setCellValue(index, 1.f);
    }
  }
}

void OccupancyGridCreator::floodfillUnoccupied(const Index2D& start_index) {
  std::queue<Index2D> open_queue;
  std::unordered_set<Index2D, VoxbloxIndexHash<2>> closed_set;

  const Index2D min_index = occupancy_grid_.getMinIndex();
  const Index2D max_index = occupancy_grid_.getMaxIndex();

  open_queue.emplace(start_index);
  while (!open_queue.empty()) {
    const Index2D current_index = open_queue.front();
    open_queue.pop();

    for (const Index2D& neighbor_offset : kNeighborOffsets) {
      const Index2D neighbor_index = current_index + neighbor_offset;
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
}  // namespace wavemap::ground_truth
