#ifndef WAVEMAP_2D_GROUND_TRUTH_OCCUPANCY_GRID_CREATOR_H_
#define WAVEMAP_2D_GROUND_TRUTH_OCCUPANCY_GRID_CREATOR_H_

#include <vector>

#include <wavemap_2d/datastructure/cell.h>
#include <wavemap_2d/datastructure/dense_grid/dense_grid.h>

#include "wavemap_2d_ground_truth/common.h"
#include "wavemap_2d_ground_truth/geometry.h"

namespace wavemap_2d::ground_truth {
class OccupancyGridCreator {
 public:
  explicit OccupancyGridCreator(FloatingPoint resolution,
                                FloatingPoint slice_height);

  void integrateTriangle(const Triangle& triangle);

  const DenseGrid<UnboundedCell>& getOccupancyGrid() { return occupancy_grid_; }

  // Floodfill the unoccupied space in the mesh, up to the bounds of the AABB.
  void floodfillUnoccupied(const Index& start_index);

 private:
  DenseGrid<UnboundedCell> occupancy_grid_;
  FloatingPoint slice_height_;

  static const std::vector<Index> kNeighborOffsets;
};
}  // namespace wavemap_2d::ground_truth

#endif  // WAVEMAP_2D_GROUND_TRUTH_OCCUPANCY_GRID_CREATOR_H_
