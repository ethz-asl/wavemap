#ifndef WAVEMAP_GROUND_TRUTH_OCCUPANCY_GRID_CREATOR_H_
#define WAVEMAP_GROUND_TRUTH_OCCUPANCY_GRID_CREATOR_H_

#include <vector>

#include <wavemap_2d/data_structure/dense_grid.h>
#include <wavemap_common/data_structure/volumetric/cell_types/scalar_cell.h>

#include "wavemap_ground_truth/common.h"
#include "wavemap_ground_truth/geometry.h"

namespace wavemap::ground_truth {
class OccupancyGridCreator {
 public:
  explicit OccupancyGridCreator(
      const VolumetricDataStructureConfig& data_structure_config,
      FloatingPoint slice_height);

  void integrateTriangle(const Triangle& triangle);

  const DenseGrid<UnboundedScalarCell>& getOccupancyGrid() {
    return occupancy_grid_;
  }

  // Floodfill the unoccupied space in the mesh, up to the bounds of the AABB.
  void floodfillUnoccupied(const Index2D& start_index);

 private:
  DenseGrid<UnboundedScalarCell> occupancy_grid_;
  FloatingPoint slice_height_;

  static const std::vector<Index2D> kNeighborOffsets;
};
}  // namespace wavemap::ground_truth

#endif  // WAVEMAP_GROUND_TRUTH_OCCUPANCY_GRID_CREATOR_H_
