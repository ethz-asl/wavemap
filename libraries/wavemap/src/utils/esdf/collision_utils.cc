#include "wavemap/utils/esdf/collision_utils.h"

namespace wavemap {
std::optional<Point3D> getCollisionFreePosition(
    const VolumetricDataStructureBase& occupancy_map, const HashedBlocks& esdf,
    FloatingPoint robot_radius) {
  RandomNumberGenerator rng;

  constexpr size_t kMaxAttempts = 1000;
  for (size_t attempt_idx = 0; attempt_idx < kMaxAttempts; ++attempt_idx) {
    const size_t nth_block =
        rng.getRandomInteger(0ul, esdf.getBlocks().size() - 1ul);
    auto it = esdf.getBlocks().begin();
    std::advance(it, nth_block);
    if (it == esdf.getBlocks().end()) {
      continue;
    }

    const LinearIndex linear_cell_index =
        rng.getRandomInteger(0, HashedBlocks::kCellsPerBlock - 1);

    const Index3D& block_index = it->first;
    const Index3D cell_index =
        convert::linearIndexToIndex<HashedBlocks::kCellsPerSide, 3>(
            linear_cell_index);
    const Index3D global_index =
        esdf.computeIndexFromBlockIndexAndCellIndex(block_index, cell_index);

    const FloatingPoint occupancy_value =
        occupancy_map.getCellValue(global_index);
    const bool is_free = occupancy_value < -1e-3f;
    if (!is_free) {
      continue;
    }

    const auto& block = it->second;
    const FloatingPoint esdf_value = block[linear_cell_index];
    if (esdf_value < robot_radius) {
      continue;
    }

    Point3D collision_free_position =
        convert::indexToCenterPoint(global_index, esdf.getMinCellWidth());
    return collision_free_position;
  }

  LOG(WARNING) << "Could not find collision free position. Giving up after "
               << kMaxAttempts << " attempts.";
  return std::nullopt;
}
}  // namespace wavemap
