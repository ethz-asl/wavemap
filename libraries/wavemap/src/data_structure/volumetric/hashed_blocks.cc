#include "wavemap/data_structure/volumetric/hashed_blocks.h"

namespace wavemap {
void HashedBlocks::prune() {
  std::unordered_set<BlockIndex, IndexHash<3>> blocks_to_delete;
  for (const auto& [block_index, block_data] : blocks_) {
    if (std::all_of(block_data.begin(), block_data.end(),
                    [default_value = default_value_](const auto& cell_value) {
                      return cell_value == default_value;
                    })) {
      blocks_to_delete.emplace(block_index);
    }
  }
  std::for_each(blocks_to_delete.cbegin(), blocks_to_delete.cend(),
                [&blocks = blocks_](const auto& block_index) {
                  blocks.erase(block_index);
                });
}

Index3D HashedBlocks::getMinIndex() const {
  if (empty()) {
    return Index3D::Zero();
  }

  Index3D min_block_index =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  for (const auto& [block_index, block] : blocks_) {
    min_block_index = min_block_index.cwiseMin(block_index);
  }
  return kCellsPerSide * min_block_index;
}

Index3D HashedBlocks::getMaxIndex() const {
  if (empty()) {
    return Index3D::Zero();
  }

  Index3D max_block_index =
      Index3D::Constant(std::numeric_limits<IndexElement>::lowest());
  for (const auto& [block_index, block] : blocks_) {
    max_block_index = max_block_index.cwiseMax(block_index);
  }
  return kCellsPerSide * (max_block_index + Index3D::Ones());
}

void HashedBlocks::forEachLeaf(
    VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn) const {
  const Index3D min_local_cell_index = Index3D::Zero();
  const Index3D max_local_cell_index = Index3D::Constant(kCellsPerSide - 1);

  for (const auto& [block_index, block_data] : blocks_) {
    // TODO(victorr): Iterate directly over linear index instead of grid
    for (const Index3D& cell_index :
         Grid(min_local_cell_index, max_local_cell_index)) {
      const FloatingPoint cell_data =
          block_data[convert::indexToLinearIndex<kCellsPerSide>(cell_index)];
      const Index3D index =
          computeIndexFromBlockIndexAndCellIndex(block_index, cell_index);
      const OctreeIndex hierarchical_cell_index =
          convert::indexAndHeightToNodeIndex(index, 0);
      visitor_fn(hierarchical_cell_index, cell_data);
    }
  }
}
}  // namespace wavemap
