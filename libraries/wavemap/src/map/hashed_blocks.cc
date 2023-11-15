#include "wavemap/map/hashed_blocks.h"

namespace wavemap {
Index3D HashedBlocks::getMinIndex() const {
  return kCellsPerSide * getMinBlockIndex();
}

Index3D HashedBlocks::getMaxIndex() const {
  if (empty()) {
    return Index3D::Zero();
  }
  return kCellsPerSide * (getMaxBlockIndex().array() + 1) - 1;
}

void HashedBlocks::prune() {
  block_map_.eraseBlockIf(
      [default_value = default_value_](const BlockIndex& /*block_index*/,
                                       const Block& block) {
        return std::all_of(block.data().cbegin(), block.data().cend(),
                           [default_value](const auto& cell_value) {
                             return cell_value == default_value;
                           });
      });
}

void HashedBlocks::forEachLeaf(
    VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn) const {
  DenseBlockHash::forEachLeaf(
      [&visitor_fn](const Index3D& index, FloatingPoint cell_data) {
        const OctreeIndex hierarchical_cell_index = OctreeIndex{0, index};
        visitor_fn(hierarchical_cell_index, cell_data);
      });
}
}  // namespace wavemap
