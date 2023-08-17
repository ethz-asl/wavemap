#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_WAVELET_OCTREE_BLOCK_INL_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_WAVELET_OCTREE_BLOCK_INL_H_

#include "wavemap/data_structure/volumetric/cell_types/occupancy_state.h"

namespace wavemap {
inline bool HashedWaveletOctreeBlock::empty() const {
  // Check if all cells in the block are equal to zero
  // NOTE: Aside from checking whether the block contains no detail
  //       coefficients, we also need to check whether its scale coefficient
  //       (average value over the whole block) is zero.
  return ndtree_.empty() &&
         !OccupancyState::isObserved(root_scale_coefficient_);
}

inline FloatingPoint HashedWaveletOctreeBlock::getTimeSinceLastUpdated() const {
  return to_seconds<FloatingPoint>(Time::now() - last_updated_stamp_);
}

inline FloatingPoint HashedWaveletOctreeBlock::getCellValue(
    const OctreeIndex& index) const {
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  const NodeType* node = &ndtree_.getRootNode();
  FloatingPoint value = root_scale_coefficient_;
  for (int parent_height = tree_height_; index.height < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    value = Transform::backwardSingleChild({value, node->data()}, child_index);
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
  }
  return value;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_WAVELET_OCTREE_BLOCK_INL_H_
