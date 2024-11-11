#ifndef WAVEMAP_CORE_MAP_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_BLOCK_INL_H_
#define WAVEMAP_CORE_MAP_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_BLOCK_INL_H_

#include "wavemap/core/utils/query/occupancy_classifier.h"

namespace wavemap {
inline bool HashedChunkedWaveletOctreeBlock::empty() const {
  // Check if all cells in the block are equal to zero
  // NOTE: Aside from checking whether the block contains no detail
  //       coefficients, we also need to check whether its scale coefficient
  //       (average value over the whole block) is zero.
  return ndtree_.empty() &&
         OccupancyClassifier::isUnobserved(root_scale_coefficient_);
}

inline FloatingPoint HashedChunkedWaveletOctreeBlock::getTimeSinceLastUpdated()
    const {
  return time::to_seconds<FloatingPoint>(Time::now() - last_updated_stamp_);
}

inline FloatingPoint HashedChunkedWaveletOctreeBlock::getCellValue(
    const OctreeIndex& index) const {
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  OctreeType::NodeConstPtrType node = &ndtree_.getRootNode();
  FloatingPoint value = root_scale_coefficient_;
  for (int parent_height = tree_height_; index.height < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    value = Transform::backwardSingleChild({value, node->data()}, child_index);
    auto child = node->getChild(child_index);
    if (!child) {
      break;
    }
    node = child;
  }
  return value;
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_MAP_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_BLOCK_INL_H_
