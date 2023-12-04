#ifndef WAVEMAP_MAP_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_BLOCK_INL_H_
#define WAVEMAP_MAP_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_BLOCK_INL_H_

#include "wavemap/utils/query/occupancy_classifier.h"

namespace wavemap {
inline bool HashedChunkedWaveletOctreeBlock::empty() const {
  // Check if all cells in the block are equal to zero
  // NOTE: Aside from checking whether the block contains no detail
  //       coefficients, we also need to check whether its scale coefficient
  //       (average value over the whole block) is zero.
  return chunked_ndtree_.empty() &&
         OccupancyClassifier::isUnobserved(root_scale_coefficient_);
}

inline FloatingPoint HashedChunkedWaveletOctreeBlock::getTimeSinceLastUpdated()
    const {
  return time::to_seconds<FloatingPoint>(Time::now() - last_updated_stamp_);
}

inline FloatingPoint HashedChunkedWaveletOctreeBlock::getCellValue(
    const OctreeIndex& index) const {
  // Descend the tree chunk by chunk
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  const NodeChunkType* current_chunk = &chunked_ndtree_.getRootChunk();
  FloatingPoint value = root_scale_coefficient_;
  for (int chunk_top_height = tree_height_; index.height < chunk_top_height;
       chunk_top_height -= kChunkHeight) {
    // Decompress level by level
    for (int parent_height = chunk_top_height;
         chunk_top_height - kChunkHeight < parent_height; --parent_height) {
      // Perform one decompression stage
      const LinearIndex relative_node_index =
          OctreeIndex::computeTreeTraversalDistance(
              morton_code, chunk_top_height, parent_height);
      const NdtreeIndexRelativeChild relative_child_index =
          OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
      value = Transform::backwardSingleChild(
          {value, current_chunk->nodeData(relative_node_index)},
          relative_child_index);
      // If we've reached the requested resolution or there are no remaining
      // higher resolution details, return
      if (parent_height == index.height + 1 ||
          !current_chunk->nodeHasAtLeastOneChild(relative_node_index)) {
        return value;
      }
    }

    // Descend to the next chunk if it exists
    const LinearIndex linear_child_index =
        OctreeIndex::computeLevelTraversalDistance(
            morton_code, chunk_top_height, chunk_top_height - kChunkHeight);
    // If there are no remaining higher resolution details, return
    if (!current_chunk->hasChild(linear_child_index)) {
      break;
    }
    current_chunk = current_chunk->getChild(linear_child_index);
  }

  return value;
}
}  // namespace wavemap

#endif  // WAVEMAP_MAP_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_BLOCK_INL_H_
