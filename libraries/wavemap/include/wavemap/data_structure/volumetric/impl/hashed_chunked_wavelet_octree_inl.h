#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_INL_H_

#include "wavemap/indexing/index_conversions.h"

namespace wavemap {
inline size_t HashedChunkedWaveletOctree::size() const {
  size_t size = 0u;
  for (const auto& [block_index, block] : blocks_) {
    size += block.size();
  }
  return size;
}

inline FloatingPoint HashedChunkedWaveletOctree::getCellValue(
    const Index3D& index) const {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  if (hasBlock(block_index)) {
    const auto& block = getBlock(block_index);
    const CellIndex cell_index =
        computeCellIndexFromBlockIndexAndIndex(block_index, {0, index});
    return block.getCellValue(cell_index);
  }
  return 0.f;
}

inline FloatingPoint HashedChunkedWaveletOctree::getCellValue(
    const OctreeIndex& index) const {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  if (hasBlock(block_index)) {
    const auto& block = getBlock(block_index);
    const CellIndex cell_index =
        computeCellIndexFromBlockIndexAndIndex(block_index, index);
    return block.getCellValue(cell_index);
  }
  return 0.f;
}

inline void HashedChunkedWaveletOctree::setCellValue(const Index3D& index,
                                                     FloatingPoint new_value) {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, {0, index});
  block.setCellValue(cell_index, new_value);
}

inline void HashedChunkedWaveletOctree::addToCellValue(const Index3D& index,
                                                       FloatingPoint update) {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, {0, index});
  block.addToCellValue(cell_index, update);
}

inline bool HashedChunkedWaveletOctree::hasBlock(
    const Index3D& block_index) const {
  return blocks_.count(block_index);
}

inline HashedChunkedWaveletOctree::Block&
HashedChunkedWaveletOctree::getOrAllocateBlock(const Index3D& block_index) {
  if (!hasBlock(block_index)) {
    blocks_.try_emplace(block_index, this);
  }
  return blocks_.at(block_index);
}

inline HashedChunkedWaveletOctree::Block& HashedChunkedWaveletOctree::getBlock(
    const Index3D& block_index) {
  return blocks_.at(block_index);
}

inline const HashedChunkedWaveletOctree::Block&
HashedChunkedWaveletOctree::getBlock(const Index3D& block_index) const {
  return blocks_.at(block_index);
}

inline void HashedChunkedWaveletOctree::forEachLeaf(
    VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn) const {
  for (const auto& [block_index, block] : blocks_) {
    block.forEachLeaf(block_index, visitor_fn);
  }
}

inline HashedChunkedWaveletOctree::CellIndex
HashedChunkedWaveletOctree::computeCellIndexFromBlockIndexAndIndex(
    const HashedChunkedWaveletOctree::BlockIndex& block_index,
    OctreeIndex index) {
  DCHECK_LE(index.height, kTreeHeight);
  const IndexElement height_difference = kTreeHeight - index.height;
  index.position -= int_math::mult_exp2(block_index, height_difference);
  DCHECK((0 <= index.position.array()).all());
  return index;
}

inline FloatingPoint
HashedChunkedWaveletOctree::Block::getTimeSinceLastUpdated() const {
  return (std::chrono::duration<FloatingPoint>(Clock::now() -
                                               last_updated_stamp_))
      .count();
}

inline FloatingPoint HashedChunkedWaveletOctree::Block::getCellValue(
    const OctreeIndex& index) const {
  // Descend the tree chunk by chunk
  const MortonCode morton_code = convert::nodeIndexToMorton(index);
  const NodeChunkType* current_chunk = &chunked_ndtree_.getRootChunk();
  FloatingPoint value = root_scale_coefficient_;
  for (int chunk_top_height = kTreeHeight; index.height < chunk_top_height;
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

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_INL_H_
