#ifndef WAVEMAP_MAP_IMPL_HASHED_WAVELET_OCTREE_INL_H_
#define WAVEMAP_MAP_IMPL_HASHED_WAVELET_OCTREE_INL_H_

#include "wavemap/indexing/index_conversions.h"

namespace wavemap {
inline size_t HashedWaveletOctree::size() const {
  size_t size = 0u;
  forEachBlock([&size](const BlockIndex& /*block_index*/, const Block& block) {
    size += block.size();
  });
  return size;
}

inline FloatingPoint HashedWaveletOctree::getCellValue(
    const Index3D& index) const {
  const BlockIndex block_index =
      convert::indexToBlockIndex(index, config_.tree_height);
  const Block* block = getBlock(block_index);
  if (!block) {
    return 0.f;
  }
  const CellIndex cell_index = indexToCellIndex({0, index});
  return block->getCellValue(cell_index);
}

inline FloatingPoint HashedWaveletOctree::getCellValue(
    const OctreeIndex& index) const {
  const BlockIndex block_index = indexToBlockIndex(index);
  const Block* block = getBlock(block_index);
  if (!block) {
    return 0.f;
  }
  const CellIndex cell_index = indexToCellIndex(index);
  return block->getCellValue(cell_index);
}

inline void HashedWaveletOctree::setCellValue(const Index3D& index,
                                              FloatingPoint new_value) {
  const BlockIndex block_index =
      convert::indexToBlockIndex(index, config_.tree_height);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = indexToCellIndex({0, index});
  block.setCellValue(cell_index, new_value);
}

inline void HashedWaveletOctree::addToCellValue(const Index3D& index,
                                                FloatingPoint update) {
  const BlockIndex block_index =
      convert::indexToBlockIndex(index, config_.tree_height);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = indexToCellIndex({0, index});
  block.addToCellValue(cell_index, update);
}

inline bool HashedWaveletOctree::hasBlock(const Index3D& block_index) const {
  return block_map_.hasBlock(block_index);
}

inline HashedWaveletOctree::Block* HashedWaveletOctree::getBlock(
    const Index3D& block_index) {
  return block_map_.getBlock(block_index);
}

inline const HashedWaveletOctree::Block* HashedWaveletOctree::getBlock(
    const Index3D& block_index) const {
  return block_map_.getBlock(block_index);
}

inline HashedWaveletOctree::Block& HashedWaveletOctree::getOrAllocateBlock(
    const Index3D& block_index) {
  return block_map_.getOrAllocateBlock(block_index, config_.tree_height,
                                       config_.min_log_odds,
                                       config_.max_log_odds);
}

inline HashedWaveletOctree::BlockIndex HashedWaveletOctree::indexToBlockIndex(
    const OctreeIndex& node_index) const {
  const Index3D index = convert::nodeIndexToMinCornerIndex(node_index);
  return convert::indexToBlockIndex(index, config_.tree_height);
}

inline HashedWaveletOctree::CellIndex HashedWaveletOctree::indexToCellIndex(
    OctreeIndex index) const {
  DCHECK_LE(index.height, config_.tree_height);
  const IndexElement height_difference = config_.tree_height - index.height;
  index.position =
      int_math::div_exp2_floor_remainder(index.position, height_difference);
  return index;
}
}  // namespace wavemap

#endif  // WAVEMAP_MAP_IMPL_HASHED_WAVELET_OCTREE_INL_H_
