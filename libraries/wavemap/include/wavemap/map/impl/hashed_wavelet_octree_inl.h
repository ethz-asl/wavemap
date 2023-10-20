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
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  const Block* block = getBlock(block_index);
  if (!block) {
    return 0.f;
  }
  const CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, {0, index});
  return block->getCellValue(cell_index);
}

inline FloatingPoint HashedWaveletOctree::getCellValue(
    const OctreeIndex& index) const {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  const Block* block = getBlock(block_index);
  if (!block) {
    return 0.f;
  }
  const CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, index);
  return block->getCellValue(cell_index);
}

inline void HashedWaveletOctree::setCellValue(const Index3D& index,
                                              FloatingPoint new_value) {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, {0, index});
  block.setCellValue(cell_index, new_value);
}

inline void HashedWaveletOctree::addToCellValue(const Index3D& index,
                                                FloatingPoint update) {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, {0, index});
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

inline HashedWaveletOctree::CellIndex
HashedWaveletOctree::computeCellIndexFromBlockIndexAndIndex(
    const HashedWaveletOctree::BlockIndex& block_index,
    OctreeIndex index) const {
  DCHECK_LE(index.height, config_.tree_height);
  const IndexElement height_difference = config_.tree_height - index.height;
  index.position -= int_math::mult_exp2(block_index, height_difference);
  DCHECK((0 <= index.position.array()).all());
  return index;
}
}  // namespace wavemap

#endif  // WAVEMAP_MAP_IMPL_HASHED_WAVELET_OCTREE_INL_H_
