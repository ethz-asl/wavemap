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
    blocks_.try_emplace(block_index, config_.tree_height, config_.min_log_odds,
                        config_.max_log_odds);
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
    OctreeIndex index) const {
  DCHECK_LE(index.height, config_.tree_height);
  const IndexElement height_difference = config_.tree_height - index.height;
  index.position -= int_math::mult_exp2(block_index, height_difference);
  DCHECK((0 <= index.position.array()).all());
  return index;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_INL_H_
