#ifndef WAVEMAP_MAP_IMPL_HASHED_BLOCKS_INL_H_
#define WAVEMAP_MAP_IMPL_HASHED_BLOCKS_INL_H_

#include <limits>
#include <string>
#include <unordered_set>

#include "wavemap/indexing/index_conversions.h"
#include "wavemap/utils/iterate/grid_iterator.h"

namespace wavemap {
inline FloatingPoint HashedBlocks::getCellValue(const Index3D& index) const {
  const BlockIndex block_index = indexToBlockIndex(index);
  const auto* block = getBlock(block_index);
  if (!block) {
    return default_value_;
  }
  const CellIndex cell_index = indexToCellIndex(index);
  return block->at(cell_index);
}

inline FloatingPoint& HashedBlocks::getCellValueRef(const Index3D& index) {
  const BlockIndex block_index = indexToBlockIndex(index);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = indexToCellIndex(index);
  return block.at(cell_index);
}

inline void HashedBlocks::setCellValue(const Index3D& index,
                                       FloatingPoint new_value) {
  const BlockIndex block_index = indexToBlockIndex(index);
  Block& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = indexToCellIndex(index);
  block.at(cell_index) = new_value;
}

inline void HashedBlocks::addToCellValue(const Index3D& index,
                                         FloatingPoint update) {
  const BlockIndex block_index = indexToBlockIndex(index);
  Block& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = indexToCellIndex(index);
  FloatingPoint& cell_data = block.at(cell_index);
  cell_data = clampedAdd(cell_data, update);
}

inline bool HashedBlocks::hasBlock(const Index3D& block_index) const {
  return block_map_.hasBlock(block_index);
}

inline HashedBlocks::Block* HashedBlocks::getBlock(const Index3D& block_index) {
  return block_map_.getBlock(block_index);
}

inline const HashedBlocks::Block* HashedBlocks::getBlock(
    const Index3D& block_index) const {
  return block_map_.getBlock(block_index);
}

inline HashedBlocks::Block& HashedBlocks::getOrAllocateBlock(
    const Index3D& block_index) {
  return block_map_.getOrAllocateBlock(block_index, default_value_);
}

inline Index3D HashedBlocks::indexToBlockIndex(const Index3D& index) {
  return convert::indexToBlockIndex(index, kCellsPerSideLog2);
}

inline Index3D HashedBlocks::indexToCellIndex(const Index3D& index) {
  return int_math::div_exp2_floor_remainder(index, kCellsPerSideLog2);
}

inline Index3D HashedBlocks::cellAndBlockIndexToIndex(
    const Index3D& block_index, const Index3D& cell_index) {
  return kCellsPerSide * block_index + cell_index;
}
}  // namespace wavemap

#endif  // WAVEMAP_MAP_IMPL_HASHED_BLOCKS_INL_H_
