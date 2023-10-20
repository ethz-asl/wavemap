#ifndef WAVEMAP_MAP_IMPL_HASHED_BLOCKS_INL_H_
#define WAVEMAP_MAP_IMPL_HASHED_BLOCKS_INL_H_

#include <limits>
#include <string>
#include <unordered_set>

#include "wavemap/indexing/index_conversions.h"
#include "wavemap/utils/iterate/grid_iterator.h"

namespace wavemap {
inline FloatingPoint HashedBlocks::getCellValue(const Index3D& index) const {
  const BlockIndex block_index =
      convert::indexToBlockIndex<kCellsPerSide>(index);
  const auto* block = getBlock(block_index);
  if (!block) {
    return default_value_;
  }
  const CellIndex cell_index = convert::indexToCellIndex<kCellsPerSide>(index);
  return block->at(cell_index);
}

inline FloatingPoint& HashedBlocks::getCellValueRef(const Index3D& index) {
  const BlockIndex block_index =
      convert::indexToBlockIndex<kCellsPerSide>(index);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = convert::indexToCellIndex<kCellsPerSide>(index);
  return block.at(cell_index);
}

inline void HashedBlocks::setCellValue(const Index3D& index,
                                       FloatingPoint new_value) {
  const BlockIndex block_index =
      convert::indexToBlockIndex<kCellsPerSide>(index);
  Block& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = convert::indexToCellIndex<kCellsPerSide>(index);
  block.at(cell_index) = new_value;
}

inline void HashedBlocks::addToCellValue(const Index3D& index,
                                         FloatingPoint update) {
  const BlockIndex block_index =
      convert::indexToBlockIndex<kCellsPerSide>(index);
  Block& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = convert::indexToCellIndex<kCellsPerSide>(index);
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
}  // namespace wavemap

#endif  // WAVEMAP_MAP_IMPL_HASHED_BLOCKS_INL_H_
