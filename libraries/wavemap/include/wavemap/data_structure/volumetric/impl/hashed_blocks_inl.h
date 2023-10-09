#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_BLOCKS_INL_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_BLOCKS_INL_H_

#include <limits>
#include <string>
#include <unordered_set>

#include "wavemap/indexing/index_conversions.h"
#include "wavemap/utils/iterate/grid_iterator.h"

namespace wavemap {
inline FloatingPoint HashedBlocks::getCellValue(const Index3D& index) const {
  const FloatingPoint* cell_data = accessCellData(index);
  if (cell_data) {
    return static_cast<FloatingPoint>(*cell_data);
  }
  return 0.f;
}

inline void HashedBlocks::setCellValue(const Index3D& index,
                                       FloatingPoint new_value) {
  constexpr bool kAutoAllocate = true;
  FloatingPoint* cell_data = accessCellData(index, kAutoAllocate);
  if (cell_data) {
    // TODO(victorr): Decide whether truncation should be applied here as well
    *cell_data = new_value;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

inline void HashedBlocks::addToCellValue(const Index3D& index,
                                         FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  FloatingPoint* cell_data = accessCellData(index, kAutoAllocate);
  if (cell_data) {
    *cell_data = clampedAdd(*cell_data, update);
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

inline FloatingPoint* HashedBlocks::accessCellData(const Index3D& index,
                                                   bool auto_allocate) {
  BlockIndex block_index = computeBlockIndexFromIndex(index);
  auto it = blocks_.find(block_index);
  if (it == blocks_.end()) {
    if (auto_allocate) {
      it = blocks_.emplace(block_index, Block{}).first;
    } else {
      return nullptr;
    }
  }
  CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, index);
  return &it->second[convert::indexToLinearIndex<kCellsPerSide>(cell_index)];
}

inline const FloatingPoint* HashedBlocks::accessCellData(
    const Index3D& index) const {
  BlockIndex block_index = computeBlockIndexFromIndex(index);
  const auto& it = blocks_.find(block_index);
  if (it != blocks_.end()) {
    CellIndex cell_index =
        computeCellIndexFromBlockIndexAndIndex(block_index, index);
    return &it->second[convert::indexToLinearIndex<kCellsPerSide>(cell_index)];
  }
  return nullptr;
}

inline HashedBlocks::BlockIndex HashedBlocks::computeBlockIndexFromIndex(
    const Index3D& index) {
  return int_math::div_exp2_floor(index, kCellsPerSideLog2);
}

inline HashedBlocks::CellIndex
HashedBlocks::computeCellIndexFromBlockIndexAndIndex(
    const HashedBlocks::BlockIndex& block_index, const Index3D& index) {
  const Index3D origin = kCellsPerSide * block_index;
  Index3D cell_index = index - origin;

  DCHECK((0 <= cell_index.array() && cell_index.array() < kCellsPerSide).all())
      << "(Local) cell indices should always be within range [0, "
         "kCellsPerSide[, but computed index equals "
      << cell_index << " instead.";

  return cell_index;
}

inline Index3D HashedBlocks::computeIndexFromBlockIndexAndCellIndex(
    const HashedBlocks::BlockIndex& block_index,
    const HashedBlocks::CellIndex& cell_index) {
  return kCellsPerSide * block_index + cell_index;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_BLOCKS_INL_H_
