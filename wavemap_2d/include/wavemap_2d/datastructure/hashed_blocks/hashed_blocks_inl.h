#ifndef WAVEMAP_2D_DATASTRUCTURE_HASHED_BLOCKS_HASHED_BLOCKS_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_HASHED_BLOCKS_HASHED_BLOCKS_INL_H_

#include <limits>
#include <string>

#include "wavemap_2d/datastructure/dense_grid/dense_grid.h"
#include "wavemap_2d/iterator/grid_iterator.h"

namespace wavemap_2d {
template <typename CellTypeT>
Index HashedBlocks<CellTypeT>::getMinIndex() const {
  if (!empty()) {
    Index min_block_index =
        Index::Constant(std::numeric_limits<IndexElement>::max());
    for (const auto& [block_index, block] : blocks_) {
      min_block_index = min_block_index.cwiseMin(block_index);
    }
    return kCellsPerSide * min_block_index;
  }
  return Index::Zero();
}

template <typename CellTypeT>
Index HashedBlocks<CellTypeT>::getMaxIndex() const {
  if (!empty()) {
    Index max_block_index =
        Index::Constant(std::numeric_limits<IndexElement>::lowest());
    for (const auto& [block_index, block] : blocks_) {
      max_block_index = max_block_index.cwiseMax(block_index);
    }
    return kCellsPerSide * (max_block_index + Index::Ones());
  }
  return Index::Zero();
}

template <typename CellTypeT>
bool HashedBlocks<CellTypeT>::hasCell(const Index& index) const {
  return blocks_.count(computeBlockIndexFromIndex(index));
}

template <typename CellTypeT>
FloatingPoint HashedBlocks<CellTypeT>::getCellValue(const Index& index) const {
  const CellDataSpecialized* cell_data = accessCellData(index);
  if (cell_data) {
    return static_cast<FloatingPoint>(*cell_data);
  }
  return 0.f;
}

template <typename CellTypeT>
void HashedBlocks<CellTypeT>::setCellValue(const Index& index,
                                           FloatingPoint new_value) {
  constexpr bool kAutoAllocate = true;
  CellDataSpecialized* cell_data = accessCellData(index, kAutoAllocate);
  if (cell_data) {
    // TODO(victorr): Decide whether truncation should be applied here as well
    *cell_data = new_value;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellTypeT>
void HashedBlocks<CellTypeT>::addToCellValue(const Index& index,
                                             FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  CellDataSpecialized* cell_data = accessCellData(index, kAutoAllocate);
  if (cell_data) {
    *cell_data = CellTypeT::add(*cell_data, update);
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellTypeT>
cv::Mat HashedBlocks<CellTypeT>::getImage(bool use_color) const {
  DenseGrid<CellTypeT> dense_grid(resolution_);
  for (const Index& index : Grid(getMinIndex(), getMaxIndex())) {
    const FloatingPoint cell_value = getCellValue(index);
    dense_grid.setCellValue(index, cell_value);
  }
  return dense_grid.getImage(use_color);
}

template <typename CellTypeT>
bool HashedBlocks<CellTypeT>::save(const std::string& /* file_path_prefix */,
                                   bool /* use_floating_precision */) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellTypeT>
bool HashedBlocks<CellTypeT>::load(const std::string& /* file_path_prefix */,
                                   bool /* used_floating_precision */) {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellTypeT>
typename CellTypeT::Specialized* HashedBlocks<CellTypeT>::accessCellData(
    const Index& index, bool auto_allocate) {
  BlockIndex block_index = computeBlockIndexFromIndex(index);
  auto it = blocks_.find(block_index);
  if (it == blocks_.end()) {
    if (auto_allocate) {
      it = blocks_.template emplace(block_index, Block::Zero()).first;
    } else {
      return nullptr;
    }
  }
  CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, index);
  return &it->second.coeffRef(cell_index.x(), cell_index.y());
}

template <typename CellTypeT>
const typename CellTypeT::Specialized* HashedBlocks<CellTypeT>::accessCellData(
    const Index& index) const {
  BlockIndex block_index = computeBlockIndexFromIndex(index);
  const auto& it = blocks_.find(block_index);
  if (it != blocks_.end()) {
    CellIndex cell_index =
        computeCellIndexFromBlockIndexAndIndex(block_index, index);
    return &it->second.coeff(cell_index.x(), cell_index.y());
  }
  return nullptr;
}

template <typename CellTypeT>
typename HashedBlocks<CellTypeT>::BlockIndex
HashedBlocks<CellTypeT>::computeBlockIndexFromIndex(const Index& index) const {
  return {
      std::floor(kCellsPerSideInv * static_cast<FloatingPoint>(index.x())),
      std::floor(kCellsPerSideInv * static_cast<FloatingPoint>(index.y())),
  };
}

template <typename CellTypeT>
typename HashedBlocks<CellTypeT>::CellIndex
HashedBlocks<CellTypeT>::computeCellIndexFromBlockIndexAndIndex(
    const HashedBlocks::BlockIndex& block_index, const Index& index) const {
  Index origin = kCellsPerSide * block_index;
  Index cell_index = index - origin;

  DCHECK((0 <= cell_index.array() && cell_index.array() < kCellsPerSide).all())
      << "(Local) cell indices should always be within range [0, "
         "kCellsPerSide[, but computed index equals "
      << cell_index << " instead.";

  return cell_index;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_HASHED_BLOCKS_HASHED_BLOCKS_INL_H_
