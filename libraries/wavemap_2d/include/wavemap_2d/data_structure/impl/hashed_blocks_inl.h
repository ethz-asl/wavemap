#ifndef WAVEMAP_2D_DATA_STRUCTURE_IMPL_HASHED_BLOCKS_INL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_IMPL_HASHED_BLOCKS_INL_H_

#include <limits>
#include <string>
#include <unordered_set>

#include <wavemap_common/iterator/grid_iterator.h>

#include "wavemap_2d/data_structure/dense_grid.h"

namespace wavemap {
template <typename CellT>
void HashedBlocks<CellT>::prune() {
  const Index2D min_local_cell_index = Index2D::Zero();
  const Index2D max_local_cell_index = Index2D::Constant(kCellsPerSide - 1);
  const Grid local_grid(min_local_cell_index, max_local_cell_index);

  std::unordered_set<BlockIndex, VoxbloxIndexHash> blocks_to_delete;
  for (const auto& [block_index, block_data] : blocks_) {
    if (!std::any_of(local_grid.begin(), local_grid.end(),
                     [&block_data = block_data](const auto& cell_index) {
                       const FloatingPoint cell_value =
                           block_data(cell_index.x(), cell_index.y());
                       return cell_value != typename CellT::Specialized{};
                     })) {
      blocks_to_delete.template emplace(block_index);
    }
  }
  std::for_each(blocks_to_delete.begin(), blocks_to_delete.end(),
                [&blocks = blocks_](const auto& block_index) {
                  blocks.erase(block_index);
                });
}

template <typename CellT>
Index2D HashedBlocks<CellT>::getMinIndex() const {
  if (!empty()) {
    Index2D min_block_index =
        Index2D::Constant(std::numeric_limits<IndexElement>::max());
    for (const auto& [block_index, block] : blocks_) {
      min_block_index = min_block_index.cwiseMin(block_index);
    }
    return kCellsPerSide * min_block_index;
  }
  return Index2D::Zero();
}

template <typename CellT>
Index2D HashedBlocks<CellT>::getMaxIndex() const {
  if (!empty()) {
    Index2D max_block_index =
        Index2D::Constant(std::numeric_limits<IndexElement>::lowest());
    for (const auto& [block_index, block] : blocks_) {
      max_block_index = max_block_index.cwiseMax(block_index);
    }
    return kCellsPerSide * (max_block_index + Index2D::Ones());
  }
  return Index2D::Zero();
}

template <typename CellT>
FloatingPoint HashedBlocks<CellT>::getCellValue(const Index2D& index) const {
  const CellDataSpecialized* cell_data = accessCellData(index);
  if (cell_data) {
    return static_cast<FloatingPoint>(*cell_data);
  }
  return 0.f;
}

template <typename CellT>
void HashedBlocks<CellT>::setCellValue(const Index2D& index,
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

template <typename CellT>
void HashedBlocks<CellT>::addToCellValue(const Index2D& index,
                                         FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  CellDataSpecialized* cell_data = accessCellData(index, kAutoAllocate);
  if (cell_data) {
    *cell_data = CellT::add(*cell_data, update);
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellT>
void HashedBlocks<CellT>::forEachLeaf(
    VolumetricDataStructure2D::IndexedLeafVisitorFunction visitor_fn) const {
  const Index2D min_local_cell_index = Index2D::Zero();
  const Index2D max_local_cell_index = Index2D::Constant(kCellsPerSide - 1);

  for (const auto& [block_index, block_data] : blocks_) {
    for (const Index2D& cell_index :
         Grid(min_local_cell_index, max_local_cell_index)) {
      const FloatingPoint cell_data =
          block_data(cell_index.x(), cell_index.y());
      const Index2D index =
          computeIndexFromBlockIndexAndCellIndex(block_index, cell_index);
      const QuadtreeIndex hierarchical_cell_index =
          convert::indexAndHeightToNodeIndex(index, 0);
      visitor_fn(hierarchical_cell_index, cell_data);
    }
  }
}

template <typename CellT>
cv::Mat HashedBlocks<CellT>::getImage(bool use_color) const {
  DenseGrid<CellT> dense_grid(min_cell_width_);
  for (const Index2D& index : Grid(getMinIndex(), getMaxIndex())) {
    const FloatingPoint cell_value = getCellValue(index);
    dense_grid.setCellValue(index, cell_value);
  }
  return dense_grid.getImage(use_color);
}

template <typename CellT>
bool HashedBlocks<CellT>::save(const std::string& /* file_path_prefix */,
                               bool /* use_floating_precision */) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
bool HashedBlocks<CellT>::load(const std::string& /* file_path_prefix */,
                               bool /* used_floating_precision */) {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
typename CellT::Specialized* HashedBlocks<CellT>::accessCellData(
    const Index2D& index, bool auto_allocate) {
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

template <typename CellT>
const typename CellT::Specialized* HashedBlocks<CellT>::accessCellData(
    const Index2D& index) const {
  BlockIndex block_index = computeBlockIndexFromIndex(index);
  const auto& it = blocks_.find(block_index);
  if (it != blocks_.end()) {
    CellIndex cell_index =
        computeCellIndexFromBlockIndexAndIndex(block_index, index);
    return &it->second.coeff(cell_index.x(), cell_index.y());
  }
  return nullptr;
}

template <typename CellT>
typename HashedBlocks<CellT>::BlockIndex
HashedBlocks<CellT>::computeBlockIndexFromIndex(const Index2D& index) const {
  return {
      std::floor(kCellsPerSideInv * static_cast<FloatingPoint>(index.x())),
      std::floor(kCellsPerSideInv * static_cast<FloatingPoint>(index.y())),
  };
}

template <typename CellT>
typename HashedBlocks<CellT>::CellIndex
HashedBlocks<CellT>::computeCellIndexFromBlockIndexAndIndex(
    const HashedBlocks::BlockIndex& block_index, const Index2D& index) const {
  const Index2D origin = kCellsPerSide * block_index;
  Index2D cell_index = index - origin;

  DCHECK((0 <= cell_index.array() && cell_index.array() < kCellsPerSide).all())
      << "(Local) cell indices should always be within range [0, "
         "kCellsPerSide[, but computed index equals "
      << cell_index << " instead.";

  return cell_index;
}

template <typename CellT>
Index2D HashedBlocks<CellT>::computeIndexFromBlockIndexAndCellIndex(
    const HashedBlocks::BlockIndex& block_index,
    const HashedBlocks::CellIndex& cell_index) const {
  return kCellsPerSide * block_index + cell_index;
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_IMPL_HASHED_BLOCKS_INL_H_
