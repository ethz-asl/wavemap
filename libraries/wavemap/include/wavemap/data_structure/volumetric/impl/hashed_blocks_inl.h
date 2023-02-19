#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_BLOCKS_INL_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_BLOCKS_INL_H_

#include <limits>
#include <string>
#include <unordered_set>

#include "wavemap/indexing/index_conversions.h"
#include "wavemap/iterator/grid_iterator.h"

namespace wavemap {
template <typename CellT>
void HashedBlocks<CellT>::prune() {
  const Index3D min_local_cell_index = Index3D::Zero();
  const Index3D max_local_cell_index = Index3D::Constant(kCellsPerSide - 1);
  // TODO(victorr): Iterate directly over linear index instead of grid
  const Grid local_grid(min_local_cell_index, max_local_cell_index);

  std::unordered_set<BlockIndex, VoxbloxIndexHash<3>> blocks_to_delete;
  for (const auto& [block_index, block_data] : blocks_) {
    if (!std::any_of(
            local_grid.begin(), local_grid.end(),
            [&block_data = block_data](const auto& cell_index) {
              const FloatingPoint cell_value =
                  block_data[convert::indexToLinearIndex<kCellsPerSide>(
                      cell_index)];
              return cell_value != typename CellT::Specialized{};
            })) {
      blocks_to_delete.template emplace(block_index);
    }
  }
  std::for_each(blocks_to_delete.cbegin(), blocks_to_delete.cend(),
                [&blocks = blocks_](const auto& block_index) {
                  blocks.erase(block_index);
                });
}

template <typename CellT>
Index3D HashedBlocks<CellT>::getMaxIndex() const {
  if (!empty()) {
    Index3D max_block_index =
        Index3D::Constant(std::numeric_limits<IndexElement>::lowest());
    for (const auto& [block_index, block] : blocks_) {
      max_block_index = max_block_index.cwiseMax(block_index);
    }
    return kCellsPerSide * (max_block_index + Index3D::Ones());
  }
  return Index3D::Zero();
}

template <typename CellT>
Index3D HashedBlocks<CellT>::getMinIndex() const {
  if (!empty()) {
    Index3D min_block_index =
        Index3D::Constant(std::numeric_limits<IndexElement>::max());
    for (const auto& [block_index, block] : blocks_) {
      min_block_index = min_block_index.cwiseMin(block_index);
    }
    return kCellsPerSide * min_block_index;
  }
  return Index3D::Zero();
}

template <typename CellT>
FloatingPoint HashedBlocks<CellT>::getCellValue(const Index3D& index) const {
  const CellDataSpecialized* cell_data = accessCellData(index);
  if (cell_data) {
    return static_cast<FloatingPoint>(*cell_data);
  }
  return 0.f;
}

template <typename CellT>
void HashedBlocks<CellT>::setCellValue(const Index3D& index,
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
void HashedBlocks<CellT>::addToCellValue(const Index3D& index,
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
    typename VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn)
    const {
  const Index3D min_local_cell_index = Index3D::Zero();
  const Index3D max_local_cell_index = Index3D::Constant(kCellsPerSide - 1);

  for (const auto& [block_index, block_data] : blocks_) {
    // TODO(victorr): Iterate directly over linear index instead of grid
    for (const Index3D& cell_index :
         Grid(min_local_cell_index, max_local_cell_index)) {
      const FloatingPoint cell_data =
          block_data[convert::indexToLinearIndex<kCellsPerSide>(cell_index)];
      const Index3D index =
          computeIndexFromBlockIndexAndCellIndex(block_index, cell_index);
      const OctreeIndex hierarchical_cell_index =
          convert::indexAndHeightToNodeIndex(index, 0);
      visitor_fn(hierarchical_cell_index, cell_data);
    }
  }
}

template <typename CellT>
bool HashedBlocks<CellT>::save(
    const std::string& /* file_path_prefix */) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
bool HashedBlocks<CellT>::load(const std::string& /* file_path_prefix */) {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
typename CellT::Specialized* HashedBlocks<CellT>::accessCellData(
    const Index3D& index, bool auto_allocate) {
  BlockIndex block_index = computeBlockIndexFromIndex(index);
  auto it = blocks_.find(block_index);
  if (it == blocks_.end()) {
    if (auto_allocate) {
      it = blocks_.template emplace(block_index, Block{}).first;
    } else {
      return nullptr;
    }
  }
  CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, index);
  return &it->second[convert::indexToLinearIndex<kCellsPerSide>(cell_index)];
}

template <typename CellT>
const typename CellT::Specialized* HashedBlocks<CellT>::accessCellData(
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

template <typename CellT>
typename HashedBlocks<CellT>::BlockIndex
HashedBlocks<CellT>::computeBlockIndexFromIndex(const Index3D& index) const {
  return int_math::div_exp2_floor(index, kCellsPerSideLog2);
}

template <typename CellT>
typename HashedBlocks<CellT>::CellIndex
HashedBlocks<CellT>::computeCellIndexFromBlockIndexAndIndex(
    const HashedBlocks::BlockIndex& block_index, const Index3D& index) const {
  const Index3D origin = kCellsPerSide * block_index;
  Index3D cell_index = index - origin;

  DCHECK((0 <= cell_index.array() && cell_index.array() < kCellsPerSide).all())
      << "(Local) cell indices should always be within range [0, "
         "kCellsPerSide[, but computed index equals "
      << cell_index << " instead.";

  return cell_index;
}

template <typename CellT>
Index3D HashedBlocks<CellT>::computeIndexFromBlockIndexAndCellIndex(
    const HashedBlocks::BlockIndex& block_index,
    const HashedBlocks::CellIndex& cell_index) const {
  return kCellsPerSide * block_index + cell_index;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_BLOCKS_INL_H_
