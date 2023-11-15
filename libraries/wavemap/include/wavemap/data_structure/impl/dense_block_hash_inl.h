#ifndef WAVEMAP_DATA_STRUCTURE_IMPL_DENSE_BLOCK_HASH_INL_H_
#define WAVEMAP_DATA_STRUCTURE_IMPL_DENSE_BLOCK_HASH_INL_H_

namespace wavemap {
template <typename CellDataT, int dim, unsigned cells_per_side>
inline DenseGrid<CellDataT, dim, cells_per_side>*
DenseBlockHash<CellDataT, dim, cells_per_side>::getBlock(
    const Index3D& block_index) {
  return block_map_.getBlock(block_index);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline const DenseGrid<CellDataT, dim, cells_per_side>*
DenseBlockHash<CellDataT, dim, cells_per_side>::getBlock(
    const Index3D& block_index) const {
  return block_map_.getBlock(block_index);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline DenseGrid<CellDataT, dim, cells_per_side>&
DenseBlockHash<CellDataT, dim, cells_per_side>::getOrAllocateBlock(
    const Index3D& block_index) {
  return block_map_.getOrAllocateBlock(block_index, default_value_);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline const CellDataT&
DenseBlockHash<CellDataT, dim, cells_per_side>::getCellValue(
    const wavemap::Index3D& index) const {
  const BlockIndex block_index = indexToBlockIndex(index);
  const auto* block = getBlock(block_index);
  if (!block) {
    return default_value_;
  }
  const CellIndex cell_index = indexToCellIndex(index);
  return block->at(cell_index);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline CellDataT&
DenseBlockHash<CellDataT, dim, cells_per_side>::getOrAllocateCellValue(
    const Index3D& index) {
  const BlockIndex block_index = indexToBlockIndex(index);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = indexToCellIndex(index);
  return block.at(cell_index);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
template <typename IndexedLeafVisitorFunction>
void DenseBlockHash<CellDataT, dim, cells_per_side>::forEachLeaf(
    IndexedLeafVisitorFunction visitor_fn) {
  block_map_.forEachBlock([&visitor_fn](const BlockIndex& block_index,
                                        const Block& block) {
    for (LinearIndex cell_idx = 0u; cell_idx < Block::kCellsPerBlock;
         ++cell_idx) {
      const Index3D cell_index =
          convert::linearIndexToIndex<kCellsPerSide, kDim>(cell_idx);
      const Index3D index = cellAndBlockIndexToIndex(block_index, cell_index);
      auto& cell_data = block[cell_idx];
      std::invoke(visitor_fn, index, cell_data);
    }
  });
}

template <typename CellDataT, int dim, unsigned cells_per_side>
template <typename IndexedLeafVisitorFunction>
void DenseBlockHash<CellDataT, dim, cells_per_side>::forEachLeaf(
    IndexedLeafVisitorFunction visitor_fn) const {
  block_map_.forEachBlock([&visitor_fn](const BlockIndex& block_index,
                                        const Block& block) {
    for (LinearIndex cell_idx = 0u; cell_idx < Block::kCellsPerBlock;
         ++cell_idx) {
      const Index3D cell_index =
          convert::linearIndexToIndex<kCellsPerSide, kDim>(cell_idx);
      const Index3D index = cellAndBlockIndexToIndex(block_index, cell_index);
      const auto& cell_data = block[cell_idx];
      std::invoke(visitor_fn, index, cell_data);
    }
  });
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline Index3D
DenseBlockHash<CellDataT, dim, cells_per_side>::indexToBlockIndex(
    const Index3D& index) {
  return convert::indexToBlockIndex(index, kCellsPerSideLog2);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline Index3D DenseBlockHash<CellDataT, dim, cells_per_side>::indexToCellIndex(
    const Index3D& index) {
  return int_math::div_exp2_floor_remainder(index, kCellsPerSideLog2);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline Index3D
DenseBlockHash<CellDataT, dim, cells_per_side>::cellAndBlockIndexToIndex(
    const Index3D& block_index, const Index3D& cell_index) {
  return kCellsPerSide * block_index + cell_index;
}

}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_IMPL_DENSE_BLOCK_HASH_INL_H_
