#ifndef WAVEMAP_DATA_STRUCTURE_IMPL_DENSE_BLOCK_HASH_INL_H_
#define WAVEMAP_DATA_STRUCTURE_IMPL_DENSE_BLOCK_HASH_INL_H_

namespace wavemap {
template <typename CellDataT, int dim, unsigned int cells_per_side>
bool DenseBlockHash<CellDataT, dim, cells_per_side>::hasBlock(
    const Index<dim>& block_index) const {
  return block_map_.hasBlock(block_index);
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
bool DenseBlockHash<CellDataT, dim, cells_per_side>::eraseBlock(
    const Index<dim>& block_index) {
  return block_map_.eraseBlock(block_index);
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
template <typename IndexedBlockVisitor>
void DenseBlockHash<CellDataT, dim, cells_per_side>::eraseBlockIf(
    IndexedBlockVisitor indicator_fn) {
  block_map_.eraseBlockIf(indicator_fn);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline DenseGrid<CellDataT, dim, cells_per_side>*
DenseBlockHash<CellDataT, dim, cells_per_side>::getBlock(
    const Index<dim>& block_index) {
  return block_map_.getBlock(block_index);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline const DenseGrid<CellDataT, dim, cells_per_side>*
DenseBlockHash<CellDataT, dim, cells_per_side>::getBlock(
    const Index<dim>& block_index) const {
  return block_map_.getBlock(block_index);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline DenseGrid<CellDataT, dim, cells_per_side>&
DenseBlockHash<CellDataT, dim, cells_per_side>::getOrAllocateBlock(
    const Index<dim>& block_index) {
  return block_map_.getOrAllocateBlock(block_index, default_value_);
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
bool DenseBlockHash<CellDataT, dim, cells_per_side>::hasValue(
    const Index<dim>& index) const {
  return getValue(index);
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
CellDataT* DenseBlockHash<CellDataT, dim, cells_per_side>::getValue(
    const Index<dim>& index) {
  return const_cast<CellDataT*>(std::as_const(*this).getValue(index));
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
const CellDataT* DenseBlockHash<CellDataT, dim, cells_per_side>::getValue(
    const Index<dim>& index) const {
  const Index<dim> block_index = indexToBlockIndex(index);
  if (const Block* block = getBlock(block_index); block) {
    const Index<dim> cell_index = indexToCellIndex(index);
    return &block->at(cell_index);
  }
  return nullptr;
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline CellDataT&
DenseBlockHash<CellDataT, dim, cells_per_side>::getOrAllocateValue(
    const Index<dim>& index) {
  const Index<dim> block_index = indexToBlockIndex(index);
  Block& block = getOrAllocateBlock(block_index);
  const Index<dim> cell_index = indexToCellIndex(index);
  return block.at(cell_index);
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
const CellDataT&
DenseBlockHash<CellDataT, dim, cells_per_side>::getValueOrDefault(
    const Index<dim>& index) const {
  if (const CellDataT* value = getValue(index); value) {
    return *value;
  }
  return default_value_;
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
bool DenseBlockHash<CellDataT, dim, cells_per_side>::equalsDefaultValue(
    const CellDataT& value) const {
  return value == default_value_;
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
template <typename IndexedBlockVisitor>
void DenseBlockHash<CellDataT, dim, cells_per_side>::forEachBlock(
    IndexedBlockVisitor visitor_fn) {
  block_map_.forEachBlock(visitor_fn);
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
template <typename IndexedBlockVisitor>
void DenseBlockHash<CellDataT, dim, cells_per_side>::forEachBlock(
    IndexedBlockVisitor visitor_fn) const {
  block_map_.forEachBlock(visitor_fn);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
template <typename IndexedLeafVisitorFunction>
void DenseBlockHash<CellDataT, dim, cells_per_side>::forEachLeaf(
    IndexedLeafVisitorFunction visitor_fn) {
  block_map_.forEachBlock(
      [&visitor_fn](const Index<dim>& block_index, Block& block) {
        for (LinearIndex cell_idx = 0u; cell_idx < Block::kCellsPerBlock;
             ++cell_idx) {
          const Index<dim> cell_index =
              convert::linearIndexToIndex<kCellsPerSide, kDim>(cell_idx);
          const Index<dim> index =
              cellAndBlockIndexToIndex(block_index, cell_index);
          CellDataT& cell_data = block[cell_idx];
          std::invoke(visitor_fn, index, cell_data);
        }
      });
}

template <typename CellDataT, int dim, unsigned cells_per_side>
template <typename IndexedLeafVisitorFunction>
void DenseBlockHash<CellDataT, dim, cells_per_side>::forEachLeaf(
    IndexedLeafVisitorFunction visitor_fn) const {
  block_map_.forEachBlock(
      [&visitor_fn](const Index<dim>& block_index, const Block& block) {
        for (LinearIndex cell_idx = 0u; cell_idx < Block::kCellsPerBlock;
             ++cell_idx) {
          const Index<dim> cell_index =
              convert::linearIndexToIndex<kCellsPerSide, kDim>(cell_idx);
          const Index<dim> index =
              cellAndBlockIndexToIndex(block_index, cell_index);
          const CellDataT& cell_data = block[cell_idx];
          std::invoke(visitor_fn, index, cell_data);
        }
      });
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline Index<dim>
DenseBlockHash<CellDataT, dim, cells_per_side>::indexToBlockIndex(
    const Index<dim>& index) {
  return convert::indexToBlockIndex(index, kCellsPerSideLog2);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline Index<dim>
DenseBlockHash<CellDataT, dim, cells_per_side>::indexToCellIndex(
    const Index<dim>& index) {
  return int_math::div_exp2_floor_remainder(index, kCellsPerSideLog2);
}

template <typename CellDataT, int dim, unsigned cells_per_side>
inline Index<dim>
DenseBlockHash<CellDataT, dim, cells_per_side>::cellAndBlockIndexToIndex(
    const Index<dim>& block_index, const Index<dim>& cell_index) {
  return kCellsPerSide * block_index + cell_index;
}

}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_IMPL_DENSE_BLOCK_HASH_INL_H_
