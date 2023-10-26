#ifndef WAVEMAP_UTILS_SDF_IMPL_VECTOR_DISTANCE_FIELD_INL_H_
#define WAVEMAP_UTILS_SDF_IMPL_VECTOR_DISTANCE_FIELD_INL_H_

namespace wavemap {
inline const VectorDistance& VectorDistanceField::getCellValue(
    const wavemap::Index3D& index) const {
  const BlockIndex block_index = indexToBlockIndex(index);
  const auto* block = getBlock(block_index);
  if (!block) {
    return default_value_;
  }
  const CellIndex cell_index = indexToCellIndex(index);
  return block->at(cell_index);
}

inline VectorDistance& VectorDistanceField::getCellValueRef(
    const Index3D& index) {
  const BlockIndex block_index = indexToBlockIndex(index);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = indexToCellIndex(index);
  return block.at(cell_index);
}

inline VectorDistanceField::Block* VectorDistanceField::getBlock(
    const Index3D& block_index) {
  return block_map_.getBlock(block_index);
}

inline const VectorDistanceField::Block* VectorDistanceField::getBlock(
    const Index3D& block_index) const {
  return block_map_.getBlock(block_index);
}

inline VectorDistanceField::Block& VectorDistanceField::getOrAllocateBlock(
    const Index3D& block_index) {
  return block_map_.getOrAllocateBlock(block_index, default_value_);
}

template <typename IndexedLeafVisitorFunction>
void VectorDistanceField::forEachLeaf(
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

inline Index3D VectorDistanceField::indexToBlockIndex(const Index3D& index) {
  return convert::indexToBlockIndex(index, kCellsPerSideLog2);
}

inline Index3D VectorDistanceField::indexToCellIndex(const Index3D& index) {
  return int_math::div_exp2_floor_remainder(index, kCellsPerSideLog2);
}

inline Index3D VectorDistanceField::cellAndBlockIndexToIndex(
    const Index3D& block_index, const Index3D& cell_index) {
  return kCellsPerSide * block_index + cell_index;
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_SDF_IMPL_VECTOR_DISTANCE_FIELD_INL_H_
