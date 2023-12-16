#ifndef WAVEMAP_UTILS_QUERY_IMPL_QUERY_ACCELERATOR_INL_H_
#define WAVEMAP_UTILS_QUERY_IMPL_QUERY_ACCELERATOR_INL_H_

#include <utility>

namespace wavemap {
template <typename BlockDataT, int dim>
BlockDataT* QueryAccelerator<SpatialHash<BlockDataT, dim>>::getBlock(
    const Index<dim>& block_index) {
  if (block_index != last_block_index_) {
    last_block_index_ = block_index;
    last_block_ = spatial_hash_.getBlock(block_index);
  }
  return last_block_;
}

template <typename BlockDataT, int dim>
template <typename... DefaultArgs>
BlockDataT& QueryAccelerator<SpatialHash<BlockDataT, dim>>::getOrAllocateBlock(
    const Index<dim>& block_index, DefaultArgs&&... args) {
  if (block_index != last_block_index_ || !last_block_) {
    last_block_index_ = block_index;
    last_block_ = &spatial_hash_.getOrAllocateBlock(
        block_index, std::forward<DefaultArgs>(args)...);
  }
  return *last_block_;
}

template <typename CellDataT, int dim>
typename QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::BlockType*
QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::getBlock(
    const Index<dim>& block_index) {
  if (block_index != last_block_index_) {
    last_block_index_ = block_index;
    last_block_ = ndtree_block_hash_.getBlock(block_index);
  }
  return last_block_;
}

template <typename CellDataT, int dim>
template <typename... DefaultArgs>
typename QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::BlockType&
QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::getOrAllocateBlock(
    const Index<dim>& block_index, DefaultArgs&&... args) {
  if (block_index != last_block_index_ || !last_block_) {
    last_block_index_ = block_index;
    last_block_ = &ndtree_block_hash_.getOrAllocateBlock(
        block_index, std::forward<DefaultArgs>(args)...);
  }
  return *last_block_;
}

}  // namespace wavemap

#endif  // WAVEMAP_UTILS_QUERY_IMPL_QUERY_ACCELERATOR_INL_H_
