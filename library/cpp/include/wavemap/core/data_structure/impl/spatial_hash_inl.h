#ifndef WAVEMAP_CORE_DATA_STRUCTURE_IMPL_SPATIAL_HASH_INL_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_IMPL_SPATIAL_HASH_INL_H_

#include <limits>
#include <utility>

namespace wavemap {
namespace convert {
template <int dim>
Index<dim> indexToBlockIndex(const Index<dim>& index,
                             IndexElement cells_per_block_side_log_2) {
  return int_math::div_exp2_floor(index, cells_per_block_side_log_2);
}
}  // namespace convert

template <typename BlockDataT, int dim>
Index<dim> SpatialHash<BlockDataT, dim>::getMinBlockIndex() const {
  if (empty()) {
    return Index<kDim>::Zero();
  }

  BlockIndex min_block_index =
      BlockIndex::Constant(std::numeric_limits<IndexElement>::max());
  forEachBlock([&min_block_index](const BlockIndex& block_index,
                                  const BlockData& /*block*/) {
    min_block_index = min_block_index.cwiseMin(block_index);
  });
  return min_block_index;
}

template <typename BlockDataT, int dim>
Index<dim> SpatialHash<BlockDataT, dim>::getMaxBlockIndex() const {
  if (empty()) {
    return Index<kDim>::Zero();
  }

  BlockIndex max_block_index =
      BlockIndex::Constant(std::numeric_limits<IndexElement>::lowest());
  forEachBlock([&max_block_index](const BlockIndex& block_index,
                                  const BlockData& /*block*/) {
    max_block_index = max_block_index.cwiseMax(block_index);
  });
  return max_block_index;
}

template <typename BlockDataT, int dim>
bool SpatialHash<BlockDataT, dim>::hasBlock(
    const SpatialHash::BlockIndex& block_index) const {
  return block_map_.count(block_index);
}

template <typename BlockDataT, int dim>
bool SpatialHash<BlockDataT, dim>::eraseBlock(
    const SpatialHash::BlockIndex& block_index) {
  return block_map_.erase(block_index);
}

template <typename BlockDataT, int dim>
template <typename IndexedBlockVisitor>
void SpatialHash<BlockDataT, dim>::eraseBlockIf(
    IndexedBlockVisitor indicator_fn) {
  for (auto it = block_map_.begin(); it != block_map_.end();) {
    const BlockIndex& block_index = it->first;
    BlockData& block = it->second;
    if (std::invoke(indicator_fn, block_index, block)) {
      it = block_map_.erase(it);
    } else {
      ++it;
    }
  }
}

template <typename BlockDataT, int dim>
typename SpatialHash<BlockDataT, dim>::BlockData*
SpatialHash<BlockDataT, dim>::getBlock(
    const SpatialHash::BlockIndex& block_index) {
  const auto& it = block_map_.find(block_index);
  if (it != block_map_.end()) {
    return &it->second;
  } else {
    return nullptr;
  }
}

template <typename BlockDataT, int dim>
const typename SpatialHash<BlockDataT, dim>::BlockData*
SpatialHash<BlockDataT, dim>::getBlock(
    const SpatialHash::BlockIndex& block_index) const {
  const auto& it = block_map_.find(block_index);
  if (it != block_map_.end()) {
    return &it->second;
  } else {
    return nullptr;
  }
}

template <typename BlockDataT, int dim>
template <typename... DefaultArgs>
typename SpatialHash<BlockDataT, dim>::BlockData&
SpatialHash<BlockDataT, dim>::getOrAllocateBlock(
    const SpatialHash::BlockIndex& block_index, DefaultArgs&&... args) {
  return block_map_.try_emplace(block_index, std::forward<DefaultArgs>(args)...)
      .first->second;
}

template <typename BlockDataT, int dim>
template <typename IndexedBlockVisitor>
void SpatialHash<BlockDataT, dim>::forEachBlock(
    IndexedBlockVisitor visitor_fn) {
  for (auto& [block_index, block_data] : block_map_) {
    std::invoke(visitor_fn, block_index, block_data);
  }
}

template <typename BlockDataT, int dim>
template <typename IndexedBlockVisitor>
void SpatialHash<BlockDataT, dim>::forEachBlock(
    IndexedBlockVisitor visitor_fn) const {
  for (const auto& [block_index, block_data] : block_map_) {
    std::invoke(visitor_fn, block_index, block_data);
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_IMPL_SPATIAL_HASH_INL_H_
