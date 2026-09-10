#ifndef WAVEMAP_CORE_MAP_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_INL_H_
#define WAVEMAP_CORE_MAP_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_INL_H_

#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/utils/profile/profiler_interface.h"

namespace wavemap {
template <typename CellDataT>
size_t HashedChunkedWaveletOctreeT<CellDataT>::size() const {
  size_t size = 0u;
  forEachBlock([&size](const BlockIndex& /*block_index*/, const Block& block) {
    size += block.size();
  });
  return size;
}

template <typename CellDataT>
void HashedChunkedWaveletOctreeT<CellDataT>::threshold() {
  ProfilerZoneScoped;
  forEachBlock([](const BlockIndex& /*block_index*/, Block& block) {
    block.threshold();
  });
}

template <typename CellDataT>
void HashedChunkedWaveletOctreeT<CellDataT>::prune() {
  ProfilerZoneScoped;
  block_map_.eraseBlockIf([](const BlockIndex& /*block_index*/, Block& block) {
    block.prune();
    return block.empty();
  });
}

template <typename CellDataT>
void HashedChunkedWaveletOctreeT<CellDataT>::pruneSmart() {
  ProfilerZoneScoped;
  block_map_.eraseBlockIf(
      [&config = config_](const BlockIndex& /*block_index*/, Block& block) {
        if (config.only_prune_blocks_if_unused_for <
            block.getTimeSinceLastUpdated()) {
          block.prune();
        }
        return block.empty();
      });
}

template <typename CellDataT>
size_t HashedChunkedWaveletOctreeT<CellDataT>::getMemoryUsage() const {
  ProfilerZoneScoped;
  // TODO(victorr): Also include the memory usage of the unordered map itself
  size_t memory_usage = 0u;
  forEachBlock(
      [&memory_usage](const BlockIndex& /*block_index*/, const Block& block) {
        memory_usage += block.getMemoryUsage();
      });
  return memory_usage;
}

template <typename CellDataT>
Index3D HashedChunkedWaveletOctreeT<CellDataT>::getMinIndex() const {
  return cells_per_block_side_ * getMinBlockIndex();
}

template <typename CellDataT>
Index3D HashedChunkedWaveletOctreeT<CellDataT>::getMaxIndex() const {
  if (empty()) {
    return Index3D::Zero();
  }
  return cells_per_block_side_ * (getMaxBlockIndex().array() + 1) - 1;
}

template <typename CellDataT>
FloatingPoint HashedChunkedWaveletOctreeT<CellDataT>::getCellValue(
    const Index3D& index) const {
  const BlockIndex block_index =
      convert::indexToBlockIndex(index, config_.tree_height);
  const Block* block = getBlock(block_index);
  if (!block) {
    return 0.f;
  }
  const CellIndex cell_index = indexToCellIndex({0, index});
  return block->getCellValue(cell_index);
}

template <typename CellDataT>
FloatingPoint HashedChunkedWaveletOctreeT<CellDataT>::getCellValue(
    const OctreeIndex& index) const {
  const BlockIndex block_index = indexToBlockIndex(index);
  const Block* block = getBlock(block_index);
  if (!block) {
    return 0.f;
  }
  const CellIndex cell_index = indexToCellIndex(index);
  return block->getCellValue(cell_index);
}

template <typename CellDataT>
CellDataT HashedChunkedWaveletOctreeT<CellDataT>::getVoxelValue(
    const Index3D& index) const {
  const BlockIndex block_index =
      convert::indexToBlockIndex(index, config_.tree_height);
  const Block* block = getBlock(block_index);
  if (!block) {
    return {};
  }
  const CellIndex cell_index = indexToCellIndex({0, index});
  return block->getVoxelValue(cell_index);
}

template <typename CellDataT>
CellDataT HashedChunkedWaveletOctreeT<CellDataT>::getVoxelValue(
    const OctreeIndex& index) const {
  const BlockIndex block_index = indexToBlockIndex(index);
  const Block* block = getBlock(block_index);
  if (!block) {
    return {};
  }
  const CellIndex cell_index = indexToCellIndex(index);
  return block->getVoxelValue(cell_index);
}

template <typename CellDataT>
void HashedChunkedWaveletOctreeT<CellDataT>::setCellValue(const Index3D& index,
                                                   FloatingPoint new_value) {
  const BlockIndex block_index =
      convert::indexToBlockIndex(index, config_.tree_height);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = indexToCellIndex({0, index});
  block.setCellValue(cell_index, new_value);
}

template <typename CellDataT>
void HashedChunkedWaveletOctreeT<CellDataT>::setVoxelValue(
    const Index3D& index, const CellDataT& new_value) {
  const BlockIndex block_index =
      convert::indexToBlockIndex(index, config_.tree_height);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = indexToCellIndex({0, index});
  block.setVoxelValue(cell_index, new_value);
}

template <typename CellDataT>
void HashedChunkedWaveletOctreeT<CellDataT>::addToCellValue(const Index3D& index,
                                                     FloatingPoint update) {
  const BlockIndex block_index =
      convert::indexToBlockIndex(index, config_.tree_height);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = indexToCellIndex({0, index});
  block.addToCellValue(cell_index, update);
}

template <typename CellDataT>
void HashedChunkedWaveletOctreeT<CellDataT>::addToVoxelValue(
    const Index3D& index, const CellDataT& update) {
  const BlockIndex block_index =
      convert::indexToBlockIndex(index, config_.tree_height);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index = indexToCellIndex({0, index});
  block.addToVoxelValue(cell_index, update);
}

template <typename CellDataT>
bool HashedChunkedWaveletOctreeT<CellDataT>::hasBlock(
    const Index3D& block_index) const {
  return block_map_.hasBlock(block_index);
}

template <typename CellDataT>
bool HashedChunkedWaveletOctreeT<CellDataT>::eraseBlock(
    const typename HashedChunkedWaveletOctreeT<CellDataT>::BlockIndex& block_index) {
  return block_map_.eraseBlock(block_index);
}

template <typename CellDataT>
template <typename IndexedBlockVisitor>
void HashedChunkedWaveletOctreeT<CellDataT>::eraseBlockIf(
    IndexedBlockVisitor indicator_fn) {
  block_map_.eraseBlockIf(indicator_fn);
}

template <typename CellDataT>
typename HashedChunkedWaveletOctreeT<CellDataT>::Block*
HashedChunkedWaveletOctreeT<CellDataT>::getBlock(const Index3D& block_index) {
  return block_map_.getBlock(block_index);
}

template <typename CellDataT>
const typename HashedChunkedWaveletOctreeT<CellDataT>::Block*
HashedChunkedWaveletOctreeT<CellDataT>::getBlock(const Index3D& block_index) const {
  return block_map_.getBlock(block_index);
}

template <typename CellDataT>
typename HashedChunkedWaveletOctreeT<CellDataT>::Block&
HashedChunkedWaveletOctreeT<CellDataT>::getOrAllocateBlock(
    const Index3D& block_index) {
  return block_map_.getOrAllocateBlock(
      block_index, config_.tree_height, config_.min_log_odds,
      config_.max_log_odds, threshold_config_, pruning_config_);
}

template <typename CellDataT>
template <typename IndexedBlockVisitor>
void HashedChunkedWaveletOctreeT<CellDataT>::forEachBlock(
    IndexedBlockVisitor visitor_fn) {
  block_map_.forEachBlock(visitor_fn);
}

template <typename CellDataT>
template <typename IndexedBlockVisitor>
void HashedChunkedWaveletOctreeT<CellDataT>::forEachBlock(
    IndexedBlockVisitor visitor_fn) const {
  block_map_.forEachBlock(visitor_fn);
}

template <typename CellDataT>
void HashedChunkedWaveletOctreeT<CellDataT>::forEachLeaf(
    MapBase::IndexedLeafVisitorFunction visitor_fn) const {
  forEachBlock(
      [&visitor_fn](const BlockIndex& block_index, const Block& block) {
        block.forEachLeaf(block_index, visitor_fn);
      });
}

template <typename CellDataT>
template <typename IndexedVoxelLeafVisitorFunction>
void HashedChunkedWaveletOctreeT<CellDataT>::forEachVoxelLeaf(
    IndexedVoxelLeafVisitorFunction visitor_fn) const {
  forEachBlock(
      [&visitor_fn](const BlockIndex& block_index, const Block& block) {
        block.forEachVoxelLeaf(block_index, visitor_fn);
      });
}

template <typename CellDataT>
typename HashedChunkedWaveletOctreeT<CellDataT>::BlockIndex
HashedChunkedWaveletOctreeT<CellDataT>::indexToBlockIndex(
    const OctreeIndex& node_index) const {
  const Index3D index = convert::nodeIndexToMinCornerIndex(node_index);
  return convert::indexToBlockIndex(index, config_.tree_height);
}

template <typename CellDataT>
typename HashedChunkedWaveletOctreeT<CellDataT>::CellIndex
HashedChunkedWaveletOctreeT<CellDataT>::indexToCellIndex(OctreeIndex index) const {
  DCHECK_LE(index.height, config_.tree_height);
  const IndexElement height_difference = config_.tree_height - index.height;
  index.position =
      int_math::div_exp2_floor_remainder(index.position, height_difference);
  return index;
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_MAP_IMPL_HASHED_CHUNKED_WAVELET_OCTREE_INL_H_
