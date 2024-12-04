#ifndef WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_CHUNK_INL_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_CHUNK_INL_H_

#include <memory>
#include <utility>

#include "wavemap/core/utils/data/comparisons.h"

namespace wavemap {
template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::empty() const {
  return !hasChildrenArray() && !hasNonzeroData();
}

template <typename DataT, int dim, int height>
void ChunkedNdtreeChunk<DataT, dim, height>::clear() {
  deleteChildrenArray();
  node_data_.fill({});
  allocated_child_mask_.fill({});
}

template <typename DataT, int dim, int height>
size_t ChunkedNdtreeChunk<DataT, dim, height>::getMemoryUsage() const {
  size_t memory_usage = sizeof(ChunkedNdtreeChunk);
  if (hasChildrenArray()) {
    memory_usage += sizeof(ChildChunkArray);
  }
  return memory_usage;
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::hasNonzeroData() const {
  return std::any_of(
      node_data_.cbegin(), node_data_.cend(),
      [](const auto& node_data) { return data::is_nonzero(node_data); });
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::hasNonzeroData(
    FloatingPoint threshold) const {
  return std::any_of(node_data_.cbegin(), node_data_.cend(),
                     [threshold](const auto& node_data) {
                       return data::is_nonzero(node_data, threshold);
                     });
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::hasAtLeastOneChild() const {
  if (hasChildrenArray()) {
    return std::any_of(
        child_chunks_->cbegin(), child_chunks_->cend(),
        [](const auto& child_ptr) { return static_cast<bool>(child_ptr); });
  }
  return false;
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::hasChild(
    LinearIndex relative_child_index) const {
  return getChild(relative_child_index);
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::eraseChild(
    LinearIndex relative_child_index) {
  if (hasChild(relative_child_index)) {
    child_chunks_->operator[](relative_child_index).reset();
    return true;
  }
  return false;
}

template <typename DataT, int dim, int height>
ChunkedNdtreeChunk<DataT, dim, height>*
ChunkedNdtreeChunk<DataT, dim, height>::getChild(
    LinearIndex relative_child_index) {
  CHECK_LT(relative_child_index, kNumChildren);
  if (hasChildrenArray()) {
    return child_chunks_->operator[](relative_child_index).get();
  }
  return nullptr;
}

template <typename DataT, int dim, int height>
const ChunkedNdtreeChunk<DataT, dim, height>*
ChunkedNdtreeChunk<DataT, dim, height>::getChild(
    LinearIndex relative_child_index) const {
  CHECK_LT(relative_child_index, kNumChildren);
  if (hasChildrenArray()) {
    return child_chunks_->operator[](relative_child_index).get();
  }
  return nullptr;
}

template <typename DataT, int dim, int height>
template <typename... DefaultArgs>
ChunkedNdtreeChunk<DataT, dim, height>&
ChunkedNdtreeChunk<DataT, dim, height>::getOrAllocateChild(
    LinearIndex relative_child_index, DefaultArgs&&... args) {
  CHECK_LT(relative_child_index, kNumChildren);
  // Make sure the children array is allocated
  if (!hasChildrenArray()) {
    child_chunks_ = std::make_unique<ChildChunkArray>();
  }
  // Get the child, allocating it if needed
  ChunkPtr& child_smart_ptr = child_chunks_->operator[](relative_child_index);
  if (!child_smart_ptr) {
    child_smart_ptr = std::make_unique<ChunkedNdtreeChunk>(
        std::forward<DefaultArgs>(args)...);
  }
  // Return a reference to the child
  return *child_smart_ptr;
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::nodeHasNonzeroData(
    NodeOffsetType relative_node_index) const {
  DCHECK_LT(relative_node_index, kNumInnerNodes);
  return data::is_nonzero(node_data_[relative_node_index]);
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::nodeHasNonzeroData(
    NodeOffsetType relative_node_index, FloatingPoint threshold) const {
  DCHECK_LT(relative_node_index, kNumInnerNodes);
  return data::is_nonzero(node_data_[relative_node_index], threshold);
}

template <typename DataT, int dim, int height>
DataT& ChunkedNdtreeChunk<DataT, dim, height>::nodeData(
    NodeOffsetType relative_node_index) {
  DCHECK_LT(relative_node_index, kNumInnerNodes);
  return node_data_[relative_node_index];
}

template <typename DataT, int dim, int height>
const DataT& ChunkedNdtreeChunk<DataT, dim, height>::nodeData(
    NodeOffsetType relative_node_index) const {
  DCHECK_LT(relative_node_index, kNumInnerNodes);
  return node_data_[relative_node_index];
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::nodeHasAtLeastOneChild(
    NodeOffsetType relative_node_index) const {
  DCHECK_LT(relative_node_index, kNumInnerNodes);
  return allocated_child_mask_[relative_node_index];
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::nodeHasChild(
    NodeOffsetType relative_node_index,
    NdtreeIndexRelativeChild child_index) const {
  DCHECK_LT(relative_node_index, kNumInnerNodes);
  const auto node_child_mask = allocated_child_mask_[relative_node_index];
  return bit_ops::is_bit_set(node_child_mask, child_index);
}

template <typename DataT, int dim, int height>
void ChunkedNdtreeChunk<DataT, dim, height>::nodeSetHasChild(
    NodeOffsetType relative_node_index, NdtreeIndexRelativeChild child_index) {
  DCHECK_LT(relative_node_index, kNumInnerNodes);
  auto& node_child_mask = allocated_child_mask_[relative_node_index];
  node_child_mask = bit_ops::set_bit(node_child_mask, child_index);
}

template <typename DataT, int dim, int height>
void ChunkedNdtreeChunk<DataT, dim, height>::nodeEraseChild(
    NodeOffsetType relative_node_index, NdtreeIndexRelativeChild child_index) {
  if (!nodeHasChild(relative_node_index, child_index)) {
    return;
  }
  {
    auto& node_child_mask = allocated_child_mask_[relative_node_index];
    node_child_mask = bit_ops::unset_bit(node_child_mask, child_index);
  }

  const NodeOffsetType child_offset =
      convert::nodeOffsetToChildOffset<kDim>(relative_node_index, child_index);
  NodeOffsetType child_start_idx =
      convert::nodeOffsetToLevelIndex<kDim>(child_offset);
  NodeOffsetType child_end_idx = child_start_idx + 1;
  for (IndexElement child_depth =
           convert::nodeOffsetToDepth<kDim>(child_offset);
       child_depth <= kHeight; ++child_depth) {
    const NodeOffsetType level_offset =
        tree_math::perfect_tree::num_total_nodes_fast<kDim>(child_depth);
    for (NodeOffsetType level_child_idx = child_start_idx;
         level_child_idx < child_end_idx; ++level_child_idx) {
      if (child_depth == kHeight) {
        eraseChild(level_child_idx);
      } else {
        const NodeOffsetType chunk_child_idx = level_offset + level_child_idx;
        nodeData(chunk_child_idx) = {};
        allocated_child_mask_[chunk_child_idx] = {};
      }
    }
    child_start_idx <<= kDim;
    child_end_idx <<= kDim;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_CHUNK_INL_H_
