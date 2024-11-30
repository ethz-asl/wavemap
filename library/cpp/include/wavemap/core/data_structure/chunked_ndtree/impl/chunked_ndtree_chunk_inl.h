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
  node_has_at_least_one_child_.reset();
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
DataT& ChunkedNdtreeChunk<DataT, dim, height>::nodeData(
    LinearIndex relative_node_index) {
  CHECK_GE(relative_node_index, 0u);
  CHECK_LT(relative_node_index, kNumInnerNodes);
  return node_data_[relative_node_index];
}

template <typename DataT, int dim, int height>
const DataT& ChunkedNdtreeChunk<DataT, dim, height>::nodeData(
    LinearIndex relative_node_index) const {
  CHECK_GE(relative_node_index, 0u);
  CHECK_LT(relative_node_index, kNumInnerNodes);
  return node_data_[relative_node_index];
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
  CHECK_GE(relative_child_index, 0u);
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
  CHECK_GE(relative_child_index, 0u);
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
  CHECK_GE(relative_child_index, 0u);
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
    LinearIndex relative_node_index) const {
  DCHECK_LT(relative_node_index, kNumInnerNodes);
  return data::is_nonzero(node_data_[relative_node_index]);
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::nodeHasNonzeroData(
    LinearIndex relative_node_index, FloatingPoint threshold) const {
  DCHECK_LT(relative_node_index, kNumInnerNodes);
  return data::is_nonzero(node_data_[relative_node_index], threshold);
}

template <typename DataT, int dim, int height>
typename ChunkedNdtreeChunk<DataT, dim, height>::BitRef
ChunkedNdtreeChunk<DataT, dim, height>::nodeHasAtLeastOneChild(
    LinearIndex relative_node_index) {
  DCHECK_LT(relative_node_index, kNumInnerNodes);
  return node_has_at_least_one_child_[relative_node_index];
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeChunk<DataT, dim, height>::nodeHasAtLeastOneChild(
    LinearIndex relative_node_index) const {
  DCHECK_LT(relative_node_index, kNumInnerNodes);
  return node_has_at_least_one_child_[relative_node_index];
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_CHUNK_INL_H_
