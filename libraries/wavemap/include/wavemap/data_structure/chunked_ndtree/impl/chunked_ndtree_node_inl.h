#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_INL_H_

#include <memory>
#include <utility>

namespace wavemap {
template <typename DataT, int dim, int height>
bool ChunkedNdtreeNode<DataT, dim, height>::empty() const {
  return !hasChildrenArray() &&
         std::all_of(data_.cbegin(), data_.cend(),
                     [](auto val) { return val == DataT{}; });
}

template <typename DataT, int dim, int height>
void ChunkedNdtreeNode<DataT, dim, height>::clear() {
  deleteChildrenArray();
  std::fill(data_.begin(), data_.end(), DataT{});
}

template <typename DataT, int dim, int height>
DataT& ChunkedNdtreeNode<DataT, dim, height>::data(LinearIndex linear_index) {
  CHECK_GE(linear_index, 0u);
  CHECK_LT(linear_index, kNumInnerNodes);
  return data_[linear_index];
}

template <typename DataT, int dim, int height>
const DataT& ChunkedNdtreeNode<DataT, dim, height>::data(
    LinearIndex linear_index) const {
  CHECK_GE(linear_index, 0u);
  CHECK_LT(linear_index, kNumInnerNodes);
  return data_[linear_index];
}

template <typename DataT, int dim, int height>
void ChunkedNdtreeNode<DataT, dim, height>::allocateChildrenArrayIfNeeded() {
  if (!hasChildrenArray()) {
    children_ = std::make_unique<ChildrenArray>();
  }
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeNode<DataT, dim, height>::hasChild(
    LinearIndex child_index) const {
  return getChild(child_index);
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeNode<DataT, dim, height>::hasAtLeastOneChild() const {
  if (hasChildrenArray()) {
    return std::any_of(
        children_->cbegin(), children_->cend(),
        [](const auto& child_ptr) { return static_cast<bool>(child_ptr); });
  }
  return false;
}

template <typename DataT, int dim, int height>
ChunkedNdtreeNode<DataT, dim, height>*
ChunkedNdtreeNode<DataT, dim, height>::allocateChild(LinearIndex child_index) {
  CHECK_GE(child_index, 0u);
  CHECK_LT(child_index, kNumChildren);
  allocateChildrenArrayIfNeeded();
  children_->operator[](child_index) = std::make_unique<ChunkedNdtreeNode>();
  return children_->operator[](child_index).get();
}

template <typename DataT, int dim, int height>
bool ChunkedNdtreeNode<DataT, dim, height>::deleteChild(
    LinearIndex child_index) {
  if (hasChild(child_index)) {
    children_->operator[](child_index).reset();
    return true;
  }
  return false;
}

template <typename DataT, int dim, int height>
ChunkedNdtreeNode<DataT, dim, height>*
ChunkedNdtreeNode<DataT, dim, height>::getChild(LinearIndex child_index) {
  CHECK_GE(child_index, 0u);
  CHECK_LT(child_index, kNumChildren);
  if (hasChildrenArray()) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}

template <typename DataT, int dim, int height>
const ChunkedNdtreeNode<DataT, dim, height>*
ChunkedNdtreeNode<DataT, dim, height>::getChild(LinearIndex child_index) const {
  CHECK_GE(child_index, 0u);
  CHECK_LT(child_index, kNumChildren);
  if (hasChildrenArray()) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}

template <typename DataT, int dim, int height>
size_t ChunkedNdtreeNode<DataT, dim, height>::getMemoryUsage() const {
  size_t memory_usage = sizeof(ChunkedNdtreeNode<DataT, dim, height>);
  if (hasChildrenArray()) {
    memory_usage += sizeof(ChildrenArray);
  }
  return memory_usage;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_INL_H_
