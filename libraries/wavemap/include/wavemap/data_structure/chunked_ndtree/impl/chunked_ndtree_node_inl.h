#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_INL_H_

#include <memory>
#include <utility>

namespace wavemap {
template <typename NodeDataType, int dim, int height>
bool ChunkedNdtreeNode<NodeDataType, dim, height>::empty() const {
  return !hasChildrenArray() &&
         std::all_of(data_.cbegin(), data_.cend(),
                     [](auto val) { return val == NodeDataType{}; });
}

template <typename NodeDataType, int dim, int height>
void ChunkedNdtreeNode<NodeDataType, dim, height>::clear() {
  deleteChildrenArray();
  data() = NodeDataType{};
}

template <typename NodeDataType, int dim, int height>
void ChunkedNdtreeNode<NodeDataType, dim,
                       height>::allocateChildrenArrayIfNeeded() {
  if (!hasChildrenArray()) {
    children_ = std::make_unique<ChildrenArray>();
  }
}

template <typename NodeDataType, int dim, int height>
bool ChunkedNdtreeNode<NodeDataType, dim, height>::hasChild(
    LinearIndex child_index) const {
  return getChild(child_index);
}

template <typename NodeDataType, int dim, int height>
bool ChunkedNdtreeNode<NodeDataType, dim, height>::hasAtLeastOneChild() const {
  if (hasChildrenArray()) {
    return std::any_of(
        children_->cbegin(), children_->cend(),
        [](const auto& child_ptr) { return static_cast<bool>(child_ptr); });
  }
  return false;
}

template <typename NodeDataType, int dim, int height>
ChunkedNdtreeNode<NodeDataType, dim, height>*
ChunkedNdtreeNode<NodeDataType, dim, height>::allocateChild(
    LinearIndex child_index) {
  allocateChildrenArrayIfNeeded();
  children_->operator[](child_index) = std::make_unique<ChunkedNdtreeNode>();
  return children_->operator[](child_index).get();
}

template <typename NodeDataType, int dim, int height>
bool ChunkedNdtreeNode<NodeDataType, dim, height>::deleteChild(
    LinearIndex child_index) {
  if (hasChild(child_index)) {
    children_->operator[](child_index).reset();
    return true;
  }
  return false;
}

template <typename NodeDataType, int dim, int height>
ChunkedNdtreeNode<NodeDataType, dim, height>*
ChunkedNdtreeNode<NodeDataType, dim, height>::getChild(
    LinearIndex child_index) {
  if (hasChildrenArray()) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}

template <typename NodeDataType, int dim, int height>
const ChunkedNdtreeNode<NodeDataType, dim, height>*
ChunkedNdtreeNode<NodeDataType, dim, height>::getChild(
    LinearIndex child_index) const {
  if (hasChildrenArray()) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}

template <typename NodeDataType, int dim, int height>
size_t ChunkedNdtreeNode<NodeDataType, dim, height>::getMemoryUsage() const {
  size_t memory_usage = sizeof(ChunkedNdtreeNode<NodeDataType, dim, height>);
  if (hasChildrenArray()) {
    memory_usage += sizeof(ChildrenArray);
  }
  return memory_usage;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_INL_H_
