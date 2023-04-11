#ifndef WAVEMAP_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_NODE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_NODE_INL_H_

#include <memory>
#include <utility>

#include "wavemap/utils/data_utils.h"

namespace wavemap {
template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::empty() const {
  return !hasChildrenArray() && !hasNonZeroData();
}

template <typename DataT, int dim>
void NdtreeNode<DataT, dim>::clear() {
  deleteChildrenArray();
  data() = DataT{};
}

template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::hasNonZeroData() const {
  return data_utils::is_non_zero(data_);
}

template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::hasNonZeroData(FloatingPoint threshold) const {
  return data_utils::is_non_zero(data_, threshold);
}

template <typename DataT, int dim>
void NdtreeNode<DataT, dim>::allocateChildrenArrayIfNeeded() {
  if (!hasChildrenArray()) {
    children_ = std::make_unique<ChildrenArray>();
  }
}

template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::hasChild(
    NdtreeIndexRelativeChild child_index) const {
  return getChild(child_index);
}

template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::hasAtLeastOneChild() const {
  if (hasChildrenArray()) {
    return std::any_of(
        children_->cbegin(), children_->cend(),
        [](const auto& child_ptr) { return static_cast<bool>(child_ptr); });
  }
  return false;
}

template <typename DataT, int dim>
template <typename... NodeConstructorArgs>
NdtreeNode<DataT, dim>* NdtreeNode<DataT, dim>::allocateChild(
    NdtreeIndexRelativeChild child_index, NodeConstructorArgs&&... args) {
  CHECK_GE(child_index, 0u);
  CHECK_LT(child_index, kNumChildren);
  allocateChildrenArrayIfNeeded();
  children_->operator[](child_index) =
      std::make_unique<NdtreeNode>(std::forward<NodeConstructorArgs>(args)...);
  return children_->operator[](child_index).get();
}

template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::deleteChild(NdtreeIndexRelativeChild child_index) {
  if (hasChild(child_index)) {
    children_->operator[](child_index).reset();
    return true;
  }
  return false;
}

template <typename DataT, int dim>
NdtreeNode<DataT, dim>* NdtreeNode<DataT, dim>::getChild(
    NdtreeIndexRelativeChild child_index) {
  CHECK_GE(child_index, 0u);
  CHECK_LT(child_index, kNumChildren);
  if (hasChildrenArray()) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}

template <typename DataT, int dim>
const NdtreeNode<DataT, dim>* NdtreeNode<DataT, dim>::getChild(
    NdtreeIndexRelativeChild child_index) const {
  if (hasChildrenArray()) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}

template <typename DataT, int dim>
size_t NdtreeNode<DataT, dim>::getMemoryUsage() const {
  size_t memory_usage = sizeof(NdtreeNode<DataT, dim>);
  if (hasChildrenArray()) {
    memory_usage += sizeof(ChildrenArray);
  }
  return memory_usage;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_NODE_INL_H_
