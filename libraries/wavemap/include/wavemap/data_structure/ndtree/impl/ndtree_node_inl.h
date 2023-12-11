#ifndef WAVEMAP_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_NODE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_NODE_INL_H_

#include <memory>
#include <utility>

#include "wavemap/utils/data/comparisons.h"

namespace wavemap {
template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::empty() const {
  return !hasChildrenArray() && !hasNonzeroData();
}

template <typename DataT, int dim>
void NdtreeNode<DataT, dim>::clear() {
  deleteChildrenArray();
  data() = DataT{};
}

template <typename DataT, int dim>
size_t NdtreeNode<DataT, dim>::getMemoryUsage() const {
  size_t memory_usage = sizeof(NdtreeNode<DataT, dim>);
  if (hasChildrenArray()) {
    memory_usage += sizeof(ChildrenArray);
  }
  return memory_usage;
}

template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::hasNonzeroData() const {
  return data::is_nonzero(data_);
}

template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::hasNonzeroData(FloatingPoint threshold) const {
  return data::is_nonzero(data_, threshold);
}

template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::hasAtLeastOneChild() const {
  if (hasChildrenArray()) {
    for (NdtreeIndexRelativeChild child_idx = 0; child_idx < kNumChildren;
         ++child_idx) {
      if (children_->operator[](child_idx)) {
        return true;
      }
    }
  }
  return false;
}

template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::hasChild(
    NdtreeIndexRelativeChild child_index) const {
  return getChild(child_index);
}

template <typename DataT, int dim>
bool NdtreeNode<DataT, dim>::eraseChild(NdtreeIndexRelativeChild child_index) {
  CHECK_GE(child_index, 0u);
  CHECK_LT(child_index, kNumChildren);
  if (hasChildrenArray()) {
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
template <typename... DefaultArgs>
NdtreeNode<DataT, dim>& NdtreeNode<DataT, dim>::getOrAllocateChild(
    NdtreeIndexRelativeChild child_index, DefaultArgs&&... args) {
  CHECK_GE(child_index, 0u);
  CHECK_LT(child_index, kNumChildren);
  // Make sure the children array is allocated
  if (!hasChildrenArray()) {
    children_ = std::make_unique<ChildrenArray>();
  }
  // Get the child, allocating it if needed
  ChildPtr& child_smart_ptr = children_->operator[](child_index);
  if (!child_smart_ptr) {
    child_smart_ptr =
        std::make_unique<NdtreeNode>(std::forward<DefaultArgs>(args)...);
  }
  // Return a reference to the child
  return *child_smart_ptr;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_NODE_INL_H_
