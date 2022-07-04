#ifndef WAVEMAP_2D_DATA_STRUCTURE_GENERIC_NDTREE_IMPL_NDTREE_NODE_INL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_GENERIC_NDTREE_IMPL_NDTREE_NODE_INL_H_

#include <memory>
#include <utility>

namespace wavemap_2d {
template <typename NodeDataType, int dim>
void NdtreeNode<NodeDataType, dim>::allocateChildrenArrayIfNeeded() {
  if (!hasChildrenArray()) {
    children_ = std::make_unique<ChildrenArray>();
  }
}

template <typename NodeDataType, int dim>
void NdtreeNode<NodeDataType, dim>::deleteChildrenArray() {
  children_.reset();
}

template <typename NodeDataType, int dim>
bool NdtreeNode<NodeDataType, dim>::hasChild(
    typename IndexType::RelativeChild child_index) const {
  return getChild(child_index);
}

template <typename NodeDataType, int dim>
bool NdtreeNode<NodeDataType, dim>::hasAtLeastOneChild() const {
  if (hasChildrenArray()) {
    for (int child_idx = 0; child_idx < IndexType::kNumChildren; ++child_idx) {
      if (hasChild(child_idx)) {
        return true;
      }
    }
  }
  return false;
}

template <typename NodeDataType, int dim>
template <typename... NodeConstructorArgs>
NdtreeNode<NodeDataType, dim>* NdtreeNode<NodeDataType, dim>::allocateChild(
    typename IndexType::RelativeChild child_index,
    NodeConstructorArgs&&... args) {
  allocateChildrenArrayIfNeeded();
  children_->operator[](child_index) =
      std::make_unique<NdtreeNode>(std::forward<NodeConstructorArgs>(args)...);
  return children_->operator[](child_index).get();
}

template <typename NodeDataType, int dim>
bool NdtreeNode<NodeDataType, dim>::deleteChild(
    typename IndexType::RelativeChild child_index) {
  if (hasChild(child_index)) {
    children_->operator[](child_index).reset();
    return true;
  }
  return false;
}

template <typename NodeDataType, int dim>
NdtreeNode<NodeDataType, dim>* NdtreeNode<NodeDataType, dim>::getChild(
    typename IndexType::RelativeChild child_index) {
  if (hasChildrenArray()) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}

template <typename NodeDataType, int dim>
const NdtreeNode<NodeDataType, dim>* NdtreeNode<NodeDataType, dim>::getChild(
    typename IndexType::RelativeChild child_index) const {
  if (hasChildrenArray()) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}

template <typename NodeDataType, int dim>
size_t NdtreeNode<NodeDataType, dim>::getMemoryUsage() const {
  size_t memory_usage = sizeof(NdtreeNode<NodeDataType, dim>);
  if (hasChildrenArray()) {
    memory_usage += sizeof(ChildrenArray);
  }
  return memory_usage;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_GENERIC_NDTREE_IMPL_NDTREE_NODE_INL_H_
