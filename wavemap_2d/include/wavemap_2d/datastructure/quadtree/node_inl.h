#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_

#include <memory>
#include <utility>

namespace wavemap_2d {
template <typename NodeDataType>
void Node<NodeDataType>::allocateChildrenArrayIfNeeded() {
  if (!hasChildrenArray()) {
    children_ = std::make_unique<ChildrenArray>();
  }
}

template <typename NodeDataType>
void Node<NodeDataType>::deleteChildrenArray() {
  children_.reset();
}

template <typename NodeDataType>
bool Node<NodeDataType>::hasChild(NodeRelativeChildIndex child_index) const {
  return getChild(child_index);
}

template <typename NodeDataType>
bool Node<NodeDataType>::hasAtLeastOneChild() const {
  if (hasChildrenArray()) {
    for (int child_idx = 0; child_idx < NodeIndex::kNumChildren; ++child_idx) {
      if (hasChild(child_idx)) {
        return true;
      }
    }
  }
  return false;
}

template <typename NodeDataType>
template <typename... NodeConstructorArgs>
Node<NodeDataType>* Node<NodeDataType>::allocateChild(
    NodeRelativeChildIndex child_index, NodeConstructorArgs&&... args) {
  allocateChildrenArrayIfNeeded();
  children_->operator[](child_index) =
      std::make_unique<Node>(std::forward<NodeConstructorArgs>(args)...);
  return children_->operator[](child_index).get();
}

template <typename NodeDataType>
bool Node<NodeDataType>::deleteChild(NodeRelativeChildIndex child_index) {
  if (hasChild(child_index)) {
    children_->operator[](child_index).reset();
    return true;
  }
  return false;
}

template <typename NodeDataType>
Node<NodeDataType>* Node<NodeDataType>::getChild(
    NodeRelativeChildIndex child_index) {
  if (hasChildrenArray()) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}

template <typename NodeDataType>
const Node<NodeDataType>* Node<NodeDataType>::getChild(
    NodeRelativeChildIndex child_index) const {
  if (hasChildrenArray()) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}

template <typename NodeDataType>
size_t Node<NodeDataType>::getMemoryUsage() const {
  size_t memory_usage = sizeof(Node<NodeDataType>);
  if (hasChildrenArray()) {
    memory_usage += sizeof(ChildrenArray);
  }
  return memory_usage;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_
