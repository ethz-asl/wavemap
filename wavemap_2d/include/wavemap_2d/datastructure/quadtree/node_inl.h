#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_

#include <memory>

namespace wavemap_2d {
template <typename NodeDataType>
void Node<NodeDataType>::allocateChildrenArray() {
  if (!children_) {
    children_ = std::make_unique<ChildrenArray>();
  }
}

template <typename NodeDataType>
void Node<NodeDataType>::pruneChildren() {
  children_.reset();
}

template <typename NodeDataType>
size_t Node<NodeDataType>::getMemoryUsage() const {
  size_t memory_usage = sizeof(Node<NodeDataType>);
  if (hasAllocatedChildrenArray()) {
    memory_usage += sizeof(ChildrenArray);
  }
  return memory_usage;
}

template <typename NodeDataType>
bool Node<NodeDataType>::hasAtLeastOneChild() const {
  if (children_) {
    for (int idx = 0; idx < NodeIndex::kNumChildren; ++idx) {
      if (children_->operator[](idx)) {
        return true;
      }
    }
  }
  return false;
}

template <typename NodeDataType>
void Node<NodeDataType>::allocateChild(
    NodeRelativeChildIndex child_index) const {
  if (!children_) {
    hasAllocatedChildrenArray();
  }
  children_->operator[](child_index) = std::make_unique<Node>();
}

template <typename NodeDataType>
Node<NodeDataType>* Node<NodeDataType>::getChild(
    NodeRelativeChildIndex child_index) {
  if (children_) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}

template <typename NodeDataType>
const Node<NodeDataType>* Node<NodeDataType>::getChild(
    NodeRelativeChildIndex child_index) const {
  if (children_) {
    return children_->operator[](child_index).get();
  }
  return nullptr;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_
