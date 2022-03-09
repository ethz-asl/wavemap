#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_

#include <memory>

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
void Node<NodeDataType>::allocateChild(NodeRelativeChildIndex child_index) {
  allocateChildrenArrayIfNeeded();
  children_->operator[](child_index) = std::make_unique<Node>();
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
void Node<NodeDataType>::pruneChildren() {
  // Recursively delete all zero nodes without children (leaves)
  if (hasChildrenArray()) {
    bool has_non_empty_child = false;
    for (int child_idx = 0; child_idx < NodeIndex::kNumChildren; ++child_idx) {
      Node* child = getChild(child_idx);
      if (child) {
        child->pruneChildren();
        if (child->empty()) {
          deleteChild(child_idx);
        } else {
          has_non_empty_child = true;
        }
      }
    }
    // Free up the children array if it only contains null ptrs
    if (!has_non_empty_child) {
      deleteChildrenArray();
    }
  }
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
