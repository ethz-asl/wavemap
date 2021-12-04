#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_

namespace wavemap_2d {
template <typename NodeDataType>
bool Node<NodeDataType>::hasNotNullChildren() const {
  if (children_) {
    for (int idx = 0; idx < NodeIndex::kNumChildren; ++idx) {
      if (children_[idx]) {
        return true;
      }
    }
  }
  return false;
}

template <typename NodeDataType>
bool Node<NodeDataType>::hasAllocatedChildren() const {
  return children_;
}

template <typename NodeDataType>
void Node<NodeDataType>::allocateChildren() {
  if (!children_) {
    children_ = new Node*[NodeIndex::kNumChildren];
    for (int idx = 0; idx < NodeIndex::kNumChildren; ++idx) {
      children_[idx] = nullptr;
    }
  }
}

template <typename NodeDataType>
void Node<NodeDataType>::pruneChildren() {
  if (children_) {
    for (int idx = 0; idx < NodeIndex::kNumChildren; ++idx) {
      if (children_[idx]) {
        delete children_[idx];
      }
    }
    delete children_;
    children_ = nullptr;
  }
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_
