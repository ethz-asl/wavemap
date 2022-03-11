#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INL_H_

#include <memory>
#include <utility>

namespace wavemap_2d {
template <typename NodeDataType>
void Node<NodeDataType>::pruneChildren() {
  // Recursively delete all zero nodes without children (empty leaves)
  applyBottomUp([](Node<NodeDataType>* parent_ptr) {
    if (parent_ptr->hasChildrenArray()) {
      bool has_non_empty_child = false;
      for (int child_idx = 0; child_idx < NodeIndex::kNumChildren;
           ++child_idx) {
        Node* child_ptr = parent_ptr->getChild(child_idx);
        if (child_ptr) {
          if (child_ptr->empty()) {
            parent_ptr->deleteChild(child_idx);
          } else {
            has_non_empty_child = true;
          }
        }
      }

      // Free up the children array if it only contains null pointers
      if (!has_non_empty_child) {
        parent_ptr->deleteChildrenArray();
      }
    }
  });
}

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
template <typename Functor>
void Node<NodeDataType>::applyToChildren(Functor&& fn) {
  if (hasChildrenArray()) {
    for (int child_idx = 0; child_idx < NodeIndex::kNumChildren; ++child_idx) {
      Node* child_ptr = getChild(child_idx);
      if (child_ptr) {
        fn(child_ptr);
      }
    }
  }
}

template <typename NodeDataType>
template <typename Functor>
void Node<NodeDataType>::applyToChildren(Functor&& fn) const {
  if (hasChildrenArray()) {
    for (int child_idx = 0; child_idx < NodeIndex::kNumChildren; ++child_idx) {
      const Node* child_ptr = getChild(child_idx);
      if (child_ptr) {
        fn(child_ptr);
      }
    }
  }
}

template <typename NodeDataType>
template <typename Functor>
void Node<NodeDataType>::applyBottomUp(Functor&& fn) {
  // Recurse through the tree and apply the function to the children first
  // NOTE: This corresponds to postorder depth-first traversal.
  // TODO(victorr): Consider managing the stack explicitly instead of using
  //                recursion
  applyToChildren(
      [&fn](Node<NodeDataType>* node_ptr) { node_ptr->applyBottomUp(fn); });
  fn(this);
}

template <typename NodeDataType>
template <typename Functor>
void Node<NodeDataType>::applyBottomUp(Functor&& fn) const {
  // Recurse through the tree and apply the function to the children first
  // NOTE: This corresponds to postorder depth-first traversal.
  // TODO(victorr): Consider managing the stack explicitly instead of using
  //                recursion
  applyToChildren([&fn](const Node<NodeDataType>* node_ptr) {
    node_ptr->applyBottomUp(fn);
  });
  fn(this);
}

template <typename NodeDataType>
template <typename Functor>
void Node<NodeDataType>::applyTopDown(Functor&& fn) {
  // Apply the function to the nodes in breadth-first order
  LOG(FATAL) << "Not yet implemented.";
}

template <typename NodeDataType>
template <typename Functor>
void Node<NodeDataType>::applyTopDown(Functor&& fn) const {
  // Apply the function to the nodes in breadth-first order
  LOG(FATAL) << "Not yet implemented.";
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
