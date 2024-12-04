#ifndef WAVEMAP_CORE_UTILS_ITERATE_IMPL_SUBTREE_ITERATOR_INL_H_
#define WAVEMAP_CORE_UTILS_ITERATE_IMPL_SUBTREE_ITERATOR_INL_H_

namespace wavemap {
template <typename NodePtrT, typename DerivedT>
DerivedT SubtreeIteratorBase<NodePtrT, DerivedT>::operator++(int) {
  DerivedT retval = *this;
  ++(*this);  // call the prefix incrementer
  return retval;
}

template <typename NodePtrT, typename DerivedT>
bool SubtreeIteratorBase<NodePtrT, DerivedT>::operator==(
    const SubtreeIteratorBase& rhs) const {
  return dequeSize() == rhs.dequeSize() &&
         (dequeSize() == 0 || dequeFront() == rhs.dequeFront());
}

template <typename NodePtrT, typename DerivedT>
bool SubtreeIteratorBase<NodePtrT, DerivedT>::operator!=(
    const SubtreeIteratorBase& rhs) const {
  return !(*this == rhs);  // NOLINT
}

template <typename NodePtrT, int num_children>
SubtreeIterator<NodePtrT, num_children, TraversalOrder::kDepthFirstPreorder>::
    SubtreeIterator(NodePtrT root_node) {
  if (root_node) {
    upcoming_nodes_.emplace_front(root_node);
  }
}

template <typename NodePtrT, int num_children>
SubtreeIterator<NodePtrT, num_children, TraversalOrder::kDepthFirstPreorder>&
SubtreeIterator<NodePtrT, num_children,
                TraversalOrder::kDepthFirstPreorder>::operator++() {
  NodePtrT node_ptr = upcoming_nodes_.front();
  upcoming_nodes_.pop_front();
  enqueueNodeChildren(node_ptr);
  return *this;
}

template <typename NodePtrT, int num_children>
void SubtreeIterator<NodePtrT, num_children,
                     TraversalOrder::kDepthFirstPreorder>::
    enqueueNodeChildren(NodePtrT parent_ptr) {
  if (parent_ptr->hasAtLeastOneChild()) {
    for (int child_idx = num_children - 1; 0 <= child_idx; --child_idx) {
      if (NodePtrT child_ptr = parent_ptr->getChild(child_idx); child_ptr) {
        upcoming_nodes_.emplace_front(child_ptr);
      }
    }
  }
}

template <typename NodePtrT, int num_children>
SubtreeIterator<NodePtrT, num_children, TraversalOrder::kDepthFirstPostorder>::
    SubtreeIterator(NodePtrT root_node) {
  if (root_node) {
    enqueueNodeAndFirstChildren(root_node);
  }
}

template <typename NodePtrT, int num_children>
SubtreeIterator<NodePtrT, num_children, TraversalOrder::kDepthFirstPostorder>&
SubtreeIterator<NodePtrT, num_children,
                TraversalOrder::kDepthFirstPostorder>::operator++() {
  upcoming_nodes_.pop_front();

  // Stop here if all nodes have already been visited
  if (upcoming_nodes_.empty()) {
    return *this;
  }

  // Otherwise enqueue the next node and its first children
  StackElement& node_and_state = upcoming_nodes_.front();
  for (++node_and_state.last_expanded_child_idx;
       node_and_state.last_expanded_child_idx < num_children;
       ++node_and_state.last_expanded_child_idx) {
    NodePtrT child_ptr = node_and_state.node_ptr->getChild(
        node_and_state.last_expanded_child_idx);
    if (child_ptr) {
      enqueueNodeAndFirstChildren(child_ptr);
      break;
    }
  }
  return *this;
}

template <typename NodePtrT, int num_children>
NodePtrT SubtreeIterator<NodePtrT, num_children,
                         TraversalOrder::kDepthFirstPostorder>::dequeFront()
    const {
  return upcoming_nodes_.front().node_ptr;
}

template <typename NodePtrT, int num_children>
void SubtreeIterator<NodePtrT, num_children,
                     TraversalOrder::kDepthFirstPostorder>::
    enqueueNodeAndFirstChildren(NodePtrT parent_ptr) {
  if (parent_ptr->hasAtLeastOneChild()) {
    // If the node has descendants, recursively enqueue all of its
    // descendants that have the lowest index on their respective level
    for (NdtreeIndexRelativeChild child_idx = 0; child_idx < num_children;
         ++child_idx) {
      if (NodePtrT child_ptr = parent_ptr->getChild(child_idx); child_ptr) {
        upcoming_nodes_.emplace_front(StackElement{parent_ptr, child_idx});
        enqueueNodeAndFirstChildren(child_ptr);
        return;
      }
    }
  } else {
    // Otherwise, only enqueue the node itself
    upcoming_nodes_.emplace_front(StackElement{parent_ptr, num_children});
  }
}

template <typename NodePtrT, int num_children>
SubtreeIterator<NodePtrT, num_children, TraversalOrder::kBreadthFirst>::
    SubtreeIterator(NodePtrT root_node) {
  if (root_node) {
    upcoming_nodes_.emplace_front(root_node);
  }
}

template <typename NodePtrT, int num_children>
SubtreeIterator<NodePtrT, num_children, TraversalOrder::kBreadthFirst>&
SubtreeIterator<NodePtrT, num_children,
                TraversalOrder::kBreadthFirst>::operator++() {
  NodePtrT node_ptr = dequeFront();
  upcoming_nodes_.pop_front();
  enqueueNodeChildren(node_ptr);

  return *this;
}

template <typename NodePtrT, int num_children>
void SubtreeIterator<NodePtrT, num_children, TraversalOrder::kBreadthFirst>::
    enqueueNodeChildren(NodePtrT parent_ptr) {
  if (parent_ptr->hasAtLeastOneChild()) {
    for (NdtreeIndexRelativeChild child_idx = 0; child_idx < num_children;
         ++child_idx) {
      if (NodePtrT child_ptr = parent_ptr->getChild(child_idx); child_ptr) {
        upcoming_nodes_.emplace_back(child_ptr);
      }
    }
  }
}

template <typename NodePtrT, int num_children, TraversalOrder traversal_order>
auto Subtree<NodePtrT, num_children, traversal_order>::begin() const {
  return SubtreeIterator<NodePtrT, num_children, traversal_order>{root_node_};
}

template <typename NodePtrT, int num_children, TraversalOrder traversal_order>
auto Subtree<NodePtrT, num_children, traversal_order>::end() const {
  return SubtreeIterator<NodePtrT, num_children, traversal_order>{nullptr};
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_ITERATE_IMPL_SUBTREE_ITERATOR_INL_H_
