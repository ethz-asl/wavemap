#ifndef WAVEMAP_COMMON_ITERATOR_IMPL_SUBTREE_ITERATOR_INL_H_
#define WAVEMAP_COMMON_ITERATOR_IMPL_SUBTREE_ITERATOR_INL_H_

namespace wavemap {
template <typename NodeT>
SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPreorder>&
SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPreorder>::operator++() {
  NodeT* node_ptr = upcoming_nodes_.front();
  upcoming_nodes_.pop_front();
  enqueueNodeChildren(node_ptr);

  return *this;
}

template <typename NodeT>
void SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPreorder>::
    enqueueNodeChildren(NodeT* parent_ptr) {
  if (parent_ptr->hasChildrenArray()) {
    for (int child_idx = QuadtreeIndex::kNumChildren - 1; 0 <= child_idx;
         --child_idx) {
      NodeT* child_ptr = parent_ptr->getChild(child_idx);
      if (child_ptr) {
        upcoming_nodes_.template emplace_front(child_ptr);
      }
    }
  }
}

template <typename NodeT>
SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPostorder>&
SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPostorder>::operator++() {
  upcoming_nodes_.pop_front();

  // Stop here if all nodes have already been visited
  if (upcoming_nodes_.empty()) {
    return *this;
  }

  // Otherwise enqueue the next node and its first children
  StackElement& node_and_state = upcoming_nodes_.front();
  for (++node_and_state.last_expanded_child_idx;
       node_and_state.last_expanded_child_idx < QuadtreeIndex::kNumChildren;
       ++node_and_state.last_expanded_child_idx) {
    NodeT* child_ptr = node_and_state.node_ptr->getChild(
        node_and_state.last_expanded_child_idx);
    if (child_ptr) {
      enqueueNodeAndFirstChildren(child_ptr);
      break;
    }
  }
  return *this;
}

template <typename NodeT>
void SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPostorder>::
    enqueueNodeAndFirstChildren(NodeT* parent_ptr) {
  if (parent_ptr->hasChildrenArray()) {
    // If the node has descendants, recursively enqueue all of its
    // descendants that have the lowest index on their respective level
    for (QuadtreeIndex::RelativeChild child_idx = 0;
         child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
      NodeT* child_ptr = parent_ptr->getChild(child_idx);
      if (child_ptr) {
        upcoming_nodes_.template emplace_front(
            StackElement{parent_ptr, child_idx});
        enqueueNodeAndFirstChildren(child_ptr);
        return;
      }
    }
  } else {
    // Otherwise, only enqueue the node itself
    upcoming_nodes_.template emplace_front(
        StackElement{parent_ptr, QuadtreeIndex::kNumChildren});
  }
}

template <typename NodeT>
SubtreeIterator<NodeT, TraversalOrder::kBreadthFirst>&
SubtreeIterator<NodeT, TraversalOrder::kBreadthFirst>::operator++() {
  NodeT* node_ptr = getFrontValuePtr();
  upcoming_nodes_.pop_front();
  enqueueNodeChildren(node_ptr);

  return *this;
}

template <typename NodeT>
void SubtreeIterator<NodeT, TraversalOrder::kBreadthFirst>::enqueueNodeChildren(
    NodeT* parent_ptr) {
  if (parent_ptr->hasChildrenArray()) {
    for (QuadtreeIndex::RelativeChild child_idx = 0;
         child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
      NodeT* child_ptr = parent_ptr->getChild(child_idx);
      if (child_ptr) {
        upcoming_nodes_.template emplace_back(child_ptr);
      }
    }
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_ITERATOR_IMPL_SUBTREE_ITERATOR_INL_H_
