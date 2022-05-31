#ifndef WAVEMAP_2D_ITERATOR_IMPL_SUBTREE_ITERATOR_INL_H_
#define WAVEMAP_2D_ITERATOR_IMPL_SUBTREE_ITERATOR_INL_H_

namespace wavemap_2d {
template <typename NodeT, TraversalOrder traversal_order>
SubtreeIterator<NodeT, traversal_order>&
SubtreeIterator<NodeT, traversal_order>::operator++() {  // prefix ++
  NodeT* node_ptr = getFrontPtr();
  Base::upcoming_nodes_.pop_front();
  enqueueNodeChildren(node_ptr);

  return *this;
}

template <typename NodeT, TraversalOrder traversal_order>
void SubtreeIterator<NodeT, traversal_order>::enqueueNodeChildren(
    NodeT* parent_ptr) {
  if (parent_ptr->hasChildrenArray()) {
    switch (traversal_order) {
      case TraversalOrder::kDepthFirstPreorder:
        for (int child_idx = QuadtreeIndex::kNumChildren - 1; 0 <= child_idx;
             --child_idx) {
          NodeT* child_ptr = parent_ptr->getChild(child_idx);
          if (child_ptr) {
            Base::upcoming_nodes_.template emplace_front(child_ptr);
          }
        }
        break;
      case TraversalOrder::kBreadthFirst:
        for (int child_idx = 0; child_idx < QuadtreeIndex::kNumChildren;
             ++child_idx) {
          NodeT* child_ptr = parent_ptr->getChild(child_idx);
          if (child_ptr) {
            Base::upcoming_nodes_.template emplace_back(child_ptr);
          }
        }
        break;
    }
  }
}

template <typename NodeT>
SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPostorder>& SubtreeIterator<
    NodeT, TraversalOrder::kDepthFirstPostorder>::operator++() {  // prefix ++
  Base::upcoming_nodes_.pop_front();

  // Stop here if all nodes have already been visited
  if (Base::upcoming_nodes_.empty()) {
    return *this;
  }

  // Otherwise enqueue the next node and its first children
  DepthFirstPostorderStackElement<NodeT>& node_and_state =
      Base::upcoming_nodes_.front();
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
        Base::upcoming_nodes_.template emplace_front(
            DepthFirstPostorderStackElement<NodeT>{parent_ptr, child_idx});
        enqueueNodeAndFirstChildren(child_ptr);
        return;
      }
    }
  } else {
    // Otherwise, only enqueue the node itself
    Base::upcoming_nodes_.template emplace_front(
        DepthFirstPostorderStackElement<NodeT>{parent_ptr,
                                               QuadtreeIndex::kNumChildren});
  }
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_ITERATOR_IMPL_SUBTREE_ITERATOR_INL_H_
