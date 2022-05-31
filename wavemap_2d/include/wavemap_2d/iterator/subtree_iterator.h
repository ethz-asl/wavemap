#ifndef WAVEMAP_2D_ITERATOR_SUBTREE_ITERATOR_H_
#define WAVEMAP_2D_ITERATOR_SUBTREE_ITERATOR_H_

#include <deque>

#include "wavemap_2d/data_structure/generic/quadtree/node.h"
#include "wavemap_2d/indexing/ndtree_index.h"

namespace wavemap_2d {
// TODO(victorr): Add iterative deepening DFS as an alternative to BFS that uses
//                less memory (at the cost of increased compute)
enum class TraversalOrder {
  kDepthFirstPreorder,
  kDepthFirstPostorder,
  kBreadthFirst
};

template <typename ValueT, typename DequeueElementT, typename DerivedT>
class SubtreeIteratorBase {
 public:
  using difference_type = std::ptrdiff_t;
  using value_type = ValueT;
  using pointer = ValueT*;
  using reference = ValueT&;
  using iterator_category = std::forward_iterator_tag;

  ValueT& operator*() { return *getFrontPtr(); }
  const ValueT& operator*() const { return *getFrontPtr(); }
  ValueT* operator->() { return getFrontPtr(); }

  friend bool operator==(const SubtreeIteratorBase& lhs,
                         const SubtreeIteratorBase& rhs) {
    return (lhs.upcoming_nodes_.empty() && rhs.upcoming_nodes_.empty()) ||
           (lhs.upcoming_nodes_.size() == rhs.upcoming_nodes_.size() &&
            lhs.upcoming_nodes_.front() == rhs.upcoming_nodes_.front());
  }
  friend bool operator!=(const SubtreeIteratorBase& lhs,
                         const SubtreeIteratorBase& rhs) {
    return !(lhs == rhs);  // NOLINT
  }

  virtual DerivedT& operator++() = 0;  // prefix ++
  DerivedT operator++(int) {           // postfix ++
    DerivedT retval = *this;
    ++(*this);  // call the above prefix incrementer
    return retval;
  }

 protected:
  std::deque<DequeueElementT> upcoming_nodes_;
  virtual pointer getFrontPtr() = 0;
};

template <typename NodeT, TraversalOrder traversal_order>
class SubtreeIterator
    : public SubtreeIteratorBase<NodeT, NodeT*,
                                 SubtreeIterator<NodeT, traversal_order>> {
 public:
  static_assert(
      traversal_order != TraversalOrder::kDepthFirstPostorder,
      "This version of the class should only be compiled when using "
      "kDepthFirstPreorder or kBreadthFirst traversal. Depth first postorder "
      "traversal is implemented separately through template specialization. If "
      "this message appears, the specialization must have broken.");

  explicit SubtreeIterator(NodeT* root_node) {
    if (root_node) {
      Base::upcoming_nodes_.template emplace_front(root_node);
    }
  }

  SubtreeIterator& operator++() {  // prefix ++
    NodeT* node_ptr = getFrontPtr();
    Base::upcoming_nodes_.pop_front();
    enqueueNodeChildren(node_ptr);

    return *this;
  }

 private:
  using Base = SubtreeIteratorBase<NodeT, NodeT*,
                                   SubtreeIterator<NodeT, traversal_order>>;

  NodeT* getFrontPtr() override { return Base::upcoming_nodes_.front(); }

  void enqueueNodeChildren(NodeT* parent_ptr) {
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
};

// Use template specialization to define the post-order subtree iterator
template <typename NodeT>
struct DepthFirstPostorderStackElement {
  NodeT* node_ptr;
  QuadtreeIndex::RelativeChild last_expanded_child_idx;
  bool operator==(const DepthFirstPostorderStackElement& rhs) const {
    return node_ptr == rhs.node_ptr &&
           last_expanded_child_idx == rhs.last_expanded_child_idx;
  }
};
template <typename NodeT>
class SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPostorder>
    : public SubtreeIteratorBase<
          NodeT, DepthFirstPostorderStackElement<NodeT>,
          SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPostorder>> {
 public:
  explicit SubtreeIterator(NodeT* root_node) {
    if (root_node) {
      enqueueNodeAndFirstChildren(root_node);
    }
  }

  SubtreeIterator& operator++() {  // prefix ++
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

 private:
  using Base = SubtreeIteratorBase<
      NodeT, DepthFirstPostorderStackElement<NodeT>,
      SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPostorder>>;

  NodeT* getFrontPtr() override {
    return Base::upcoming_nodes_.front().node_ptr;
  }

  void enqueueNodeAndFirstChildren(NodeT* parent_ptr) {
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
};

template <typename NodeT, TraversalOrder traversal_order>
class Subtree {
 public:
  explicit Subtree(NodeT* root_node) : root_node_(root_node) {}

  auto begin() { return SubtreeIterator<NodeT, traversal_order>(root_node_); }
  auto end() { return SubtreeIterator<NodeT, traversal_order>(nullptr); }

 private:
  NodeT* const root_node_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_ITERATOR_SUBTREE_ITERATOR_H_
