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
  SubtreeIterator& operator++() override;

 private:
  using Base = SubtreeIteratorBase<NodeT, NodeT*,
                                   SubtreeIterator<NodeT, traversal_order>>;

  NodeT* getFrontPtr() override { return Base::upcoming_nodes_.front(); }
  void enqueueNodeChildren(NodeT* parent_ptr);
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
  SubtreeIterator& operator++() override;

 private:
  using Base = SubtreeIteratorBase<
      NodeT, DepthFirstPostorderStackElement<NodeT>,
      SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPostorder>>;

  NodeT* getFrontPtr() override {
    return Base::upcoming_nodes_.front().node_ptr;
  }
  void enqueueNodeAndFirstChildren(NodeT* parent_ptr);
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

#include "wavemap_2d/iterator/impl/subtree_iterator_inl.h"

#endif  // WAVEMAP_2D_ITERATOR_SUBTREE_ITERATOR_H_
