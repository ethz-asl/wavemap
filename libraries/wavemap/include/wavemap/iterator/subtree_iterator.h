#ifndef WAVEMAP_ITERATOR_SUBTREE_ITERATOR_H_
#define WAVEMAP_ITERATOR_SUBTREE_ITERATOR_H_

#include <deque>

#include "wavemap/data_structure/ndtree/ndtree_node.h"
#include "wavemap/indexing/ndtree_index.h"

namespace wavemap {
// TODO(victorr): Add iterative deepening DFS as an alternative to BFS that uses
//                less memory (at the cost of increased compute)
enum class TraversalOrder {
  kDepthFirstPreorder,
  kDepthFirstPostorder,
  kBreadthFirst
};

template <typename NodeT, typename DerivedT>
class SubtreeIteratorBase {
 public:
  using difference_type = std::ptrdiff_t;
  using value_type = NodeT;
  using pointer = NodeT*;
  using reference = NodeT&;
  using iterator_category = std::forward_iterator_tag;

  NodeT& operator*() { return *getFrontValuePtr(); }
  const NodeT& operator*() const { return *getFrontValuePtr(); }
  NodeT* operator->() { return getFrontValuePtr(); }

  friend bool operator==(const SubtreeIteratorBase& lhs,
                         const SubtreeIteratorBase& rhs) {
    return (lhs.dequeSize() == rhs.dequeSize()) &&
           (lhs.dequeSize() == 0 ||
            *lhs.getFrontValuePtr() == *rhs.getFrontValuePtr());
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

 private:
  virtual size_t dequeSize() const = 0;
  virtual NodeT* getFrontValuePtr() const = 0;
};

template <typename NodeT, TraversalOrder traversal_order>
class SubtreeIterator;

template <typename NodeT>
class SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPreorder>
    : public SubtreeIteratorBase<
          NodeT, SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPreorder>> {
 public:
  explicit SubtreeIterator(NodeT* root_node) {
    if (root_node) {
      upcoming_nodes_.template emplace_front(root_node);
    }
  }
  SubtreeIterator& operator++() override;

 private:
  std::deque<NodeT*> upcoming_nodes_;
  size_t dequeSize() const override { return upcoming_nodes_.size(); }
  NodeT* getFrontValuePtr() const override { return upcoming_nodes_.front(); }
  void enqueueNodeChildren(NodeT* parent_ptr);
};

template <typename NodeT>
class SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPostorder>
    : public SubtreeIteratorBase<
          NodeT, SubtreeIterator<NodeT, TraversalOrder::kDepthFirstPostorder>> {
 public:
  explicit SubtreeIterator(NodeT* root_node) {
    if (root_node) {
      enqueueNodeAndFirstChildren(root_node);
    }
  }
  SubtreeIterator& operator++() override;

 private:
  struct StackElement {
    NodeT* node_ptr;
    NdtreeIndexRelativeChild last_expanded_child_idx;
    bool operator==(const StackElement& rhs) const {
      return node_ptr == rhs.node_ptr &&
             last_expanded_child_idx == rhs.last_expanded_child_idx;
    }
  };
  std::deque<StackElement> upcoming_nodes_;
  size_t dequeSize() const override { return upcoming_nodes_.size(); }
  NodeT* getFrontValuePtr() const override {
    return upcoming_nodes_.front().node_ptr;
  }
  void enqueueNodeAndFirstChildren(NodeT* parent_ptr);
};

template <typename NodeT>
class SubtreeIterator<NodeT, TraversalOrder::kBreadthFirst>
    : public SubtreeIteratorBase<
          NodeT, SubtreeIterator<NodeT, TraversalOrder::kBreadthFirst>> {
 public:
  explicit SubtreeIterator(NodeT* root_node) {
    if (root_node) {
      upcoming_nodes_.template emplace_front(root_node);
    }
  }
  SubtreeIterator& operator++() override;

 private:
  std::deque<NodeT*> upcoming_nodes_;
  size_t dequeSize() const override { return upcoming_nodes_.size(); }
  NodeT* getFrontValuePtr() const override { return upcoming_nodes_.front(); }
  void enqueueNodeChildren(NodeT* parent_ptr);
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
}  // namespace wavemap

#include "wavemap/iterator/impl/subtree_iterator_inl.h"

#endif  // WAVEMAP_ITERATOR_SUBTREE_ITERATOR_H_
