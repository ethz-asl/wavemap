#ifndef WAVEMAP_CORE_UTILS_ITERATE_SUBTREE_ITERATOR_H_
#define WAVEMAP_CORE_UTILS_ITERATE_SUBTREE_ITERATOR_H_

#include <deque>

#include "wavemap/core/indexing/ndtree_index.h"

namespace wavemap {
enum class TraversalOrder {
  kDepthFirstPreorder,
  kDepthFirstPostorder,
  kBreadthFirst
};

template <typename NodePtrT, typename DerivedT>
class SubtreeIteratorBase {
 public:
  using difference_type = std::ptrdiff_t;
  using pointer = NodePtrT;
  using reference = decltype(*std::declval<NodePtrT>());
  using value_type = reference;
  using iterator_category = std::forward_iterator_tag;

  virtual ~SubtreeIteratorBase() = default;

  reference operator*() { return *getFrontValuePtr(); }
  pointer operator->() { return getFrontValuePtr(); }

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
  virtual pointer getFrontValuePtr() const = 0;
};

template <typename NodePtrT, int num_children, TraversalOrder traversal_order>
class SubtreeIterator;

template <typename NodePtrT, int num_children>
class SubtreeIterator<NodePtrT, num_children,
                      TraversalOrder::kDepthFirstPreorder>
    : public SubtreeIteratorBase<
          NodePtrT, SubtreeIterator<NodePtrT, num_children,
                                    TraversalOrder::kDepthFirstPreorder>> {
 public:
  explicit SubtreeIterator(NodePtrT root_node);
  SubtreeIterator& operator++() override;

 private:
  std::deque<NodePtrT> upcoming_nodes_;
  size_t dequeSize() const override { return upcoming_nodes_.size(); }
  NodePtrT getFrontValuePtr() const override { return upcoming_nodes_.front(); }
  void enqueueNodeChildren(NodePtrT parent_ptr);
};

template <typename NodePtrT, int num_children>
class SubtreeIterator<NodePtrT, num_children,
                      TraversalOrder::kDepthFirstPostorder>
    : public SubtreeIteratorBase<
          NodePtrT, SubtreeIterator<NodePtrT, num_children,
                                    TraversalOrder::kDepthFirstPostorder>> {
 public:
  explicit SubtreeIterator(NodePtrT root_node);
  SubtreeIterator& operator++() override;

 private:
  struct StackElement {
    NodePtrT node_ptr;
    NdtreeIndexRelativeChild last_expanded_child_idx;
    bool operator==(const StackElement& rhs) const;
  };
  std::deque<StackElement> upcoming_nodes_;
  size_t dequeSize() const override { return upcoming_nodes_.size(); }
  NodePtrT getFrontValuePtr() const override;
  void enqueueNodeAndFirstChildren(NodePtrT parent_ptr);
};

template <typename NodePtrT, int num_children>
class SubtreeIterator<NodePtrT, num_children, TraversalOrder::kBreadthFirst>
    : public SubtreeIteratorBase<
          NodePtrT, SubtreeIterator<NodePtrT, num_children,
                                    TraversalOrder::kBreadthFirst>> {
 public:
  explicit SubtreeIterator(NodePtrT root_node);
  SubtreeIterator& operator++() override;

 private:
  std::deque<NodePtrT> upcoming_nodes_;
  size_t dequeSize() const override { return upcoming_nodes_.size(); }
  NodePtrT getFrontValuePtr() const override { return upcoming_nodes_.front(); }
  void enqueueNodeChildren(NodePtrT parent_ptr);
};

template <typename NodePtrT, int num_children, TraversalOrder traversal_order>
class Subtree {
 public:
  explicit Subtree(NodePtrT root_node) : root_node_(root_node) {}

  auto begin() const;
  auto end() const;

 private:
  NodePtrT const root_node_;
};
}  // namespace wavemap

#include "wavemap/core/utils/iterate/impl/subtree_iterator_inl.h"

#endif  // WAVEMAP_CORE_UTILS_ITERATE_SUBTREE_ITERATOR_H_
