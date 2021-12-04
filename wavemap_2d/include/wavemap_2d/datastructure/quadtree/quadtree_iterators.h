#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_ITERATORS_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_ITERATORS_H_

#include <deque>

namespace wavemap_2d {
template <typename NodeDataType>
class Quadtree;

enum class TraversalOrder { kBreadthFirst, kDepthFirst };

template <typename NodeDataType, TraversalOrder traversal_order>
class QuadtreeIterator {
 public:
  explicit QuadtreeIterator(const Quadtree<NodeDataType>* quadtree)
      : quadtree_(quadtree) {}

  using IndexConstPointerPair =
      typename Node<NodeDataType>::IndexConstPointerPair;
  class iterator
      : public std::iterator<std::forward_iterator_tag, IndexConstPointerPair> {
   public:
    explicit iterator(const Quadtree<NodeDataType>* quadtree) {
      if (quadtree) {
        IndexConstPointerPair root_node = {NodeIndex(),
                                           quadtree->getRootNodeConstPtr()};
        next_nodes_.emplace_front(root_node);
      }
    }
    iterator(const iterator& other) : next_nodes_(other.next_nodes_) {}
    iterator& operator=(const iterator& other) { this->queue_ = other.queue_; }

    iterator& operator++() {  // prefix ++
      IndexConstPointerPair node = next_nodes_.front();
      next_nodes_.pop_front();

      if (node.ptr->hasAllocatedChildren()) {
        for (NodeRelativeChildIndex relative_child_idx = 0;
             relative_child_idx < NodeIndex::kNumChildren;
             ++relative_child_idx) {
          if (node.ptr->hasChild(relative_child_idx)) {
            IndexConstPointerPair child_node = {
                node.index.computeChildIndex(relative_child_idx),
                node.ptr->getChildConstPtr(relative_child_idx)};
            enqueueChildNode(child_node);
          }
        }
      }

      return *this;
    }
    iterator operator++(int) {  // postfix ++
      iterator retval = *this;
      ++(*this);  // call the above prefix incrementer
      return retval;
    }

    bool operator==(const iterator& other) const {
      return (next_nodes_.empty() && other.next_nodes_.empty()) ||
             (!(next_nodes_.empty() ^ other.next_nodes_.empty()) &&
              (next_nodes_.front() == other.next_nodes_.front()));
    }
    bool operator!=(const iterator& other) const {
      return !(*this == other);  // NOLINT
    }

    const IndexConstPointerPair& operator*() const {
      return next_nodes_.front();
    }
    const IndexConstPointerPair* operator->() const {
      return &next_nodes_.front();
    }

   private:
    std::deque<IndexConstPointerPair> next_nodes_;
    void enqueueChildNode(IndexConstPointerPair child_node) {
      switch (traversal_order) {
        case (TraversalOrder::kBreadthFirst):
          next_nodes_.emplace_back(child_node);
          break;
        case (TraversalOrder::kDepthFirst):
          next_nodes_.emplace_front(child_node);
          break;
      }
    }
  };

  iterator begin() { return iterator(quadtree_); }
  iterator end() { return iterator(nullptr); }

 private:
  const Quadtree<NodeDataType>* quadtree_;
};

}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_ITERATORS_H_
