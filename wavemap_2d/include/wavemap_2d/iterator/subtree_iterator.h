#ifndef WAVEMAP_2D_ITERATOR_SUBTREE_ITERATOR_H_
#define WAVEMAP_2D_ITERATOR_SUBTREE_ITERATOR_H_

#include <deque>

#include "wavemap_2d/data_structure/generic/quadtree/node.h"
#include "wavemap_2d/indexing/quadtree_index.h"

namespace wavemap_2d {
enum class TraversalOrder {
  kDepthFirstPreorder,
  kDepthFirstPostorder,
  kBreadthFirst
};
// TODO(victorr): Add iterative deepening DFS as an alternative to BFS that uses
//                less memory (at the cost of increased compute)
// TODO(victorr): Add an IndexedSubtree class

template <typename NodeType, TraversalOrder traversal_order>
class Subtree {
 public:
  explicit Subtree(NodeType* root_node) : root_node_(root_node) {}

  class Iterator {
   public:
    using difference_type = std::ptrdiff_t;
    using value_type = NodeType;
    using pointer = NodeType*;
    using reference = NodeType&;
    using iterator_category = std::forward_iterator_tag;

    static_assert(
        traversal_order != TraversalOrder::kDepthFirstPostorder,
        "This class should only be compiled when using depth first preorder "
        "or breadth first traversal. Depth first postorder traversal is "
        "implemented separately through template specialization. If this "
        "message appears, the specialization must have broken.");

    explicit Iterator(NodeType* root_node) {
      if (root_node) {
        upcoming_nodes_.template emplace_front(root_node);
      }
    }

    NodeType& operator*() { return *upcoming_nodes_.front(); }
    const NodeType& operator*() const { return *upcoming_nodes_.front(); }
    NodeType* operator->() { return upcoming_nodes_.front(); }

    Iterator& operator++() {  // prefix ++
      NodeType* node_ptr = dequeueNode();
      enqueueNodeAndFirstChildren(node_ptr);

      return *this;
    }
    Iterator operator++(int) {  // postfix ++
      Iterator retval = *this;
      ++(*this);  // call the above prefix incrementer
      return retval;
    }

    friend bool operator==(const Iterator& lhs, const Iterator& rhs) {
      return (lhs.upcoming_nodes_.empty() && rhs.upcoming_nodes_.empty()) ||
             (lhs.upcoming_nodes_.size() == rhs.upcoming_nodes_.size() &&
              lhs.upcoming_nodes_.front() == rhs.upcoming_nodes_.front());
    }
    friend bool operator!=(const Iterator& lhs, const Iterator& rhs) {
      return !(lhs == rhs);  // NOLINT
    }

   protected:
    std::deque<NodeType*> upcoming_nodes_;
    NodeType* dequeueNode() {
      NodeType* node_ptr = upcoming_nodes_.front();
      upcoming_nodes_.pop_front();
      return node_ptr;
    }
    void enqueueNodeAndFirstChildren(NodeType* parent_ptr) {
      if (parent_ptr->hasChildrenArray()) {
        switch (traversal_order) {
          case TraversalOrder::kDepthFirstPreorder:
            for (int child_idx = NodeIndex::kNumChildren - 1; 0 <= child_idx;
                 --child_idx) {
              NodeType* child_ptr = parent_ptr->getChild(child_idx);
              if (child_ptr) {
                upcoming_nodes_.template emplace_front(child_ptr);
              }
            }
            break;
          case TraversalOrder::kBreadthFirst:
            for (int child_idx = 0; child_idx < NodeIndex::kNumChildren;
                 ++child_idx) {
              NodeType* child_ptr = parent_ptr->getChild(child_idx);
              if (child_ptr) {
                upcoming_nodes_.template emplace_back(child_ptr);
              }
            }
            break;
        }
      }
    }
  };

  auto begin() { return Iterator(root_node_); }
  auto end() { return Iterator(nullptr); }

 protected:
  NodeType* const root_node_;
};

template <typename NodeType>
class Subtree<NodeType, TraversalOrder::kDepthFirstPostorder> {
 public:
  explicit Subtree(NodeType* root_node) : root_node_(root_node) {}

  class Iterator {
   public:
    using difference_type = std::ptrdiff_t;
    using value_type = NodeType;
    using pointer = NodeType*;
    using reference = NodeType&;
    using iterator_category = std::forward_iterator_tag;

    explicit Iterator(NodeType* root_node) {
      if (root_node) {
        enqueueNodeAndFirstChildren(root_node);
      }
    }

    NodeType& operator*() { return *upcoming_nodes_.front().node_ptr; }
    const NodeType& operator*() const {
      return *upcoming_nodes_.front().node_ptr;
    }
    NodeType* operator->() { return upcoming_nodes_.front().node_ptr; }

    Iterator& operator++() {  // prefix ++
      upcoming_nodes_.pop_front();

      // Stop here if all nodes have already been visited
      if (upcoming_nodes_.empty()) {
        return *this;
      }

      // Otherwise enqueue the next node and its
      NodeAndState& node_and_state = upcoming_nodes_.front();
      for (++node_and_state.last_expanded_child_idx;
           node_and_state.last_expanded_child_idx < NodeIndex::kNumChildren;
           ++node_and_state.last_expanded_child_idx) {
        NodeType* child_ptr = node_and_state.node_ptr->getChild(
            node_and_state.last_expanded_child_idx);
        if (child_ptr) {
          enqueueNodeAndFirstChildren(child_ptr);
          break;
        }
      }
      return *this;
    }
    Iterator operator++(int) {  // postfix ++
      Iterator retval = *this;
      ++(*this);  // call the above prefix incrementer
      return retval;
    }

    friend bool operator==(const Iterator& lhs, const Iterator& rhs) {
      return (lhs.upcoming_nodes_.empty() && rhs.upcoming_nodes_.empty()) ||
             (lhs.upcoming_nodes_.size() == rhs.upcoming_nodes_.size() &&
              lhs.upcoming_nodes_.front() == rhs.upcoming_nodes_.front());
    }
    friend bool operator!=(const Iterator& lhs, const Iterator& rhs) {
      return !(lhs == rhs);  // NOLINT
    }

   protected:
    struct NodeAndState {
      NodeType* node_ptr;
      NodeRelativeChildIndex last_expanded_child_idx;
      bool operator==(const NodeAndState& rhs) const {
        return node_ptr == rhs.node_ptr &&
               last_expanded_child_idx == rhs.last_expanded_child_idx;
      }
    };
    std::deque<NodeAndState> upcoming_nodes_;
    void enqueueNodeAndFirstChildren(NodeType* parent_ptr) {
      if (parent_ptr->hasChildrenArray()) {
        // If the node has descendants, recursively enqueue all of its
        // descendants that have the lowest index on their respective level
        for (NodeRelativeChildIndex child_idx = 0;
             child_idx < NodeIndex::kNumChildren; ++child_idx) {
          NodeType* child_ptr = parent_ptr->getChild(child_idx);
          if (child_ptr) {
            upcoming_nodes_.template emplace_front(
                NodeAndState{parent_ptr, child_idx});
            enqueueNodeAndFirstChildren(child_ptr);
            return;
          }
        }
      } else {
        // Otherwise, only enqueue the node itself
        upcoming_nodes_.template emplace_front(
            NodeAndState{parent_ptr, NodeIndex::kNumChildren});
      }
    }
  };

  auto begin() { return Iterator(root_node_); }
  auto end() { return Iterator(nullptr); }

 protected:
  NodeType* const root_node_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_ITERATOR_SUBTREE_ITERATOR_H_
