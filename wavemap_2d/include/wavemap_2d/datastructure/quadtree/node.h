#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/quadtree/node_index.h"

namespace wavemap_2d {
template <typename NodeDataType>
class Node {
 public:
  template <typename PointerT>
  struct IndexPointerPair {
    NodeIndex index;
    PointerT ptr;
    bool operator==(const IndexPointerPair& other) const {
      return index == other.index && ptr == other.ptr;
    }
    bool operator!=(const IndexPointerPair& other) const {
      return !(*this == other);  // NOLINT
    }
  };

  Node() : data_(0), children_(nullptr) {}
  ~Node() { pruneChildren(); }

  NodeDataType& data() { return data_; }
  const NodeDataType& data() const { return data_; }

  // TODO(victorr): Give these methods clearer names
  void allocateChildren();
  bool hasAllocatedChildren() const;
  bool hasNotNullChildren() const;
  void pruneChildren();

  bool hasChild(NodeRelativeChildIndex child_index) const {
    CHECK_NOTNULL(children_);
    return children_[child_index];
  }
  // TODO(victorr): Clean this up
  Node*& getChildPtr(NodeRelativeChildIndex child_index) {
    CHECK_NOTNULL(children_);
    return children_[child_index];
  }
  const Node* getChildPtr(NodeRelativeChildIndex child_index) const {
    CHECK_NOTNULL(children_);
    return children_[child_index];
  }

 protected:
  NodeDataType data_;
  Node** children_;
};
}  // namespace wavemap_2d

#include "wavemap_2d/datastructure/quadtree/node_inl.h"

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_
