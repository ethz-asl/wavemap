#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/quadtree/node_index.h"

namespace wavemap_2d {
template <typename NodeDataType>
class Node {
 public:
  Node() : data_(0), children_(nullptr) {}
  ~Node() { pruneChildren(); }

  NodeDataType& data() { return data_; }
  const NodeDataType& data() const { return data_; }

  bool hasAllocatedChildrenArray() const { return children_; }
  void allocateChildrenArray();
  void pruneChildren();

  bool hasAtLeastOneChild() const;
  bool hasChild(NodeRelativeChildIndex child_index) const {
    return children_ && children_[child_index];
  }
  void allocateChild(NodeRelativeChildIndex child_index) const;
  Node* getChild(NodeRelativeChildIndex child_index);
  const Node* getChild(NodeRelativeChildIndex child_index) const;

 protected:
  NodeDataType data_;
  Node** children_;
};
}  // namespace wavemap_2d

#include "wavemap_2d/datastructure/quadtree/node_inl.h"

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_
