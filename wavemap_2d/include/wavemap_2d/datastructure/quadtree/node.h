#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_

#include <array>
#include <memory>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/quadtree/node_index.h"

namespace wavemap_2d {
template <typename NodeDataType>
class Node {
 public:
  Node() : data_(0) {}
  ~Node() { pruneChildren(); }

  NodeDataType& data() { return data_; }
  const NodeDataType& data() const { return data_; }

  bool hasAllocatedChildrenArray() const {
    return static_cast<bool>(children_);
  }
  void allocateChildrenArray();
  void pruneChildren();

  size_t getMemoryUsage() const;

  bool hasAtLeastOneChild() const;
  bool hasChild(NodeRelativeChildIndex child_index) const {
    return children_ && children_->operator[](child_index);
  }

  void allocateChild(NodeRelativeChildIndex child_index);
  bool deleteChild(NodeRelativeChildIndex child_index);
  Node* getChild(NodeRelativeChildIndex child_index);
  const Node* getChild(NodeRelativeChildIndex child_index) const;

 protected:
  using ChildrenArray =
      std::array<std::unique_ptr<Node>, NodeIndex::kNumChildren>;

  NodeDataType data_;
  std::unique_ptr<ChildrenArray> children_;
};
}  // namespace wavemap_2d

#include "wavemap_2d/datastructure/quadtree/node_inl.h"

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_
