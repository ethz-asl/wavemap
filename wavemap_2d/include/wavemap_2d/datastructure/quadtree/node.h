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
  ~Node() = default;

  bool empty() const {
    return (data_ == static_cast<NodeDataType>(0)) && !hasAtLeastOneChild();
  }
  void pruneChildren();

  NodeDataType& data() { return data_; }
  const NodeDataType& data() const { return data_; }

  bool hasChildrenArray() const { return static_cast<bool>(children_); }
  void allocateChildrenArrayIfNeeded();
  void deleteChildrenArray();

  bool hasChild(NodeRelativeChildIndex child_index) const;
  bool hasAtLeastOneChild() const;
  void allocateChild(NodeRelativeChildIndex child_index);
  bool deleteChild(NodeRelativeChildIndex child_index);
  Node* getChild(NodeRelativeChildIndex child_index);
  const Node* getChild(NodeRelativeChildIndex child_index) const;

  template <typename Functor>
  void applyToChildren(Functor&& fn);
  template <typename Functor>
  void applyToChildren(Functor&& fn) const;
  template <typename Functor>
  void applyBottomUp(Functor&& fn);
  template <typename Functor>
  void applyBottomUp(Functor&& fn) const;
  template <typename Functor>
  void applyTopDown(Functor&& fn);
  template <typename Functor>
  void applyTopDown(Functor&& fn) const;

  size_t getMemoryUsage() const;

 protected:
  using ChildrenArray =
      std::array<std::unique_ptr<Node>, NodeIndex::kNumChildren>;

  NodeDataType data_;
  std::unique_ptr<ChildrenArray> children_;
};
}  // namespace wavemap_2d

#include "wavemap_2d/datastructure/quadtree/node_inl.h"

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_
