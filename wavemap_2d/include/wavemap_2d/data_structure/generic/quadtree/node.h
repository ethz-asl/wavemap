#ifndef WAVEMAP_2D_DATA_STRUCTURE_GENERIC_QUADTREE_NODE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_GENERIC_QUADTREE_NODE_H_

#include <array>
#include <memory>

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/quadtree_index.h"

namespace wavemap_2d {
template <typename NodeDataType>
class Node {
 public:
  Node() : data_(0) {}
  explicit Node(NodeDataType data) : data_(data) {}
  ~Node() = default;

  bool empty() const {
    return (data_ == static_cast<NodeDataType>(0)) && !hasAtLeastOneChild();
  }

  NodeDataType& data() { return data_; }
  const NodeDataType& data() const { return data_; }

  bool hasChildrenArray() const { return static_cast<bool>(children_); }
  void allocateChildrenArrayIfNeeded();
  void deleteChildrenArray();

  bool hasChild(NodeRelativeChildIndex child_index) const;
  bool hasAtLeastOneChild() const;
  template <typename... NodeConstructorArgs>
  Node* allocateChild(NodeRelativeChildIndex child_index,
                      NodeConstructorArgs&&... args);
  bool deleteChild(NodeRelativeChildIndex child_index);
  Node* getChild(NodeRelativeChildIndex child_index);
  const Node* getChild(NodeRelativeChildIndex child_index) const;

  size_t getMemoryUsage() const;

 protected:
  using ChildrenArray =
      std::array<std::unique_ptr<Node>, NodeIndex::kNumChildren>;

  NodeDataType data_;
  std::unique_ptr<ChildrenArray> children_;
};
}  // namespace wavemap_2d

#include "wavemap_2d/data_structure/generic/quadtree/impl/node_inl.h"

#endif  // WAVEMAP_2D_DATA_STRUCTURE_GENERIC_QUADTREE_NODE_H_
