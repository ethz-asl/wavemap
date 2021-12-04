#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/quadtree/node_index.h"

namespace wavemap_2d {
template <typename NodeDataType>
class Node {
 public:
  struct IndexPointerPair {
    NodeIndex index;
    Node<NodeDataType>* ptr;
    bool operator==(const IndexPointerPair& other) const {
      return index == other.index && ptr == other.ptr;
    }
    bool operator!=(const IndexPointerPair& other) const {
      return !(*this == other);  // NOLINT
    }
  };
  struct IndexConstPointerPair {
    NodeIndex index;
    const Node<NodeDataType>* ptr;
    bool operator==(const IndexConstPointerPair& other) const {
      return index == other.index && ptr == other.ptr;
    }
    bool operator!=(const IndexConstPointerPair& other) const {
      return !(*this == other);  // NOLINT
    }
  };

  Node() : children_(nullptr) {}
  ~Node() { pruneChildren(); }

  void allocateChildren();
  bool hasAllocatedChildren() const;
  bool hasNotNullChildren() const;
  void pruneChildren();

  bool hasChild(NodeRelativeChildIndex child_index) const {
    CHECK_NOTNULL(children_);
    return children_[child_index];
  }
  Node*& getChildPtr(NodeRelativeChildIndex child_index) {
    CHECK_NOTNULL(children_);
    return children_[child_index];
  }
  const Node* getChildConstPtr(NodeRelativeChildIndex child_index) const {
    CHECK_NOTNULL(children_);
    return children_[child_index];
  }

  const NodeDataType& getNodeData() const { return data_; }
  NodeDataType* getNodeDataPtr() { return &data_; }
  const NodeDataType* getNodeDataConstPtr() const { return &data_; }

 protected:
  NodeDataType data_;
  Node** children_;
};
}  // namespace wavemap_2d

#include "wavemap_2d/datastructure/quadtree/node_inl.h"

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_H_
