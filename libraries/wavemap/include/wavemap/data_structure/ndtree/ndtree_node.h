#ifndef WAVEMAP_DATA_STRUCTURE_NDTREE_NDTREE_NODE_H_
#define WAVEMAP_DATA_STRUCTURE_NDTREE_NDTREE_NODE_H_

#include <array>
#include <memory>

#include "wavemap/common.h"
#include "wavemap/indexing/ndtree_index.h"

namespace wavemap {
template <typename NodeDataType, int dim>
class NdtreeNode {
 public:
  using IndexType = NdtreeIndex<dim>;

  NdtreeNode() : data_{} {}
  explicit NdtreeNode(NodeDataType data) : data_(data) {}
  ~NdtreeNode() = default;

  bool empty() const { return data_ == NodeDataType{} && !hasChildrenArray(); }
  void clear() {
    deleteChildrenArray();
    data() = NodeDataType{};
  }

  friend bool operator==(const NdtreeNode& lhs, const NdtreeNode& rhs) {
    return &rhs == &lhs;
  }

  NodeDataType& data() { return data_; }
  const NodeDataType& data() const { return data_; }

  bool hasChildrenArray() const { return static_cast<bool>(children_); }
  void allocateChildrenArrayIfNeeded();
  void deleteChildrenArray();

  bool hasChild(typename IndexType::RelativeChild child_index) const;
  bool hasAtLeastOneChild() const;
  template <typename... NodeConstructorArgs>
  NdtreeNode* allocateChild(typename IndexType::RelativeChild child_index,
                            NodeConstructorArgs&&... args);
  bool deleteChild(typename IndexType::RelativeChild child_index);
  NdtreeNode* getChild(typename IndexType::RelativeChild child_index);
  const NdtreeNode* getChild(
      typename IndexType::RelativeChild child_index) const;

  size_t getMemoryUsage() const;

 private:
  using ChildrenArray =
      std::array<std::unique_ptr<NdtreeNode>, IndexType::kNumChildren>;

  NodeDataType data_;
  std::unique_ptr<ChildrenArray> children_;
};
}  // namespace wavemap

#include "wavemap/data_structure/ndtree/impl/ndtree_node_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_NDTREE_NDTREE_NODE_H_
