#ifndef WAVEMAP_DATA_STRUCTURE_NDTREE_NDTREE_NODE_H_
#define WAVEMAP_DATA_STRUCTURE_NDTREE_NDTREE_NODE_H_

#include <array>
#include <memory>

#include "wavemap/common.h"
#include "wavemap/indexing/ndtree_index.h"

namespace wavemap {
template <typename DataT, int dim>
class NdtreeNode {
 public:
  using DataType = DataT;
  static constexpr int kNumChildren = NdtreeIndex<dim>::kNumChildren;

  NdtreeNode() = default;
  explicit NdtreeNode(DataT data) : data_(data) {}
  ~NdtreeNode() = default;

  bool empty() const;
  void clear();

  friend bool operator==(const NdtreeNode& lhs, const NdtreeNode& rhs) {
    return &rhs == &lhs;
  }

  bool hasNonzeroData() const;
  bool hasNonzeroData(FloatingPoint threshold) const;
  DataT& data() { return data_; }
  const DataT& data() const { return data_; }

  bool hasChildrenArray() const { return static_cast<bool>(children_); }
  void deleteChildrenArray() { children_.reset(); }

  bool hasChild(NdtreeIndexRelativeChild child_index) const;
  bool hasAtLeastOneChild() const;
  template <typename... NodeConstructorArgs>
  NdtreeNode* allocateChild(NdtreeIndexRelativeChild child_index,
                            NodeConstructorArgs&&... args);
  bool deleteChild(NdtreeIndexRelativeChild child_index);
  NdtreeNode* getChild(NdtreeIndexRelativeChild child_index);
  const NdtreeNode* getChild(NdtreeIndexRelativeChild child_index) const;

  size_t getMemoryUsage() const;

 private:
  using ChildrenArray = std::array<std::unique_ptr<NdtreeNode>, kNumChildren>;

  DataT data_{};
  std::unique_ptr<ChildrenArray> children_;
};
}  // namespace wavemap

#include "wavemap/data_structure/ndtree/impl/ndtree_node_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_NDTREE_NDTREE_NODE_H_
