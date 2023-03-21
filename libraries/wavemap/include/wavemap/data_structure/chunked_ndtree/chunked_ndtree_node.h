#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_H_

#include <array>
#include <limits>
#include <memory>

#include "wavemap/common.h"
#include "wavemap/indexing/ndtree_index.h"
#include "wavemap/utils/tree_math.h"

namespace wavemap {
template <typename DataT, int dim, int height>
class ChunkedNdtreeNode {
 public:
  using DataType = DataT;

  static constexpr int kNumInnerNodes =
      tree_math::perfect_tree::num_total_nodes<dim>(height);
  static constexpr int kNumChildren =
      tree_math::perfect_tree::num_leaf_nodes<dim>(height + 1);

  ChunkedNdtreeNode() = default;
  ~ChunkedNdtreeNode() = default;

  bool empty() const;
  void clear();

  friend bool operator==(const ChunkedNdtreeNode& lhs,
                         const ChunkedNdtreeNode& rhs) {
    return &rhs == &lhs;
  }

  DataT& data(LinearIndex linear_index);
  const DataT& data(LinearIndex linear_index) const;

  bool hasChildrenArray() const { return static_cast<bool>(children_); }
  void allocateChildrenArrayIfNeeded();
  void deleteChildrenArray() { children_.reset(); }

  bool hasChild(LinearIndex child_index) const;
  bool hasAtLeastOneChild() const;
  ChunkedNdtreeNode* allocateChild(LinearIndex child_index);
  bool deleteChild(LinearIndex child_index);
  ChunkedNdtreeNode* getChild(LinearIndex child_index);
  const ChunkedNdtreeNode* getChild(LinearIndex child_index) const;

  size_t getMemoryUsage() const;

 private:
  using DataArray = std::array<DataT, kNumInnerNodes>;
  using ChildrenArray =
      std::array<std::unique_ptr<ChunkedNdtreeNode>, kNumChildren>;

  DataArray data_{};
  std::unique_ptr<ChildrenArray> children_;
};
}  // namespace wavemap

#include "wavemap/data_structure/chunked_ndtree/impl/chunked_ndtree_node_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_H_
