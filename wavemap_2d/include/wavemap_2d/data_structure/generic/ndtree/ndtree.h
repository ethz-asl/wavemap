#ifndef WAVEMAP_2D_DATA_STRUCTURE_GENERIC_NDTREE_NDTREE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_GENERIC_NDTREE_NDTREE_H_

#include <string>
#include <utility>
#include <vector>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/generic/ndtree/ndtree_node.h"
#include "wavemap_2d/data_structure/generic/pointcloud.h"
#include "wavemap_2d/data_structure/volumetric/volumetric_data_structure.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/indexing/index_conversions.h"
#include "wavemap_2d/indexing/ndtree_index.h"
#include "wavemap_2d/iterator/subtree_iterator.h"

namespace wavemap_2d {
template <typename NodeDataType, int dim, int max_height>
class Ndtree {
 public:
  using IndexType = NdtreeIndex<dim>;
  using NodeType = NdtreeNode<NodeDataType, dim>;

  Ndtree() = default;
  ~Ndtree() = default;

  bool empty() const { return root_node_.empty(); }
  size_t size() const;
  void clear() { root_node_.clear(); }
  void prune();

  bool hasNode(const IndexType& index) { return getNode(index); }
  void allocateNode(const IndexType& index) {
    constexpr bool kAutoAllocate = true;
    getNode(index, kAutoAllocate);
  }
  bool removeNode(const IndexType& index);
  NodeType* getNode(const IndexType& index, bool auto_allocate = false);
  const NodeType* getNode(const IndexType& index) const;

  NodeType& getRootNode() { return root_node_; }
  const NodeType& getRootNode() const { return root_node_; }

  template <TraversalOrder traversal_order>
  auto getIterator() {
    return Subtree<NodeType, traversal_order>(&root_node_);
  }
  template <TraversalOrder traversal_order>
  auto getIterator() const {
    return Subtree<const NodeType, traversal_order>(&root_node_);
  }

  size_t getMemoryUsage() const;

 private:
  NodeType root_node_;
};
}  // namespace wavemap_2d

#include "wavemap_2d/data_structure/generic/ndtree/impl/ndtree_inl.h"

#endif  // WAVEMAP_2D_DATA_STRUCTURE_GENERIC_NDTREE_NDTREE_H_
