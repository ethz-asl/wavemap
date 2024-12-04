#ifndef WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_LINEAR_NDTREE_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_LINEAR_NDTREE_H_

#include <limits>
#include <utility>
#include <vector>

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/linear_ndtree/linear_ndtree_node_address.h"
#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/indexing/ndtree_index.h"

namespace wavemap {
template <typename NodeDataT, int dim>
class LinearNdtree {
 public:
  using IndexType = NdtreeIndex<dim>;
  using HeightType = IndexElement;
  using NodeOffsetType = uint32_t;
  using NodeRefType = LinearNdtreeNodeRef<LinearNdtree>;
  using NodeConstRefType = LinearNdtreeNodeRef<const LinearNdtree>;
  using NodePtrType = LinearNdtreeNodePtr<LinearNdtree>;
  using NodeConstPtrType = LinearNdtreeNodePtr<const LinearNdtree>;
  using NodeDataType = NodeDataT;

  static constexpr IndexElement kMaxHeight =
      convert::nodeOffsetToDepth<dim, size_t>(
          std::numeric_limits<NodeOffsetType>::max());

  explicit LinearNdtree(HeightType max_height);
  ~LinearNdtree() = default;

  // Delete copy constructor and assignment operator to avoid accidental copies
  LinearNdtree(const LinearNdtree& other_tree) = delete;
  LinearNdtree& operator=(const LinearNdtree&) = delete;

  // Allow move construction and assignments
  LinearNdtree(LinearNdtree&&) = default;
  LinearNdtree& operator=(LinearNdtree&&) = default;

  // Explicit deep copies, with support for type conversions
  template <typename OtherTreeT>
  static LinearNdtree from(const OtherTreeT& other_tree);

  bool empty() const { return first_child_offset_.empty(); }
  size_t size() const { return first_child_offset_.size(); }
  void clear();

  HeightType getMaxHeight() const { return max_height_; }
  size_t getMemoryUsage() const;

  // Methods to operate on nodes given their global index
  bool hasNode(const IndexType& index) const { return getNode(index); }
  NodePtrType getNode(const IndexType& index);
  NodeConstPtrType getNode(const IndexType& index) const;

  std::pair<NodePtrType, HeightType> getNodeOrAncestor(const IndexType& index);
  std::pair<NodeConstPtrType, HeightType> getNodeOrAncestor(
      const IndexType& index) const;

  NodeRefType getRootNode() { return {*this}; }
  NodeConstRefType getRootNode() const { return {*this}; }

  template <TraversalOrder traversal_order>
  auto getIterator();
  template <TraversalOrder traversal_order>
  auto getIterator() const;

  // Methods to operate on nodes given their relative position inside the tree
  bool nodeHasNonzeroData(NodeOffsetType relative_node_index) const;
  bool nodeHasNonzeroData(NodeOffsetType relative_node_index,
                          FloatingPoint threshold) const;

  NodeDataT& nodeData(NodeOffsetType relative_node_index);
  const NodeDataT& nodeData(NodeOffsetType relative_node_index) const;

  bool nodeHasAtLeastOneChild(NodeOffsetType relative_node_index) const;
  bool nodeHasChild(NodeOffsetType relative_node_index,
                    NdtreeIndexRelativeChild child_index) const;
  std::optional<NodeOffsetType> getChildOffset(
      NodeOffsetType relative_node_index,
      NdtreeIndexRelativeChild child_index) const;

 private:
  using ChildAllocationMaskType = uint8_t;

  const HeightType max_height_;

  std::vector<NodeOffsetType> first_child_offset_;
  std::vector<ChildAllocationMaskType> allocated_child_mask_;
  std::vector<NodeDataT> node_data_;
};

template <typename NodeDataT>
using LinearBinaryTree = LinearNdtree<NodeDataT, 1>;
template <typename NodeDataT>
using LinearQuadtree = LinearNdtree<NodeDataT, 2>;
template <typename NodeDataT>
using LinearOctree = LinearNdtree<NodeDataT, 3>;
}  // namespace wavemap

#include "wavemap/core/data_structure/linear_ndtree/impl/linear_ndtree_inl.h"

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_LINEAR_NDTREE_H_
