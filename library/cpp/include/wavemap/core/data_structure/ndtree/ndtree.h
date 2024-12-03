#ifndef WAVEMAP_CORE_DATA_STRUCTURE_NDTREE_NDTREE_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_NDTREE_NDTREE_H_

#include <string>
#include <utility>
#include <vector>

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/ndtree/ndtree_node.h"
#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/utils/iterate/subtree_iterator.h"

namespace wavemap {
template <typename NodeDataT, int dim>
class Ndtree {
 public:
  using IndexType = NdtreeIndex<dim>;
  using HeightType = IndexElement;
  using NodeType = NdtreeNode<NodeDataT, dim>;
  using NodeRefType = NodeType&;
  using NodeConstRefType = const NodeType&;
  using NodePtrType = NodeType*;
  using NodeConstPtrType = const NodeType*;
  using NodeDataType = NodeDataT;
  static constexpr HeightType kChunkHeight = 1;

  template <typename... RootNodeArgs>
  explicit Ndtree(HeightType max_height, RootNodeArgs&&... args);
  ~Ndtree() = default;

  // Deep copy constructor
  template <typename OtherTreeT>
  explicit Ndtree(const OtherTreeT& other_tree);

  // Delete the copy assignment operator to avoid expensive accidental copies
  Ndtree& operator=(const Ndtree&) = delete;

  // Move constructor and assignment operator
  Ndtree(Ndtree&&) = default;
  Ndtree& operator=(Ndtree&&) = default;

  bool empty() const { return root_node_.empty(); }
  size_t size() const;
  void clear() { root_node_.clear(); }
  void prune();

  HeightType getMaxHeight() const { return max_height_; }
  size_t getMemoryUsage() const;

  bool hasNode(const IndexType& index) const { return getNode(index); }
  bool eraseNode(const IndexType& index);
  NodeType* getNode(const IndexType& index);
  const NodeType* getNode(const IndexType& index) const;
  template <typename... DefaultArgs>
  NodeType& getOrAllocateNode(const IndexType& index, DefaultArgs&&... args);

  std::pair<NodeType*, HeightType> getNodeOrAncestor(const IndexType& index);
  std::pair<const NodeType*, HeightType> getNodeOrAncestor(
      const IndexType& index) const;

  NodeType& getRootNode() { return root_node_; }
  const NodeType& getRootNode() const { return root_node_; }

  template <TraversalOrder traversal_order>
  auto getIterator();
  template <TraversalOrder traversal_order>
  auto getIterator() const;

 private:
  const HeightType max_height_;
  NodeType root_node_;

  template <typename OtherNodeConstRefT>
  void clone(OtherNodeConstRefT other_node, NodeRefType node);
};

template <typename NodeDataT>
using BinaryTree = Ndtree<NodeDataT, 1>;
template <typename NodeDataT>
using Quadtree = Ndtree<NodeDataT, 2>;
template <typename NodeDataT>
using Octree = Ndtree<NodeDataT, 3>;
}  // namespace wavemap

#include "wavemap/core/data_structure/ndtree/impl/ndtree_inl.h"

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_NDTREE_NDTREE_H_
