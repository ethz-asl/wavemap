#ifndef WAVEMAP_CORE_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_INL_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_INL_H_

#include <stack>
#include <utility>
#include <vector>

#include "wavemap/core/data_structure/pointcloud.h"
#include "wavemap/core/indexing/index_conversions.h"

namespace wavemap {
template <typename NodeDataT, int dim>
template <typename... RootNodeArgs>
Ndtree<NodeDataT, dim>::Ndtree(int max_height, RootNodeArgs&&... args)
    : max_height_(max_height), root_node_(std::forward<RootNodeArgs>(args)...) {
  CHECK_LE(max_height_, morton::kMaxTreeHeight<dim>);
}

template <typename NodeDataT, int dim>
size_t Ndtree<NodeDataT, dim>::size() const {
  auto subtree_iterator = getIterator<TraversalOrder::kDepthFirstPreorder>();
  return std::distance(subtree_iterator.begin(), subtree_iterator.end());
}

template <typename NodeDataT, int dim>
void Ndtree<NodeDataT, dim>::prune() {
  for (NodeType& node : getIterator<TraversalOrder::kDepthFirstPostorder>()) {
    if (node.hasChildrenArray()) {
      bool has_non_empty_child = false;
      for (NdtreeIndexRelativeChild child_idx = 0;
           child_idx < NodeType::kNumChildren; ++child_idx) {
        NodeType* child_ptr = node.getChild(child_idx);
        if (child_ptr) {
          if (child_ptr->empty()) {
            node.eraseChild(child_idx);
          } else {
            has_non_empty_child = true;
          }
        }
      }
      // Free up the children array if it only contains null pointers
      if (!has_non_empty_child) {
        node.deleteChildrenArray();
      }
    }
  }
}

template <typename NodeDataT, int dim>
size_t Ndtree<NodeDataT, dim>::getMemoryUsage() const {
  size_t memory_usage = 0u;
  for (const NodeType& node :
       getIterator<TraversalOrder::kDepthFirstPreorder>()) {
    memory_usage += node.getMemoryUsage();
  }
  return memory_usage;
}

template <typename NodeDataT, int dim>
bool Ndtree<NodeDataT, dim>::eraseNode(const IndexType& index) {
  IndexType parent_index = index.computeParentIndex();
  NodeType* parent_node = getNode(parent_index);
  if (parent_node) {
    return parent_node->eraseChild(index.computeRelativeChildIndex());
  }
  return false;
}

template <typename NodeDataT, int dim>
typename Ndtree<NodeDataT, dim>::NodeType* Ndtree<NodeDataT, dim>::getNode(
    const IndexType& index) {
  return const_cast<NodeType*>(std::as_const(*this).getNode(index));
}

template <typename NodeDataT, int dim>
const typename Ndtree<NodeDataT, dim>::NodeType*
Ndtree<NodeDataT, dim>::getNode(const IndexType& index) const {
  const NodeType* node = &root_node_;
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; node && index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    node = node->getChild(child_index);
  }
  return node;
}

template <typename NodeDataT, int dim>
template <typename... DefaultArgs>
typename Ndtree<NodeDataT, dim>::NodeType&
Ndtree<NodeDataT, dim>::getOrAllocateNode(const Ndtree::IndexType& index,
                                          DefaultArgs&&... args) {
  NodeType* node = &root_node_;
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    // Get the child, allocating if needed
    node = &node->getOrAllocateChild(child_index,
                                     std::forward<DefaultArgs>(args)...);
  }
  return *node;
}

template <typename NodeDataT, int dim>
std::pair<typename Ndtree<NodeDataT, dim>::NodeType*, IndexElement>
Ndtree<NodeDataT, dim>::getNodeOrAncestor(const Ndtree::IndexType& index) {
  auto rv = std::as_const(*this).getNodeOrAncestor(index);
  return {const_cast<NodeType*>(rv.first), rv.second};
}

template <typename NodeDataT, int dim>
std::pair<const typename Ndtree<NodeDataT, dim>::NodeType*, IndexElement>
Ndtree<NodeDataT, dim>::getNodeOrAncestor(
    const Ndtree::IndexType& index) const {
  const NodeType* node = &root_node_;
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    // Check if the child is allocated
    const NodeType* child = node->getChild(child_index);
    if (!child) {
      return {node, node_height};
    }
    node = child;
  }
  return {node, index.height};
}

template <typename NodeDataT, int dim>
template <TraversalOrder traversal_order>
auto Ndtree<NodeDataT, dim>::getIterator() {
  return Subtree<NodeType, traversal_order>(&root_node_);
}

template <typename NodeDataT, int dim>
template <TraversalOrder traversal_order>
auto Ndtree<NodeDataT, dim>::getIterator() const {
  return Subtree<const NodeType, traversal_order>(&root_node_);
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_INL_H_
