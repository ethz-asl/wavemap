#ifndef WAVEMAP_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_INL_H_

#include <stack>
#include <vector>

#include "wavemap/data_structure/pointcloud.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/utils/eigen_format.h"

namespace wavemap {
template <typename NodeDataT, int dim>
Ndtree<NodeDataT, dim>::Ndtree(int max_height) : max_height_(max_height) {
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
            node.deleteChild(child_idx);
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

  std::stack<const NodeType*> stack;
  stack.emplace(&root_node_);
  while (!stack.empty()) {
    const NodeType* node = stack.top();
    stack.pop();
    memory_usage += node->getMemoryUsage();

    if (node->hasChildrenArray()) {
      for (NdtreeIndexRelativeChild child_idx = 0;
           child_idx < NodeType::kNumChildren; ++child_idx) {
        if (node->hasChild(child_idx)) {
          stack.emplace(node->getChild(child_idx));
        }
      }
    }
  }

  return memory_usage;
}

template <typename NodeDataT, int dim>
bool Ndtree<NodeDataT, dim>::deleteNode(const IndexType& index) {
  IndexType parent_index = index.computeParentIndex();
  NodeType* parent_node = getNode(parent_index, /*auto_allocate*/ false);
  if (parent_node) {
    return parent_node->deleteChild(index.computeRelativeChildIndex());
  }
  return false;
}

template <typename NodeDataT, int dim>
NodeDataT* Ndtree<NodeDataT, dim>::getNodeData(const Ndtree::IndexType& index,
                                               bool auto_allocate) {
  if (NodeType* node = getNode(index, auto_allocate); node) {
    return &node->data();
  }
  return nullptr;
}

template <typename NodeDataT, int dim>
const NodeDataT* Ndtree<NodeDataT, dim>::getNodeData(
    const Ndtree::IndexType& index) const {
  if (const NodeType* node = getNode(index); node) {
    return &node->data();
  }
  return nullptr;
}

template <typename NodeDataT, int dim>
typename Ndtree<NodeDataT, dim>::NodeType* Ndtree<NodeDataT, dim>::getNode(
    const IndexType& index, bool auto_allocate) {
  NodeType* current_parent = &root_node_;
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int parent_height = max_height_; index.height < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, parent_height);
    // Check if the child is allocated
    if (!current_parent->hasChild(child_index)) {
      if (auto_allocate) {
        current_parent->allocateChild(child_index);
      } else {
        return nullptr;
      }
    }

    current_parent = current_parent->getChild(child_index);
  }

  return current_parent;
}

template <typename NodeDataT, int dim>
const typename Ndtree<NodeDataT, dim>::NodeType*
Ndtree<NodeDataT, dim>::getNode(const IndexType& index) const {
  const NodeType* current_parent = &root_node_;
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int parent_height = max_height_; index.height < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, parent_height);
    // Check if the child is allocated
    if (!current_parent->hasChild(child_index)) {
      return nullptr;
    }

    current_parent = current_parent->getChild(child_index);
  }

  return current_parent;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_NDTREE_IMPL_NDTREE_INL_H_
