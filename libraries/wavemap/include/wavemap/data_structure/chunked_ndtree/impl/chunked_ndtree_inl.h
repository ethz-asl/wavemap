#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_INL_H_

#include <stack>
#include <utility>
#include <vector>

#include "wavemap/data_structure/pointcloud.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/utils/eigen_format.h"

namespace wavemap {
template <typename NodeDataT, int dim, int chunk_height>
ChunkedNdtree<NodeDataT, dim, chunk_height>::ChunkedNdtree(int max_height)
    : max_height_(chunk_height *
                  int_math::div_round_up(max_height, chunk_height)) {
  CHECK_EQ(max_height_ % chunk_height, 0);
}

template <typename NodeDataT, int dim, int chunk_height>
size_t ChunkedNdtree<NodeDataT, dim, chunk_height>::size() const {
  auto subtree_iterator = getIterator<TraversalOrder::kDepthFirstPreorder>();
  const size_t num_chunks =
      std::distance(subtree_iterator.begin(), subtree_iterator.end());
  return num_chunks * NodeType::kNumInnerNodes;
}

template <typename NodeDataT, int dim, int chunk_height>
void ChunkedNdtree<NodeDataT, dim, chunk_height>::prune() {
  for (NodeType& node : getIterator<TraversalOrder::kDepthFirstPostorder>()) {
    if (node.hasChildrenArray()) {
      bool has_non_empty_child = false;
      for (LinearIndex child_idx = 0; child_idx < NodeType::kNumChildren;
           ++child_idx) {
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

template <typename NodeDataT, int dim, int chunk_height>
bool ChunkedNdtree<NodeDataT, dim, chunk_height>::hasNode(
    const ChunkedNdtree::IndexType& index) const {
  return getNodeAndRelativeIndex(index).first;
}

template <typename NodeDataT, int dim, int chunk_height>
void ChunkedNdtree<NodeDataT, dim, chunk_height>::allocateNode(
    const ChunkedNdtree::IndexType& index) {
  getNodeAndRelativeIndex(index, /*auto_allocate*/ true);
}

template <typename NodeDataT, int dim, int chunk_height>
void ChunkedNdtree<NodeDataT, dim, chunk_height>::resetNode(
    const ChunkedNdtree::IndexType& index) {
  auto [chunked_node, relative_index] =
      getNodeAndRelativeIndex(index, /*auto_allocate*/ false);
  if (chunked_node) {
    chunked_node.data(relative_index) = NodeDataT{};
    // TODO Set the node's descendants in the chunked node to zero
    // TODO Reset all descendant child pointers
  }
}

template <typename NodeDataT, int dim, int chunk_height>
NodeDataT* ChunkedNdtree<NodeDataT, dim, chunk_height>::getNodeData(
    const ChunkedNdtree::IndexType& index, bool auto_allocate) {
  auto [chunked_node, relative_index] =
      getNodeAndRelativeIndex(index, auto_allocate);
  if (chunked_node) {
    return &chunked_node->data(relative_index);
  }
  return nullptr;
}

template <typename NodeDataT, int dim, int chunk_height>
const NodeDataT* ChunkedNdtree<NodeDataT, dim, chunk_height>::getNodeData(
    const ChunkedNdtree::IndexType& index) const {
  auto [chunked_node, relative_index] = getNodeAndRelativeIndex(index);
  if (chunked_node) {
    return &chunked_node->data(relative_index);
  }
  return nullptr;
}

template <typename NodeDataT, int dim, int chunk_height>
size_t ChunkedNdtree<NodeDataT, dim, chunk_height>::getMemoryUsage() const {
  size_t memory_usage = 0u;

  std::stack<const NodeType*> stack;
  stack.emplace(&root_node_);
  while (!stack.empty()) {
    const NodeType* node = stack.top();
    stack.pop();
    memory_usage += node->getMemoryUsage();

    if (node->hasChildrenArray()) {
      for (LinearIndex child_idx = 0; child_idx < NodeType::kNumChildren;
           ++child_idx) {
        if (node->hasChild(child_idx)) {
          stack.emplace(node->getChild(child_idx));
        }
      }
    }
  }

  return memory_usage;
}

template <typename NodeDataT, int dim, int chunk_height>
std::pair<typename ChunkedNdtree<NodeDataT, dim, chunk_height>::NodeType*,
          LinearIndex>
ChunkedNdtree<NodeDataT, dim, chunk_height>::getNodeAndRelativeIndex(
    const ChunkedNdtree::IndexType& index, bool auto_allocate) {
  const MortonCode morton_code = convert::nodeIndexToMorton(index);
  const int chunk_top_height =
      chunk_height * int_math::div_round_up(index.height, chunk_height);

  NodeType* current_parent = &root_node_;
  for (int parent_height = max_height_; chunk_top_height < parent_height;
       parent_height -= chunk_height) {
    const int child_height = parent_height - chunk_height;
    const LinearIndex child_index =
        NdtreeIndex<dim>::computeLevelTraversalDistance(
            morton_code, parent_height, child_height);
    // Check if the child is allocated
    if (!current_parent->hasChild(child_index)) {
      if (auto_allocate) {
        current_parent->allocateChild(child_index);
      } else {
        return {nullptr, 0u};
      }
    }
    current_parent = current_parent->getChild(child_index);
  }

  const LinearIndex relative_index =
      NdtreeIndex<dim>::computeTreeTraversalDistance(
          morton_code, chunk_top_height, index.height);

  return {current_parent, relative_index};
}

template <typename NodeDataT, int dim, int chunk_height>
std::pair<const typename ChunkedNdtree<NodeDataT, dim, chunk_height>::NodeType*,
          LinearIndex>
ChunkedNdtree<NodeDataT, dim, chunk_height>::getNodeAndRelativeIndex(
    const ChunkedNdtree::IndexType& index) const {
  const MortonCode morton_code = convert::nodeIndexToMorton(index);
  const int chunk_top_height =
      chunk_height * int_math::div_round_up(index.height, chunk_height);

  const NodeType* current_parent = &root_node_;
  for (int parent_height = max_height_; chunk_top_height < parent_height;
       parent_height -= chunk_height) {
    const int child_height = parent_height - chunk_height;
    const LinearIndex child_index =
        NdtreeIndex<dim>::computeLevelTraversalDistance(
            morton_code, parent_height, child_height);
    // Return if the child is not allocated
    if (!current_parent->hasChild(child_index)) {
      return {nullptr, 0u};
    }
    current_parent = current_parent->getChild(child_index);
  }

  const LinearIndex relative_index =
      NdtreeIndex<dim>::computeTreeTraversalDistance(
          morton_code, chunk_top_height, index.height);

  return {current_parent, relative_index};
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_INL_H_
