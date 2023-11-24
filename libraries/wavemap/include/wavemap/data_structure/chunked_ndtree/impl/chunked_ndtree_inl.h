#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_INL_H_

#include <stack>
#include <utility>
#include <vector>

#include "wavemap/data_structure/pointcloud.h"
#include "wavemap/indexing/index_conversions.h"

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
  return num_chunks * NodeChunkType::kNumInnerNodes;
}

template <typename NodeDataT, int dim, int chunk_height>
void ChunkedNdtree<NodeDataT, dim, chunk_height>::prune() {
  for (NodeChunkType& chunk :
       getIterator<TraversalOrder::kDepthFirstPostorder>()) {
    if (chunk.hasChildrenArray()) {
      bool has_non_empty_child = false;
      for (LinearIndex child_idx = 0; child_idx < NodeChunkType::kNumChildren;
           ++child_idx) {
        NodeChunkType* child_ptr = chunk.getChild(child_idx);
        if (child_ptr) {
          if (child_ptr->empty()) {
            chunk.deleteChild(child_idx);
          } else {
            has_non_empty_child = true;
          }
        }
      }

      // Free up the children array if it only contains null pointers
      if (!has_non_empty_child) {
        chunk.deleteChildrenArray();
      }
    }
  }
}

template <typename NodeDataT, int dim, int chunk_height>
size_t ChunkedNdtree<NodeDataT, dim, chunk_height>::getMemoryUsage() const {
  size_t memory_usage = 0u;

  std::stack<const NodeChunkType*> stack;
  stack.emplace(&root_chunk_);
  while (!stack.empty()) {
    const NodeChunkType* chunk = stack.top();
    stack.pop();
    memory_usage += chunk->getMemoryUsage();

    if (chunk->hasChildrenArray()) {
      for (LinearIndex child_idx = 0; child_idx < NodeChunkType::kNumChildren;
           ++child_idx) {
        if (chunk->hasChild(child_idx)) {
          stack.emplace(chunk->getChild(child_idx));
        }
      }
    }
  }

  return memory_usage;
}

template <typename NodeDataT, int dim, int chunk_height>
bool ChunkedNdtree<NodeDataT, dim, chunk_height>::hasNode(
    const ChunkedNdtree::IndexType& index) const {
  return getChunkAndRelativeIndex(index).first;
}

template <typename NodeDataT, int dim, int chunk_height>
void ChunkedNdtree<NodeDataT, dim, chunk_height>::getOrAllocateNode(
    const IndexType& index) {
  getChunkAndRelativeIndex(index, /*auto_allocate*/ true);
}

template <typename NodeDataT, int dim, int chunk_height>
NodeDataT* ChunkedNdtree<NodeDataT, dim, chunk_height>::getNodeData(
    const ChunkedNdtree::IndexType& index, bool auto_allocate) {
  auto [chunk, relative_index] = getChunkAndRelativeIndex(index, auto_allocate);
  if (chunk) {
    return &chunk->nodeData(relative_index);
  }
  return nullptr;
}

template <typename NodeDataT, int dim, int chunk_height>
const NodeDataT* ChunkedNdtree<NodeDataT, dim, chunk_height>::getNodeData(
    const ChunkedNdtree::IndexType& index) const {
  auto [chunk, relative_index] = getChunkAndRelativeIndex(index);
  if (chunk) {
    return &chunk->nodeData(relative_index);
  }
  return nullptr;
}

template <typename NodeDataT, int dim, int chunk_height>
std::pair<typename ChunkedNdtree<NodeDataT, dim, chunk_height>::NodeChunkType*,
          LinearIndex>
ChunkedNdtree<NodeDataT, dim, chunk_height>::getChunkAndRelativeIndex(
    const ChunkedNdtree::IndexType& index, bool auto_allocate) {
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  const int chunk_top_height =
      chunk_height * int_math::div_round_up(index.height, chunk_height);

  NodeChunkType* current_parent = &root_chunk_;
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
std::pair<
    const typename ChunkedNdtree<NodeDataT, dim, chunk_height>::NodeChunkType*,
    LinearIndex>
ChunkedNdtree<NodeDataT, dim, chunk_height>::getChunkAndRelativeIndex(
    const ChunkedNdtree::IndexType& index) const {
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  const int chunk_top_height =
      chunk_height * int_math::div_round_up(index.height, chunk_height);

  const NodeChunkType* current_parent = &root_chunk_;
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
