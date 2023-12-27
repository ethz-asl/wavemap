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
  auto subtree_iterator =
      getChunkIterator<TraversalOrder::kDepthFirstPreorder>();
  const size_t num_chunks =
      std::distance(subtree_iterator.begin(), subtree_iterator.end());
  return num_chunks * ChunkType::kNumInnerNodes;
}

template <typename NodeDataT, int dim, int chunk_height>
void ChunkedNdtree<NodeDataT, dim, chunk_height>::prune() {
  for (ChunkType& chunk :
       getChunkIterator<TraversalOrder::kDepthFirstPostorder>()) {
    if (chunk.hasChildrenArray()) {
      bool has_non_empty_child = false;
      for (LinearIndex child_idx = 0; child_idx < ChunkType::kNumChildren;
           ++child_idx) {
        ChunkType* child_ptr = chunk.getChild(child_idx);
        if (child_ptr) {
          if (child_ptr->empty()) {
            chunk.eraseChild(child_idx);
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

  std::stack<const ChunkType*> stack;
  stack.emplace(&root_chunk_);
  while (!stack.empty()) {
    const ChunkType* chunk = stack.top();
    stack.pop();
    memory_usage += chunk->getMemoryUsage();

    if (chunk->hasChildrenArray()) {
      for (LinearIndex child_idx = 0; child_idx < ChunkType::kNumChildren;
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
typename ChunkedNdtree<NodeDataT, dim, chunk_height>::NodePtrType
ChunkedNdtree<NodeDataT, dim, chunk_height>::getNode(
    const ChunkedNdtree::IndexType& index) {
  NodePtrType node = getRootNode();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    // Check if the child is allocated
    NodePtrType child = node.getChild(child_index);
    if (!child) {
      return {};
    }
    node = child;
  }
  return node;
}

template <typename NodeDataT, int dim, int chunk_height>
typename ChunkedNdtree<NodeDataT, dim, chunk_height>::NodeConstPtrType
ChunkedNdtree<NodeDataT, dim, chunk_height>::getNode(
    const ChunkedNdtree::IndexType& index) const {
  NodeConstPtrType node = getRootNode();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    // Check if the child is allocated
    NodeConstPtrType child = node.getChild(child_index);
    if (!child) {
      return {};
    }
    node = child;
  }
  return node;
}

template <typename NodeDataT, int dim, int chunk_height>
template <typename... DefaultArgs>
typename ChunkedNdtree<NodeDataT, dim, chunk_height>::NodePtrType
ChunkedNdtree<NodeDataT, dim, chunk_height>::getOrAllocateNode(
    const ChunkedNdtree::IndexType& index, DefaultArgs&&... args) {
  NodePtrType node = getRootNode();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    // Get the child, allocating if needed
    node = node.getOrAllocateChild(child_index,
                                   std::forward<DefaultArgs>(args)...);
  }
  return node;
}

template <typename NodeDataT, int dim, int chunk_height>
std::pair<typename ChunkedNdtree<NodeDataT, dim, chunk_height>::NodePtrType,
          typename ChunkedNdtree<NodeDataT, dim, chunk_height>::HeightType>
ChunkedNdtree<NodeDataT, dim, chunk_height>::getNodeOrAncestor(
    const ChunkedNdtree::IndexType& index) {
  NodePtrType node = getRootNode();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    // Check if the child is allocated
    NodePtrType child = node.getChild(child_index);
    if (!child) {
      return {node, node_height};
    }
    node = child;
  }
  return {node, index.height};
}

template <typename NodeDataT, int dim, int chunk_height>
std::pair<
    typename ChunkedNdtree<NodeDataT, dim, chunk_height>::NodeConstPtrType,
    typename ChunkedNdtree<NodeDataT, dim, chunk_height>::HeightType>
ChunkedNdtree<NodeDataT, dim, chunk_height>::getNodeOrAncestor(
    const ChunkedNdtree::IndexType& index) const {
  NodeConstPtrType node = getRootNode();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    // Check if the child is allocated
    NodeConstPtrType child = node.getChild(child_index);
    if (!child) {
      return {node, node_height};
    }
    node = child;
  }
  return {node, index.height};
}

template <typename NodeDataT, int dim, int chunk_height>
template <TraversalOrder traversal_order>
auto ChunkedNdtree<NodeDataT, dim, chunk_height>::getChunkIterator() {
  return Subtree<ChunkType, traversal_order>(&root_chunk_);
}

template <typename NodeDataT, int dim, int chunk_height>
template <TraversalOrder traversal_order>
auto ChunkedNdtree<NodeDataT, dim, chunk_height>::getChunkIterator() const {
  return Subtree<const ChunkType, traversal_order>(&root_chunk_);
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_INL_H_
