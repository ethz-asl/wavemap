#ifndef WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_H_

#include <string>
#include <utility>
#include <vector>

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/chunked_ndtree/chunked_ndtree_chunk.h"
#include "wavemap/core/data_structure/chunked_ndtree/chunked_ndtree_node_address.h"
#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/utils/iterate/subtree_iterator.h"

namespace wavemap {
template <typename NodeDataT, int dim, int chunk_height>
class ChunkedNdtree {
 public:
  using IndexType = NdtreeIndex<dim>;
  using HeightType = IndexElement;
  using ChunkType = ChunkedNdtreeChunk<NodeDataT, dim, chunk_height>;
  using NodeRefType = ChunkedNdtreeNodeRef<ChunkType>;
  using NodeConstRefType = ChunkedNdtreeNodeRef<const ChunkType>;
  using NodePtrType = ChunkedNdtreeNodePtr<ChunkType>;
  using NodeConstPtrType = ChunkedNdtreeNodePtr<const ChunkType>;
  using NodeDataType = NodeDataT;
  static constexpr HeightType kChunkHeight = chunk_height;

  explicit ChunkedNdtree(HeightType max_height);
  ~ChunkedNdtree() = default;

  bool empty() const { return getRootNode().empty(); }
  size_t size() const;
  void clear() { root_chunk_.clear(); }
  void prune();

  HeightType getMaxHeight() const { return max_height_; }
  size_t getMemoryUsage() const;

  bool hasNode(const IndexType& index) const { return getNode(index); }
  bool eraseNode(const IndexType& index);
  NodePtrType getNode(const IndexType& index);
  NodeConstPtrType getNode(const IndexType& index) const;
  template <typename... DefaultArgs>
  NodeRefType getOrAllocateNode(const IndexType& index, DefaultArgs&&... args);

  std::pair<NodePtrType, HeightType> getNodeOrAncestor(const IndexType& index);
  std::pair<NodeConstPtrType, HeightType> getNodeOrAncestor(
      const IndexType& index) const;

  ChunkType& getRootChunk() { return root_chunk_; }
  const ChunkType& getRootChunk() const { return root_chunk_; }

  NodeRefType getRootNode() { return {root_chunk_}; }
  NodeConstRefType getRootNode() const { return {root_chunk_}; }

  template <TraversalOrder traversal_order>
  auto getChunkIterator();
  template <TraversalOrder traversal_order>
  auto getChunkIterator() const;

 private:
  ChunkType root_chunk_;
  const HeightType max_height_;
};

template <typename NodeDataT, int chunk_height>
using ChunkedBinaryTree = ChunkedNdtree<NodeDataT, 1, chunk_height>;
template <typename NodeDataT, int chunk_height>
using ChunkedQuadtree = ChunkedNdtree<NodeDataT, 2, chunk_height>;
template <typename NodeDataT, int chunk_height>
using ChunkedOctree = ChunkedNdtree<NodeDataT, 3, chunk_height>;
}  // namespace wavemap

#include "wavemap/core/data_structure/chunked_ndtree/impl/chunked_ndtree_inl.h"

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_H_
