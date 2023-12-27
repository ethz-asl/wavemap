#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_H_

#include <string>
#include <utility>
#include <vector>

#include "wavemap/common.h"
#include "wavemap/data_structure/chunked_ndtree/chunked_ndtree_chunk.h"
#include "wavemap/data_structure/chunked_ndtree/chunked_ndtree_node_ptr.h"
#include "wavemap/indexing/ndtree_index.h"
#include "wavemap/utils/iterate/subtree_iterator.h"

namespace wavemap {
template <typename NodeDataT, int dim, int chunk_height>
class ChunkedNdtree {
 public:
  using IndexType = NdtreeIndex<dim>;
  using HeightType = IndexElement;
  using ChunkType = ChunkedNdtreeChunk<NodeDataT, dim, chunk_height>;
  using NodePtrType = ChunkedNdtreeNodePtr<ChunkType>;
  using NodeConstPtrType = ChunkedNdtreeNodePtr<const ChunkType>;
  using NodeDataType = NodeDataT;
  static constexpr HeightType kChunkHeight = chunk_height;

  explicit ChunkedNdtree(HeightType max_height);
  ~ChunkedNdtree() = default;

  bool empty() const { return root_chunk_.empty(); }
  size_t size() const;
  void clear() { root_chunk_.clear(); }
  void prune();

  HeightType getMaxHeight() const { return max_height_; }
  size_t getMemoryUsage() const;

  // TODO(victorr): Add methods to directly query and operate on chunks,
  //                once a proper index type for chunks has been defined.
  //                The ChunkIndex type would be similar to NdtreeIndex, but has
  //                to account for the chunks having a branching factor that
  //                differs from 2 (probably 2^(dim * chunk_height)).

  bool hasNode(const IndexType& index) const { return getNode(index); }
  NodePtrType getNode(const IndexType& index);
  NodeConstPtrType getNode(const IndexType& index) const;
  template <typename... DefaultArgs>
  NodePtrType getOrAllocateNode(const IndexType& index, DefaultArgs&&... args);

  std::pair<NodePtrType, HeightType> getNodeOrAncestor(const IndexType& index);
  std::pair<NodeConstPtrType, HeightType> getNodeOrAncestor(
      const IndexType& index) const;

  ChunkType& getRootChunk() { return root_chunk_; }
  const ChunkType& getRootChunk() const { return root_chunk_; }

  NodePtrType getRootNode() { return {root_chunk_, 0, 0}; }
  NodeConstPtrType getRootNode() const { return {root_chunk_, 0, 0}; }

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

#include "wavemap/data_structure/chunked_ndtree/impl/chunked_ndtree_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_H_
