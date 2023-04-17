#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_H_

#include <string>
#include <utility>
#include <vector>

#include "wavemap/common.h"
#include "wavemap/data_structure/chunked_ndtree/ndtree_node_chunk.h"
#include "wavemap/indexing/ndtree_index.h"
#include "wavemap/iterator/subtree_iterator.h"

namespace wavemap {
template <typename NodeDataT, int dim, int chunk_height>
class ChunkedNdtree {
 public:
  using IndexType = NdtreeIndex<dim>;
  using NodeChunkType = NdtreeNodeChunk<NodeDataT, dim, chunk_height>;
  using NodeDataType = NodeDataT;
  static constexpr int kChunkHeight = chunk_height;

  explicit ChunkedNdtree(int max_height);
  ~ChunkedNdtree() = default;

  bool empty() const { return root_chunk_.empty(); }
  size_t size() const;
  void clear() { root_chunk_.clear(); }
  void prune();

  bool hasNode(const IndexType& index) const;
  void allocateNode(const IndexType& index);
  NodeDataT* getNodeData(const IndexType& index, bool auto_allocate = true);
  const NodeDataT* getNodeData(const IndexType& index) const;

  int getMaxHeight() const { return max_height_; }
  NodeChunkType& getRootChunk() { return root_chunk_; }
  const NodeChunkType& getRootChunk() const { return root_chunk_; }

  template <TraversalOrder traversal_order>
  auto getIterator() {
    return Subtree<NodeChunkType, traversal_order>(&root_chunk_);
  }
  template <TraversalOrder traversal_order>
  auto getIterator() const {
    return Subtree<const NodeChunkType, traversal_order>(&root_chunk_);
  }

  size_t getMemoryUsage() const;

 private:
  NodeChunkType root_chunk_;
  const int max_height_;

  std::pair<NodeChunkType*, LinearIndex> getNodeAndRelativeIndex(
      const IndexType& index, bool auto_allocate);
  std::pair<const NodeChunkType*, LinearIndex> getNodeAndRelativeIndex(
      const IndexType& index) const;
};
}  // namespace wavemap

#include "wavemap/data_structure/chunked_ndtree/impl/chunked_ndtree_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_H_
