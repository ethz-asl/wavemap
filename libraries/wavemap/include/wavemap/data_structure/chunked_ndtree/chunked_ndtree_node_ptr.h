#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_PTR_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_PTR_H_

#include "wavemap/common.h"
#include "wavemap/data_structure/chunked_ndtree/chunked_ndtree_chunk.h"
#include "wavemap/utils/data/comparisons.h"

namespace wavemap {
template <typename ChunkType>
class ChunkedNdtreeNodePtr {
 public:
  static constexpr int kDim = ChunkType::kDim;
  static constexpr int kNumChildren = NdtreeIndex<kDim>::kNumChildren;

  ChunkedNdtreeNodePtr() = default;
  ChunkedNdtreeNodePtr(ChunkType& chunk, IndexElement relative_node_depth,
                       LinearIndex level_traversal_distance)
      : ChunkedNdtreeNodePtr(&chunk, relative_node_depth,
                             level_traversal_distance) {}
  ChunkedNdtreeNodePtr(ChunkType* chunk, IndexElement relative_node_depth,
                       LinearIndex level_traversal_distance)
      : chunk_(chunk),
        relative_node_depth_(relative_node_depth),
        level_traversal_distance_(level_traversal_distance) {}

  operator bool() const { return chunk_; }  // NOLINT

  bool empty() const { return !hasAtLeastOneChild() && !hasNonzeroData(); }

  bool hasNonzeroData() const;
  bool hasNonzeroData(FloatingPoint threshold) const;
  auto& data();
  const auto& data() const;

  typename ChunkType::BitRef hasAtLeastOneChild();
  bool hasAtLeastOneChild() const;

  bool hasChild(NdtreeIndexRelativeChild child_index) const;

  ChunkedNdtreeNodePtr<ChunkType> getChild(
      NdtreeIndexRelativeChild child_index);
  ChunkedNdtreeNodePtr<const ChunkType> getChild(
      NdtreeIndexRelativeChild child_index) const;
  template <typename... DefaultArgs>
  ChunkedNdtreeNodePtr getOrAllocateChild(NdtreeIndexRelativeChild child_index,
                                          DefaultArgs&&... args) const;

 private:
  ChunkType* chunk_ = nullptr;
  IndexElement relative_node_depth_ = 0;
  LinearIndex level_traversal_distance_ = 0u;

  LinearIndex computeRelativeNodeIndex() const;
  LinearIndex computeChildLevelTraversalDistance(
      NdtreeIndexRelativeChild child_index) const;
};
}  // namespace wavemap

#include "wavemap/data_structure/chunked_ndtree/impl/chunked_ndtree_node_ptr_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_PTR_H_
