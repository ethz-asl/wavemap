#ifndef WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_CHUNK_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_CHUNK_H_

#include <array>
#include <bitset>
#include <limits>
#include <memory>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/utils/math/tree_math.h"

namespace wavemap {
template <typename DataT, int dim, int height>
class ChunkedNdtreeChunk {
 public:
  using DataType = DataT;
  using NodeOffsetType = uint32_t;

  static constexpr int kDim = dim;
  static constexpr int kHeight = height;
  static constexpr int kNumInnerNodes =
      tree_math::perfect_tree::num_total_nodes<dim>(height);
  static constexpr int kNumChildren =
      tree_math::perfect_tree::num_leaf_nodes<dim>(height + 1);
  static constexpr NodeOffsetType kMaxNodeOffset =
      tree_math::perfect_tree::num_total_nodes<dim>(height) - 1;

  static_assert(
      height <= convert::nodeOffsetToDepth<dim, size_t>(
                    std::numeric_limits<NodeOffsetType>::max()),
      "Keys for nodes within chunks of the given height and dimensionality are "
      "not guaranteed to fit within the chosen KeyType. Make the chunks "
      "smaller or change the KeyType alias to a larger unsigned integer type.");

  ChunkedNdtreeChunk() = default;
  ~ChunkedNdtreeChunk() = default;

  // Delete copy constructor and assignment operator to avoid accidental copies
  ChunkedNdtreeChunk(const ChunkedNdtreeChunk& other_tree) = delete;
  ChunkedNdtreeChunk& operator=(const ChunkedNdtreeChunk&) = delete;

  // Allow move construction and assignments
  ChunkedNdtreeChunk(ChunkedNdtreeChunk&&) = default;
  ChunkedNdtreeChunk& operator=(ChunkedNdtreeChunk&&) = default;

  bool empty() const;
  void clear();

  size_t getMemoryUsage() const;

  // Methods to operate at the chunk level
  bool hasNonzeroData() const;
  bool hasNonzeroData(FloatingPoint threshold) const;

  bool hasChildrenArray() const { return static_cast<bool>(child_chunks_); }
  bool hasAtLeastOneChild() const;
  void deleteChildrenArray() { child_chunks_.reset(); }

  bool hasChild(LinearIndex relative_child_index) const;
  bool eraseChild(LinearIndex relative_child_index);

  ChunkedNdtreeChunk* getChild(LinearIndex relative_child_index);
  const ChunkedNdtreeChunk* getChild(LinearIndex relative_child_index) const;
  template <typename... DefaultArgs>
  ChunkedNdtreeChunk& getOrAllocateChild(LinearIndex relative_child_index,
                                         DefaultArgs&&... args);

  // Methods to operate on nodes given their relative position inside the chunk
  bool nodeHasNonzeroData(NodeOffsetType relative_node_index) const;
  bool nodeHasNonzeroData(NodeOffsetType relative_node_index,
                          FloatingPoint threshold) const;

  DataT& nodeData(NodeOffsetType relative_node_index);
  const DataT& nodeData(NodeOffsetType relative_node_index) const;

  bool nodeHasAtLeastOneChild(NodeOffsetType relative_node_index) const;
  bool nodeHasChild(NodeOffsetType relative_node_index,
                    NdtreeIndexRelativeChild child_index) const;
  void nodeSetHasChild(NodeOffsetType relative_node_index,
                       NdtreeIndexRelativeChild child_index);
  void nodeEraseChild(NodeOffsetType relative_node_index,
                      NdtreeIndexRelativeChild child_index);

  friend bool operator==(const ChunkedNdtreeChunk& lhs,
                         const ChunkedNdtreeChunk& rhs) {
    return &rhs == &lhs;
  }

 private:
  using ChildAllocationMaskType = uint8_t;
  using NodeDataArray = std::array<DataT, kNumInnerNodes>;
  using NodeChildBitset = std::array<ChildAllocationMaskType, kNumInnerNodes>;
  using ChunkPtr = std::unique_ptr<ChunkedNdtreeChunk>;
  using ChildChunkArray = std::array<ChunkPtr, kNumChildren>;

  NodeDataArray node_data_{};
  NodeChildBitset allocated_child_mask_{};
  std::unique_ptr<ChildChunkArray> child_chunks_;
};
}  // namespace wavemap

#include "wavemap/core/data_structure/chunked_ndtree/impl/chunked_ndtree_chunk_inl.h"

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_CHUNK_H_
