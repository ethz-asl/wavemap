#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_CHUNK_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_CHUNK_H_

#include <array>
#include <bitset>
#include <limits>
#include <memory>

#include "wavemap/common.h"
#include "wavemap/indexing/ndtree_index.h"
#include "wavemap/utils/math/tree_math.h"

namespace wavemap {
template <typename DataT, int dim, int height>
class ChunkedNdtreeChunk {
 public:
  static constexpr int kDim = dim;
  static constexpr int kHeight = height;
  static constexpr int kNumInnerNodes =
      tree_math::perfect_tree::num_total_nodes<dim>(height);
  static constexpr int kNumChildren =
      tree_math::perfect_tree::num_leaf_nodes<dim>(height + 1);

  using DataType = DataT;
  using BitRef = typename std::bitset<kNumInnerNodes>::reference;

  ChunkedNdtreeChunk() = default;
  ~ChunkedNdtreeChunk() = default;

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

  // Methods to operate on individual nodes inside the chunk
  bool nodeHasNonzeroData(LinearIndex relative_node_index) const;
  bool nodeHasNonzeroData(LinearIndex relative_node_index,
                          FloatingPoint threshold) const;

  DataT& nodeData(LinearIndex relative_node_index);
  const DataT& nodeData(LinearIndex relative_node_index) const;

  BitRef nodeHasAtLeastOneChild(LinearIndex relative_node_index);
  bool nodeHasAtLeastOneChild(LinearIndex relative_node_index) const;

  friend bool operator==(const ChunkedNdtreeChunk& lhs,
                         const ChunkedNdtreeChunk& rhs) {
    return &rhs == &lhs;
  }

 private:
  using NodeDataArray = std::array<DataT, kNumInnerNodes>;
  using NodeChildBitset = std::bitset<kNumInnerNodes>;
  using ChunkPtr = std::unique_ptr<ChunkedNdtreeChunk>;
  using ChildChunkArray = std::array<ChunkPtr, kNumChildren>;

  NodeDataArray node_data_{};
  NodeChildBitset node_has_at_least_one_child_{};
  std::unique_ptr<ChildChunkArray> child_chunks_;
};
}  // namespace wavemap

#include "wavemap/data_structure/chunked_ndtree/impl/chunked_ndtree_chunk_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_CHUNK_H_
