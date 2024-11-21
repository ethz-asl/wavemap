#ifndef WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_ADDRESS_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_ADDRESS_H_

#include <optional>

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/chunked_ndtree/chunked_ndtree_chunk.h"
#include "wavemap/core/utils/data/comparisons.h"

namespace wavemap {
template <typename ChunkType>
class ChunkedNdtreeNodeRef;

template <typename ChunkType>
class ChunkedNdtreeNodePtr {
 public:
  using NodeRef = ChunkedNdtreeNodeRef<ChunkType>;

  // Constructors
  ChunkedNdtreeNodePtr() = default;
  ChunkedNdtreeNodePtr(ChunkType* chunk, IndexElement relative_node_depth,
                       LinearIndex level_traversal_distance);

  // Copy/move constructors
  ChunkedNdtreeNodePtr(const ChunkedNdtreeNodePtr& other);
  ChunkedNdtreeNodePtr(ChunkedNdtreeNodePtr&& other) noexcept;

  // Copy/move assignment operators
  ChunkedNdtreeNodePtr& operator=(const ChunkedNdtreeNodePtr& other);
  ChunkedNdtreeNodePtr& operator=(ChunkedNdtreeNodePtr&& other) noexcept;

  // Emulate pointer semantics
  void reset() { node_.reset(); }
  operator bool() const { return node_.has_value(); }  // NOLINT
  NodeRef operator*() const { return node_.value(); }
  NodeRef* operator->() { return node_.operator->(); }
  const NodeRef* operator->() const { return node_.operator->(); }

 private:
  std::optional<NodeRef> node_;
};

template <typename ChunkType>
class ChunkedNdtreeNodeRef {
 public:
  using NodeRef = ChunkedNdtreeNodeRef<ChunkType>;
  using NodePtr = ChunkedNdtreeNodePtr<ChunkType>;

  static constexpr int kDim = ChunkType::kDim;
  static constexpr int kNumChildren = NdtreeIndex<kDim>::kNumChildren;
  using NodeDataType = typename ChunkType::DataType;

  ChunkedNdtreeNodeRef() = delete;
  ChunkedNdtreeNodeRef(ChunkType& chunk, IndexElement relative_node_depth,
                       LinearIndex level_traversal_distance);

  // Copy/move constructor
  ChunkedNdtreeNodeRef(const ChunkedNdtreeNodeRef& other);
  ChunkedNdtreeNodeRef(ChunkedNdtreeNodeRef&& other) noexcept;

  // Copy/move assignment operators are deleted (to behave like a ref)
  ChunkedNdtreeNodeRef& operator=(const ChunkedNdtreeNodeRef&) = delete;
  ChunkedNdtreeNodeRef& operator=(ChunkedNdtreeNodeRef&&) = delete;

  // Conversion of non-const ref to const ref
  operator ChunkedNdtreeNodeRef<const ChunkType>() const;  // NOLINT

  // Conversion to pointer
  NodePtr operator&() const;  // NOLINT

  // Regular node methods
  bool empty() const;

  bool hasNonzeroData() const;
  bool hasNonzeroData(FloatingPoint threshold) const;
  auto& data() const;

  auto hasAtLeastOneChild() const;  // Returns a BitRef or bool, depending on
                                    // whether the ChunkType is const-qualified

  bool hasChild(NdtreeIndexRelativeChild child_index) const;
  // TODO(victorr): Add tests for this method
  void eraseChild(NdtreeIndexRelativeChild child_index) const;

  NodePtr getChild(NdtreeIndexRelativeChild child_index) const;
  template <typename... DefaultArgs>
  NodeRef getOrAllocateChild(NdtreeIndexRelativeChild child_index,
                             DefaultArgs&&... args) const;

 private:
  ChunkType& chunk_;
  // TODO(victorr): Benchmark with int8_t
  const IndexElement relative_node_depth_ = 0;
  // TODO(victorr): Benchmark with uint32_t
  const LinearIndex level_traversal_distance_ = 0u;

  LinearIndex computeRelativeNodeIndex() const;
  LinearIndex computeChildLevelTraversalDistance(
      NdtreeIndexRelativeChild child_index) const;

  template <typename T>
  friend class ChunkedNdtreeNodePtr;
};
}  // namespace wavemap

#include "wavemap/core/data_structure/chunked_ndtree/impl/chunked_ndtree_node_address_inl.h"

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_ADDRESS_H_
