#ifndef WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_ADDRESS_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_ADDRESS_H_

#include <limits>
#include <optional>

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/chunked_ndtree/chunked_ndtree_chunk.h"
#include "wavemap/core/indexing/index_conversions.h"

namespace wavemap {
template <typename ChunkT>
class ChunkedNdtreeNodeRef;

template <typename ChunkT>
class ChunkedNdtreeNodePtr {
 public:
  using NodeRef = ChunkedNdtreeNodeRef<ChunkT>;
  using OffsetType = typename NodeRef::OffsetType;

  // Constructors
  ChunkedNdtreeNodePtr() = default;
  ChunkedNdtreeNodePtr(ChunkT* chunk);  // NOLINT
  ChunkedNdtreeNodePtr(ChunkT* chunk, OffsetType offset);

  // Copy/move constructors
  ChunkedNdtreeNodePtr(const ChunkedNdtreeNodePtr& other);
  ChunkedNdtreeNodePtr(ChunkedNdtreeNodePtr&& other) noexcept;

  // Copy/move assignment operators
  ChunkedNdtreeNodePtr& operator=(const ChunkedNdtreeNodePtr& other);
  ChunkedNdtreeNodePtr& operator=(ChunkedNdtreeNodePtr&& other) noexcept;

  // Emulate pointer semantics
  void reset() { node_.reset(); }
  NodeRef operator*() const { return node_.value(); }
  NodeRef* operator->() { return node_.operator->(); }
  const NodeRef* operator->() const { return node_.operator->(); }

  // Emulate null check semantics
  operator bool() const { return node_.has_value(); }  // NOLINT
  bool operator==(std::nullptr_t) noexcept { return !node_.has_value(); }

 private:
  std::optional<NodeRef> node_{};
};

template <typename ChunkT>
class ChunkedNdtreeNodeRef {
 public:
  using NodeRef = ChunkedNdtreeNodeRef;
  using NodePtr = ChunkedNdtreeNodePtr<ChunkT>;
  static constexpr IndexElement kDim = ChunkT::kDim;

  using OffsetType = uint32_t;
  static constexpr IndexElement kMaxHeight =
      convert::nodeOffsetToDepth<kDim, size_t>(
          std::numeric_limits<OffsetType>::max());
  static_assert(
      ChunkT::kHeight <= kMaxHeight,
      "Keys for nodes within chunks of the given height and dimensionality are "
      "not guaranteed to fit within the chosen KeyType. Make the chunks "
      "smaller or change the KeyType alias to a larger unsigned integer type.");
  static constexpr OffsetType kRootOffset = 0u;

  ChunkedNdtreeNodeRef() = delete;
  ChunkedNdtreeNodeRef(ChunkT& chunk,  // NOLINT
                       OffsetType offset = kRootOffset)
      : chunk_(chunk), offset_(offset) {}

  // Copy/move constructor
  ChunkedNdtreeNodeRef(const ChunkedNdtreeNodeRef& other);
  ChunkedNdtreeNodeRef(ChunkedNdtreeNodeRef&& other) noexcept;

  // Copy/move assignment operators are deleted (to behave like a ref)
  ChunkedNdtreeNodeRef& operator=(const ChunkedNdtreeNodeRef&) = delete;
  ChunkedNdtreeNodeRef& operator=(ChunkedNdtreeNodeRef&&) = delete;

  // Conversion of non-const ref to const ref
  operator ChunkedNdtreeNodeRef<const ChunkT>() const;  // NOLINT

  // Conversion to pointer
  NodePtr operator&() const;  // NOLINT

  // Regular node methods
  bool empty() const;

  bool hasNonzeroData() const;
  bool hasNonzeroData(FloatingPoint threshold) const;
  auto& data() const;

  auto hasAtLeastOneChild() const;  // Returns a BitRef or bool, depending on
                                    // whether the chunk type is const-qualified

  bool hasChild(NdtreeIndexRelativeChild child_index) const;
  // TODO(victorr): Add tests for this method
  void eraseChild(NdtreeIndexRelativeChild child_index) const;

  NodePtr getChild(NdtreeIndexRelativeChild child_index) const;
  template <typename... DefaultArgs>
  NodeRef getOrAllocateChild(NdtreeIndexRelativeChild child_index,
                             DefaultArgs&&... args) const;

 private:
  ChunkT& chunk_;
  const OffsetType offset_;
  static constexpr OffsetType kMaxOffsetInChunk =
      tree_math::perfect_tree::num_total_nodes<kDim>(ChunkT::kHeight) - 1;

  IndexElement computeDepthIndex() const;
  OffsetType computeLevelIndex() const;
  OffsetType computeChildOffset(NdtreeIndexRelativeChild child_index) const;

  template <typename T>
  friend class ChunkedNdtreeNodePtr;
};
}  // namespace wavemap

#include "wavemap/core/data_structure/chunked_ndtree/impl/chunked_ndtree_node_address_inl.h"

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_ADDRESS_H_
