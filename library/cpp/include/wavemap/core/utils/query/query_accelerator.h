#ifndef WAVEMAP_CORE_UTILS_QUERY_QUERY_ACCELERATOR_H_
#define WAVEMAP_CORE_UTILS_QUERY_QUERY_ACCELERATOR_H_

#include <limits>

#include "wavemap/core/data_structure/dense_block_hash.h"
#include "wavemap/core/data_structure/ndtree_block_hash.h"
#include "wavemap/core/data_structure/spatial_hash.h"
#include "wavemap/core/indexing/index_hashes.h"
#include "wavemap/core/map/hashed_chunked_wavelet_octree.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"

namespace wavemap {
// Base template
template <typename DataStructureT>
class QueryAccelerator {};

// Template deduction guide
template <typename T>
QueryAccelerator(T& type) -> QueryAccelerator<T>;
template <typename T>
QueryAccelerator(const T& type) -> QueryAccelerator<T>;

// Query accelerator for vanilla spatial hashes
template <typename BlockDataT, int dim>
class QueryAccelerator<SpatialHash<BlockDataT, dim>> {
 public:
  static constexpr int kDim = dim;

  explicit QueryAccelerator(SpatialHash<BlockDataT, dim>& spatial_hash)
      : spatial_hash_(spatial_hash) {}

  //! Copy and move constructors
  QueryAccelerator(const QueryAccelerator& other) = default;
  QueryAccelerator(QueryAccelerator&& other) = default;

  void reset();

  BlockDataT* getBlock(const Index<dim>& block_index);
  template <typename... DefaultArgs>
  BlockDataT& getOrAllocateBlock(const Index<dim>& block_index,
                                 DefaultArgs&&... args);

 private:
  SpatialHash<BlockDataT, dim>& spatial_hash_;

  Index<dim> last_block_index_ =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  BlockDataT* last_block_ = nullptr;
};

// Query accelerator for dense block hashes
template <typename CellDataT, int dim, unsigned int cells_per_side>
class QueryAccelerator<DenseBlockHash<CellDataT, dim, cells_per_side>> {
 public:
  static constexpr int kDim = dim;
  using BlockType =
      typename DenseBlockHash<CellDataT, dim, cells_per_side>::Block;
  using CellType =
      typename DenseBlockHash<CellDataT, dim, cells_per_side>::Cell;

  explicit QueryAccelerator(
      const DenseBlockHash<CellDataT, dim, cells_per_side>& dense_block_hash)
      : dense_block_hash_(dense_block_hash) {}

  //! Copy and move constructors
  QueryAccelerator(const QueryAccelerator& other) = default;
  QueryAccelerator(QueryAccelerator&& other) = default;

  void reset();

  const BlockType* getBlock(const Index<dim>& block_index);
  const CellDataT* getValue(const Index<dim>& index);

 private:
  const DenseBlockHash<CellDataT, dim, cells_per_side>& dense_block_hash_;

  Index<dim> block_index_ =
      Index<dim>::Constant(std::numeric_limits<IndexElement>::max());
  const BlockType* block_ = nullptr;
};

// Query accelerator for ndtree block hashes
template <typename CellDataT, int dim>
class QueryAccelerator<NdtreeBlockHash<CellDataT, dim>> {
 public:
  static constexpr int kDim = dim;
  using BlockType = typename NdtreeBlockHash<CellDataT, dim>::Block;
  using NodeType = typename BlockType::NodeType;

  explicit QueryAccelerator(NdtreeBlockHash<CellDataT, dim>& ndtree_block_hash)
      : ndtree_block_hash_(ndtree_block_hash) {}

  //! Copy and move constructors
  QueryAccelerator(const QueryAccelerator& other) = default;
  QueryAccelerator(QueryAccelerator&& other) = default;

  void reset();

  BlockType* getBlock(const Index<dim>& block_index);
  template <typename... DefaultArgs>
  BlockType& getOrAllocateBlock(const Index<dim>& block_index,
                                DefaultArgs&&... args);

  NodeType* getNode(const OctreeIndex& index);
  template <typename... DefaultArgs>
  NodeType& getOrAllocateNode(const OctreeIndex& index, DefaultArgs&&... args);

 private:
  NdtreeBlockHash<CellDataT, dim>& ndtree_block_hash_;
  const IndexElement tree_height_ = ndtree_block_hash_.getMaxHeight();

  Index<dim> block_index_ =
      Index<dim>::Constant(std::numeric_limits<IndexElement>::max());
  IndexElement height = tree_height_;
  MortonIndex morton_code = std::numeric_limits<MortonIndex>::max();

  BlockType* block_ = nullptr;
  std::array<NodeType*, morton::kMaxTreeHeight<dim>> node_stack_{};
};

/**
 * A class that accelerates queries by caching block and parent node addresses
 * to speed up data structure traversals, and intermediate wavelet decompression
 * results to reduce redundant computation.
 * @note This class is safe to use in a multi-threaded environment. However,
 *       concurrent calls to a single instance from multiple threads are not.
 *       Since the accelerator is lightweight and cheap to construct, we
 *       recommend using a separate instance per thread for the best performance
 *       and simplicity.
 */
template <>
class QueryAccelerator<HashedWaveletOctree> {
 public:
  static constexpr int kDim = HashedWaveletOctree::kDim;

  explicit QueryAccelerator(const HashedWaveletOctree& map) : map_(map) {}

  //! Copy and move constructors
  QueryAccelerator(const QueryAccelerator& other) = default;
  QueryAccelerator(QueryAccelerator&& other) = default;

  //! Reset the cache
  //! @note This method must be called whenever the map changes, not only to
  //!       guarantee correct values (after node value changes) but also to
  //!       avoid segmentation faults after map topology changes (e.g. after
  //!       pruning).
  void reset();

  //! Query the value of the map at a given index
  FloatingPoint getCellValue(const Index3D& index) {
    return getCellValue(OctreeIndex{0, index});
  }

  //! Query the value of the map at a given octree node index
  FloatingPoint getCellValue(const OctreeIndex& index);

  //! Convenience function to get the map's minimum cell width
  FloatingPoint getMinCellWidth() const { return map_.getMinCellWidth(); }

 private:
  using BlockIndex = HashedWaveletOctree::BlockIndex;
  using NodePtrType = HashedWaveletOctree::Block::OctreeType::NodeConstPtrType;

  const HashedWaveletOctree& map_;
  const IndexElement tree_height_ = map_.getTreeHeight();

  std::array<NodePtrType, morton::kMaxTreeHeight<3>> node_stack_{};
  std::array<FloatingPoint, morton::kMaxTreeHeight<3>> value_stack_{};

  BlockIndex block_index_ =
      BlockIndex::Constant(std::numeric_limits<IndexElement>::max());
  MortonIndex morton_code_ = std::numeric_limits<MortonIndex>::max();
  IndexElement height_ = tree_height_;
};

/**
 * A class that accelerates queries by caching block and parent node addresses
 * to speed up data structure traversals, and intermediate wavelet decompression
 * results to reduce redundant computation.
 * @note This class is safe to use in a multi-threaded environment. However,
 *       concurrent calls to a single instance from multiple threads are not.
 *       Since the accelerator is lightweight and cheap to construct, we
 *       recommend using a separate instance per thread for the best performance
 *       and simplicity.
 */
template <>
class QueryAccelerator<HashedChunkedWaveletOctree> {
 public:
  static constexpr int kDim = HashedChunkedWaveletOctree::kDim;

  explicit QueryAccelerator(const HashedChunkedWaveletOctree& map)
      : map_(map) {}

  //! Copy and move constructors
  QueryAccelerator(const QueryAccelerator& other) = default;
  QueryAccelerator(QueryAccelerator&& other) = default;

  //! Reset the cache
  //! @note This method must be called whenever the map changes, not only to
  //!       guarantee correct values (after node value changes) but also to
  //!       avoid segmentation faults after map topology changes (e.g. after
  //!       pruning).
  void reset();

  //! Query the value of the map at a given index
  FloatingPoint getCellValue(const Index3D& index) {
    return getCellValue(OctreeIndex{0, index});
  }

  //! Query the value of the map at a given octree node index
  FloatingPoint getCellValue(const OctreeIndex& index);

  //! Convenience function to get the map's minimum cell width
  FloatingPoint getMinCellWidth() const { return map_.getMinCellWidth(); }

 private:
  using BlockIndex = HashedChunkedWaveletOctree::BlockIndex;
  using NodePtrType =
      HashedChunkedWaveletOctree::Block::OctreeType::NodeConstPtrType;

  const HashedChunkedWaveletOctree& map_;
  const IndexElement tree_height_ = map_.getTreeHeight();

  std::array<NodePtrType, morton::kMaxTreeHeight<3>> node_stack_{};
  std::array<FloatingPoint, morton::kMaxTreeHeight<3>> value_stack_{};

  BlockIndex block_index_ =
      BlockIndex::Constant(std::numeric_limits<IndexElement>::max());
  MortonIndex morton_code_ = std::numeric_limits<MortonIndex>::max();
  IndexElement height_ = tree_height_;
};
}  // namespace wavemap

#include "wavemap/core/utils/query/impl/query_accelerator_inl.h"

#endif  // WAVEMAP_CORE_UTILS_QUERY_QUERY_ACCELERATOR_H_
