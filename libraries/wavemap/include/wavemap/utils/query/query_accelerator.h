#ifndef WAVEMAP_UTILS_QUERY_QUERY_ACCELERATOR_H_
#define WAVEMAP_UTILS_QUERY_QUERY_ACCELERATOR_H_

#include <limits>

#include "wavemap/data_structure/ndtree_block_hash.h"
#include "wavemap/data_structure/spatial_hash.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/map/hashed_wavelet_octree.h"

namespace wavemap {
// Base template
template <typename DataStructureT>
class QueryAccelerator {};

// Template deduction guide
template <typename T>
QueryAccelerator(T type) -> QueryAccelerator<T>;

// Query accelerator for vanilla spatial hashes
template <typename BlockDataT, int dim>
class QueryAccelerator<SpatialHash<BlockDataT, dim>> {
 public:
  static constexpr int kDim = dim;

  explicit QueryAccelerator(SpatialHash<BlockDataT, dim>& spatial_hash)
      : spatial_hash_(spatial_hash) {}

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

// Query accelerator for ndtree block hashes
template <typename CellDataT, int dim>
class QueryAccelerator<NdtreeBlockHash<CellDataT, dim>> {
 public:
  static constexpr int kDim = dim;
  using BlockType = typename NdtreeBlockHash<CellDataT, dim>::Block;
  using NodeType = typename BlockType::NodeType;

  explicit QueryAccelerator(NdtreeBlockHash<CellDataT, dim>& ndtree_block_hash)
      : ndtree_block_hash_(ndtree_block_hash) {}

  void reset();

  BlockType* getBlock(const Index<dim>& block_index);
  template <typename... DefaultArgs>
  BlockType& getOrAllocateBlock(const Index<dim>& block_index,
                                DefaultArgs&&... args);

  const NodeType* getNode(const OctreeIndex& index);

 private:
  NdtreeBlockHash<CellDataT, dim>& ndtree_block_hash_;
  const IndexElement tree_height_ = ndtree_block_hash_.getMaxHeight();

  Index<dim> block_index_ =
      Index<dim>::Constant(std::numeric_limits<IndexElement>::max());
  BlockType* block_ = nullptr;
};

// Query accelerator for hashed wavelet octrees
template <>
class QueryAccelerator<HashedWaveletOctree> {
 public:
  static constexpr int kDim = HashedWaveletOctree::kDim;

  explicit QueryAccelerator(const HashedWaveletOctree& map) : map_(map) {}

  void reset();

  FloatingPoint getCellValue(const Index3D& index) {
    return getCellValue(OctreeIndex{0, index});
  }

  FloatingPoint getCellValue(const OctreeIndex& index);

 private:
  using BlockIndex = HashedWaveletOctree::BlockIndex;
  using NodeType = HashedWaveletOctree::Block::NodeType;

  const HashedWaveletOctree& map_;
  const IndexElement tree_height_ = map_.getTreeHeight();

  std::array<const NodeType*, morton::kMaxTreeHeight<3>> node_stack_{};
  std::array<FloatingPoint, morton::kMaxTreeHeight<3>> value_stack_{};

  BlockIndex block_index_ =
      BlockIndex ::Constant(std::numeric_limits<IndexElement>::max());
  MortonIndex morton_code_ = std::numeric_limits<MortonIndex>::max();
  IndexElement height_ = tree_height_;
};
}  // namespace wavemap

#include "wavemap/utils/query/impl/query_accelerator_inl.h"

#endif  // WAVEMAP_UTILS_QUERY_QUERY_ACCELERATOR_H_
