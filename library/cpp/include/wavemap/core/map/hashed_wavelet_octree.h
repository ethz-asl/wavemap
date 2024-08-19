#ifndef WAVEMAP_CORE_MAP_HASHED_WAVELET_OCTREE_H_
#define WAVEMAP_CORE_MAP_HASHED_WAVELET_OCTREE_H_

#include <memory>
#include <unordered_map>

#include "wavemap/core/common.h"
#include "wavemap/core/config/config_base.h"
#include "wavemap/core/data_structure/spatial_hash.h"
#include "wavemap/core/indexing/index_hashes.h"
#include "wavemap/core/map/hashed_wavelet_octree_block.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/math/int_math.h"

namespace wavemap {
/**
 * Config struct for the hashed wavelet octree volumetric data structure.
 */
struct HashedWaveletOctreeConfig : ConfigBase<HashedWaveletOctreeConfig, 5> {
  //! Maximum resolution of the map, set as the width of the smallest cell that
  //! it can represent.
  Meters<FloatingPoint> min_cell_width = 0.1f;

  //! Lower threshold for the occupancy values stored in the map, in log-odds.
  FloatingPoint min_log_odds = -2.f;
  //! Upper threshold for the occupancy values stored in the map, in log-odds.
  FloatingPoint max_log_odds = 4.f;

  //! Height of the octree in each hashed block.
  IndexElement tree_height = 6;
  //! Only prune blocks if they have not been updated for at least this amount
  //! of time. Useful to avoid pruning blocks that are still being updated,
  //! whose nodes would most likely directly be reallocated if pruned.
  Seconds<FloatingPoint> only_prune_blocks_if_unused_for = 5.f;

  static MemberMap memberMap;

  // Constructors
  HashedWaveletOctreeConfig() = default;
  HashedWaveletOctreeConfig(FloatingPoint min_cell_width,
                            FloatingPoint min_log_odds,
                            FloatingPoint max_log_odds,
                            IndexElement tree_height,
                            FloatingPoint only_prune_blocks_if_unused_for)
      : min_cell_width(min_cell_width),
        min_log_odds(min_log_odds),
        max_log_odds(max_log_odds),
        tree_height(tree_height),
        only_prune_blocks_if_unused_for(only_prune_blocks_if_unused_for) {}

  // Conversion to DataStructureBase config
  operator MapBaseConfig() const {  // NOLINT
    return {min_cell_width, min_log_odds, max_log_odds};
  }

  bool isValid(bool verbose) const override;
};

class HashedWaveletOctree : public MapBase {
 public:
  using Ptr = std::shared_ptr<HashedWaveletOctree>;
  using ConstPtr = std::shared_ptr<const HashedWaveletOctree>;
  using Config = HashedWaveletOctreeConfig;
  static constexpr bool kRequiresExplicitThresholding = true;

  using BlockIndex = Index3D;
  using CellIndex = OctreeIndex;
  using Block = HashedWaveletOctreeBlock;
  using BlockHashMap = SpatialHash<Block, kDim>;

  explicit HashedWaveletOctree(const HashedWaveletOctreeConfig& config)
      : MapBase(config), config_(config.checkValid()) {}

  bool empty() const override { return block_map_.empty(); }
  size_t size() const override;
  void threshold() override;
  void prune() override;
  void pruneSmart() override;
  void clear() override { block_map_.clear(); }

  size_t getMemoryUsage() const override;

  Index3D getMinIndex() const override;
  Index3D getMaxIndex() const override;
  Index3D getMinBlockIndex() const { return block_map_.getMinBlockIndex(); }
  Index3D getMaxBlockIndex() const { return block_map_.getMaxBlockIndex(); }
  IndexElement getTreeHeight() const override { return config_.tree_height; }
  Index3D getBlockSize() const {
    return Index3D::Constant(cells_per_block_side_);
  }
  const HashedWaveletOctreeConfig& getConfig() { return config_; }

  FloatingPoint getCellValue(const Index3D& index) const override;
  FloatingPoint getCellValue(const OctreeIndex& index) const;
  void setCellValue(const Index3D& index, FloatingPoint new_value) override;
  void addToCellValue(const Index3D& index, FloatingPoint update) override;

  bool hasBlock(const Index3D& block_index) const;
  bool eraseBlock(const BlockIndex& block_index);
  template <typename IndexedBlockVisitor>
  void eraseBlockIf(IndexedBlockVisitor indicator_fn);

  Block* getBlock(const Index3D& block_index);
  const Block* getBlock(const Index3D& block_index) const;
  Block& getOrAllocateBlock(const Index3D& block_index);

  auto& getHashMap() { return block_map_.getHashMap(); }
  const auto& getHashMap() const { return block_map_.getHashMap(); }

  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn);
  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn) const;

  void forEachLeaf(
      typename MapBase::IndexedLeafVisitorFunction visitor_fn) const override;

  BlockIndex indexToBlockIndex(const OctreeIndex& node_index) const;
  CellIndex indexToCellIndex(OctreeIndex index) const;

 private:
  const HashedWaveletOctreeConfig config_;
  const IndexElement cells_per_block_side_ =
      int_math::exp2(config_.tree_height);

  BlockHashMap block_map_;
};
}  // namespace wavemap

#include "wavemap/core/map/impl/hashed_wavelet_octree_inl.h"

#endif  // WAVEMAP_CORE_MAP_HASHED_WAVELET_OCTREE_H_
