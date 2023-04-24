#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_CHUNKED_WAVELET_OCTREE_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_CHUNKED_WAVELET_OCTREE_H_

#include <memory>
#include <unordered_map>

#include "wavemap/common.h"
#include "wavemap/config/config_base.h"
#include "wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree_block.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/utils/int_math.h"

namespace wavemap {
struct HashedChunkedWaveletOctreeConfig
    : ConfigBase<HashedChunkedWaveletOctreeConfig, 5> {
  static constexpr IndexElement kMaxSupportedTreeHeight =
      HashedChunkedWaveletOctreeBlock::kMaxSupportedTreeHeight;

  FloatingPoint min_cell_width = 0.1f;

  FloatingPoint min_log_odds = -2.f;
  FloatingPoint max_log_odds = 4.f;

  IndexElement tree_height = 6;
  FloatingPoint only_prune_blocks_if_unused_for = 5.f;

  static MemberMap memberMap;

  // Constructors
  HashedChunkedWaveletOctreeConfig() = default;
  HashedChunkedWaveletOctreeConfig(
      FloatingPoint min_cell_width, FloatingPoint min_log_odds,
      FloatingPoint max_log_odds, IndexElement tree_height,
      FloatingPoint only_prune_blocks_if_unused_for)
      : min_cell_width(min_cell_width),
        min_log_odds(min_log_odds),
        max_log_odds(max_log_odds),
        tree_height(tree_height),
        only_prune_blocks_if_unused_for(only_prune_blocks_if_unused_for) {}

  // Conversion to DataStructureBase config
  operator VolumetricDataStructureConfig() const {  // NOLINT
    return {min_cell_width, min_log_odds, max_log_odds};
  }

  bool isValid(bool verbose) const override;
};

class HashedChunkedWaveletOctree : public VolumetricDataStructureBase {
 public:
  using Ptr = std::shared_ptr<HashedChunkedWaveletOctree>;
  using ConstPtr = std::shared_ptr<const HashedChunkedWaveletOctree>;
  using Config = HashedChunkedWaveletOctreeConfig;
  static constexpr bool kRequiresExplicitThresholding = true;

  using Block = HashedChunkedWaveletOctreeBlock;
  using BlockIndex = Block::BlockIndex;
  using CellIndex = OctreeIndex;

  explicit HashedChunkedWaveletOctree(
      const HashedChunkedWaveletOctreeConfig& config)
      : VolumetricDataStructureBase(config), config_(config.checkValid()) {}

  bool empty() const override { return blocks_.empty(); }
  size_t size() const override;
  void threshold() override;
  void prune() override;
  void pruneDistant() override;
  void clear() override { blocks_.clear(); }

  size_t getMemoryUsage() const override;

  Index3D getMinIndex() const override;
  Index3D getMaxIndex() const override;
  IndexElement getTreeHeight() const { return config_.tree_height; }
  IndexElement getChunkHeight() const { return Block::kChunkHeight; }
  Index3D getBlockSize() const {
    return Index3D::Constant(cells_per_block_side_);
  }

  FloatingPoint getCellValue(const Index3D& index) const override;
  FloatingPoint getCellValue(const OctreeIndex& index) const;
  void setCellValue(const Index3D& index, FloatingPoint new_value) override;
  void addToCellValue(const Index3D& index, FloatingPoint update) override;

  bool hasBlock(const Index3D& block_index) const;
  Block& getOrAllocateBlock(const Index3D& block_index);
  Block& getBlock(const Index3D& block_index);
  const Block& getBlock(const Index3D& block_index) const;
  auto& getBlocks() { return blocks_; }
  const auto& getBlocks() const { return blocks_; }

  void forEachLeaf(
      typename VolumetricDataStructureBase::IndexedLeafVisitorFunction
          visitor_fn) const override;

 private:
  const HashedChunkedWaveletOctreeConfig config_;

  const IndexElement cells_per_block_side_ =
      int_math::exp2(config_.tree_height);

  std::unordered_map<BlockIndex, Block, IndexHash<kDim>> blocks_;

  BlockIndex computeBlockIndexFromIndex(const Index3D& index) const {
    return int_math::div_exp2_floor(index, config_.tree_height);
  }
  BlockIndex computeBlockIndexFromIndex(const OctreeIndex& node_index) const {
    // TODO(victorr): Divide by height difference directly instead of round trip
    //                through height 0
    const Index3D index = convert::nodeIndexToMinCornerIndex(node_index);
    return int_math::div_exp2_floor(index, config_.tree_height);
  }
  CellIndex computeCellIndexFromBlockIndexAndIndex(
      const BlockIndex& block_index, OctreeIndex index) const;
};
}  // namespace wavemap

#include "wavemap/data_structure/volumetric/impl/hashed_chunked_wavelet_octree_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_CHUNKED_WAVELET_OCTREE_H_
