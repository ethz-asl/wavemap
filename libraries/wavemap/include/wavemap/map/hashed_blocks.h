#ifndef WAVEMAP_MAP_HASHED_BLOCKS_H_
#define WAVEMAP_MAP_HASHED_BLOCKS_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "wavemap/common.h"
#include "wavemap/config/config_base.h"
#include "wavemap/data_structure/dense_grid.h"
#include "wavemap/data_structure/spatial_hash.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/map/volumetric_data_structure_base.h"
#include "wavemap/utils/math/int_math.h"

namespace wavemap {
class HashedBlocks : public VolumetricDataStructureBase {
 private:
  static constexpr IndexElement kCellsPerSideLog2 = 4;
  static constexpr IndexElement kCellsPerSide =
      int_math::exp2(kCellsPerSideLog2);

 public:
  using Ptr = std::shared_ptr<HashedBlocks>;
  using ConstPtr = std::shared_ptr<const HashedBlocks>;
  using Config = VolumetricDataStructureConfig;
  static constexpr bool kRequiresExplicitThresholding = false;

  using BlockIndex = Index3D;
  using CellIndex = Index3D;
  using Block = DenseGrid<FloatingPoint, kDim, kCellsPerSide>;
  using BlockHashMap = SpatialHash<Block, kDim>;

  explicit HashedBlocks(const VolumetricDataStructureConfig& config,
                        FloatingPoint default_value = 0.f)
      : VolumetricDataStructureBase(config.checkValid()),
        default_value_(default_value) {}

  bool empty() const override { return block_map_.empty(); }
  size_t size() const override {
    return Block::kCellsPerBlock * block_map_.size();
  }
  void threshold() override {
    // Not needed. Data is already tresholded when added.
  }
  void prune() override;
  void clear() override { block_map_.clear(); }

  size_t getMemoryUsage() const override {
    return size() * sizeof(FloatingPoint);
  }

  Index3D getMinIndex() const override;
  Index3D getMaxIndex() const override;
  Index3D getMinBlockIndex() const { return block_map_.getMinBlockIndex(); }
  Index3D getMaxBlockIndex() const { return block_map_.getMaxBlockIndex(); }
  IndexElement getTreeHeight() const override { return 0; }
  const VolumetricDataStructureConfig& getConfig() { return config_; }

  FloatingPoint getCellValue(const Index3D& index) const override;
  FloatingPoint& getCellValueRef(const Index3D& index);
  FloatingPoint getDefaultCellValue() const { return default_value_; }
  void setCellValue(const Index3D& index, FloatingPoint new_value) override;
  void addToCellValue(const Index3D& index, FloatingPoint update) override;

  bool hasBlock(const Index3D& block_index) const;
  Block* getBlock(const Index3D& block_index);
  const Block* getBlock(const Index3D& block_index) const;
  Block& getOrAllocateBlock(const Index3D& block_index);

  auto& getHashMap() { return block_map_; }
  const auto& getHashMap() const { return block_map_; }

  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn) {
    block_map_.forEachBlock(visitor_fn);
  }
  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn) const {
    block_map_.forEachBlock(visitor_fn);
  }

  void forEachLeaf(
      typename VolumetricDataStructureBase::IndexedLeafVisitorFunction
          visitor_fn) const override;

 private:
  const FloatingPoint default_value_;
  BlockHashMap block_map_;

  static Index3D indexToBlockIndex(const Index3D& index);
  static Index3D indexToCellIndex(const Index3D& index);
  static Index3D cellAndBlockIndexToIndex(const Index3D& block_index,
                                          const Index3D& cell_index);
};
}  // namespace wavemap

#include "wavemap/map/impl/hashed_blocks_inl.h"

#endif  // WAVEMAP_MAP_HASHED_BLOCKS_H_
