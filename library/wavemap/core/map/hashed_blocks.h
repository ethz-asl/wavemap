#ifndef WAVEMAP_MAP_HASHED_BLOCKS_H_
#define WAVEMAP_MAP_HASHED_BLOCKS_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "wavemap/core/common.h"
#include "wavemap/core/config/config_base.h"
#include "wavemap/core/data_structure/dense_block_hash.h"
#include "wavemap/core/map/map_base.h"

namespace wavemap {
class HashedBlocks : public MapBase,
                     public DenseBlockHash<FloatingPoint, MapBase::kDim, 16> {
 public:
  using Ptr = std::shared_ptr<HashedBlocks>;
  using ConstPtr = std::shared_ptr<const HashedBlocks>;
  using Config = MapBaseConfig;
  static constexpr bool kRequiresExplicitThresholding = false;

  using MapBase::kDim;

  explicit HashedBlocks(const MapBaseConfig& config,
                        FloatingPoint default_value = 0.f)
      : MapBase(config.checkValid()), DenseBlockHash(default_value) {}

  bool empty() const override { return DenseBlockHash::empty(); }
  size_t size() const override { return DenseBlockHash::size(); }
  void threshold() override {
    // Not needed. Data is already tresholded when added.
  }
  void prune() override;
  void clear() override { DenseBlockHash::clear(); }

  size_t getMemoryUsage() const override {
    return size() * sizeof(FloatingPoint);
  }

  Index3D getMinIndex() const override;
  Index3D getMaxIndex() const override;
  Index3D getMinBlockIndex() const { return block_map_.getMinBlockIndex(); }
  Index3D getMaxBlockIndex() const { return block_map_.getMaxBlockIndex(); }
  IndexElement getTreeHeight() const override { return 0; }
  const MapBaseConfig& getConfig() { return config_; }

  FloatingPoint getCellValue(const Index3D& index) const override;
  void setCellValue(const Index3D& index, FloatingPoint new_value) override;
  void addToCellValue(const Index3D& index, FloatingPoint update) override;

  void forEachLeaf(
      typename MapBase::IndexedLeafVisitorFunction visitor_fn) const override;
};
}  // namespace wavemap

#include "wavemap/core/map/impl/hashed_blocks_inl.h"

#endif  // WAVEMAP_MAP_HASHED_BLOCKS_H_
