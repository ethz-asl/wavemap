#ifndef WAVEMAP_MAP_HASHED_BLOCKS_H_
#define WAVEMAP_MAP_HASHED_BLOCKS_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "wavemap/common.h"
#include "wavemap/config/config_base.h"
#include "wavemap/data_structure/dense_block_hash.h"
#include "wavemap/map/volumetric_data_structure_base.h"

namespace wavemap {
class HashedBlocks
    : public VolumetricDataStructureBase,
      public DenseBlockHash<FloatingPoint, VolumetricDataStructureBase::kDim,
                            16> {
 public:
  using Ptr = std::shared_ptr<HashedBlocks>;
  using ConstPtr = std::shared_ptr<const HashedBlocks>;
  using Config = VolumetricDataStructureConfig;
  static constexpr bool kRequiresExplicitThresholding = false;

  using VolumetricDataStructureBase::kDim;

  explicit HashedBlocks(const VolumetricDataStructureConfig& config,
                        FloatingPoint default_value = 0.f)
      : VolumetricDataStructureBase(config.checkValid()),
        DenseBlockHash(config.min_cell_width, default_value) {}

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
  const VolumetricDataStructureConfig& getConfig() { return config_; }

  FloatingPoint getCellValue(const Index3D& index) const override;
  void setCellValue(const Index3D& index, FloatingPoint new_value) override;
  void addToCellValue(const Index3D& index, FloatingPoint update) override;

  auto& getHashMap() { return block_map_; }
  const auto& getHashMap() const { return block_map_; }

  void forEachLeaf(
      typename VolumetricDataStructureBase::IndexedLeafVisitorFunction
          visitor_fn) const override;
};
}  // namespace wavemap

#include "wavemap/map/impl/hashed_blocks_inl.h"

#endif  // WAVEMAP_MAP_HASHED_BLOCKS_H_
