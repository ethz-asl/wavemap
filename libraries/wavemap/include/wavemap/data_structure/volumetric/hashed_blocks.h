#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_BLOCKS_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_BLOCKS_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "wavemap/common.h"
#include "wavemap/config/config_base.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/utils/int_math.h"

namespace wavemap {
class HashedBlocks : public VolumetricDataStructureBase {
 public:
  using Ptr = std::shared_ptr<HashedBlocks>;
  using ConstPtr = std::shared_ptr<const HashedBlocks>;
  using Config = VolumetricDataStructureConfig;
  static constexpr bool kRequiresExplicitThresholding = false;

  static constexpr IndexElement kCellsPerSideLog2 = 4;
  static constexpr IndexElement kCellsPerSide =
      int_math::exp2(kCellsPerSideLog2);
  static constexpr IndexElement kCellsPerBlock =
      int_math::exp2(kDim * kCellsPerSideLog2);

  using Block = std::array<FloatingPoint, kCellsPerBlock>;
  using BlockIndex = Index3D;
  using CellIndex = Index3D;

  explicit HashedBlocks(const VolumetricDataStructureConfig& config,
                        FloatingPoint default_value = 0.f)
      : VolumetricDataStructureBase(config.checkValid()),
        default_value_(default_value) {}

  bool empty() const override { return blocks_.empty(); }
  size_t size() const override { return kCellsPerBlock * blocks_.size(); }
  void threshold() override {
    // Not needed. Data is already tresholded when added.
  }
  void prune() override;
  void clear() override { blocks_.clear(); }

  size_t getMemoryUsage() const override {
    return size() * sizeof(FloatingPoint);
  }

  Index3D getMinIndex() const override;
  Index3D getMaxIndex() const override;
  IndexElement getTreeHeight() const override { return 0; }
  const VolumetricDataStructureConfig& getConfig() { return config_; }

  FloatingPoint getCellValue(const Index3D& index) const override;
  void setCellValue(const Index3D& index, FloatingPoint new_value) override;
  void addToCellValue(const Index3D& index, FloatingPoint update) override;

  bool hasBlock(const Index3D& block_index) const {
    return blocks_.count(block_index);
  }
  Block& getOrAllocateBlock(const Index3D& block_index);
  Block& getBlock(const Index3D& block_index) {
    return blocks_.at(block_index);
  }
  const Block& getBlock(const Index3D& block_index) const {
    return blocks_.at(block_index);
  }
  auto& getBlocks() { return blocks_; }
  const auto& getBlocks() const { return blocks_; }

  FloatingPoint* accessCellData(const Index3D& index,
                                bool auto_allocate = false);
  const FloatingPoint* accessCellData(const Index3D& index) const;

  void forEachLeaf(
      typename VolumetricDataStructureBase::IndexedLeafVisitorFunction
          visitor_fn) const override;

 private:
  const FloatingPoint default_value_;

  std::unordered_map<BlockIndex, Block, IndexHash<3>> blocks_;

  static BlockIndex computeBlockIndexFromIndex(const Index3D& index);
  static CellIndex computeCellIndexFromBlockIndexAndIndex(
      const BlockIndex& block_index, const Index3D& index);
  static Index3D computeIndexFromBlockIndexAndCellIndex(
      const BlockIndex& block_index, const CellIndex& cell_index);
};
}  // namespace wavemap

#include "wavemap/data_structure/volumetric/impl/hashed_blocks_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_BLOCKS_H_
