#ifndef WAVEMAP_2D_DATASTRUCTURE_HASHED_BLOCKS_HASHED_BLOCKS_H_
#define WAVEMAP_2D_DATASTRUCTURE_HASHED_BLOCKS_HASHED_BLOCKS_H_

#include <string>
#include <unordered_map>

#include "wavemap_2d/datastructure/datastructure_base.h"

namespace wavemap_2d {
template <typename CellT>
class HashedBlocks : public DataStructureBase {
 public:
  using CellType = CellT;

  explicit HashedBlocks(const FloatingPoint resolution)
      : DataStructureBase(resolution) {}

  bool empty() const override { return blocks_.empty(); }
  size_t size() const override { return kCellsPerBlock * blocks_.size(); }
  void clear() override { blocks_.clear(); }

  size_t getMemoryUsage() const override {
    return size() * sizeof(CellDataSpecialized);
  }

  Index getMinIndex() const override;
  Index getMaxIndex() const override;

  bool hasCell(const Index& index) const override;
  FloatingPoint getCellValue(const Index& index) const override;
  void setCellValue(const Index& index, FloatingPoint new_value) override;
  void addToCellValue(const Index& index, FloatingPoint update) override;

  cv::Mat getImage(bool use_color) const override;
  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

 protected:
  using CellDataSpecialized = typename CellT::Specialized;

  static constexpr IndexElement kCellsPerSide = 64;
  static constexpr IndexElement kCellsPerBlock = kCellsPerSide * kCellsPerSide;
  static constexpr FloatingPoint kCellsPerSideInv =
      1.f / static_cast<FloatingPoint>(kCellsPerSide);

  using Block =
      Eigen::Matrix<CellDataSpecialized, kCellsPerSide, kCellsPerSide>;
  using BlockIndex = Index;
  using CellIndex = Index;

  std::unordered_map<BlockIndex, Block, IndexHash> blocks_;

  CellDataSpecialized* accessCellData(const Index& index,
                                      bool auto_allocate = false);
  const CellDataSpecialized* accessCellData(const Index& index) const;

  BlockIndex computeBlockIndexFromIndex(const Index& index) const;
  CellIndex computeCellIndexFromBlockIndexAndIndex(
      const BlockIndex& block_index, const Index& index) const;
};
}  // namespace wavemap_2d

#include "wavemap_2d/datastructure/hashed_blocks/hashed_blocks_inl.h"

#endif  // WAVEMAP_2D_DATASTRUCTURE_HASHED_BLOCKS_HASHED_BLOCKS_H_
