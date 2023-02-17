#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_BLOCKS_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_BLOCKS_H_

#include <string>
#include <unordered_map>

#include "wavemap/common.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/utils/int_math.h"

namespace wavemap {
template <typename CellT, int dim>
class HashedBlocks : public virtual VolumetricDataStructureBase<dim> {
 public:
  using CellType = CellT;
  static constexpr bool kRequiresPruningForThresholding = false;

  // Use the base class' constructor
  using VolumetricDataStructureBase<dim>::VolumetricDataStructureBase;

  bool empty() const override { return blocks_.empty(); }
  size_t size() const override { return kCellsPerBlock * blocks_.size(); }
  void prune() override;
  void clear() override { blocks_.clear(); }

  size_t getMemoryUsage() const override {
    return size() * sizeof(CellDataSpecialized);
  }

  Index<dim> getMinIndex() const override;
  Index<dim> getMaxIndex() const override;

  FloatingPoint getCellValue(const Index<dim>& index) const override;
  void setCellValue(const Index<dim>& index, FloatingPoint new_value) override;
  void addToCellValue(const Index<dim>& index, FloatingPoint update) override;

  void forEachLeaf(
      typename VolumetricDataStructureBase<dim>::IndexedLeafVisitorFunction
          visitor_fn) const override;

  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

 private:
  using CellDataSpecialized = typename CellT::Specialized;

  static constexpr IndexElement kCellsPerSideLog2 = (dim == 2 ? 6 : 4);
  static constexpr IndexElement kCellsPerSide =
      int_math::exp2(kCellsPerSideLog2);
  static constexpr IndexElement kCellsPerBlock =
      int_math::exp2(dim * kCellsPerSideLog2);

  using Block = std::array<CellDataSpecialized, kCellsPerBlock>;
  using BlockIndex = Index<dim>;
  using CellIndex = Index<dim>;

  std::unordered_map<BlockIndex, Block, VoxbloxIndexHash<dim>> blocks_;

  CellDataSpecialized* accessCellData(const Index<dim>& index,
                                      bool auto_allocate = false);
  const CellDataSpecialized* accessCellData(const Index<dim>& index) const;

  BlockIndex computeBlockIndexFromIndex(const Index<dim>& index) const;
  CellIndex computeCellIndexFromBlockIndexAndIndex(
      const BlockIndex& block_index, const Index<dim>& index) const;
  Index<dim> computeIndexFromBlockIndexAndCellIndex(
      const BlockIndex& block_index, const CellIndex& cell_index) const;
};
}  // namespace wavemap

#include "wavemap/data_structure/volumetric/impl/hashed_blocks_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_BLOCKS_H_
