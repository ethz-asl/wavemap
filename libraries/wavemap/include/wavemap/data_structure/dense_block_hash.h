#ifndef WAVEMAP_DATA_STRUCTURE_DENSE_BLOCK_HASH_H_
#define WAVEMAP_DATA_STRUCTURE_DENSE_BLOCK_HASH_H_

#include "wavemap/data_structure/dense_grid.h"
#include "wavemap/data_structure/spatial_hash.h"
#include "wavemap/utils/math/int_math.h"

namespace wavemap {
template <typename CellDataT, int dim, unsigned cells_per_side>
class DenseBlockHash {
 public:
  static constexpr IndexElement kCellsPerSide = cells_per_side;
  static constexpr IndexElement kCellsPerSideLog2 =
      int_math::log2_floor(cells_per_side);
  static constexpr IndexElement kDim = dim;

  using BlockIndex = Index3D;
  using CellIndex = Index3D;
  using Block = DenseGrid<CellDataT, dim, cells_per_side>;
  using BlockHashMap = SpatialHash<Block, kDim>;

  explicit DenseBlockHash(FloatingPoint min_cell_width,
                          CellDataT default_value = 0.f)
      : min_cell_width_(min_cell_width), default_value_(default_value) {}

  bool empty() const { return block_map_.empty(); }
  size_t size() const { return Block::kCellsPerBlock * block_map_.size(); }
  void clear() { block_map_.clear(); }

  bool hasBlock(const Index3D& block_index) const {
    return block_map_.hasBlock(block_index);
  }
  Block* getBlock(const Index3D& block_index);
  const Block* getBlock(const Index3D& block_index) const;
  Block& getOrAllocateBlock(const Index3D& block_index);

  const CellDataT& getCellValue(const Index3D& index) const;
  CellDataT& getOrAllocateCellValue(const Index3D& index);
  const CellDataT& getDefaultCellValue() const { return default_value_; }

  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn) {
    block_map_.forEachBlock(visitor_fn);
  }
  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn) const {
    block_map_.forEachBlock(visitor_fn);
  }

  template <typename IndexedLeafVisitorFunction>
  void forEachLeaf(IndexedLeafVisitorFunction visitor_fn);
  template <typename IndexedLeafVisitorFunction>
  void forEachLeaf(IndexedLeafVisitorFunction visitor_fn) const;

 protected:
  const FloatingPoint min_cell_width_;
  const CellDataT default_value_;
  BlockHashMap block_map_;

  static Index3D indexToBlockIndex(const Index3D& index);
  static Index3D indexToCellIndex(const Index3D& index);
  static Index3D cellAndBlockIndexToIndex(const Index3D& block_index,
                                          const Index3D& cell_index);
};
}  // namespace wavemap

#include "wavemap/data_structure/impl/dense_block_hash_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_DENSE_BLOCK_HASH_H_
