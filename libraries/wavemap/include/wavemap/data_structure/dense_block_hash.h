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

  using Block = DenseGrid<CellDataT, dim, cells_per_side>;
  using BlockHashMap = SpatialHash<Block, kDim>;

  explicit DenseBlockHash(FloatingPoint min_cell_width,
                          CellDataT default_value = CellDataT{})
      : min_cell_width_(min_cell_width), default_value_(default_value) {}

  bool empty() const { return block_map_.empty(); }
  size_t size() const { return Block::kCellsPerBlock * block_map_.size(); }
  void clear() { block_map_.clear(); }

  bool hasBlock(const Index<dim>& block_index) const;
  bool eraseBlock(const Index<dim>& block_index);
  Block* getBlock(const Index<dim>& block_index);
  const Block* getBlock(const Index<dim>& block_index) const;
  Block& getOrAllocateBlock(const Index<dim>& block_index);

  // NOTE: The values are stored in blocks. Therefore, allocating a value in a
  //       block that does not yet exist also allocates all its siblings. These
  //       will be initialized to default_value_.
  //       If you want to make sure a value has been explicitly set by you,
  //       check that it differs from the default_value_. For example, using the
  //       equalsDefaultValue(value) method below.
  bool hasValue(const Index<dim>& index) const;
  CellDataT* getValue(const Index<dim>& index);
  const CellDataT* getValue(const Index<dim>& index) const;
  CellDataT& getOrAllocateValue(const Index<dim>& index);

  const CellDataT& getValueOrDefault(const Index<dim>& index) const;
  const CellDataT& getDefaultValue() const { return default_value_; }
  bool equalsDefaultValue(const CellDataT& value) const;

  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn);
  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn) const;

  template <typename IndexedLeafVisitorFunction>
  void forEachLeaf(IndexedLeafVisitorFunction visitor_fn);
  template <typename IndexedLeafVisitorFunction>
  void forEachLeaf(IndexedLeafVisitorFunction visitor_fn) const;

 protected:
  const FloatingPoint min_cell_width_;
  const CellDataT default_value_;
  BlockHashMap block_map_;

  static Index<dim> indexToBlockIndex(const Index<dim>& index);
  static Index<dim> indexToCellIndex(const Index<dim>& index);
  static Index<dim> cellAndBlockIndexToIndex(const Index<dim>& block_index,
                                             const Index<dim>& cell_index);
};
}  // namespace wavemap

#include "wavemap/data_structure/impl/dense_block_hash_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_DENSE_BLOCK_HASH_H_
