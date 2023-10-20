#ifndef WAVEMAP_DATA_STRUCTURE_SPATIAL_HASH_H_
#define WAVEMAP_DATA_STRUCTURE_SPATIAL_HASH_H_

#include <unordered_map>

#include "wavemap/common.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/utils/math/int_math.h"

namespace wavemap {
namespace convert {
template <unsigned cells_per_side, int dim>
Index<dim> indexToBlockIndex(const Index<dim>& index);

template <unsigned cells_per_side, int dim>
Index<dim> indexToCellIndex(const Index<dim>& index);

template <unsigned cells_per_side, int dim>
Index<dim> cellAndBlockIndexToIndex(const Index<dim>& block_index,
                                    const Index<dim>& cell_index);
}  // namespace convert

template <typename BlockDataT, int dim>
class SpatialHash {
 public:
  static constexpr IndexElement kDim = dim;

  using BlockIndex = Index<dim>;
  using CellIndex = Index<dim>;
  using BlockData = BlockDataT;

  bool empty() const { return block_map_.empty(); }
  size_t size() const { return block_map_.size(); }
  void clear() { block_map_.clear(); }

  Index<dim> getMinBlockIndex() const;
  Index<dim> getMaxBlockIndex() const;

  bool hasBlock(const BlockIndex& block_index) const;

  BlockData* getBlock(const BlockIndex& block_index);
  const BlockData* getBlock(const BlockIndex& block_index) const;
  template <typename... Args>
  BlockData& getOrAllocateBlock(const BlockIndex& block_index, Args... args);

  void eraseBlock(const BlockIndex& block_index);
  template <typename IndexedBlockVisitor>
  void eraseBlockIf(IndexedBlockVisitor indicator_fn);

  auto& getHashMap() { return block_map_; }
  const auto& getHashMap() const { return block_map_; }

  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn);
  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn) const;

 private:
  std::unordered_map<Index<dim>, BlockDataT, IndexHash<dim>> block_map_;
};
}  // namespace wavemap

#include "wavemap/data_structure/impl/spatial_hash_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_SPATIAL_HASH_H_
