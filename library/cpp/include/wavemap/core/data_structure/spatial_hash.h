#ifndef WAVEMAP_CORE_DATA_STRUCTURE_SPATIAL_HASH_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_SPATIAL_HASH_H_

#include <unordered_map>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/indexing/index_hashes.h"
#include "wavemap/core/utils/math/int_math.h"

namespace wavemap {
namespace convert {
template <int dim>
Index<dim> indexToBlockIndex(const Index<dim>& index,
                             IndexElement cells_per_block_side_log_2);
}  // namespace convert

template <typename BlockDataT, int dim>
class SpatialHash {
 public:
  static constexpr IndexElement kDim = dim;

  using BlockIndex = Index<dim>;
  using BlockData = BlockDataT;

  bool empty() const { return block_map_.empty(); }
  size_t size() const { return block_map_.size(); }
  void clear() { block_map_.clear(); }

  Index<dim> getMinBlockIndex() const;
  Index<dim> getMaxBlockIndex() const;

  bool hasBlock(const BlockIndex& block_index) const;
  bool eraseBlock(const BlockIndex& block_index);
  template <typename IndexedBlockVisitor>
  void eraseBlockIf(IndexedBlockVisitor indicator_fn);

  BlockData* getBlock(const BlockIndex& block_index);
  const BlockData* getBlock(const BlockIndex& block_index) const;
  template <typename... DefaultArgs>
  BlockData& getOrAllocateBlock(const BlockIndex& block_index,
                                DefaultArgs&&... args);

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

#include "wavemap/core/data_structure/impl/spatial_hash_inl.h"

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_SPATIAL_HASH_H_
