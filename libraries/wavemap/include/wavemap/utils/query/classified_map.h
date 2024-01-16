#ifndef WAVEMAP_UTILS_QUERY_CLASSIFIED_MAP_H_
#define WAVEMAP_UTILS_QUERY_CLASSIFIED_MAP_H_

#include <utility>

#include <wavemap/data_structure/ndtree/ndtree.h>
#include <wavemap/data_structure/ndtree_block_hash.h>
#include <wavemap/map/hashed_wavelet_octree.h>
#include <wavemap/utils/query/occupancy_classifier.h>

namespace wavemap {
class ClassifiedMap {
 public:
  struct NodeData {
    std::bitset<OctreeIndex::kNumChildren> has_free;
    std::bitset<OctreeIndex::kNumChildren> has_occupied;
    std::bitset<OctreeIndex::kNumChildren> has_unobserved;

    Occupancy::Mask occupancyMask() const;
    Occupancy::Mask childOccupancyMask(
        NdtreeIndexRelativeChild child_idx) const;
  };
  using HeightType = IndexElement;
  using BlockHashMap = OctreeBlockHash<NodeData>;
  using Block = BlockHashMap::Block;
  using Node = BlockHashMap::Node;
  static constexpr int kDim = 3;

  ClassifiedMap(FloatingPoint min_cell_width, IndexElement tree_height,
                const OccupancyClassifier& classifier)
      : min_cell_width_(min_cell_width),
        classifier_(classifier),
        block_map_(tree_height) {}

  ClassifiedMap(const HashedWaveletOctree& occupancy_map,
                const OccupancyClassifier& classifier)
      : ClassifiedMap(occupancy_map.getMinCellWidth(),
                      occupancy_map.getTreeHeight(), classifier) {
    update(occupancy_map);
  }

  FloatingPoint getMinCellWidth() const { return min_cell_width_; }
  IndexElement getTreeHeight() const { return block_map_.getMaxHeight(); }

  void update(const HashedWaveletOctree& occupancy_map);

  bool has(const OctreeIndex& index, Occupancy::Id occupancy_type) const;
  bool has(const OctreeIndex& index, Occupancy::Mask occupancy_mask) const;

  bool isFully(const OctreeIndex& index, Occupancy::Id occupancy_type) const;
  bool isFully(const OctreeIndex& index, Occupancy::Mask occupancy_mask) const;

  bool hasBlock(const Index3D& block_index) const;
  Block* getBlock(const Index3D& block_index);
  const Block* getBlock(const Index3D& block_index) const;
  Block& getOrAllocateBlock(const Index3D& block_index);
  BlockHashMap& getBlockMap() { return block_map_; }
  const BlockHashMap& getBlockMap() const { return block_map_; }

  bool hasValue(const OctreeIndex& index) const;
  std::optional<Occupancy::Mask> getValue(const OctreeIndex& index) const;
  std::pair<std::optional<Occupancy::Mask>, HeightType> getValueOrAncestor(
      const OctreeIndex& index) const;

  void forEachLeafMatching(Occupancy::Id occupancy_type,
                           std::function<void(const OctreeIndex&)> visitor_fn,
                           IndexElement termination_height = 0) const;
  void forEachLeafMatching(Occupancy::Mask occupancy_mask,
                           std::function<void(const OctreeIndex&)> visitor_fn,
                           IndexElement termination_height = 0) const;

 private:
  const FloatingPoint min_cell_width_;
  const OccupancyClassifier classifier_;
  BlockHashMap block_map_;

  void recursiveClassifier(
      const HashedWaveletOctreeBlock::NodeType& occupancy_node,
      FloatingPoint average_occupancy, Node& classified_node);
};
}  // namespace wavemap

#include "wavemap/utils/query/impl/classified_map_inl.h"

#endif  // WAVEMAP_UTILS_QUERY_CLASSIFIED_MAP_H_
