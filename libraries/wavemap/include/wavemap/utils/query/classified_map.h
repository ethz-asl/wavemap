#ifndef WAVEMAP_UTILS_QUERY_CLASSIFIED_MAP_H_
#define WAVEMAP_UTILS_QUERY_CLASSIFIED_MAP_H_

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
  };
  using BlockHashMap = OctreeBlockHash<NodeData>;
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

  template <typename VisitorFn>
  void forEachOccupiedLeaf(VisitorFn visitor_fn) const {}

 private:
  const FloatingPoint min_cell_width_;
  const OccupancyClassifier classifier_;
  BlockHashMap block_map_;

  void recursiveClassifier(
      const HashedWaveletOctreeBlock::NodeType& occupancy_node,
      FloatingPoint average_occupancy, Node& classified_node);
};
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_QUERY_CLASSIFIED_MAP_H_
